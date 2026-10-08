"""Runtime test harness: launch auv_runtime against a fake STM32 PTY.

The harness mirrors the proven offline replay/arm-simulation fixtures: it starts
the native runtime with a rewired config (file-camera video, PTY serial device,
private socket/log dirs, motion output disabled for safety), drives a virtual
STM32 that streams STATUS frames, and polls the JSON control socket.
"""
from __future__ import annotations

import json
import os
import pathlib
import pty
import re
import select
import shutil
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
RUNTIME_CONFIG = REPO_ROOT / "runtime" / "config" / "runtime.yaml"


def crc(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, init 0xFFFF, poly 0x1021 (matches serial protocol v1)."""
    value = 0xFFFF
    for byte in data:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def frame(kind: int, payload: bytes) -> bytes:
    """Wrap a payload in a serial frame: AA 55 ver type len payload CRC16."""
    body = bytes([1, kind, len(payload)]) + payload
    return b"\xaa\x55" + body + struct.pack("<H", crc(body))


def mission_phases(events: list[dict]) -> list[str]:
    """Return the ordered MISSION phase names from the event log."""
    return [e["detail"] for e in events if e["event"] == "MISSION"]


def plan_path(events: list[dict]) -> list[tuple[int, int]]:
    """Parse the planned route (r,c)(r,c)... from the PLAN event."""
    for e in events:
        if e["event"] == "PLAN":
            tail = e["detail"].split("path=")[-1]
            return [(int(r), int(c)) for r, c in re.findall(r"\((\d+),(\d+)\)", tail)]
    return []


class RuntimeHarness:
    """Context manager that runs auv_runtime against a virtual STM32."""

    def __init__(self, binary: str, video: pathlib.Path, motion: bool = False):
        self.binary = binary
        self.video = video
        self.motion = motion
        self.root = None
        self._proc = None
        self._master = None
        self._slave = None
        self._stop = None
        self._worker = None
        self._motion_targets: list = []
        self._arm_events: list = []

    def __enter__(self) -> "RuntimeHarness":
        self.root = pathlib.Path(tempfile.mkdtemp(prefix="auv-test-"))
        self._master, self._slave = pty.openpty()
        tty = os.ttyname(self._slave)

        config = RUNTIME_CONFIG.read_text()
        # Point the camera at the synthetic video, the serial at the PTY, and
        # every filesystem/socket output at the private temp dir.
        config = config.replace(
            "source: csi:0", f"source: file:{self.video}"
        )
        config = config.replace('device: ""', f'device: "{tty}"')
        config = config.replace(
            "/run/auv-runtime/control.sock", str(self.root / "control.sock")
        )
        config = config.replace("/run/auv-runtime/hls", str(self.root / "hls"))
        config = config.replace(
            "/var/log/auv-runtime/events.ndjson", str(self.root / "events.ndjson")
        )
        config = config.replace("/var/log/auv-runtime/debug", str(self.root / "debug"))
        config = config.replace(
            "/usr/local/share/auv-runtime/web", str(REPO_ROOT / "runtime" / "web")
        )
        # Keep video/auto-start off regardless. In motion mode also enable a
        # calibrated (synthetic) motion configuration so the runtime can ARM and
        # emit MOTION_TARGET frames; the serial stays on the PTY only.
        config = config.replace("enabled: true", "enabled: false")
        if self.motion:
            config = config.replace(
                "motion_commands_enabled: false", "motion_commands_enabled: true"
            )
            config = config.replace(
                "directions_calibrated: false", "directions_calibrated: true"
            )
            config = config.replace(
                "limits_calibrated: false", "limits_calibrated: true"
            )
            config = config.replace(
                "camera_matrix: []", "camera_matrix: [600, 0, 320, 0, 600, 240, 0, 0, 1]"
            )
            config = config.replace(
                "distortion_coefficients: []", "distortion_coefficients: [0, 0, 0, 0, 0]"
            )
            config = config.replace("surge_from_row: 0.0", "surge_from_row: 0.1")
            config = config.replace("sway_from_col: 0.0", "sway_from_col: 0.1")
            config = config.replace("pose_timeout_sec: 0.5", "pose_timeout_sec: 2.0")
        (self.root / "runtime.yaml").write_text(config)

        self._proc = subprocess.Popen(
            [self.binary, str(self.root / "runtime.yaml")],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        self._stop = threading.Event()

        # Virtual STM32: streams STATUS (the armed flag follows ARM requests),
        # ACKs SET_ARMED / ACTUATOR_COMMAND, and records every decoded
        # MOTION_TARGET frame as a (seq, vx, vy, depth, yaw) timeline so tests
        # can assert the onset of propulsion to the STM32.
        def stm32() -> None:
            incoming = bytearray()
            sequence = 0
            armed = False
            current_depth = 1.0
            while not self._stop.is_set():
                try:
                    sequence += 1
                    payload = struct.pack(
                        "<IBIfffffB", sequence, int(armed), 0,
                        12.0, current_depth, 0.0, 0.0, 0.0, 0)
                    os.write(self._master, frame(0x80, payload))
                    readable, _, _ = select.select([self._master], [], [], 0.05)
                    if readable:
                        incoming.extend(os.read(self._master, 8192))
                    while len(incoming) >= 7:
                        if incoming[:2] != b"\xaa\x55":
                            del incoming[0]
                            continue
                        length = 7 + incoming[4]
                        if len(incoming) < length:
                            break
                        packet = bytes(incoming[:length])
                        del incoming[:length]
                        if crc(packet[2:-2]) != struct.unpack("<H", packet[-2:])[0]:
                            continue
                        kind, data = packet[3], packet[5:-2]
                        if kind == 2:  # SET_ARMED -> ACK + flip armed flag
                            armed = bool(data[4])
                            self._arm_events.append((time.monotonic(), armed))
                            os.write(self._master, frame(0x7f, bytes([2, 0]) + data[:4]))
                        elif kind == 3:  # MOTION_TARGET -> record + track depth
                            seq, vx, vy, cmd_depth, yaw = struct.unpack("<Iffff", data)
                            current_depth = cmd_depth  # emulate depth PID tracking the target
                            self._motion_targets.append({
                                "seq": seq, "vx": vx, "vy": vy,
                                "depth": cmd_depth, "yaw": yaw,
                                "armed": armed, "t": time.monotonic(),
                            })
                        elif kind == 4:  # ACTUATOR_COMMAND -> ACK
                            seq, actuator, value = struct.unpack("<IBf", data)
                            os.write(
                                self._master,
                                frame(0x7f, bytes([4, 0]) + struct.pack("<I", seq)))
                except OSError:
                    return

        self._worker = threading.Thread(target=stm32, daemon=True)
        self._worker.start()

        for _ in range(200):
            if (self.root / "control.sock").exists():
                break
            time.sleep(0.01)
        else:
            raise AssertionError("control socket did not appear")
        return self

    def status(self) -> dict:
        return json.loads(self._request("status"))

    def command(self, text: str) -> str:
        return self._request(text)

    def _request(self, text: str) -> str:
        with socket.socket(socket.AF_UNIX) as peer:
            peer.connect(str(self.root / "control.sock"))
            peer.sendall(text.encode())
            chunks = []
            while True:
                data = peer.recv(65536)
                if not data:
                    break
                chunks.append(data)
            return b"".join(chunks).decode()

    def wait_for(self, predicate, timeout: float = 8.0, interval: float = 0.05) -> dict:
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.status()
            if last.get("phase") == "FAULT":
                raise AssertionError(f"runtime faulted: {last.get('fault')}")
            if predicate(last):
                return last
            time.sleep(interval)
        raise AssertionError(f"condition not met within {timeout}s; last={last}")

    def wait_ready(self, timeout: float = 8.0) -> dict:
        return self.wait_for(
            lambda s: s["serial"] and s["camera_age_sec"] >= 0, timeout=timeout
        )

    def start(self) -> str:
        reply = self.command("start")
        assert reply.startswith("OK"), f"start rejected: {reply!r}"
        return reply

    def motion_targets(self) -> list:
        return list(self._motion_targets)

    def has_nonzero_motion(self) -> bool:
        return any(
            abs(t["vx"]) > 1e-3 or abs(t["vy"]) > 1e-3 for t in self._motion_targets
        )

    def run_to_visit(
        self,
        snapshot_phases=("SEARCH_APRILTAG", "BUILD_MAP", "VISIT_CONES"),
        timeout: float = 8.0,
        interval: float = 0.01,
    ) -> tuple[dict, dict[str, dict]]:
        """Poll until VISIT_CONES, recording one status snapshot per observed phase."""
        snapshots: dict[str, dict] = {}
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            last = self.status()
            phase = last.get("phase")
            if phase == "FAULT":
                raise AssertionError(f"runtime faulted: {last.get('fault')}")
            if phase in snapshot_phases and phase not in snapshots:
                snapshots[phase] = last
            if phase == "VISIT_CONES":
                return last, snapshots
            time.sleep(interval)
        raise AssertionError(f"VISIT_CONES not reached within {timeout}s; last={last}")

    def events(self) -> list[dict]:
        path = self.root / "events.ndjson"
        if not path.exists():
            return []
        return [json.loads(line) for line in path.read_text().splitlines()]

    def __exit__(self, *exc) -> None:
        if self._stop is not None:
            self._stop.set()
        if self._proc is not None and self._proc.poll() is None:
            self._proc.send_signal(signal.SIGTERM)
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        if self._worker is not None:
            self._worker.join(timeout=1)
        if self._master is not None:
            os.close(self._master)
        if self._slave is not None:
            os.close(self._slave)
        if self.root is not None:
            shutil.rmtree(self.root, ignore_errors=True)
