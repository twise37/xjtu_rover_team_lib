"""Persist task-one runtime test outputs under a per-run directory.

Each pytest run creates ``output/<timestamp>/`` and writes every scene's
rendered semantic map, planned path, the runtime's grid-corner debug image, a
text summary, the raw event log and per-phase status snapshots there. Content
is deterministic for identical inputs; only the timestamped directory name
differs between runs (so two runs are distinguishable).
"""
from __future__ import annotations

import json
import pathlib
from datetime import datetime

import cv2
import numpy as np

import scenes
from harness import mission_phases, plan_path

_run_dir: pathlib.Path | None = None

OBJECT_COLORS = {
    "circle_cone": (0, 150, 255),  # orange
    "square_cone": (80, 200, 80),  # green
    "unknown": (225, 225, 225),    # light gray
}

CELL = 180
PAD = 60


def run_dir() -> pathlib.Path:
    """Return (and lazily create) this run's timestamped output directory."""
    global _run_dir
    if _run_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        candidate = scenes.OUTPUT_DIR / stamp
        suffix = 2
        while candidate.exists():
            candidate = scenes.OUTPUT_DIR / f"{stamp}_{suffix}"
            suffix += 1
        candidate.mkdir(parents=True, exist_ok=True)
        _run_dir = candidate
    return _run_dir


def render_map(cells, visited=None, title: str = "semantic map") -> np.ndarray:
    """Render a 3x3 semantic map (object type per cell, visited markers)."""
    visited = visited or set()
    canvas = np.full((PAD + 3 * CELL + PAD, PAD + 3 * CELL + PAD, 3), 40, np.uint8)
    for r in range(3):
        for c in range(3):
            obj = cells.get((r, c), "unknown")
            color = OBJECT_COLORS.get(obj, OBJECT_COLORS["unknown"])
            x0 = PAD + c * CELL
            y0 = PAD + r * CELL
            cv2.rectangle(canvas, (x0, y0), (x0 + CELL, y0 + CELL), color, -1)
            cv2.rectangle(canvas, (x0, y0), (x0 + CELL, y0 + CELL), (0, 0, 0), 2)
            cv2.putText(canvas, obj, (x0 + 8, y0 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            cv2.putText(canvas, f"({r},{c})", (x0 + 8, y0 + 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1)
            if (r, c) in visited:
                cv2.putText(canvas, "VISITED", (x0 + 8, y0 + CELL - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 120, 0), 2)
    cv2.putText(canvas, title, (PAD, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    return canvas


def render_path(cells, path, visited=None, title: str = "planned path") -> np.ndarray:
    """Render the semantic map overlaid with the planned visit order."""
    canvas = render_map(cells, visited=visited, title=title)
    centers = [(PAD + c * CELL + CELL // 2, PAD + r * CELL + CELL // 2) for r, c in path]
    for i in range(1, len(centers)):
        cv2.line(canvas, centers[i - 1], centers[i], (0, 0, 255), 2)
    for i, (cx, cy) in enumerate(centers):
        cv2.circle(canvas, (cx, cy), 11, (0, 0, 255), -1)
        cv2.circle(canvas, (cx, cy), 11, (255, 255, 255), 1)
        cv2.putText(canvas, str(i), (cx - 6, cy + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
    return canvas


def _write_png(name: str, image: np.ndarray) -> None:
    cv2.imwrite(str(run_dir() / f"{name}.png"), image)


def _copy_runtime_debug(harness, name: str) -> int:
    """Copy the runtime's grid-corner overlay images into the run dir."""
    debug_dir = harness.root / "debug"
    images = sorted(debug_dir.glob("map_*.jpg")) if debug_dir.exists() else []
    for i, image_path in enumerate(images):
        suffix = "" if i == 0 else str(i)
        _write_png(f"grid_{name}{suffix}", cv2.imread(str(image_path)))
    return len(images)


def _cells_and_visited(state):
    cells = {(c["row"], c["col"]): c["object"] for c in state["cells"]}
    visited = {(c["row"], c["col"]) for c in state["cells"] if c["visited"]}
    return cells, visited


def _save_summary(name, state, cells, visited, path, phases, motion=None) -> None:
    lines = [
        f"scene: {name}",
        f"phase: {state.get('phase')}",
        f"apriltag_found: {state.get('apriltag_found')}",
        f"grid_complete: {state.get('grid_complete')}",
        f"cone_count: {state.get('cone_count')}",
        "phases: " + (" -> ".join(phases) if phases else "(none)"),
        "path: " + (" -> ".join(f"({r},{c})" for r, c in path) if path else "(none)"),
        "cells:",
    ]
    for r in range(3):
        row = []
        for c in range(3):
            obj = cells.get((r, c), "unknown")
            mark = "*" if (r, c) in visited else " "
            row.append(f"({r},{c})={obj}{mark}")
        lines.append("  " + " ".join(row))
    if motion:
        moving = [m for m in motion if abs(m["vx"]) > 1e-3 or abs(m["vy"]) > 1e-3]
        lines.append("motion output:")
        if moving:
            first = moving[0]
            lines.append(
                f"  first_nonzero: vx={first['vx']:.6f} vy={first['vy']:.6f} "
                f"depth={first['depth']:.6f} yaw={first['yaw']:.6f}")
            lines.append(f"  frames: {len(motion)} total, {len(moving)} nonzero")
            bounded = all(
                abs(m["vx"]) <= 0.1 + 1e-6 and abs(m["vy"]) <= 0.1 + 1e-6
                for m in motion)
            lines.append(f"  bounded(<=0.1): {'yes' if bounded else 'no'}")
        else:
            lines.append("  (no nonzero motion observed)")
    (run_dir() / f"summary_{name}.txt").write_text("\n".join(lines) + "\n")


def _save_events(name, harness) -> None:
    events_path = harness.root / "events.ndjson"
    if events_path.exists():
        (run_dir() / f"events_{name}.ndjson").write_bytes(events_path.read_bytes())


def _save_snapshots(name, snapshots) -> None:
    for phase, status in sorted((snapshots or {}).items()):
        (run_dir() / f"status_{name}_{phase}.json").write_text(
            json.dumps(status, indent=2, sort_keys=True) + "\n"
        )


def _write_motion_timeline(name, motion) -> None:
    """Persist the decoded MOTION_TARGET timeline as one NDJSON line per frame."""
    if not motion:
        return
    (run_dir() / f"motion_{name}.ndjson").write_text(
        "".join(json.dumps(m, sort_keys=True) + "\n" for m in motion)
    )


def write_scene_artifacts(name, harness, state, snapshots=None, motion=None) -> None:
    """Render and persist map / path / grid / summary / events / snapshots.

    ``motion``, when given, is the list of decoded MOTION_TARGET frames recorded
    by the virtual STM32; it is archived as an NDJSON timeline and summarised.
    """
    run_dir()
    cells, visited = _cells_and_visited(state)
    events = harness.events()
    path = plan_path(events)
    phases = mission_phases(events)
    _write_png(f"map_{name}", render_map(cells, visited, title=f"{name}: semantic map"))
    if path:
        _write_png(f"path_{name}", render_path(cells, path, visited, title=f"{name}: planned path"))
    _copy_runtime_debug(harness, name)
    _save_summary(name, state, cells, visited, path, phases, motion=motion)
    _save_events(name, harness)
    _save_snapshots(name, snapshots)
    _write_motion_timeline(name, motion)
