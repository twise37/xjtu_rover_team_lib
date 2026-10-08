"""Traversal-start surfacing + power-output assertions for task one.

With motion enabled and a virtual ARM/ACK STM32, verify that once the mission
finishes mapping and enters VISIT_CONES, arming first ascends the vehicle to
surface_depth_m (surfacing: depth held, no horizontal motion) and only then
starts the cone traversal — emitting bounded MOTION_TARGET frames (vx/vy/depth/
yaw) to the STM32.

Determinism note: the assertions depend only on the *presence* of a surfacing
phase followed by nonzero motion, on per-frame bounds (<= maximum_speed) and on
the surfaced depth/yaw hold values; they never depend on frame counts, sequence
numbers or wall-clock time, which are timing artefacts. The archived motion
timeline is a report.
"""
from __future__ import annotations

import time

import pytest

import scenes
import visualize
from harness import RuntimeHarness

MAX_SPEED = 0.1
SURFACE_DEPTH = 0.5  # motion.surface_depth_m in runtime.yaml


def _with_apriltag_scenes():
    index = scenes.load_ground_truth()
    out = []
    for f in scenes.list_read_scenes():
        meta = index.get(f.name, {})
        if meta.get("with_apriltag"):
            out.append((f.stem, f, meta))
    return out


def test_traversal_emits_bounded_motion(runtime_binary, tmp_path):
    full = _with_apriltag_scenes()
    if not full:
        pytest.skip("no with_apriltag scenes in input/read; run generate_scenes.py")

    for stem, f, _meta in full:
        video = tmp_path / f"{stem}.avi"
        scenes.build_video_from_jpg(scenes.READ_DIR / f.name, video)

        with RuntimeHarness(runtime_binary, video, motion=True) as h:
            h.wait_ready()
            h.start()
            state, snapshots = h.run_to_visit()

            assert state["apriltag_found"], f"{stem}: AprilTag not detected"
            assert state["grid_complete"], f"{stem}: grid not complete"

            # Mapping/planning must never produce propulsion.
            assert not h.motion_targets(), f"{stem}: motion emitted before ARM"

            reply = h.command("arm SAFE_TO_ARM")
            assert reply.startswith("OK"), f"{stem}: ARM rejected: {reply!r}"

            h.wait_for(lambda s: h.has_nonzero_motion(), timeout=8.0, interval=0.05)

            targets = h.motion_targets()
            assert targets, f"{stem}: no MOTION_TARGET frames observed"
            for t in targets:
                assert abs(t["vx"]) <= MAX_SPEED + 1e-6, f"{stem}: vx out of bounds: {t}"
                assert abs(t["vy"]) <= MAX_SPEED + 1e-6, f"{stem}: vy out of bounds: {t}"
                assert abs(t["depth"] - SURFACE_DEPTH) < 1e-6, f"{stem}: depth not surfaced: {t}"
                assert abs(t["yaw"]) < 1e-6, f"{stem}: yaw not latched: {t}"

            # Surfacing precedes traversal: a zero-horizontal phase (depth held at
            # SURFACE_DEPTH) must be followed by a nonzero-horizontal phase.
            first_moving = next(
                i for i, t in enumerate(targets)
                if abs(t["vx"]) > 1e-3 or abs(t["vy"]) > 1e-3
            )
            assert first_moving > 0, f"{stem}: no surfacing phase before traversal"
            assert all(
                abs(t["vx"]) < 1e-3 and abs(t["vy"]) < 1e-3
                for t in targets[:first_moving]
            ), f"{stem}: nonzero motion during surfacing"

            # SURFACED must be emitted once the depth holds in band (before
            # traversal). Poll the log briefly to avoid a logger-flush race.
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not any(
                e["event"] == "SURFACED" for e in h.events()
            ):
                time.sleep(0.05)
            assert any(e["event"] == "SURFACED" for e in h.events()), (
                f"{stem}: no SURFACED event"
            )

            phases = [e["detail"] for e in h.events() if e["event"] == "MISSION"]
            assert "VISIT_CONES" in phases, f"{stem}: never entered VISIT_CONES"

            visualize.write_scene_artifacts(
                stem, h, state, snapshots, motion=targets
            )
