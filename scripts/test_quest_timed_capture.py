#!/usr/bin/env python3

from __future__ import annotations

from importlib import import_module


capture_module = import_module("debug_quest_timed_capture")
PhaseSpec = getattr(capture_module, "PhaseSpec")
build_capture_record = getattr(capture_module, "build_capture_record")
build_phases = getattr(capture_module, "build_phases")
phase_at_time = getattr(capture_module, "phase_at_time")
total_duration = getattr(capture_module, "total_duration")


def main() -> None:
    phases = build_phases(2.0, 1.0, 3.0, 1.0)
    assert [phase.name for phase in phases] == [
        "settle",
        "pre_hold",
        "move",
        "post_hold",
    ]
    assert abs(total_duration(phases) - 7.0) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 0.5)
    assert idx == 0 and phase.name == "settle" and abs(elapsed - 0.5) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 2.5)
    assert idx == 1 and phase.name == "pre_hold" and abs(elapsed - 0.5) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 2.0)
    assert idx == 0 and phase.name == "settle" and abs(elapsed - 2.0) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 3.0)
    assert idx == 1 and phase.name == "pre_hold" and abs(elapsed - 1.0) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 3.5)
    assert idx == 2 and phase.name == "move" and abs(elapsed - 0.5) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 6.0)
    assert idx == 2 and phase.name == "move" and abs(elapsed - 3.0) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 6.8)
    assert idx == 3 and phase.name == "post_hold"
    assert abs(elapsed - 0.8) < 1e-9

    idx, phase, elapsed = phase_at_time(phases, 8.2)
    assert idx == 3 and phase.name == "post_hold" and abs(elapsed - 1.0) < 1e-9

    record = build_capture_record(
        sample_index=4,
        sequence_elapsed_s=3.2,
        phase_index=2,
        phase=PhaseSpec("move", 3.0, "move slowly"),
        phase_elapsed_s=0.2,
        action={"quest.enabled": 1.0},
    )
    assert set(record) == {
        "capture_mode",
        "sample_index",
        "sequence_elapsed_s",
        "phase_index",
        "phase",
        "phase_elapsed_s",
        "action",
    }
    assert record["capture_mode"] == "axis_align_rg_only"
    assert record["sample_index"] == 4
    assert record["phase"] == "move"
    assert record["phase_index"] == 2
    assert record["sequence_elapsed_s"] == 3.2
    assert record["phase_elapsed_s"] == 0.2
    assert record["action"]["quest.enabled"] == 1.0

    print("Quest timed capture validation passed.")


if __name__ == "__main__":
    main()
