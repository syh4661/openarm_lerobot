#!/usr/bin/env python3

from __future__ import annotations

import argparse
from dataclasses import dataclass
from importlib import import_module
import json
import logging
import time
from pathlib import Path
from typing import Any


debug_input_module = import_module("debug_quest_input_only")
load_teleop_config = getattr(debug_input_module, "load_teleop_config")


logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PhaseSpec:
    name: str
    duration_s: float
    instruction: str


def build_phases(
    settle_s: float, pre_hold_s: float, move_s: float, post_hold_s: float
) -> list[PhaseSpec]:
    return [
        PhaseSpec(
            name="settle",
            duration_s=max(0.0, settle_s),
            instruction="Relax. Do not press RG yet. Let the system settle.",
        ),
        PhaseSpec(
            name="pre_hold",
            duration_s=max(0.0, pre_hold_s),
            instruction="Press and hold RG only. Ignore the index trigger and gripper. Keep wrist still.",
        ),
        PhaseSpec(
            name="move",
            duration_s=max(0.0, move_s),
            instruction="Keep RG held and move slowly in one straight direction only. Ignore the index trigger. Avoid wrist rotation.",
        ),
        PhaseSpec(
            name="post_hold",
            duration_s=max(0.0, post_hold_s),
            instruction="Stop moving and hold still with RG only. Ignore the index trigger and gripper.",
        ),
    ]


def total_duration(phases: list[PhaseSpec]) -> float:
    return sum(phase.duration_s for phase in phases)


def phase_at_time(
    phases: list[PhaseSpec], elapsed_s: float
) -> tuple[int, PhaseSpec, float]:
    remaining = max(0.0, elapsed_s)
    for index, phase in enumerate(phases):
        if remaining <= phase.duration_s or index == len(phases) - 1:
            return index, phase, min(remaining, phase.duration_s)
        remaining -= phase.duration_s
    return len(phases) - 1, phases[-1], phases[-1].duration_s


def build_capture_record(
    *,
    sample_index: int,
    sequence_elapsed_s: float,
    phase_index: int,
    phase: PhaseSpec,
    phase_elapsed_s: float,
    action: dict[str, Any],
) -> dict[str, Any]:
    return {
        "capture_mode": "axis_align_rg_only",
        "sample_index": sample_index,
        "sequence_elapsed_s": sequence_elapsed_s,
        "phase_index": phase_index,
        "phase": phase.name,
        "phase_elapsed_s": phase_elapsed_s,
        "action": action,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a timed RG-only Quest axis-alignment sequence with explicit operator phases."
    )
    parser.add_argument(
        "--config", type=Path, required=True, help="Path to record config JSON."
    )
    parser.add_argument(
        "--settle-s", type=float, default=2.0, help="Initial settle duration."
    )
    parser.add_argument(
        "--pre-hold-s",
        type=float,
        default=1.0,
        help="Hold-still duration before movement.",
    )
    parser.add_argument("--move-s", type=float, default=3.0, help="Movement duration.")
    parser.add_argument(
        "--post-hold-s",
        type=float,
        default=1.0,
        help="Hold-still duration after movement.",
    )
    parser.add_argument(
        "--hz", type=float, default=10.0, help="Sampling rate for action capture."
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )

    if args.hz <= 0:
        raise ValueError("--hz must be a positive number.")

    teleop_cls, config = load_teleop_config(args.config)
    teleop: Any = teleop_cls(config)
    phases = build_phases(args.settle_s, args.pre_hold_s, args.move_s, args.post_hold_s)
    period_s = 1.0 / args.hz
    total_s = total_duration(phases)

    logger.info(
        "Starting RG-only axis-alignment capture config=%s hz=%s total_s=%.3f",
        args.config,
        args.hz,
        total_s,
    )

    last_phase_index: int | None = None
    sequence_start: float | None = None

    try:
        teleop.connect(calibrate=True)
        sequence_start = time.monotonic()
        sample_index = 0

        while True:
            loop_started = time.monotonic()
            if sequence_start is None:
                raise RuntimeError(
                    "sequence_start must be initialized after teleop.connect()."
                )

            sequence_elapsed_s = loop_started - sequence_start
            if sequence_elapsed_s > total_s:
                break

            phase_index, phase, phase_elapsed_s = phase_at_time(
                phases, sequence_elapsed_s
            )
            if phase_index != last_phase_index:
                logger.info(
                    "PHASE %s/%s %s duration=%.3fs instruction=%s",
                    phase_index + 1,
                    len(phases),
                    phase.name,
                    phase.duration_s,
                    phase.instruction,
                )
                last_phase_index = phase_index

            action = teleop.get_action()
            record = build_capture_record(
                sample_index=sample_index,
                sequence_elapsed_s=sequence_elapsed_s,
                phase_index=phase_index,
                phase=phase,
                phase_elapsed_s=phase_elapsed_s,
                action=action,
            )
            print(json.dumps(record, sort_keys=True), flush=True)
            sample_index += 1

            remaining = period_s - (time.monotonic() - loop_started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        logger.info("Stopping timed Quest capture on keyboard interrupt.")
    finally:
        teleop.disconnect()


if __name__ == "__main__":
    main()
