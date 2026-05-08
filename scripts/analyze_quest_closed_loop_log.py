#!/usr/bin/env python3

from __future__ import annotations

import argparse
from collections import Counter
import datetime as dt
import json
import re
from pathlib import Path
from typing import Any


QUEST_DEBUG_RE = re.compile(
    r"^(?P<ts>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}).*QUEST_DEBUG (?P<payload>\{.*\})"
)


def _load_joint_limits(config_path: Path | None) -> dict[str, tuple[float, float]]:
    if config_path is None:
        return {}

    raw = json.loads(config_path.read_text())
    limits = raw.get("robot", {}).get("joint_limits", {})
    parsed: dict[str, tuple[float, float]] = {}
    for name, values in limits.items():
        if not isinstance(values, list | tuple) or len(values) != 2:
            continue
        parsed[str(name)] = (float(values[0]), float(values[1]))
    return parsed


def _parse_log(
    log_path: Path,
) -> tuple[
    list[tuple[dt.datetime, dict[str, Any]]],
    int,
    int,
    list[tuple[dt.datetime, list[Any]]],
    dt.datetime | None,
    dt.datetime | None,
    list[tuple[dt.datetime, dict[str, Any]]],
]:
    tracking: list[tuple[dt.datetime, dict[str, Any]]] = []
    idle = 0
    delta_unavailable = 0
    commands: list[tuple[dt.datetime, list[Any]]] = []
    events: list[tuple[dt.datetime, dict[str, Any]]] = []
    first_ts: dt.datetime | None = None
    last_ts: dt.datetime | None = None

    for line in log_path.read_text(errors="replace").splitlines():
        match = QUEST_DEBUG_RE.match(line)
        if match is None:
            continue

        ts = dt.datetime.strptime(match.group("ts"), "%Y-%m-%d %H:%M:%S,%f")
        first_ts = first_ts or ts
        last_ts = ts

        try:
            payload = json.loads(match.group("payload"))
        except json.JSONDecodeError:
            continue

        events.append((ts, payload))
        event = payload.get("event")
        if event == "spatial_tracking":
            tracking.append((ts, payload))
        elif event == "spatial_idle_hold":
            idle += 1
        elif event == "spatial_delta_unavailable":
            delta_unavailable += 1
        elif event == "closed_loop_joint_command":
            commands.append((ts, payload.get("commanded_joint_angles_deg") or []))

    return tracking, idle, delta_unavailable, commands, first_ts, last_ts, events


def _mean_vec(rows: list[dict[str, Any]], key: str) -> list[float] | None:
    values = [row.get(key) for row in rows if row.get(key) is not None]
    if not values:
        return None
    return [sum(float(value[i]) for value in values) / len(values) for i in range(3)]


def _format_vec(values: list[float] | None) -> str:
    if values is None:
        return "n/a"
    return "[" + ", ".join(f"{value:.6f}" for value in values) + "]"


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    if len(values) == 1:
        return values[0]
    ordered = sorted(values)
    rank = (len(ordered) - 1) * percentile
    lower = int(rank)
    upper = min(lower + 1, len(ordered) - 1)
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _format_stats_ms(values_s: list[float]) -> str:
    if not values_s:
        return "n/a"
    p50 = _percentile(values_s, 0.50)
    p95 = _percentile(values_s, 0.95)
    p99 = _percentile(values_s, 0.99)
    assert p50 is not None and p95 is not None and p99 is not None
    return f"p50={p50 * 1000:.1f} p95={p95 * 1000:.1f} p99={p99 * 1000:.1f} ms"


def _timing_summary(
    events: list[tuple[dt.datetime, dict[str, Any]]],
) -> dict[str, Any]:
    teleop_events = [
        payload
        for _ts, payload in events
        if payload.get("event")
        in {
            "spatial_tracking",
            "spatial_idle_hold",
            "spatial_grip_settle",
            "spatial_delta_unavailable",
            "spatial_controller_unavailable",
        }
        and payload.get("tick_t_pre_teleop") is not None
    ]
    command_events = [
        payload
        for _ts, payload in events
        if payload.get("event") == "closed_loop_joint_command"
        and payload.get("tick_t_pre_send") is not None
    ]

    teleop_durations: list[float] = []
    for payload in teleop_events:
        try:
            teleop_durations.append(
                float(payload["tick_t_post_teleop"])
                - float(payload["tick_t_pre_teleop"])
            )
        except (KeyError, TypeError, ValueError):
            continue

    tick_intervals: list[float] = []
    previous: float | None = None
    for payload in teleop_events:
        try:
            current = float(payload["tick_t_pre_teleop"])
        except (TypeError, ValueError):
            continue
        if previous is not None:
            tick_intervals.append(current - previous)
        previous = current

    pre_send_latencies: list[float] = []
    for teleop_payload, command_payload in zip(teleop_events, command_events, strict=False):
        try:
            pre_send_latencies.append(
                float(command_payload["tick_t_pre_send"])
                - float(teleop_payload["tick_t_pre_teleop"])
            )
        except (KeyError, TypeError, ValueError):
            continue

    reasons = Counter()
    for _ts, payload in events:
        event = payload.get("event")
        if event == "spatial_delta_unavailable":
            reasons[str(payload.get("delta_unavailable_reason", "unknown"))] += 1
        elif event == "spatial_controller_unavailable":
            reasons[str(payload.get("controller_unavailable_reason", "unknown"))] += 1

    effective_input_rate_hz: float | None = None
    if teleop_events:
        starts = [
            float(payload["tick_t_pre_teleop"])
            for payload in teleop_events
            if payload.get("tick_t_pre_teleop") is not None
        ]
        if len(starts) >= 2:
            span_s = max(starts) - min(starts)
            if span_s > 0:
                effective_input_rate_hz = (len(starts) - 1) / span_s

    return {
        "teleop_durations": teleop_durations,
        "tick_intervals": tick_intervals,
        "pre_send_latencies": pre_send_latencies,
        "reasons": reasons,
        "effective_input_rate_hz": effective_input_rate_hz,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Summarize QUEST_DEBUG output from record_quest_closed_loop.py."
    )
    parser.add_argument("log", type=Path, help="Closed-loop runtime log.")
    parser.add_argument(
        "--joint-limits-config",
        type=Path,
        default=None,
        help="JSON config whose robot.joint_limits should bound commanded joints.",
    )
    parser.add_argument(
        "--motor-names",
        nargs="+",
        default=[
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "joint_7",
            "gripper",
        ],
        help="Motor names in commanded_joint_angles_deg order.",
    )
    parser.add_argument(
        "--window-s",
        type=float,
        default=10.0,
        help="Tracking summary window size in seconds.",
    )
    parser.add_argument(
        "--num-windows",
        type=int,
        default=3,
        help="Number of tracking windows to summarize.",
    )
    parser.add_argument(
        "--limit-eps",
        type=float,
        default=1e-6,
        help="Absolute tolerance for joint-limit comparisons.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    (
        tracking,
        idle,
        delta_unavailable,
        commands,
        first_ts,
        last_ts,
        events,
    ) = _parse_log(args.log)
    span_s = (last_ts - first_ts).total_seconds() if first_ts and last_ts else None

    print(f"log {args.log}")
    print(f"debug_span_s {span_s:.3f}" if span_s is not None else "debug_span_s n/a")
    print(
        f"tracking {len(tracking)} idle {idle} "
        f"delta_unavailable {delta_unavailable} commands {len(commands)}"
    )
    timing = _timing_summary(events)
    effective_input_rate_hz = timing["effective_input_rate_hz"]
    if effective_input_rate_hz is None:
        print("effective_input_rate_hz n/a")
    else:
        print(f"effective_input_rate_hz {effective_input_rate_hz:.2f}")
    print(
        "teleop_duration "
        f"{_format_stats_ms(timing['teleop_durations'])}"
    )
    print(
        "tick_interval "
        f"{_format_stats_ms(timing['tick_intervals'])}"
    )
    print(
        "pre_send_latency "
        f"{_format_stats_ms(timing['pre_send_latencies'])}"
    )
    print(f"delta_unavailable_reasons {dict(timing['reasons'])}")

    if tracking:
        tracking_t0 = tracking[0][0]
        for index in range(args.num_windows):
            start_s = index * args.window_s
            end_s = start_s + args.window_s
            rows = [
                payload
                for ts, payload in tracking
                if start_s <= (ts - tracking_t0).total_seconds() < end_s
            ]
            print(f"window {start_s:.0f}-{end_s:.0f}s rows {len(rows)}")
            if not rows:
                continue
            print(
                "  calibrated_pos_delta "
                f"{_format_vec(_mean_vec(rows, 'calibrated_pos_delta'))}"
            )
            print(
                "  scaled_pos_delta "
                f"{_format_vec(_mean_vec(rows, 'scaled_pos_delta'))}"
            )
            print(
                "  clipped_pos_delta "
                f"{_format_vec(_mean_vec(rows, 'clipped_pos_delta'))}"
            )
            print(
                "  clipped "
                f"{sum(1 for row in rows if row.get('clipped_by_max_ee_step'))}"
            )

    joint_limits = _load_joint_limits(args.joint_limits_config)
    if not joint_limits:
        return

    mins = [float("inf")] * len(args.motor_names)
    maxs = [float("-inf")] * len(args.motor_names)
    violating_frames = 0
    first_violations: list[tuple[str, list[tuple[str, float, float, float]]]] = []

    for ts, values in commands:
        if len(values) < len(args.motor_names):
            continue
        frame_violations: list[tuple[str, float, float, float]] = []
        for index, name in enumerate(args.motor_names):
            value = float(values[index])
            mins[index] = min(mins[index], value)
            maxs[index] = max(maxs[index], value)
            if name not in joint_limits:
                continue
            low, high = joint_limits[name]
            if value < low - args.limit_eps or value > high + args.limit_eps:
                frame_violations.append((name, value, low, high))
        if frame_violations:
            violating_frames += 1
            if len(first_violations) < 10:
                first_violations.append((str(ts.time()), frame_violations))

    print(f"violating_cmd_frames {violating_frames}")
    print("mins [" + ", ".join(f"{value:.3f}" for value in mins) + "]")
    print("maxs [" + ", ".join(f"{value:.3f}" for value in maxs) + "]")
    print(f"first_violations {first_violations}")


if __name__ == "__main__":
    main()
