#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import cast


JsonScalar = str | int | float | bool | None
JsonValue = JsonScalar | list["JsonValue"] | dict[str, "JsonValue"]
JsonObject = dict[str, JsonValue]
JsonArray = list[JsonValue]
ReportValue = JsonValue | list[str]


class ParsedArgs(argparse.Namespace):
    trace: str = ""
    report: str | None = None


QUEST_SPATIAL_ACTION_FEATURES = (
    "quest.pos_delta.x",
    "quest.pos_delta.y",
    "quest.pos_delta.z",
    "quest.rot_delta.rx",
    "quest.rot_delta.ry",
    "quest.rot_delta.rz",
    "quest.gripper",
    "quest.enabled",
)
REQUIRED_TOP_LEVEL_FIELDS = (
    "schema_version",
    "replay_name",
    "control_rate_hz",
    "controller_mode",
    "samples",
)
REQUIRED_SAMPLE_FIELDS = (
    "t_s",
    "arm",
    *QUEST_SPATIAL_ACTION_FEATURES,
    "tracking_state",
    "expected_behavior",
)
ALLOWED_CONTROLLER_MODES = {"right", "bimanual"}
ALLOWED_ARMS = {"left", "right"}
ALLOWED_TRACKING_STATES = {"idle", "tracking", "missing", "stale", "invalid"}
FAIL_CLOSED_TRACKING_STATES = {"missing", "stale", "invalid"}
FAIL_CLOSED_EXPECTED_BEHAVIORS = {"hold", "stop"}


def parse_args() -> ParsedArgs:
    parser = argparse.ArgumentParser(
        description="Validate a Quest spatial replay trace fixture."
    )
    _ = parser.add_argument(
        "--trace", required=True, help="Path to the replay trace JSON file."
    )
    _ = parser.add_argument(
        "--report", help="Optional path to write a machine-readable validation report."
    )
    return parser.parse_args(namespace=ParsedArgs())


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def is_finite_number(value: JsonValue) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def is_schema_version_1(value: JsonValue) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value == 1


def validate_number(
    value: JsonValue,
    field_path: str,
    missing_fields: list[str],
    range_errors: list[str],
) -> bool:
    if value is None:
        missing_fields.append(field_path)
        return False
    if not is_finite_number(value):
        range_errors.append(f"{field_path} must be a finite number")
        return False
    return True


def load_trace(path: Path) -> tuple[JsonObject | None, str | None]:
    try:
        loaded = cast(object, json.loads(path.read_text()))
    except FileNotFoundError:
        return None, f"trace not found: {path}"
    except json.JSONDecodeError as exc:
        return None, f"trace is not valid JSON: {path}: {exc}"

    if not isinstance(loaded, dict):
        return None, "trace root must be a JSON object"
    loaded_object = cast(dict[object, object], loaded)
    if not all(isinstance(key, str) for key in loaded_object):
        return None, "trace root keys must be strings"
    return cast(JsonObject, loaded_object), None


def as_number(value: JsonValue) -> float:
    if not is_finite_number(value):
        raise AssertionError("value passed numeric validation without numeric type")
    return float(cast(int | float, value))


def as_samples(value: JsonValue) -> JsonArray:
    if not isinstance(value, list):
        return []
    return value


def validate_sample(
    sample: JsonValue,
    index: int,
    previous_t_s: float | None,
    missing_fields: list[str],
    range_errors: list[str],
    errors: list[str],
) -> tuple[float | None, str | None, bool]:
    sample_path = f"samples[{index}]"
    if not isinstance(sample, dict):
        errors.append(f"{sample_path} must be an object")
        return previous_t_s, None, False

    for field in REQUIRED_SAMPLE_FIELDS:
        if field not in sample:
            missing_fields.append(f"{sample_path}.{field}")

    t_s: float | None = previous_t_s
    if validate_number(sample.get("t_s"), f"{sample_path}.t_s", missing_fields, range_errors):
        t_s = as_number(sample["t_s"])
        if previous_t_s is not None and t_s < previous_t_s:
            range_errors.append(f"{sample_path}.t_s must be monotonic non-decreasing")

    arm = sample.get("arm")
    if arm is not None and arm not in ALLOWED_ARMS:
        errors.append(f"{sample_path}.arm must be one of {sorted(ALLOWED_ARMS)}")

    for field in QUEST_SPATIAL_ACTION_FEATURES:
        if field not in sample:
            continue
        if not validate_number(sample.get(field), f"{sample_path}.{field}", missing_fields, range_errors):
            continue
        if field == "quest.enabled" and sample[field] not in (0, 1):
            range_errors.append(f"{sample_path}.quest.enabled must be 0 or 1")
        if field == "quest.gripper" and not 0.0 <= as_number(sample[field]) <= 1.0:
            range_errors.append(f"{sample_path}.quest.gripper must be between 0 and 1")

    tracking_state = sample.get("tracking_state")
    if tracking_state is not None:
        if not isinstance(tracking_state, str):
            errors.append(f"{sample_path}.tracking_state must be a string")
        elif tracking_state not in ALLOWED_TRACKING_STATES:
            errors.append(
                f"{sample_path}.tracking_state must be one of {sorted(ALLOWED_TRACKING_STATES)}"
            )

    expected_behavior = sample.get("expected_behavior")
    if expected_behavior is not None:
        if not isinstance(expected_behavior, str):
            errors.append(f"{sample_path}.expected_behavior must be a string")
        elif not expected_behavior:
            errors.append(f"{sample_path}.expected_behavior must not be empty")

    enabled = sample.get("quest.enabled")
    fail_closed_condition = tracking_state in FAIL_CLOSED_TRACKING_STATES or enabled == 0
    fail_closed_counted = (
        fail_closed_condition and expected_behavior in FAIL_CLOSED_EXPECTED_BEHAVIORS
    )
    if fail_closed_condition and expected_behavior not in FAIL_CLOSED_EXPECTED_BEHAVIORS:
        errors.append(
            f"{sample_path} fail-closed sample must expect hold or stop behavior"
        )

    return t_s, arm if isinstance(arm, str) else None, fail_closed_counted


def validate_trace(
    trace: JsonObject, trace_path: str
) -> tuple[dict[str, ReportValue], list[str]]:
    errors: list[str] = []
    missing_fields: list[str] = []
    range_errors: list[str] = []
    arms_seen: set[str] = set()
    fail_closed_count = 0

    for field in REQUIRED_TOP_LEVEL_FIELDS:
        if field not in trace:
            missing_fields.append(field)

    expect(
        is_schema_version_1(trace.get("schema_version")),
        "schema_version must be integer 1",
        errors,
    )
    expect(
        isinstance(trace.get("replay_name"), str) and bool(trace.get("replay_name")),
        "replay_name must be a non-empty string",
        errors,
    )

    control_rate_hz = trace.get("control_rate_hz")
    if validate_number(control_rate_hz, "control_rate_hz", missing_fields, range_errors):
        numeric_control_rate_hz = as_number(control_rate_hz)
        if numeric_control_rate_hz <= 0.0:
            range_errors.append("control_rate_hz must be positive")

    controller_mode = trace.get("controller_mode")
    if controller_mode not in ALLOWED_CONTROLLER_MODES:
        errors.append(
            f"controller_mode must be one of {sorted(ALLOWED_CONTROLLER_MODES)}"
        )

    raw_samples = trace.get("samples")
    if not isinstance(raw_samples, list):
        errors.append("samples must be an array")
        samples = []
    else:
        samples = as_samples(raw_samples)

    if isinstance(raw_samples, list) and not samples:
        errors.append("samples must not be empty")

    previous_t_s: float | None = None
    first_t_s: float | None = None
    last_t_s: float | None = None
    for index, sample in enumerate(samples):
        current_t_s, arm, fail_closed_counted = validate_sample(
            sample, index, previous_t_s, missing_fields, range_errors, errors
        )
        if current_t_s is not None:
            if first_t_s is None:
                first_t_s = current_t_s
            last_t_s = current_t_s
            previous_t_s = current_t_s
        if arm is not None:
            arms_seen.add(arm)
        if fail_closed_counted:
            fail_closed_count += 1

    if controller_mode == "right":
        expect(arms_seen <= {"right"}, "right-mode traces may only contain right arm samples", errors)
    elif controller_mode == "bimanual":
        expect(
            arms_seen == {"left", "right"},
            "bimanual traces must contain both left and right arm samples",
            errors,
        )

    duration_s = 0.0
    if first_t_s is not None and last_t_s is not None:
        duration_s = max(0.0, last_t_s - first_t_s)

    all_errors = [*errors, *missing_fields, *range_errors]
    report = {
        "schema_valid": not all_errors,
        "missing_fields": missing_fields,
        "range_errors": range_errors,
        "sample_count": len(samples),
        "duration_s": duration_s,
        "arms": sorted(arms_seen),
        "status": "pass" if not all_errors else "fail",
        "trace_file": trace_path,
        "schema_version": trace.get("schema_version"),
        "control_rate_hz": control_rate_hz,
        "controller_mode": controller_mode,
        "fail_closed_count": fail_closed_count,
        "errors": errors,
    }
    return report, all_errors


def write_report(path: str | None, report: dict[str, ReportValue]) -> None:
    if not path:
        return
    report_path = Path(path)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    _ = report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


def main() -> int:
    args = parse_args()
    trace_path = Path(args.trace)
    trace, load_error = load_trace(trace_path)
    if load_error is not None:
        report: dict[str, ReportValue] = {
            "schema_valid": False,
            "missing_fields": [],
            "range_errors": [],
            "sample_count": 0,
            "duration_s": 0.0,
            "arms": [],
            "status": "fail",
            "trace_file": args.trace,
            "schema_version": None,
            "control_rate_hz": None,
            "controller_mode": None,
            "fail_closed_count": 0,
            "errors": [load_error],
        }
        write_report(args.report, report)
        _ = sys.stderr.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
        return 1

    if trace is None:
        raise AssertionError("load_trace returned no trace without a load error")

    report, errors = validate_trace(trace, args.trace)
    write_report(args.report, report)

    if errors:
        failure = {"status": "fail", "trace": args.trace, "errors": errors}
        _ = sys.stderr.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1

    _ = print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
