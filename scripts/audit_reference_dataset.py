#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping


CANONICAL_CAMERA_KEYS = ["chest", "left_wrist", "right_wrist"]
REQUIRED_METADATA_ARTIFACTS = {
    "info": "meta/info.json",
    "stats": "meta/stats.json",
    "tasks": "meta/tasks.parquet",
    "episodes": "meta/episodes",
}
REQUIRED_SCALAR_FIELDS = ["timestamp", "frame_index", "episode_index", "index"]
JsonObject = dict[str, object]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Audit the frozen OpenArm reference dataset without mutating it."
    )
    parser.add_argument(
        "--dataset-root",
        default="data/openarm_phase1_test12",
        help="Path to the reference dataset root.",
    )
    parser.add_argument(
        "--report", help="Optional path to write a machine-readable audit report."
    )
    return parser.parse_args()


def load_json(path: Path) -> JsonObject:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"missing required JSON file: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"invalid JSON in {path}: {exc}") from exc


def find_features(info: Mapping[str, object]) -> JsonObject:
    for key in ("features", "feature", "schema", "columns"):
        value = info.get(key)
        if isinstance(value, dict):
            return value
    return {}


def feature_shape(feature: object) -> list[object]:
    if isinstance(feature, dict):
        shape = feature.get("shape")
        if isinstance(shape, list):
            return shape
        if isinstance(shape, tuple):
            return list(shape)
    return []


def feature_dtype(feature: object) -> str | None:
    if isinstance(feature, dict):
        dtype = feature.get("dtype")
        return dtype if isinstance(dtype, str) else None
    return None


def validate_dataset(dataset_root: Path) -> tuple[JsonObject, list[str]]:
    errors: list[str] = []

    required_paths = {
        name: dataset_root / relative_path
        for name, relative_path in REQUIRED_METADATA_ARTIFACTS.items()
    }
    missing_artifacts = [
        path.as_posix() for _, path in required_paths.items() if not path.exists()
    ]
    if missing_artifacts:
        errors.append(
            f"missing required metadata artifact(s): {sorted(missing_artifacts)}"
        )

    info = load_json(required_paths["info"]) if required_paths["info"].exists() else {}
    features = find_features(info)

    state_feature = features.get("observation.state")
    action_feature = features.get("action")
    timestamp_features = {
        name: features.get(name) for name in REQUIRED_SCALAR_FIELDS if name in features
    }
    image_features = {
        name: feature
        for name, feature in features.items()
        if isinstance(name, str) and name.startswith("observation.images.")
    }

    state_shape = feature_shape(state_feature)
    action_shape = feature_shape(action_feature)
    state_dtype = feature_dtype(state_feature)
    action_dtype = feature_dtype(action_feature)

    if state_shape != [48]:
        errors.append(f"observation.state.shape must be [48], got {state_shape}")
    if action_shape != [48]:
        errors.append(f"action.shape must be [48], got {action_shape}")
    if state_dtype not in (None, "float32"):
        errors.append(f"observation.state.dtype must be float32, got {state_dtype}")
    if action_dtype not in (None, "float32"):
        errors.append(f"action.dtype must be float32, got {action_dtype}")

    missing_timestamps = [
        name for name in REQUIRED_SCALAR_FIELDS if name not in timestamp_features
    ]
    if missing_timestamps:
        errors.append(f"missing timestamp fields: {missing_timestamps}")

    legacy_image_keys = [
        name
        for name in image_features
        if name not in {f"observation.images.{name}" for name in CANONICAL_CAMERA_KEYS}
    ]
    canonical_image_keys = [
        name
        for name in image_features
        if name in {f"observation.images.{name}" for name in CANONICAL_CAMERA_KEYS}
    ]

    stats_path = dataset_root / REQUIRED_METADATA_ARTIFACTS["stats"]
    stats = load_json(stats_path) if stats_path.exists() else {}
    reference_only = bool(
        info.get("reference_only")
        or info.get("reference_dataset_role") == "audit_and_shape_evidence_only"
        or stats.get("reference_only")
        or stats.get("reference_dataset_role") == "audit_and_shape_evidence_only"
    )
    if not reference_only:
        errors.append("dataset must be explicitly marked reference-only")

    episodes_dir = dataset_root / REQUIRED_METADATA_ARTIFACTS["episodes"]
    episode_count = (
        sum(1 for entry in episodes_dir.iterdir() if entry.is_file())
        if episodes_dir.exists()
        else 0
    )

    report = {
        "status": "pass" if not errors else "fail",
        "dataset_root": str(dataset_root),
        "reference_only": reference_only,
        "mutates_dataset": False,
        "reference_dataset_role": info.get(
            "reference_dataset_role", stats.get("reference_dataset_role")
        ),
        "required_metadata_artifacts": {
            name: str(path) for name, path in required_paths.items()
        },
        "legacy_image_keys": sorted(legacy_image_keys),
        "canonical_image_keys": sorted(canonical_image_keys),
        "shapes": {
            "observation.state": state_shape,
            "action": action_shape,
        },
        "timestamps": {
            "required_scalar_fields": REQUIRED_SCALAR_FIELDS,
            "present_scalar_fields": sorted(timestamp_features.keys()),
        },
        "episode_count": episode_count,
        "fps": info.get("fps"),
        "notes": ["reference dataset is audited read-only; no files are rewritten"],
    }
    return report, errors


def main() -> int:
    args = parse_args()
    dataset_root = Path(args.dataset_root)
    report, errors = validate_dataset(dataset_root)

    if args.report:
        Path(args.report).write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n"
        )

    if errors:
        failure = {
            "status": "fail",
            "dataset_root": str(dataset_root),
            "errors": errors,
        }
        sys.stderr.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
