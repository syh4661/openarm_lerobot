#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


EXPECTED_CANONICAL_CAMERAS = ["chest", "left_wrist", "right_wrist"]
EXPECTED_CANONICAL_FEATURES = [
    f"observation.images.{name}" for name in EXPECTED_CANONICAL_CAMERAS
]
REQUIRED_METADATA_ARTIFACTS = {
    ("meta/info.json", "file"),
    ("meta/stats.json", "file"),
    ("meta/tasks.parquet", "file"),
    ("meta/episodes", "directory"),
}
REQUIRED_TIMING_FIELDS = ["timestamp", "frame_index", "episode_index", "index"]
REQUIRED_RAW_GROUPS = ["observation.state", "action.commanded", "timing"]
FORBIDDEN_CANONICAL_RAW_KEYS = {
    "head",
    "left_chest",
    "left_left_wrist",
    "right_right_wrist",
    "wrist_a",
    "wrist_b",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the unified rich dataset contract manifest."
    )
    parser.add_argument(
        "--manifest", required=True, help="Path to the contract manifest JSON file."
    )
    parser.add_argument(
        "--report", help="Optional path to write a machine-readable validation report."
    )
    return parser.parse_args()


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_manifest(path: Path) -> dict:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"manifest not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"manifest is not valid JSON: {path}: {exc}") from exc


def validate_manifest(manifest: dict, manifest_path: str) -> tuple[dict, list[str]]:
    errors: list[str] = []

    expect(
        manifest.get("contract_name") == "openarm_unified_rich_dataset_contract",
        "contract_name must be openarm_unified_rich_dataset_contract",
        errors,
    )
    expect(
        manifest.get("contract_version") == "openarm_unified_rich_dataset_contract/v1",
        "contract_version must be openarm_unified_rich_dataset_contract/v1",
        errors,
    )
    expect(
        manifest.get("status") == "authoritative",
        "status must be authoritative",
        errors,
    )
    expect(
        manifest.get("documentation_path")
        == ".sisyphus/docs/unified-rich-dataset-contract.md",
        "documentation_path must point to the unified rich dataset contract doc",
        errors,
    )

    authority = manifest.get("authority", {})
    expect(
        authority.get("recorder_base_path") == "scripts/run_record.sh",
        "authority.recorder_base_path must remain scripts/run_record.sh",
        errors,
    )
    expect(
        authority.get("reference_dataset_evidence")
        == "data/openarm_phase1_test12/meta/info.json",
        "authority.reference_dataset_evidence must reference data/openarm_phase1_test12/meta/info.json",
        errors,
    )

    dataset_root = manifest.get("dataset_root", {})
    expect(
        dataset_root.get("required_directories") == ["data", "meta"],
        "dataset_root.required_directories must equal ['data', 'meta']",
        errors,
    )
    expect(
        dataset_root.get("video_directory_when_video_enabled") == "videos",
        "dataset_root.video_directory_when_video_enabled must be videos",
        errors,
    )

    artifacts = {
        (entry.get("path"), entry.get("kind"))
        for entry in manifest.get("required_metadata_artifacts", [])
        if isinstance(entry, dict)
    }
    missing_artifacts = sorted(REQUIRED_METADATA_ARTIFACTS - artifacts)
    expect(
        not missing_artifacts,
        f"required_metadata_artifacts missing entries: {missing_artifacts}",
        errors,
    )

    camera_policy = manifest.get("camera_policy", {})
    canonical_cameras = camera_policy.get("canonical_camera_keys")
    canonical_features = camera_policy.get("canonical_feature_keys")
    reject_as_new_raw_keys = set(camera_policy.get("reject_as_new_raw_keys", []))
    legacy_aliases = camera_policy.get("legacy_aliases_audit_only", {})
    semantic_aliases = camera_policy.get("semantic_aliases_non_emitted", {})
    alias_policy = camera_policy.get("alias_policy", {})

    expect(
        canonical_cameras == EXPECTED_CANONICAL_CAMERAS,
        f"canonical_camera_keys must be exactly {EXPECTED_CANONICAL_CAMERAS}",
        errors,
    )
    expect(
        canonical_features == EXPECTED_CANONICAL_FEATURES,
        f"canonical_feature_keys must be exactly {EXPECTED_CANONICAL_FEATURES}",
        errors,
    )
    expect(
        not (set(canonical_cameras or []) & FORBIDDEN_CANONICAL_RAW_KEYS),
        f"unsupported canonical camera key(s): {sorted(set(canonical_cameras or []) & FORBIDDEN_CANONICAL_RAW_KEYS)}",
        errors,
    )
    expect(
        not (
            set(canonical_features or [])
            & {f"observation.images.{name}" for name in FORBIDDEN_CANONICAL_RAW_KEYS}
        ),
        f"unsupported canonical feature key(s): {sorted(set(canonical_features or []) & {f'observation.images.{name}' for name in FORBIDDEN_CANONICAL_RAW_KEYS})}",
        errors,
    )
    expect(
        semantic_aliases.get("head") == "chest",
        "semantic_aliases_non_emitted.head must map to chest",
        errors,
    )
    expect(
        "head" in reject_as_new_raw_keys,
        "camera_policy.reject_as_new_raw_keys must include head",
        errors,
    )
    expect(
        legacy_aliases.get("left_chest") == "chest",
        "legacy alias left_chest must map to chest",
        errors,
    )
    expect(
        legacy_aliases.get("left_left_wrist") == "left_wrist",
        "legacy alias left_left_wrist must map to left_wrist",
        errors,
    )
    expect(
        legacy_aliases.get("right_right_wrist") == "right_wrist",
        "legacy alias right_right_wrist must map to right_wrist",
        errors,
    )
    expect(
        alias_policy.get("legacy_aliases_audit_only") is True,
        "alias_policy.legacy_aliases_audit_only must be true",
        errors,
    )
    expect(
        alias_policy.get("semantic_aliases_non_emitted_only") is True,
        "alias_policy.semantic_aliases_non_emitted_only must be true",
        errors,
    )
    expect(
        alias_policy.get("allow_aliases_in_new_collection_output") is False,
        "alias_policy.allow_aliases_in_new_collection_output must be false",
        errors,
    )
    expect(
        alias_policy.get("allow_aliases_as_canonical_raw_keys") is False,
        "alias_policy.allow_aliases_as_canonical_raw_keys must be false",
        errors,
    )

    raw_storage_groups = manifest.get("raw_storage_groups", {})
    missing_groups = [
        name for name in REQUIRED_RAW_GROUPS if name not in raw_storage_groups
    ]
    expect(
        not missing_groups,
        f"raw_storage_groups missing required group(s): {missing_groups}",
        errors,
    )

    state_group = raw_storage_groups.get("observation.state", {})
    action_group = raw_storage_groups.get("action.commanded", {})
    timing_group = raw_storage_groups.get("timing", {})
    image_group = raw_storage_groups.get("observation.images", {})

    expect(
        state_group.get("storage_key") == "observation.state",
        "observation.state.storage_key must be observation.state",
        errors,
    )
    expect(
        action_group.get("storage_key") == "action",
        "action.commanded.storage_key must remain action for recorder compatibility",
        errors,
    )
    expect(
        timing_group.get("scalar_fields") == REQUIRED_TIMING_FIELDS,
        f"timing.scalar_fields must equal {REQUIRED_TIMING_FIELDS}",
        errors,
    )
    expect(
        image_group.get("storage_keys") == EXPECTED_CANONICAL_FEATURES,
        f"observation.images.storage_keys must equal {EXPECTED_CANONICAL_FEATURES}",
        errors,
    )

    timestamp_policy = manifest.get("timestamp_policy", {})
    expect(
        timestamp_policy.get("primary_timestamp_field") == "timestamp",
        "timestamp_policy.primary_timestamp_field must be timestamp",
        errors,
    )
    expect(
        timestamp_policy.get("timestamp_unit") == "seconds",
        "timestamp_policy.timestamp_unit must be seconds",
        errors,
    )
    expect(
        timestamp_policy.get("monotonic_within_episode") is True,
        "timestamp_policy.monotonic_within_episode must be true",
        errors,
    )
    expect(
        timestamp_policy.get("resume_must_preserve_timestamp_semantics") is True,
        "timestamp_policy.resume_must_preserve_timestamp_semantics must be true",
        errors,
    )

    units_policy = manifest.get("units_policy", {})
    state_action_policy = units_policy.get("state_and_action_policy", {})
    expect(
        units_policy.get("time") == "seconds",
        "units_policy.time must be seconds",
        errors,
    )
    expect(
        units_policy.get("image_dimensions") == "pixels",
        "units_policy.image_dimensions must be pixels",
        errors,
    )
    expect(
        units_policy.get("index_fields") == "counts",
        "units_policy.index_fields must be counts",
        errors,
    )
    expect(
        state_action_policy.get(
            "observation_state_and_action_commanded_must_use_matching_units"
        )
        is True,
        "units_policy.state_and_action_policy must preserve matching units between observation.state and action.commanded",
        errors,
    )
    expect(
        state_action_policy.get("derived_views_must_declare_any_unit_conversion")
        is True,
        "units_policy.state_and_action_policy must require explicit derived-view unit conversions",
        errors,
    )

    raw_vs_derived = manifest.get("raw_vs_derived_policy", {})
    expect(
        raw_vs_derived.get("raw_storage_is_authoritative") is True,
        "raw_vs_derived_policy.raw_storage_is_authoritative must be true",
        errors,
    )
    expect(
        raw_vs_derived.get("derived_views_are_non_authoritative") is True,
        "raw_vs_derived_policy.derived_views_are_non_authoritative must be true",
        errors,
    )
    expect(
        raw_vs_derived.get(
            "derived_views_must_be_materialized_from_metadata_without_mutating_raw_storage"
        )
        is True,
        "raw_vs_derived_policy must forbid mutating raw storage when creating derived views",
        errors,
    )
    expect(
        raw_vs_derived.get(
            "derived_views_must_not_introduce_new_canonical_raw_camera_keys"
        )
        is True,
        "raw_vs_derived_policy must forbid new canonical raw camera keys in derived views",
        errors,
    )

    compatibility_bridge = manifest.get("compatibility_bridge", {})
    current_layout = compatibility_bridge.get("current_recorder_feature_layout", {})
    expect(
        current_layout.get("action.commanded") == "action",
        "compatibility_bridge.current_recorder_feature_layout must map action.commanded to action",
        errors,
    )
    expect(
        compatibility_bridge.get("reference_dataset_role")
        == "audit_and_shape_evidence_only",
        "compatibility_bridge.reference_dataset_role must be audit_and_shape_evidence_only",
        errors,
    )

    report = {
        "status": "pass" if not errors else "fail",
        "manifest": manifest_path,
        "contract_version": manifest.get("contract_version"),
        "canonical_camera_keys": canonical_cameras,
        "canonical_feature_keys": canonical_features,
        "mandatory_raw_groups": REQUIRED_RAW_GROUPS,
        "timing_scalar_fields": timing_group.get("scalar_fields"),
        "alias_policy": {
            "legacy_aliases_audit_only": alias_policy.get("legacy_aliases_audit_only"),
            "semantic_aliases_non_emitted_only": alias_policy.get(
                "semantic_aliases_non_emitted_only"
            ),
            "reject_as_new_raw_keys": sorted(reject_as_new_raw_keys),
        },
        "raw_vs_derived_separation": {
            "raw_storage_is_authoritative": raw_vs_derived.get(
                "raw_storage_is_authoritative"
            ),
            "derived_views_are_non_authoritative": raw_vs_derived.get(
                "derived_views_are_non_authoritative"
            ),
            "materialize_without_mutating_raw_storage": raw_vs_derived.get(
                "derived_views_must_be_materialized_from_metadata_without_mutating_raw_storage"
            ),
        },
    }
    return report, errors


def main() -> int:
    args = parse_args()
    manifest_path = Path(args.manifest)
    manifest = load_manifest(manifest_path)
    report, errors = validate_manifest(manifest, args.manifest)

    if args.report:
        report_path = Path(args.report)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    if errors:
        failure = {"status": "fail", "manifest": args.manifest, "errors": errors}
        sys.stderr.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
