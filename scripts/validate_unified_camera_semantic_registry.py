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
EXPECTED_METADATA_ARTIFACTS = {
    "camera_alias_metadata": ("meta/camera_aliases.json", "camera_registry"),
    "semantic_slice_metadata": ("meta/semantic_slices.json", "semantic_groups"),
    "derived_view_metadata": ("meta/derived_views.json", "derived_view_rules"),
}
FORBIDDEN_OUTPUT_KEYS = {
    "head",
    "left_chest",
    "left_left_wrist",
    "right_right_wrist",
    "wrist_a",
    "wrist_b",
}
EXPECTED_LEGACY_ALIASES = {
    "left_chest": "chest",
    "left_left_wrist": "left_wrist",
    "right_right_wrist": "right_wrist",
    "wrist_a": "left_wrist",
    "wrist_b": "right_wrist",
}
EXPECTED_SEGMENTS = [
    ("left_arm", 0, 21),
    ("left_gripper", 21, 24),
    ("right_arm", 24, 45),
    ("right_gripper", 45, 48),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the unified camera and semantic registry manifest."
    )
    parser.add_argument(
        "--manifest", required=True, help="Path to the registry manifest JSON file."
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


def validate_metadata_artifacts(metadata_artifacts: dict, errors: list[str]) -> None:
    for artifact_name, (
        expected_path,
        expected_section,
    ) in EXPECTED_METADATA_ARTIFACTS.items():
        artifact = metadata_artifacts.get(artifact_name, {})
        expect(
            artifact.get("dataset_relative_path") == expected_path,
            f"metadata_artifacts.{artifact_name}.dataset_relative_path must be {expected_path}",
            errors,
        )
        expect(
            artifact.get("kind") == "file",
            f"metadata_artifacts.{artifact_name}.kind must be file",
            errors,
        )
        expect(
            artifact.get("format") == "json",
            f"metadata_artifacts.{artifact_name}.format must be json",
            errors,
        )
        expect(
            artifact.get("authority_section") == expected_section,
            f"metadata_artifacts.{artifact_name}.authority_section must be {expected_section}",
            errors,
        )


def validate_camera_registry(
    camera_registry: dict, errors: list[str]
) -> tuple[list[str], list[str]]:
    canonical_cameras = camera_registry.get("canonical_new_dataset_keys")
    canonical_features = camera_registry.get("canonical_raw_feature_keys")
    legacy_aliases = camera_registry.get("legacy_aliases_audit_only", {})
    semantic_aliases = camera_registry.get("semantic_aliases_non_emitted", {})
    reject_keys = set(camera_registry.get("reject_as_new_output_keys", []))
    alias_policy = camera_registry.get("alias_policy", {})
    source_alignment = camera_registry.get("source_alignment", {})

    expect(
        canonical_cameras == EXPECTED_CANONICAL_CAMERAS,
        f"camera_registry.canonical_new_dataset_keys must be exactly {EXPECTED_CANONICAL_CAMERAS}",
        errors,
    )
    expect(
        canonical_features == EXPECTED_CANONICAL_FEATURES,
        f"camera_registry.canonical_raw_feature_keys must be exactly {EXPECTED_CANONICAL_FEATURES}",
        errors,
    )
    forbidden_cameras = sorted(set(canonical_cameras or []) & FORBIDDEN_OUTPUT_KEYS)
    expect(
        not forbidden_cameras,
        f"unsupported canonical new-output key(s): {forbidden_cameras}",
        errors,
    )
    forbidden_features = sorted(
        set(canonical_features or [])
        & {f"observation.images.{name}" for name in FORBIDDEN_OUTPUT_KEYS}
    )
    expect(
        not forbidden_features,
        f"unsupported canonical raw feature key(s): {forbidden_features}",
        errors,
    )
    expect(
        legacy_aliases == EXPECTED_LEGACY_ALIASES,
        f"camera_registry.legacy_aliases_audit_only must equal {EXPECTED_LEGACY_ALIASES}",
        errors,
    )
    expect(
        semantic_aliases.get("head") == "chest",
        "camera_registry.semantic_aliases_non_emitted.head must map to chest",
        errors,
    )
    expect(
        reject_keys == FORBIDDEN_OUTPUT_KEYS,
        f"camera_registry.reject_as_new_output_keys must equal {sorted(FORBIDDEN_OUTPUT_KEYS)}",
        errors,
    )
    expect(
        alias_policy.get("legacy_aliases_audit_only") is True,
        "camera_registry.alias_policy.legacy_aliases_audit_only must be true",
        errors,
    )
    expect(
        alias_policy.get("semantic_aliases_non_emitted_only") is True,
        "camera_registry.alias_policy.semantic_aliases_non_emitted_only must be true",
        errors,
    )
    expect(
        alias_policy.get("allow_aliases_in_new_collection_output") is False,
        "camera_registry.alias_policy.allow_aliases_in_new_collection_output must be false",
        errors,
    )
    expect(
        alias_policy.get("allow_aliases_as_canonical_raw_keys") is False,
        "camera_registry.alias_policy.allow_aliases_as_canonical_raw_keys must be false",
        errors,
    )
    expect(
        alias_policy.get("reject_ambiguous_wrist_names") is True,
        "camera_registry.alias_policy.reject_ambiguous_wrist_names must be true",
        errors,
    )
    expect(
        source_alignment.get("rgb_config") == "configs/record_rgb.json",
        "camera_registry.source_alignment.rgb_config must be configs/record_rgb.json",
        errors,
    )
    expect(
        source_alignment.get("legacy_inventory_reference")
        == "configs/realsense_3cam_mapping.yaml",
        "camera_registry.source_alignment.legacy_inventory_reference must be configs/realsense_3cam_mapping.yaml",
        errors,
    )
    expect(
        source_alignment.get("camera_alias_metadata_path")
        == "meta/camera_aliases.json",
        "camera_registry.source_alignment.camera_alias_metadata_path must be meta/camera_aliases.json",
        errors,
    )
    return canonical_cameras or [], canonical_features or []


def validate_observation_state(group: dict, errors: list[str]) -> None:
    ordering = group.get("ordering", {})
    segments = ordering.get("segments", [])
    expect(
        group.get("storage_key") == "observation.state",
        "semantic_groups.observation.state.storage_key must be observation.state",
        errors,
    )
    expect(
        group.get("metadata_file") == "meta/semantic_slices.json",
        "semantic_groups.observation.state.metadata_file must be meta/semantic_slices.json",
        errors,
    )
    expect(
        group.get("required") is True,
        "semantic_groups.observation.state.required must be true",
        errors,
    )
    expect(
        group.get("raw_vs_derived") == "raw",
        "semantic_groups.observation.state.raw_vs_derived must be raw",
        errors,
    )
    expect(
        group.get("dtype") == "float32",
        "semantic_groups.observation.state.dtype must be float32",
        errors,
    )
    expect(
        group.get("shape") == [48],
        "semantic_groups.observation.state.shape must be [48]",
        errors,
    )
    expect(
        group.get("names_source")
        == "data/openarm_phase1_test12/meta/info.json#/features/observation.state/names",
        "semantic_groups.observation.state.names_source must point to the reference dataset names",
        errors,
    )
    expect(
        group.get("unit_families") == [".pos", ".vel", ".torque"],
        "semantic_groups.observation.state.unit_families must be ['.pos', '.vel', '.torque']",
        errors,
    )
    expect(
        ordering.get("policy") == "explicit_segments",
        "semantic_groups.observation.state.ordering.policy must be explicit_segments",
        errors,
    )
    expect(
        ordering.get("segment_order") == [name for name, _, _ in EXPECTED_SEGMENTS],
        "semantic_groups.observation.state.ordering.segment_order must preserve left/right arm and gripper grouping",
        errors,
    )
    expect(
        ordering.get("suffix_cycle") == [".pos", ".vel", ".torque"],
        "semantic_groups.observation.state.ordering.suffix_cycle must be ['.pos', '.vel', '.torque']",
        errors,
    )
    expect(
        len(segments) == len(EXPECTED_SEGMENTS),
        "semantic_groups.observation.state.ordering.segments must declare four explicit segments",
        errors,
    )

    for segment, expected in zip(segments, EXPECTED_SEGMENTS):
        expected_name, expected_start, expected_end = expected
        expect(
            segment.get("name") == expected_name,
            f"semantic_groups.observation.state.ordering segment must be {expected_name}",
            errors,
        )
        expect(
            segment.get("start") == expected_start,
            f"segment {expected_name} start must be {expected_start}",
            errors,
        )
        expect(
            segment.get("end") == expected_end,
            f"segment {expected_name} end must be {expected_end}",
            errors,
        )
        expect(
            segment.get("count") == expected_end - expected_start,
            f"segment {expected_name} count must be {expected_end - expected_start}",
            errors,
        )


def validate_action_groups(semantic_groups: dict, errors: list[str]) -> None:
    action_commanded = semantic_groups.get("action.commanded", {})
    action_applied = semantic_groups.get("action.applied", {})

    expect(
        action_commanded.get("storage_key") == "action",
        "semantic_groups.action.commanded.storage_key must remain action",
        errors,
    )
    expect(
        action_commanded.get("metadata_file") == "meta/semantic_slices.json",
        "semantic_groups.action.commanded.metadata_file must be meta/semantic_slices.json",
        errors,
    )
    expect(
        action_commanded.get("required") is True,
        "semantic_groups.action.commanded.required must be true",
        errors,
    )
    expect(
        action_commanded.get("raw_vs_derived") == "raw",
        "semantic_groups.action.commanded.raw_vs_derived must be raw",
        errors,
    )
    expect(
        action_commanded.get("shape") == [48],
        "semantic_groups.action.commanded.shape must be [48]",
        errors,
    )
    expect(
        action_commanded.get("names_source")
        == "data/openarm_phase1_test12/meta/info.json#/features/action/names",
        "semantic_groups.action.commanded.names_source must point to the reference dataset names",
        errors,
    )
    expect(
        action_commanded.get("ordering", {}).get("policy")
        == "same_as_observation.state",
        "semantic_groups.action.commanded.ordering.policy must be same_as_observation.state",
        errors,
    )
    expect(
        action_commanded.get("ordering", {}).get("source_group") == "observation.state",
        "semantic_groups.action.commanded.ordering.source_group must be observation.state",
        errors,
    )

    expect(
        action_applied.get("storage_key_when_present") == "action.applied",
        "semantic_groups.action.applied.storage_key_when_present must be action.applied",
        errors,
    )
    expect(
        action_applied.get("metadata_file") == "meta/semantic_slices.json",
        "semantic_groups.action.applied.metadata_file must be meta/semantic_slices.json",
        errors,
    )
    expect(
        action_applied.get("required") is False,
        "semantic_groups.action.applied.required must be false",
        errors,
    )
    expect(
        action_applied.get("raw_vs_derived") == "optional_raw",
        "semantic_groups.action.applied.raw_vs_derived must be optional_raw",
        errors,
    )
    expect(
        action_applied.get("present_in_current_recorder_output") is False,
        "semantic_groups.action.applied.present_in_current_recorder_output must be false",
        errors,
    )
    expect(
        action_applied.get("ordering", {}).get("policy") == "same_as_observation.state",
        "semantic_groups.action.applied.ordering.policy must be same_as_observation.state",
        errors,
    )


def validate_derived_view_rules(derived_view_rules: dict, errors: list[str]) -> None:
    views = derived_view_rules.get("views", {})
    pi05_view = views.get("pi05_action_chunk", {})
    relative_view = views.get("relative_action_delta", {})
    rgb_subset_view = views.get("rgb_camera_subset", {})

    expect(
        derived_view_rules.get("metadata_file") == "meta/derived_views.json",
        "derived_view_rules.metadata_file must be meta/derived_views.json",
        errors,
    )
    expect(
        derived_view_rules.get("raw_storage_remains_authoritative") is True,
        "derived_view_rules.raw_storage_remains_authoritative must be true",
        errors,
    )
    expect(
        derived_view_rules.get("materialize_from_metadata_only") is True,
        "derived_view_rules.materialize_from_metadata_only must be true",
        errors,
    )
    expect(
        derived_view_rules.get("derived_views_are_non_authoritative") is True,
        "derived_view_rules.derived_views_are_non_authoritative must be true",
        errors,
    )
    expect(
        derived_view_rules.get("must_not_create_new_canonical_raw_keys") is True,
        "derived_view_rules.must_not_create_new_canonical_raw_keys must be true",
        errors,
    )

    expect(
        pi05_view.get("view_name") == "pi05.action.commanded.chunked",
        "derived_view_rules.views.pi05_action_chunk.view_name must be pi05.action.commanded.chunked",
        errors,
    )
    expect(
        pi05_view.get("kind") == "chunked_action_window",
        "derived_view_rules.views.pi05_action_chunk.kind must be chunked_action_window",
        errors,
    )
    expect(
        pi05_view.get("source_group") == "action.commanded",
        "derived_view_rules.views.pi05_action_chunk.source_group must be action.commanded",
        errors,
    )
    expect(
        pi05_view.get("source_storage_key") == "action",
        "derived_view_rules.views.pi05_action_chunk.source_storage_key must be action",
        errors,
    )
    expect(
        pi05_view.get("window", {}).get("horizon") == 50,
        "derived_view_rules.views.pi05_action_chunk.window.horizon must be 50",
        errors,
    )
    expect(
        pi05_view.get("camera_inputs") == EXPECTED_CANONICAL_FEATURES,
        f"derived_view_rules.views.pi05_action_chunk.camera_inputs must equal {EXPECTED_CANONICAL_FEATURES}",
        errors,
    )
    expect(
        pi05_view.get("raw_storage_mutation") is False,
        "derived_view_rules.views.pi05_action_chunk.raw_storage_mutation must be false",
        errors,
    )

    expect(
        relative_view.get("view_name") == "relative.action.commanded_delta",
        "derived_view_rules.views.relative_action_delta.view_name must be relative.action.commanded_delta",
        errors,
    )
    expect(
        relative_view.get("kind") == "relative_transform",
        "derived_view_rules.views.relative_action_delta.kind must be relative_transform",
        errors,
    )
    expect(
        relative_view.get("reference_group") == "observation.state",
        "derived_view_rules.views.relative_action_delta.reference_group must be observation.state",
        errors,
    )
    expect(
        relative_view.get("unit_conversion_declared_in_metadata") is True,
        "derived_view_rules.views.relative_action_delta.unit_conversion_declared_in_metadata must be true",
        errors,
    )
    expect(
        relative_view.get("raw_storage_mutation") is False,
        "derived_view_rules.views.relative_action_delta.raw_storage_mutation must be false",
        errors,
    )

    expect(
        rgb_subset_view.get("view_name") == "camera_subset.rgb_only",
        "derived_view_rules.views.rgb_camera_subset.view_name must be camera_subset.rgb_only",
        errors,
    )
    expect(
        rgb_subset_view.get("kind") == "camera_subset_export",
        "derived_view_rules.views.rgb_camera_subset.kind must be camera_subset_export",
        errors,
    )
    expect(
        rgb_subset_view.get("allowed_camera_keys") == EXPECTED_CANONICAL_CAMERAS,
        f"derived_view_rules.views.rgb_camera_subset.allowed_camera_keys must equal {EXPECTED_CANONICAL_CAMERAS}",
        errors,
    )
    expect(
        rgb_subset_view.get("raw_storage_mutation") is False,
        "derived_view_rules.views.rgb_camera_subset.raw_storage_mutation must be false",
        errors,
    )


def validate_manifest(manifest: dict, manifest_path: str) -> tuple[dict, list[str]]:
    errors: list[str] = []

    expect(
        manifest.get("contract_name") == "openarm_unified_camera_semantic_registry",
        "contract_name must be openarm_unified_camera_semantic_registry",
        errors,
    )
    expect(
        manifest.get("contract_version")
        == "openarm_unified_camera_semantic_registry/v1",
        "contract_version must be openarm_unified_camera_semantic_registry/v1",
        errors,
    )
    expect(
        manifest.get("status") == "authoritative",
        "status must be authoritative",
        errors,
    )
    expect(
        manifest.get("documentation_path")
        == ".sisyphus/docs/unified-camera-semantic-registry.md",
        "documentation_path must point to the unified camera and semantic registry doc",
        errors,
    )

    authority = manifest.get("authority", {})
    expect(
        authority.get("owner_contract")
        == ".sisyphus/contracts/unified-rich-dataset-contract.json",
        "authority.owner_contract must point to the unified rich dataset contract",
        errors,
    )
    expect(
        authority.get("recorder_base_path") == "scripts/run_record.sh",
        "authority.recorder_base_path must remain scripts/run_record.sh",
        errors,
    )
    expect(
        authority.get("reference_dataset_evidence")
        == "data/openarm_phase1_test12/meta/info.json",
        "authority.reference_dataset_evidence must point to data/openarm_phase1_test12/meta/info.json",
        errors,
    )

    validate_metadata_artifacts(manifest.get("metadata_artifacts", {}), errors)
    canonical_cameras, canonical_features = validate_camera_registry(
        manifest.get("camera_registry", {}), errors
    )

    semantic_groups = manifest.get("semantic_groups", {})
    missing_groups = [
        name
        for name in ["observation.state", "action.commanded", "action.applied"]
        if name not in semantic_groups
    ]
    expect(
        not missing_groups,
        f"semantic_groups missing required declarations: {missing_groups}",
        errors,
    )
    validate_observation_state(semantic_groups.get("observation.state", {}), errors)
    validate_action_groups(semantic_groups, errors)
    validate_derived_view_rules(manifest.get("derived_view_rules", {}), errors)

    report = {
        "status": "pass" if not errors else "fail",
        "manifest": manifest_path,
        "contract_version": manifest.get("contract_version"),
        "canonical_camera_keys": canonical_cameras,
        "canonical_raw_feature_keys": canonical_features,
        "metadata_files": {
            name: artifact.get("dataset_relative_path")
            for name, artifact in manifest.get("metadata_artifacts", {}).items()
            if isinstance(artifact, dict)
        },
        "semantic_groups": sorted(semantic_groups.keys()),
        "derived_views": sorted(
            manifest.get("derived_view_rules", {}).get("views", {}).keys()
        ),
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
