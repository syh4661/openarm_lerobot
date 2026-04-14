#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
JsonObject = dict[str, Any]
EXPECTED_PREREQUISITES = [
    (
        "contract_validation",
        ".sisyphus/evidence/task-3-unified-contract-validation.json",
    ),
    (
        "semantic_registry_validation",
        ".sisyphus/evidence/task-3-unified-registry-validation.json",
    ),
    ("reference_audit", ".sisyphus/evidence/task-3-reference-audit.json"),
    (
        "recording_path_compatibility",
        ".sisyphus/evidence/task-4-lerobot-compat.json",
    ),
]
EXPECTED_VIEW = {
    "view_name": "pi05.action.commanded.chunked",
    "kind": "chunked_action_window",
    "source_group": "action.commanded",
    "source_storage_key": "action",
    "window_horizon": 50,
    "camera_inputs": [
        "observation.images.chest",
        "observation.images.left_wrist",
        "observation.images.right_wrist",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the unified derived view specification."
    )
    parser.add_argument(
        "--spec",
        required=True,
        help="Path to the derived-view spec JSON file.",
    )
    parser.add_argument(
        "--report",
        help="Optional path to write a machine-readable validation report.",
    )
    parser.add_argument(
        "--require-prerequisite",
        action="append",
        default=[],
        help="Additional prerequisite evidence path that must exist and report pass.",
    )
    return parser.parse_args()


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_json(path: Path, label: str) -> JsonObject:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"{label} not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"{label} is not valid JSON: {path}: {exc}") from exc


def validate_prerequisite(path_text: str, errors: list[str]) -> JsonObject:
    path = ROOT / path_text
    evidence = load_json(path, "required prerequisite evidence")
    expect(
        evidence.get("status") == "pass",
        f"required prerequisite evidence must report pass: {path_text}",
        errors,
    )
    return evidence


def validate_spec(
    spec: JsonObject, spec_path: str, extra_prereqs: list[str]
) -> tuple[JsonObject, list[str]]:
    errors: list[str] = []

    expect(
        spec.get("contract_name") == "openarm_unified_derived_view_spec",
        "contract_name must be openarm_unified_derived_view_spec",
        errors,
    )
    expect(
        spec.get("contract_version") == "openarm_unified_derived_view_spec/v1",
        "contract_version must be openarm_unified_derived_view_spec/v1",
        errors,
    )
    expect(
        spec.get("status") == "authoritative",
        "status must be authoritative",
        errors,
    )
    expect(
        spec.get("documentation_path") == ".sisyphus/docs/unified-derived-view-spec.md",
        "documentation_path must point to the derived-view doc",
        errors,
    )

    source_truth = spec.get("source_truth", {})
    expect(
        source_truth.get("raw_storage_is_authoritative") is True,
        "source_truth.raw_storage_is_authoritative must be true",
        errors,
    )
    expect(
        source_truth.get("derived_views_are_metadata_only") is True,
        "source_truth.derived_views_are_metadata_only must be true",
        errors,
    )
    expect(
        source_truth.get("pi0_5_style_slicing_is_reference_only") is True,
        "source_truth.pi0_5_style_slicing_is_reference_only must be true",
        errors,
    )
    expect(
        source_truth.get("must_not_mutate_raw_storage") is True,
        "source_truth.must_not_mutate_raw_storage must be true",
        errors,
    )

    derived_view_rules = spec.get("derived_view_rules", {})
    expect(
        derived_view_rules.get("metadata_file") == "meta/derived_views.json",
        "derived_view_rules.metadata_file must be meta/derived_views.json",
        errors,
    )
    expect(
        derived_view_rules.get("source_registry_path")
        == ".sisyphus/contracts/unified-camera-semantic-registry.json",
        "derived_view_rules.source_registry_path must point to the camera semantic registry",
        errors,
    )
    expect(
        derived_view_rules.get("source_registry_section") == "derived_view_rules",
        "derived_view_rules.source_registry_section must be derived_view_rules",
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

    views = derived_view_rules.get("views", {})
    pi05_view = views.get("pi05_action_chunk", {})
    expect(
        pi05_view.get("view_name") == EXPECTED_VIEW["view_name"],
        "derived_view_rules.views.pi05_action_chunk.view_name must be pi05.action.commanded.chunked",
        errors,
    )
    expect(
        pi05_view.get("kind") == EXPECTED_VIEW["kind"],
        "derived_view_rules.views.pi05_action_chunk.kind must be chunked_action_window",
        errors,
    )
    expect(
        pi05_view.get("source_group") == EXPECTED_VIEW["source_group"],
        "derived_view_rules.views.pi05_action_chunk.source_group must be action.commanded",
        errors,
    )
    expect(
        pi05_view.get("source_storage_key") == EXPECTED_VIEW["source_storage_key"],
        "derived_view_rules.views.pi05_action_chunk.source_storage_key must be action",
        errors,
    )
    expect(
        pi05_view.get("window", {}).get("horizon") == EXPECTED_VIEW["window_horizon"],
        "derived_view_rules.views.pi05_action_chunk.window.horizon must be 50",
        errors,
    )
    expect(
        pi05_view.get("camera_inputs") == EXPECTED_VIEW["camera_inputs"],
        f"derived_view_rules.views.pi05_action_chunk.camera_inputs must equal {EXPECTED_VIEW['camera_inputs']}",
        errors,
    )
    expect(
        pi05_view.get("raw_storage_mutation") is False,
        "derived_view_rules.views.pi05_action_chunk.raw_storage_mutation must be false",
        errors,
    )

    prerequisites = spec.get("prerequisite_evidence", [])
    expected_paths = [path for _, path in EXPECTED_PREREQUISITES]
    observed_paths = [
        entry.get("path") for entry in prerequisites if isinstance(entry, dict)
    ]
    expect(
        observed_paths == expected_paths,
        f"prerequisite_evidence must list {expected_paths} in order",
        errors,
    )

    report_entries: list[JsonObject] = []
    for expected_role, expected_path in EXPECTED_PREREQUISITES:
        entry = next(
            (
                item
                for item in prerequisites
                if isinstance(item, dict) and item.get("role") == expected_role
            ),
            {},
        )
        expect(
            entry.get("path") == expected_path,
            f"prerequisite {expected_role} must point to {expected_path}",
            errors,
        )
        expect(
            entry.get("required_status") == "pass",
            f"prerequisite {expected_role} must require pass status",
            errors,
        )
        evidence = validate_prerequisite(expected_path, errors)
        report_entries.append(
            {
                "role": expected_role,
                "path": expected_path,
                "status": evidence.get("status"),
            }
        )

    extra_entries: list[JsonObject] = []
    for path_text in extra_prereqs:
        evidence = validate_prerequisite(path_text, errors)
        extra_entries.append({"path": path_text, "status": evidence.get("status")})

    report = {
        "status": "pass" if not errors else "fail",
        "spec": spec_path,
        "derived_view": pi05_view.get("view_name"),
        "prerequisite_evidence": report_entries,
        "extra_prerequisites": extra_entries,
        "raw_storage_remains_authoritative": True,
        "derived_views_are_metadata_only": True,
    }
    return report, errors


def main() -> int:
    args = parse_args()
    spec_path = Path(args.spec)
    spec = load_json(spec_path, "spec")
    report, errors = validate_spec(spec, args.spec, args.require_prerequisite)

    if args.report:
        report_path = Path(args.report)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")

    if errors:
        failure = {"status": "fail", "spec": args.spec, "errors": errors}
        sys.stderr.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
