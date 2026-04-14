#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
JsonObject = dict[str, object]
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


def get_prerequisites(manifest: JsonObject) -> list[JsonObject]:
    value = manifest.get("prerequisite_evidence")
    if not isinstance(value, list):
        return []

    return [item for item in value if isinstance(item, dict)]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate the unified data collection rollout gate."
    )
    parser.add_argument(
        "--manifest",
        required=True,
        help="Path to the rollout gate manifest JSON file.",
    )
    parser.add_argument(
        "--report",
        help="Optional path to write a machine-readable validation report.",
    )
    return parser.parse_args()


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_manifest(path: Path) -> JsonObject:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"manifest not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"manifest is not valid JSON: {path}: {exc}") from exc


def load_evidence(path: Path) -> JsonObject:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"required evidence missing: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"evidence is not valid JSON: {path}: {exc}") from exc


def validate_manifest(
    manifest: JsonObject, manifest_path: str
) -> tuple[JsonObject, list[str]]:
    errors: list[str] = []

    expect(
        manifest.get("contract_name") == "openarm_unified_data_collection_rollout_gate",
        "contract_name must be openarm_unified_data_collection_rollout_gate",
        errors,
    )
    expect(
        manifest.get("contract_version")
        == "openarm_unified_data_collection_rollout_gate/v1",
        "contract_version must be openarm_unified_data_collection_rollout_gate/v1",
        errors,
    )
    expect(
        manifest.get("status") == "authoritative",
        "status must be authoritative",
        errors,
    )
    expect(
        manifest.get("documentation_path")
        == ".sisyphus/docs/unified-data-collection-rollout-gate.md",
        "documentation_path must point to the rollout gate doc",
        errors,
    )
    expect(
        manifest.get("rollout_rule")
        == "broader recollection stays blocked until every prerequisite evidence artifact reports pass",
        "rollout_rule must freeze the broader recollection gate",
        errors,
    )

    prerequisites = get_prerequisites(manifest)
    expected_paths = [path for _, path in EXPECTED_PREREQUISITES]
    observed_paths = [entry.get("path") for entry in prerequisites]
    expect(
        observed_paths == expected_paths,
        f"prerequisite_evidence must list {expected_paths} in order",
        errors,
    )

    report_entries: list[JsonObject] = []
    for expected_role, expected_path in EXPECTED_PREREQUISITES:
        entry = next(
            (item for item in prerequisites if item.get("role") == expected_role),
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

        evidence_path = ROOT / expected_path
        evidence = load_evidence(evidence_path)
        expect(
            evidence.get("status") == "pass",
            f"evidence {expected_path} must report pass",
            errors,
        )
        if expected_role == "reference_audit":
            expect(
                evidence.get("reference_only") is True,
                "reference audit evidence must remain reference_only",
                errors,
            )
            expect(
                evidence.get("mutates_dataset") is False,
                "reference audit evidence must remain non-mutating",
                errors,
            )
        report_entries.append(
            {
                "role": expected_role,
                "path": expected_path,
                "status": evidence.get("status"),
            }
        )

    report = {
        "status": "pass" if not errors else "fail",
        "manifest": manifest_path,
        "gate_name": manifest.get("gate_name"),
        "rollout_rule": manifest.get("rollout_rule"),
        "prerequisite_evidence": report_entries,
        "broader_recollection_blocked_until_all_prerequisites_pass": True,
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
