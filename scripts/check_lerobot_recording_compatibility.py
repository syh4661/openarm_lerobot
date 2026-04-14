#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from typing import Any
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUN_RECORD_SCRIPT = ROOT / "scripts" / "run_record.sh"
CONTRACT_PATH = ROOT / ".sisyphus" / "contracts" / "unified-rich-dataset-contract.json"
JsonObject = dict[str, Any]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that the existing LeRobot record wrapper/config flow stays "
            "compatible with the unified raw dataset contract."
        )
    )
    parser.add_argument(
        "--preset",
        default="rgb",
        choices=["nocam", "rgb", "full"],
        help="Existing run_record.sh preset to exercise.",
    )
    parser.add_argument(
        "--run-name",
        default="task4_compat",
        help="Synthetic run name used to materialize the generated config path.",
    )
    parser.add_argument(
        "--config-override",
        help="Optional config template path to feed through the wrapper for negative fixtures.",
    )
    parser.add_argument(
        "--report",
        help="Optional path to write a machine-readable compatibility report.",
    )
    return parser.parse_args()


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def load_json(path: Path) -> JsonObject:
    try:
        return json.loads(path.read_text())
    except FileNotFoundError as exc:
        raise SystemExit(f"required JSON file not found: {path}") from exc
    except json.JSONDecodeError as exc:
        raise SystemExit(f"invalid JSON file: {path}: {exc}") from exc


def collect_camera_keys(config: JsonObject) -> list[str]:
    robot = config.get("robot", {})
    keys: list[str] = []
    for arm_name in ("left_arm_config", "right_arm_config"):
        arm_config = robot.get(arm_name, {})
        cameras = arm_config.get("cameras", {})
        if isinstance(cameras, dict):
            keys.extend(key for key in cameras if isinstance(key, str))
    return keys


def run_wrapper(
    preset: str, run_name: str, config_override: str | None
) -> tuple[Path, JsonObject]:
    generated = ROOT / ".tmp" / f"record_{preset}_{run_name}.json"
    if generated.exists():
        generated.unlink()

    env = os.environ.copy()
    env["OPENARM_RECORD_COMPAT_ONLY"] = "1"
    if config_override:
        env["OPENARM_RECORD_TEMPLATE_OVERRIDE"] = config_override

    command = [str(RUN_RECORD_SCRIPT), preset, run_name]
    result = subprocess.run(
        command,
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        failure = {
            "status": "fail",
            "wrapper_command": command,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
        raise SystemExit(json.dumps(failure, indent=2, sort_keys=True))
    if not generated.exists():
        raise SystemExit(f"generated config missing after wrapper run: {generated}")
    return generated, load_json(generated)


def validate_config(
    generated_config: JsonObject,
    run_name: str,
    contract: JsonObject,
    generated_path: Path,
    preset: str,
) -> tuple[JsonObject, list[str]]:
    errors: list[str] = []
    dataset = generated_config.get("dataset", {})
    camera_policy = contract.get("camera_policy", {})
    compatibility_bridge = contract.get("compatibility_bridge", {})

    expected_data_root = ROOT / "data" / run_name
    expected_repo_id = f"local/{run_name}"
    camera_keys = collect_camera_keys(generated_config)
    canonical_keys = camera_policy.get("canonical_camera_keys", [])
    forbidden_keys = set(camera_policy.get("reject_as_new_raw_keys", []))
    canonical_features = [f"observation.images.{name}" for name in camera_keys]
    unsupported_keys = sorted([key for key in camera_keys if key not in canonical_keys])
    rejected_keys = sorted([key for key in camera_keys if key in forbidden_keys])
    video_enabled = bool(dataset.get("video"))
    camera_backed_output = bool(camera_keys)
    expected_feature_keys = camera_policy.get("canonical_feature_keys", [])
    canonical_key_set = sorted(canonical_keys)
    actual_key_set = sorted(camera_keys)
    expected_feature_set = sorted(expected_feature_keys)
    actual_feature_set = sorted(canonical_features)

    expect(
        contract.get("authority", {}).get("recorder_base_path")
        == "scripts/run_record.sh",
        "contract authority must keep scripts/run_record.sh as the recorder base path",
        errors,
    )
    expect(
        compatibility_bridge.get("current_recorder_feature_layout", {}).get(
            "action.commanded"
        )
        == "action",
        "compatibility_bridge must preserve action.commanded -> action storage compatibility",
        errors,
    )
    expect(
        dataset.get("root") == str(expected_data_root),
        f"dataset.root must reuse the existing LeRobot data base path {expected_data_root}",
        errors,
    )
    expect(
        dataset.get("repo_id") == expected_repo_id,
        f"dataset.repo_id must be {expected_repo_id}",
        errors,
    )
    expect(
        dataset.get("rename_map") == {},
        "dataset.rename_map must stay empty so camera keys are emitted canonically",
        errors,
    )
    expect(
        generated_path.name == f"record_{preset}_{run_name}.json",
        "generated config path must match the existing wrapper naming convention",
        errors,
    )

    if camera_backed_output:
        expect(
            video_enabled,
            "camera-backed compatibility flow must keep dataset.video enabled",
            errors,
        )
        expect(
            actual_key_set == canonical_key_set,
            f"camera-backed output must emit canonical camera keys exactly {canonical_key_set}, got {actual_key_set}",
            errors,
        )
        expect(
            actual_feature_set == expected_feature_set,
            f"camera-backed output must emit canonical raw feature keys exactly {expected_feature_set}, got {actual_feature_set}",
            errors,
        )
    else:
        expect(
            contract.get("camera_policy", {}).get("allow_no_camera_dataset") is True,
            "contract must continue to allow no-camera datasets for the nocam preset",
            errors,
        )
        expect(
            video_enabled is False,
            "nocam compatibility flow must keep dataset.video disabled",
            errors,
        )

    expect(
        not unsupported_keys,
        f"unsupported canonical output key(s): {unsupported_keys}",
        errors,
    )
    expect(
        not rejected_keys,
        f"unsupported canonical output key(s): {rejected_keys}",
        errors,
    )

    report = {
        "status": "pass" if not errors else "fail",
        "wrapper_command": [
            str(RUN_RECORD_SCRIPT),
            preset,
            run_name,
        ],
        "generated_config_path": str(generated_path),
        "contract_manifest": str(CONTRACT_PATH),
        "dataset_root": dataset.get("root"),
        "repo_id": dataset.get("repo_id"),
        "camera_backed_output": camera_backed_output,
        "video_enabled": video_enabled,
        "camera_keys": camera_keys,
        "camera_feature_keys": canonical_features,
        "contract_compatible_output_structure": {
            "required_directories": contract.get("dataset_root", {}).get(
                "required_directories", []
            ),
            "video_directory_when_video_enabled": contract.get("dataset_root", {}).get(
                "video_directory_when_video_enabled"
            ),
            "required_metadata_artifacts": contract.get(
                "required_metadata_artifacts", []
            ),
            "required_raw_storage_groups": sorted(
                contract.get("raw_storage_groups", {}).keys()
            ),
            "required_timing_fields": contract.get("raw_storage_groups", {})
            .get("timing", {})
            .get("scalar_fields", []),
            "action_commanded_storage_key": compatibility_bridge.get(
                "current_recorder_feature_layout", {}
            ).get("action.commanded"),
        },
        "canonical_camera_policy": {
            "expected_camera_keys": canonical_keys,
            "expected_feature_keys": expected_feature_keys,
            "observed_camera_keys": camera_keys,
            "observed_feature_keys": canonical_features,
            "reject_as_new_raw_keys": sorted(forbidden_keys),
            "semantic_aliases_non_emitted": camera_policy.get(
                "semantic_aliases_non_emitted", {}
            ),
        },
    }
    return report, errors


def main() -> int:
    args = parse_args()
    contract = load_json(CONTRACT_PATH)
    generated_path, generated_config = run_wrapper(
        args.preset, args.run_name, args.config_override
    )
    report, errors = validate_config(
        generated_config,
        args.run_name,
        contract,
        generated_path,
        args.preset,
    )

    if args.report:
        Path(args.report).write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n"
        )

    if errors:
        failure = {
            "status": "fail",
            "generated_config_path": str(generated_path),
            "errors": errors,
        }
        sys.stderr.write(json.dumps(failure, indent=2, sort_keys=True) + "\n")
        return 1

    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
