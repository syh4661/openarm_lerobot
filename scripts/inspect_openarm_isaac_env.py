#!/usr/bin/env python3

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any, TypedDict, cast


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "configs" / "record_quest_right_nocam.json"
DEFAULT_ENV_NAME = "Isaac-Reach-OpenArm-v0"
FAIL_REASONS = {
    "isaac_unavailable",
    "openarm_isaac_unavailable",
    "env_unavailable",
}


class ParsedArgs(argparse.Namespace):
    report: Path = Path()
    env_name: str = DEFAULT_ENV_NAME
    include_play: bool = False
    instantiate_headless: bool = False


class IsaacReport(TypedDict):
    schema_version: int
    gate_name: str
    status: str
    reason: str
    env_name: str
    requested_env_name: str
    matching_env_names: list[str]
    headless_supported: bool
    headless_instantiated: bool
    asset_path: str | None
    joint_names: list[str]
    joint_names_discovered: bool
    tcp_frame: str
    tcp_frame_discovered: bool
    gripper_action: dict[str, Any]
    control_frequency_hz: float | None
    availability: dict[str, str]
    errors: list[str]
    warnings: list[str]
    metadata_sources: list[str]
    bootstrap: dict[str, Any]


def parse_args() -> ParsedArgs:
    parser = argparse.ArgumentParser(
        description="Safely inspect the local Isaac Lab OpenArm environment registration."
    )
    _ = parser.add_argument("--report", type=Path, required=True, help="JSON report path.")
    _ = parser.add_argument(
        "--env-name",
        default=DEFAULT_ENV_NAME,
        help="Isaac/OpenArm gym environment ID to inspect.",
    )
    _ = parser.add_argument(
        "--include-play",
        action="store_true",
        help="Include Play-v0 environments in discovered OpenArm env IDs.",
    )
    _ = parser.add_argument(
        "--instantiate-headless",
        action="store_true",
        help=(
            "Attempt a safe headless gym.make() to resolve runtime joint/body names. "
            "Default only inspects registration/static config."
        ),
    )
    return parser.parse_args(namespace=ParsedArgs())


def bootstrap_paths() -> dict[str, Any]:
    inserted_paths: list[str] = []
    for source_path in (REPO_ROOT / "src", REPO_ROOT.parent / "lerobot" / "src"):
        if source_path.exists():
            sys.path.insert(0, str(source_path))
            inserted_paths.append(str(source_path))

    ros_package_path_updated = False
    if (REPO_ROOT.parent / "openarm_description" / "package.xml").exists():
        ros_package_paths = [
            path for path in os.environ.get("ROS_PACKAGE_PATH", "").split(os.pathsep) if path
        ]
        workspace_path = str(REPO_ROOT.parent)
        if workspace_path not in ros_package_paths:
            os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(
                [workspace_path, *ros_package_paths]
            )
            ros_package_path_updated = True

    return {
        "repo_root": str(REPO_ROOT),
        "inserted_sys_paths": inserted_paths,
        "ros_package_path": os.environ.get("ROS_PACKAGE_PATH", ""),
        "ros_package_path_updated": ros_package_path_updated,
    }


def load_json_object(path: Path) -> dict[str, Any]:
    loaded = json.loads(path.read_text())
    if not isinstance(loaded, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return cast(dict[str, Any], loaded)


def fallback_metadata() -> dict[str, Any]:
    config = load_json_object(DEFAULT_CONFIG)
    robot = cast(dict[str, Any], config.get("robot", {}))
    teleop = cast(dict[str, Any], config.get("teleop", {}))
    dataset = cast(dict[str, Any], config.get("dataset", {}))
    joint_limits = cast(dict[str, Any], robot.get("joint_limits", {}))
    gripper_range = teleop.get("gripper_range_deg", joint_limits.get("gripper"))
    if not isinstance(gripper_range, list):
        gripper_range = [-65.0, 0.0]

    return {
        "asset_path": None,
        "joint_names": list(
            cast(
                list[str],
                teleop.get(
                    "urdf_joint_names",
                    [f"openarm_joint{index}" for index in range(1, 8)],
                ),
            )
        ),
        "tcp_frame": str(teleop.get("target_frame", "openarm_hand_tcp")),
        "gripper_action": {
            "source": "fallback_config",
            "type": "normalized_to_degrees",
            "command_field": "quest.gripper",
            "output_field": "gripper_vel",
            "range_deg": gripper_range,
        },
        "control_frequency_hz": float(dataset.get("fps", 30.0)),
    }


def base_report(args: ParsedArgs, bootstrap: dict[str, Any]) -> IsaacReport:
    fallback = fallback_metadata()
    return IsaacReport(
        schema_version=1,
        gate_name="inspect_openarm_isaac_env",
        status="fail",
        reason="env_unavailable",
        env_name=args.env_name,
        requested_env_name=args.env_name,
        matching_env_names=[],
        headless_supported=False,
        headless_instantiated=False,
        asset_path=cast(str | None, fallback["asset_path"]),
        joint_names=cast(list[str], fallback["joint_names"]),
        joint_names_discovered=False,
        tcp_frame=cast(str, fallback["tcp_frame"]),
        tcp_frame_discovered=False,
        gripper_action=cast(dict[str, Any], fallback["gripper_action"]),
        control_frequency_hz=cast(float, fallback["control_frequency_hz"]),
        availability={
            "isaac_lab": "unknown",
            "gym": "unknown",
            "openarm_isaac": "unknown",
            "env": "unknown",
        },
        errors=[],
        warnings=[],
        metadata_sources=["configs/record_quest_right_nocam.json", "docs/quest_isaac_real_control_contract.md"],
        bootstrap=bootstrap,
    )


def write_report(path: Path, report: IsaacReport) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    _ = path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")


def fail_report(path: Path, report: IsaacReport, reason: str, message: str) -> int:
    if reason not in FAIL_REASONS:
        raise ValueError(f"unsupported failure reason: {reason}")
    report["status"] = "fail"
    report["reason"] = reason
    report["errors"].append(message)
    write_report(path, report)
    _ = sys.stderr.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
    return 1


def has_required_discovered_metadata(report: IsaacReport) -> bool:
    return bool(report["joint_names_discovered"] and report["tcp_frame_discovered"])


def import_first(module_names: Iterable[str]) -> tuple[object | None, str | None, str | None]:
    errors: list[str] = []
    for module_name in module_names:
        try:
            return importlib.import_module(module_name), module_name, None
        except ModuleNotFoundError as exc:
            errors.append(f"{module_name}: missing {exc.name or module_name}")
        except Exception as exc:  # Isaac imports may fail on GPU/display/runtime setup.
            errors.append(f"{module_name}: {type(exc).__name__}: {exc}")
    return None, None, "; ".join(errors)


def require_attr(module: object, name: str) -> object | None:
    return cast(object | None, getattr(module, name, None))


def import_isaac_symbols(report: IsaacReport) -> tuple[type[Any] | None, Any | None]:
    app_module, app_module_name, app_error = import_first(
        ("isaaclab.app", "omni.isaac.lab.app")
    )
    if app_module is None:
        report["availability"]["isaac_lab"] = "missing"
        report["availability"]["gym"] = "not_checked"
        report["warnings"].append("Isaac Lab AppLauncher import failed before any app launch.")
        raise RuntimeError(app_error or "Isaac Lab AppLauncher is unavailable")

    app_launcher = require_attr(app_module, "AppLauncher")
    if app_launcher is None:
        report["availability"]["isaac_lab"] = "missing"
        raise RuntimeError(f"{app_module_name} does not expose AppLauncher")

    report["availability"]["isaac_lab"] = f"available:{app_module_name}"
    report["headless_supported"] = True

    gym_module, gym_module_name, gym_error = import_first(("gymnasium", "gym"))
    if gym_module is None:
        report["availability"]["gym"] = "missing"
        raise RuntimeError(gym_error or "gymnasium/gym is unavailable")
    report["availability"]["gym"] = f"available:{gym_module_name}"
    return cast(type[Any], app_launcher), gym_module


def import_openarm_registration(report: IsaacReport) -> None:
    try:
        _ = importlib.import_module("openarm")
    except ModuleNotFoundError as exc:
        report["availability"]["openarm_isaac"] = "missing"
        raise RuntimeError(
            "OpenArm Isaac package import failed; install/activate the local "
            + f"openarm_isaac_lab environment: {exc.name or 'openarm'}"
        ) from exc
    except Exception as exc:
        report["availability"]["openarm_isaac"] = "error"
        raise RuntimeError(f"OpenArm Isaac registration import failed: {exc}") from exc
    report["availability"]["openarm_isaac"] = "available:openarm"


def registry_values(gym_module: Any) -> list[Any]:
    registry = getattr(gym_module, "registry", None)
    if registry is None:
        envs = getattr(gym_module, "envs", None)
        registry = getattr(envs, "registry", None)
    if registry is None:
        return []
    if isinstance(registry, Mapping):
        return list(registry.values())
    values = getattr(registry, "values", None)
    if callable(values):
        registry_values_object = values()
        if isinstance(registry_values_object, Iterable):
            return list(registry_values_object)
    return []


def spec_id(spec: Any) -> str | None:
    env_id = getattr(spec, "id", None)
    if isinstance(env_id, str):
        return env_id
    if isinstance(spec, str):
        return spec
    return None


def enumerate_openarm_envs(gym_module: Any, include_play: bool) -> list[str]:
    env_names: set[str] = set()
    for spec in registry_values(gym_module):
        env_id = spec_id(spec)
        if env_id and "OpenArm" in env_id and (include_play or "Play-v0" not in env_id):
            env_names.add(env_id)
    return sorted(env_names)


def get_env_spec(gym_module: Any, env_name: str) -> Any:
    spec_fn = getattr(gym_module, "spec", None)
    if not callable(spec_fn):
        raise RuntimeError("gym registry does not expose spec()")
    return spec_fn(env_name)


def import_parse_env_cfg() -> Any | None:
    for module_name in (
        "isaaclab_tasks.utils.parse_cfg",
        "omni.isaac.lab_tasks.utils.parse_cfg",
    ):
        try:
            module = importlib.import_module(module_name)
        except ModuleNotFoundError:
            continue
        parse_env_cfg = getattr(module, "parse_env_cfg", None)
        if callable(parse_env_cfg):
            return parse_env_cfg
    return None


def safe_getattr_chain(root: object, chain: tuple[str, ...]) -> object | None:
    current: object | None = root
    for name in chain:
        if current is None:
            return None
        current = getattr(current, name, None)
    return current


def string_list(value: object | None) -> list[str]:
    if value is None or isinstance(value, (str, bytes)):
        return []
    if isinstance(value, Iterable):
        return [str(item) for item in value]
    return []


def first_number(*values: object | None) -> float | None:
    for value in values:
        if isinstance(value, bool):
            continue
        if isinstance(value, int | float):
            return float(value)
    return None


def find_asset_path(value: object, depth: int = 0) -> str | None:
    if depth > 6:
        return None
    if isinstance(value, str) and (".usd" in value or ".urdf" in value):
        return value
    if isinstance(value, Mapping):
        preferred_keys = ("usd_path", "asset_path", "prim_path", "file", "filename")
        for key in preferred_keys:
            if key in value:
                found = find_asset_path(value[key], depth + 1)
                if found:
                    return found
        for item in value.values():
            found = find_asset_path(item, depth + 1)
            if found:
                return found
    if not isinstance(value, (str, bytes)) and isinstance(value, Iterable):
        for item in value:
            found = find_asset_path(item, depth + 1)
            if found:
                return found
    if hasattr(value, "__dict__"):
        return find_asset_path(vars(value), depth + 1)
    return None


def extract_cfg_metadata(cfg: object, report: IsaacReport) -> None:
    asset_path = find_asset_path(cfg)
    if asset_path:
        report["asset_path"] = asset_path
        report["metadata_sources"].append("isaac_env_cfg.asset_path")

    joint_names = string_list(
        safe_getattr_chain(cfg, ("scene", "robot", "init_state", "joint_pos"))
    )
    if joint_names:
        report["joint_names"] = joint_names
        report["joint_names_discovered"] = True
        report["metadata_sources"].append("isaac_env_cfg.scene.robot.init_state.joint_pos")

    body_names = string_list(safe_getattr_chain(cfg, ("scene", "robot", "body_names")))
    tcp_candidates = [name for name in body_names if "tcp" in name.lower() or "hand" in name.lower()]
    if tcp_candidates:
        report["tcp_frame"] = tcp_candidates[0]
        report["tcp_frame_discovered"] = True
        report["metadata_sources"].append("isaac_env_cfg.scene.robot.body_names")

    decimation = first_number(getattr(cfg, "decimation", None))
    sim_dt = first_number(safe_getattr_chain(cfg, ("sim", "dt")))
    if decimation and sim_dt and sim_dt > 0:
        report["control_frequency_hz"] = 1.0 / (sim_dt * decimation)
        report["metadata_sources"].append("isaac_env_cfg.sim.dt_decimation")


def extract_runtime_metadata(env: object, report: IsaacReport) -> None:
    scene = getattr(env, "scene", None)
    robot = None
    if scene is not None:
        try:
            robot = scene["robot"]
        except Exception:
            robot = getattr(scene, "robot", None)

    if robot is not None:
        joint_names = string_list(getattr(robot, "joint_names", None))
        if joint_names:
            report["joint_names"] = joint_names
            report["joint_names_discovered"] = True
            report["metadata_sources"].append("headless_env.scene.robot.joint_names")
        body_names = string_list(getattr(robot, "body_names", None))
        tcp_candidates = [name for name in body_names if "tcp" in name.lower() or "hand" in name.lower()]
        if tcp_candidates:
            report["tcp_frame"] = tcp_candidates[0]
            report["tcp_frame_discovered"] = True
            report["metadata_sources"].append("headless_env.scene.robot.body_names")

    unwrapped = getattr(env, "unwrapped", env)
    cfg = getattr(unwrapped, "cfg", None)
    if cfg is not None:
        extract_cfg_metadata(cfg, report)


def maybe_instantiate_headless(
    args: ParsedArgs,
    app_launcher: type[Any] | None,
    gym_module: Any,
    report: IsaacReport,
) -> None:
    if not args.instantiate_headless:
        report["warnings"].append(
            "Headless instantiation was not requested; static registry/config inspection only."
        )
        return
    if app_launcher is None:
        report["warnings"].append("Headless instantiation skipped because AppLauncher is unavailable.")
        return

    app_launcher_instance = None
    simulation_app = None
    env = None
    try:
        app_launcher_instance = app_launcher(headless=True)
        simulation_app = getattr(app_launcher_instance, "app", None)
        make_fn = getattr(gym_module, "make", None)
        if not callable(make_fn):
            raise RuntimeError("gym module does not expose make()")
        try:
            env = make_fn(args.env_name, render_mode=None)
        except TypeError:
            env = make_fn(args.env_name)
        report["headless_instantiated"] = True
        extract_runtime_metadata(env, report)
    except Exception as exc:
        report["warnings"].append(f"Headless instantiation failed safely: {type(exc).__name__}: {exc}")
    finally:
        if env is not None:
            close = getattr(env, "close", None)
            if callable(close):
                try:
                    _ = close()
                except Exception as exc:
                    report["warnings"].append(f"Headless env close failed: {exc}")
        if simulation_app is not None:
            close = getattr(simulation_app, "close", None)
            if callable(close):
                try:
                    _ = close()
                except Exception as exc:
                    report["warnings"].append(f"Isaac app close failed: {exc}")


def main() -> int:
    args = parse_args()
    bootstrap = bootstrap_paths()
    report = base_report(args, bootstrap)

    try:
        app_launcher_cls, gym_module = import_isaac_symbols(report)
    except RuntimeError as exc:
        return fail_report(args.report, report, "isaac_unavailable", str(exc))
    if app_launcher_cls is None:
        return fail_report(args.report, report, "isaac_unavailable", "Isaac Lab AppLauncher is unavailable")

    app_launcher_instance = None
    simulation_app = None
    try:
        try:
            app_launcher_instance = app_launcher_cls({"headless": True})
        except TypeError as exc:
            report["warnings"].append(
                f"AppLauncher dict initialization failed; retried with headless keyword: {exc}"
            )
            app_launcher_instance = app_launcher_cls(headless=True)
        simulation_app = getattr(app_launcher_instance, "app", None)
        report["headless_supported"] = True
        report["headless_instantiated"] = True
    except Exception as exc:
        return fail_report(
            args.report,
            report,
            "isaac_unavailable",
            f"Isaac AppLauncher headless launch failed: {type(exc).__name__}: {exc}",
        )

    try:
        try:
            import_openarm_registration(report)
        except RuntimeError as exc:
            return fail_report(args.report, report, "openarm_isaac_unavailable", str(exc))

        discovered_envs = enumerate_openarm_envs(gym_module, args.include_play)
        report["matching_env_names"] = discovered_envs

        if args.env_name not in discovered_envs:
            if discovered_envs:
                report["env_name"] = discovered_envs[0]
                report["warnings"].append(
                    f"Requested env {args.env_name!r} was not registered; reporting first discovered OpenArm env {discovered_envs[0]!r}."
                )
            report["availability"]["env"] = "missing"
            return fail_report(
                args.report,
                report,
                "env_unavailable",
                f"Requested OpenArm Isaac env {args.env_name!r} is not registered.",
            )

        try:
            _ = get_env_spec(gym_module, args.env_name)
        except Exception as exc:
            report["availability"]["env"] = "spec_error"
            return fail_report(
                args.report,
                report,
                "env_unavailable",
                f"gym.spec({args.env_name!r}) failed: {type(exc).__name__}: {exc}",
            )
        report["availability"]["env"] = "available"

        parse_env_cfg = import_parse_env_cfg()
        if parse_env_cfg is None:
            report["warnings"].append("Isaac Lab parse_env_cfg was unavailable; kept fallback/static metadata.")
        else:
            try:
                try:
                    cfg = parse_env_cfg(args.env_name, device="cpu", num_envs=1, use_fabric=False)
                except TypeError:
                    cfg = parse_env_cfg(args.env_name)
                extract_cfg_metadata(cfg, report)
            except Exception as exc:
                report["warnings"].append(f"parse_env_cfg failed safely: {type(exc).__name__}: {exc}")

        if args.instantiate_headless:
            env = None
            try:
                make_fn = getattr(gym_module, "make", None)
                if not callable(make_fn):
                    raise RuntimeError("gym module does not expose make()")
                try:
                    env = make_fn(args.env_name, render_mode=None)
                except TypeError:
                    env = make_fn(args.env_name)
                extract_runtime_metadata(env, report)
            except Exception as exc:
                report["warnings"].append(f"Headless env instantiation failed safely: {type(exc).__name__}: {exc}")
            finally:
                if env is not None:
                    close = getattr(env, "close", None)
                    if callable(close):
                        try:
                            _ = close()
                        except Exception as exc:
                            report["warnings"].append(f"Headless env close failed: {exc}")
        else:
            report["warnings"].append(
                "Headless instantiation was not requested; static registry/config inspection only."
            )

        if not report["joint_names_discovered"]:
            return fail_report(
                args.report,
                report,
                "env_unavailable",
                "OpenArm Isaac env is registered, but Isaac-derived joint names could not be discovered; fallback contract joint names are context only.",
            )
        if not report["tcp_frame_discovered"]:
            return fail_report(
                args.report,
                report,
                "env_unavailable",
                "OpenArm Isaac env is registered, but Isaac-derived TCP frame could not be discovered; fallback contract TCP frame is context only.",
            )
        if not has_required_discovered_metadata(report):
            return fail_report(
                args.report,
                report,
                "env_unavailable",
                "OpenArm Isaac env is registered, but required discovered metadata is incomplete.",
            )

        report["status"] = "pass"
        report["reason"] = "pass"
        write_report(args.report, report)
        _ = sys.stdout.write(json.dumps(report, indent=2, sort_keys=True) + "\n")
        return 0
    finally:
        if simulation_app is not None:
            close = getattr(simulation_app, "close", None)
            if callable(close):
                try:
                    _ = close()
                except Exception as exc:
                    report["warnings"].append(f"Isaac app close failed: {exc}")


if __name__ == "__main__":
    raise SystemExit(main())
