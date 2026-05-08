from __future__ import annotations

import os
import subprocess
from typing import Literal

NotifyKind = Literal["info", "ready", "warn", "error", "go"]

_SOUND_MAP = {
    "info": "/usr/share/sounds/freedesktop/stereo/message.oga",
    "ready": "/usr/share/sounds/freedesktop/stereo/complete.oga",
    "go": "/usr/share/sounds/freedesktop/stereo/bell.oga",
    "warn": "/usr/share/sounds/freedesktop/stereo/dialog-warning.oga",
    "error": "/usr/share/sounds/freedesktop/stereo/suspend-error.oga",
}


def _popen_silent(argv: list[str]) -> None:
    try:
        subprocess.Popen(
            argv,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        return


def notify(message: str, kind: NotifyKind = "info", *, urgent: bool = False) -> None:
    """Non-blocking operator notification: sound + desktop popup + stdout."""

    sound = _SOUND_MAP.get(kind, _SOUND_MAP["info"])
    if os.path.exists(sound):
        _popen_silent(["paplay", sound])

    urgency = "critical" if urgent else "normal"
    _popen_silent(
        [
            "notify-send",
            "-u",
            urgency,
            "-a",
            "OpenArm",
            "OpenArm Teleop",
            message,
        ]
    )
    print(f"\n>>> [{kind.upper()}] {message} <<<\n", flush=True)


def confirm(message: str, *, kind: NotifyKind = "ready") -> bool:
    """Blocking operator confirmation: sound + zenity dialog. Returns True on OK."""

    sound = _SOUND_MAP.get(kind, _SOUND_MAP["ready"])
    if os.path.exists(sound):
        _popen_silent(["paplay", sound])

    print(f"\n>>> [CONFIRM] {message} (zenity dialog open) <<<\n", flush=True)
    try:
        result = subprocess.run(
            [
                "zenity",
                "--question",
                "--title=OpenArm Teleop",
                f"--text={message}",
                "--width=420",
            ],
            check=False,
            timeout=300,
        )
        return result.returncode == 0
    except FileNotFoundError:
        ans = input(f"{message} [y/N] ").strip().lower()
        return ans in {"y", "yes"}
    except subprocess.TimeoutExpired:
        return False
