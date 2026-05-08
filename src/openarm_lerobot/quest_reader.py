"""Thin Quest VR lifecycle wrapper around DROID's OculusReader.

This MVP stays USB-only and avoids hardware side effects until connect().
"""

from __future__ import annotations

from importlib import import_module
from time import monotonic
from typing import Any


OculusReader: type[Any] | None = None


def _load_oculus_reader_class() -> type[Any]:
    global OculusReader

    if OculusReader is not None:
        return OculusReader

    for module_name in (
        "oculus_reader",
        "droid.oculus_reader.oculus_reader",
    ):
        try:
            module = import_module(module_name)
        except ModuleNotFoundError:
            continue

        reader_cls = getattr(module, "OculusReader", None)
        if reader_cls is not None:
            OculusReader = reader_cls
            return reader_cls

    raise ModuleNotFoundError(
        "Could not resolve OculusReader from 'oculus_reader' or the local DROID checkout."
    )


class QuestReader:
    """USB-only lifecycle wrapper for the Quest teleop reader."""

    def __init__(self, ip_address: str | None = None):
        self.ip_address = ip_address
        self._reader: Any = None
        self._connected = False
        self._read_count = 0
        self._last_read_started_t: float | None = None
        self._last_read_finished_t: float | None = None
        self._last_payload_ok = False
        self._last_none_reason = "never_read"

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return

        if self.ip_address is not None:
            raise ValueError(
                "QuestReader MVP only supports USB connections; ip_address must be None."
            )

        reader_cls = _load_oculus_reader_class()
        self._reader = reader_cls(ip_address=None)
        self._connected = True

    def disconnect(self) -> None:
        reader = self._reader
        self._reader = None
        self._connected = False

        if reader is None:
            return

        stop = getattr(reader, "stop", None)
        if callable(stop):
            _ = stop()

    @property
    def diagnostics(self) -> dict[str, object]:
        return {
            "read_count": self._read_count,
            "last_read_started_t": self._last_read_started_t,
            "last_read_finished_t": self._last_read_finished_t,
            "last_payload_ok": self._last_payload_ok,
            "last_none_reason": self._last_none_reason,
        }

    def get_transforms_and_buttons(self):
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestReader is not connected.")

        self._read_count += 1
        self._last_read_started_t = monotonic()
        payload = self._reader.get_transformations_and_buttons()
        self._last_read_finished_t = monotonic()

        self._last_payload_ok = (
            isinstance(payload, tuple)
            and len(payload) == 2
            and isinstance(payload[0], dict)
            and isinstance(payload[1], dict)
        )
        self._last_none_reason = "ok" if self._last_payload_ok else "bad_payload"
        return payload
