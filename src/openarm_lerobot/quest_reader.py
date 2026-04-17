"""Thin Quest VR lifecycle wrapper around DROID's OculusReader.

This MVP stays USB-only and avoids hardware side effects until connect().
"""

from __future__ import annotations

from importlib import import_module
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

    def get_transforms_and_buttons(self):
        if not self._connected or self._reader is None:
            raise RuntimeError("QuestReader is not connected.")

        return self._reader.get_transformations_and_buttons()
