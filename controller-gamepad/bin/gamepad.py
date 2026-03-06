#!/usr/bin/env python3

import struct
import logging
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import usb.core
import usb.util

logger = logging.getLogger("gamepad")

NEUTRAL_BUTTONS = {
    "A": False, "B": False, "X": False, "Y": False,
    "LB": False, "RB": False, "BACK": False, "START": False,
    "L3": False, "R3": False,
    "UP": False, "DOWN": False, "LEFT": False, "RIGHT": False,
}


@dataclass
class GamepadState:
    left_x: float = 0.0
    left_y: float = 0.0
    right_x: float = 0.0
    right_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    buttons: dict[str, bool] = field(default_factory=lambda: dict(NEUTRAL_BUTTONS))


class Gamepad(ABC):
    @abstractmethod
    def open(self):
        ...

    @abstractmethod
    def close(self):
        ...

    @abstractmethod
    def read(self) -> GamepadState:
        ...

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()


class LogitechF310(Gamepad):
    VENDOR_ID = 0x046D
    PRODUCT_ID = 0xC21D
    EP_IN = 0x81

    def __init__(self, deadzone: float = 0.05):
        self.deadzone = deadzone
        self._dev = None
        self._last_state = GamepadState()

    def open(self):
        self._dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)
        if self._dev is None:
            raise RuntimeError("Logitech F310 not found (ensure XInput mode and USB connected)")
        try:
            self._dev.set_configuration()
        except usb.core.USBError:
            pass
        try:
            usb.util.claim_interface(self._dev, 0)
        except usb.core.USBError as exc:
            self._dev = None
            raise RuntimeError(f"Cannot claim USB interface (device busy?): {exc}") from exc
        logger.info("Logitech F310 opened")

    def close(self):
        if self._dev is not None:
            try:
                usb.util.release_interface(self._dev, 0)
            except usb.core.USBError:
                pass
            self._dev = None
        logger.info("Logitech F310 closed")

    def read(self) -> GamepadState:
        try:
            data = self._dev.read(self.EP_IN, 32, timeout=50)
        except usb.core.USBTimeoutError:
            return self._last_state
        except usb.core.USBError as exc:
            logger.error("USB read error: %s", exc)
            raise

        if len(data) < 14:
            logger.debug("Partial USB read (%d bytes), using last state", len(data))
            return self._last_state

        btn_lo = data[2]
        btn_hi = data[3]
        lt = data[4] / 255.0
        rt = data[5] / 255.0
        lx, ly, rx, ry = struct.unpack_from("<hhhh", data, 6)

        buttons = {
            "UP": bool(btn_lo & 0x01),
            "DOWN": bool(btn_lo & 0x02),
            "LEFT": bool(btn_lo & 0x04),
            "RIGHT": bool(btn_lo & 0x08),
            "BACK": bool(btn_lo & 0x10),  # renamed from Guide in some mappings
            "START": bool(btn_lo & 0x20),
            "L3": bool(btn_lo & 0x40),
            "R3": bool(btn_lo & 0x80),
            "LB": bool(btn_hi & 0x01),
            "RB": bool(btn_hi & 0x02),
            "A": bool(btn_hi & 0x10),
            "B": bool(btn_hi & 0x20),
            "X": bool(btn_hi & 0x40),
            "Y": bool(btn_hi & 0x80),
        }

        state = GamepadState(
            left_x=self._apply_deadzone(max(-1.0, lx / 32767.0)),
            left_y=self._apply_deadzone(max(-1.0, ly / 32767.0)),
            right_x=self._apply_deadzone(max(-1.0, rx / 32767.0)),
            right_y=self._apply_deadzone(max(-1.0, ry / 32767.0)),
            left_trigger=lt,
            right_trigger=rt,
            buttons=buttons,
        )
        self._last_state = state
        return state

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value


def create_gamepad(kind: str = "f310", **kwargs) -> Gamepad:
    if kind == "f310":
        return LogitechF310(**kwargs)
    raise ValueError(f"Unknown gamepad: {kind}")
