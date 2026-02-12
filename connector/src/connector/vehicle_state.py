import time
from dataclasses import dataclass, field

ROVER_MODES: dict[int, str] = {
    0: "MANUAL",
    1: "ACRO",
    3: "STEERING",
    4: "HOLD",
    5: "LOITER",
    6: "FOLLOW",
    7: "SIMPLE",
    10: "AUTO",
    11: "RTL",
    12: "SMART_RTL",
    15: "GUIDED",
}

ROVER_MODE_NAMES: dict[str, int] = {v: k for k, v in ROVER_MODES.items()}


@dataclass
class VehicleState:
    # From HEARTBEAT
    mode: int = 0
    mode_name: str = "UNKNOWN"
    armed: bool = False

    # From GPS_RAW_INT
    gps_fix_type: int = 0
    lat: float = 0.0
    lon: float = 0.0
    satellites_visible: int = 0

    # From SYS_STATUS
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: int = -1

    # From VFR_HUD
    heading: int = 0
    groundspeed: float = 0.0
    throttle: int = 0

    # Timestamps (monotonic)
    last_heartbeat_received: float = field(default_factory=time.monotonic)
    last_command_received: float = 0.0

    # Connection
    mavlink_connected: bool = False

    def update_from_heartbeat(self, msg: dict) -> None:
        self.mode = msg["custom_mode"]
        self.mode_name = ROVER_MODES.get(self.mode, "UNKNOWN")
        base_bits = msg["base_mode"]
        if isinstance(base_bits, dict):
            base_bits = base_bits.get("bits", 0)
        self.armed = bool(base_bits & 128)
        self.last_heartbeat_received = time.monotonic()

    def update_from_gps(self, msg: dict) -> None:
        fix = msg.get("fix_type", 0)
        if isinstance(fix, dict):
            fix = fix.get("type", 0)
        self.gps_fix_type = fix if isinstance(fix, int) else 0
        self.lat = msg.get("lat", 0) / 1e7
        self.lon = msg.get("lon", 0) / 1e7
        self.satellites_visible = msg.get("satellites_visible", 0)

    def update_from_sys_status(self, msg: dict) -> None:
        self.battery_voltage = msg.get("voltage_battery", 0) / 1000.0
        self.battery_current = msg.get("current_battery", 0) / 100.0
        self.battery_remaining = msg.get("battery_remaining", -1)

    def update_from_vfr_hud(self, msg: dict) -> None:
        self.heading = msg.get("heading", 0)
        self.groundspeed = msg.get("groundspeed", 0.0)
        self.throttle = msg.get("throttle", 0)
