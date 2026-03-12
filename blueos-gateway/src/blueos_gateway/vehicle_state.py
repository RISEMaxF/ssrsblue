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
    # NOTE: mode defaults to 0 which maps to MANUAL in ROVER_MODES, but
    # mode_name starts as "UNKNOWN". Before the first heartbeat, consumers
    # should check mavlink_connected rather than trusting mode/mode_name.
    mode: int = 0
    mode_name: str = "UNKNOWN"
    armed: bool | None = None  # None = unknown (no heartbeat received yet)

    # From GPS_RAW_INT
    # None = no GPS_RAW_INT received yet or no fix (fix_type < 2).
    # ArduPilot reports 0,0 when there's no fix, which is a real coordinate
    # (Gulf of Guinea), so we use None to avoid misleading consumers.
    gps_fix_type: int = 0
    lat: float | None = None
    lon: float | None = None
    satellites_visible: int = 0

    # From SYS_STATUS
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: int = -1

    # From VFR_HUD
    heading: int = 0
    groundspeed: float = 0.0
    throttle: int = 0

    # Last manual control command (None = no command sent yet)
    last_command_steering: float | None = None
    last_command_throttle: float | None = None
    last_command_received: float = 0.0

    # Motor mode (from SCR_USER1 PARAM_VALUE)
    motor_mode: str | None = None  # None = unknown, "blue_robotics" / "all" / "flipsky"

    # Timestamps (monotonic)
    last_heartbeat_received: float = field(default_factory=time.monotonic)

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
        self.satellites_visible = msg.get("satellites_visible", 0)

        # Only trust lat/lon when ArduPilot has at least a 2D fix.
        # With no fix, ArduPilot reports 0,0 which is a real place.
        if self.gps_fix_type >= 2:
            self.lat = msg.get("lat", 0) / 1e7
            self.lon = msg.get("lon", 0) / 1e7
        else:
            self.lat = None
            self.lon = None

    def update_from_sys_status(self, msg: dict) -> None:
        self.battery_voltage = msg.get("voltage_battery", 0) / 1000.0
        self.battery_current = msg.get("current_battery", 0) / 100.0
        self.battery_remaining = msg.get("battery_remaining", -1)

    def update_from_vfr_hud(self, msg: dict) -> None:
        self.heading = msg.get("heading", 0)
        self.groundspeed = msg.get("groundspeed", 0.0)
        self.throttle = msg.get("throttle", 0)

    def update_from_param_value(self, msg: dict) -> None:
        param_id = msg.get("param_id", "")
        if param_id == "SCR_USER1":
            value = round(msg.get("param_value", 0))
            self.motor_mode = _MOTOR_MODE_NAMES.get(value)


_MOTOR_MODE_NAMES: dict[int, str] = {0: "blue_robotics", 1: "all", 2: "flipsky"}
