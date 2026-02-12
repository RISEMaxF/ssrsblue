from connector.vehicle_state import ROVER_MODES, ROVER_MODE_NAMES


def clamp(value: float, lo: float, hi: float) -> int:
    return int(max(lo, min(hi, value)))


def clamp_steering(value: float) -> int:
    return clamp(value, -1000, 1000)


def clamp_throttle(value: float) -> int:
    return clamp(value, -1000, 1000)


def validate_mode(mode: str | int) -> int:
    """Convert mode name or number to a validated ArduRover mode number."""
    if isinstance(mode, str):
        upper = mode.upper()
        if upper not in ROVER_MODE_NAMES:
            raise ValueError(f"Unknown mode: {mode}. Valid: {list(ROVER_MODE_NAMES)}")
        return ROVER_MODE_NAMES[upper]
    if mode not in ROVER_MODES:
        raise ValueError(f"Unknown mode number: {mode}. Valid: {list(ROVER_MODES)}")
    return mode


def is_controllable_mode(mode: int) -> bool:
    """True if mode allows companion commands (GUIDED or STEERING)."""
    return mode in (15, 3)
