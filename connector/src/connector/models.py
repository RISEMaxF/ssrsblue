from pydantic import BaseModel, Field


# ── Requests ───────────────────────────────────────────────


class ManualControlRequest(BaseModel):
    steering: float = Field(
        0, ge=-1000, le=1000,
        description="Steering: -1000=full left, 0=straight, 1000=full right",
    )
    throttle: float = Field(
        0, ge=-1000, le=1000,
        description="Throttle: -1000=full reverse, 0=stop, 1000=full forward",
    )


class SetModeRequest(BaseModel):
    mode: str | int = Field(..., description="Mode name (MANUAL, GUIDED, …) or number")


class GuidedPositionRequest(BaseModel):
    lat: float = Field(..., ge=-90, le=90)
    lon: float = Field(..., ge=-180, le=180)


class GuidedVelocityRequest(BaseModel):
    vx: float = Field(0, ge=-10, le=10, description="Forward velocity m/s (body frame)")
    vy: float = Field(0, ge=-10, le=10, description="Lateral velocity m/s")
    yaw: float | None = Field(None, ge=0, lt=360, description="Target yaw degrees")


class GuidedHeadingRequest(BaseModel):
    heading: float = Field(..., ge=0, lt=360, description="Target heading degrees")
    speed: float = Field(0, ge=-5, le=5, description="Target speed m/s (negative=reverse)")


# ── Responses ──────────────────────────────────────────────


class CommandResponse(BaseModel):
    success: bool
    message: str


class VehicleStatusResponse(BaseModel):
    mode: int
    mode_name: str
    armed: bool
    gps_fix_type: int
    lat: float
    lon: float
    satellites_visible: int
    battery_voltage: float
    battery_current: float
    battery_remaining: int
    heading: int
    groundspeed: float
    throttle: int


class HealthResponse(BaseModel):
    status: str
    mavlink_connected: bool
    last_heartbeat_age_s: float
    uptime_s: float
    watchdog_active: bool
