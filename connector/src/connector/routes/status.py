import time

from fastapi import APIRouter, Request

from connector.models import HealthResponse, VehicleStatusResponse
from connector.validators import is_controllable_mode

router = APIRouter(tags=["status"])


@router.get("/status", response_model=VehicleStatusResponse)
async def get_status(request: Request) -> VehicleStatusResponse:
    s = request.app.state.vehicle_state
    return VehicleStatusResponse(
        mode=s.mode,
        mode_name=s.mode_name,
        armed=s.armed,
        gps_fix_type=s.gps_fix_type,
        lat=s.lat,
        lon=s.lon,
        satellites_visible=s.satellites_visible,
        battery_voltage=round(s.battery_voltage, 2),
        battery_current=round(s.battery_current, 2),
        battery_remaining=s.battery_remaining,
        heading=s.heading,
        groundspeed=round(s.groundspeed, 2),
        throttle=s.throttle,
    )


@router.get("/health", response_model=HealthResponse)
async def get_health(request: Request) -> HealthResponse:
    s = request.app.state.vehicle_state
    start = request.app.state.start_time
    age = time.monotonic() - s.last_heartbeat_received

    if not s.mavlink_connected:
        status = "disconnected"
    elif age > 5.0:
        status = "degraded"
    else:
        status = "ok"

    watchdog_active = (
        s.armed
        and is_controllable_mode(s.mode)
        and s.last_command_received > 0
    )

    return HealthResponse(
        status=status,
        mavlink_connected=s.mavlink_connected,
        last_heartbeat_age_s=round(age, 1),
        uptime_s=round(time.monotonic() - start, 1),
        watchdog_active=watchdog_active,
    )
