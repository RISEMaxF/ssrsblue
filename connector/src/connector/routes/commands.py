from fastapi import APIRouter, HTTPException, Request

from connector.models import (
    CommandResponse,
    GuidedHeadingRequest,
    GuidedPositionRequest,
    GuidedVelocityRequest,
    ManualControlRequest,
    SetModeRequest,
)
from connector.validators import validate_mode
from connector.vehicle_state import ROVER_MODES

router = APIRouter(prefix="/command", tags=["commands"])


def _require_connected(request: Request) -> None:
    if not request.app.state.vehicle_state.mavlink_connected:
        raise HTTPException(503, "Not connected to BlueOS")


def _require_guided(request: Request) -> None:
    _require_connected(request)
    s = request.app.state.vehicle_state
    if s.mode != 15:
        raise HTTPException(
            400, f"Vehicle not in GUIDED mode (current: {s.mode_name})"
        )
    if not s.armed:
        raise HTTPException(400, "Vehicle not armed")


@router.post("/manual_control", response_model=CommandResponse)
async def manual_control(
    req: ManualControlRequest, request: Request
) -> CommandResponse:
    _require_connected(request)
    client = request.app.state.mavlink_client
    ok = await client.send_manual_control(req.steering, req.throttle)
    return CommandResponse(success=ok, message="OK" if ok else "Send failed")


@router.post("/set_mode", response_model=CommandResponse)
async def set_mode(req: SetModeRequest, request: Request) -> CommandResponse:
    _require_connected(request)
    try:
        mode_num = validate_mode(req.mode)
    except ValueError as exc:
        raise HTTPException(400, str(exc))
    client = request.app.state.mavlink_client
    ok = await client.send_set_mode(mode_num)
    name = ROVER_MODES.get(mode_num, str(mode_num))
    return CommandResponse(
        success=ok, message=f"Requested {name}" if ok else "Send failed"
    )


@router.post("/arm", response_model=CommandResponse)
async def arm(request: Request) -> CommandResponse:
    _require_connected(request)
    client = request.app.state.mavlink_client
    ok = await client.send_arm(True)
    return CommandResponse(success=ok, message="Arm sent" if ok else "Send failed")


@router.post("/disarm", response_model=CommandResponse)
async def disarm(request: Request) -> CommandResponse:
    _require_connected(request)
    client = request.app.state.mavlink_client
    ok = await client.send_arm(False)
    return CommandResponse(
        success=ok, message="Disarm sent" if ok else "Send failed"
    )


@router.post("/guided/position", response_model=CommandResponse)
async def guided_position(
    req: GuidedPositionRequest, request: Request
) -> CommandResponse:
    _require_guided(request)
    client = request.app.state.mavlink_client
    ok = await client.send_guided_position(req.lat, req.lon)
    return CommandResponse(
        success=ok,
        message=f"Go to {req.lat:.6f}, {req.lon:.6f}" if ok else "Send failed",
    )


@router.post("/guided/velocity", response_model=CommandResponse)
async def guided_velocity(
    req: GuidedVelocityRequest, request: Request
) -> CommandResponse:
    _require_guided(request)
    client = request.app.state.mavlink_client
    ok = await client.send_guided_velocity(req.vx, req.vy, req.yaw)
    return CommandResponse(success=ok, message="OK" if ok else "Send failed")


@router.post("/guided/heading", response_model=CommandResponse)
async def guided_heading(
    req: GuidedHeadingRequest, request: Request
) -> CommandResponse:
    _require_guided(request)
    client = request.app.state.mavlink_client
    ok = await client.send_guided_heading(req.heading, req.speed)
    return CommandResponse(
        success=ok,
        message=f"Heading {req.heading:.0f}° @ {req.speed:.1f} m/s"
        if ok
        else "Send failed",
    )
