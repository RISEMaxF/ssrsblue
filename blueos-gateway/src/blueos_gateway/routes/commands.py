from fastapi import APIRouter, HTTPException, Request

from blueos_gateway.models import (
    CommandResponse,
    GuidedHeadingRequest,
    GuidedPositionRequest,
    GuidedVelocityRequest,
    ManualControlRequest,
    MotorModeRequest,
    SetModeRequest,
)
from blueos_gateway.validators import validate_mode
from blueos_gateway.vehicle_state import ROVER_MODES

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



MOTOR_MODE_MAP = {
    "blue_robotics": 0,
    "all": 1,
    "flipsky": 2,
}


@router.post("/motor_mode", response_model=CommandResponse)
async def motor_mode(
    req: MotorModeRequest, request: Request
) -> CommandResponse:
    _require_connected(request)
    mode_str = req.mode.lower()
    if mode_str not in MOTOR_MODE_MAP:
        raise HTTPException(
            400, f"Invalid mode '{req.mode}'. Use: {list(MOTOR_MODE_MAP.keys())}"
        )
    client = request.app.state.mavlink_client
    value = float(MOTOR_MODE_MAP[mode_str])
    ok = await client.send_param_set("SCR_USER1", value)
    return CommandResponse(
        success=ok,
        message=f"Motor mode set to {mode_str}" if ok else "Send failed",
    )


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
    result = await client.send_set_mode(mode_num)
    name = ROVER_MODES.get(mode_num, str(mode_num))
    if not result.sent:
        return CommandResponse(success=False, message="Send failed")
    return CommandResponse(
        success=result.accepted,
        message=f"Requested {name}" if result.accepted else f"{name} rejected: {result.ack_result}",
        ack_result=result.ack_result,
    )


@router.post("/arm", response_model=CommandResponse)
async def arm(request: Request) -> CommandResponse:
    _require_connected(request)
    client = request.app.state.mavlink_client
    result = await client.send_arm(True)
    if not result.sent:
        return CommandResponse(success=False, message="Send failed")
    return CommandResponse(
        success=result.accepted,
        message="Armed" if result.accepted else f"Arm rejected: {result.ack_result}",
        ack_result=result.ack_result,
    )


@router.post("/disarm", response_model=CommandResponse)
async def disarm(request: Request) -> CommandResponse:
    _require_connected(request)
    client = request.app.state.mavlink_client
    result = await client.send_arm(False)
    if not result.sent:
        return CommandResponse(success=False, message="Send failed")
    return CommandResponse(
        success=result.accepted,
        message="Disarmed" if result.accepted else f"Disarm rejected: {result.ack_result}",
        ack_result=result.ack_result,
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
