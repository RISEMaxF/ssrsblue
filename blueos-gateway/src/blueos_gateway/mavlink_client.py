import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass

import httpx
import websockets
from websockets.exceptions import ConnectionClosed

from blueos_gateway.config import Settings
from blueos_gateway.validators import clamp_steering, clamp_throttle
from blueos_gateway.vehicle_state import VehicleState

logger = logging.getLogger(__name__)


@dataclass
class CommandResult:
    sent: bool
    ack_result: str  # "MAV_RESULT_ACCEPTED", "MAV_RESULT_FAILED", "TIMEOUT", etc.
    accepted: bool   # convenience: ack_result == "MAV_RESULT_ACCEPTED"


class MAVLinkClient:
    """Manages MAVLink communication with BlueOS via MAVLink2REST.

    - Receives telemetry over a filtered WebSocket (persistent connection).
    - Sends commands and heartbeats via HTTP POST (simple, reliable).
    """

    def __init__(self, config: Settings, state: VehicleState) -> None:
        self.config = config
        self.state = state
        self._http: httpx.AsyncClient | None = None
        self._tasks: list[asyncio.Task] = []
        self._ack_queues: dict[str, asyncio.Queue] = {}

    # ── Lifecycle ──────────────────────────────────────────

    async def start(self) -> None:
        self._http = httpx.AsyncClient(timeout=2.0)
        self._tasks = [
            asyncio.create_task(self._telemetry_loop(), name="telemetry"),
            asyncio.create_task(self._heartbeat_loop(), name="heartbeat"),
        ]
        logger.info("MAVLinkClient started (target: %s)", self.config.blueos_host)

    async def stop(self) -> None:
        for t in self._tasks:
            t.cancel()
        await asyncio.gather(*self._tasks, return_exceptions=True)
        if self._http:
            await self._http.aclose()
        logger.info("MAVLinkClient stopped")

    # ── Telemetry (WebSocket) ──────────────────────────────

    async def _telemetry_loop(self) -> None:
        msg_filter = "HEARTBEAT|GPS_RAW_INT|SYS_STATUS|VFR_HUD|COMMAND_ACK|PARAM_VALUE"
        url = f"{self.config.mavlink_ws_url}?filter={msg_filter}"
        backoff = 1.0

        while True:
            try:
                async with websockets.connect(
                    url, ping_interval=5, ping_timeout=10
                ) as ws:
                    self.state.mavlink_connected = True
                    backoff = 1.0
                    logger.info("WebSocket connected to %s", url)

                    # Request current SCR_USER1 value for motor mode readback
                    await self._request_param("SCR_USER1")

                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                            self._process_message(msg)
                        except (json.JSONDecodeError, KeyError, TypeError) as exc:
                            logger.debug("Skipping malformed message: %s", exc)

            except (ConnectionClosed, ConnectionRefusedError, OSError) as exc:
                self.state.mavlink_connected = False
                logger.warning(
                    "WebSocket disconnected: %s. Reconnecting in %.0fs…", exc, backoff
                )
            except asyncio.CancelledError:
                return

            await asyncio.sleep(backoff)
            backoff = min(backoff * 2, 10.0)

    def _process_message(self, msg: dict) -> None:
        header = msg.get("header", {})
        # Ignore our own messages
        if header.get("system_id") == self.config.system_id:
            return
        # Only process messages from the target vehicle
        if header.get("system_id") != self.config.target_system:
            return

        body = msg.get("message", {})
        mtype = body.get("type", "")

        if mtype == "HEARTBEAT":
            self.state.update_from_heartbeat(body)
        elif mtype == "GPS_RAW_INT":
            self.state.update_from_gps(body)
        elif mtype == "SYS_STATUS":
            self.state.update_from_sys_status(body)
        elif mtype == "VFR_HUD":
            self.state.update_from_vfr_hud(body)
        elif mtype == "COMMAND_ACK":
            self._handle_command_ack(body)
        elif mtype == "PARAM_VALUE":
            self.state.update_from_param_value(body)

    def _handle_command_ack(self, body: dict) -> None:
        cmd_name = body.get("command", {}).get("type", "")
        q = self._ack_queues.get(cmd_name)
        if q is not None:
            try:
                q.put_nowait(body)
            except asyncio.QueueFull:
                logger.debug("ACK queue full for %s, dropping", cmd_name)

    # ── Heartbeats (HTTP) ──────────────────────────────────

    async def _heartbeat_loop(self) -> None:
        while True:
            try:
                await self.send_message(self._build_heartbeat())
            except asyncio.CancelledError:
                return
            except Exception as exc:
                logger.warning("Heartbeat send failed: %s", exc)
            await asyncio.sleep(1.0)

    # ── Send (HTTP POST) ───────────────────────────────────

    async def send_message(self, payload: dict) -> bool:
        if not self._http:
            return False
        try:
            resp = await self._http.post(self.config.mavlink_rest_url, json=payload)
            return resp.status_code == 200
        except httpx.RequestError as exc:
            logger.error("Send failed: %s", exc)
            return False

    # ── Message Builders ───────────────────────────────────

    def _header(self) -> dict:
        return {
            "system_id": self.config.system_id,
            "component_id": 0,
            "sequence": 0,
        }

    def _build_heartbeat(self) -> dict:
        return {
            "header": self._header(),
            "message": {
                "type": "HEARTBEAT",
                "mavtype": {"type": "MAV_TYPE_GCS"},
                "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
                "base_mode": {"bits": 0},
                "custom_mode": 0,
                "system_status": {"type": "MAV_STATE_ACTIVE"},
                "mavlink_version": 3,
            },
        }

    def _build_manual_control(self, steering: float, throttle: float) -> dict:
        # ArduRover maps: y → channel_steer, z → channel_throttle
        # Both axes use range -1000..+1000 (mapped to radio_min..radio_max)
        # x and r are ignored by Rover
        return {
            "header": self._header(),
            "message": {
                "type": "MANUAL_CONTROL",
                "target": self.config.target_system,
                "x": 0,
                "y": clamp_steering(steering),
                "z": clamp_throttle(throttle),
                "r": 0,
                "buttons": 0,
            },
        }

    def _build_set_mode(self, mode_number: int) -> dict:
        # Use COMMAND_LONG + MAV_CMD_DO_SET_MODE instead of deprecated SET_MODE msg.
        # SET_MODE's base_mode is MAV_MODE enum in MAVLink2REST (not a bitmask),
        # which can't represent arbitrary flag combos. COMMAND_LONG params are
        # plain floats so no format issues.
        # param1 = base_mode flags (CUSTOM_MODE_ENABLED=1, ARMED=128)
        # param2 = custom_mode (ArduPilot mode number)
        base = 1 | (128 if self.state.armed else 0)
        return {
            "header": self._header(),
            "message": {
                "type": "COMMAND_LONG",
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "command": {"type": "MAV_CMD_DO_SET_MODE"},
                "confirmation": 0,
                "param1": float(base),
                "param2": float(mode_number),
                "param3": 0.0,
                "param4": 0.0,
                "param5": 0.0,
                "param6": 0.0,
                "param7": 0.0,
            },
        }

    def _build_param_request_read(self, param_id: str) -> dict:
        return {
            "header": self._header(),
            "message": {
                "type": "PARAM_REQUEST_READ",
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "param_id": param_id,
                "param_index": -1,
            },
        }

    def _build_param_set(self, param_id: str, value: float) -> dict:
        return {
            "header": self._header(),
            "message": {
                "type": "PARAM_SET",
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "param_id": param_id,
                "param_value": value,
                "param_type": {"type": "MAV_PARAM_TYPE_REAL32"},
            },
        }

    def _build_arm_disarm(self, arm: bool) -> dict:
        return {
            "header": self._header(),
            "message": {
                "type": "COMMAND_LONG",
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
                "confirmation": 0,
                "param1": 1.0 if arm else 0.0,
                "param2": 0.0,
                "param3": 0.0,
                "param4": 0.0,
                "param5": 0.0,
                "param6": 0.0,
                "param7": 0.0,
            },
        }

    def _build_guided_position(self, lat: float, lon: float) -> dict:
        return {
            "header": self._header(),
            "message": {
                "type": "SET_POSITION_TARGET_GLOBAL_INT",
                "time_boot_ms": 0,
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "coordinate_frame": {"type": "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT"},
                "type_mask": {"bits": 3580},
                "lat_int": int(lat * 1e7),
                "lon_int": int(lon * 1e7),
                "alt": 0.0,
                "vx": 0.0, "vy": 0.0, "vz": 0.0,
                "afx": 0.0, "afy": 0.0, "afz": 0.0,
                "yaw": 0.0, "yaw_rate": 0.0,
            },
        }

    def _build_guided_velocity(
        self, vx: float, vy: float, yaw: float | None
    ) -> dict:
        # type_mask bits tell ArduPilot which fields to IGNORE
        # From ArduPilot wiki mavlink-rover-commands:
        #   velocity only = 3559 (0x0DE7)
        #   velocity + yaw = 2535 (0x09E7)
        if yaw is not None:
            type_mask = 2535  # velocity + yaw
            yaw_rad = math.radians(yaw)
        else:
            type_mask = 3559  # velocity only
            yaw_rad = 0.0

        return {
            "header": self._header(),
            "message": {
                "type": "SET_POSITION_TARGET_LOCAL_NED",
                "time_boot_ms": 0,
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                "coordinate_frame": {"type": "MAV_FRAME_BODY_NED"},
                "type_mask": {"bits": type_mask},
                "x": 0.0, "y": 0.0, "z": 0.0,
                "vx": vx, "vy": vy, "vz": 0.0,
                "afx": 0.0, "afy": 0.0, "afz": 0.0,
                "yaw": yaw_rad, "yaw_rate": 0.0,
            },
        }

    def _build_guided_heading(self, heading: float, speed: float) -> dict:
        # Quaternion encoding heading as yaw rotation: [cos(h/2), 0, 0, sin(h/2)]
        h_rad = math.radians(heading)
        q = [math.cos(h_rad / 2), 0.0, 0.0, math.sin(h_rad / 2)]
        # Thrust: -1..+1 where ±1 = WP_SPEED (from ArduPilot source:
        # target_speed = get_speed_default() * thrust)
        # Clamp to -1..+1, pass speed as a fraction of WP_SPEED.
        # The caller sends m/s; we normalize against WP_SPEED (default 2 m/s).
        # NOTE: WP_SPEED is configurable on the vehicle. If changed, update
        # CONNECTOR_WP_SPEED or use velocity commands instead.
        wp_speed = 2.0  # ArduRover default WP_SPEED
        thrust = max(-1.0, min(1.0, speed / wp_speed))

        return {
            "header": self._header(),
            "message": {
                "type": "SET_ATTITUDE_TARGET",
                "time_boot_ms": 0,
                "target_system": self.config.target_system,
                "target_component": self.config.target_component,
                # Bits 0-2: ignore roll/pitch/yaw rates. Bit 5 (32) is reserved
                # but harmless — ArduPilot ignores it. Cleaner value would be 7.
                "type_mask": {"bits": 39},
                "q": q,
                "body_roll_rate": 0.0,
                "body_pitch_rate": 0.0,
                "body_yaw_rate": 0.0,
                "thrust": thrust,
            },
        }

    async def _send_command_long(
        self, payload: dict, timeout: float = 3.0
    ) -> CommandResult:
        cmd_name = payload["message"]["command"]["type"]
        q: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._ack_queues[cmd_name] = q
        try:
            if not await self.send_message(payload):
                return CommandResult(sent=False, ack_result="SEND_FAILED", accepted=False)
            try:
                ack = await asyncio.wait_for(q.get(), timeout=timeout)
            except asyncio.TimeoutError:
                logger.warning("No COMMAND_ACK for %s within %.1fs", cmd_name, timeout)
                return CommandResult(sent=True, ack_result="TIMEOUT", accepted=False)
            result = ack.get("result", {}).get("type", "UNKNOWN")
            return CommandResult(
                sent=True, ack_result=result, accepted=result == "MAV_RESULT_ACCEPTED"
            )
        finally:
            self._ack_queues.pop(cmd_name, None)

    # ── Public Command Methods ─────────────────────────────

    async def send_manual_control(
        self, steering: float, throttle: float
    ) -> bool:
        ok = await self.send_message(
            self._build_manual_control(steering, throttle)
        )
        if ok:
            self.state.last_command_steering = steering
            self.state.last_command_throttle = throttle
            self.state.last_command_received = time.monotonic()
        return ok

    async def send_set_mode(self, mode_number: int) -> CommandResult:
        return await self._send_command_long(self._build_set_mode(mode_number))

    async def send_arm(self, arm: bool) -> CommandResult:
        return await self._send_command_long(self._build_arm_disarm(arm))

    async def _request_param(self, param_id: str) -> None:
        """Request a parameter value from the vehicle (fire-and-forget)."""
        await self.send_message(self._build_param_request_read(param_id))

    async def send_param_set(self, param_id: str, value: float) -> bool:
        return await self.send_message(
            self._build_param_set(param_id, value)
        )

    async def send_guided_position(self, lat: float, lon: float) -> bool:
        ok = await self.send_message(self._build_guided_position(lat, lon))
        if ok:
            self.state.last_command_received = time.monotonic()
        return ok

    async def send_guided_velocity(
        self, vx: float, vy: float, yaw: float | None
    ) -> bool:
        ok = await self.send_message(
            self._build_guided_velocity(vx, vy, yaw)
        )
        if ok:
            self.state.last_command_received = time.monotonic()
        return ok

    async def send_guided_heading(
        self, heading: float, speed: float
    ) -> bool:
        ok = await self.send_message(
            self._build_guided_heading(heading, speed)
        )
        if ok:
            self.state.last_command_received = time.monotonic()
        return ok
