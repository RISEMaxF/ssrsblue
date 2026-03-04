"""Tests that verify the actual MAVLink JSON payload structure matches
what ArduRover expects. These catch axis-mapping and range bugs.

Reference: ArduPilot source (Rover/GCS_MAVLink_Rover.cpp):
  manual_override(rover.channel_steer, packet.y, 1000, 2000, tnow);
  manual_override(rover.channel_throttle, packet.z, 1000, 2000, tnow);
"""

import asyncio
import math

import pytest

from blueos_gateway.config import Settings
from blueos_gateway.mavlink_client import MAVLinkClient
from blueos_gateway.vehicle_state import VehicleState


def _make_client(**state_kwargs) -> MAVLinkClient:
    config = Settings()
    state = VehicleState(**state_kwargs)
    return MAVLinkClient(config, state)


class TestManualControlPayload:
    def test_steering_goes_to_y_axis(self):
        client = _make_client()
        payload = client._build_manual_control(steering=500, throttle=0)
        msg = payload["message"]
        assert msg["y"] == 500, "Steering must map to y axis"
        assert msg["x"] == 0, "x must be 0 (ignored by Rover)"
        assert msg["r"] == 0, "r must be 0 (ignored by Rover)"

    def test_throttle_goes_to_z_axis(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=700)
        msg = payload["message"]
        assert msg["z"] == 700, "Throttle must map to z axis"

    def test_throttle_zero_is_stop(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=0)
        assert payload["message"]["z"] == 0, "z=0 must mean stop"

    def test_throttle_negative_is_reverse(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=-500)
        assert payload["message"]["z"] == -500, "Negative z = reverse"

    def test_steering_clamped(self):
        client = _make_client()
        payload = client._build_manual_control(steering=9999, throttle=0)
        assert payload["message"]["y"] == 1000

    def test_throttle_clamped_high(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=5000)
        assert payload["message"]["z"] == 1000

    def test_throttle_clamped_low(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=-5000)
        assert payload["message"]["z"] == -1000

    def test_system_id_from_config(self):
        client = _make_client()
        payload = client._build_manual_control(steering=0, throttle=0)
        assert payload["header"]["system_id"] == 254


class TestSetModePayload:
    def test_uses_command_long(self):
        client = _make_client()
        payload = client._build_set_mode(15)
        assert payload["message"]["type"] == "COMMAND_LONG"
        assert payload["message"]["command"]["type"] == "MAV_CMD_DO_SET_MODE"

    def test_base_mode_disarmed(self):
        client = _make_client(armed=False)
        payload = client._build_set_mode(15)
        base = payload["message"]["param1"]
        assert int(base) & 1, "CUSTOM_MODE_ENABLED (bit 0) must be set"
        assert not (int(base) & 128), "ARMED (bit 7) must NOT be set when disarmed"

    def test_base_mode_armed(self):
        client = _make_client(armed=True)
        payload = client._build_set_mode(15)
        base = payload["message"]["param1"]
        assert int(base) & 1, "CUSTOM_MODE_ENABLED must be set"
        assert int(base) & 128, "ARMED must be set when armed"

    def test_custom_mode_passthrough(self):
        client = _make_client()
        payload = client._build_set_mode(3)
        assert payload["message"]["param2"] == 3.0


class TestGuidedPositionPayload:
    def test_type_mask(self):
        client = _make_client()
        payload = client._build_guided_position(47.0, 8.0)
        assert payload["message"]["type_mask"]["bits"] == 3580

    def test_lat_lon_scaled(self):
        client = _make_client()
        payload = client._build_guided_position(47.3977, 8.5455)
        msg = payload["message"]
        assert msg["lat_int"] == 473977000
        assert msg["lon_int"] == 85455000


class TestGuidedVelocityPayload:
    def test_velocity_only_mask(self):
        client = _make_client()
        payload = client._build_guided_velocity(1.0, 0.0, yaw=None)
        assert payload["message"]["type_mask"]["bits"] == 3559

    def test_velocity_plus_yaw_mask(self):
        client = _make_client()
        payload = client._build_guided_velocity(1.0, 0.0, yaw=90.0)
        assert payload["message"]["type_mask"]["bits"] == 2535

    def test_yaw_converted_to_radians(self):
        client = _make_client()
        payload = client._build_guided_velocity(0.0, 0.0, yaw=180.0)
        assert abs(payload["message"]["yaw"] - math.pi) < 0.001


class TestGuidedHeadingPayload:
    def test_type_mask(self):
        client = _make_client()
        payload = client._build_guided_heading(90.0, 1.0)
        assert payload["message"]["type_mask"]["bits"] == 39

    def test_thrust_range(self):
        client = _make_client()
        # speed=2.0 with wp_speed=2.0 → thrust=1.0
        payload = client._build_guided_heading(0.0, 2.0)
        assert payload["message"]["thrust"] == 1.0

    def test_thrust_negative(self):
        client = _make_client()
        payload = client._build_guided_heading(0.0, -2.0)
        assert payload["message"]["thrust"] == -1.0

    def test_thrust_clamped(self):
        client = _make_client()
        payload = client._build_guided_heading(0.0, 99.0)
        assert payload["message"]["thrust"] == 1.0

    def test_quaternion_north(self):
        client = _make_client()
        payload = client._build_guided_heading(0.0, 0.0)
        q = payload["message"]["q"]
        # heading=0 → yaw=0 → q=[1,0,0,0]
        assert abs(q[0] - 1.0) < 0.001
        assert abs(q[3]) < 0.001

    def test_quaternion_east(self):
        client = _make_client()
        payload = client._build_guided_heading(90.0, 0.0)
        q = payload["message"]["q"]
        # heading=90° → yaw=π/2 → q=[cos(π/4), 0, 0, sin(π/4)]
        assert abs(q[0] - math.cos(math.pi / 4)) < 0.001
        assert abs(q[3] - math.sin(math.pi / 4)) < 0.001


class TestArmDisarmPayload:
    def test_arm(self):
        client = _make_client()
        payload = client._build_arm_disarm(True)
        assert payload["message"]["param1"] == 1.0

    def test_disarm(self):
        client = _make_client()
        payload = client._build_arm_disarm(False)
        assert payload["message"]["param1"] == 0.0


class TestHeartbeatPayload:
    def test_gcs_type(self):
        client = _make_client()
        payload = client._build_heartbeat()
        assert payload["message"]["mavtype"]["type"] == "MAV_TYPE_GCS"

    def test_system_id(self):
        client = _make_client()
        payload = client._build_heartbeat()
        assert payload["header"]["system_id"] == 254


class TestCommandAckRouting:
    def test_ack_routed_to_matching_queue(self):
        client = _make_client()
        q: asyncio.Queue = asyncio.Queue(maxsize=1)
        client._ack_queues["MAV_CMD_DO_SET_MODE"] = q

        ack_msg = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 0},
            "message": {
                "type": "COMMAND_ACK",
                "command": {"type": "MAV_CMD_DO_SET_MODE"},
                "result": {"type": "MAV_RESULT_ACCEPTED"},
            },
        }
        client._process_message(ack_msg)

        assert not q.empty()
        body = q.get_nowait()
        assert body["result"]["type"] == "MAV_RESULT_ACCEPTED"

    def test_ack_ignored_when_no_queue(self):
        client = _make_client()
        # No queue registered — should not raise
        ack_msg = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 0},
            "message": {
                "type": "COMMAND_ACK",
                "command": {"type": "MAV_CMD_DO_SET_MODE"},
                "result": {"type": "MAV_RESULT_ACCEPTED"},
            },
        }
        client._process_message(ack_msg)  # should not raise

    def test_ack_wrong_command_not_routed(self):
        client = _make_client()
        q: asyncio.Queue = asyncio.Queue(maxsize=1)
        client._ack_queues["MAV_CMD_DO_SET_MODE"] = q

        ack_msg = {
            "header": {"system_id": 1, "component_id": 1, "sequence": 0},
            "message": {
                "type": "COMMAND_ACK",
                "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
                "result": {"type": "MAV_RESULT_ACCEPTED"},
            },
        }
        client._process_message(ack_msg)

        assert q.empty()

    @pytest.mark.asyncio
    async def test_send_command_long_timeout(self):
        client = _make_client()
        # Mock send_message to succeed but no ACK arrives
        async def fake_send(payload):
            return True
        client.send_message = fake_send

        payload = client._build_set_mode(15)
        result = await client._send_command_long(payload, timeout=0.05)

        assert result.sent is True
        assert result.ack_result == "TIMEOUT"
        assert result.accepted is False
        # Queue should be cleaned up
        assert len(client._ack_queues) == 0

    @pytest.mark.asyncio
    async def test_send_command_long_accepted(self):
        client = _make_client()

        async def fake_send(payload):
            # Simulate ACK arriving shortly after send
            async def inject_ack():
                await asyncio.sleep(0.01)
                ack_body = {
                    "type": "COMMAND_ACK",
                    "command": {"type": "MAV_CMD_DO_SET_MODE"},
                    "result": {"type": "MAV_RESULT_ACCEPTED"},
                }
                client._handle_command_ack(ack_body)
            asyncio.create_task(inject_ack())
            return True
        client.send_message = fake_send

        payload = client._build_set_mode(15)
        result = await client._send_command_long(payload, timeout=1.0)

        assert result.sent is True
        assert result.ack_result == "MAV_RESULT_ACCEPTED"
        assert result.accepted is True

    @pytest.mark.asyncio
    async def test_send_command_long_failed(self):
        client = _make_client()

        async def fake_send(payload):
            async def inject_ack():
                await asyncio.sleep(0.01)
                ack_body = {
                    "type": "COMMAND_ACK",
                    "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
                    "result": {"type": "MAV_RESULT_FAILED"},
                }
                client._handle_command_ack(ack_body)
            asyncio.create_task(inject_ack())
            return True
        client.send_message = fake_send

        payload = client._build_arm_disarm(True)
        result = await client._send_command_long(payload, timeout=1.0)

        assert result.sent is True
        assert result.ack_result == "MAV_RESULT_FAILED"
        assert result.accepted is False

    @pytest.mark.asyncio
    async def test_send_command_long_send_fails(self):
        client = _make_client()

        async def fake_send(payload):
            return False
        client.send_message = fake_send

        payload = client._build_set_mode(15)
        result = await client._send_command_long(payload, timeout=0.05)

        assert result.sent is False
        assert result.ack_result == "SEND_FAILED"
        assert result.accepted is False
