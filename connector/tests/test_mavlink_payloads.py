"""Tests that verify the actual MAVLink JSON payload structure matches
what ArduRover expects. These catch axis-mapping and range bugs.

Reference: ArduPilot source (Rover/GCS_MAVLink_Rover.cpp):
  manual_override(rover.channel_steer, packet.y, 1000, 2000, tnow);
  manual_override(rover.channel_throttle, packet.z, 1000, 2000, tnow);
"""

import math

from connector.config import Settings
from connector.mavlink_client import MAVLinkClient
from connector.vehicle_state import VehicleState


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
    def test_base_mode_disarmed(self):
        client = _make_client(armed=False)
        payload = client._build_set_mode(15)
        bits = payload["message"]["base_mode"]["bits"]
        assert bits & 1, "CUSTOM_MODE_ENABLED (bit 0) must be set"
        assert not (bits & 128), "ARMED (bit 7) must NOT be set when disarmed"

    def test_base_mode_armed(self):
        client = _make_client(armed=True)
        payload = client._build_set_mode(15)
        bits = payload["message"]["base_mode"]["bits"]
        assert bits & 1, "CUSTOM_MODE_ENABLED must be set"
        assert bits & 128, "ARMED must be set when armed"

    def test_custom_mode_passthrough(self):
        client = _make_client()
        payload = client._build_set_mode(3)
        assert payload["message"]["custom_mode"] == 3


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
        assert payload["message"]["type_"]["type"] == "MAV_TYPE_GCS"

    def test_system_id(self):
        client = _make_client()
        payload = client._build_heartbeat()
        assert payload["header"]["system_id"] == 254
