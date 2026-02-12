import time
from contextlib import asynccontextmanager
from unittest.mock import AsyncMock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from connector.config import Settings
from connector.routes.commands import router as commands_router
from connector.routes.status import router as status_router
from connector.vehicle_state import VehicleState


def _make_app(state: VehicleState, mock_mavlink: AsyncMock) -> FastAPI:
    """Build a test app with no real lifespan (no WebSocket/heartbeat tasks)."""

    @asynccontextmanager
    async def _noop_lifespan(app: FastAPI):
        app.state.vehicle_state = state
        app.state.mavlink_client = mock_mavlink
        app.state.config = Settings()
        app.state.start_time = time.monotonic()
        yield

    test_app = FastAPI(lifespan=_noop_lifespan)
    test_app.include_router(commands_router)
    test_app.include_router(status_router)
    return test_app


@pytest.fixture()
def client():
    state = VehicleState(
        mode=15,
        mode_name="GUIDED",
        armed=True,
        mavlink_connected=True,
        last_heartbeat_received=time.monotonic(),
    )

    mock_mavlink = AsyncMock()
    mock_mavlink.send_manual_control = AsyncMock(return_value=True)
    mock_mavlink.send_set_mode = AsyncMock(return_value=True)
    mock_mavlink.send_arm = AsyncMock(return_value=True)
    mock_mavlink.send_guided_position = AsyncMock(return_value=True)
    mock_mavlink.send_guided_velocity = AsyncMock(return_value=True)
    mock_mavlink.send_guided_heading = AsyncMock(return_value=True)

    test_app = _make_app(state, mock_mavlink)

    with TestClient(test_app, raise_server_exceptions=False) as tc:
        yield tc, state, mock_mavlink


class TestStatus:
    def test_status(self, client):
        tc, state, _ = client
        resp = tc.get("/status")
        assert resp.status_code == 200
        data = resp.json()
        assert data["mode"] == 15
        assert data["mode_name"] == "GUIDED"
        assert data["armed"] is True

    def test_health_ok(self, client):
        tc, _, _ = client
        resp = tc.get("/health")
        assert resp.status_code == 200
        assert resp.json()["status"] == "ok"

    def test_health_disconnected(self, client):
        tc, state, _ = client
        state.mavlink_connected = False
        resp = tc.get("/health")
        assert resp.json()["status"] == "disconnected"


class TestManualControl:
    def test_send(self, client):
        tc, _, mock = client
        resp = tc.post(
            "/command/manual_control",
            json={"steering": 100, "throttle": 600},
        )
        assert resp.status_code == 200
        assert resp.json()["success"] is True
        mock.send_manual_control.assert_awaited_once_with(100, 600)

    def test_disconnected(self, client):
        tc, state, _ = client
        state.mavlink_connected = False
        resp = tc.post(
            "/command/manual_control",
            json={"steering": 0, "throttle": 500},
        )
        assert resp.status_code == 503


class TestSetMode:
    def test_by_name(self, client):
        tc, _, mock = client
        resp = tc.post("/command/set_mode", json={"mode": "MANUAL"})
        assert resp.status_code == 200
        mock.send_set_mode.assert_awaited_once_with(0)

    def test_by_number(self, client):
        tc, _, mock = client
        resp = tc.post("/command/set_mode", json={"mode": 15})
        assert resp.status_code == 200
        mock.send_set_mode.assert_awaited_once_with(15)

    def test_invalid_mode(self, client):
        tc, _, _ = client
        resp = tc.post("/command/set_mode", json={"mode": "HOVER"})
        assert resp.status_code == 400


class TestArmDisarm:
    def test_arm(self, client):
        tc, _, mock = client
        resp = tc.post("/command/arm")
        assert resp.status_code == 200
        mock.send_arm.assert_awaited_once_with(True)

    def test_disarm(self, client):
        tc, _, mock = client
        resp = tc.post("/command/disarm")
        assert resp.status_code == 200
        mock.send_arm.assert_awaited_once_with(False)


class TestGuided:
    def test_position(self, client):
        tc, _, mock = client
        resp = tc.post(
            "/command/guided/position", json={"lat": 47.3977, "lon": 8.5455}
        )
        assert resp.status_code == 200
        mock.send_guided_position.assert_awaited_once()

    def test_position_wrong_mode(self, client):
        tc, state, _ = client
        state.mode = 0
        state.mode_name = "MANUAL"
        resp = tc.post(
            "/command/guided/position", json={"lat": 47.3977, "lon": 8.5455}
        )
        assert resp.status_code == 400
        assert "GUIDED" in resp.json()["detail"]

    def test_velocity(self, client):
        tc, _, mock = client
        resp = tc.post(
            "/command/guided/velocity", json={"vx": 2.0, "vy": 0.0}
        )
        assert resp.status_code == 200
        mock.send_guided_velocity.assert_awaited_once()

    def test_heading(self, client):
        tc, _, mock = client
        resp = tc.post(
            "/command/guided/heading", json={"heading": 90, "speed": 1.5}
        )
        assert resp.status_code == 200
        mock.send_guided_heading.assert_awaited_once()
