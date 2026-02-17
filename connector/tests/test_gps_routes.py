import time
from contextlib import asynccontextmanager
from unittest.mock import AsyncMock

import pytest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from connector.config import Settings
from connector.gps_state import GPSState
from connector.routes.gps import router as gps_router
from connector.routes.status import router as status_router
from connector.vehicle_state import VehicleState


def _make_app(*, gps_enabled: bool = False) -> FastAPI:
    @asynccontextmanager
    async def _noop_lifespan(app: FastAPI):
        app.state.vehicle_state = VehicleState(
            mavlink_connected=True,
            last_heartbeat_received=time.monotonic(),
        )
        app.state.config = Settings()
        app.state.start_time = time.monotonic()
        app.state.mavlink_client = AsyncMock()

        if gps_enabled:
            app.state.gps_state = GPSState(
                lat=48.1173,
                lon=11.5167,
                altitude=545.4,
                fix_quality=1,
                satellites=8,
                hdop=0.9,
                speed_knots=5.2,
                course=84.4,
                utc_time="12:35:19",
                serial_connected=True,
                sentences_received=100,
                parse_errors=2,
                last_sentence_time=time.monotonic(),
            )
        yield

    test_app = FastAPI(lifespan=_noop_lifespan)
    test_app.include_router(gps_router)
    test_app.include_router(status_router)
    return test_app


class TestGPSRoute:
    def test_gps_disabled_returns_404(self):
        app = _make_app(gps_enabled=False)
        with TestClient(app, raise_server_exceptions=False) as tc:
            resp = tc.get("/gps")
            assert resp.status_code == 404

    def test_gps_enabled_returns_data(self):
        app = _make_app(gps_enabled=True)
        with TestClient(app, raise_server_exceptions=False) as tc:
            resp = tc.get("/gps")
            assert resp.status_code == 200
            data = resp.json()
            assert data["enabled"] is True
            assert data["serial_connected"] is True
            assert data["fix_quality"] == 1
            assert abs(data["lat"] - 48.1173) < 0.001
            assert abs(data["lon"] - 11.5167) < 0.001
            assert data["satellites"] == 8
            assert data["sentences_received"] == 100
            assert data["parse_errors"] == 2


class TestHealthWithGPS:
    def test_health_without_gps(self):
        app = _make_app(gps_enabled=False)
        with TestClient(app, raise_server_exceptions=False) as tc:
            resp = tc.get("/health")
            assert resp.status_code == 200
            data = resp.json()
            assert data["gps_reader_enabled"] is False
            assert data["gps_serial_connected"] is False

    def test_health_with_gps(self):
        app = _make_app(gps_enabled=True)
        with TestClient(app, raise_server_exceptions=False) as tc:
            resp = tc.get("/health")
            assert resp.status_code == 200
            data = resp.json()
            assert data["gps_reader_enabled"] is True
            assert data["gps_serial_connected"] is True
