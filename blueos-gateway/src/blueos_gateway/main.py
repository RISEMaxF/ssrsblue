import asyncio
import logging
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI

from blueos_gateway.config import Settings
from blueos_gateway.gps_reader import GPSReader
from blueos_gateway.gps_state import GPSState
from blueos_gateway.mavlink_client import MAVLinkClient
from blueos_gateway.routes.commands import router as commands_router
from blueos_gateway.routes.gps import router as gps_router
from blueos_gateway.routes.status import router as status_router
from blueos_gateway.vehicle_state import VehicleState
from blueos_gateway.watchdog import watchdog_loop


@asynccontextmanager
async def lifespan(app: FastAPI):
    config = Settings()
    logging.basicConfig(
        level=config.log_level,
        format="%(asctime)s %(levelname)-8s %(name)s  %(message)s",
    )
    logger = logging.getLogger("blueos_gateway")

    state = VehicleState()
    client = MAVLinkClient(config, state)

    app.state.config = config
    app.state.vehicle_state = state
    app.state.mavlink_client = client
    app.state.start_time = time.monotonic()

    gps_reader: GPSReader | None = None
    if config.gps_enabled:
        gps_state = GPSState()
        gps_reader = GPSReader(config, gps_state)
        app.state.gps_state = gps_state
        await gps_reader.start()
        logger.info("GPS reader enabled on %s", config.gps_serial_port)

    await client.start()
    wd_task = asyncio.create_task(watchdog_loop(state, client, config))
    logger.info("BlueOS Gateway ready — target %s", config.blueos_host)

    yield

    wd_task.cancel()
    if gps_reader:
        await gps_reader.stop()
    await client.stop()
    logger.info("BlueOS Gateway stopped")


app = FastAPI(
    title="BlueOS Gateway",
    description="REST API gateway for BlueOS ArduRover boat control",
    version="0.1.0",
    lifespan=lifespan,
)

app.include_router(commands_router)
app.include_router(status_router)
app.include_router(gps_router)
