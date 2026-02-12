import asyncio
import logging
import time
from contextlib import asynccontextmanager

from fastapi import FastAPI

from connector.config import Settings
from connector.mavlink_client import MAVLinkClient
from connector.routes.commands import router as commands_router
from connector.routes.status import router as status_router
from connector.vehicle_state import VehicleState
from connector.watchdog import watchdog_loop


@asynccontextmanager
async def lifespan(app: FastAPI):
    config = Settings()
    logging.basicConfig(
        level=config.log_level,
        format="%(asctime)s %(levelname)-8s %(name)s  %(message)s",
    )
    logger = logging.getLogger("connector")

    state = VehicleState()
    client = MAVLinkClient(config, state)

    app.state.config = config
    app.state.vehicle_state = state
    app.state.mavlink_client = client
    app.state.start_time = time.monotonic()

    await client.start()
    wd_task = asyncio.create_task(watchdog_loop(state, client, config))
    logger.info("BlueOS Connector ready — target %s", config.blueos_host)

    yield

    wd_task.cancel()
    await client.stop()
    logger.info("BlueOS Connector stopped")


app = FastAPI(
    title="BlueOS Connector",
    description="REST API gateway for BlueOS ArduRover boat control",
    version="0.1.0",
    lifespan=lifespan,
)

app.include_router(commands_router)
app.include_router(status_router)
