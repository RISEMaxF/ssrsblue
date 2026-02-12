import asyncio
import logging
import time

from connector.config import Settings
from connector.mavlink_client import MAVLinkClient
from connector.validators import is_controllable_mode
from connector.vehicle_state import VehicleState

logger = logging.getLogger(__name__)


async def watchdog_loop(
    state: VehicleState, client: MAVLinkClient, config: Settings
) -> None:
    """Send neutral commands if no external commands received recently.

    Only activates when the vehicle is armed and in a companion-controlled
    mode (GUIDED or STEERING). Sends steering=0, throttle=0 (stop).
    """
    warned = False

    while True:
        await asyncio.sleep(0.5)

        if not state.armed or not is_controllable_mode(state.mode):
            warned = False
            continue

        if state.last_command_received == 0.0:
            # No commands ever sent — don't activate watchdog
            continue

        elapsed = time.monotonic() - state.last_command_received
        if elapsed > config.watchdog_timeout:
            if not warned:
                logger.warning(
                    "Watchdog: no commands for %.1fs, sending neutral", elapsed
                )
                warned = True
            await client.send_manual_control(steering=0, throttle=0)
        else:
            warned = False
