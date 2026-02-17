import asyncio
import logging
import socket
import time

import pynmea2
import serial

from connector.config import Settings
from connector.gps_state import GPSState

logger = logging.getLogger(__name__)


class GPSReader:
    """Reads NMEA from a serial GPS, forwards raw sentences via UDP, and parses locally."""

    def __init__(self, config: Settings, state: GPSState) -> None:
        self.config = config
        self.state = state
        self._task: asyncio.Task | None = None
        self._udp_sock: socket.socket | None = None

    # ── Lifecycle ──────────────────────────────────────────

    async def start(self) -> None:
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._task = asyncio.create_task(self._read_loop(), name="gps-reader")
        logger.info(
            "GPSReader started — port=%s baud=%s udp=%s:%s",
            self.config.gps_serial_port,
            self.config.gps_serial_baud,
            self.config.gps_udp_host,
            self.config.gps_udp_port,
        )

    async def stop(self) -> None:
        if self._task:
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        if self._udp_sock:
            self._udp_sock.close()
        logger.info("GPSReader stopped")

    # ── Main loop ─────────────────────────────────────────

    async def _read_loop(self) -> None:
        backoff = 1.0

        while True:
            try:
                ser = serial.Serial(
                    self.config.gps_serial_port,
                    self.config.gps_serial_baud,
                    timeout=2.0,
                )
                self.state.serial_connected = True
                backoff = 1.0
                logger.info("Serial port %s opened", self.config.gps_serial_port)

                try:
                    while True:
                        raw = await asyncio.to_thread(ser.readline)
                        if not raw:
                            continue
                        line = raw.decode("ascii", errors="replace").strip()
                        if not line:
                            continue

                        self._forward_udp(line)
                        self._parse_sentence(line)
                finally:
                    ser.close()

            except serial.SerialException as exc:
                self.state.serial_connected = False
                logger.warning(
                    "Serial error on %s: %s. Reconnecting in %.0fs…",
                    self.config.gps_serial_port,
                    exc,
                    backoff,
                )
            except asyncio.CancelledError:
                self.state.serial_connected = False
                return
            except Exception as exc:
                self.state.serial_connected = False
                logger.error("Unexpected GPS error: %s", exc)

            try:
                await asyncio.sleep(backoff)
            except asyncio.CancelledError:
                return
            backoff = min(backoff * 2, 10.0)

    # ── UDP forwarding ────────────────────────────────────

    def _forward_udp(self, line: str) -> None:
        if not self._udp_sock:
            return
        try:
            self._udp_sock.sendto(
                (line + "\r\n").encode("ascii"),
                (self.config.gps_udp_host, self.config.gps_udp_port),
            )
        except OSError as exc:
            logger.debug("UDP forward failed: %s", exc)

    # ── NMEA parsing ──────────────────────────────────────

    def _parse_sentence(self, line: str) -> None:
        try:
            msg = pynmea2.parse(line)
        except pynmea2.ParseError:
            self.state.parse_errors += 1
            return

        self.state.sentences_received += 1
        self.state.last_sentence_time = time.monotonic()

        if isinstance(msg, pynmea2.types.talker.GGA):
            self.state.fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
            self.state.satellites = int(msg.num_sats) if msg.num_sats else 0
            self.state.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9
            self.state.altitude = float(msg.altitude) if msg.altitude else 0.0
            if msg.latitude and msg.longitude:
                self.state.lat = msg.latitude
                self.state.lon = msg.longitude
            if msg.timestamp:
                self.state.utc_time = str(msg.timestamp)

        elif isinstance(msg, pynmea2.types.talker.RMC):
            self.state.speed_knots = float(msg.spd_over_grnd) if msg.spd_over_grnd else 0.0
            self.state.course = float(msg.true_course) if msg.true_course else 0.0
            if msg.latitude and msg.longitude:
                self.state.lat = msg.latitude
                self.state.lon = msg.longitude
            if msg.timestamp:
                self.state.utc_time = str(msg.timestamp)
