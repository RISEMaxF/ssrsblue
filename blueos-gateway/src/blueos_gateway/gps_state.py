import time
from dataclasses import dataclass, field


@dataclass
class GPSState:
    # From GGA
    lat: float = 0.0
    lon: float = 0.0
    altitude: float = 0.0
    fix_quality: int = 0
    satellites: int = 0
    hdop: float = 99.9

    # From RMC
    speed_knots: float = 0.0
    course: float = 0.0
    utc_time: str = ""

    # Connection
    serial_connected: bool = False

    # Counters
    sentences_received: int = 0
    parse_errors: int = 0
    last_sentence_time: float = field(default_factory=time.monotonic)
