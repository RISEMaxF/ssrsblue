"""Shared utility functions for keelson-connector-blueos.

Re-exports standard helpers from keelson.helpers and adds custom ones
for types not covered by the SDK (boolean, lon/lat with altitude).
"""

import time

from keelson import enclose
from keelson.helpers import enclose_from_float, enclose_from_integer, enclose_from_string  # noqa: F401
from keelson.payloads.Primitives_pb2 import TimestampedBool
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix


def enclose_from_boolean(value: bool, timestamp: int = None) -> bytes:
    payload = TimestampedBool()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.value = value
    return enclose(payload.SerializeToString())


def enclose_from_lon_lat(
    longitude: float, latitude: float, altitude: float = 0.0, timestamp: int = None
) -> bytes:
    """Like keelson.helpers.enclose_from_lon_lat but with altitude support."""
    payload = LocationFix()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.latitude = latitude
    payload.longitude = longitude
    payload.altitude = altitude
    return enclose(payload.SerializeToString())
