"""Shared utility functions for keelson-connector-blueos."""

import logging
import time

from keelson import enclose
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt, TimestampedString
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix

logger = logging.getLogger(__name__)


def enclose_from_float(value: float, timestamp: int = None) -> bytes:
    payload = TimestampedFloat()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.value = value
    return enclose(payload.SerializeToString())


def enclose_from_integer(value: int, timestamp: int = None) -> bytes:
    payload = TimestampedInt()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.value = value
    return enclose(payload.SerializeToString())


def enclose_from_string(value: str, timestamp: int = None) -> bytes:
    payload = TimestampedString()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.value = value
    return enclose(payload.SerializeToString())


def enclose_from_lon_lat(
    longitude: float, latitude: float, altitude: float = 0.0, timestamp: int = None
) -> bytes:
    payload = LocationFix()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.latitude = latitude
    payload.longitude = longitude
    payload.altitude = altitude
    return enclose(payload.SerializeToString())
