#!/usr/bin/env python3

import time

from keelson import enclose  # noqa: F401
from keelson.helpers import enclose_from_float, enclose_from_integer, enclose_from_string  # noqa: F401
from keelson.payloads.Primitives_pb2 import TimestampedBool


def enclose_from_boolean(value: bool, timestamp: int = None) -> bytes:
    payload = TimestampedBool()
    payload.timestamp.FromNanoseconds(timestamp or time.time_ns())
    payload.value = value
    return enclose(payload.SerializeToString())
