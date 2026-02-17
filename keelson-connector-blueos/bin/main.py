#!/usr/bin/env python3

"""
keelson-connector-blueos

Polls the BlueOS connector REST API and publishes telemetry
to keelson/Zenoh subjects using standard protobuf types.

Two source IDs are used:
  {source_id}/autopilot  — data from GET /status (ArduPilot telemetry)
  {source_id}/gps        — data from GET /gps (raw GPS, may be unavailable)
"""

import time
import logging
import argparse

import zenoh
import keelson
import requests

from utils import enclose_from_float, enclose_from_integer, enclose_from_lon_lat

logger = logging.getLogger("keelson-connector-blueos")

# Lazy publisher cache: (realm, entity_id, subject, source_id) -> publisher
PUBLISHERS = {}


def get_or_create_publisher(session, realm, entity_id, subject, source_id):
    key = (realm, entity_id, subject, source_id)
    if key not in PUBLISHERS:
        key_expr = keelson.construct_pubsub_key(realm, entity_id, subject, source_id)
        PUBLISHERS[key] = session.declare_publisher(key_expr)
        logger.debug("Created publisher for %s", key_expr)
    return PUBLISHERS[key]


def publish(session, realm, entity_id, subject, source_id, value):
    pub = get_or_create_publisher(session, realm, entity_id, subject, source_id)
    pub.put(value)


def publish_status(session, realm, entity_id, source_id, data, ts):
    """Publish fields from GET /status to {source_id}/autopilot subjects."""
    src = f"{source_id}/autopilot"

    publish(
        session, realm, entity_id,
        "heading_true_north_deg", src,
        enclose_from_float(float(data["heading"]), ts),
    )

    speed_knots = data["groundspeed"] * 1.94384
    publish(
        session, realm, entity_id,
        "speed_over_ground_knots", src,
        enclose_from_float(speed_knots, ts),
    )

    publish(
        session, realm, entity_id,
        "battery_voltage_v", src,
        enclose_from_float(float(data["battery_voltage"]), ts),
    )

    publish(
        session, realm, entity_id,
        "battery_current_a", src,
        enclose_from_float(float(data["battery_current"]), ts),
    )

    publish(
        session, realm, entity_id,
        "battery_state_of_charge_pct", src,
        enclose_from_float(float(data["battery_remaining"]), ts),
    )

    if data.get("last_command_steering") is not None:
        publish(
            session, realm, entity_id,
            "rudder_angle_deg", src,
            enclose_from_float(float(data["last_command_steering"]), ts),
        )

    if data.get("last_command_throttle") is not None:
        publish(
            session, realm, entity_id,
            "engine_throttle_pct", src,
            enclose_from_float(float(data["last_command_throttle"]), ts),
        )


def publish_gps(session, realm, entity_id, source_id, data, ts):
    """Publish fields from GET /gps to {source_id}/gps subjects."""
    src = f"{source_id}/gps"

    publish(
        session, realm, entity_id,
        "location_fix", src,
        enclose_from_lon_lat(
            longitude=data["lon"],
            latitude=data["lat"],
            altitude=data.get("altitude", 0.0),
            timestamp=ts,
        ),
    )

    publish(
        session, realm, entity_id,
        "location_fix_satellites_used", src,
        enclose_from_integer(int(data["satellites"]), ts),
    )

    publish(
        session, realm, entity_id,
        "location_fix_hdop", src,
        enclose_from_float(float(data["hdop"]), ts),
    )

    publish(
        session, realm, entity_id,
        "speed_over_ground_knots", src,
        enclose_from_float(float(data["speed_knots"]), ts),
    )

    publish(
        session, realm, entity_id,
        "course_over_ground_deg", src,
        enclose_from_float(float(data["course"]), ts),
    )

    publish(
        session, realm, entity_id,
        "altitude_above_msl_m", src,
        enclose_from_float(float(data["altitude"]), ts),
    )


def main():
    parser = argparse.ArgumentParser(
        description="keelson-connector-blueos: BlueOS REST API to keelson/Zenoh bridge"
    )
    parser.add_argument("-r", "--realm", required=True, help="Keelson realm")
    parser.add_argument("-e", "--entity-id", required=True, help="Entity (vessel) ID")
    parser.add_argument("-s", "--source-id", required=True, help="Base source ID")
    parser.add_argument("--blueos-url", required=True, help="BlueOS connector base URL")
    parser.add_argument(
        "--poll-interval", type=float, default=1.0, help="Seconds between polls"
    )
    parser.add_argument("--connect", type=str, default=None, help="Zenoh router endpoint")
    parser.add_argument("--mode", type=str, default=None, help="Zenoh session mode")
    parser.add_argument(
        "--log-level", type=int, default=20, help="Python log level (10=DEBUG, 20=INFO)"
    )

    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    base_url = args.blueos_url.rstrip("/")

    # Configure Zenoh
    conf = zenoh.Config()
    if args.mode:
        conf.insert_json5("mode", f'"{args.mode}"')
    if args.connect:
        conf.insert_json5("connect/endpoints", f'["{args.connect}"]')

    logger.info(
        "Starting keelson-connector-blueos: realm=%s entity=%s source=%s url=%s interval=%.1fs",
        args.realm, args.entity_id, args.source_id, base_url, args.poll_interval,
    )

    session = zenoh.open(conf)
    logger.info("Zenoh session opened")

    try:
        while True:
            ts = time.time_ns()

            # Poll /status (always expected to be available)
            try:
                resp = requests.get(f"{base_url}/status", timeout=2)
                if resp.ok:
                    publish_status(
                        session, args.realm, args.entity_id,
                        args.source_id, resp.json(), ts,
                    )
                else:
                    logger.warning("GET /status returned %d", resp.status_code)
            except requests.RequestException as exc:
                logger.warning("GET /status failed: %s", exc)

            # Poll /gps (may 404 if GPS reader is disabled)
            try:
                resp = requests.get(f"{base_url}/gps", timeout=2)
                if resp.ok:
                    data = resp.json()
                    if data.get("fix_quality", 0) > 0:
                        publish_gps(
                            session, args.realm, args.entity_id,
                            args.source_id, data, ts,
                        )
                    else:
                        logger.debug("GPS has no fix, skipping publish")
                elif resp.status_code != 404:
                    logger.warning("GET /gps returned %d", resp.status_code)
            except requests.RequestException as exc:
                logger.warning("GET /gps failed: %s", exc)

            time.sleep(args.poll_interval)

    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        session.close()


if __name__ == "__main__":
    main()
