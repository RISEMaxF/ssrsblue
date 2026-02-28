#!/usr/bin/env python3

"""
keelson-connector-blueos

Bidirectional bridge between the BlueOS connector REST API and keelson/Zenoh:

  READ PATH (telemetry):
    Polls GET /status and GET /gps, publishes to Zenoh subjects.

  WRITE PATH (commands):
    Subscribes to cmd_manual_control/*, cmd_set_mode/*, cmd_arm/* on Zenoh.
    A CommandMux selects the highest-priority active source and forwards
    its steering+throttle to POST /command/manual_control.
    Mode and arm commands bypass priority and forward directly.
"""

import json
import time
import logging
import argparse
import threading

import zenoh
import keelson
import requests

from keelson.payloads.Primitives_pb2 import TimestampedString, TimestampedBool

from utils import enclose_from_boolean, enclose_from_float, enclose_from_integer, enclose_from_lon_lat, enclose_from_string
from command_mux import CommandMux

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
        "vehicle_mode", src,
        enclose_from_string(data["mode_name"], ts),
    )

    publish(
        session, realm, entity_id,
        "vehicle_armed", src,
        enclose_from_boolean(data["armed"], ts),
    )

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

    # Autopilot position (always available via MAVLink, even without external GPS)
    publish(
        session, realm, entity_id,
        "location_fix", src,
        enclose_from_lon_lat(
            longitude=data["lon"],
            latitude=data["lat"],
            timestamp=ts,
        ),
    )

    publish(
        session, realm, entity_id,
        "gps_fix_type", src,
        enclose_from_integer(int(data["gps_fix_type"]), ts),
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

    publish(
        session, realm, entity_id,
        "autopilot_throttle_pct", src,
        enclose_from_float(float(data["throttle"]), ts),
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


# ── Command forwarding ────────────────────────────────────────────


def forward_commands(mux, base_url, session, realm, entity_id, source_id,
                     interval, stop_event):
    """Background thread: forwards active mux command to connector REST API."""
    mux_src = f"{source_id}/mux"
    last_active = None

    while not stop_event.is_set():
        result = mux.get_active_command()

        if result:
            active_id, steering, throttle = result
            try:
                requests.post(
                    f"{base_url}/command/manual_control",
                    json={"steering": steering, "throttle": throttle},
                    timeout=1,
                )
            except requests.RequestException as exc:
                logger.warning("Forward manual_control failed: %s", exc)

            if active_id != last_active:
                ts = time.time_ns()
                publish(session, realm, entity_id,
                        "active_command_source", mux_src,
                        enclose_from_string(active_id, ts))
                last_active = active_id

        elif last_active is not None:
            # All sources timed out — send neutral and clear active
            try:
                requests.post(
                    f"{base_url}/command/manual_control",
                    json={"steering": 0, "throttle": 0},
                    timeout=1,
                )
            except requests.RequestException as exc:
                logger.warning("Forward neutral failed: %s", exc)

            ts = time.time_ns()
            publish(session, realm, entity_id,
                    "active_command_source", mux_src,
                    enclose_from_string("", ts))
            last_active = None

        stop_event.wait(interval)


def setup_command_subscribers(session, mux, realm, entity_id, base_url):
    """Subscribe to command subjects and wire callbacks to the mux."""
    # Key prefix for extracting source_id from received keys
    prefix = f"{realm}/@v0/{entity_id}/pubsub/"

    # cmd_manual_control — feeds into mux priority arbitration
    def on_manual_control(sample):
        try:
            key = str(sample.key_expr)
            source_id = key[len(prefix + "cmd_manual_control/"):]
            _, _, payload = keelson.uncover(bytes(sample.payload))
            msg = TimestampedString()
            msg.ParseFromString(payload)
            data = json.loads(msg.value)
            mux.on_manual_control(source_id, data["steering"], data["throttle"])
        except Exception as exc:
            logger.warning("cmd_manual_control error: %s", exc)

    # cmd_set_mode — bypasses priority, forwards directly
    def on_set_mode(sample):
        try:
            _, _, payload = keelson.uncover(bytes(sample.payload))
            msg = TimestampedString()
            msg.ParseFromString(payload)
            mode = msg.value
            logger.info("cmd_set_mode: %s", mode)
            requests.post(f"{base_url}/command/set_mode",
                          json={"mode": mode}, timeout=2)
        except Exception as exc:
            logger.warning("cmd_set_mode error: %s", exc)

    # cmd_arm — bypasses priority, forwards directly
    def on_arm(sample):
        try:
            _, _, payload = keelson.uncover(bytes(sample.payload))
            msg = TimestampedBool()
            msg.ParseFromString(payload)
            arm = msg.value
            endpoint = "arm" if arm else "disarm"
            logger.info("cmd_arm: %s", endpoint)
            requests.post(f"{base_url}/command/{endpoint}", timeout=2)
        except Exception as exc:
            logger.warning("cmd_arm error: %s", exc)

    # cmd_active_source — manual override for mux
    def on_active_source(sample):
        try:
            _, _, payload = keelson.uncover(bytes(sample.payload))
            msg = TimestampedString()
            msg.ParseFromString(payload)
            mux.on_active_source(msg.value)
        except Exception as exc:
            logger.warning("cmd_active_source error: %s", exc)

    subscribers = []
    for subject, handler in [
        ("cmd_manual_control", on_manual_control),
        ("cmd_set_mode", on_set_mode),
        ("cmd_arm", on_arm),
        ("cmd_active_source", on_active_source),
    ]:
        key_expr = keelson.construct_pubsub_key(realm, entity_id, subject, "**")
        sub = session.declare_subscriber(key_expr, handler)
        logger.info("Subscribed to %s", key_expr)
        subscribers.append(sub)

    return subscribers


# ── Main ──────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(
        description="keelson-connector-blueos: BlueOS REST API to keelson/Zenoh bridge"
    )
    parser.add_argument("-r", "--realm", required=True, help="Keelson realm")
    parser.add_argument("-e", "--entity-id", required=True, help="Entity (vessel) ID")
    parser.add_argument("-s", "--source-id", required=True, help="Base source ID")
    parser.add_argument("--blueos-url", required=True, help="BlueOS connector base URL")
    parser.add_argument(
        "--poll-interval", type=float, default=1.0, help="Seconds between telemetry polls"
    )
    parser.add_argument(
        "--command-timeout", type=float, default=2.0,
        help="Default source timeout for command mux (seconds)",
    )
    parser.add_argument(
        "--forward-interval", type=float, default=0.1,
        help="Interval for forwarding commands to connector (seconds)",
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
        conf.insert_json5("mode", json.dumps(args.mode))
    if args.connect:
        conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    logger.info(
        "Starting keelson-connector-blueos: realm=%s entity=%s source=%s url=%s poll=%.1fs fwd=%.2fs",
        args.realm, args.entity_id, args.source_id, base_url,
        args.poll_interval, args.forward_interval,
    )

    with zenoh.open(conf) as session:
        logger.info("Zenoh session opened")

        # Command mux + subscribers
        mux = CommandMux(default_timeout=args.command_timeout)
        subscribers = setup_command_subscribers(
            session, mux, args.realm, args.entity_id, base_url,
        )

        # Command forwarding thread
        stop_event = threading.Event()
        fwd_thread = threading.Thread(
            target=forward_commands,
            args=(mux, base_url, session, args.realm, args.entity_id,
                  args.source_id, args.forward_interval, stop_event),
            daemon=True,
        )
        fwd_thread.start()
        logger.info("Command forwarding started (interval=%.2fs, default_timeout=%.1fs)",
                     args.forward_interval, args.command_timeout)

        try:
            # Telemetry polling loop (unchanged)
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
                except (KeyError, TypeError, ValueError) as exc:
                    logger.warning("GET /status payload error: %s", exc)

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
                except (KeyError, TypeError, ValueError) as exc:
                    logger.warning("GET /gps payload error: %s", exc)

                time.sleep(args.poll_interval)

        except KeyboardInterrupt:
            logger.info("Shutting down")
        finally:
            stop_event.set()
            fwd_thread.join(timeout=2)
            for sub in subscribers:
                sub.undeclare()


if __name__ == "__main__":
    main()
