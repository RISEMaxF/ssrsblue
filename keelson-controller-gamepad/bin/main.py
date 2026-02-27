#!/usr/bin/env python3

import json
import time
import logging
import argparse

import requests
import usb.core

from gamepad import create_gamepad, GamepadState

logger = logging.getLogger("keelson-controller-gamepad")


PUBLISHERS = {}


def get_or_create_publisher(session, realm, entity_id, subject, source_id):
    import keelson

    key = (realm, entity_id, subject, source_id)
    if key not in PUBLISHERS:
        key_expr = keelson.construct_pubsub_key(realm, entity_id, subject, source_id)
        PUBLISHERS[key] = session.declare_publisher(key_expr)
        logger.debug("Created publisher for %s", key_expr)
    return PUBLISHERS[key]


def publish(session, realm, entity_id, subject, source_id, value):
    pub = get_or_create_publisher(session, realm, entity_id, subject, source_id)
    pub.put(value)


def send_command(base_url, steering, throttle):
    try:
        requests.post(
            f"{base_url}/command/manual_control",
            json={"steering": steering, "throttle": throttle},
            timeout=1,
        )
    except requests.RequestException as exc:
        logger.warning("POST /command/manual_control failed: %s", exc)


def send_neutral(base_url):
    logger.warning("Sending neutral command (steering=0, throttle=0)")
    send_command(base_url, 0, 0)


def run_loop(args, session=None):
    base_url = args.blueos_url.rstrip("/")
    pad = create_gamepad(args.gamepad, deadzone=args.deadzone)
    last_command_time = 0.0

    while True:
        try:
            pad.open()
            logger.info("Gamepad connected")

            while True:
                state = pad.read()

                steering = int(state.left_x * 1000)
                throttle = int(-state.left_y * 1000)

                now = time.monotonic()
                if now - last_command_time >= args.command_interval:
                    logger.debug("steering=%d throttle=%d (lx=%.2f ly=%.2f rx=%.2f ry=%.2f lt=%.2f rt=%.2f)",
                                 steering, throttle, state.left_x, state.left_y,
                                 state.right_x, state.right_y, state.left_trigger, state.right_trigger)
                    send_command(base_url, steering, throttle)
                    last_command_time = now

                    if session and args.realm:
                        ts = time.time_ns()
                        from utils import enclose_from_float
                        src = args.source_id or "gamepad/0"
                        publish(session, args.realm, args.entity_id,
                                "rudder_angle_deg", src,
                                enclose_from_float(float(steering) / 10.0, ts))
                        publish(session, args.realm, args.entity_id,
                                "engine_throttle_pct", src,
                                enclose_from_float(float(throttle) / 10.0, ts))

                time.sleep(args.poll_interval)

        except usb.core.USBError:
            logger.error("Gamepad disconnected")
            send_neutral(base_url)
            try:
                pad.close()
            except Exception:
                pass
            logger.info("Attempting reconnect in 2s...")
            time.sleep(2)

        except RuntimeError as exc:
            logger.error("Gamepad not found: %s", exc)
            time.sleep(2)


def main():
    parser = argparse.ArgumentParser(
        description="keelson-controller-gamepad: USB gamepad to BlueOS manual control"
    )
    parser.add_argument("--blueos-url", required=True, help="BlueOS connector base URL")
    parser.add_argument("--gamepad", default="f310", help="Gamepad type (default: f310)")
    parser.add_argument("--deadzone", type=float, default=0.05, help="Analog stick deadzone")
    parser.add_argument("--poll-interval", type=float, default=0.05, help="Seconds between reads")
    parser.add_argument("--command-interval", type=float, default=0.1, help="Min seconds between API posts")
    parser.add_argument("--log-level", type=int, default=20, help="Python log level (10=DEBUG, 20=INFO)")

    # Optional Zenoh/keelson args
    parser.add_argument("-r", "--realm", default=None, help="Keelson realm (enables Zenoh publishing)")
    parser.add_argument("-e", "--entity-id", default=None, help="Entity ID")
    parser.add_argument("-s", "--source-id", default=None, help="Source ID (e.g. gamepad/0)")
    parser.add_argument("--connect", default=None, help="Zenoh router endpoint")
    parser.add_argument("--mode", default=None, help="Zenoh session mode")

    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    logger.info(
        "Starting keelson-controller-gamepad: url=%s gamepad=%s poll=%.3fs cmd=%.3fs",
        args.blueos_url, args.gamepad, args.poll_interval, args.command_interval,
    )

    session = None
    if args.realm:
        import zenoh
        conf = zenoh.Config()
        if args.mode:
            conf.insert_json5("mode", json.dumps(args.mode))
        if args.connect:
            conf.insert_json5("connect/endpoints", json.dumps([args.connect]))
        session = zenoh.open(conf)
        logger.info("Zenoh session opened")

    try:
        run_loop(args, session)
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        if session is not None:
            session.close()


if __name__ == "__main__":
    main()
