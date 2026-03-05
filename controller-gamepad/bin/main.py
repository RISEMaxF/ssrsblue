#!/usr/bin/env python3

import json
import time
import logging
import argparse

import zenoh
import usb.core

from gamepad import create_gamepad, GamepadState
from utils import enclose_from_boolean, enclose_from_float, enclose_from_string

logger = logging.getLogger("controller-gamepad")


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


def publish_command(session, realm, entity_id, source_id, steering, throttle, ts):
    """Publish steering+throttle as cmd_manual_control (JSON in TimestampedString)."""
    payload = json.dumps({"steering": steering, "throttle": throttle})
    publish(session, realm, entity_id,
            "cmd_manual_control", source_id,
            enclose_from_string(payload, ts))


def publish_arm(session, realm, entity_id, source_id, armed, ts):
    """Publish arm/disarm command as cmd_arm (TimestampedBool)."""
    publish(session, realm, entity_id,
            "cmd_arm", source_id,
            enclose_from_boolean(armed, ts))
    logger.info("Published cmd_arm=%s", armed)


def publish_telemetry(session, realm, entity_id, source_id, steering, throttle, ts):
    """Publish steering+throttle as telemetry subjects for recording."""
    publish(session, realm, entity_id,
            "rudder_angle_deg", source_id,
            enclose_from_float(float(steering) / 10.0, ts))
    publish(session, realm, entity_id,
            "engine_throttle_pct", source_id,
            enclose_from_float(float(throttle) / 10.0, ts))


def run_loop(args, session):
    pad = create_gamepad(args.gamepad, deadzone=args.deadzone)
    last_command_time = 0.0
    armed = False
    prev_start = False
    last_arm_toggle_time = 0.0

    while True:
        try:
            pad.open()
            logger.info("Gamepad connected")

            while True:
                state = pad.read()

                throttle = int(state.left_y * 1000)
                steering = int(state.right_x * 1000)

                now = time.monotonic()

                # Arm/disarm toggle on Start button (rising edge + 1s cooldown)
                start_pressed = state.buttons.get("START", False)
                if start_pressed and not prev_start and (now - last_arm_toggle_time) >= 1.0:
                    armed = not armed
                    ts = time.time_ns()
                    publish_arm(session, args.realm, args.entity_id,
                                args.source_id, armed, ts)
                    last_arm_toggle_time = now
                prev_start = start_pressed

                if now - last_command_time >= args.command_interval:
                    logger.debug("throttle=%d steering=%d (ly=%.2f rx=%.2f lx=%.2f ry=%.2f lt=%.2f rt=%.2f armed=%s)",
                                 throttle, steering, state.left_y, state.right_x,
                                 state.left_x, state.right_y, state.left_trigger, state.right_trigger, armed)

                    ts = time.time_ns()
                    publish_command(session, args.realm, args.entity_id,
                                   args.source_id, steering, throttle, ts)
                    publish_telemetry(session, args.realm, args.entity_id,
                                     args.source_id, steering, throttle, ts)
                    last_command_time = now

                time.sleep(args.poll_interval)

        except usb.core.USBError:
            logger.error("Gamepad disconnected")
            # Publish neutral so mux gets an immediate zero before timeout
            ts = time.time_ns()
            publish_command(session, args.realm, args.entity_id,
                           args.source_id, 0, 0, ts)
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
        description="controller-gamepad: USB gamepad to keelson/Zenoh commands"
    )
    parser.add_argument("-r", "--realm", required=True, help="Keelson realm")
    parser.add_argument("-e", "--entity-id", required=True, help="Entity ID")
    parser.add_argument("-s", "--source-id", required=True, help="Source ID (e.g. gamepad/0)")
    parser.add_argument("--connect", required=True, help="Zenoh router endpoint")
    parser.add_argument("--gamepad", default="f310", help="Gamepad type (default: f310)")
    parser.add_argument("--deadzone", type=float, default=0.05, help="Analog stick deadzone")
    parser.add_argument("--poll-interval", type=float, default=0.05, help="Seconds between reads")
    parser.add_argument("--command-interval", type=float, default=0.1, help="Min seconds between publishes")
    parser.add_argument("--mode", default=None, help="Zenoh session mode")
    parser.add_argument("--log-level", type=int, default=20, help="Python log level (10=DEBUG, 20=INFO)")

    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    logger.info(
        "Starting controller-gamepad: realm=%s entity=%s source=%s gamepad=%s poll=%.3fs cmd=%.3fs",
        args.realm, args.entity_id, args.source_id,
        args.gamepad, args.poll_interval, args.command_interval,
    )

    conf = zenoh.Config()
    if args.mode:
        conf.insert_json5("mode", json.dumps(args.mode))
    conf.insert_json5("connect/endpoints", json.dumps([args.connect]))

    session = zenoh.open(conf)
    logger.info("Zenoh session opened")

    try:
        run_loop(args, session)
    except KeyboardInterrupt:
        logger.info("Shutting down")
    finally:
        session.close()


if __name__ == "__main__":
    main()
