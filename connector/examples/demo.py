#!/usr/bin/env python3
"""
Example usage of the BlueOS Connector API.

Run the connector first:
    docker compose up
    # or: uv run uvicorn connector.main:app --port 8080

Then run this script:
    uv run python examples/demo.py
    uv run python examples/demo.py --base-url http://192.168.2.100:8080  # from another machine
"""

import argparse
import sys
import time

import httpx

# ── Defaults ───────────────────────────────────────────────

# localhost works when running on the same NUC as the connector.
# If calling from a different machine, pass --base-url with the NUC's IP.
DEFAULT_BASE_URL = "http://localhost:8080"
COMMAND_RATE_HZ = 10


def main():
    parser = argparse.ArgumentParser(description="BlueOS Connector demo")
    parser.add_argument(
        "--base-url", default=DEFAULT_BASE_URL,
        help=f"Connector URL (default: {DEFAULT_BASE_URL})",
    )
    args = parser.parse_args()
    base = args.base_url.rstrip("/")

    client = httpx.Client(base_url=base, timeout=5.0)

    # ── 1. Health check ────────────────────────────────────
    print("=== Health Check ===")
    health = client.get("/health").json()
    print(f"  Status:    {health['status']}")
    print(f"  Connected: {health['mavlink_connected']}")
    print(f"  Heartbeat: {health['last_heartbeat_age_s']}s ago")

    if health["status"] == "disconnected":
        print("\n  Connector is not connected to BlueOS.")
        print("  Check that the Pi is powered and ethernet is plugged in.")
        print("  The connector will keep retrying automatically.")
        sys.exit(1)

    # ── 2. Vehicle status ──────────────────────────────────
    print("\n=== Vehicle Status ===")
    status = client.get("/status").json()
    print(f"  Mode:       {status['mode_name']} ({status['mode']})")
    print(f"  Armed:      {status['armed']}")
    print(f"  GPS fix:    {status['gps_fix_type']}")
    print(f"  Position:   {status['lat']:.6f}, {status['lon']:.6f}")
    print(f"  Satellites:  {status['satellites_visible']}")
    print(f"  Battery:    {status['battery_voltage']}V  {status['battery_remaining']}%")
    print(f"  Heading:    {status['heading']}°")
    print(f"  Speed:      {status['groundspeed']} m/s")

    # ── 3. Set mode ────────────────────────────────────────
    # NOTE: This only works if the RC mode switch is NOT active.
    # If MODE_CH is assigned, the RC switch overrides this immediately.
    # See BLUEOS-RESEARCH.md section 15 for details.

    print("\n=== Set Mode to MANUAL ===")
    r = client.post("/command/set_mode", json={"mode": "MANUAL"})
    print(f"  {r.json()['message']}")

    # ── 4. Arm ─────────────────────────────────────────────
    # WARNING: This will spin up the motor if throttle is non-zero.
    # Make sure the boat is in water or the prop is clear.

    print("\n=== Arm ===")
    r = client.post("/command/arm")
    print(f"  {r.json()['message']}")

    # Give ArduPilot a moment to arm
    time.sleep(1.0)

    # Confirm armed
    status = client.get("/status").json()
    if not status["armed"]:
        print("  Vehicle did not arm. Check pre-arm conditions.")
        print("  Common issues: no GPS fix, safety switch, RC not detected.")
        sys.exit(1)

    print(f"  Armed: {status['armed']}")

    # ── 5. Send manual control ─────────────────────────────
    # Gentle forward for 3 seconds, then stop.
    # y = steering (-1000..1000), z = throttle (-1000..1000)

    print("\n=== Manual Control: gentle forward for 3s ===")
    duration = 3.0
    start = time.time()
    count = 0
    while time.time() - start < duration:
        r = client.post("/command/manual_control", json={
            "steering": 0,    # straight
            "throttle": 200,  # ~20% forward
        })
        count += 1
        time.sleep(1.0 / COMMAND_RATE_HZ)

    print(f"  Sent {count} commands in {duration}s")

    # Stop
    print("\n=== Stop ===")
    client.post("/command/manual_control", json={"steering": 0, "throttle": 0})
    print("  Sent throttle=0 (stop)")

    # ── 6. Switch to GUIDED and send a heading command ─────
    print("\n=== Switch to GUIDED ===")
    r = client.post("/command/set_mode", json={"mode": "GUIDED"})
    print(f"  {r.json()['message']}")
    time.sleep(0.5)

    # Verify mode
    status = client.get("/status").json()
    if status["mode_name"] != "GUIDED":
        print(f"  Mode is {status['mode_name']}, not GUIDED.")
        print("  If the RC mode switch is active, the pilot's switch wins.")
    else:
        # Send heading command: go north at 1 m/s
        print("\n=== GUIDED: heading 0° (north) at 1 m/s ===")
        r = client.post("/command/guided/heading", json={
            "heading": 0,
            "speed": 1.0,
        })
        print(f"  {r.json()['message']}")

        # Keep sending for 5 seconds (GUIDED velocity/heading commands
        # must be re-sent or ArduPilot stops after ~3s)
        for _ in range(50):
            client.post("/command/guided/heading", json={
                "heading": 0,
                "speed": 1.0,
            })
            time.sleep(0.1)

        print("  Done — vehicle will hold position (GUIDED default)")

    # ── 7. Disarm ──────────────────────────────────────────
    print("\n=== Disarm ===")
    r = client.post("/command/disarm")
    print(f"  {r.json()['message']}")

    # ── 8. Final status ────────────────────────────────────
    print("\n=== Final Status ===")
    status = client.get("/status").json()
    print(f"  Mode:  {status['mode_name']}")
    print(f"  Armed: {status['armed']}")
    print()


if __name__ == "__main__":
    main()
