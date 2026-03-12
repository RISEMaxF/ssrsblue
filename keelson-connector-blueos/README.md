# keelson-connector-blueos

Bridges the [blueos-gateway](../blueos-gateway/) REST API onto the keelson/Zenoh bus using standard keelson subjects and protobuf types. Does not talk to BlueOS/ArduPilot directly — it polls the gateway's HTTP endpoints. This makes ArduPilot telemetry and GPS data available to the rest of the RISE Maritime ecosystem — MCAP recording, Foxglove visualization, and other keelson services.

## Architecture

```
blueos-gateway (REST)            keelson-connector-blueos             Zenoh Bus
┌──────────────────────┐         ┌─────────────────────┐         ┌──────────────────┐
│  GET /status         │─poll──→ │  Parse + Protobuf   │─pub───→ │  .../autopilot   │
│  (ArduPilot state)   │  1Hz    │  enclose()          │         │  heading, speed,  │
│                      │         │                     │         │  battery, etc.    │
│  GET /gps            │─poll──→ │  Parse + Protobuf   │─pub───→ │  .../gps         │
│  (raw NMEA GPS)      │  1Hz    │  enclose()          │         │  location, sats,  │
│  (may be disabled)   │         │                     │         │  hdop, course     │
└──────────────────────┘         └─────────────────────┘         └──────────────────┘
```

Follows the standard `keelson-connector-*` pattern: synchronous poll loop, lazy publisher caching, `keelson.enclose()` + protobuf serialization.

## Zenoh Key Structure

All keys follow the keelson convention:

```
{realm}/@v0/{entity_id}/pubsub/{subject}/{source_id}
```

Two source IDs are published. The base `--source-id` (e.g. `blueos/0`) gets suffixed with `/autopilot` and `/gps` so consumers can distinguish ArduPilot's filtered telemetry from the raw GPS receiver. The connector also publishes mux status under `{source_id}/mux`.

All payloads are keelson-enveloped protobuf (the `enclose()` wrapper adds framing and a schema tag).

## Telemetry (Published)

### From `GET /status` → source `{source_id}/autopilot`

| API Field               | Keelson Subject               | Protobuf Type          | Notes                                 |
| ----------------------- | ----------------------------- | ---------------------- | ------------------------------------- |
| `mode_name`             | `vehicle_mode`                | `TimestampedString`    | MANUAL, GUIDED, HOLD, etc.            |
| `armed`                 | `vehicle_armed`               | `TimestampedBool`      | Safety-critical                       |
| `heading`               | `heading_true_north_deg`      | `TimestampedFloat`     | ArduPilot compass heading             |
| `groundspeed`           | `speed_over_ground_knots`     | `TimestampedFloat`     | Converted from m/s (x 1.94384)        |
| `lat`, `lon`            | `location_fix`                | `foxglove.LocationFix` | Autopilot position (always available) |
| `gps_fix_type`          | `gps_fix_type`                | `TimestampedInt`       | 0=none, 2=2D, 3=3D, 4=DGPS, 5=RTK     |
| `battery_voltage`       | `battery_voltage_v`           | `TimestampedFloat`     |                                       |
| `battery_current`       | `battery_current_a`           | `TimestampedFloat`     |                                       |
| `battery_remaining`     | `battery_state_of_charge_pct` | `TimestampedFloat`     | 0-100                                 |
| `throttle`              | `autopilot_throttle_pct`      | `TimestampedFloat`     | Actual output (vs commanded)          |
| `last_command_steering` | `rudder_angle_deg`            | `TimestampedFloat`     | Only when commands have been sent     |
| `last_command_throttle` | `engine_throttle_pct`         | `TimestampedFloat`     | Only when commands have been sent     |

### From `GET /gps` → source `{source_id}/gps`

| API Field                | Keelson Subject                | Protobuf Type          | Notes                     |
| ------------------------ | ------------------------------ | ---------------------- | ------------------------- |
| `lat`, `lon`, `altitude` | `location_fix`                 | `foxglove.LocationFix` | WGS84, altitude in meters |
| `satellites`             | `location_fix_satellites_used` | `TimestampedInt`       |                           |
| `hdop`                   | `location_fix_hdop`            | `TimestampedFloat`     |                           |
| `speed_knots`            | `speed_over_ground_knots`      | `TimestampedFloat`     | From NMEA RMC             |
| `course`                 | `course_over_ground_deg`       | `TimestampedFloat`     | From NMEA RMC             |
| `altitude`               | `altitude_above_msl_m`         | `TimestampedFloat`     | From NMEA GGA             |

GPS subjects are only published when the GPS reader is enabled and has a fix (`fix_quality > 0`). If `/gps` returns 404 (GPS reader disabled), it is silently skipped.

## Commands (Subscribed)

The connector subscribes to command subjects with a wildcard source (`**`) so it receives commands from any source (gamepad, autonomy, UI, etc.).

| Subject              | Protobuf Type       | Handling                                                       |
| -------------------- | ------------------- | -------------------------------------------------------------- |
| `cmd_manual_control` | `TimestampedString` | JSON `{"steering":N,"throttle":N}`, fed into CommandMux        |
| `cmd_arm`            | `TimestampedBool`   | `true` → POST `/command/arm`, `false` → POST `/command/disarm` |
| `cmd_set_mode`       | `TimestampedString` | Mode name/number → POST `/command/set_mode`                    |
| `cmd_active_source`  | `TimestampedString` | Manual override of active mux source                           |

The CommandMux applies priority-based arbitration to `cmd_manual_control` — the source ID from the key (e.g. `gamepad/0`) determines priority. Higher-priority sources preempt lower ones. If a source stops publishing, it times out (default 0.5s for gamepad) and falls through.

The mux publishes the currently active source to `active_command_source/{source_id}/mux`.

`cmd_arm` and `cmd_set_mode` bypass the mux and are queued for immediate forwarding.

### Example Zenoh Key Paths

With `-r rise -e ssrs18 -s blueos/0`:

```
rise/@v0/ssrs18/pubsub/heading_true_north_deg/blueos/0/autopilot
rise/@v0/ssrs18/pubsub/speed_over_ground_knots/blueos/0/autopilot
rise/@v0/ssrs18/pubsub/battery_voltage_v/blueos/0/autopilot
rise/@v0/ssrs18/pubsub/location_fix/blueos/0/gps
rise/@v0/ssrs18/pubsub/location_fix_satellites_used/blueos/0/gps
```

## Usage

```bash
python bin/main.py \
  -r rise -e ssrs18 -s blueos/0 \
  --blueos-url http://localhost:8080 \
  --poll-interval 1.0 \
  --connect tcp/localhost:7447 \
  --log-level 20
```

### Arguments

| Argument             | Required | Default | Description                                            |
| -------------------- | -------- | ------- | ------------------------------------------------------ |
| `-r` / `--realm`     | yes      |         | Keelson realm                                          |
| `-e` / `--entity-id` | yes      |         | Entity (vessel) ID                                     |
| `-s` / `--source-id` | yes      |         | Base source ID (suffixed with `/autopilot` and `/gps`) |
| `--blueos-url`       | yes      |         | blueos-gateway base URL                                |
| `--poll-interval`    | no       | `1.0`   | Seconds between telemetry polls                        |
| `--command-timeout`  | no       | `2.0`   | Default source timeout for command mux (seconds)       |
| `--forward-interval` | no       | `0.1`   | Interval for forwarding commands to gateway (seconds)  |
| `--connect`          | no       |         | Zenoh router endpoint(s)                               |
| `--mode`             | no       |         | Zenoh session mode (`peer` / `client`)                 |
| `--log-level`        | no       | `20`    | Python log level (`10`=DEBUG, `20`=INFO, `30`=WARNING) |

## Docker

```bash
docker compose up --build
```

Edit `docker-compose.yml` to configure realm, entity, source ID, BlueOS URL, and Zenoh router endpoint for your deployment.

## Verification

1. Start the blueos-gateway (or a mock returning JSON on `/status` and `/gps`)
2. Start a Zenoh router:
   ```bash
   docker run --rm -p 7447:7447 eclipse/zenoh:1.7.2
   ```
3. Run the connector:
   ```bash
   python bin/main.py -r test -e test -s blueos/0 \
     --blueos-url http://localhost:8080 \
     --connect tcp/localhost:7447 \
     --log-level 10
   ```
4. Subscribe to verify publications arrive:
   ```bash
   z_sub -k "test/@v0/test/pubsub/**"
   ```
5. Confirm GPS subjects are absent when `/gps` returns 404, and appear when GPS has a fix.
