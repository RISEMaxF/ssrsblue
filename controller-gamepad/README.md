# controller-gamepad

Reads a USB gamepad and publishes steering, throttle, and arm/disarm commands to the keelson/Zenoh bus. The keelson-connector-blueos subscribes to these and forwards them to the blueos-gateway REST API.

## Architecture

```
USB Gamepad        controller-gamepad              Zenoh Bus
┌──────────┐       ┌────────────────────┐       ┌──────────────────────┐
│ Logitech │─USB─→ │ Read + Normalize   │─pub─→ │ cmd_manual_control   │
│ F310     │ 20Hz  │ Deadzone filter    │ 10Hz  │ cmd_arm              │
│ (X mode) │       │ Arm/disarm toggle  │       │ rudder_angle_deg     │
│          │       │                    │       │ engine_throttle_pct  │
└──────────┘       └────────────────────┘       └──────────────────────┘
                                                         │
                                                         ▼
                                                keelson-connector-blueos
                                                         │
                                                         ▼
                                                  blueos-gateway
                                                  /command/manual_control
                                                  /command/arm
                                                  /command/disarm
```

## Control Mapping

| Input         | Action     | Output Range  | Notes                   |
| ------------- | ---------- | ------------- | ----------------------- |
| Left stick Y  | Throttle   | -1000 to 1000 | Forward positive        |
| Right stick X | Steering   | -1000 to 1000 | Right positive          |
| Start button  | Arm/Disarm | bool          | Toggle with 1s cooldown |

## Zenoh Topics

All topics follow the keelson key structure:

```
{realm}/@v0/{entity_id}/pubsub/{subject}/{source_id}
```

With the default config (`-r rise -e ssrs18 -s gamepad/0`), the gamepad publishes:

| Full key                                               | Subject               | Payload           | Description                                           |
| ------------------------------------------------------ | --------------------- | ----------------- | ----------------------------------------------------- |
| `rise/@v0/ssrs18/pubsub/cmd_manual_control/gamepad/0`  | `cmd_manual_control`  | TimestampedString | JSON `{"steering":N,"throttle":N}`, range -1000..1000 |
| `rise/@v0/ssrs18/pubsub/cmd_arm/gamepad/0`             | `cmd_arm`             | TimestampedBool   | `true` = arm, `false` = disarm                        |
| `rise/@v0/ssrs18/pubsub/rudder_angle_deg/gamepad/0`    | `rudder_angle_deg`    | TimestampedFloat  | Steering / 10 (telemetry)                             |
| `rise/@v0/ssrs18/pubsub/engine_throttle_pct/gamepad/0` | `engine_throttle_pct` | TimestampedFloat  | Throttle / 10 (telemetry)                             |

The keelson-connector-blueos subscribes to these with a wildcard source (`**`) and routes them:

- **`cmd_manual_control`** — fed into the CommandMux priority arbitration, then forwarded to the gateway at `--forward-interval`
- **`cmd_arm`** — bypasses the mux, forwarded directly to `/command/arm` or `/command/disarm`
- **`rudder_angle_deg`** / **`engine_throttle_pct`** — telemetry for recording, not consumed by the connector

All payloads are keelson-enveloped protobuf (the `enclose()` wrapper adds framing and a schema tag).

## Supported Gamepads

| Gamepad       | `--gamepad` value | USB ID      | Notes                      |
| ------------- | ----------------- | ----------- | -------------------------- |
| Logitech F310 | `f310`            | `046d:c21d` | Must be in X (XInput) mode |

## Arguments

| Argument             | Required | Default | Description                                      |
| -------------------- | -------- | ------- | ------------------------------------------------ |
| `-r` / `--realm`     | yes      |         | Keelson realm                                    |
| `-e` / `--entity-id` | yes      |         | Entity ID                                        |
| `-s` / `--source-id` | yes      |         | Source ID (e.g. `gamepad/0`)                     |
| `--connect`          | yes      |         | Zenoh router endpoint                            |
| `--gamepad`          | no       | `f310`  | Gamepad type                                     |
| `--deadzone`         | no       | `0.05`  | Analog stick deadzone (0.0-1.0)                  |
| `--poll-interval`    | no       | `0.05`  | Seconds between gamepad reads                    |
| `--mode`             | no       |         | Zenoh session mode (`peer` / `client`)           |
| `--log-level`        | no       | `20`    | Python log level (10=DEBUG, 20=INFO, 30=WARNING) |

## Deployment

### Prerequisites

All three services must be running on the NUC:

1. **Zenoh router** — `docker run --rm --net host eclipse/zenoh:1.7.2`
2. **blueos-gateway** — see `blueos-gateway/docker-compose.yml`
3. **keelson-connector-blueos** — see `keelson-connector-blueos/docker-compose.yml`

### Starting the gamepad controller (Docker, on the NUC)

```bash
cd controller-gamepad
docker compose up --build -d
```

The container runs with `privileged: true` and mounts `/dev/bus/usb` for raw USB access. The F310 must be plugged in before starting (it will retry on disconnect).

### Running on macOS

You can run the gamepad controller on a Mac with the F310 connected locally, publishing to a remote Zenoh router over an SSH tunnel.

**Install dependencies:**

```bash
brew install libusb
cd controller-gamepad
uv venv && uv pip install -r requirements.txt
```

**Set up an SSH tunnel** to forward the Zenoh router port from the remote NUC:

```bash
ssh -L 7447:localhost:7447 <nuc-host>
```

**Run the controller** (in a separate terminal):

```bash
DYLD_LIBRARY_PATH=$(brew --prefix libusb)/lib uv run python bin/main.py \
  -r rise -e ssrs18 -s gamepad/0 \
  --connect tcp/localhost:7447 \
  --gamepad f310 \
  --log-level 10
```

The `DYLD_LIBRARY_PATH` is needed because pyusb requires libusb but macOS doesn't add Homebrew libraries to the default search path.

### Verifying

Check logs to confirm the gamepad is detected and publishing:

```bash
docker compose logs -f
```

You should see:

- `Gamepad connected` — USB device found
- `throttle=X steering=Y` — debug output at `--log-level 10`
- `Published cmd_arm=True/False` — when Start is pressed

### Configuration

Edit `docker-compose.yml` to change realm, entity ID, Zenoh endpoint, etc. The defaults match the SSRS18 vessel setup:

```yaml
command: >-
  main.py
  -r rise -e ssrs18 -s gamepad/0
  --connect tcp/localhost:7447
```

## Adding New Gamepad Types

1. Subclass `Gamepad` in `bin/gamepad.py`
2. Implement `open()`, `close()`, and `read()` (must return `GamepadState`)
3. Register the new class in `create_gamepad()` with a name string
4. Use `--gamepad <name>` to select it

## Safety

- On gamepad disconnect (USB error), the controller immediately publishes a neutral command (steering=0, throttle=0) before attempting reconnection.
- Arm/disarm has a 1-second cooldown to prevent accidental double-toggles.
- The keelson-connector-blueos CommandMux applies priority and timeout — if the gamepad stops publishing for 0.5s, it falls through to lower-priority sources.
- The blueos-gateway watchdog sends neutral if no commands arrive for 2s while armed.
- The RC controller always has physical override via ArduRover's mode channel.
