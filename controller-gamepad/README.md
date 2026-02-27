# controller-gamepad

Reads a USB gamepad and sends `manual_control` commands (steering + throttle) to the BlueOS connector REST API. Optionally publishes commanded values to the keelson/Zenoh bus.

## Architecture

```
USB Gamepad             controller-gamepad          BlueOS Connector
┌──────────────┐        ┌──────────────────────┐           ┌──────────────────┐
│  Logitech    │─USB──→ │  Read + Normalize    │─POST────→ │  /command/       │
│  F310        │ 20Hz   │  Deadzone filter     │  10Hz     │  manual_control  │
│  (X mode)    │        │                      │           │  {steering,      │
│              │        │  (optional)          │           │   throttle}      │
│              │        │  enclose() + publish │─pub─────→ │                  │
└──────────────┘        └──────────────────────┘           └──────────────────┘
                                                     Zenoh Bus (optional)
```

## Control Mapping

| Input        | Action   | Output Range  | Notes                         |
| ------------ | -------- | ------------- | ----------------------------- |
| Left stick Y | Throttle | -1000 to 1000 | Forward positive (Y inverted) |
| Left stick X | Steering | -1000 to 1000 | Right positive                |

## Supported Gamepads

| Gamepad       | `--gamepad` value | USB ID      | Notes                      |
| ------------- | ----------------- | ----------- | -------------------------- |
| Logitech F310 | `f310`            | `046d:c21d` | Must be in X (XInput) mode |

## Arguments

| Argument             | Required | Default | Description                                      |
| -------------------- | -------- | ------- | ------------------------------------------------ |
| `--blueos-url`       | yes      |         | BlueOS connector base URL                        |
| `--gamepad`          | no       | `f310`  | Gamepad type                                     |
| `--deadzone`         | no       | `0.05`  | Analog stick deadzone (0.0-1.0)                  |
| `--poll-interval`    | no       | `0.05`  | Seconds between gamepad reads                    |
| `--command-interval` | no       | `0.1`   | Min seconds between API posts                    |
| `--log-level`        | no       | `20`    | Python log level (10=DEBUG, 20=INFO, 30=WARNING) |
| `-r` / `--realm`     | no       |         | Keelson realm (enables Zenoh publishing)         |
| `-e` / `--entity-id` | no       |         | Entity ID                                        |
| `-s` / `--source-id` | no       |         | Source ID (e.g. `gamepad/0`)                     |
| `--connect`          | no       |         | Zenoh router endpoint                            |
| `--mode`             | no       |         | Zenoh session mode (`peer` / `client`)           |

## Docker

```bash
docker compose up --build
```

Edit `docker-compose.yml` to configure the BlueOS URL and optionally enable Zenoh publishing. The container runs with `privileged: true` and mounts `/dev/bus/usb` for USB gamepad access.

## Adding New Gamepad Types

1. Subclass `Gamepad` in `bin/gamepad.py`
2. Implement `open()`, `close()`, and `read()` (must return `GamepadState`)
3. Register the new class in `create_gamepad()` with a name string
4. Use `--gamepad <name>` to select it

## Safety

On gamepad disconnect (USB error), the controller immediately sends a neutral command (steering=0, throttle=0) to the BlueOS connector before attempting reconnection.
