# ssrsblue

Software stack for the SSRS autonomous surface vessel. Three containers run on a companion NUC connected to a Raspberry Pi running BlueOS + ArduRover.

## System overview

```mermaid
graph LR
    subgraph Vessel
        RC[RC Controller<br/>ELRS]
        GPS[USB GPS<br/>Receiver]
        Gamepad[USB Gamepad<br/>Logitech F310]

        subgraph RPi [Raspberry Pi · 192.168.2.2]
            BlueOS[BlueOS<br/>ArduRover]
        end

        subgraph NUC [Companion NUC]
            Conn[connector<br/>REST API :8080]
            KC[keelson-connector-blueos<br/>Telemetry + CommandMux]
            GC[controller-gamepad<br/>Gamepad → Zenoh]
        end

        RC --->|PWM| BlueOS
        GPS -.->|serial| Conn
        Gamepad -->|USB| GC
    end

    subgraph Zenoh [Zenoh Bus :7447]
        Telemetry[heading, speed, battery,<br/>location, GPS, ...]
        Commands[cmd_manual_control,<br/>cmd_set_mode, cmd_arm]
    end

    subgraph ROC [Remote Operations Center]
        RemoteUI[Remote UI]
        Autonomy[Autonomy]
    end

    subgraph Consumers
        MCAP[MCAP Recording]
        Fox[Foxglove Studio]
        Svc[Other Services]
    end

    Conn <-->|MAVLink via<br/>HTTP + WebSocket| BlueOS
    KC -->|GET /status<br/>GET /gps| Conn
    KC -->|POST commands| Conn
    KC -->|publish telemetry| Telemetry
    Commands -.->|commands| KC
    GC -->|publish| Commands
    RemoteUI -->|publish| Commands
    Autonomy -->|publish| Commands
    Telemetry --> MCAP
    Telemetry --> Fox
    Telemetry --> Svc
```

## Data flow

```mermaid
graph TD
    subgraph Telemetry ["Telemetry (read path)"]
        AP[ArduRover] -->|MAVLink WS| C1[connector]
        NMEA[USB GPS] -.->|serial| C1
        C1 -->|status + GPS data| KC1[keelson-connector-blueos]
        KC1 -->|protobuf + enclose| Z1[Zenoh subjects]
    end

    subgraph Control ["Control (write path — via Zenoh CommandMux)"]
        F310[Gamepad F310] -->|raw USB| GC1[controller-gamepad]
        RemUI[Remote UI] -->|Zenoh| Z2[cmd_manual_control/*]
        Auto[Autonomy] -->|Zenoh| Z2
        GC1 -->|Zenoh| Z2
        Z2 -->|subscribe| MUX[CommandMux<br/>in keelson-connector-blueos]
        MUX -->|highest-priority<br/>active source| C2[connector]
        C2 -->|MAVLink HTTP| AP2[ArduRover]
    end
```

## Command arbitration (CommandMux)

All command sources publish to Zenoh and the vessel-side `CommandMux` (in keelson-connector-blueos) selects the highest-priority active source. Inspired by the ROS `twist_mux` pattern.

| Source        | Priority | Timeout  | Description                   |
| ------------- | -------- | -------- | ----------------------------- |
| `e_stop`      | 255      | 0 (lock) | Emergency stop — stays active |
| `gamepad/*`   | 100      | 0.5s     | Manual gamepad control        |
| `autonomy/*`  | 50       | 2.0s     | Autonomous navigation         |
| `remote_ui/*` | 10       | 1.0s     | Remote operator UI            |

When a source stops publishing and its timeout expires, the mux falls through to the next lower-priority active source. If all sources time out, neutral (0,0) is sent.

Mode changes (`cmd_set_mode`) and arm/disarm (`cmd_arm`) bypass priority — any source can send them directly.

### Command subjects

| Subject                 | Type                | Description                                   |
| ----------------------- | ------------------- | --------------------------------------------- |
| `cmd_manual_control`    | `TimestampedString` | JSON `{"steering": float, "throttle": float}` |
| `cmd_set_mode`          | `TimestampedString` | Mode name (MANUAL, GUIDED, HOLD, etc.)        |
| `cmd_arm`               | `TimestampedBool`   | true=arm, false=disarm                        |
| `cmd_active_source`     | `TimestampedString` | Force a specific source (manual override)     |
| `active_command_source` | `TimestampedString` | Published by mux: current active source       |

## Containers

| Container                    | Purpose                                                 | Port    | Source                                                   |
| ---------------------------- | ------------------------------------------------------- | ------- | -------------------------------------------------------- |
| **connector**                | REST API gateway to ArduRover via BlueOS MAVLink2REST   | `:8080` | [`connector/`](connector/)                               |
| **keelson-connector-blueos** | Telemetry publisher + command mux (subscribe → forward) | —       | [`keelson-connector-blueos/`](keelson-connector-blueos/) |
| **controller-gamepad**       | Reads USB gamepad, publishes commands to Zenoh          | —       | [`controller-gamepad/`](controller-gamepad/)             |

## Quick start

```bash
# Start everything
docker compose -f connector/docker-compose.yml up -d
docker compose -f keelson-connector-blueos/docker-compose.yml up -d
docker compose -f controller-gamepad/docker-compose.yml up -d

# Or run locally for development
cd connector && uv sync && uv run uvicorn connector.main:app --port 8080
cd keelson-connector-blueos && pip install -r requirements.txt && python bin/main.py -r rise -e ssrs18 -s blueos/0 --blueos-url http://localhost:8080 --connect tcp/localhost:7447
cd controller-gamepad && pip install -r requirements.txt && python bin/main.py -r rise -e ssrs18 -s gamepad/0 --connect tcp/localhost:7447
```

## Keelson subjects published

### From `/status` → `{source}/autopilot`

| Subject                       | Type                   | Description                |
| ----------------------------- | ---------------------- | -------------------------- |
| `vehicle_mode`                | `TimestampedString`    | MANUAL, GUIDED, HOLD, etc. |
| `vehicle_armed`               | `TimestampedBool`      | Arm state                  |
| `heading_true_north_deg`      | `TimestampedFloat`     | Compass heading            |
| `speed_over_ground_knots`     | `TimestampedFloat`     | Converted from m/s         |
| `location_fix`                | `foxglove.LocationFix` | Autopilot position         |
| `gps_fix_type`                | `TimestampedInt`       | 0=none, 3=3D, 5=RTK        |
| `battery_voltage_v`           | `TimestampedFloat`     |                            |
| `battery_current_a`           | `TimestampedFloat`     |                            |
| `battery_state_of_charge_pct` | `TimestampedFloat`     | 0–100                      |
| `autopilot_throttle_pct`      | `TimestampedFloat`     | Actual output              |
| `rudder_angle_deg`            | `TimestampedFloat`     | Last commanded steering    |
| `engine_throttle_pct`         | `TimestampedFloat`     | Last commanded throttle    |

### From `/gps` → `{source}/gps`

| Subject                        | Type                   | Description      |
| ------------------------------ | ---------------------- | ---------------- |
| `location_fix`                 | `foxglove.LocationFix` | WGS84 + altitude |
| `location_fix_satellites_used` | `TimestampedInt`       |                  |
| `location_fix_hdop`            | `TimestampedFloat`     |                  |
| `speed_over_ground_knots`      | `TimestampedFloat`     | From NMEA RMC    |
| `course_over_ground_deg`       | `TimestampedFloat`     | From NMEA RMC    |
| `altitude_above_msl_m`         | `TimestampedFloat`     | From NMEA GGA    |

## Network

```
NUC ──ethernet── Raspberry Pi (192.168.2.2)
                  └── BlueOS DHCP → NUC gets 192.168.2.x
```

BlueOS exposes MAVLink2REST at `http://192.168.2.2/mavlink2rest`. The connector talks to it over HTTP (commands) and WebSocket (telemetry).

## Safety

- **Command mux timeout**: if a command source stops publishing, the mux falls through to lower-priority sources; if all time out, neutral (0,0) is sent
- **Gamepad disconnect**: controller publishes neutral immediately, then mux timeout provides backup
- **Pilot override**: RC controller always wins via ArduRover's mode channel
- **GCS failsafe**: if connector dies, ArduPilot triggers failsafe after 5s (no heartbeats)
- **Watchdog**: connector sends neutral steering+throttle if no commands arrive for 2s while armed
