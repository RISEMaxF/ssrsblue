# BlueOS Research: Programmatic Vehicle Control

> Platform: BlueOS 1.4.3 on Raspberry Pi 4 + Navigator Hat
> Vehicle: **Surface vessel (boat)** -- ArduRover with FRAME_CLASS=2 (Boat)
> Companion: NUC connected via **ethernet** cable to Pi
> RC: ELRS controller via CRSF serial
> Latest stable: **1.4.3** (Jan 23, 2025) | Beta track: **1.5.0-beta.29** (Feb 5, 2025)
> Research date: 2025-02-12

---

## Table of Contents

1. [What BlueOS Is](#1-what-blueos-is)
2. [Architecture & Services](#2-architecture--services)
3. [Navigator Hardware](#3-navigator-hardware)
4. [Control Methods Compared](#4-control-methods-compared)
5. [MAVLink2REST (Recommended Start)](#5-mavlink2rest-recommended-start)
6. [pymavlink (UDP)](#6-pymavlink-udp)
7. [Direct PWM via navigator-lib](#7-direct-pwm-via-navigator-lib)
8. [BlueOS Extensions](#8-blueos-extensions)
9. [MAVROS / ROS Integration](#9-mavros--ros-integration)
10. [BlueOS REST API Reference](#10-blueos-rest-api-reference)
11. [Key ArduRover Parameters](#11-key-ardurover-parameters)
12. [Repos, Papers & Community](#12-repos-papers--community)
13. [Recommendations](#13-recommendations)

---

## 1. What BlueOS Is

BlueOS is Blue Robotics' open-source onboard computer OS for marine vehicles (ROVs, AUVs, USVs). It is the **companion computer software** -- it does NOT run the flight controller firmware itself. Instead, it manages ArduPilot (ArduSub/ArduRover/etc.), video streams, network, and extensions.

**Key facts:**
- Successor to the legacy "Companion" software
- Docker-based microservice architecture
- Web UI at `http://192.168.2.2` (or `http://blueos.local`)
- ArduPilot runs as a **Linux process** on the Pi (not on a separate MCU -- the Navigator is a passive HAT)
- All services expose REST APIs with Swagger/OpenAPI documentation
- Extensible via Docker container extensions

**Version 1.4.3** (your version) includes:
- Safe Mode to prevent unsafe config while armed
- Improved `register_service` for cross-extension communication
- MAVLink Server as a routing alternative with all-endpoint logging
- Basic Zenoh communication support
- SSH and system limits startup configuration
- Backup DHCP server for ethernet interface

---

## 2. Architecture & Services

```
[QGroundControl / Custom App (Topside)]
        |
    UDP/MAVLink over Ethernet tether
        |
[BlueOS on Raspberry Pi 4] ── http://192.168.2.2
    |── Nginx (reverse proxy, port 80)
    |── MAVLink Router/Server (routes MAVLink)
    |── MAVLink2REST (REST/WebSocket bridge, port 6040)
    |── Autopilot Manager (port 8000)
    |── Video Manager (port 6020)
    |── Cable Guy - network (port 9090)
    |── WiFi Manager (port 9000)
    |── Kraken - extensions (port 9134)
    |── System Information (port 6030)
    |── File Browser (port 7777)
    |── Terminal/ttyd (port 8088)
    |── Commander, Beacon, NMEA Injector, Ping...
        |
    SPI/I2C (hardware)
        |
[Navigator HAT → ArduPilot process]
    |── PCA9685 PWM → ESCs/Thrusters/Servos
    |── IMUs, Magnetometer, Barometer, ADC
```

### Service Endpoints

| Service | Port | Base URL |
|---------|------|----------|
| Autopilot Manager | 8000 | `/ardupilot-manager/v1.0` |
| MAVLink2REST | 6040 | `/mavlink2rest` |
| Video Manager | 6020 | `/mavlink-camera-manager` |
| Cable Guy (network) | 9090 | `/cable-guy/v1.0` |
| WiFi Manager | 9000 | `/wifi-manager/v1.0` |
| System Info | 6030 | `/system-information` |
| Extensions (Kraken) | 9134 | `/kraken/v1.0` |
| File Browser | 7777 | `/file-browser` |
| Terminal | 8088 | direct |
| Version Chooser | 8081 | `/version-chooser/v1.0` |
| Commander | - | `/commander/v1.0` |
| Bag of Holding | 9101 | DB modification utility |

Every service has **interactive Swagger docs** at `http://192.168.2.2/{service-path}/docs`.

### Pirate Mode

Enable via the skull icon in the header. Reveals advanced/development features: raw parameter editing, service URLs, dev tools, MJPG/YUYV stream support. Required for some API operations.

### MAVLink Routing (v1.4+)

Three backend options:
1. **MAVLinkRouter** (default) -- standard routing
2. **MAVP2P** -- higher CPU, recommended if you get "GCS Heartbeat Lost" errors
3. **MAVLinkServer** (new in 1.4) -- all-endpoint logging, WebSocket support, MAVLink2REST-compatible API

---

## 3. Navigator Hardware

The Navigator is a **passive HAT** with no onboard MCU or firmware. ArduPilot runs as a Linux process on the Pi using the `navigator` HAL backend.

### Specifications

| Feature | Detail |
|---------|--------|
| PWM outputs | **16 channels**, PCA9685 driver, 1 us resolution at 250 Hz |
| PWM accuracy | +/-0.5 us at 50, 100, 200, 250, 400, 500 Hz |
| Signal voltage | 3.3V, 15 mA max per channel |
| IMU | ICM-20602 (6-axis accel + gyro) on SPI |
| Magnetometers | MMC5983 + AK09915 (dual 3-axis compass) |
| Barometer | BMP280 (300-1100 mbar, -900 to 9000m altitude) |
| ADC | ADS1115 16-bit (14.69 bits effective), 2 ports (3.3V + 6.6V input) |
| Serial | 4x UART (up to 3 MBd, no flow control) |
| I2C | 2 ports (10 kHz - 1 MHz), path `/dev/i2c-6` |
| RC Input | SBUS, Crossfire, IBUS |
| Leak detection | 2 probes with 5V pull-up |
| LEDs | RGB NeoPixel |
| Power | Dual inputs with auto-switching, 4A continuous to Pi |

### I2C/SPI Device Map

```
Raspberry Pi 4
  |
  |── I2C Bus ──┬── PCA9685 (0x40)   → 16x PWM → ESCs/Thrusters/Servos
  |             ├── ADS1115 (0x48)   → 4x Analog (battery V/I)
  |             ├── BMP280/390 (0x77) → Pressure/Temp
  |             └── MMC5983 (0x30)   → Magnetometer
  |
  |── SPI Bus 0 ┬── CE0 → ICM-20602  → Primary IMU (accel+gyro)
  |             └── CE1 → Secondary IMU → Redundant (accel+gyro)
  |
  |── GPIO 18 → NeoPixel LEDs
  |── GPIO 26 → Leak Detection
```

### PWM Channel Mapping (typical)

| PCA9685 Ch | ArduPilot SERVO# | Typical Use |
|------------|-----------------|-------------|
| 0 | SERVO1 | Thruster 1 |
| 1 | SERVO2 | Thruster 2 |
| 2 | SERVO3 | Thruster 3 |
| 3 | SERVO4 | Thruster 4 |
| 4 | SERVO5 | Thruster 5 |
| 5 | SERVO6 | Thruster 6 |
| 6-7 | SERVO7-8 | Thruster 7-8 (heavy config) |
| 8 | SERVO9 | Lights |
| 9 | SERVO10 | Camera tilt servo |
| 10-15 | SERVO11-16 | Gripper / Aux |

Channel-to-function mapping is configurable via `SERVOn_FUNCTION` ArduPilot parameters.

---

## 4. Control Methods Compared

| Method | Latency | Setup | ArduPilot Features | Best For |
|--------|---------|-------|-------------------|----------|
| **MAVLink2REST (HTTP)** | ~10-50ms | Trivial (just HTTP) | Full | Quick scripting, any language |
| **MAVLink2REST (WebSocket)** | ~5-15ms | Easy | Full | Web UIs, real-time dashboards |
| **pymavlink (UDP)** | ~1-5ms | Medium | Full | Low-latency autonomous control |
| **Direct PWM (navigator-lib)** | ~1ms | Easy (stop ArduPilot) | None | Custom vehicles, raw control |
| **MAVROS/ROS** | ~5-20ms | Hard (full ROS stack) | Full | ROS-based research |
| **MAVSDK** | ~5-10ms | Medium | Limited (PX4-focused) | Not recommended for ArduPilot |

### Which method keeps ArduPilot features?

| Feature | MAVLink methods | Direct PWM |
|---------|----------------|------------|
| Stabilization / depth hold | Yes | No (must implement) |
| Sensor fusion / EKF | Yes | No (raw sensors only) |
| Failsafes | Yes | None |
| Mission planning | Yes | No |
| Parameter system | Yes | No |

**Key insight**: MAVLink-based methods (REST, pymavlink, MAVROS) send commands *through* ArduPilot, which handles motor mixing, stabilization, and safety. Direct PWM bypasses all of that -- you get raw hardware control but must implement everything yourself.

---

## 5. MAVLink2REST (Recommended Start)

The easiest path. No dependencies beyond HTTP. Works from any language, any machine on the network.

### Reading Data

```python
import requests

BLUEOS = "http://192.168.2.2"

# Get all MAVLink messages
msgs = requests.get(f"{BLUEOS}/mavlink2rest/mavlink").json()

# Get specific message
attitude = requests.get(f"{BLUEOS}/mavlink2rest/mavlink/ATTITUDE").json()
print(f"Roll: {attitude['message']['roll']}")

# Get by vehicle/component
msgs = requests.get(
    f"{BLUEOS}/mavlink2rest/mavlink/vehicles/1/components/1/messages"
).json()
```

### Sending Commands

```python
import requests
import time

BLUEOS = "http://192.168.2.2"
MAVLINK = f"{BLUEOS}/mavlink2rest/mavlink"

def send_mavlink(message):
    """Send a MAVLink message via REST."""
    payload = {
        "header": {"system_id": 255, "component_id": 0, "sequence": 0},
        "message": message
    }
    return requests.post(MAVLINK, json=payload, timeout=1.0)

# --- Arm ---
send_mavlink({
    "type": "COMMAND_LONG",
    "target_system": 1,
    "target_component": 1,
    "command": {"type": "MAV_CMD_COMPONENT_ARM_DISARM"},
    "confirmation": 0,
    "param1": 1.0,  # 1=arm, 0=disarm
    "param2": 0.0, "param3": 0.0, "param4": 0.0,
    "param5": 0.0, "param6": 0.0, "param7": 0.0
})

# --- Set Mode (ArduRover modes) ---
# MANUAL=0, ACRO=1, STEERING=3, HOLD=4, LOITER=5, AUTO=10, RTL=11, GUIDED=15
send_mavlink({
    "type": "SET_MODE",
    "target_system": 1,
    "base_mode": {"bits": 209},  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | ARMED
    "custom_mode": 0  # MANUAL
})

# --- MANUAL_CONTROL (joystick-style, ArduRover) ---
# For Rover: y → steering (-1000 to 1000), z → throttle (-1000..1000, 0=stop)
# x and r are ignored by ArduRover for standard boat frames
send_mavlink({
    "type": "MANUAL_CONTROL",
    "target": 1,
    "x": 0,     # ignored by ArduRover
    "y": 0,     # steering (0 = straight)
    "z": 400,   # throttle (0=stop, >0=forward, <0=reverse)
    "r": 0,     # ignored by ArduRover
    "buttons": 0
})

# --- RC_CHANNELS_OVERRIDE (per-channel PWM, ArduRover) ---
# Channel 1 = Steering (SERVO1_FUNCTION=26), Channel 3 = Throttle (SERVO3_FUNCTION=70)
# Values: 1100-1900 us (1500 = neutral), 0 or 65535 = no override
send_mavlink({
    "type": "RC_CHANNELS_OVERRIDE",
    "target_system": 1,
    "target_component": 1,
    "chan1_raw": 1500, "chan2_raw": 0,     # ch1=steering center, ch2=pass-through
    "chan3_raw": 1700,  # ch3=throttle forward
    "chan4_raw": 0, "chan5_raw": 0,
    "chan6_raw": 0, "chan7_raw": 0,
    "chan8_raw": 0,
    "chan9_raw": 0, "chan10_raw": 0,
    "chan11_raw": 0, "chan12_raw": 0,
    "chan13_raw": 0, "chan14_raw": 0,
    "chan15_raw": 0, "chan16_raw": 0,
    "chan17_raw": 0, "chan18_raw": 0,
})

# --- DO_SET_SERVO (set individual servo output) ---
send_mavlink({
    "type": "COMMAND_LONG",
    "target_system": 1,
    "target_component": 1,
    "command": {"type": "MAV_CMD_DO_SET_SERVO"},
    "confirmation": 0,
    "param1": 9.0,     # servo number (SERVO9 = lights)
    "param2": 1800.0,  # PWM value
    "param3": 0.0, "param4": 0.0,
    "param5": 0.0, "param6": 0.0, "param7": 0.0
})
```

### WebSocket (Real-Time Bidirectional)

```python
import asyncio
import json
import websockets

async def control_loop():
    uri = "ws://192.168.2.2/mavlink2rest/ws/mavlink"
    async with websockets.connect(uri) as ws:

        async def sender():
            """Send MANUAL_CONTROL at 20 Hz."""
            while True:
                msg = {
                    "header": {"system_id": 255, "component_id": 0, "sequence": 0},
                    "message": {
                        "type": "MANUAL_CONTROL",
                        "target": 1,
                        "x": 0, "y": 0, "z": 300, "r": 0, "buttons": 0  # gentle forward (y=steer, z=throttle)
                    }
                }
                await ws.send(json.dumps(msg))
                await asyncio.sleep(0.05)

        async def receiver():
            """Print attitude data as it arrives."""
            async for message in ws:
                data = json.loads(message)
                if data.get("message", {}).get("type") == "ATTITUDE":
                    att = data["message"]
                    print(f"Roll: {att['roll']:.2f}, Pitch: {att['pitch']:.2f}, Yaw: {att['yaw']:.2f}")

        await asyncio.gather(sender(), receiver())

asyncio.run(control_loop())
```

### JavaScript WebSocket (Browser)

```javascript
const ws = new WebSocket('ws://192.168.2.2/mavlink2rest/ws/mavlink');

ws.onopen = () => {
    // Send MANUAL_CONTROL at 20 Hz
    setInterval(() => {
        ws.send(JSON.stringify({
            header: { system_id: 255, component_id: 0, sequence: 0 },
            message: {
                type: "MANUAL_CONTROL",
                target: 1, x: 0, y: 0, z: 0, r: 0, buttons: 0  // z=0 = stop (y=steer, z=throttle)
            }
        }));
    }, 50);
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    if (data.message?.type === 'ATTITUDE') {
        console.log('Attitude:', data.message);
    }
};
```

---

## 6. pymavlink (UDP)

Lower latency than REST. Best for serious autonomous control from a topside computer.

### Setup

```bash
pip install pymavlink
```

You need a dedicated MAVLink endpoint in BlueOS:
1. Open BlueOS web UI → MAVLink Endpoints
2. Add endpoint: UDP Client, `192.168.2.1:14551` (or your topside IP)
3. Connect your script to `udpin:0.0.0.0:14551`

### Complete Example

```python
from pymavlink import mavutil
import time
import threading

# Connect
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')
master.wait_heartbeat()
print(f"Connected: sys={master.target_system} comp={master.target_component}")

# CRITICAL: Send heartbeats to prevent GCS failsafe
def heartbeat_loop():
    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(1)

threading.Thread(target=heartbeat_loop, daemon=True).start()

# Set MANUAL mode (ArduRover: MANUAL=0, STEERING=3, HOLD=4, GUIDED=15)
master.set_mode_apm(0)  # MANUAL=0

# Arm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
master.motors_armed_wait()
print("Armed")

# Send MANUAL_CONTROL at 20 Hz for 3 seconds
# ArduRover: y=steering (-1000..1000), z=throttle (-1000..1000, 0=stop)
start = time.time()
while time.time() - start < 3.0:
    master.mav.manual_control_send(
        master.target_system,
        0,     # x: ignored by ArduRover
        0,     # y: steering (0 = straight)
        300,   # z: throttle (0=stop, 300=~30% forward)
        0,     # r: ignored by ArduRover
        0      # buttons
    )
    time.sleep(0.05)

# RC_CHANNELS_OVERRIDE alternative (ArduRover: ch1=steering, ch3=throttle)
rc = [0] * 18  # 0 = pass-through to physical RC
rc[0] = 1500       # channel 1 = steering center
rc[2] = 1700       # channel 3 = throttle forward
master.mav.rc_channels_override_send(
    master.target_system, master.target_component, *rc
)

# Read attitude
msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
if msg:
    print(f"Roll={msg.roll:.2f} Pitch={msg.pitch:.2f} Yaw={msg.yaw:.2f}")

# Disarm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Disarmed")
```

---

## 7. Direct PWM via navigator-lib

Bypasses ArduPilot entirely. You get raw hardware control but lose stabilization, failsafes, and sensor fusion.

### When to use this

- Custom vehicle frames where ArduPilot frame configs don't apply
- Direct actuator testing on the bench
- Building your own control system from scratch
- Simple vehicles (surface boat with rudder + single thruster)

### Prerequisites

**You MUST stop ArduPilot first.** Running both simultaneously causes I2C bus conflicts:

```python
import requests
# Stop ArduPilot via BlueOS API
requests.post("http://192.168.2.2/ardupilot-manager/v1.0/stop")
# ... do direct PWM control ...
# Restart when done
requests.post("http://192.168.2.2/ardupilot-manager/v1.0/start")
```

### Control Example

```python
import bluerobotics_navigator as navigator
import time

navigator.init()

# Configure for standard ESC frequency
navigator.set_pwm_freq_hz(50)
navigator.set_pwm_enable(True)

# Helper: convert microseconds to PCA9685 12-bit value at 50Hz
def us_to_pwm(microseconds):
    """At 50Hz, period=20ms=20000us, 12-bit=4096 steps."""
    return int(microseconds / 20000.0 * 4096)

# Send neutral (1500us) to all channels first (ESC arming sequence)
neutral = us_to_pwm(1500)  # ~307
for ch in range(16):
    navigator.set_pwm_channel_value(ch, neutral)
time.sleep(3)  # Wait for ESCs to arm

# Thruster forward at ~60%
navigator.set_pwm_channel_value(0, us_to_pwm(1700))
time.sleep(2)

# Rudder to 30 degrees (example)
navigator.set_pwm_channel_value(2, us_to_pwm(1650))
time.sleep(1)

# Stop everything
for ch in range(16):
    navigator.set_pwm_channel_value(ch, neutral)
navigator.set_pwm_enable(False)
```

### Reading Sensors (without ArduPilot)

```python
import bluerobotics_navigator as navigator

navigator.init()

# IMU
accel = navigator.read_accel()     # [x, y, z]
gyro = navigator.read_gyro()       # [x, y, z]
mag = navigator.read_mag()         # [x, y, z]

# Environment
pressure = navigator.read_pressure()      # mbar
temperature = navigator.read_temperature() # Celsius

# ADC (battery monitoring etc.)
adc = navigator.read_adc_all()     # 4 channels

# LEDs
navigator.set_neopixel([[0, 255, 0]])  # Green
```

---

## 8. BlueOS Extensions

Package your control logic as a Docker container that runs persistently on the vehicle. This is the recommended deployment method for any on-vehicle autonomy.

### Extension Architecture

Extensions are Docker containers managed by the Kraken service. They:
- Start automatically on boot
- Can expose a web UI (embedded as iframe in BlueOS)
- Access all BlueOS APIs via internal network
- Persist data in `/usr/blueos/extensions/`
- Declare permissions in Dockerfile labels

### Minimal Extension Example

**Dockerfile:**
```dockerfile
FROM python:3.11-slim

RUN pip install requests websockets

COPY app/ /app/
WORKDIR /app
EXPOSE 8080

LABEL version="1.0.0"
LABEL permissions='{\
    "ExposedPorts": {"8080/tcp": {}},\
    "HostConfig": {\
        "PortBindings": {"8080/tcp": [{"HostPort": ""}]},\
        "NetworkMode": "host"\
    }\
}'

CMD ["python", "main.py"]
```

**app/main.py:**
```python
"""BlueOS Extension: Custom thruster/rudder control."""
import requests
import time
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

MAVLINK2REST = "http://127.0.0.1/mavlink2rest/mavlink"

def send_manual_control(x, y, z, r):
    payload = {
        "header": {"system_id": 255, "component_id": 0, "sequence": 0},
        "message": {
            "type": "MANUAL_CONTROL",
            "target": 1, "x": x, "y": y, "z": z, "r": r, "buttons": 0
        }
    }
    requests.post(MAVLINK2REST, json=payload, timeout=0.5)

def control_loop():
    while True:
        # Your autonomous logic here (x, y=steer, z=throttle, r)
        send_manual_control(0, 400, 300, 0)
        time.sleep(0.05)

threading.Thread(target=control_loop, daemon=True).start()

# Web UI for the extension
httpd = HTTPServer(('0.0.0.0', 8080), SimpleHTTPRequestHandler)
httpd.serve_forever()
```

### Service Registration

Extensions should expose a `register_service` endpoint:

```json
{
  "name": "Thruster Controller",
  "description": "Custom thruster/rudder control",
  "icon": "mdi-engine",
  "company": "Your Name",
  "version": "1.0.0",
  "api": "http://host.docker.internal:8080",
  "works_in_relative_paths": true
}
```

### Installing Extensions

1. **Manual**: BlueOS Extensions Manager → "+" → specify Docker image
2. **Docker Hub**: Build for `linux/arm/v7`, push, install by image name
3. **Bazaar**: Submit PR to [BlueOS-Extensions-Repository](https://github.com/bluerobotics/BlueOS-Extensions-Repository)

Template repo: [BlueOS-Extension-template](https://github.com/bluerobotics/BlueOS-Extension-template)

---

## 9. MAVROS / ROS Integration

For integrating with the ROS ecosystem. MAVROS bridges MAVLink to ROS topics/services.

### ROS 2 Launch

```python
# mavros_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros', executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'udp://192.168.2.2:14550@14550',
                'target_system_id': 1,
                'target_component_id': 1,
            }]
        ),
    ])
```

### ROS Control

```python
import rospy
from mavros_msgs.msg import ManualControl
from mavros_msgs.srv import CommandBool, SetMode

rospy.init_node('ardurover_control')

set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
manual_pub = rospy.Publisher('/mavros/manual_control/send', ManualControl, queue_size=10)

set_mode(custom_mode='MANUAL')
arm(True)

msg = ManualControl()
msg.x, msg.y, msg.z, msg.r = 0, 200, 300, 0  # y=steer (200=slight right), z=throttle (300=~30% fwd)
manual_pub.publish(msg)
```

---

## 10. BlueOS REST API Reference

### Autopilot Manager

```
POST /ardupilot-manager/v1.0/start        # Start ArduPilot
POST /ardupilot-manager/v1.0/stop         # Stop ArduPilot
POST /ardupilot-manager/v1.0/restart      # Restart ArduPilot
GET  /ardupilot-manager/v1.0/firmware/installed
GET  /ardupilot-manager/v1.0/endpoints    # MAVLink endpoints
POST /ardupilot-manager/v1.0/endpoints    # Create endpoint
GET  /ardupilot-manager/v1.0/vehicle      # Vehicle type
GET  /ardupilot-manager/v1.0/board        # Board info
```

### MAVLink2REST

```
GET  /mavlink2rest/mavlink                 # All current messages
GET  /mavlink2rest/mavlink/{message_name}  # Specific message
POST /mavlink2rest/mavlink                 # Send a message
WS   /mavlink2rest/ws/mavlink             # WebSocket stream
GET  /mavlink2rest/mavlink/vehicles/{id}/components/{id}/messages
```

### System

```
GET  /system-information/system            # Full system info
GET  /system-information/system/cpu
GET  /system-information/system/memory
GET  /system-information/system/temperature
GET  /version-chooser/v1.0/version/current
POST /commander/v1.0/shutdown
POST /commander/v1.0/reboot
```

### Network

```
GET  /cable-guy/v1.0/ethernet             # Ethernet interfaces
POST /cable-guy/v1.0/ethernet             # Configure interface
GET  /wifi-manager/v1.0/status
POST /wifi-manager/v1.0/connect
```

### Extensions

```
GET    /kraken/v1.0/extensions/installed
POST   /kraken/v1.0/extensions/install
DELETE /kraken/v1.0/extensions/{id}
GET    /kraken/v1.0/extensions/manifest    # Store manifest
```

### Video / Sensors

```
GET  /mavlink-camera-manager/streams
GET  /ping/v1.0/sensors
GET  /nmea-injector/v1.0/socks
```

Full interactive docs: `http://192.168.2.2/{service}/docs`

---

## 11. Key ArduRover Parameters

### Boat Configuration

```
FRAME_CLASS = 2             # Boat
FRAME_TYPE = 0              # Normal (rudder+throttle). 1=Omni3, 2=OmniX, 3=OmniPlus

# Servo output mapping (standard boat: rudder + single thruster)
SERVO1_FUNCTION = 26        # GroundSteering (rudder servo on output 1)
SERVO3_FUNCTION = 70        # Throttle (ESC/thruster on output 3)
# For skid steer: SERVO1_FUNCTION=73 (ThrottleLeft), SERVO3_FUNCTION=74 (ThrottleRight)

MOT_PWM_TYPE = 0            # Normal PWM (1000-2000us). 4=BrushedBiPolar for some ESCs
MOT_THR_MIN = 0             # Minimum throttle % (increase if motor has dead zone)
MOT_SPD_SCA_BASE = 0        # Speed at which steering is reduced (0=disabled)
MANUAL_OPTIONS = 0          # Bit 0: enable steering speed scaling in MANUAL mode
```

### System

```
SYSID_MYGCS = 255          # Accept commands from GCS ID 255
FS_GCS_ENABLE = 0          # Disable GCS failsafe (testing only!)
BRD_SAFETYENABLE = 0       # Disable safety switch if not present
ARMING_CHECK = 0            # Disable pre-arm checks (bench testing)
```

### RC Input

```
SERIAL4_PROTOCOL = 23       # RCIN via CRSF (ELRS)
RSSI_TYPE = 3               # RSSI from CRSF telemetry
MODE_CH = 7                 # RC channel for mode switching (Rover uses MODE_CH, not FLTMODE_CH)
MODE1 = 0                   # MANUAL
MODE2 = 3                   # STEERING
MODE3 = 15                  # GUIDED
```

**ArduRover modes:**

| Mode | Number | Description |
|------|--------|-------------|
| MANUAL | 0 | Direct RC pass-through (no stabilization) |
| ACRO | 1 | Rate-controlled steering |
| STEERING | 3 | Pilot controls throttle, ArduPilot holds heading on neutral stick |
| HOLD | 4 | Stop and hold position (needs GPS) |
| LOITER | 5 | Circle around a point (boats only, needs GPS) |
| FOLLOW | 6 | Follow another MAVLink vehicle |
| SIMPLE | 7 | Simplified control relative to vehicle heading |
| CIRCLE | 9 | Orbit a point |
| AUTO | 10 | Autonomous waypoint missions |
| RTL | 11 | Return to launch |
| SMART_RTL | 12 | Retrace path home |
| GUIDED | 15 | External position/velocity/heading commands |

### MANUAL_CONTROL Axis Mapping (ArduRover)

| Axis | Range | ArduRover mapping |
|------|-------|-------------------|
| x | -1000 to 1000 | Ignored by ArduRover (used by Sub for forward/back) |
| y | -1000 to 1000 | **Steering** (-1000=full left, 0=straight, 1000=full right) |
| z | -1000 to 1000 | **Throttle** (-1000=full reverse, 0=stop, 1000=full forward) |
| r | -1000 to 1000 | Ignored by ArduRover (used by Sub for yaw) |

**Note**: The y→steering, z→throttle mapping is confirmed from ArduRover source code (`Rover/GCS_MAVLink_Rover.cpp`: `manual_override(channel_steer, packet.y, ...)`). Send a gentle z=200 first and confirm the boat moves forward.

### RC Channel Mapping (ArduRover default)

| RC Channel | Function | SERVO output |
|------------|----------|-------------|
| 1 | Steering | SERVO1 (FUNCTION=26, GroundSteering) |
| 2 | (unused) | - |
| 3 | Throttle | SERVO3 (FUNCTION=70, Throttle) |
| 4 | (unused) | - |
| 7+ | Mode switch, aux functions | - |

---

## 12. Repos, Papers & Community

### Core Repositories

| Repository | Description |
|------------|-------------|
| [BlueOS](https://github.com/bluerobotics/BlueOS) | Main OS -- Docker-based, Python/Vue.js/Rust |
| [mavlink2rest](https://github.com/bluerobotics/mavlink2rest) | REST/WebSocket gateway for MAVLink (Rust) |
| [navigator-lib](https://github.com/bluerobotics/navigator-lib) | Python/Rust library for Navigator hardware |
| [BlueOS-Extensions-Repository](https://github.com/bluerobotics/BlueOS-Extensions-Repository) | Central extension registry |
| [BlueOS-Extension-template](https://github.com/bluerobotics/BlueOS-Extension-template) | Starter template for extensions |
| [ping-python](https://github.com/bluerobotics/ping-python) | Ping sonar Python library |
| [ardusub-zola](https://github.com/bluerobotics/ardusub-zola) | ArduSub documentation |

### Notable Community Extensions

| Extension | Description |
|-----------|-------------|
| Water Linked DVL | DVL A50/A125 integration for positioning |
| ZeroTier | VPN for remote access |
| DWE Camera Manager | DeepWater Exploration camera support |
| Cerulean Sonar | Cerulean sonar integration |
| Nucleus DVL | Nortek DVL integration |
| Cockpit | Next-gen web control interface (Blue Robotics) |

### Community Resources

| Resource | URL |
|----------|-----|
| BlueOS Docs | https://blueos.cloud/docs/stable/ |
| ArduRover Docs | https://ardupilot.org/rover/ |
| Blue Robotics Forum | https://discuss.bluerobotics.com/ |
| Extension Dev Guide | https://blueos.cloud/docs/stable/development/extensions/ |
| Navigator Dev Guide | https://bluerobotics.com/learn/navigator-developer-guide/ |
| Navigator Hardware Setup | https://bluerobotics.com/learn/navigator-hardware-setup/ |
| MAVLink Messages Ref | https://mavlink.io/en/messages/common.html |
| BlueOS Releases | https://github.com/bluerobotics/BlueOS/releases |

### Key Forum Threads

- [Using REST to Send MAVLink Messages](https://discuss.bluerobotics.com/t/using-rest-to-send-mavlink-messages-on-bluerov2/13006) -- practical MAVLink2REST examples
- [BlueOS Thruster Extension](https://discuss.bluerobotics.com/t/blueos-thruster-extension/17449) -- RC_CHANNELS_OVERRIDE approach for motor control
- [Control Thrusters by Python](https://discuss.bluerobotics.com/t/control-thruser-by-python/10886) -- pymavlink examples
- [Navigator Library + ArduPilot Simultaneously](https://discuss.bluerobotics.com/t/use-navigator-library-and-ardupilot-at-the-same-time/17489) -- why you can't (and how to work around it)
- [PWM Control from Ground Station](https://discuss.bluerobotics.com/t/control-of-pwm-on-navigator-board-from-ground-station/14118)

### Academic / Research Use

BlueOS + BlueROV2 is widely used in marine robotics research. Common applications:
- Coral reef monitoring / marine biology surveys
- Underwater inspection (infrastructure, ships)
- MATE ROV Competition platforms
- Labs: Woods Hole, MBARI, University of Rhode Island, various European maritime institutes
- Papers typically reference "BlueROV2" rather than "BlueOS" specifically

---

## 13. Recommendations

### For your setup (RPi4 + Navigator, coming from MAVLink/ArduPilot)

**Start with MAVLink2REST** -- it's the path of least resistance:
- No library installs needed (just HTTP)
- Works from your laptop, a web browser, or an extension
- Keeps all ArduPilot features (stabilization, failsafes, sensor fusion)
- Interactive Swagger docs at `http://192.168.2.2/mavlink2rest/docs`

**For production/deployment**, package your control logic as a **BlueOS Extension** (Docker container running on the Pi) using MAVLink2REST internally.

**If latency matters** (tight control loops, <10ms), use **pymavlink over UDP** with a dedicated MAVLink endpoint.

**Only use direct PWM (navigator-lib)** if:
- You're building a non-standard vehicle frame ArduPilot doesn't support
- You want to implement your own control system from scratch
- You're bench-testing individual actuators

### MAVLink is still the right choice

Since you used MAVLink with ArduPilot before, the good news is **MAVLink2REST wraps the same protocol in HTTP/WebSocket**. The messages (`MANUAL_CONTROL`, `RC_CHANNELS_OVERRIDE`, `COMMAND_LONG`) are identical -- you're just sending them as JSON over HTTP instead of binary over UDP. This means:

- Your existing MAVLink knowledge transfers directly
- You can choose between HTTP (simple) and UDP (fast)
- No need to learn a new protocol
- The MAVLink2REST WebSocket option gives you the best of both worlds (real-time + web-friendly)

### Quick Start Checklist

1. Connect to BlueOS at `http://192.168.2.2`
2. Enable Pirate Mode (skull icon in header)
3. Visit `http://192.168.2.2/mavlink2rest/` for interactive API docs
4. Try a GET request: `curl http://192.168.2.2/mavlink2rest/mavlink/HEARTBEAT`
5. Add a dedicated MAVLink endpoint for your scripts (MAVLink Endpoints page)
6. Start with `MANUAL_CONTROL` messages for basic movement
7. Graduate to `RC_CHANNELS_OVERRIDE` when you need per-channel control
8. Package as a BlueOS extension for deployment

---

## 14. Dual-Input Architecture: RC Controller + Companion NUC

### System Diagram

```
                                [External System]
                                 (MQTT / ROS / HTTP / custom)
                                       |
                                       v
                              ┌─────────────────┐
                              │  Docker on NUC   │
                              │  (command relay)  │
                              └────────┬─────────┘
                                       |
                          Ethernet / USB-Ethernet
                                       |
    [ELRS TX + RC]                     v
         |                   ┌──────────────────────┐
    CRSF serial              │   BlueOS on Pi 4     │
         |                   │                      │
         v                   │  ┌── MAVLink2REST ◄──┼──── NUC commands (HTTP/WS)
    [Navigator UART]──→ ArduPilot                   │
         RC input            │  └── MAVLink Router  │
                             │      (UDP endpoints) │
                             └──────────┬───────────┘
                                        |
                                   PWM outputs
                                        |
                              [Thrusters / Rudder / Servos]
```

### How ArduPilot Arbitrates Between RC and Companion

ArduPilot does NOT blend inputs. The active **flight mode** determines which input source drives the vehicle:

| Mode | Who drives | RC sticks | Companion commands |
|------|-----------|-----------|-------------------|
| MANUAL (0) | RC pilot | **Active** -- direct pass-through | Ignored (unless RC_CHANNELS_OVERRIDE sent) |
| STEERING (3) | RC pilot | **Active** -- heading hold on neutral stick | Ignored (unless RC_CHANNELS_OVERRIDE sent) |
| GUIDED (15) | Companion | Ignored | **Active** -- position/velocity/heading targets via MAVLink |
| AUTO (10) | Mission | Ignored | Mission waypoints only |
| HOLD (4) | Nobody | Vehicle holds position (GPS) | Ignored |
| LOITER (5) | Nobody | Boat circles a point (GPS) | Ignored |

**Critical nuance**: `RC_CHANNELS_OVERRIDE` and `MANUAL_CONTROL` from the companion **override physical RC** regardless of mode. They replace the RC input at the protocol level. This is powerful but dangerous -- the mode-based approach above is safer because the pilot can always flip back to MANUAL.

### RC Mode Switching Setup (ELRS)

#### ELRS Connection

```
# ArduPilot parameters for ELRS on Navigator UART
# (adjust serial port number to match your physical wiring)
SERIAL4_PROTOCOL = 23       # RCIN via CRSF protocol
RSSI_TYPE = 3               # RSSI from CRSF telemetry
# Baud rate is auto-configured by firmware for CRSF
```

ELRS uses the CRSF protocol over a full-duplex UART. Connect the ELRS receiver's TX to the Navigator UART RX and vice versa. The Navigator has 4 UART ports available. It also has a dedicated RC input supporting SBUS/Crossfire/IBUS -- check which physical connector your ELRS receiver is wired to.

#### Flight Mode Channel

ArduRover uses `MODE_CH` (not `FLTMODE_CH` like Copter/Sub):

```
MODE_CH = 7                 # 3-position switch on RC channel 7
MODE1 = 0                   # Switch LOW  (~1165us) → MANUAL (direct RC control)
MODE2 = 3                   # Switch MID  (~1425us) → STEERING (heading hold on neutral)
MODE3 = 15                  # Switch HIGH (~1815us) → GUIDED (companion NUC control)
```

**Note**: `MODE_CH` defaults to channel 8 in ArduRover. Set it to whichever RC channel your 3-position switch is on. The channel must not conflict with stick channels (typically 1 and 3 for steering/throttle).

PWM mapping for a 3-position switch:
- ~1000-1230 us → MODE1
- ~1231-1360 us → MODE2
- ~1361-1490 us → MODE3
- ~1491-1620 us → MODE4
- ~1621-1749 us → MODE5
- ~1750-2000 us → MODE6

#### Safety Override Switch

```
RC8_OPTION = 46             # "RC Override Enable" on channel 8 (2-pos switch)
```

A 2-position switch on channel 8:
- **LOW**: MAVLink overrides allowed (companion can send commands)
- **HIGH**: MAVLink overrides blocked (physical RC has absolute priority)

This is your **emergency takeback**. It doesn't change the flight mode -- it forcibly silences all MAVLink RC input at the protocol level.

### Programmatic Mode Control (from Companion)

#### Reading Current Mode

```python
import requests

BLUEOS = "http://192.168.2.2"

def get_current_mode():
    resp = requests.get(f"{BLUEOS}/mavlink2rest/mavlink/HEARTBEAT")
    hb = resp.json()
    return hb["message"]["custom_mode"]

# ArduRover mode numbers: MANUAL=0, STEERING=3, HOLD=4, LOITER=5, AUTO=10, GUIDED=15
mode = get_current_mode()
```

#### WebSocket Mode Monitoring (real-time)

```python
import asyncio, json, websockets

async def watch_mode():
    uri = "ws://192.168.2.2/mavlink2rest/ws/mavlink"
    async with websockets.connect(uri) as ws:
        async for raw in ws:
            msg = json.loads(raw)
            if msg.get("message", {}).get("type") == "HEARTBEAT":
                mode = msg["message"]["custom_mode"]
                armed = msg["message"]["base_mode"]["bits"] & 128  # MAV_MODE_FLAG_SAFETY_ARMED
                print(f"Mode: {mode}, Armed: {bool(armed)}")
```

#### Setting Mode from Companion

```python
def set_mode(mode_number):
    """Set flight mode. Only works if RC Override switch allows it."""
    requests.post(f"{BLUEOS}/mavlink2rest/mavlink", json={
        "header": {"system_id": 255, "component_id": 0, "sequence": 0},
        "message": {
            "type": "SET_MODE",
            "target_system": 1,
            "base_mode": {"bits": 209},
            "custom_mode": mode_number
        }
    })
```

**Be aware**: If the pilot has `MODE_CH` active, the RC switch continuously asserts its mode. If the companion sends `SET_MODE` to change to GUIDED but the RC switch is in the MANUAL position, ArduPilot will immediately flip back to MANUAL on the next RC update (~50Hz). The pilot's physical switch wins. This is correct behavior.

### Companion NUC Connection

#### Recommended: Ethernet (network)

Best option if the NUC has an ethernet port. Connect NUC to Pi via:
- Direct ethernet cable (Pi gets IP from BlueOS DHCP, typically `192.168.2.x`)
- Or an ethernet switch in the vehicle

The Docker container on the NUC then uses MAVLink2REST at `http://192.168.2.2` -- no special device passthrough needed.

#### Alternative: USB with network gadget

If the NUC connects to the Pi via USB, the Pi can expose a USB Ethernet gadget (RNDIS/CDC-ECM). This creates a virtual network interface. You'd then access BlueOS over that virtual ethernet link.

#### Not recommended: raw USB serial

Raw serial (`/dev/ttyACMx`) works but creates friction:
- Need `--device /dev/ttyACM0` in Docker
- Device names can change on reconnect
- Need to configure a BlueOS serial bridge
- Baud rate management
- Network-based approach is strictly better

### Parameter Summary

```
# === Vehicle ===
FRAME_CLASS = 2             # Boat
FRAME_TYPE = 0              # Normal (rudder+throttle)
SERVO1_FUNCTION = 26        # GroundSteering (rudder on output 1)
SERVO3_FUNCTION = 70        # Throttle (thruster on output 3)

# === RC Input (ELRS) ===
SERIAL4_PROTOCOL = 23       # RCIN via CRSF (match to your wiring)
RSSI_TYPE = 3               # RSSI from CRSF

# === Mode Switching (ArduRover uses MODE_CH, not FLTMODE_CH) ===
MODE_CH = 7                 # 3-pos switch on channel 7
MODE1 = 0                   # MANUAL (pilot direct control)
MODE2 = 3                   # STEERING (pilot with heading hold)
MODE3 = 15                  # GUIDED (companion NUC control)

# === Safety ===
RC8_OPTION = 46             # RC Override Enable (emergency takeback)
RC_OPTIONS = 0              # Accept both RC and MAVLink (default)
RC_OVERRIDE_TIME = 3.0      # MAVLink override timeout (seconds)

# === Companion MAVLink (NUC via ethernet to Pi) ===
SYSID_MYGCS = 255           # Accept commands from system ID 255
FS_GCS_ENABLE = 0           # Disable GCS failsafe for dev (RE-ENABLE FOR PRODUCTION)
```

---

## 15. Corner Cases, Gotchas & Sanity Check

### GUIDED Mode in ArduRover (works with GPS)

Unlike ArduSub (which needs DVL for position), **ArduRover GUIDED mode works out of the box with GPS** -- and your surface vessel has GPS. This is a major advantage.

In GUIDED mode, the companion NUC can command:

| Command | Message | Key fields |
|---------|---------|------------|
| **Go to position** | `SET_POSITION_TARGET_GLOBAL_INT` | lat, lon (x1e7), type_mask=`3580` |
| **Set velocity** | `SET_POSITION_TARGET_LOCAL_NED` | vx, vy (m/s), type_mask=`3559` |
| **Set heading** | `SET_POSITION_TARGET_LOCAL_NED` | yaw (radians), type_mask=`2559` |
| **Set turn rate** | `SET_POSITION_TARGET_LOCAL_NED` | yaw_rate (rad/s), type_mask=`1535` |
| **Set heading + speed** | `SET_ATTITUDE_TARGET` | quaternion + thrust (-1..1), type_mask=`39` |

**Velocity commands must be re-sent every second** -- the vehicle stops after 3 seconds of silence.

Example: send velocity forward at 2 m/s via MAVLink2REST:

```python
requests.post(f"{BLUEOS}/mavlink2rest/mavlink", json={
    "header": {"system_id": 255, "component_id": 0, "sequence": 0},
    "message": {
        "type": "SET_POSITION_TARGET_LOCAL_NED",
        "target_system": 1,
        "target_component": 1,
        "coordinate_frame": {"type": "MAV_FRAME_BODY_NED"},
        "type_mask": {"bits": 2535},  # velocity + yaw (0x09E7)
        "vx": 2.0,   # 2 m/s forward (body frame)
        "vy": 0.0,
        "vz": 0.0,
        "yaw": 0.0,
        "x": 0, "y": 0, "z": 0,
        "afx": 0, "afy": 0, "afz": 0,
        "yaw_rate": 0.0,
        "time_boot_ms": 0
    }
})
```

**Important**: Confirm `type_mask` bitmask values by testing on your vehicle. The bitmask tells ArduPilot which fields to **ignore**. Getting it wrong can cause unexpected behavior. Start with heading-only commands before adding velocity.

### Alternative: MANUAL_CONTROL in STEERING Mode (simpler)

If GUIDED mode feels heavy for your use case, a simpler approach:

- Pilot flips to **STEERING** mode (mode 3)
- In STEERING, ArduPilot holds heading when sticks are neutral
- Companion sends `MANUAL_CONTROL` messages (y=steering, z=throttle)
- This acts like a virtual joystick with heading stabilization
- No position targeting, no type_mask complexity

This is the lowest-friction path to get the companion controlling the boat.

### RC_CHANNELS_OVERRIDE vs MANUAL_CONTROL: Which to Use

| | RC_CHANNELS_OVERRIDE | MANUAL_CONTROL |
|-|---------------------|----------------|
| **Granularity** | Per-channel PWM (1100-1900us) | Axes: x/y/z/r (-1000 to 1000) |
| **Interacts with** | Replaces RC input channels | Replaces joystick input |
| **Mode-aware** | No -- always overrides RC regardless of mode | Yes -- processed per mode logic |
| **Per-channel passthrough** | Yes -- send `0` to pass channel to physical RC | No -- all-or-nothing |
| **Safety** | Dangerous -- overrides RC even in MANUAL | Slightly safer -- respects mode |
| **Use case** | Direct actuator control, custom mixing | Virtual joystick, companion control |

**Recommendation**: Use `MANUAL_CONTROL` from the companion. It respects ArduPilot's mode logic and motor mixing. Only use `RC_CHANNELS_OVERRIDE` if you need per-channel control or need to override specific channels while passing others through to physical RC.

### What ArduPilot Does with Invalid Commands

ArduPilot validates and clamps, but doesn't reject silently in all cases:

| Input | ArduPilot behavior |
|-------|-------------------|
| `MANUAL_CONTROL` x=5000 (out of range) | **Clamped** to -1000..1000 |
| `RC_CHANNELS_OVERRIDE` chan=2500us | **Clamped** to RCx_MIN..RCx_MAX |
| `RC_CHANNELS_OVERRIDE` chan=0 | **Pass-through** to physical RC for that channel |
| `RC_CHANNELS_OVERRIDE` chan=65535 | **Ignored** (field not used) |
| `COMMAND_LONG` with invalid command | Returns `MAV_RESULT_UNSUPPORTED` or `MAV_RESULT_DENIED` |
| `SET_MODE` to invalid mode number | Returns `MAV_RESULT_FAILED`, mode unchanged |
| Any command while disarmed | Thrusters won't spin regardless (except `DO_SET_SERVO` which can bypass arming) |
| `MANUAL_CONTROL` at <1 Hz | **Timeout** -- ArduPilot stops responding after ~1s (`JS_TIMEOUT`) |

**Validate on the companion side anyway.** ArduPilot clamps values but doesn't distinguish between "intentional full throttle" and "bug sent 9999". A rogue command at full thrust with no sanity check could be dangerous.

### Heartbeat Requirements

The companion **must** send MAVLink heartbeats at ≥1 Hz:

```python
# If using pymavlink
master.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_GCS,
    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
)
```

If using MAVLink2REST, heartbeats are **not automatically sent**. You must explicitly POST heartbeat messages at 1 Hz, or ArduPilot's GCS failsafe will trigger (if `FS_GCS_ENABLE` is set).

```python
def send_heartbeat():
    requests.post(f"{BLUEOS}/mavlink2rest/mavlink", json={
        "header": {"system_id": 255, "component_id": 0, "sequence": 0},
        "message": {
            "type": "HEARTBEAT",
            "type_": {"type": "MAV_TYPE_GCS"},
            "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
            "base_mode": {"bits": 0},
            "custom_mode": 0,
            "system_status": {"type": "MAV_STATE_ACTIVE"}
        }
    })
```

### Command Rate Requirements

| Message | Minimum rate | Timeout | What happens on timeout |
|---------|-------------|---------|------------------------|
| `MANUAL_CONTROL` | ~10 Hz | `JS_TIMEOUT` (default 1000ms) | ArduPilot treats as zero input |
| `RC_CHANNELS_OVERRIDE` | ~10 Hz | `RC_OVERRIDE_TIME` (default 3s) | Falls back to physical RC |
| Position targets (GUIDED) | ≥25 Hz | ~1-2s | Vehicle stops / holds position |
| Heartbeat | 1 Hz | `FS_GCS_TIMEOUT` (default 5s) | GCS failsafe triggers |

### Mode Switch Race Conditions

**Scenario**: RC switch is in MANUAL position. Companion sends `SET_MODE(GUIDED)`.

**What happens**: ArduPilot briefly enters GUIDED, then on the next RC input cycle (~20ms later) reads the mode switch channel PWM and switches back to MANUAL. The mode "flickers" but effectively stays MANUAL.

**The fix**: The companion should **not** forcibly set modes. It should only **read** the current mode and react. The pilot controls mode via the physical switch. The companion watches for GUIDED and starts/stops commanding accordingly.

**Exception**: If you want the companion to request a mode change (e.g., for autonomous mission start), you'd need:
1. No `MODE_CH` assigned (disable RC mode switching), OR
2. A dedicated "companion control" switch that sets `RC_OPTIONS` bit 0 (ignore RC receiver), OR
3. Use `RCx_OPTION = 76` (STANDBY) which explicitly hands off to the companion

### USB Disconnect / Reconnect

If the NUC-to-Pi USB cable disconnects:
- Network-over-USB link drops immediately
- MAVLink2REST WebSocket connection breaks
- `RC_OVERRIDE_TIME` expires (3s) → ArduPilot falls back to physical RC
- GCS failsafe timer starts → if `FS_GCS_ENABLE` is set, failsafe action triggers after timeout
- On reconnect, the Docker container must re-establish the WebSocket/HTTP connection

**Mitigation**: The companion Docker container should have reconnection logic with exponential backoff. And physical RC should always be available as fallback.

### ArduRover Boat-Specific Notes

This vehicle runs **ArduRover** (FRAME_CLASS=2, Boat), not ArduSub. Key differences from ArduSub documentation you may find online:

- **Mode numbers are different**: MANUAL=0 (not 19), GUIDED=15 (not 4), no STABILIZE mode
- **STEERING mode** (3) is the Rover equivalent of Sub's STABILIZE -- pilot controls throttle, ArduPilot holds heading
- **GUIDED works with GPS** out of the box (no DVL needed like ArduSub)
- **RC channel mapping**: ch1=steering, ch3=throttle (not the 6-DOF mapping of ArduSub)
- **`MANUAL_CONTROL` axes**: y=steering, z=throttle with range -1000..+1000 (not x=forward, z=vertical like ArduSub)
- **Mode channel parameter**: `MODE_CH` (not `FLTMODE_CH` like Copter/Sub)
- **No depth/altitude control** -- pressure sensor reads are irrelevant for surface vessel
- **`SERVO1_FUNCTION=26`** (GroundSteering) and **`SERVO3_FUNCTION=70`** (Throttle) are the key output mappings

### DO_SET_SERVO Bypasses Arming

`MAV_CMD_DO_SET_SERVO` can drive servo outputs **even when disarmed**. This is a feature (for testing lights, cameras, grippers) but also a safety risk. A companion sending `DO_SET_SERVO` to a thruster channel will spin that thruster regardless of arm state. Use `MANUAL_CONTROL` or `RC_CHANNELS_OVERRIDE` instead -- those respect arm state.

### System ID Collisions

If you have QGroundControl running on a laptop AND the companion NUC both sending commands, ensure they use **different MAVLink system IDs**:

```python
# QGroundControl defaults to system_id=255
# Set your companion to a different ID
COMPANION_SYSTEM_ID = 254  # or any unused ID

# And update ArduPilot to accept it:
# SYSID_MYGCS only accepts ONE system ID for RC_CHANNELS_OVERRIDE
# For COMMAND_LONG and SET_MODE, any system ID works
```

`SYSID_MYGCS` controls which system ID is accepted for `RC_CHANNELS_OVERRIDE`. Only **one** system ID is accepted. If QGC (255) and the companion (254) both need to send RC overrides, this is a conflict. Solutions:
- Only one sender uses `RC_CHANNELS_OVERRIDE`; the other uses `MANUAL_CONTROL`
- Change `SYSID_MYGCS` dynamically (not practical)
- Use `MANUAL_CONTROL` from the companion (no `SYSID_MYGCS` restriction)

---

## 16. Companion NUC Docker Architecture

### Recommended Stack

```
┌─────────────────────────────────────────────┐
│              Docker on NUC                   │
│                                              │
│  ┌─────────────────────────────────────┐    │
│  │       command-relay container        │    │
│  │                                      │    │
│  │  [Inbound]        [Outbound]         │    │
│  │  MQTT / HTTP /    MAVLink2REST WS    │    │
│  │  ROS / custom  →  to BlueOS Pi       │    │
│  │                                      │    │
│  │  ┌──────────────────────────────┐    │    │
│  │  │  Command Validator/Scaler    │    │    │
│  │  │  - Clamp ranges              │    │    │
│  │  │  - Rate limit                │    │    │
│  │  │  - Mode check                │    │    │
│  │  │  - Heartbeat manager         │    │    │
│  │  │  - Watchdog (kill on stale)  │    │    │
│  │  └──────────────────────────────┘    │    │
│  │                                      │    │
│  │  [Telemetry]                         │    │
│  │  MAVLink2REST WS ← BlueOS           │    │
│  │  → publish to inbound system         │    │
│  └─────────────────────────────────────┘    │
│                                              │
└──────────────────┬──────────────────────────┘
                   │ Ethernet / USB-Ethernet
                   v
            ┌─────────────┐
            │  BlueOS Pi  │ (192.168.2.2)
            └─────────────┘
```

### Connection: NUC to BlueOS

**Use network, not serial.** The Docker container only needs HTTP/WebSocket access to `192.168.2.2`:

```python
# No special Docker flags needed -- just network access
# docker run --network=host command-relay
# OR: docker run command-relay  (if bridge network can reach 192.168.2.2)

BLUEOS = "http://192.168.2.2"
WS_URL = "ws://192.168.2.2/mavlink2rest/ws/mavlink"
```

If using USB between NUC and Pi, set up USB Ethernet gadget mode on the Pi so the NUC sees it as a network interface. Alternatively, just use a short ethernet cable -- it's more reliable.

### Skeleton: Command Relay Container

```python
"""
command_relay.py -- Docker container on companion NUC.
Receives commands from external system, validates, and forwards to ArduPilot via BlueOS.
"""
import asyncio
import json
import time
import websockets
from dataclasses import dataclass
from enum import IntEnum

BLUEOS_WS = "ws://192.168.2.2/mavlink2rest/ws/mavlink"
BLUEOS_REST = "http://192.168.2.2/mavlink2rest/mavlink"
SYSTEM_ID = 254  # Different from QGC (255)

class Mode(IntEnum):
    MANUAL = 0
    ACRO = 1
    STEERING = 3
    HOLD = 4
    LOITER = 5
    AUTO = 10
    RTL = 11
    GUIDED = 15

@dataclass
class VehicleState:
    mode: int = 0
    armed: bool = False
    last_heartbeat: float = 0.0

state = VehicleState()


# ── Validation ──────────────────────────────────────────────

def validate_manual_control(x, y, z, r):
    """Clamp and validate MANUAL_CONTROL values for ArduRover.
    y = steering (-1000..1000), z = throttle (-1000..1000, 0=stop)
    x and r are ignored by ArduRover.
    """
    def clamp(v, lo, hi):
        return max(lo, min(hi, int(v)))

    return (
        clamp(x, -1000, 1000),   # ignored by ArduRover
        clamp(y, -1000, 1000),   # steering (left/right)
        clamp(z, -1000, 1000),   # throttle (-1000=full reverse, 0=stop, 1000=full forward)
        clamp(r, -1000, 1000),   # ignored by ArduRover
    )

def validate_rc_override(channels: dict) -> dict:
    """Validate RC override channel values."""
    validated = {}
    for ch, pwm in channels.items():
        ch = int(ch)
        if ch < 1 or ch > 18:
            continue
        if pwm == 0 or pwm == 65535:  # pass-through / ignore
            validated[ch] = pwm
        else:
            validated[ch] = max(1100, min(1900, int(pwm)))
    return validated

def is_companion_control_allowed() -> bool:
    """Check if current mode allows companion commands."""
    if not state.armed:
        return False
    # Only send commands in modes where the companion should be driving
    # GUIDED: companion sends position/velocity targets
    # STEERING: companion can send MANUAL_CONTROL (virtual joystick)
    # MANUAL/HOLD/RTL: pilot or autopilot is in control, don't interfere
    return state.mode in (Mode.GUIDED, Mode.STEERING)


# ── MAVLink message builders ────────────────────────────────

def build_heartbeat():
    return {
        "header": {"system_id": SYSTEM_ID, "component_id": 0, "sequence": 0},
        "message": {
            "type": "HEARTBEAT",
            "type_": {"type": "MAV_TYPE_GCS"},
            "autopilot": {"type": "MAV_AUTOPILOT_INVALID"},
            "base_mode": {"bits": 0},
            "custom_mode": 0,
            "system_status": {"type": "MAV_STATE_ACTIVE"}
        }
    }

def build_manual_control(x, y, z, r):
    x, y, z, r = validate_manual_control(x, y, z, r)
    return {
        "header": {"system_id": SYSTEM_ID, "component_id": 0, "sequence": 0},
        "message": {
            "type": "MANUAL_CONTROL",
            "target": 1,
            "x": x, "y": y, "z": z, "r": r,
            "buttons": 0
        }
    }


# ── Main loop ───────────────────────────────────────────────

async def mavlink_loop():
    """Bidirectional WebSocket: read telemetry, send commands + heartbeats."""
    while True:
        try:
            async with websockets.connect(BLUEOS_WS) as ws:
                print(f"Connected to BlueOS at {BLUEOS_WS}")

                async def heartbeat_sender():
                    """Send heartbeats at 1 Hz."""
                    while True:
                        await ws.send(json.dumps(build_heartbeat()))
                        await asyncio.sleep(1.0)

                async def telemetry_receiver():
                    """Process incoming MAVLink messages."""
                    async for raw in ws:
                        msg = json.loads(raw)
                        mtype = msg.get("message", {}).get("type", "")

                        if mtype == "HEARTBEAT":
                            state.mode = msg["message"]["custom_mode"]
                            state.armed = bool(
                                msg["message"]["base_mode"]["bits"] & 128
                            )
                            state.last_heartbeat = time.time()

                async def command_sender():
                    """
                    Replace this with your actual command source.
                    E.g., subscribe to MQTT, read from ROS topic, HTTP endpoint, etc.
                    """
                    while True:
                        if is_companion_control_allowed():
                            # Example: gentle forward, straight ahead
                            # y=0 (no steering), z=200 (~20% forward throttle)
                            cmd = build_manual_control(x=0, y=0, z=200, r=0)
                            await ws.send(json.dumps(cmd))

                        await asyncio.sleep(0.05)  # 20 Hz

                await asyncio.gather(
                    heartbeat_sender(),
                    telemetry_receiver(),
                    command_sender(),
                )

        except (websockets.ConnectionClosed, ConnectionRefusedError, OSError) as e:
            print(f"Connection lost: {e}. Reconnecting in 2s...")
            await asyncio.sleep(2.0)

if __name__ == "__main__":
    asyncio.run(mavlink_loop())
```

### Dockerfile

```dockerfile
FROM python:3.11-slim
RUN pip install websockets requests
COPY command_relay.py /app/command_relay.py
WORKDIR /app
CMD ["python", "-u", "command_relay.py"]
```

```bash
# Build and run
docker build -t command-relay .
docker run --network=host --restart=unless-stopped command-relay
```

### What the Relay Should Do

1. **Always**: Send heartbeats at 1 Hz (prevents GCS failsafe)
2. **Always**: Monitor `HEARTBEAT` for current mode and arm state
3. **Only when allowed**: Forward movement commands (check mode + armed)
4. **Validate everything**: Clamp ranges, reject garbage, rate-limit
5. **Watchdog**: If the external command source goes stale (no new commands for N seconds), send neutral/zero commands, don't just stop sending (ArduPilot interprets silence as "keep doing what you were doing" for `RC_OVERRIDE_TIME` seconds)
6. **Log**: Record what's being sent and received for debugging

### What NOT to Do

- Don't send `SET_MODE` to override the pilot's RC switch -- the pilot's switch will immediately flip it back (race condition, section 15)
- Don't send `RC_CHANNELS_OVERRIDE` in MANUAL mode -- you'd be fighting the pilot's RC sticks
- Don't use `DO_SET_SERVO` for thrusters -- it bypasses arming checks
- Don't skip heartbeats -- GCS failsafe will trigger
- Don't send commands faster than 50 Hz -- unnecessary CPU load, MAVLink2REST queues up
- Don't assume the vehicle is in the mode you requested -- always read back and verify
