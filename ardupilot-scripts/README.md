# ArduPilot Lua Scripts

Lua scripts that run on ArduPilot (via the Navigator board / Raspberry Pi). These are uploaded to the vehicle, not run on the NUC.

## motor_mixer.lua

Custom motor mixing for a 3-motor setup: 2x Blue Robotics thrusters (skid steering) + 1x Flipsky thruster (forward boost).

### How it works

The script runs at 50Hz and:

1. Reads ArduPilot's internal throttle and steering demands via `vehicle:get_control_output()`
2. Checks the current motor mode (RC aux switch or `SCR_USER1` parameter)
3. Mixes and writes to 3 servo outputs via `SRV_Channels:set_output_norm()`

The gamepad/autonomy/remote UI just send normal steering + throttle commands. ArduPilot processes them through its normal control pipeline. The Lua script intercepts the output and distributes it to the right motors.

### Motor modes

| Switch Position | SCR_USER1 | Mode            | Blue Robotics L/R        | Flipsky     |
| --------------- | --------- | --------------- | ------------------------ | ----------- |
| Low (default)   | 0         | `blue_robotics` | Skid steering (T+S, T-S) | Off         |
| Mid             | 1         | `all`           | Skid steering (T+S, T-S) | Forward (T) |
| High            | 2         | `flipsky`       | Off                      | Forward (T) |

### Mode selection priority

1. **RC aux switch** (`RCx_OPTION=300`) — physical toggle, always wins
2. **SCR_USER1 parameter** — set via gateway `POST /command/motor_mode` or any MAVLink GCS

### Required ArduPilot parameters

```
SCR_ENABLE       = 1       # Enable Lua scripting (reboot required)
SERVO1_FUNCTION  = 94      # Script1 → Blue Robotics Left
SERVO2_FUNCTION  = 95      # Script2 → Blue Robotics Right
SERVO3_FUNCTION  = 96      # Script3 → Flipsky
RCx_OPTION       = 300     # Assign an RC channel to Scripting1 (motor mode switch)
```

Adjust `SERVO1`/`SERVO2`/`SERVO3` to match whichever physical outputs your ESCs are wired to.

### Deployment

1. Set `SCR_ENABLE = 1` in ArduPilot parameters and reboot
2. Open BlueOS File Browser: `http://blueos.local:7777/file-browser/`
3. Navigate to `configs/ardupilot-manager/firmware/scripts/`
4. Upload `motor_mixer.lua`
5. Restart the autopilot

**Do NOT use the standard ArduPilot path `APM/scripts/`** — BlueOS uses a different directory structure.

### Verification

After uploading and restarting, check the GCS messages tab for:

```
motor_mixer.lua is running
motor_mixer: RC switch found for mode select
motor_mixer: mode=BLUE_ROBOTICS
```

If no RC switch is assigned, you'll see:

```
motor_mixer: No RC switch (RCx_OPTION=300), using SCR_USER1 only
```

### Safety

- All outputs are zeroed when disarmed
- If the script crashes, servo outputs freeze at last PWM value — ArduPilot's own failsafe still applies
- RC aux switch takes priority over software mode selection
- Default mode (no RC switch, SCR_USER1=0) is `blue_robotics` (safest — no Flipsky)

### RC and Gamepad Coexistence

MANUAL_CONTROL (from the gamepad via the gateway) only overrides the steer and throttle RC channels. RC aux channels (including the motor mode switch) remain fully functional. Key parameters:

```
RC_OPTIONS       = 0       # Don't ignore RC receiver
RC_OVERRIDE_TIME = 3.0     # Override expires 3s after gamepad stops sending
```

To hand control back to the RC transmitter, stop the gamepad process. ArduPilot's manual override expires after `RC_OVERRIDE_TIME` seconds and RC steering/throttle resume.
