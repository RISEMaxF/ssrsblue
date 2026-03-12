-- motor_mixer.lua — Custom motor mixing for 2x Blue Robotics + 1x Flipsky
--
-- Reads ArduPilot's internal throttle/steering demands and distributes
-- them to three independent servo outputs via scripting servo functions.
--
-- Motor mode is selected by:
--   1. RC aux switch (RCx_OPTION=300) — physical override, always wins
--   2. SCR_USER1 parameter — software control via MAVLink PARAM_SET
--
-- Modes (switch position / SCR_USER1 value):
--   0 / low  = Blue Robotics only (skid steering, Flipsky off)
--   1 / mid  = All three (skid steering + Flipsky forward thrust)
--   2 / high = Flipsky only (single thruster, no differential)
--
-- Required parameters:
--   SCR_ENABLE = 1
--   SERVO1_FUNCTION = 94  (Script1 → Blue Robotics Left)
--   SERVO2_FUNCTION = 95  (Script2 → Blue Robotics Right)
--   SERVO3_FUNCTION = 96  (Script3 → Flipsky)
--   RCx_OPTION = 300      (Scripting1 — aux switch for mode)
--
-- Upload to: BlueOS File Browser →
--   configs/ardupilot-manager/firmware/scripts/motor_mixer.lua

local K_SCRIPT1 = 94  -- Blue Robotics Left
local K_SCRIPT2 = 95  -- Blue Robotics Right
local K_SCRIPT3 = 96  -- Flipsky

local CONTROL_THROTTLE = 3
local CONTROL_YAW = 4

local MODE_BLUE = 0
local MODE_ALL = 1
local MODE_FLIPSKY = 2

-- Try to find the RC switch assigned to Scripting1 (RCx_OPTION=300)
local rc_switch = rc:find_channel_for_option(300)
if rc_switch then
    gcs:send_text(6, "motor_mixer: RC switch found for mode select")
else
    gcs:send_text(5, "motor_mixer: No RC switch (RCx_OPTION=300), using SCR_USER1 only")
end

-- Cache SCR_USER1 parameter object for fast reads
local user_mode = Parameter('SCR_USER1')

local last_mode = -1

local function get_motor_mode()
    -- RC switch takes priority if assigned and not at low (default) position
    if rc_switch then
        local pos = rc_switch:get_aux_switch_pos()
        if pos == 0 then
            return MODE_BLUE
        elseif pos == 1 then
            return MODE_ALL
        else
            return MODE_FLIPSKY
        end
    end

    -- Fallback to SCR_USER1
    local val = user_mode:get()
    if val then
        local mode = math.floor(val + 0.5)  -- round to nearest int
        if mode >= MODE_BLUE and mode <= MODE_FLIPSKY then
            return mode
        end
    end

    return MODE_BLUE  -- safe default
end

local function update()
    local mode = get_motor_mode()

    -- Log mode changes
    if mode ~= last_mode then
        local names = { [0] = "BLUE_ROBOTICS", [1] = "ALL", [2] = "FLIPSKY" }
        gcs:send_text(6, string.format("motor_mixer: mode=%s", names[mode] or "?"))
        last_mode = mode
    end

    if not arming:is_armed() then
        -- Zero all outputs when disarmed
        SRV_Channels:set_output_norm(K_SCRIPT1, 0)
        SRV_Channels:set_output_norm(K_SCRIPT2, 0)
        SRV_Channels:set_output_norm(K_SCRIPT3, 0)
        return update, 20  -- 50Hz
    end

    local throttle = vehicle:get_control_output(CONTROL_THROTTLE)
    local steering = vehicle:get_control_output(CONTROL_YAW)

    if not throttle or not steering then
        return update, 20
    end

    local left = 0
    local right = 0
    local flipsky = 0

    if mode == MODE_BLUE then
        -- Skid steering on Blue Robotics, Flipsky off
        left = throttle + steering
        right = throttle - steering
        flipsky = 0

    elseif mode == MODE_ALL then
        -- Skid steering on Blue Robotics + Flipsky forward thrust
        left = throttle + steering
        right = throttle - steering
        flipsky = throttle

    elseif mode == MODE_FLIPSKY then
        -- Flipsky only, no differential steering
        left = 0
        right = 0
        flipsky = throttle
    end

    -- Clamp outputs to [-1, 1]
    left = math.max(-1, math.min(1, left))
    right = math.max(-1, math.min(1, right))
    flipsky = math.max(-1, math.min(1, flipsky))

    SRV_Channels:set_output_norm(K_SCRIPT1, left)
    SRV_Channels:set_output_norm(K_SCRIPT2, right)
    SRV_Channels:set_output_norm(K_SCRIPT3, flipsky)

    return update, 20  -- 50Hz
end

gcs:send_text(6, "motor_mixer.lua is running")
return update()
