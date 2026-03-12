-- motor_mixer.lua — Custom motor mixing for 2x Blue Robotics + 1x Flipsky
--
-- Reads ArduPilot's internal throttle/steering demands and distributes
-- them to three independent servo outputs via scripting servo functions.
--
-- Uses set_output_pwm_chan_timeout() so that if this script crashes,
-- outputs revert to servo trim (neutral) within 100ms automatically.
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

-- Timeout for set_output_pwm_chan_timeout (ms).
-- If the script crashes, outputs revert to servo trim after this period.
local PWM_TIMEOUT_MS = 100

-- ESC PWM range (standard for Blue Robotics Basic ESC and most ESCs)
local PWM_MIN = 1100
local PWM_TRIM = 1500
local PWM_MAX = 1900

-- Look up physical channel numbers (zero-indexed) for each scripting function.
-- These are needed for set_output_pwm_chan_timeout().
local chan1 = SRV_Channels:find_channel(K_SCRIPT1)
local chan2 = SRV_Channels:find_channel(K_SCRIPT2)
local chan3 = SRV_Channels:find_channel(K_SCRIPT3)

if not chan1 or not chan2 or not chan3 then
    gcs:send_text(3, "motor_mixer: SERVO functions 94/95/96 not all assigned!")
    gcs:send_text(3, string.format("motor_mixer: ch1=%s ch2=%s ch3=%s",
        tostring(chan1), tostring(chan2), tostring(chan3)))
    -- Return nil to stop the script — can't run without all 3 channels
    return
end

gcs:send_text(6, string.format("motor_mixer: channels=%d/%d/%d", chan1, chan2, chan3))

-- RC switch — re-resolved periodically so late-connecting receivers work
local rc_switch = rc:find_channel_for_option(300)
local rc_switch_last_check = 0
local RC_SWITCH_CHECK_INTERVAL_MS = 5000  -- re-check every 5s if not found

if rc_switch then
    gcs:send_text(6, "motor_mixer: RC switch found for mode select")
else
    gcs:send_text(5, "motor_mixer: No RC switch (RCx_OPTION=300), will keep checking")
end

-- Cache SCR_USER1 parameter object for fast reads
local user_mode = Parameter('SCR_USER1')

local last_mode = -1

-- Convert normalized value (-1..1) to PWM (with defensive clamp)
local function norm_to_pwm(value)
    value = math.max(-1, math.min(1, value))
    if value >= 0 then
        return math.floor(PWM_TRIM + value * (PWM_MAX - PWM_TRIM) + 0.5)
    else
        return math.floor(PWM_TRIM + value * (PWM_TRIM - PWM_MIN) + 0.5)
    end
end

-- Write PWM with timeout — reverts to trim if script stops calling
local function set_output(chan, norm_value)
    SRV_Channels:set_output_pwm_chan_timeout(chan, norm_to_pwm(norm_value), PWM_TIMEOUT_MS)
end

local function get_motor_mode()
    -- Re-resolve RC switch periodically if not yet found
    if not rc_switch then
        local now = millis():tofloat()
        if now - rc_switch_last_check >= RC_SWITCH_CHECK_INTERVAL_MS then
            rc_switch_last_check = now
            rc_switch = rc:find_channel_for_option(300)
            if rc_switch then
                gcs:send_text(6, "motor_mixer: RC switch found (late connect)")
            end
        end
    end

    -- RC switch takes priority if assigned (all positions are meaningful)
    if rc_switch then
        local pos = rc_switch:get_aux_switch_pos()
        if pos == 0 then
            return MODE_BLUE
        elseif pos == 1 then
            return MODE_ALL
        elseif pos == 2 then
            return MODE_FLIPSKY
        end
        -- pos is nil (RC link lost) — fall through to SCR_USER1 or default
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
        set_output(chan1, 0)
        set_output(chan2, 0)
        set_output(chan3, 0)
        return update, 20  -- 50Hz
    end

    local throttle = vehicle:get_control_output(CONTROL_THROTTLE)
    local steering = vehicle:get_control_output(CONTROL_YAW)

    if not throttle or not steering
       or throttle ~= throttle or steering ~= steering then
        -- Zero outputs if control data unavailable or NaN
        set_output(chan1, 0)
        set_output(chan2, 0)
        set_output(chan3, 0)
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

    -- Scale left/right pair proportionally to preserve steering balance
    local skid_max = math.max(math.abs(left), math.abs(right))
    if skid_max > 1 then
        left = left / skid_max
        right = right / skid_max
    end

    -- Clamp flipsky independently (it's a separate motor group)
    flipsky = math.max(-1, math.min(1, flipsky))

    set_output(chan1, left)
    set_output(chan2, right)
    set_output(chan3, flipsky)

    return update, 20  -- 50Hz
end

gcs:send_text(6, "motor_mixer.lua is running")
return update()
