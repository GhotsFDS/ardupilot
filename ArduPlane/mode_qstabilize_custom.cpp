#include "Plane.h"

#if AP_SCRIPTING_ENABLED && HAL_QUADPLANE_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <string.h>

// constructor registers custom number and names
ModeQStabilizeCustom::ModeQStabilizeCustom(const Number _number, const char* _full_name, const char* _short_name):
    number(_number),
    full_name(_full_name),
    short_name(_short_name)
{
}

bool ModeQStabilizeCustom::_enter()
{
    // Script can block entry
    if (!state.allow_entry) {
        return false;
    }

    // v9 P7.1: reset heartbeat timestamp on mode entry so watchdog has 500ms
    // grace before first check — avoids race where mode change between e.g. 29→27
    // immediately bounces back to 29 before lua's wig_auto.on_enter can tick.
    state.last_heartbeat_ms = AP_HAL::millis();

    // QStabilize entry checks must also pass
    return ModeQStabilize::_enter();
}

void ModeQStabilizeCustom::update()
{
    // Heartbeat watchdog: if Lua hasn't ticked in >500ms and a
    // failsafe target mode is configured, switch to that mode.
    if (state.last_heartbeat_ms > 0 &&
        (AP_HAL::millis() - state.last_heartbeat_ms) > 500 &&
        state.failsafe_target_mode != 0 &&
        state.failsafe_target_mode != (uint8_t)mode_number()) {
        plane.set_mode_by_number((Mode::Number)state.failsafe_target_mode,
                                 ModeReason::SCRIPTING);
        return;
    }

    ModeQStabilize::update();
}

// override run() — when state.yaw_target_active=true, drive ATC with yaw ANGLE target
// (跟 quadplane mission AUTO 同 angle path: input_euler_angle_roll_pitch_yaw_cd, ATC 自跑
// angle->rate->motor 两层级联, slew_yaw 自带平滑). false 时委托父类走 rate path.
void ModeQStabilizeCustom::run()
{
    if (!state.yaw_target_active) {
        // 默认 yaw rate path, 跟 QStabilize 行为一致
        ModeQStabilize::run();
        return;
    }

    // yaw heading lock path. 复用 ModeQStabilize::update() 设置 nav_roll/pitch,
    // 然后 ATC 用 angle yaw 而不是 rate yaw.
    const uint32_t now = AP_HAL::millis();
    if (quadplane.tailsitter.in_vtol_transition(now)) {
        Mode::run();
        return;
    }
    plane.quadplane.assign_tilt_to_fwd_thr();

    if (quadplane.esc_calibration != 0) {
        quadplane.run_esc_calibration();
        plane.stabilize_roll();
        plane.stabilize_pitch();
        return;
    }

    float pilot_throttle_scaled = quadplane.get_pilot_throttle();

    // 走 ATC angle yaw path (slew_yaw=true 自带过渡平滑)
    quadplane.attitude_control->input_euler_angle_roll_pitch_yaw_cd(
        plane.nav_roll_cd,
        plane.nav_pitch_cd,
        state.yaw_target_cd,
        true);

    // throttle / spool (复制 hold_stabilize 的 throttle 段, 撤掉 multicopter_attitude_rate_update)
    if ((pilot_throttle_scaled <= 0) && !quadplane.air_mode_active()) {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        quadplane.attitude_control->set_throttle_out(0, true, 0);
        quadplane.relax_attitude_control();
    } else {
        quadplane.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
        bool should_boost = true;
        if (quadplane.tailsitter.enabled() && quadplane.assisted_flight) {
            should_boost = false;
        }
        quadplane.attitude_control->set_throttle_out(pilot_throttle_scaled, should_boost, 0);
    }

    plane.stabilize_roll();
    plane.stabilize_pitch();
    output_rudder_and_steering(0.0);
}

bool ModeQStabilizeCustom::_pre_arm_checks(size_t buflen, char *buffer) const
{
    if (!state.pre_arm_ok) {
        // Copy script-provided message into the caller's buffer
        const char *msg = state.pre_arm_msg;
        if (msg[0] == '\0') {
            msg = "WIG: scripting blocked arming";
        }
        strncpy(buffer, msg, buflen);
        if (buflen > 0) {
            buffer[buflen - 1] = '\0';
        }
        return false;
    }
    if (!state.preflight_done) {
        strncpy(buffer, "WIG: preflight sweep not done", buflen);
        if (buflen > 0) {
            buffer[buflen - 1] = '\0';
        }
        return false;
    }
    return ModeQStabilize::_pre_arm_checks(buflen, buffer);
}

#endif // AP_SCRIPTING_ENABLED && HAL_QUADPLANE_ENABLED
