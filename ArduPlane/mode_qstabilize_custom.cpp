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
