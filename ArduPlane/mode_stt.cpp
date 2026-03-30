#include "mode.h"
#include "Plane.h"

/*
  STT mode — Skid-To-Turn guided munition mode

  Core design: Lua scripting owns the flight phase state machine and writes
  pitch/yaw/roll/throttle commands to Script Motor functions (94-97).
  The C++ mode_stt only provides:
    1. Lock nav_roll_cd = 0 (STT zero-roll constraint)
    2. Disable ArduPilot's standard attitude controller output
       (Lua writes directly to SRV channels via stt_mixer)
    3. Allow stt_mixer() in servos.cpp to run (inertial→body rotation + 4-fin mix)

  The standard attitude stabilization (Mode::run → stabilize_xxx) is intentionally
  NOT called here — Lua + C++ libraries (AP_STTGuidance) handle all stabilization.
  ArduPilot's PID controllers (rollController, pitchController, yawController) are
  bypassed; the only servo output comes from Lua → k_scripting2/3/4 → stt_mixer().
*/

bool ModeSTT::_enter()
{
    // Force zero roll target
    plane.nav_roll_cd = 0;
    plane.nav_pitch_cd = 0;

    return true;
}

void ModeSTT::update()
{
    // Lock zero roll — STT core: all maneuvering through pitch + yaw
    plane.nav_roll_cd = 0;

    // nav_pitch_cd is not used (Lua controls pitch directly via Script Motor),
    // but set to zero to avoid stale values in logs
    plane.nav_pitch_cd = 0;
}

void ModeSTT::run()
{
    // Do NOT call Mode::run() — it would run stabilize_roll/pitch/yaw
    // which writes to k_aileron/k_elevator/k_rudder, conflicting with
    // Lua's Script Motor outputs (k_scripting2/3/4).

    update();

    // Throttle: Lua controls via Script Motor 1 (k_scripting → FUNC 94).
    // Set ArduPilot throttle to zero so it doesn't interfere.
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0);
}
