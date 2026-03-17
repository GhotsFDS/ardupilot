#include "mode.h"
#include "Plane.h"

bool ModeSTT::_enter()
{
    return true;
}

/*
  STT 模式：类似 FBWA 但强制零滚转
  - 利用 ArduPilot 标准 PID 控制器（含 speed_scaler、积分器、前馈）
  - 滚转锁零，俯仰/偏航由打杆或 Lua 脚本控制
  - C-tail 混控由 stt_mixer()（纯输出混控器）自动完成
*/
void ModeSTT::update()
{
    // 锁定零滚转（STT 核心：全程零滚转，用 pitch+yaw 合成任意方向机动）
    plane.nav_roll_cd = 0;

    // 俯仰：跟 FBWA 一样由打杆控制（Lua 可通过 guided/set_target_altitude 覆盖）
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max * 100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min * 100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd,
                                         plane.pitch_limit_min * 100,
                                         plane.aparm.pitch_limit_max.get() * 100);
}

void ModeSTT::run()
{
    // 标准姿态稳定（含 speed_scaler）— 写 k_aileron/k_elevator/k_rudder
    Mode::run();

    // 模式层更新（零滚转 + 俯仰指令）
    update();

    // 油门按飞手打杆输出
    output_pilot_throttle();
}
