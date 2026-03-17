#pragma once

// 只用到 constrain_float
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

/**
 * STT 滑模控制器
 * - 输入：期望姿态 / 偏航角速度 + 实测姿态 / 角速度
 * - 输出：已经缩放到 [-4500,4500] 的 u_pitch / u_roll / u_yaw
 */
class AP_STTController {
public:
    // 参数表，用于 AP_SUBGROUPINFO
    static const struct AP_Param::GroupInfo var_info[];
    // ==== 可调滑模参数（通过 AP_Param） ====
    // λ：误差收敛速度系数，越大响应越快，但可能更“凶猛”
    AP_Float _lambda;       // STT_LAMBDA

    // k_sw：滑模切换增益，越大鲁棒/纠偏更强，但抖振风险更大
    AP_Float _k_sw;         // STT_KSW

    // k_adapt：扰动估计自适应更新步长，越大收敛越快，但容易发散或抖动
    AP_Float _k_adapt;      // STT_KADAPT

    // s_sat：滑模面饱和阈值，大误差时限制 |s|，防止 u_sw 过大，同时减小抖振
    AP_Float _s_sat;        // STT_SSAT

    // servo_scale：控制量到舵机输出的比例系数，用来把 u_raw 映射到 ±4500 标度
    AP_Float _servo_scale;  // STT_SCL

    AP_STTController();

    // 设置目标值（单位：度 / 度每秒）
    void set_targets(float roll_deg,
                     float pitch_deg,
                     float yaw_rate_deg_s);

    // 更新控制输出（姿态单位：度，角速度单位：度/秒）
    // speed_scaler: 空速缩放 = scaling_speed / airspeed（补偿 V² 气动力矩）
    void update(float pitch_meas_deg,
                float roll_meas_deg,
                float yaw_meas_deg,
                float p_deg_s,
                float q_deg_s,
                float r_deg_s,
                float speed_scaler = 1.0f);

    // 已缩放到 [-4500,4500] 的舵机控制量
    float u_pitch() const { return _u_pitch; }
    float u_roll()  const { return _u_roll; }
    float u_yaw()   const { return _u_yaw; }

private:
    // 期望值
    float _ref_roll_deg  = 0.0f;
    float _ref_pitch_deg = 0.0f;
    float _ref_yaw_rate  = 0.0f;   // deg/s

    // 扰动估计
    float _d_hat_pitch   = 0.0f;
    float _d_hat_roll    = 0.0f;
    float _d_hat_yaw     = 0.0f;

    // 控制输出（舵机缩放后的量）
    float _u_pitch = 0.0f;
    float _u_roll  = 0.0f;
    float _u_yaw   = 0.0f;

    // 单轴滑模更新
    void compute_axis(float ref,
                      float meas,
                      float rate,
                      float &d_hat,
                      float &u_out,
                      float speed_scaler);
};
