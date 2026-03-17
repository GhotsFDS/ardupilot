#include "AP_STTController.h"

// 最终参数名会是：STT_LAMBDA、STT_KSW、STT_KADAPT、STT_SSAT、STT_SCL
const AP_Param::GroupInfo AP_STTController::var_info[] = {

    // @Param: LAMBDA
    // @DisplayName: STT sliding surface lambda
    // @Description: 误差收敛速度系数，越大响应越快，太大可能导致控制太“猛”
    // @Range: 0.1 5.0
    // @Units: -
    AP_GROUPINFO("LAMBDA", 0, AP_STTController, _lambda, 1.0f),

    // @Param: KSW
    // @DisplayName: STT switching gain
    // @Description: 滑模切换增益，越大鲁棒性越强，但可能引入舵面抖振
    // @Range: 0.1 5.0
    // @Units: -
    AP_GROUPINFO("KSW",    1, AP_STTController, _k_sw, 0.8f),

    // @Param: KADAPT
    // @DisplayName: STT disturbance adaptation gain
    // @Description: 扰动估计自适应步长，越大收敛更快，但太大容易发散或激烈抖动
    // @Range: 0.001 0.1
    // @Units: -
    AP_GROUPINFO("KADAPT", 2, AP_STTController, _k_adapt, 0.01f),

    // @Param: SSAT
    // @DisplayName: STT sliding surface saturation
    // @Description: 滑模面饱和阈值，大误差时限制 |s|，防止控制量过大
    // @Range: 5 60
    // @Units: -
    AP_GROUPINFO("SSAT",   3, AP_STTController, _s_sat, 20.0f),

    // @Param: SCL
    // @DisplayName: STT servo output scale
    // @Description: u_raw 映射到舵机 [-4500,4500] 的缩放系数
    // @Range: 50 800
    // @Units: -
    AP_GROUPINFO("SCL",    5, AP_STTController, _servo_scale, 300.0f),

    AP_GROUPEND
};

AP_STTController::AP_STTController()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_STTController::set_targets(float roll_deg,
                                   float pitch_deg,
                                   float yaw_rate_deg_s)
{
    _ref_roll_deg  = roll_deg;
    _ref_pitch_deg = pitch_deg;
    _ref_yaw_rate  = yaw_rate_deg_s;
}

/**
 * 单轴滑模更新
 * ref          : 期望量（姿态/角速度）
 * meas         : 实测量
 * rate         : 实测一阶导（姿态微分，用陀螺；若 ref 是角速度，可传 0）
 * d_hat        : 扰动估计（持久状态，调用间保留）
 * u_out        : 输出（已缩放到舵机单位）
 * speed_scaler : 空速缩放 = scaling_speed / airspeed
 */
void AP_STTController::compute_axis(float ref,
                                    float meas,
                                    float rate,
                                    float &d_hat,
                                    float &u_out,
                                    float speed_scaler)
{
    const float e     = ref - meas;
    const float e_dot = -rate;
    float s           = e_dot + _lambda * e;

    const float u_eq  = _lambda * e;
    // 归一化 s 到 [-1, 1] 再乘 k_sw，减少抖振
    const float u_sw  = -_k_sw * constrain_float(s / _s_sat, -1.0f, 1.0f);

    // 自适应扰动估计 + 限幅防发散
    d_hat += -_k_adapt * s;
    d_hat = constrain_float(d_hat, -10.0f, 10.0f);

    const float u_raw = u_eq + u_sw - d_hat;

    // 应用空速缩放后输出到舵机单位
    u_out = constrain_float(u_raw * _servo_scale * speed_scaler, -4500.0f, 4500.0f);
}

void AP_STTController::update(float pitch_meas_deg,
                              float roll_meas_deg,
                              float /*yaw_meas_deg*/,
                              float p_deg_s,
                              float q_deg_s,
                              float r_deg_s,
                              float speed_scaler)
{
    // pitch：跟踪姿态角，速率用 q
    compute_axis(_ref_pitch_deg, pitch_meas_deg, q_deg_s,
                 _d_hat_pitch, _u_pitch, speed_scaler);

    // roll：跟踪姿态角，速率用 p
    compute_axis(_ref_roll_deg,  roll_meas_deg,  p_deg_s,
                 _d_hat_roll, _u_roll, speed_scaler);

    // yaw：跟踪偏航角速度 ref_yaw_rate，meas 用 r，rate 先给 0
    compute_axis(_ref_yaw_rate,  r_deg_s,       0.0f,
                 _d_hat_yaw, _u_yaw, speed_scaler);
}
