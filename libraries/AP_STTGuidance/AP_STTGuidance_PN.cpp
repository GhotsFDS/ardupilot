#include "AP_STTGuidance.h"

#if AP_STT_GUIDANCE_ENABLED

// --- Kalman Filter (double precision) ---

void AP_STTGuidance::_kf_init(KFState &kf, double z)
{
    kf.x1 = z;
    kf.x2 = 0;
    kf.p11 = double(kf_r.get());
    kf.p12 = 0;
    kf.p21 = 0;
    kf.p22 = P_RATE_INIT;
    kf.inited = true;
}

void AP_STTGuidance::_kf_predict(KFState &kf, double dt, double q_spec, double r_meas)
{
    (void)r_meas;
    if (!kf.inited) {
        return;
    }

    // state predict: x' = F*x (constant rate model)
    kf.x1 += kf.x2 * dt;
    // x2 unchanged

    // covariance predict: P' = F*P*F' + Q
    const double dt2 = dt * dt;
    const double dt3 = dt2 * dt;
    const double q = q_spec;

    const double np11 = kf.p11 + dt * (kf.p12 + kf.p21) + dt2 * kf.p22 + q * dt3 / 3.0;
    const double np12 = kf.p12 + dt * kf.p22 + q * dt2 / 2.0;
    const double np21 = kf.p21 + dt * kf.p22 + q * dt2 / 2.0;
    const double np22 = kf.p22 + q * dt;

    kf.p11 = np11;
    kf.p12 = np12;
    kf.p21 = np21;
    kf.p22 = np22;
}

bool AP_STTGuidance::_kf_update(KFState &kf, double z, double r_meas)
{
    if (!kf.inited) {
        _kf_init(kf, z);
        return true;
    }

    // innovation: y = z - H*x
    double y = z - kf.x1;
    // angle wrap
    if (y > 180.0) {
        y -= 360.0;
    } else if (y < -180.0) {
        y += 360.0;
    }

    // innovation covariance: S = P[1,1] + R
    const double S = kf.p11 + r_meas;

    // innovation gate
    if (y * y > GATE_SQ * S) {
        return false;
    }

    // Kalman gain: K = P*H'/S
    const double k1 = kf.p11 / S;
    const double k2 = kf.p21 / S;

    // state update
    kf.x1 += k1 * y;
    kf.x2 += k2 * y;

    // covariance update: P = (I - K*H) * P
    const double new_p11 = (1.0 - k1) * kf.p11;
    const double new_p12 = (1.0 - k1) * kf.p12;
    const double new_p21 = kf.p21 - k2 * kf.p11;
    const double new_p22 = kf.p22 - k2 * kf.p12;

    kf.p11 = new_p11;
    kf.p12 = new_p12;
    kf.p21 = new_p21;
    kf.p22 = new_p22;

    return true;
}

// --- PN init ---

void AP_STTGuidance::pn_init(float nav_ratio, float filter_alpha, float max_g)
{
    _pn.N = double(nav_ratio);
    _pn.alpha = double(filter_alpha);
    _pn.max_g = double(max_g);
    _pn.max_los_rate = double(kf_maxr.get());

    _pn.initialized = false;
    _pn.prev_ms = 0;
    _pn.prev_pitch_los = 0;
    _pn.prev_yaw_los = 0;
    _pn.a_pitch = 0;
    _pn.a_yaw = 0;
    _pn.frame_count = 0;

    // reset KFs
    _pn.pitch_kf = {};
    _pn.yaw_kf = {};
}

// --- accel to command ---

void AP_STTGuidance::_pn_accel_to_cmd(double a_pitch, double a_yaw,
                                       float &pitch_cmd, float &yaw_cmd) const
{
    const double cmd_scale = 5.0 * G;
    pitch_cmd = float(_clamp(a_pitch / cmd_scale, -1.0f, 1.0f));
    yaw_cmd   = float(_clamp(a_yaw / cmd_scale, -1.0f, 1.0f));
}

// --- PN update ---

bool AP_STTGuidance::pn_update(float pitch_los_deg, float yaw_los_deg, float Vc,
                                uint32_t now_ms,
                                float &pitch_cmd, float &yaw_cmd)
{
    const double q_spec = double(kf_q.get());
    const double r_meas = double(kf_r.get());
    const double max_rate = double(kf_maxr.get());

    const double plos = double(pitch_los_deg);
    const double ylos = double(yaw_los_deg);

    // first frame: init KFs
    if (!_pn.initialized) {
        _kf_init(_pn.pitch_kf, plos);
        _kf_init(_pn.yaw_kf, ylos);
        _pn.prev_pitch_los = plos;
        _pn.prev_yaw_los = ylos;
        _pn.prev_ms = now_ms;
        _pn.initialized = true;
        pitch_cmd = 0;
        yaw_cmd = 0;
        return true;
    }

    // dt
    const int32_t dt_ms = int32_t(now_ms - _pn.prev_ms);
    if (dt_ms <= 0 || dt_ms > 500) {
        _pn.prev_ms = now_ms;
        _pn_accel_to_cmd(_pn.a_pitch, _pn.a_yaw, pitch_cmd, yaw_cmd);
        return true;
    }
    const double dt = dt_ms * 0.001;
    _pn.prev_ms = now_ms;

    // 1. predict step (every control cycle, 50Hz)
    _kf_predict(_pn.pitch_kf, dt, q_spec, r_meas);
    _kf_predict(_pn.yaw_kf, dt, q_spec, r_meas);

    // 2. detect new measurement (use double for angle_diff to match Lua precision)
    double d_pitch = fmod(plos - _pn.prev_pitch_los, 360.0);
    if (d_pitch >= 180.0) d_pitch -= 360.0;
    if (d_pitch < -180.0) d_pitch += 360.0;

    double d_yaw = fmod(ylos - _pn.prev_yaw_los, 360.0);
    if (d_yaw >= 180.0) d_yaw -= 360.0;
    if (d_yaw < -180.0) d_yaw += 360.0;

    const bool angle_changed = (fabs(d_pitch) > 0.001 || fabs(d_yaw) > 0.001);

    if (angle_changed) {
        _pn.frame_count++;
        // 3. update step (new measurement arrived)
        _kf_update(_pn.pitch_kf, plos, r_meas);
        _kf_update(_pn.yaw_kf, ylos, r_meas);
        _pn.prev_pitch_los = plos;
        _pn.prev_yaw_los = ylos;
    }

    // 4. read KF rate estimates
    const double pitch_rate = fmax(fmin(_pn.pitch_kf.x2, max_rate), -max_rate);
    const double yaw_rate = fmax(fmin(_pn.yaw_kf.x2, max_rate), -max_rate);

    // ramp-up gain
    double gain = 1.0;
    if (_pn.frame_count <= 5) {
        gain = double(_pn.frame_count) / 5.0;
    }

    // PN law: a = N * Vc * dLOS/dt (rad/s)
    const double RAD = M_PI / 180.0;
    _pn.a_pitch = gain * _pn.N * double(Vc) * pitch_rate * RAD;
    _pn.a_yaw   = gain * _pn.N * double(Vc) * yaw_rate * RAD;

    // max accel clamp
    const double a_max = _pn.max_g * G;
    _pn.a_pitch = fmax(fmin(_pn.a_pitch, a_max), -a_max);
    _pn.a_yaw   = fmax(fmin(_pn.a_yaw, a_max), -a_max);

    _pn_accel_to_cmd(_pn.a_pitch, _pn.a_yaw, pitch_cmd, yaw_cmd);
    return true;
}

// --- getters ---

bool AP_STTGuidance::pn_get_los_rates(float &pitch_rate, float &yaw_rate) const
{
    if (!_pn.pitch_kf.inited) {
        pitch_rate = 0;
        yaw_rate = 0;
        return false;
    }
    pitch_rate = float(_pn.pitch_kf.x2);
    yaw_rate = float(_pn.yaw_kf.x2);
    return true;
}

bool AP_STTGuidance::pn_get_accel_g(float &ag_pitch, float &ag_yaw) const
{
    ag_pitch = float(_pn.a_pitch / G);
    ag_yaw = float(_pn.a_yaw / G);
    return true;
}

#endif  // AP_STT_GUIDANCE_ENABLED
