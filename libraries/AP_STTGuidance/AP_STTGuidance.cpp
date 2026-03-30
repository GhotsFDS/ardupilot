#include "AP_STTGuidance.h"

#if AP_STT_GUIDANCE_ENABLED

#include <AP_Math/AP_Math.h>

AP_STTGuidance *AP_STTGuidance::_singleton = nullptr;

// @Group: STT_G_
// @Path: AP_STTGuidance.cpp
const AP_Param::GroupInfo AP_STTGuidance::var_info[] = {
    // @Param: MID_H_KP
    // @DisplayName: Mid heading P gain
    // @Range: 0.1 2.0
    AP_GROUPINFO("MID_H_KP", 1, AP_STTGuidance, mid_h_kp, 0.80f),

    // @Param: MID_H_KI
    // @DisplayName: Mid heading I gain
    // @Range: 0.0 0.2
    AP_GROUPINFO("MID_H_KI", 2, AP_STTGuidance, mid_h_ki, 0.040f),

    // @Param: MID_H_KD
    // @DisplayName: Mid heading D gain
    // @Range: 0.0 0.5
    AP_GROUPINFO("MID_H_KD", 3, AP_STTGuidance, mid_h_kd, 0.06f),

    // @Param: MID_P_KP
    // @DisplayName: Mid pitch P gain
    // @Range: 0.01 1.0
    AP_GROUPINFO("MID_P_KP", 4, AP_STTGuidance, mid_p_kp, 0.10f),

    // @Param: MID_VREF
    // @DisplayName: Mid V-squared reference speed (m/s)
    // @Range: 10 100
    AP_GROUPINFO("MID_VREF", 5, AP_STTGuidance, mid_vref, 35.0f),

    // @Param: KF_Q
    // @DisplayName: KF process noise intensity (deg/s^2)^2
    // @Range: 10 2000
    AP_GROUPINFO("KF_Q", 6, AP_STTGuidance, kf_q, 400.0f),

    // @Param: KF_R
    // @DisplayName: KF measurement noise variance (deg^2)
    // @Range: 0.001 1.0
    AP_GROUPINFO("KF_R", 7, AP_STTGuidance, kf_r, 0.04f),

    // @Param: KF_MAXR
    // @DisplayName: KF max LOS rate limit (deg/s)
    // @Range: 5 100
    AP_GROUPINFO("KF_MAXR", 8, AP_STTGuidance, kf_maxr, 40.0f),

    // @Param: ROLL_KP
    // @DisplayName: Roll PD P gain
    // @Range: 0.001 0.1
    AP_GROUPINFO("ROLL_KP", 9, AP_STTGuidance, roll_kp, 0.020f),

    // @Param: ROLL_KD
    // @DisplayName: Roll PD D gain
    // @Range: 0.001 0.05
    AP_GROUPINFO("ROLL_KD", 10, AP_STTGuidance, roll_kd, 0.004f),

    // @Param: TRM_VREF
    // @DisplayName: Terminal V-squared reference speed (m/s)
    // @Range: 30 200
    AP_GROUPINFO("TRM_VREF", 11, AP_STTGuidance, trm_vref, 80.0f),

    // @Param: T_FF_P1
    // @DisplayName: Terminal pitch feedforward gain (<300m)
    // @Range: 0.01 1.0
    AP_GROUPINFO("T_FF_P1", 12, AP_STTGuidance, t_ff_p1, 0.30f),

    // @Param: T_FF_P2
    // @DisplayName: Terminal pitch feedforward gain (300-500m)
    // @Range: 0.01 1.0
    AP_GROUPINFO("T_FF_P2", 13, AP_STTGuidance, t_ff_p2, 0.20f),

    // @Param: T_FF_P3
    // @DisplayName: Terminal pitch feedforward gain (>500m)
    // @Range: 0.01 1.0
    AP_GROUPINFO("T_FF_P3", 14, AP_STTGuidance, t_ff_p3, 0.08f),

    // @Param: T_RFF_G1
    // @DisplayName: Terminal rate feedforward gain (<300m)
    // @Range: 0.001 0.1
    AP_GROUPINFO("T_RFF_G1", 15, AP_STTGuidance, t_rff_g1, 0.030f),

    // @Param: T_RFF_G2
    // @DisplayName: Terminal rate feedforward gain (>=300m)
    // @Range: 0.001 0.1
    AP_GROUPINFO("T_RFF_G2", 16, AP_STTGuidance, t_rff_g2, 0.012f),

    // @Param: T_YAW_FF
    // @DisplayName: Terminal yaw feedforward gain
    // @Range: 0.001 0.1
    AP_GROUPINFO("T_YAW_FF", 17, AP_STTGuidance, t_yaw_ff, 0.020f),

    // @Param: T_YAW_MAX
    // @DisplayName: Terminal yaw command clamp
    // @Range: 0.01 0.5
    AP_GROUPINFO("T_YAW_MAX", 18, AP_STTGuidance, t_yaw_max, 0.05f),

    // @Param: T_PN_P
    // @DisplayName: Terminal PN pitch gain
    // @Range: 0.0 2.0
    AP_GROUPINFO("T_PN_P", 19, AP_STTGuidance, t_pn_p, 0.50f),

    // @Param: T_PN_Y
    // @DisplayName: Terminal PN yaw gain
    // @Range: 0.0 2.0
    AP_GROUPINFO("T_PN_Y", 20, AP_STTGuidance, t_pn_y, 0.0f),

    // @Param: T_SKR_P1
    // @DisplayName: Terminal seeker pitch gain (<300m)
    // @Range: 0.001 0.2
    AP_GROUPINFO("T_SKR_P1", 21, AP_STTGuidance, t_skr_p1, 0.03f),

    // @Param: T_SKR_P2
    // @DisplayName: Terminal seeker pitch gain (300-1200m)
    // @Range: 0.001 0.1
    AP_GROUPINFO("T_SKR_P2", 22, AP_STTGuidance, t_skr_p2, 0.015f),

    // @Param: T_KD_P_CL
    // @DisplayName: Terminal pitch damping close (<200m)
    // @Range: 0.001 0.05
    AP_GROUPINFO("T_KD_P_CL", 23, AP_STTGuidance, t_kd_p_cl, 0.003f),

    // @Param: T_KD_P_FR
    // @DisplayName: Terminal pitch damping far (>=200m)
    // @Range: 0.001 0.05
    AP_GROUPINFO("T_KD_P_FR", 24, AP_STTGuidance, t_kd_p_fr, 0.010f),

    // @Param: T_KD_Y_CL
    // @DisplayName: Terminal yaw damping close (<300m)
    // @Range: 0.001 0.05
    AP_GROUPINFO("T_KD_Y_CL", 25, AP_STTGuidance, t_kd_y_cl, 0.005f),

    // @Param: T_KD_Y_FR
    // @DisplayName: Terminal yaw damping far (>=300m)
    // @Range: 0.001 0.05
    AP_GROUPINFO("T_KD_Y_FR", 26, AP_STTGuidance, t_kd_y_fr, 0.015f),

    // @Param: SKR_SRC
    // @DisplayName: Seeker source (0=real RS-422, 1=virtual GPS)
    // @Range: 0 1
    AP_GROUPINFO("SKR_SRC", 27, AP_STTGuidance, skr_src, 0),

    // @Param: TRM_CPP
    // @DisplayName: Terminal mode (0=Lua 50Hz, 1=C++ 300Hz)
    // @Range: 0 1
    AP_GROUPINFO("TRM_CPP", 28, AP_STTGuidance, trm_cpp, 0),

    // @Param: MID_CPP
    // @DisplayName: Midcourse servo mode (0=Lua, 1=C++ 300Hz damping+roll)
    // @Range: 0 1
    AP_GROUPINFO("MID_CPP", 29, AP_STTGuidance, mid_cpp, 0),

    // @Param: MID_KDP
    // @DisplayName: Midcourse pitch rate damping gain
    // @Range: 0.001 0.1
    AP_GROUPINFO("MID_KDP", 30, AP_STTGuidance, mid_kdp, 0.025f),

    // @Param: MID_KDY
    // @DisplayName: Midcourse yaw rate damping gain
    // @Range: 0.001 0.1
    AP_GROUPINFO("MID_KDY", 31, AP_STTGuidance, mid_kdy, 0.005f),

    AP_GROUPEND
};

AP_STTGuidance::AP_STTGuidance()
{
    if (_singleton == nullptr) {
        _singleton = this;
    }
    AP_Param::setup_object_defaults(this, var_info);
}

// --- static helpers ---

float AP_STTGuidance::_clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double AP_STTGuidance::_angle_diff(double a, double b)
{
    double d = fmod(a - b, 360.0);
    if (d >= 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

double AP_STTGuidance::_clampd(double v, double lo, double hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double AP_STTGuidance::_gps_distance(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double EARTH_R = 6371000.0;
    constexpr double RAD = M_PI / 180.0;
    const double dlat = (lat2 - lat1) * RAD;
    const double dlon = (lon2 - lon1) * RAD;
    const double a = sin(dlat * 0.5) * sin(dlat * 0.5)
                   + cos(lat1 * RAD) * cos(lat2 * RAD) * sin(dlon * 0.5) * sin(dlon * 0.5);
    return 2.0 * EARTH_R * atan2(sqrt(a), sqrt(1.0 - a));
}

double AP_STTGuidance::_gps_bearing(double lat1, double lon1, double lat2, double lon2)
{
    constexpr double RAD = M_PI / 180.0;
    constexpr double DEG = 180.0 / M_PI;
    const double dlon = (lon2 - lon1) * RAD;
    const double y = sin(dlon) * cos(lat2 * RAD);
    const double x = cos(lat1 * RAD) * sin(lat2 * RAD)
                   - sin(lat1 * RAD) * cos(lat2 * RAD) * cos(dlon);
    double brg = atan2(y, x) * DEG;
    if (brg < 0) brg += 360.0;
    return brg;
}

// === roll stabilizer ===

float AP_STTGuidance::roll_stabilize(float roll_deg, float roll_rate_dps,
                                      float airspeed, float v_ref) const
{
    const float aspd_c = MAX(airspeed, 30.0f);
    const float r = v_ref / aspd_c;
    const float sg = MIN(r * r, 4.0f);

    const float kp = roll_kp.get() * sg;
    const float kd = roll_kd.get() * sg;

    float cmd = -(roll_deg * kp + roll_rate_dps * kd);
    return _clamp(cmd, -1.0f, 1.0f);
}

#endif  // AP_STT_GUIDANCE_ENABLED
