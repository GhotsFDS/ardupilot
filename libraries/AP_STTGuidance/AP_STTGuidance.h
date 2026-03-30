#pragma once

#include "AP_STTGuidance_config.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#if AP_STT_GUIDANCE_ENABLED

class AP_STTGuidance {
public:
    AP_STTGuidance();

    static AP_STTGuidance *get_singleton() { return _singleton; }
    static const struct AP_Param::GroupInfo var_info[];

    // === midcourse navigation ===
    void mid_init(float target_lat, float target_lon, float target_alt, float skr_dist);

    // returns true on success, pitch_cmd/yaw_cmd are normalized -1..+1
    bool mid_update(float lat, float lon, float alt,
                    float heading_deg, float pitch_deg, float airspeed,
                    uint32_t now_ms,
                    float &pitch_cmd, float &yaw_cmd);

    float mid_get_distance(float lat, float lon) const;
    uint8_t mid_get_phase() const { return _mid.phase; }
    float mid_get_heading_integral() const;

    // midcourse yaw-only update (for terminal reuse of heading PID)
    bool mid_yaw_update(float heading_deg, float bearing_deg, float airspeed,
                        uint32_t now_ms, float &yaw_cmd);

    // === PN guidance ===
    void pn_init(float nav_ratio, float filter_alpha, float max_g);

    // returns true on success, pitch_cmd/yaw_cmd are normalized -1..+1
    bool pn_update(float pitch_los_deg, float yaw_los_deg, float Vc,
                   uint32_t now_ms,
                   float &pitch_cmd, float &yaw_cmd);

    bool pn_get_los_rates(float &pitch_rate, float &yaw_rate) const;
    bool pn_get_accel_g(float &ag_pitch, float &ag_yaw) const;

    // === roll stabilizer ===
    float roll_stabilize(float roll_deg, float roll_rate_dps,
                         float airspeed, float v_ref) const;

    // === terminal guidance (C++ 300Hz path) ===
    // Lua passes float; internally stored as double via carrier int32
    void terminal_init(float target_lat, float target_lon, float target_alt);
    uint8_t terminal_update(uint32_t now_ms);  // 0=running, 1=impact
    uint8_t terminal_get_status() const { return _trm.status; }
    bool terminal_is_active() const { return _trm.active; }
    void terminal_stop() { _trm.active = false; }
    float terminal_get_best_dist() const { return float(_trm.best_dist_3d); }
    float terminal_get_best_h() const { return float(_trm.best_h_dist); }
    float terminal_get_best_v() const { return float(_trm.best_v_dist); }

    // AP_Param: all STT_ gains
    AP_Float mid_h_kp;
    AP_Float mid_h_ki;
    AP_Float mid_h_kd;
    AP_Float mid_p_kp;
    AP_Float mid_vref;
    AP_Float kf_q;
    AP_Float kf_r;
    AP_Float kf_maxr;
    AP_Float roll_kp;
    AP_Float roll_kd;
    AP_Float trm_vref;

    // terminal gain params (idx 12-26)
    AP_Float t_ff_p1;       // pitch feedforward gain (<300m)
    AP_Float t_ff_p2;       // pitch feedforward gain (300-500m)
    AP_Float t_ff_p3;       // pitch feedforward gain (>500m)
    AP_Float t_rff_g1;      // rate feedforward gain (<300m)
    AP_Float t_rff_g2;      // rate feedforward gain (>=300m)
    AP_Float t_yaw_ff;      // yaw feedforward gain
    AP_Float t_yaw_max;     // yaw command clamp
    AP_Float t_pn_p;        // PN pitch gain
    AP_Float t_pn_y;        // PN yaw gain
    AP_Float t_skr_p1;      // seeker pitch gain (<300m)
    AP_Float t_skr_p2;      // seeker pitch gain (300-1200m)
    AP_Float t_kd_p_cl;     // pitch damping (<200m)
    AP_Float t_kd_p_fr;     // pitch damping (>=200m)
    AP_Float t_kd_y_cl;     // yaw damping (<300m)
    AP_Float t_kd_y_fr;     // yaw damping (>=300m)

    // new params for terminal C++ mode and seeker source
    AP_Int8  skr_src;       // 0=real RS-422, 1=virtual GPS
    AP_Int8  trm_cpp;       // 0=Lua terminal, 1=C++ 300Hz terminal

private:
    // --- midcourse state ---
    static constexpr uint8_t PHASE_GLIDE = 1;
    static constexpr uint8_t PHASE_DIVE  = 2;

    struct MidState {
        double target_lat = 0;
        double target_lon = 0;
        double target_alt = 0;
        uint8_t phase = PHASE_GLIDE;
        double skr_dist = 1500;
        // PID state (double precision to match Lua)
        double heading_integral = 0;
        double heading_prev_err = 0;
        uint32_t heading_last_ms = 0;
        double imax = 0.5;
    } _mid;

    double _speed_gain_mid(double aspd) const;
    double _heading_pid_update(double err, uint32_t now_ms);

    // --- PN + KF state (double precision to match Lua's 64-bit floats) ---
    struct KFState {
        double x1 = 0;  // angle (deg)
        double x2 = 0;  // rate (deg/s)
        double p11 = 1, p12 = 0, p21 = 0, p22 = 400;
        bool inited = false;
    };

    struct PNState {
        double N = 4.0;
        double alpha = 0.1;  // API compat (KF ignores)
        double max_g = 15.0;
        double max_los_rate = 40.0;

        KFState pitch_kf;
        KFState yaw_kf;

        double prev_pitch_los = 0;
        double prev_yaw_los = 0;
        uint32_t prev_ms = 0;
        bool initialized = false;
        uint16_t frame_count = 0;

        // outputs
        double a_pitch = 0;
        double a_yaw = 0;
    } _pn;

    static constexpr double G = 9.81;
    static constexpr double P_RATE_INIT = 400.0;
    static constexpr double GATE_SQ = 25.0;  // 5 sigma

    void _kf_init(KFState &kf, double z);
    void _kf_predict(KFState &kf, double dt, double q_spec, double r_meas);
    bool _kf_update(KFState &kf, double z, double r_meas);
    void _pn_accel_to_cmd(double a_pitch, double a_yaw, float &pitch_cmd, float &yaw_cmd) const;

    // --- terminal guidance state ---
    struct TerminalState {
        bool active = false;
        uint8_t status = 0;       // 0=running, 1=impact
        double target_lat = 0;
        double target_lon = 0;
        double target_alt = 0;
        double best_dist_3d = 999999;
        double best_h_dist = 999999;
        double best_v_dist = 999999;
        double prev_dist_3d = 999999;
        uint32_t dist_inc_start_ms = 0;   // time-based flyover detection
        double last_yaw_cmd = 0;
        uint32_t last_ms = 0;
        // PN decimation: keep PN at ~50Hz while rest runs at 300Hz
        uint32_t last_pn_ms = 0;
        double pn_pitch_cmd = 0;
        double pn_yaw_cmd = 0;
    } _trm;

    // --- GPS helpers (double precision) ---
    static double _gps_distance(double lat1, double lon1, double lat2, double lon2);
    static double _gps_bearing(double lat1, double lon1, double lat2, double lon2);
    static double _angle_diff(double a, double b);
    static float _clamp(float v, float lo, float hi);
    static double _clampd(double v, double lo, double hi);

    static AP_STTGuidance *_singleton;
};

#endif  // AP_STT_GUIDANCE_ENABLED
