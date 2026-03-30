#pragma once

#include "AP_STTVirtualSeeker_config.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>

#if AP_STT_VIRTUAL_SEEKER_ENABLED

class AP_STTVirtualSeeker {
public:
    AP_STTVirtualSeeker();

    static AP_STTVirtualSeeker *get_singleton() { return _singleton; }
    static const struct AP_Param::GroupInfo var_info[];

    // update — compute LOS from AHRS position/attitude to carrier target
    void update();

    // state queries (same interface as AP_STTSeeker)
    bool is_captured() const { return _state == STATE_CAPTURED; }
    bool is_angle_valid() const { return _angle_valid; }
    float get_pitch_deg() const { return _los_pitch_deg; }
    float get_yaw_deg() const { return _los_yaw_deg; }
    float get_dist_3d() const { return _dist_3d; }
    float get_h_dist() const { return _h_dist; }

    // control commands (same interface as AP_STTSeeker)
    void allow_capture() { _capture_allowed = true; }
    void forbid_capture() { _capture_allowed = false; _state = STATE_SEARCH; _angle_valid = false; }
    void reset();

private:
    enum State : uint8_t {
        STATE_SEARCH = 0,
        STATE_CAPTURED = 1,
        STATE_LOST = 2,
    };

    State _state = STATE_SEARCH;
    bool _capture_allowed = false;
    bool _angle_valid = false;

    float _los_pitch_deg = 0;
    float _los_yaw_deg = 0;
    float _dist_3d = 0;
    float _h_dist = 0;

    uint32_t _lost_ms = 0;     // time when target was lost

    // AP_Param
    AP_Float _acq_rng;    // acquire range (m)
    AP_Float _cap_fov;    // capture half-angle (deg)
    AP_Float _trk_fov;    // tracking half-angle (deg)
    AP_Float _noise;      // Gaussian noise sigma (deg)
    AP_Float _blind;      // blind range (m)
    AP_Float _memory;     // memory time after loss (s)

    // PRNG state for Gaussian noise (Box-Muller)
    uint32_t _rng_state = 12345;
    float _gauss_spare = 0;
    bool _gauss_has_spare = false;
    float _rand_gauss();
    float _rand_uniform();

    static AP_STTVirtualSeeker *_singleton;
};

#endif  // AP_STT_VIRTUAL_SEEKER_ENABLED
