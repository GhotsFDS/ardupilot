#include "AP_STTVirtualSeeker.h"

#if AP_STT_VIRTUAL_SEEKER_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_STTCarrier/AP_STTCarrier.h>
#include <AP_Math/AP_Math.h>

AP_STTVirtualSeeker *AP_STTVirtualSeeker::_singleton = nullptr;

const AP_Param::GroupInfo AP_STTVirtualSeeker::var_info[] = {
    // @Param: ACQ_RNG
    // @DisplayName: Acquire range (m)
    // @Range: 100 5000
    AP_GROUPINFO("ACQ_RNG", 1, AP_STTVirtualSeeker, _acq_rng, 2500.0f),

    // @Param: CAP_FOV
    // @DisplayName: Capture half-angle (deg)
    // @Range: 1 30
    AP_GROUPINFO("CAP_FOV", 2, AP_STTVirtualSeeker, _cap_fov, 10.0f),

    // @Param: TRK_FOV
    // @DisplayName: Tracking half-angle (deg)
    // @Range: 5 45
    AP_GROUPINFO("TRK_FOV", 3, AP_STTVirtualSeeker, _trk_fov, 20.0f),

    // @Param: NOISE
    // @DisplayName: LOS Gaussian noise sigma (deg)
    // @Range: 0 2.0
    AP_GROUPINFO("NOISE", 4, AP_STTVirtualSeeker, _noise, 0.15f),

    // @Param: BLIND
    // @DisplayName: Blind range (m)
    // @Range: 0 200
    AP_GROUPINFO("BLIND", 5, AP_STTVirtualSeeker, _blind, 50.0f),

    // @Param: MEMORY
    // @DisplayName: Memory time after loss (s)
    // @Range: 0 2.0
    AP_GROUPINFO("MEMORY", 6, AP_STTVirtualSeeker, _memory, 0.6f),

    AP_GROUPEND
};

AP_STTVirtualSeeker::AP_STTVirtualSeeker()
{
    if (_singleton == nullptr) {
        _singleton = this;
    }
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_STTVirtualSeeker::reset()
{
    _state = STATE_SEARCH;
    _capture_allowed = false;
    _angle_valid = false;
    _los_pitch_deg = 0;
    _los_yaw_deg = 0;
    _dist_3d = 0;
    _h_dist = 0;
    _lost_ms = 0;
}

// simple xorshift PRNG for deterministic noise
float AP_STTVirtualSeeker::_rand_uniform()
{
    _rng_state ^= _rng_state << 13;
    _rng_state ^= _rng_state >> 17;
    _rng_state ^= _rng_state << 5;
    return float(_rng_state) / 4294967295.0f;
}

// Box-Muller Gaussian
float AP_STTVirtualSeeker::_rand_gauss()
{
    if (_gauss_has_spare) {
        _gauss_has_spare = false;
        return _gauss_spare;
    }
    float u1, u2;
    do {
        u1 = _rand_uniform();
    } while (u1 < 1e-10f);
    u2 = _rand_uniform();
    const float mag = sqrtf(-2.0f * logf(u1));
    const float angle = 2.0f * M_PI * u2;
    _gauss_spare = mag * sinf(angle);
    _gauss_has_spare = true;
    return mag * cosf(angle);
}

void AP_STTVirtualSeeker::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // need carrier target
    auto *carrier = AP_STTCarrier::get_singleton();
    if (carrier == nullptr || !carrier->has_target()) {
        _angle_valid = false;
        return;
    }

    // get missile position from AHRS
    auto &ahrs = AP::ahrs();
    Location loc;
    if (!ahrs.get_location(loc)) {
        _angle_valid = false;
        return;
    }

    const double m_lat = loc.lat * 1e-7;
    const double m_lon = loc.lng * 1e-7;
    const double m_alt = loc.alt * 0.01;  // cm → m
    const float m_hdg = degrees(ahrs.get_yaw());
    const float m_pitch = degrees(ahrs.get_pitch());

    // target coordinates (use int32 for double precision)
    const double t_lat = carrier->get_target_lat_int32() * 1e-7;
    const double t_lon = carrier->get_target_lon_int32() * 1e-7;
    const double t_alt = carrier->get_target_alt_cm() * 0.01;

    // horizontal distance (simple flat-earth for short ranges)
    constexpr double EARTH_R = 6371000.0;
    constexpr double RAD = M_PI / 180.0;
    const double dlat = (t_lat - m_lat) * RAD * EARTH_R;
    const double dlon = (t_lon - m_lon) * RAD * EARTH_R * cos(m_lat * RAD);
    _h_dist = float(sqrt(dlat * dlat + dlon * dlon));

    // altitude difference
    const double dalt = t_alt - m_alt;

    // 3D distance
    _dist_3d = float(sqrt(double(_h_dist) * double(_h_dist) + dalt * dalt));

    // bearing to target
    const float bearing = degrees(atan2f(float(dlon), float(dlat)));

    // target elevation from horizontal
    float target_elev;
    if (_h_dist > 0.1f) {
        target_elev = degrees(atan2f(float(dalt), _h_dist));
    } else {
        target_elev = (dalt < 0) ? -90.0f : 0.0f;
    }

    // LOS angles = target direction - missile axis
    float los_pitch = target_elev - m_pitch;
    float los_yaw = wrap_180(bearing - m_hdg);

    // add noise
    const float sigma = _noise.get();
    if (sigma > 0) {
        los_pitch += _rand_gauss() * sigma;
        los_yaw += _rand_gauss() * sigma;
    }

    // blind range check
    const bool in_blind = (_dist_3d < _blind.get());

    // state machine
    switch (_state) {
    case STATE_SEARCH:
        if (_capture_allowed &&
            _h_dist < _acq_rng.get() &&
            fabsf(los_pitch) < _cap_fov.get() &&
            fabsf(los_yaw) < _cap_fov.get() &&
            !in_blind) {
            _state = STATE_CAPTURED;
            _angle_valid = true;
            _los_pitch_deg = los_pitch;
            _los_yaw_deg = los_yaw;
        }
        break;

    case STATE_CAPTURED:
        if (in_blind) {
            // enter blind zone — hold last angles
            _state = STATE_LOST;
            _lost_ms = now_ms;
            // keep _angle_valid true during memory period
        } else if (fabsf(los_pitch) > _trk_fov.get() ||
                   fabsf(los_yaw) > _trk_fov.get()) {
            // lost tracking
            _state = STATE_LOST;
            _lost_ms = now_ms;
        } else {
            _los_pitch_deg = los_pitch;
            _los_yaw_deg = los_yaw;
            _angle_valid = true;
        }
        break;

    case STATE_LOST: {
        const float memory_ms = _memory.get() * 1000.0f;
        if ((now_ms - _lost_ms) > uint32_t(memory_ms)) {
            _angle_valid = false;
            // stay in LOST, don't reacquire automatically
        }
        // during memory period, keep old angles valid
        break;
    }
    }
}

#endif  // AP_STT_VIRTUAL_SEEKER_ENABLED
