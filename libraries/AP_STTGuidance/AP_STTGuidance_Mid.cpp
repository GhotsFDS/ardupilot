#include "AP_STTGuidance.h"

#if AP_STT_GUIDANCE_ENABLED

#include <GCS_MAVLink/GCS.h>

// --- midcourse speed gain: V² scaling (double precision) ---

double AP_STTGuidance::_speed_gain_mid(double aspd) const
{
    const double vref = double(mid_vref.get());
    if (aspd <= vref) {
        return 1.0;
    }
    const double r = vref / aspd;
    return r * r;
}

// --- heading PID (double precision, matches Lua utils.pid_update) ---

double AP_STTGuidance::_heading_pid_update(double err, uint32_t now_ms)
{
    const double kp = double(mid_h_kp.get());
    const double ki = double(mid_h_ki.get());
    const double kd = double(mid_h_kd.get());

    if (_mid.heading_last_ms == 0) {
        _mid.heading_last_ms = now_ms;
        _mid.heading_prev_err = err;
        return kp * err;
    }

    const double dt = (now_ms - _mid.heading_last_ms) / 1000.0;
    if (dt <= 0 || dt > 1.0) {
        _mid.heading_last_ms = now_ms;
        _mid.heading_prev_err = err;
        return kp * err;
    }

    const double p_term = kp * err;

    _mid.heading_integral += err * dt;
    _mid.heading_integral = _clampd(_mid.heading_integral, -_mid.imax, _mid.imax);
    const double i_term = ki * _mid.heading_integral;

    const double d_term = kd * (err - _mid.heading_prev_err) / dt;

    _mid.heading_prev_err = err;
    _mid.heading_last_ms = now_ms;

    return p_term + i_term + d_term;
}

// --- midcourse init ---

void AP_STTGuidance::mid_init(float target_lat, float target_lon, float target_alt, float skr_dist)
{
    _mid.target_lat = double(target_lat);
    _mid.target_lon = double(target_lon);
    _mid.target_alt = double(target_alt);
    _mid.phase = PHASE_GLIDE;

    if (skr_dist > 0) {
        _mid.skr_dist = double(skr_dist);
    }

    // reset heading PID
    _mid.heading_integral = 0;
    _mid.heading_prev_err = 0;
    _mid.heading_last_ms = 0;
    _mid.imax = 0.5;
}

// --- midcourse update ---

bool AP_STTGuidance::mid_update(float lat, float lon, float alt,
                                 float heading_deg, float pitch_deg, float airspeed,
                                 uint32_t now_ms,
                                 float &pitch_cmd, float &yaw_cmd)
{
    // cast inputs to double for precision
    const double dlat = double(lat);
    const double dlon = double(lon);
    const double dalt = double(alt);
    const double dhdg = double(heading_deg);
    const double dpitch = double(pitch_deg);
    const double daspd = double(airspeed);

    const double h_dist = _gps_distance(dlat, dlon, _mid.target_lat, _mid.target_lon);
    const double bearing = _gps_bearing(dlat, dlon, _mid.target_lat, _mid.target_lon);

    const double sg = _speed_gain_mid(daspd);

    // yaw: PID tracking of bearing (speed-scaled)
    const double heading_err = _angle_diff(bearing, dhdg);
    const double yaw_raw = _heading_pid_update(heading_err, now_ms) * sg;
    yaw_cmd = float(_clampd(yaw_raw, -1.0, 1.0));

    // phase transition: GLIDE -> DIVE
    if (_mid.phase == PHASE_GLIDE && h_dist <= _mid.skr_dist) {
        _mid.phase = PHASE_DIVE;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "STT: GLIDE->DIVE dist=%.0fm alt=%.0fm",
                      float(h_dist), alt);
    }

    // geometry angle
    const double d_alt = dalt - _mid.target_alt;
    double geo_angle = 0;
    if (h_dist > 1.0) {
        geo_angle = atan2(d_alt, h_dist) * (180.0 / M_PI);
    }

    const double max_angle = (_mid.phase == PHASE_GLIDE) ? 30.0 : 70.0;
    geo_angle = _clampd(geo_angle, 0, max_angle);

    const double target_pitch = -geo_angle;
    const double pitch_err = target_pitch - dpitch;
    pitch_cmd = float(_clampd(pitch_err * double(mid_p_kp.get()), -0.8, 0.8));

    return true;
}

// --- distance query ---

float AP_STTGuidance::mid_get_distance(float lat, float lon) const
{
    return float(_gps_distance(double(lat), double(lon), _mid.target_lat, _mid.target_lon));
}

// --- heading integral query ---

float AP_STTGuidance::mid_get_heading_integral() const
{
    return float(_mid.heading_integral);
}

// --- yaw-only update (for terminal reuse of heading PID) ---

bool AP_STTGuidance::mid_yaw_update(float heading_deg, float bearing_deg, float airspeed,
                                     uint32_t now_ms, float &yaw_cmd)
{
    const double heading_err = _angle_diff(double(bearing_deg), double(heading_deg));
    const double daspd = double(airspeed);
    double sg = 1.0;
    const double vref = double(mid_vref.get());
    if (daspd > vref) {
        const double r = vref / daspd;
        sg = r * r;
    }
    const double raw = _heading_pid_update(heading_err, now_ms) * sg;
    yaw_cmd = float(_clampd(raw, -1.0, 1.0));
    return true;
}

#endif  // AP_STT_GUIDANCE_ENABLED
