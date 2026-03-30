#include "AP_STTGuidance.h"

#if AP_STT_GUIDANCE_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_STTSeeker/AP_STTSeeker.h>
#include <AP_STTCarrier/AP_STTCarrier.h>
#include <SRV_Channel/SRV_Channel.h>

#include <AP_STTVirtualSeeker/AP_STTVirtualSeeker_config.h>
#if AP_STT_VIRTUAL_SEEKER_ENABLED
#include <AP_STTVirtualSeeker/AP_STTVirtualSeeker.h>
#endif

void AP_STTGuidance::terminal_init(float target_lat, float target_lon, float target_alt)
{
    // prefer carrier int32 for double precision (avoid float truncation)
    auto *carrier = AP_STTCarrier::get_singleton();
    if (carrier != nullptr && carrier->has_target()) {
        _trm.target_lat = double(carrier->get_target_lat_int32()) * 1e-7;
        _trm.target_lon = double(carrier->get_target_lon_int32()) * 1e-7;
        _trm.target_alt = double(carrier->get_target_alt_cm()) * 0.01;
    } else {
        _trm.target_lat = double(target_lat);
        _trm.target_lon = double(target_lon);
        _trm.target_alt = double(target_alt);
    }
    _trm.best_dist_3d = 999999;
    _trm.best_h_dist = 999999;
    _trm.best_v_dist = 999999;
    _trm.prev_dist_3d = 999999;
    _trm.dist_inc_start_ms = 0;
    _trm.last_yaw_cmd = 0;
    _trm.last_ms = 0;
    _trm.last_pn_ms = 0;
    _trm.pn_pitch_cmd = 0;
    _trm.pn_yaw_cmd = 0;
    _trm.status = 0;
    // only activate C++ terminal path if TRM_CPP=1
    _trm.active = (trm_cpp.get() == 1);
}

uint8_t AP_STTGuidance::terminal_update(uint32_t now_ms)
{
    if (!_trm.active) {
        return 0;
    }
    if (_trm.status == 1) {
        return 1;  // impact already detected
    }

    // --- dt calculation ---
    double dt = 0;
    if (_trm.last_ms != 0) {
        const int32_t dt_ms = int32_t(now_ms - _trm.last_ms);
        if (dt_ms <= 0 || dt_ms > 100) {
            _trm.last_ms = now_ms;
            return 0;
        }
        dt = dt_ms * 0.001;
    }
    _trm.last_ms = now_ms;

    // --- read AHRS ---
    auto &ahrs = AP::ahrs();
    Location loc;
    if (!ahrs.get_location(loc)) {
        return 0;
    }
    const double lat = loc.lat * 1e-7;
    const double lon = loc.lng * 1e-7;
    const double alt = loc.alt * 0.01;

    const double heading_deg = fmod(degrees(double(ahrs.get_yaw())) + 360.0, 360.0);
    const double pitch_deg = degrees(double(ahrs.get_pitch()));
    const double roll_deg = degrees(double(ahrs.get_roll()));

    Vector3f vel_ned;
    if (!ahrs.get_velocity_NED(vel_ned)) {
        return 0;
    }
    const double vn = double(vel_ned.x);
    const double ve = double(vel_ned.y);
    const double vd = double(vel_ned.z);
    const double aspd = sqrt(vn*vn + ve*ve + vd*vd);
    const double aspd_c = fmax(aspd, 30.0);

    // ground track
    const double gspd = sqrt(vn*vn + ve*ve);
    double gtrack;
    if (gspd < 10.0) {
        gtrack = heading_deg;
    } else {
        gtrack = fmod(degrees(atan2(ve, vn)) + 360.0, 360.0);
    }

    // gyro rates
    const Vector3f &gyro_vec = ahrs.get_gyro();
    const double pitch_rate_dps = degrees(double(gyro_vec.y));
    const double yaw_rate_dps = degrees(double(gyro_vec.z));

    // --- gains ---
    const double v_ref = double(trm_vref.get());
    const double r = v_ref / aspd_c;
    const double sg = fmin(r * r, 4.0);
    const double sg_lin = fmin(v_ref / aspd_c, 2.0);

    // --- GPS geometry ---
    const double t_lat = _trm.target_lat;
    const double t_lon = _trm.target_lon;
    const double t_alt = _trm.target_alt;

    const double h_dist = _gps_distance(lat, lon, t_lat, t_lon);
    const double bearing = _gps_bearing(lat, lon, t_lat, t_lon);
    const double d_alt = alt - t_alt;
    double geo_angle = 0;
    if (h_dist > 1.0) {
        geo_angle = degrees(atan2(d_alt, h_dist));
    }

    // --- GPS-based inertial PN (decimated to ~50Hz — KF Q/R tuned for 50Hz) ---
    if (now_ms - _trm.last_pn_ms >= 20) {  // 20ms = 50Hz
        _trm.last_pn_ms = now_ms;
        const double pn_pitch_los = -geo_angle;
        const double pn_yaw_los = bearing;
        const double Vc = _clampd(aspd, 50.0, 500.0);
        float pn_pitch_cmd_f, pn_yaw_cmd_f;
        pn_update(float(pn_pitch_los), float(pn_yaw_los), float(Vc), now_ms,
                  pn_pitch_cmd_f, pn_yaw_cmd_f);
        _trm.pn_pitch_cmd = double(pn_pitch_cmd_f);
        _trm.pn_yaw_cmd = double(pn_yaw_cmd_f);
    }
    const double pn_pitch_cmd = _trm.pn_pitch_cmd;
    const double pn_yaw_cmd = _trm.pn_yaw_cmd;

    // --- geometry feedforward pitch ---
    const double target_pitch = -_clampd(geo_angle, 0.0, 70.0);
    double pitch_gain;
    if (h_dist < 300.0) {
        pitch_gain = double(t_ff_p1.get()) * sg_lin;
    } else if (h_dist < 500.0) {
        pitch_gain = double(t_ff_p2.get()) * sg_lin;
    } else {
        pitch_gain = double(t_ff_p3.get()) * sg_lin;
    }
    const double pitch_err = target_pitch - pitch_deg;
    const double v_h = fmax(aspd * cos(radians(fmin(fabs(pitch_deg), 80.0))), 30.0);
    const double geo_rate_dps = v_h * d_alt / fmax(h_dist * h_dist + d_alt * d_alt, 100.0) * 57.3;
    const double rate_ff_gain = (h_dist < 300.0) ? double(t_rff_g1.get()) : double(t_rff_g2.get());
    const double rate_ff = -geo_rate_dps * rate_ff_gain * sg_lin;
    const double ff_pitch_cmd = _clampd(pitch_err * pitch_gain + rate_ff, -1.0, 1.0);

    // --- yaw feedforward (range-gated) ---
    const double heading_err = _angle_diff(bearing, gtrack);
    const double yaw_gate = _clampd((h_dist - 500.0) / 500.0, 0.0, 1.0);
    const double ff_yaw_cmd = _clampd(heading_err * double(t_yaw_ff.get()) * yaw_gate * sg, -1.0, 1.0);

    // --- flyover detection (time-based: 100ms of distance increasing) ---
    const double dist_3d = sqrt(h_dist * h_dist + d_alt * d_alt);
    if (dist_3d < _trm.best_dist_3d) {
        _trm.best_dist_3d = dist_3d;
        _trm.best_h_dist = h_dist;
        _trm.best_v_dist = fabs(d_alt);
    }
    if (dist_3d > _trm.prev_dist_3d) {
        if (_trm.dist_inc_start_ms == 0) {
            _trm.dist_inc_start_ms = now_ms;
        }
    } else {
        _trm.dist_inc_start_ms = 0;
    }
    _trm.prev_dist_3d = dist_3d;

    const bool flyover = (_trm.dist_inc_start_ms != 0 &&
                          (now_ms - _trm.dist_inc_start_ms) >= 100);
    if (flyover || alt <= t_alt) {
        _trm.status = 1;
        // keep active=true so Lua can detect status=1 via terminal_is_active()
        // set neutral outputs
        SRV_Channels::set_output_norm(SRV_Channel::k_scripting2, 0);
        SRV_Channels::set_output_norm(SRV_Channel::k_scripting3, 0);
        SRV_Channels::set_output_norm(SRV_Channel::k_scripting4, 0);
        return 1;
    }

    // --- composite command ---
    double pitch_cmd = _clampd(ff_pitch_cmd + pn_pitch_cmd * sg * double(t_pn_p.get()), -1.0, 1.0);
    double yaw_cmd = _clampd(ff_yaw_cmd + pn_yaw_cmd * sg * double(t_pn_y.get()), -1.0, 1.0);

    // --- seeker angle feedback ---
    // select seeker source based on SKR_SRC param
    bool skr_valid = false;
    float skr_pitch = 0;
    // float skr_yaw = 0;  // yaw feedback disabled (c_y_b=-4.0)

    if (skr_src.get() == 0) {
        // real RS-422 seeker
#if AP_STT_SEEKER_ENABLED
        auto *seeker = AP_STTSeeker::get_singleton();
        if (seeker != nullptr && seeker->is_angle_valid()) {
            skr_valid = true;
            skr_pitch = seeker->get_pitch_deg();
        }
#endif
    } else {
        // virtual GPS seeker
#if AP_STT_VIRTUAL_SEEKER_ENABLED
        auto *vseeker = AP_STTVirtualSeeker::get_singleton();
        if (vseeker != nullptr && vseeker->is_angle_valid()) {
            skr_valid = true;
            skr_pitch = vseeker->get_pitch_deg();
        }
#endif
    }

    if (skr_valid && h_dist < 300.0) {
        const double K_pitch = double(t_skr_p1.get()) * sg_lin *
                               _clampd(300.0 / fmax(h_dist, 20.0), 1.0, 8.0);
        pitch_cmd = _clampd(pitch_cmd - double(skr_pitch) * K_pitch, -1.0, 1.0);
    } else if (skr_valid && h_dist < 1200.0) {
        pitch_cmd = _clampd(pitch_cmd - double(skr_pitch) * double(t_skr_p2.get()), -1.0, 1.0);
    }

    // --- rate damping ---
    {
        double kd_pitch = (h_dist < 200.0) ? double(t_kd_p_cl.get()) : double(t_kd_p_fr.get());
        double kd_yaw = (h_dist < 300.0) ? double(t_kd_y_cl.get()) : double(t_kd_y_fr.get());
        if (aspd > v_ref) {
            kd_pitch *= v_ref / aspd;
            kd_yaw *= v_ref / aspd;
        }
        pitch_cmd = _clampd(pitch_cmd - pitch_rate_dps * kd_pitch, -1.0, 1.0);
        yaw_cmd = _clampd(yaw_cmd - yaw_rate_dps * kd_yaw, -1.0, 1.0);
    }

    // --- yaw clamp + rate limit (time-based, 300Hz compatible) ---
    const double yaw_max_val = double(t_yaw_max.get());
    yaw_cmd = _clampd(yaw_cmd, -yaw_max_val, yaw_max_val);

    // rate limit: 0.25 deg/s equivalent at any frequency
    // Lua was 0.005/frame@50Hz = 0.25/s for close, 0.003/frame@50Hz = 0.15/s for far
    if (dt > 0) {
        const double max_rate = (h_dist < 300.0) ? 0.25 : 0.15;  // per second
        const double max_delta = max_rate * dt;
        const double delta = yaw_cmd - _trm.last_yaw_cmd;
        if (fabs(delta) > max_delta) {
            yaw_cmd = _trm.last_yaw_cmd + (delta > 0 ? max_delta : -max_delta);
        }
    }
    _trm.last_yaw_cmd = yaw_cmd;

    // --- roll-dependent yaw attenuation ---
    const double roll_abs = fabs(roll_deg);
    if (roll_abs > 15.0) {
        const double atten = fmax(1.0 - (roll_abs - 15.0) / 30.0, 0.0);
        yaw_cmd *= atten;
    }

    // --- roll PD ---
    const float roll_rate_dps = degrees(gyro_vec.x);
    const float roll_cmd = roll_stabilize(float(roll_deg), roll_rate_dps,
                                           float(aspd), float(v_ref));

    // --- output to SRV_Channels ---
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting2, float(pitch_cmd));
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting3, roll_cmd);
    SRV_Channels::set_output_norm(SRV_Channel::k_scripting4, float(yaw_cmd));

    return 0;
}

#endif  // AP_STT_GUIDANCE_ENABLED
