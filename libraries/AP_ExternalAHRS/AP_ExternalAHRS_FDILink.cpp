/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for FDISystems FDILink protocol AHRS/IMU units (e.g. DETA40)
  used as an external IMU + compass source. see AP_ExternalAHRS_FDILink.h
  for the frame layout.
 */

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_FDILINK_ENABLED

#include "AP_ExternalAHRS_FDILink.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL &hal;

#define FDILINK_FRAME_HEAD  0xFC
#define FDILINK_FRAME_END   0xFD
#define FDILINK_TYPE_IMU    0x40
#define FDILINK_TYPE_AHRS   0x41
#define FDILINK_TYPE_INSGPS 0x42
#define FDILINK_TYPE_GEOPOS 0x5C
#define FDILINK_HEADER_LEN  7

// payload of type 0x40, little-endian
struct PACKED FDILink_imu_payload {
    float gyro[3];        // rad/s, body FRD
    float accel[3];       // m/s^2, body FRD
    float mag[3];         // milliGauss, body FRD
    float imu_temp;       // degC
    float pressure;       // Pa. constant 101325 placeholder on DETA40, never used
    float pressure_temp;  // degC, placeholder
    int64_t timestamp_us; // device time since power-up
};
static_assert(sizeof(FDILink_imu_payload) == 56, "FDILink IMU payload must be 56 bytes");

// payload of type 0x41, little-endian
struct PACKED FDILink_ahrs_payload {
    float roll_speed;     // rad/s
    float pitch_speed;    // rad/s
    float heading_speed;  // rad/s
    float roll;           // rad
    float pitch;          // rad
    float heading;        // rad
    float q[4];           // wxyz, rotates body FRD into NED
    int64_t timestamp_us; // device time since power-up
};
static_assert(sizeof(FDILink_ahrs_payload) == 48, "FDILink AHRS payload must be 48 bytes");

// payload of type 0x42 (INS mode only), little-endian.
// note: local-NED location fields have a device-internal origin and no
// lat/lon; global position arrives separately as type 0x5C
struct PACKED FDILink_insgps_payload {
    float body_vel[3];    // m/s, body frame
    float body_accel[3];  // m/s^2, body frame
    float location_ned[3];// m, local NED (origin device-internal, logged only)
    float vel_ned[3];     // m/s, NED
    float accel_ned[3];   // m/s^2, NED
    float pressure_alt;   // m
    int64_t timestamp_us; // device time since power-up
};
static_assert(sizeof(FDILink_insgps_payload) == 72, "FDILink INSGPS payload must be 72 bytes");

// payload of type 0x5C (INS mode only), little-endian
struct PACKED FDILink_geopos_payload {
    double latitude_rad;  // rad (vendor ROS driver divides by DEG_TO_RAD)
    double longitude_rad; // rad
    double height_m;      // m
    float hacc_m;         // m
    float vacc_m;         // m
};
static_assert(sizeof(FDILink_geopos_payload) == 32, "FDILink GEOPOS payload must be 32 bytes");

// constructor
AP_ExternalAHRS_FDILink::AP_ExternalAHRS_FDILink(AP_ExternalAHRS *_frontend,
                                                 AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "FDILink ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    // pure IMU + compass source: no GNSS, and the pressure field is a
    // placeholder constant on DETA40 so no baro either
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::IMU) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_FDILink::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("FDILink Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FDILink ExternalAHRS initialised");
}

void AP_ExternalAHRS_FDILink::update_thread()
{
    if (!port_open) {
        uart->begin(baudrate);
        port_open = true;
    }
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay_microseconds(500);
        }
    }
}

/*
  drop the first byte of the buffer and shift to the next candidate
  frame head. only call with sem held and pktoffset > 0
 */
void AP_ExternalAHRS_FDILink::resync_buffer(void)
{
    resync_count++;
    const uint8_t *p = (const uint8_t *)memchr(&pktbuf[1], FDILINK_FRAME_HEAD, pktoffset - 1U);
    if (p != nullptr) {
        const uint16_t newlen = pktoffset - (p - pktbuf);
        memmove(&pktbuf[0], p, newlen);
        pktoffset = newlen;
    } else {
        pktoffset = 0;
    }
}

/*
  read from the UART and parse any complete frames.
  returns true if progress was made (data read or frame consumed)
 */
bool AP_ExternalAHRS_FDILink::check_uart()
{
    WITH_SEMAPHORE(sem);
    if (!port_open) {
        return false;
    }

    bool progress = false;

    const uint32_t n = uart->available();
    if (n > 0 && pktoffset < sizeof(pktbuf)) {
        const ssize_t nread = uart->read(&pktbuf[pktoffset], MIN(n, uint32_t(sizeof(pktbuf) - pktoffset)));
        if (nread > 0) {
            pktoffset += nread;
            progress = true;
        }
    }

    while (pktoffset > 0) {
        // align buffer start to a frame head byte
        if (pktbuf[0] != FDILINK_FRAME_HEAD) {
            const uint8_t *p = (const uint8_t *)memchr(pktbuf, FDILINK_FRAME_HEAD, pktoffset);
            if (p == nullptr) {
                pktoffset = 0;
                break;
            }
            const uint16_t newlen = pktoffset - (p - pktbuf);
            memmove(&pktbuf[0], p, newlen);
            pktoffset = newlen;
        }
        if (pktoffset < FDILINK_HEADER_LEN) {
            // wait for a full header
            break;
        }

        // header CRC8 over head|type|len|sn. identical table to the vendor
        // driver (Maxim/Dallas, init 0)
        if (crc8_maxim(pktbuf, 4) != pktbuf[4]) {
            resync_buffer();
            continue;
        }

        const uint8_t frame_type = pktbuf[1];
        const uint8_t flen = pktbuf[2];
        if (flen > sizeof(pktbuf) - FDILINK_HEADER_LEN - 1U) {
            // cannot ever buffer this frame; treat as garbage
            resync_buffer();
            continue;
        }
        const uint16_t frame_len = FDILINK_HEADER_LEN + flen + 1U;
        if (pktoffset < frame_len) {
            // wait for the full frame
            break;
        }

        const bool known_type = (frame_type == FDILINK_TYPE_IMU && flen == sizeof(FDILink_imu_payload)) ||
                                (frame_type == FDILINK_TYPE_AHRS && flen == sizeof(FDILink_ahrs_payload)) ||
                                (frame_type == FDILINK_TYPE_INSGPS && flen == sizeof(FDILink_insgps_payload)) ||
                                (frame_type == FDILINK_TYPE_GEOPOS && flen == sizeof(FDILink_geopos_payload));

        if (pktbuf[FDILINK_HEADER_LEN + flen] != FDILINK_FRAME_END) {
            if (known_type) {
                crc_fail_count++;
                last_crc_error_ms = AP_HAL::millis();
            }
            resync_buffer();
            continue;
        }

        const uint8_t *payload = &pktbuf[FDILINK_HEADER_LEN];
        // payload CRC16, big-endian in header bytes 5(H)/6(L).
        // XMODEM: poly 0x1021, init 0, MSB first - matches the vendor table
        const uint16_t crc16_expect = uint16_t(pktbuf[5] << 8) | pktbuf[6];
        if (crc_xmodem(payload, flen) != crc16_expect) {
            if (known_type) {
                crc_fail_count++;
                last_crc_error_ms = AP_HAL::millis();
            }
            resync_buffer();
            continue;
        }

        if (known_type) {
            // IMU/AHRS frames share one serial number sequence (verified on
            // captured DETA40 data: 0 gaps over 6200 frames); the vendor ROS
            // driver treats 0x42/0x5C as part of the same sequence, so track
            // them too (to be re-confirmed outdoors in INS mode). the 1Hz
            // 0xF0 ground frame runs an independent counter (verified) and
            // must not take part in loss tracking
            const uint8_t sn = pktbuf[3];
            if (sn_valid) {
                const uint8_t delta = uint8_t(sn - last_sn);
                if (delta != 1) {
                    // delta==0 (duplicated sn) counts as a full wrap
                    sn_lost_count += (delta == 0) ? 255U : (uint32_t(delta) - 1U);
                }
            }
            last_sn = sn;
            sn_valid = true;

            switch (frame_type) {
            case FDILINK_TYPE_IMU:
                process_imu_packet(payload);
                break;
            case FDILINK_TYPE_AHRS:
                process_ahrs_packet(payload);
                break;
            case FDILINK_TYPE_INSGPS:
                process_insgps_packet(payload);
                break;
            case FDILINK_TYPE_GEOPOS:
                process_geopos_packet(payload);
                break;
            }
        } else {
            // e.g. 0xF0 ground frame at 1Hz on DETA40
            skip_frame_count++;
        }

        memmove(&pktbuf[0], &pktbuf[frame_len], pktoffset - frame_len);
        pktoffset -= frame_len;
        progress = true;
    }

    update_nav_timeout();
    log_status();

    return progress;
}

void AP_ExternalAHRS_FDILink::process_imu_packet(const uint8_t *payload)
{
    FDILink_imu_payload pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    last_imu_pkt_ms = AP_HAL::millis();
    imu_frame_count++;

    // device body frame is FRD == ArduPilot body conventions, no remapping
    const Vector3f accel{pkt.accel[0], pkt.accel[1], pkt.accel[2]};
    const Vector3f gyro{pkt.gyro[0], pkt.gyro[1], pkt.gyro[2]};

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accel;
        state.gyro = gyro;
    }

    {
        const AP_ExternalAHRS::ins_data_message_t ins {
            accel: accel,
            gyro: gyro,
            temperature: pkt.imu_temp,
        };
        AP::ins().handle_external(ins);
    }

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    {
        // device outputs milliGauss, which is what the compass library expects
        const AP_ExternalAHRS::mag_data_message_t mag {
            field: Vector3f{pkt.mag[0], pkt.mag[1], pkt.mag[2]},
        };
        AP::compass().handle_external(mag);
    }
#endif

    // deliberately no AP::baro().handle_external(): the DETA40 pressure
    // field is a constant 101325 Pa placeholder

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: FDIM
    // @Description: FDILink IMU data
    // @Field: TimeUS: Time since system startup
    // @Field: DTS: device timestamp
    // @Field: GX: Rotation rate X-axis
    // @Field: GY: Rotation rate Y-axis
    // @Field: GZ: Rotation rate Z-axis
    // @Field: AX: Acceleration X-axis
    // @Field: AY: Acceleration Y-axis
    // @Field: AZ: Acceleration Z-axis
    // @Field: MX: Magnetic field X-axis, milliGauss
    // @Field: MY: Magnetic field Y-axis, milliGauss
    // @Field: MZ: Magnetic field Z-axis, milliGauss
    // @Field: Temp: IMU temperature
    AP::logger().WriteStreaming("FDIM", "TimeUS,DTS,GX,GY,GZ,AX,AY,AZ,MX,MY,MZ,Temp",
                                "ssEEEooo---O", "FF0000000000",
                                "QQffffffffff",
                                AP_HAL::micros64(),
                                uint64_t(pkt.timestamp_us),
                                gyro.x, gyro.y, gyro.z,
                                accel.x, accel.y, accel.z,
                                pkt.mag[0], pkt.mag[1], pkt.mag[2],
                                pkt.imu_temp);
#endif  // HAL_LOGGING_ENABLED
}

void AP_ExternalAHRS_FDILink::process_ahrs_packet(const uint8_t *payload)
{
    FDILink_ahrs_payload pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    last_ahrs_pkt_ms = AP_HAL::millis();
    ahrs_frame_count++;

    if (state_feed_enabled()) {
        // wxyz quaternion rotating body FRD into NED (verified on captured
        // data). only used by AP_AHRS when AHRS_EKF_TYPE=11 (EXTERNAL);
        // with EKF3 active this is shadow state for comparison/logging.
        // note: the device yaw is magnetic - see header comment
        WITH_SEMAPHORE(state.sem);
        state.quat = Quaternion{pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]};
        state.have_quaternion = true;
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: FDAT
    // @Description: FDILink device attitude data
    // @Field: TimeUS: Time since system startup
    // @Field: DTS: device timestamp
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler heading
    // @Field: Q1: quaternion w
    // @Field: Q2: quaternion x
    // @Field: Q3: quaternion y
    // @Field: Q4: quaternion z
    AP::logger().WriteStreaming("FDAT", "TimeUS,DTS,Roll,Pitch,Yaw,Q1,Q2,Q3,Q4",
                                "ssddd----", "FF0000000",
                                "QQfffffff",
                                AP_HAL::micros64(),
                                uint64_t(pkt.timestamp_us),
                                degrees(pkt.roll), degrees(pkt.pitch), degrees(pkt.heading),
                                pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
#endif  // HAL_LOGGING_ENABLED
}

// 0x42 INSGPS (INS mode): NED velocity into state; local-NED position and
// pressure altitude are device-internal (logged via FDNV, not used)
void AP_ExternalAHRS_FDILink::process_insgps_packet(const uint8_t *payload)
{
    FDILink_insgps_payload pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    last_insgps_pkt_ms = AP_HAL::millis();
    insgps_frame_count++;
    latest_palt_m = pkt.pressure_alt;
    memcpy(latest_vel_ned, pkt.vel_ned, sizeof(latest_vel_ned));

    if (state_feed_enabled()) {
        WITH_SEMAPHORE(state.sem);
        state.velocity = Vector3f{pkt.vel_ned[0], pkt.vel_ned[1], pkt.vel_ned[2]};
        state.have_velocity = true;
    }
}

// 0x5C GEODETIC_POS (INS mode): global position into state
void AP_ExternalAHRS_FDILink::process_geopos_packet(const uint8_t *payload)
{
    FDILink_geopos_payload pkt;
    memcpy(&pkt, payload, sizeof(pkt));

    last_geopos_pkt_ms = AP_HAL::millis();
    geopos_frame_count++;

    // vendor sends lat/lon in radians (ROS driver divides by DEG_TO_RAD).
    // keep the whole conversion in double: float would quantise at ~0.4m
    // and destroy the RTK-level accuracy of the aided INS solution
    const Location loc{
        int32_t(pkt.latitude_rad * RAD_TO_DEG_DOUBLE * 1.0e7),
        int32_t(pkt.longitude_rad * RAD_TO_DEG_DOUBLE * 1.0e7),
        int32_t(pkt.height_m * 1.0e2),
        Location::AltFrame::ABSOLUTE
    };

    if (state_feed_enabled()) {
        WITH_SEMAPHORE(state.sem);
        state.location = loc;
        state.last_location_update_us = AP_HAL::micros();
        state.have_location = true;
        if (!state.have_origin) {
            state.origin = loc;
            state.have_origin = true;
        }
    }

#if HAL_LOGGING_ENABLED
    // @LoggerMessage: FDNV
    // @Description: FDILink INS navigation data
    // @Field: TimeUS: Time since system startup
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Hgt: height AMSL
    // @Field: HAcc: horizontal position accuracy
    // @Field: VAcc: vertical position accuracy
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PAlt: pressure altitude
    AP::logger().WriteStreaming("FDNV", "TimeUS,Lat,Lon,Hgt,HAcc,VAcc,VN,VE,VD,PAlt",
                                "sDUmmmnnnm", "FGG0000000",
                                "QLLfffffff",
                                AP_HAL::micros64(),
                                loc.lat, loc.lng, float(pkt.height_m),
                                pkt.hacc_m, pkt.vacc_m,
                                latest_vel_ned[0], latest_vel_ned[1], latest_vel_ned[2],
                                latest_palt_m);
#endif  // HAL_LOGGING_ENABLED
}

/*
  layered nav feed: while 0x42+0x5C are fresh (within EAHRS_NAV_TMO) the
  backend claims position/velocity. on timeout it withdraws those claims so
  the vehicle falls back to attitude-only external AHRS: on ArduPlane the
  fixed-wing fallback in AP_AHRS::_active_EKF_type() then demotes to DCM,
  which navigates on the FC's own GPS.
 */
void AP_ExternalAHRS_FDILink::update_nav_timeout(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    const uint32_t tmo = get_nav_timeout_ms();
    const bool fresh = last_insgps_pkt_ms != 0 && last_geopos_pkt_ms != 0 &&
                       now_ms - last_insgps_pkt_ms < tmo &&
                       now_ms - last_geopos_pkt_ms < tmo;
    if (fresh == nav_feed_active) {
        return;
    }
    nav_feed_active = fresh;
    if (fresh) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FDILink: INS nav active");
    } else {
        // withdraw position/velocity claims; each new 0x42/0x5C frame
        // re-latches them, so recovery is automatic
        {
            WITH_SEMAPHORE(state.sem);
            state.have_velocity = false;
            state.have_location = false;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FDILink: INS nav timeout, attitude-only");
    }
}

// 1Hz stream health counters
void AP_ExternalAHRS_FDILink::log_status(void)
{
#if HAL_LOGGING_ENABLED
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_status_log_ms < 1000) {
        return;
    }
    last_status_log_ms = now_ms;

    // @LoggerMessage: FDIS
    // @Description: FDILink stream statistics
    // @Field: TimeUS: Time since system startup
    // @Field: NI: valid IMU frames
    // @Field: NA: valid AHRS frames
    // @Field: NG: valid INSGPS frames
    // @Field: NP: valid geodetic position frames
    // @Field: NS: valid frames of skipped types
    // @Field: CE: known-type frames failing CRC or frame-end check
    // @Field: RS: parser resync events
    // @Field: SL: frames lost per serial-number gaps
    // @Field: NAV: INS nav feed active
    AP::logger().WriteStreaming("FDIS", "TimeUS,NI,NA,NG,NP,NS,CE,RS,SL,NAV",
                                "s---------", "F000000000",
                                "QIIIIIIIIB",
                                AP_HAL::micros64(),
                                imu_frame_count, ahrs_frame_count,
                                insgps_frame_count, geopos_frame_count,
                                skip_frame_count, crc_fail_count,
                                resync_count, sn_lost_count,
                                uint8_t(nav_feed_active));
#endif  // HAL_LOGGING_ENABLED
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_FDILink::get_port(void) const
{
    if (uart == nullptr) {
        return -1;
    }
    return port_num;
}

// accessors for AP_AHRS
bool AP_ExternalAHRS_FDILink::healthy(void) const
{
    const uint32_t now_ms = AP_HAL::millis();
    // 4 missed frames at the fixed 100Hz output rate
    if (now_ms - last_imu_pkt_ms >= 40) {
        return false;
    }
    if (state_feed_enabled() && last_ahrs_pkt_ms != 0 &&
        now_ms - last_ahrs_pkt_ms >= 500) {
        // attitude consumers rely on the 0x41 stream once it has been seen
        return false;
    }
    return true;
}

bool AP_ExternalAHRS_FDILink::initialised(void) const
{
    return last_imu_pkt_ms != 0;
}

bool AP_ExternalAHRS_FDILink::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (uart == nullptr) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FDILink no UART");
        return false;
    }
    if (!initialised()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FDILink no data");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FDILink unhealthy");
        return false;
    }
    if (crc_fail_count != 0 && AP_HAL::millis() - last_crc_error_ms < 10000) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FDILink CRC errors: %u", unsigned(crc_fail_count));
        return false;
    }
    if (state_feed_enabled() && last_ahrs_pkt_ms == 0) {
        hal.util->snprintf(failure_msg, failure_msg_len, "FDILink no attitude data");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_FDILink::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    status.flags.initalized = initialised();
    if (!state_feed_enabled()) {
        // IMU + compass source only, no estimator claims
        return;
    }
    // attitude available while the 0x41 stream is fresh
    status.flags.attitude = healthy() && last_ahrs_pkt_ms != 0;
    // position/velocity available while the 0x42/0x5C INS solution is
    // fresh. these flags gate ArduPlane's EXTERNAL->DCM fallback, so they
    // must reflect the real nav feed state
    if (nav_feed_active) {
        status.flags.horiz_vel = true;
        status.flags.vert_vel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.horiz_pos_rel = true;
        status.flags.vert_pos = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.using_gps = true;
    }
}

#endif  // AP_EXTERNAL_AHRS_FDILINK_ENABLED
