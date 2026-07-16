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

  frame layout (from vendor ROS driver fdilink_ahrs):
    0xFC | type | len | sn | crc8(bytes 0-3) | crc16_H | crc16_L | payload[len] | 0xFD
  crc8  = Maxim/Dallas table CRC over the first 4 header bytes
  crc16 = CCITT/XMODEM (poly 0x1021, init 0, MSB first) over the payload

  type 0x40 (len 56): IMU  - gyro rad/s, accel m/s^2, mag milliGauss,
                      imu temperature degC, pressure Pa (constant placeholder
                      101325 on DETA40 - never fed to baro), timestamp us
  type 0x41 (len 48): AHRS - euler rad, quaternion wxyz (FRD body -> NED
                      world). in INS mode (external GNSS wired into the
                      device) this is the GNSS-aided INS attitude
  type 0x42 (len 72): INSGPS - body velocity, NED velocity (m/s), NED accel,
                      local-NED position (unknown origin, logged only),
                      pressure altitude, timestamp. no lat/lon, no attitude
  type 0x5C (len 32): GEODETIC_POS - lat/lon (double, radians), height
                      (double, m), hAcc/vAcc (float, m)
  other types (e.g. 0xF0 1Hz ground frame) are skipped by length.

  device body frame is FRD, matching ArduPilot conventions: verified against
  captured data by rotating measured accel with the device quaternion, giving
  (0, 0, -9.81) in NED. no axis remapping required.

  state feed (unless EAHRS_OPTIONS bit 2 set), layered by freshness:
  - 0x41 quaternion -> state.quat (attitude source for AHRS_EKF_TYPE=11)
  - 0x42 NED velocity -> state.velocity and 0x5C -> state.location/origin
    while fresh (EAHRS_NAV_TMO window). when the INS nav solution times out
    the backend drops its position/velocity claims (attitude-only): on
    ArduPlane the fixed-wing fallback in AP_AHRS::_active_EKF_type then
    demotes the active estimator to DCM, which navigates on the FC's own GPS.
  yaw quality depends on the device's aiding (magnetic when standalone,
  GNSS-aided in INS mode) - evaluate before use as the primary source.
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_FDILINK_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_FDILink : public AP_ExternalAHRS_backend {
public:

    AP_ExternalAHRS_FDILink(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // Get model/type name
    const char* get_name() const override {
        return "FDILink";
    }

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;

    // check for new data
    void update() override {
        check_uart();
    }

protected:

    uint8_t num_gps_sensors(void) const override {
        // no GNSS on the FDILink IMU/AHRS units we support
        return 0;
    }

private:
    void update_thread();
    bool check_uart();
    // resync: drop buffer head up to the next candidate frame head byte
    void resync_buffer(void);

    void process_imu_packet(const uint8_t *payload);
    void process_ahrs_packet(const uint8_t *payload);
    void process_insgps_packet(const uint8_t *payload);
    void process_geopos_packet(const uint8_t *payload);

    // drop stale position/velocity claims (fall back to attitude-only)
    // when the 0x42/0x5C INS solution exceeds the EAHRS_NAV_TMO window
    void update_nav_timeout(void);

    // attitude + nav feed gate (EAHRS_OPTIONS bit 2 clear = enabled)
    bool state_feed_enabled(void) const {
        return !option_is_set(AP_ExternalAHRS::OPTIONS::FDILINK_FEED_DISABLE);
    }

    void log_status(void);

    AP_HAL::UARTDriver *uart;
    HAL_Semaphore sem;
    int8_t port_num;
    uint32_t baudrate;
    bool port_open;

    // enough for at least two max-size (64 byte) frames plus slack
    uint8_t pktbuf[256];
    uint16_t pktoffset;

    uint32_t last_imu_pkt_ms;
    uint32_t last_ahrs_pkt_ms;
    uint32_t last_insgps_pkt_ms;  // 0x42 freshness (velocity feed)
    uint32_t last_geopos_pkt_ms;  // 0x5C freshness (position feed)
    uint32_t last_crc_error_ms;
    uint32_t last_status_log_ms;
    bool nav_feed_active;         // pos/vel currently claimed in state

    // latest 0x42 extras for FDNV logging
    float latest_palt_m;
    float latest_vel_ned[3];

    // stream statistics, logged at 1Hz in FDIS
    uint32_t imu_frame_count;
    uint32_t ahrs_frame_count;
    uint32_t insgps_frame_count;
    uint32_t geopos_frame_count;
    uint32_t skip_frame_count;   // valid frames of types we don't consume
    uint32_t crc_fail_count;     // known-type frames failing crc16/frame-end
    uint32_t resync_count;       // resync events (garbage / header crc8 fail)
    uint32_t sn_lost_count;      // lost frames detected via serial number gaps

    uint8_t last_sn;
    bool sn_valid;
};

#endif  // AP_EXTERNAL_AHRS_FDILINK_ENABLED
