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
  simulate a serial FDILink AHRS/IMU (FDISystems DETA40 style)
*/

#include "SIM_FDILink.h"
#include <AP_Math/crc.h>

using namespace SITL;

// payload of type 0x40, little-endian, matches AP_ExternalAHRS_FDILink
struct PACKED FDILink_imu_payload_sim {
    float gyro[3];        // rad/s, body FRD
    float accel[3];       // m/s^2, body FRD
    float mag[3];         // milliGauss, body FRD
    float imu_temp;       // degC
    float pressure;       // Pa, constant placeholder on DETA40
    float pressure_temp;  // degC
    int64_t timestamp_us;
};
static_assert(sizeof(FDILink_imu_payload_sim) == 56, "FDILink IMU payload must be 56 bytes");

// payload of type 0x41, little-endian
struct PACKED FDILink_ahrs_payload_sim {
    float roll_speed;     // rad/s
    float pitch_speed;    // rad/s
    float heading_speed;  // rad/s
    float roll;           // rad
    float pitch;          // rad
    float heading;        // rad
    float q[4];           // wxyz, body FRD -> NED
    int64_t timestamp_us;
};
static_assert(sizeof(FDILink_ahrs_payload_sim) == 48, "FDILink AHRS payload must be 48 bytes");

// payload of type 0x42 (INS mode), little-endian
struct PACKED FDILink_insgps_payload_sim {
    float body_vel[3];    // m/s
    float body_accel[3];  // m/s^2
    float location_ned[3];// m, local NED (device-internal origin)
    float vel_ned[3];     // m/s
    float accel_ned[3];   // m/s^2
    float pressure_alt;   // m
    int64_t timestamp_us;
};
static_assert(sizeof(FDILink_insgps_payload_sim) == 72, "FDILink INSGPS payload must be 72 bytes");

// payload of type 0x5C (INS mode), little-endian
struct PACKED FDILink_geopos_payload_sim {
    double latitude_rad;
    double longitude_rad;
    double height_m;
    float hacc_m;
    float vacc_m;
};
static_assert(sizeof(FDILink_geopos_payload_sim) == 32, "FDILink GEOPOS payload must be 32 bytes");

/*
  frame: 0xFC | type | len | sn | crc8(bytes 0-3) | crc16_H | crc16_L | payload | 0xFD
 */
void FDILink::send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t len)
{
    uint8_t header[7];
    header[0] = 0xFC;
    header[1] = frame_type;
    header[2] = len;
    header[3] = (frame_type == 0xF0) ? ground_sn++ : sn++;
    header[4] = crc8_maxim(header, 4);
    const uint16_t crc16 = crc_xmodem(payload, len);
    header[5] = uint8_t(crc16 >> 8);
    header[6] = uint8_t(crc16 & 0xFF);

    write_to_autopilot((const char *)header, sizeof(header));
    write_to_autopilot((const char *)payload, len);
    const uint8_t frame_end = 0xFD;
    write_to_autopilot((const char *)&frame_end, 1);
}

void FDILink::send_imu_packet(void)
{
    const auto &fdm = _sitl->state;

    struct FDILink_imu_payload_sim pkt {};

    pkt.gyro[0] = radians(fdm.rollRate);
    pkt.gyro[1] = radians(fdm.pitchRate);
    pkt.gyro[2] = radians(fdm.yawRate);

    pkt.accel[0] = fdm.xAccel;
    pkt.accel[1] = fdm.yAccel;
    pkt.accel[2] = fdm.zAccel;

    // bodyMagField is already in milliGauss, the FDILink native unit
    pkt.mag[0] = fdm.bodyMagField.x;
    pkt.mag[1] = fdm.bodyMagField.y;
    pkt.mag[2] = fdm.bodyMagField.z;

    pkt.imu_temp = 40.0f;
    // real DETA40 sends a constant placeholder here; replicate it so the
    // driver's no-baro behaviour is exercised
    pkt.pressure = 101325.0f;
    pkt.pressure_temp = 25.0f;
    pkt.timestamp_us = int64_t(AP_HAL::micros64());

    send_frame(0x40, (const uint8_t *)&pkt, sizeof(pkt));
}

void FDILink::send_ahrs_packet(void)
{
    const auto &fdm = _sitl->state;

    struct FDILink_ahrs_payload_sim pkt {};

    pkt.roll_speed = radians(fdm.rollRate);
    pkt.pitch_speed = radians(fdm.pitchRate);
    pkt.heading_speed = radians(fdm.yawRate);

    pkt.roll = radians(fdm.rollDeg);
    pkt.pitch = radians(fdm.pitchDeg);
    pkt.heading = radians(fdm.yawDeg);

    // fdm quaternion is w,x,y,z in q1..q4; FDILink is wxyz: direct map
    pkt.q[0] = fdm.quaternion.q1;
    pkt.q[1] = fdm.quaternion.q2;
    pkt.q[2] = fdm.quaternion.q3;
    pkt.q[3] = fdm.quaternion.q4;

    pkt.timestamp_us = int64_t(AP_HAL::micros64());

    send_frame(0x41, (const uint8_t *)&pkt, sizeof(pkt));
}

void FDILink::send_insgps_packet(void)
{
    const auto &fdm = _sitl->state;

    struct FDILink_insgps_payload_sim pkt {};

    // body velocity: rotate NED velocity into body frame
    Quaternion q{fdm.quaternion.q1, fdm.quaternion.q2, fdm.quaternion.q3, fdm.quaternion.q4};
    Vector3f vel_ned{float(fdm.speedN), float(fdm.speedE), float(fdm.speedD)};
    Vector3f vel_body = q.inverse() * vel_ned;
    pkt.body_vel[0] = vel_body.x;
    pkt.body_vel[1] = vel_body.y;
    pkt.body_vel[2] = vel_body.z;

    pkt.body_accel[0] = fdm.xAccel;
    pkt.body_accel[1] = fdm.yAccel;
    pkt.body_accel[2] = fdm.zAccel;

    // local NED position left zero: device-internal origin, not consumed

    pkt.vel_ned[0] = fdm.speedN;
    pkt.vel_ned[1] = fdm.speedE;
    pkt.vel_ned[2] = fdm.speedD;

    pkt.pressure_alt = fdm.altitude;
    pkt.timestamp_us = int64_t(AP_HAL::micros64());

    send_frame(0x42, (const uint8_t *)&pkt, sizeof(pkt));
}

void FDILink::send_geopos_packet(void)
{
    const auto &fdm = _sitl->state;

    struct FDILink_geopos_payload_sim pkt {};
    // double conversion: float radians() would quantise at ~0.4m
    pkt.latitude_rad = fdm.latitude * DEG_TO_RAD_DOUBLE;
    pkt.longitude_rad = fdm.longitude * DEG_TO_RAD_DOUBLE;
    pkt.height_m = fdm.altitude;
    pkt.hacc_m = 0.02f;  // RTK-ish
    pkt.vacc_m = 0.03f;

    send_frame(0x5C, (const uint8_t *)&pkt, sizeof(pkt));
}

void FDILink::send_ground_packet(void)
{
    // 0xF0 ground frame with 17 byte payload, as seen at 1Hz on the real
    // unit. content is opaque; the driver must skip it by length
    uint8_t payload[17] {};
    payload[0] = 0x01;
    send_frame(0xF0, payload, sizeof(payload));
}

/*
  send FDILink data
 */
void FDILink::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    const uint32_t now = AP_HAL::micros();
    if (now - last_pkt_us >= 10000) {
        // 100Hz, AHRS/IMU/INSGPS frames back to back
        last_pkt_us = now;
        send_ahrs_packet();
        send_imu_packet();
        send_insgps_packet();
    }
    if (now - last_geopos_pkt_us >= 100000) {
        // 10Hz global position
        last_geopos_pkt_us = now;
        send_geopos_packet();
    }
    if (now - last_ground_pkt_us >= 1000000) {
        last_ground_pkt_us = now;
        send_ground_packet();
    }
}
