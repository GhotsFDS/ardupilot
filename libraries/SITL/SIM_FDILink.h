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

  Usage example:
     SERIAL5_PROTOCOL = 36
     SERIAL5_BAUD = 921600
     EAHRS_TYPE = 11
     AHRS_EKF_TYPE = 11 (optional, to use it as the AHRS source)

     sim_vehicle.py -A --serial5=sim:fdilink

  emits 0x40 IMU and 0x41 AHRS frames at 100Hz each, 0x42 INSGPS at 100Hz
  and 0x5C geodetic position at 10Hz (INS mode), plus a 1Hz 0xF0 ground
  frame with an independent serial number counter, matching the observed
  behaviour of the real DETA40 (including the constant 101325 Pa pressure
  placeholder).
*/

#pragma once

#include "SIM_Aircraft.h"

#include <SITL/SITL.h>
#include "SIM_SerialDevice.h"

namespace SITL {

class FDILink : public SerialDevice {
public:
    FDILink() {};

    // update state
    void update(void);

private:
    uint32_t last_pkt_us;
    uint32_t last_geopos_pkt_us;
    uint32_t last_ground_pkt_us;
    uint8_t sn;         // shared 0x40/0x41/0x42/0x5C serial number
    uint8_t ground_sn;  // independent 0xF0 serial number

    void send_frame(uint8_t frame_type, const uint8_t *payload, uint8_t len);
    void send_imu_packet(void);
    void send_ahrs_packet(void);
    void send_insgps_packet(void);
    void send_geopos_packet(void);
    void send_ground_packet(void);
};

}
