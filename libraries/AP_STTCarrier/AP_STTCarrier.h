#pragma once

#include "AP_STTCarrier_config.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_STT_CARRIER_ENABLED

class AP_STTCarrier {
public:
    AP_STTCarrier();

    static AP_STTCarrier *get_singleton() { return _singleton; }

    // Lua callable
    bool init(uint8_t serial_instance);
    void update();

    // state queries
    bool is_connected() const { return _state.connected; }
    bool is_launch_commanded() const { return _state.launch_cmd; }
    bool has_target() const { return _state.target_lat != 0 || _state.target_lon != 0; }

    // target coordinates (degrees, degrees, meters MSL)
    float get_target_lat_deg() const { return _state.target_lat * 1e-7f; }
    float get_target_lon_deg() const { return _state.target_lon * 1e-7f; }
    float get_target_alt_m() const { return _state.target_alt_cm * 0.01f; }

    // double-precision access (avoid float truncation for C++ guidance)
    int32_t get_target_lat_int32() const { return _state.target_lat; }
    int32_t get_target_lon_int32() const { return _state.target_lon; }
    int32_t get_target_alt_cm() const { return _state.target_alt_cm; }

    uint8_t get_laser_code_type() const { return _state.laser_code_type; }
    uint8_t get_laser_code_param() const { return _state.laser_code_param; }

    uint16_t get_rx_count() const { return _state.rx_count; }
    uint16_t get_err_count() const { return _state.err_count; }
    uint32_t get_last_rx_ms() const { return _state.last_rx_ms; }

private:
    AP_HAL::UARTDriver *_port = nullptr;

    // frame constants
    static constexpr uint8_t HDR_0 = 0xEB;
    static constexpr uint8_t HDR_1 = 0x90;

    // carrier -> munition
    static constexpr uint8_t FID_MOUNT_QUERY  = 0x01;
    static constexpr uint8_t FID_TARGET_COORD = 0x02;
    static constexpr uint8_t FID_SEEKER_CODE  = 0x03;
    static constexpr uint8_t FID_LAUNCH_CMD   = 0x04;

    // munition -> carrier
    static constexpr uint8_t FID_MOUNT_ACK  = 0x81;
    static constexpr uint8_t FID_TARGET_ACK = 0x82;
    static constexpr uint8_t FID_CODE_ACK   = 0x83;
    static constexpr uint8_t FID_LAUNCH_ACK = 0x84;

    // parse FSM
    enum ParseState : uint8_t {
        PS_HDR0 = 0,
        PS_HDR1 = 1,
        PS_FID  = 2,
        PS_LEN  = 3,
        PS_DATA = 4,
        PS_CSUM = 5,
    };
    ParseState _parse_state = PS_HDR0;
    uint8_t _parse_fid = 0;
    uint8_t _parse_len = 0;
    uint8_t _parse_buf[32];
    uint8_t _parse_idx = 0;

    // state
    struct State {
        bool connected = false;
        int32_t target_lat = 0;      // 1e-7 deg
        int32_t target_lon = 0;      // 1e-7 deg
        int32_t target_alt_cm = 0;   // cm
        uint8_t laser_code_type = 0;
        uint8_t laser_code_param = 0;
        bool launch_cmd = false;
        uint32_t last_rx_ms = 0;
        uint16_t rx_count = 0;
        uint16_t err_count = 0;
    } _state;

    void _parse_byte(uint8_t b);
    void _handle_frame(uint8_t fid, const uint8_t *buf, uint8_t len);
    void _send_frame(uint8_t fid, const uint8_t *payload, uint8_t len);

    static int32_t _s32_le(const uint8_t *buf, uint8_t idx);

    static AP_STTCarrier *_singleton;
};

#endif  // AP_STT_CARRIER_ENABLED
