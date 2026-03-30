#pragma once

#include "AP_STTSeeker_config.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#if AP_STT_SEEKER_ENABLED

class AP_STTSeeker {
public:
    AP_STTSeeker();

    static AP_STTSeeker *get_singleton() { return _singleton; }

    // Lua callable
    bool init(uint8_t serial_instance);
    void update();

    // state queries
    bool is_captured() const { return _data.captured; }
    bool is_angle_valid() const { return _data.angle_valid; }
    float get_pitch_deg() const { return _data.pitch_deg; }
    float get_yaw_deg() const { return _data.yaw_deg; }
    uint8_t get_status() const { return _data.status; }
    uint32_t get_last_rx_ms() const { return _data.last_rx_ms; }
    uint16_t get_rx_count() const { return _data.rx_count; }
    uint16_t get_err_count() const { return _data.err_count; }

    // control commands
    void allow_capture();
    void forbid_capture();
    void reset();
    void code_bind(uint8_t code_type, uint32_t param);
    void set_period_precision(uint8_t us);
    void agc_dynamic();
    void agc_fixed(uint8_t val);
    void gate_width(uint8_t front_us, uint8_t rear_us);

private:
    AP_HAL::UARTDriver *_port = nullptr;

    // frame constants
    static constexpr uint8_t HEADER_0 = 0x55;
    static constexpr uint8_t HEADER_1 = 0xAA;

    static constexpr uint8_t FID_CONTROL     = 0x02;
    static constexpr uint8_t FID_PERIODIC    = 0x03;
    static constexpr uint8_t FID_CODE_INJECT = 0x07;
    static constexpr uint8_t FID_CODE_QUERY  = 0x08;
    static constexpr uint8_t FID_CODE_REPLY  = 0x17;
    static constexpr uint8_t FID_ALL_QUERY   = 0x09;
    static constexpr uint8_t FID_ALL_REPLY   = 0x27;

    static constexpr uint8_t CTRL_PERIOD_PREC    = 0x11;
    static constexpr uint8_t CTRL_AGC_DYNAMIC    = 0x33;
    static constexpr uint8_t CTRL_AGC_FIXED      = 0x44;
    static constexpr uint8_t CTRL_ALLOW_CAPTURE  = 0x55;
    static constexpr uint8_t CTRL_RESET          = 0x77;
    static constexpr uint8_t CTRL_CODE_BIND      = 0x88;
    static constexpr uint8_t CTRL_GATE_WIDTH     = 0x99;
    static constexpr uint8_t CTRL_FORBID_CAPTURE = 0xFF;

    static constexpr uint8_t STATUS_CAPTURED  = 0x03;

    static constexpr uint8_t PERIODIC_FRAME_LEN = 53;

    // parse FSM
    enum ParseState : uint8_t {
        WAIT_HDR0 = 0,
        WAIT_HDR1 = 1,
        RECV_DATA = 2,
    };
    ParseState _parse_state = WAIT_HDR0;
    uint8_t _buf[64];
    uint8_t _buf_len = 0;
    uint8_t _expected_len = 0;

    // parsed data
    struct Data {
        uint8_t status = 0;
        float pitch_deg = 0;
        float yaw_deg = 0;
        uint8_t code_type = 0;
        uint32_t laser_period = 0;
        uint8_t status_info = 0;
        uint8_t gain_level = 0;
        uint8_t agc_rt = 0;
        uint8_t agc_ref = 0;
        uint16_t vsum = 0;
        float vsum_volt = 0;
        uint16_t vin[4] {};
        uint8_t period_prec = 0;
        uint8_t front_gate = 0;
        uint8_t rear_gate = 0;
        uint16_t ver_major = 0;
        uint16_t ver_minor = 0;
        uint8_t frame_cnt = 0;
        bool blind_zone = false;
        bool captured = false;
        bool angle_valid = false;
        uint16_t rx_count = 0;
        uint16_t err_count = 0;
        uint32_t last_rx_ms = 0;
    } _data;

    void _parse_byte(uint8_t b);
    void _parse_periodic();
    uint8_t _get_expected_len(uint8_t fid) const;
    void _send_frame(uint8_t fid, const uint8_t *payload, uint8_t len);
    void _send_control(uint8_t ctrl, uint8_t p1 = 0, uint32_t p2 = 0);

    // helpers
    static uint16_t _u16_le(const uint8_t *buf, uint8_t idx);
    static int16_t _s16_le(const uint8_t *buf, uint8_t idx);
    static uint32_t _u24_le(const uint8_t *buf, uint8_t idx);

    static AP_STTSeeker *_singleton;
};

#endif  // AP_STT_SEEKER_ENABLED
