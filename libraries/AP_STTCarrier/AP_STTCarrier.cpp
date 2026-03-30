#include "AP_STTCarrier.h"

#if AP_STT_CARRIER_ENABLED

#include <AP_Math/AP_Math.h>

AP_STTCarrier *AP_STTCarrier::_singleton = nullptr;

AP_STTCarrier::AP_STTCarrier()
{
    if (_singleton == nullptr) {
        _singleton = this;
    }
}

bool AP_STTCarrier::init(uint8_t serial_instance)
{
    const AP_SerialManager &sm = AP::serialmanager();
    _port = sm.find_serial(AP_SerialManager::SerialProtocol_Scripting, serial_instance);
    if (_port == nullptr) {
        return false;
    }
    _port->begin(115200);
    _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    return true;
}

void AP_STTCarrier::update()
{
    if (_port == nullptr) {
        return;
    }
    const uint32_t avail = _port->available();
    const uint32_t max_read = MIN(avail, 128U);
    for (uint32_t i = 0; i < max_read; i++) {
        const int16_t b = _port->read();
        if (b >= 0) {
            _parse_byte(static_cast<uint8_t>(b));
        }
    }
}

// --- byte helpers ---

int32_t AP_STTCarrier::_s32_le(const uint8_t *buf, uint8_t idx)
{
    uint32_t v = buf[idx]
               | (uint32_t(buf[idx + 1]) << 8)
               | (uint32_t(buf[idx + 2]) << 16)
               | (uint32_t(buf[idx + 3]) << 24);
    return static_cast<int32_t>(v);
}

// --- frame sending ---

void AP_STTCarrier::_send_frame(uint8_t fid, const uint8_t *payload, uint8_t len)
{
    if (_port == nullptr) {
        return;
    }
    _port->write(HDR_0);
    _port->write(HDR_1);
    _port->write(fid);
    _port->write(len);

    uint8_t sum = fid + len;
    for (uint8_t i = 0; i < len; i++) {
        _port->write(payload[i]);
        sum += payload[i];
    }
    _port->write(sum & 0xFF);
}

// --- frame handling ---

void AP_STTCarrier::_handle_frame(uint8_t fid, const uint8_t *buf, uint8_t len)
{
    _state.last_rx_ms = AP_HAL::millis();
    _state.rx_count++;
    _state.connected = true;

    const uint8_t ack_byte = 0x01;

    switch (fid) {
    case FID_MOUNT_QUERY:
        _send_frame(FID_MOUNT_ACK, &ack_byte, 1);
        break;

    case FID_TARGET_COORD:
        if (len >= 12) {
            _state.target_lat    = _s32_le(buf, 0);
            _state.target_lon    = _s32_le(buf, 4);
            _state.target_alt_cm = _s32_le(buf, 8);
            _send_frame(FID_TARGET_ACK, &ack_byte, 1);
        }
        break;

    case FID_SEEKER_CODE:
        if (len >= 2) {
            _state.laser_code_type  = buf[0];
            _state.laser_code_param = buf[1];
            _send_frame(FID_CODE_ACK, &ack_byte, 1);
        }
        break;

    case FID_LAUNCH_CMD:
        _state.launch_cmd = true;
        _send_frame(FID_LAUNCH_ACK, &ack_byte, 1);
        break;
    }
}

// --- parse FSM ---

void AP_STTCarrier::_parse_byte(uint8_t b)
{
    switch (_parse_state) {
    case PS_HDR0:
        if (b == HDR_0) {
            _parse_state = PS_HDR1;
        }
        break;

    case PS_HDR1:
        if (b == HDR_1) {
            _parse_state = PS_FID;
        } else if (b != HDR_0) {
            _parse_state = PS_HDR0;
        }
        break;

    case PS_FID:
        _parse_fid = b;
        _parse_state = PS_LEN;
        break;

    case PS_LEN:
        _parse_len = b;
        _parse_idx = 0;
        if (_parse_len == 0) {
            _parse_state = PS_CSUM;
        } else if (_parse_len > sizeof(_parse_buf)) {
            _state.err_count++;
            _parse_state = PS_HDR0;
        } else {
            _parse_state = PS_DATA;
        }
        break;

    case PS_DATA:
        _parse_buf[_parse_idx++] = b;
        if (_parse_idx >= _parse_len) {
            _parse_state = PS_CSUM;
        }
        break;

    case PS_CSUM: {
        uint8_t sum = _parse_fid + _parse_len;
        for (uint8_t i = 0; i < _parse_len; i++) {
            sum += _parse_buf[i];
        }
        if ((sum & 0xFF) == b) {
            _handle_frame(_parse_fid, _parse_buf, _parse_len);
        } else {
            _state.err_count++;
        }
        _parse_state = PS_HDR0;
        break;
    }
    }
}

#endif  // AP_STT_CARRIER_ENABLED
