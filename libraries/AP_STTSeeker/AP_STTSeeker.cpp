#include "AP_STTSeeker.h"

#if AP_STT_SEEKER_ENABLED

#include <AP_Math/AP_Math.h>

AP_STTSeeker *AP_STTSeeker::_singleton = nullptr;

AP_STTSeeker::AP_STTSeeker()
{
    if (_singleton == nullptr) {
        _singleton = this;
    }
}

bool AP_STTSeeker::init(uint8_t serial_instance)
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

void AP_STTSeeker::update()
{
    if (_port == nullptr) {
        return;
    }
    const uint32_t avail = _port->available();
    const uint32_t max_read = MIN(avail, 256U);
    for (uint32_t i = 0; i < max_read; i++) {
        const int16_t b = _port->read();
        if (b >= 0) {
            _parse_byte(static_cast<uint8_t>(b));
        }
    }
}

// --- byte helpers ---

uint16_t AP_STTSeeker::_u16_le(const uint8_t *buf, uint8_t idx)
{
    return buf[idx] | (uint16_t(buf[idx + 1]) << 8);
}

int16_t AP_STTSeeker::_s16_le(const uint8_t *buf, uint8_t idx)
{
    uint16_t v = _u16_le(buf, idx);
    return (v >= 32768) ? int16_t(v - 65536) : int16_t(v);
}

uint32_t AP_STTSeeker::_u24_le(const uint8_t *buf, uint8_t idx)
{
    return buf[idx] | (uint32_t(buf[idx + 1]) << 8) | (uint32_t(buf[idx + 2]) << 16);
}

// --- frame sending ---

void AP_STTSeeker::_send_frame(uint8_t fid, const uint8_t *payload, uint8_t len)
{
    if (_port == nullptr) {
        return;
    }
    _port->write(HEADER_0);
    _port->write(HEADER_1);
    _port->write(fid);

    uint8_t cksum = fid;
    for (uint8_t i = 0; i < len; i++) {
        _port->write(payload[i]);
        cksum += payload[i];
    }
    _port->write(cksum & 0xFF);
}

void AP_STTSeeker::_send_control(uint8_t ctrl, uint8_t p1, uint32_t p2)
{
    uint8_t payload[6];
    payload[0] = ctrl;
    payload[1] = p1;
    payload[2] = p2 & 0xFF;
    payload[3] = (p2 >> 8) & 0xFF;
    payload[4] = (p2 >> 16) & 0xFF;
    payload[5] = 0;  // reserved
    _send_frame(FID_CONTROL, payload, 6);
}

// --- parse FSM ---

uint8_t AP_STTSeeker::_get_expected_len(uint8_t fid) const
{
    if (fid == FID_PERIODIC) {
        return PERIODIC_FRAME_LEN;
    }
    return 0;  // variable length
}

void AP_STTSeeker::_parse_periodic()
{
    if (_buf_len != PERIODIC_FRAME_LEN) {
        return;
    }

    // checksum: sum of bytes [3..len-2], compare with byte [len-1]
    // buf is 0-indexed, frame bytes: [0]=hdr0 [1]=hdr1 [2]=fid [3..51]=data [52]=cksum
    uint8_t ck = 0;
    for (uint8_t i = 2; i < _buf_len - 1; i++) {
        ck += _buf[i];
    }
    if (ck != _buf[_buf_len - 1]) {
        _data.err_count++;
        return;
    }

    _data.rx_count++;
    _data.last_rx_ms = AP_HAL::millis();

    const uint8_t *b = _buf;
    _data.status      = b[3];
    _data.pitch_deg   = _s16_le(b, 4) * 0.001f;
    _data.yaw_deg     = _s16_le(b, 6) * 0.001f;
    _data.code_type   = b[8];
    _data.laser_period = _u24_le(b, 9);
    _data.status_info = b[12];
    _data.gain_level  = b[13];
    _data.agc_rt      = b[14];
    _data.agc_ref     = b[15];
    _data.vsum        = _u16_le(b, 16);
    _data.vsum_volt   = _data.vsum * (4.096f / 16384.0f);
    _data.vin[0]      = _u16_le(b, 18);
    _data.vin[1]      = _u16_le(b, 20);
    _data.vin[2]      = _u16_le(b, 22);
    _data.vin[3]      = _u16_le(b, 24);
    _data.period_prec = b[33];
    _data.front_gate  = b[34];
    _data.rear_gate   = b[35];
    _data.ver_major   = _u16_le(b, 47);
    _data.ver_minor   = _u16_le(b, 49);
    _data.frame_cnt   = b[51];

    _data.blind_zone  = ((b[41] >> 1) & 1) == 1;
    _data.captured    = (_data.status == STATUS_CAPTURED);
    _data.angle_valid = _data.captured && (_data.vsum_volt <= 1.3f);
}

void AP_STTSeeker::_parse_byte(uint8_t b)
{
    switch (_parse_state) {
    case WAIT_HDR0:
        if (b == HEADER_0) {
            _buf[0] = b;
            _buf_len = 1;
            _parse_state = WAIT_HDR1;
        }
        break;

    case WAIT_HDR1:
        if (b == HEADER_1) {
            _buf[1] = b;
            _buf_len = 2;
            _parse_state = RECV_DATA;
        } else if (b != HEADER_0) {
            _parse_state = WAIT_HDR0;
        }
        // if b == HEADER_0, stay in WAIT_HDR1
        break;

    case RECV_DATA:
        if (_buf_len < sizeof(_buf)) {
            _buf[_buf_len++] = b;
        }

        if (_buf_len == 3) {
            _expected_len = _get_expected_len(b);
        }

        // variable-length frame length inference
        if (_expected_len == 0 && _buf_len >= 5) {
            const uint8_t fid = _buf[2];
            if (fid == FID_CODE_REPLY) {
                uint16_t n = _buf[3] | (uint16_t(_buf[4]) << 8);
                _expected_len = 4 * n + 9;
            } else if (fid == FID_ALL_REPLY) {
                uint16_t m = _buf[3] | (uint16_t(_buf[4]) << 8);
                _expected_len = 4 * m + 6;
            }
        }

        if (_expected_len > 0 && _buf_len >= _expected_len) {
            if (_buf[2] == FID_PERIODIC) {
                _parse_periodic();
            }
            _parse_state = WAIT_HDR0;
        }

        // overflow guard
        if (_buf_len >= sizeof(_buf)) {
            _parse_state = WAIT_HDR0;
            _data.err_count++;
        }
        break;
    }
}

// --- control commands ---

void AP_STTSeeker::allow_capture()
{
    _send_control(CTRL_ALLOW_CAPTURE);
}

void AP_STTSeeker::forbid_capture()
{
    _send_control(CTRL_FORBID_CAPTURE);
}

void AP_STTSeeker::reset()
{
    _send_control(CTRL_RESET);
}

void AP_STTSeeker::code_bind(uint8_t code_type, uint32_t param)
{
    _send_control(CTRL_CODE_BIND, code_type, param);
}

void AP_STTSeeker::set_period_precision(uint8_t us)
{
    _send_control(CTRL_PERIOD_PREC, us * 2);
}

void AP_STTSeeker::agc_dynamic()
{
    _send_control(CTRL_AGC_DYNAMIC);
}

void AP_STTSeeker::agc_fixed(uint8_t val)
{
    _send_control(CTRL_AGC_FIXED, val);
}

void AP_STTSeeker::gate_width(uint8_t front_us, uint8_t rear_us)
{
    _send_control(CTRL_GATE_WIDTH, front_us * 2, rear_us * 2);
}

#endif  // AP_STT_SEEKER_ENABLED
