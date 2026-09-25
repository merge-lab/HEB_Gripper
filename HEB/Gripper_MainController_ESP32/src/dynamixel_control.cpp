#include "dynamixel_control.h"

void DynamixelController::begin(long baud, int rx_pin, int tx_pin)
{
    _serial->begin(baud, SERIAL_8N1, rx_pin, tx_pin);
}

bool DynamixelController::send_command(const Dyn_Command_Packet& packet,
                                       Dyn_Response_Packet* response,
                                       unsigned long timeout_ms)
{
    // Drop any stale bytes left over from a prior timed-out exchange so a
    // late reply can never bleed into (and misframe) this call's read.
    while (_serial->available())
        _serial->read();

    _serial->write((const uint8_t*)&packet, sizeof(packet));

    if (response == nullptr)
        return true;

    bool valid = false;
    *response = read_response(&valid, timeout_ms);
    return valid;
}

Dyn_Response_Packet DynamixelController::read_response(bool* valid,
                                                        unsigned long timeout_ms)
{
    Dyn_Response_Packet result = {};
    if (valid) *valid = false;

    unsigned long start = millis();

    // 1. Resync: discard bytes until START_BYTE is found.
    bool found_start = false;
    while (millis() - start < timeout_ms)
    {
        if (_serial->available())
        {
            char b = _serial->read();
            if (b == DYN_START_BYTE)
            {
                result.startByte = DYN_START_BYTE;
                found_start = true;
                break;
            }
        }
    }

    if (!found_start)
        return result;

    // 2. Read the remaining bytes of the struct.
    size_t remaining = sizeof(Dyn_Response_Packet) - 1;
    uint8_t* ptr = (uint8_t*)&result;
    size_t index = 1;

    while (remaining > 0 && (millis() - start < timeout_ms))
    {
        if (_serial->available())
        {
            ptr[index++] = _serial->read();
            remaining--;
        }
    }

    if (remaining != 0)
        return result;

    // 3. Validate END_BYTE.
    if (result.endByte != DYN_END_BYTE)
        return result;

    if (valid) *valid = true;
    return result;
}

bool DynamixelController::move_servo(float left_deg, float right_deg,
                                     float left_speed, float right_speed,
                                     Dyn_Response_Packet* response,
                                     unsigned long timeout_ms)
{
    uint32_t left_deg_bits, right_deg_bits, left_spd_bits, right_spd_bits;
    memcpy(&left_deg_bits,  &left_deg,    sizeof(left_deg_bits));
    memcpy(&right_deg_bits, &right_deg,   sizeof(right_deg_bits));
    memcpy(&left_spd_bits,  &left_speed,  sizeof(left_spd_bits));
    memcpy(&right_spd_bits, &right_speed, sizeof(right_spd_bits));

    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_MOVE_SERVO,
        left_deg_bits,
        right_deg_bits,
        left_spd_bits,
        right_spd_bits,
        DYN_END_BYTE
    };

    return send_command(packet, response, timeout_ms);
}

void DynamixelController::move_servo_fast(float left_deg, float right_deg)
{
    uint32_t left_bits, right_bits, zero = 0;
    memcpy(&left_bits,  &left_deg,  sizeof(left_bits));
    memcpy(&right_bits, &right_deg, sizeof(right_bits));

    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_MOVE_SERVO_FAST,
        left_bits,
        right_bits,
        zero,
        zero,
        DYN_END_BYTE
    };

    _serial->write((const uint8_t*)&packet, sizeof(packet));
}

bool DynamixelController::stop(Dyn_Response_Packet* response,
                               unsigned long timeout_ms)
{
    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_STOP,
        0, 0, 0, 0,
        DYN_END_BYTE
    };

    return send_command(packet, response, timeout_ms);
}

bool DynamixelController::ping(unsigned long timeout_ms)
{
    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_PING,
        0, 0, 0, 0,
        DYN_END_BYTE
    };

    Dyn_Response_Packet resp = {};
    bool ok = send_command(packet, &resp, timeout_ms);
    return ok && (resp.response == DYN_PONG);
}
