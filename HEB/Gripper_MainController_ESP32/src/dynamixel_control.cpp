#include "dynamixel_control.h"

static bool clamp_between(float* v, float a, float b)
{
    float lo = (a < b) ? a : b;
    float hi = (a < b) ? b : a;
    // NaN fails both comparisons below; pin it to the closed end instead.
    if (*v != *v) { *v = a; return true; }
    if (*v < lo)  { *v = lo; return true; }
    if (*v > hi)  { *v = hi; return true; }
    return false;
}

bool DynamixelController::clamp_to_range(float* id0_deg, float* id1_deg)
{
    bool c0 = clamp_between(id0_deg, DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG);
    bool c1 = clamp_between(id1_deg, DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG);
    return c0 || c1;
}

void DynamixelController::opening_to_goals(float g, float* id0_deg, float* id1_deg)
{
    if (g != g)    g = 0.0f;
    if (g < 0.0f)  g = 0.0f;
    if (g > 1.0f)  g = 1.0f;
    *id0_deg = DXL_ID0_CLOSED_DEG + g * (DXL_ID0_OPEN_DEG - DXL_ID0_CLOSED_DEG);
    *id1_deg = DXL_ID1_CLOSED_DEG + g * (DXL_ID1_OPEN_DEG - DXL_ID1_CLOSED_DEG);
}

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

bool DynamixelController::move_servo(float id0_deg, float id1_deg,
                                     float id0_speed, float id1_speed,
                                     Dyn_Response_Packet* response,
                                     unsigned long timeout_ms)
{
    clamp_to_range(&id0_deg, &id1_deg);

    // Packet "left" = ID 0, "right" = ID 1 (see dynamixel_control.h).
    uint32_t left_deg_bits, right_deg_bits, left_spd_bits, right_spd_bits;
    memcpy(&left_deg_bits,  &id0_deg,    sizeof(left_deg_bits));
    memcpy(&right_deg_bits, &id1_deg,   sizeof(right_deg_bits));
    memcpy(&left_spd_bits,  &id0_speed,  sizeof(left_spd_bits));
    memcpy(&right_spd_bits, &id1_speed, sizeof(right_spd_bits));

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

void DynamixelController::move_servo_fast(float id0_deg, float id1_deg)
{
    clamp_to_range(&id0_deg, &id1_deg);

    uint32_t left_bits, right_bits, zero = 0;
    memcpy(&left_bits,  &id0_deg,  sizeof(left_bits));
    memcpy(&right_bits, &id1_deg, sizeof(right_bits));

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

bool DynamixelController::move_servo_log(float id0_deg, float id1_deg,
                                         Dyn_Response_Packet* response,
                                         unsigned long timeout_ms)
{
    clamp_to_range(&id0_deg, &id1_deg);

    uint32_t left_bits, right_bits, zero = 0;
    memcpy(&left_bits,  &id0_deg,  sizeof(left_bits));
    memcpy(&right_bits, &id1_deg, sizeof(right_bits));

    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_MOVE_SERVO_LOG,
        left_bits,
        right_bits,
        zero,
        zero,
        DYN_END_BYTE
    };

    return send_command(packet, response, timeout_ms);
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

bool DynamixelController::read_position(float* id0_deg, float* id1_deg,
                                        unsigned long timeout_ms)
{
    Dyn_Command_Packet packet = {
        DYN_START_BYTE,
        DYN_PING,
        0, 0, 0, 0,
        DYN_END_BYTE
    };

    Dyn_Response_Packet resp = {};
    if (!send_command(packet, &resp, timeout_ms) || resp.response != DYN_PONG)
        return false;

    float p0, p1;
    memcpy(&p0, &resp.left_motor_position,  sizeof(float));
    memcpy(&p1, &resp.right_motor_position, sizeof(float));

    // Reject samples that are known glitch patterns from a failed bus read
    // on the Mega side, rather than passing garbage to callers:
    //  - NaN/inf or wildly out of range (misframed float)
    //  - ID1 exactly 0.0: ID1's range is 90..180, so 0.0 means a failed read
    //  - both exactly equal: seen when one servo's value was copied into the
    //    other slot; the two ranges barely overlap, so this is never real
    if (!isfinite(p0) || !isfinite(p1) ||
        fabsf(p0) > 1500.0f || fabsf(p1) > 1500.0f ||
        p1 == 0.0f || p0 == p1)
        return false;

    if (id0_deg) *id0_deg = p0;
    if (id1_deg) *id1_deg = p1;
    return true;
}
