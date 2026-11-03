#include "dynamixel_control.h"

static bool within(float v, float a, float b, float tol)
{
    float lo = ((a < b) ? a : b) - tol;
    float hi = ((a < b) ? b : a) + tol;
    return isfinite(v) && v >= lo && v <= hi;   // NaN/inf -> false
}

// Known glitch patterns from a failed bus read on the Mega side:
//  - NaN/inf or wildly out of range (misframed float)
//  - ID1 exactly 0.0: ID1 never sits at 0, so 0.0 means a failed read
//  - both exactly equal: one servo's value copied into the other slot
static bool sample_valid(float p0, float p1)
{
    return isfinite(p0) && isfinite(p1) &&
           fabsf(p0) <= 1500.0f && fabsf(p1) <= 1500.0f &&
           p1 != 0.0f && p0 != p1;
}

bool DynamixelController::command_in_range(float id0_deg, float id1_deg)
{
    const float eps = 1e-3f;   // float rounding at the range ends
    return within(id0_deg, DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG, eps) &&
           within(id1_deg, DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG, eps);
}

bool DynamixelController::pose_in_range(float id0_deg, float id1_deg)
{
    return within(id0_deg, DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG, DXL_POSE_TOL_DEG) &&
           within(id1_deg, DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG, DXL_POSE_TOL_DEG);
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
    _last_tx_ms = millis();   // any command resets the Mega's heartbeat

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

void DynamixelController::reject_and_relax(const char* why, bool have_pose, float p0, float p1)
{
    // Relax first, then explain. STOP is fire-and-forget so a dead link
    // can't stall the caller.
    stop(nullptr);
    _last_blocked = true;

    // Rate-limit the message: tm/hu may hit this every 15 ms.
    if (millis() - _last_warn_ms > 1000UL) {
        _last_warn_ms = millis();
        if (have_pose)
            Serial.printf("[DXL GUARD] %s -> servos RELAXED. ID0=%.2f (allowed %.0f..%.0f)  ID1=%.2f (allowed %.0f..%.0f)\n",
                          why, p0, DXL_ID0_OPEN_DEG, DXL_ID0_CLOSED_DEG,
                          p1, DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG);
        else
            Serial.printf("[DXL GUARD] %s -> servos RELAXED.\n", why);
    }
}

bool DynamixelController::check_pose_or_relax(float id0_deg, float id1_deg)
{
    if (pose_in_range(id0_deg, id1_deg))
        return true;
    reject_and_relax("present position out of range", true, id0_deg, id1_deg);
    return false;
}

bool DynamixelController::pose_guard()
{
    unsigned long now = millis();
    bool may_engage = _torque_maybe_off || (now - _last_tx_ms) >= HEARTBEAT_SAFE_MS;
    bool due        = (now - _last_pose_check_ms) >= POSE_CHECK_MS;
    if (!may_engage && !due)
        return true;

    // Two tries, since read_position drops glitch samples.
    float p0 = 0.0f, p1 = 0.0f;
    if (!(read_position(&p0, &p1, 200) || read_position(&p0, &p1, 200))) {
        reject_and_relax("could not read servo positions", false, 0.0f, 0.0f);
        return false;
    }
    _last_pose_check_ms = millis();
    return check_pose_or_relax(p0, p1);
}

bool DynamixelController::move_servo(float id0_deg, float id1_deg,
                                     float id0_speed, float id1_speed,
                                     Dyn_Response_Packet* response,
                                     unsigned long timeout_ms)
{
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return false;
    }
    if (!pose_guard()) return false;
    _last_blocked = false;

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

    bool ok = send_command(packet, response, timeout_ms);
    if (ok) _torque_maybe_off = false;
    if (ok && response) {
        float p0, p1;
        memcpy(&p0, &response->left_motor_position,  sizeof(float));
        memcpy(&p1, &response->right_motor_position, sizeof(float));
        if (sample_valid(p0, p1)) {
            _last_pose_check_ms = millis();
            if (!check_pose_or_relax(p0, p1)) return false;
        }
    }
    return ok;
}

void DynamixelController::move_servo_fast(float id0_deg, float id1_deg)
{
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return;
    }
    if (!pose_guard()) return;
    _last_blocked = false;

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
    _last_tx_ms       = millis();
    _torque_maybe_off = false;
}

bool DynamixelController::move_servo_log(float id0_deg, float id1_deg,
                                         Dyn_Response_Packet* response,
                                         unsigned long timeout_ms)
{
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return false;
    }
    if (!pose_guard()) return false;
    _last_blocked = false;

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

    bool ok = send_command(packet, response, timeout_ms);
    if (ok) _torque_maybe_off = false;
    if (ok && response) {
        float p0, p1;
        memcpy(&p0, &response->left_motor_position,  sizeof(float));
        memcpy(&p1, &response->right_motor_position, sizeof(float));
        if (sample_valid(p0, p1)) {
            _last_pose_check_ms = millis();
            if (!check_pose_or_relax(p0, p1)) return false;
        }
    }
    return ok;
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

    _torque_maybe_off = true;   // next move re-enables torque
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

    if (!sample_valid(p0, p1))
        return false;

    if (id0_deg) *id0_deg = p0;
    if (id1_deg) *id1_deg = p1;
    return true;
}
