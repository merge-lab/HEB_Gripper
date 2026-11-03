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
    // That also discards any pending asynchronous reply.
    _rx_idx = 0;
    _req_outstanding = false;

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
    _bad_pose_streak = 0;

    // Rate-limit the message: tm/hu may hit this every few ms.
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
    if (pose_in_range(id0_deg, id1_deg)) {
        _bad_pose_streak = 0;
        return true;
    }
    // One out-of-range sample is usually a bad read on the Mega (e.g. one
    // servo's value landing in the other's slot). Act when the next agrees.
    if (++_bad_pose_streak < 2)
        return true;
    reject_and_relax("present position out of range", true, id0_deg, id1_deg);
    return false;
}

void DynamixelController::note_feedback(float p0, float p1)
{
    _fb0 = p0;
    _fb1 = p1;
    _fb_ms = millis();
    _fb_new = true;
}

bool DynamixelController::latest_feedback(float* id0_deg, float* id1_deg, unsigned long* age_ms) const
{
    if (_fb_ms == 0) return false;
    if (id0_deg) *id0_deg = _fb0;
    if (id1_deg) *id1_deg = _fb1;
    if (age_ms)  *age_ms  = millis() - _fb_ms;
    return true;
}

void DynamixelController::poll_feedback()
{
    // Non-blocking: consume whatever bytes have arrived, frame packets.
    while (_serial->available()) {
        uint8_t b = (uint8_t)_serial->read();
        if (_rx_idx == 0 && b != (uint8_t)DYN_START_BYTE)
            continue;                                   // resync
        _rx_buf[_rx_idx++] = b;
        if (_rx_idx < sizeof(Dyn_Response_Packet))
            continue;
        _rx_idx = 0;

        Dyn_Response_Packet r;
        memcpy(&r, _rx_buf, sizeof(r));
        if (r.endByte != DYN_END_BYTE)
            continue;

        // Any well-framed reply means the Mega is free again, and every reply
        // type carries freshly read positions.
        if (_req_outstanding) {
            _fb_latency_ms = millis() - _req_ms;
            _req_outstanding = false;
        }
        if (r.response != DYN_ACK && r.response != DYN_PONG)
            _bad_replies++;
        float p0, p1;
        memcpy(&p0, &r.left_motor_position,  sizeof(float));
        memcpy(&p1, &r.right_motor_position, sizeof(float));
        if (sample_valid(p0, p1))
            note_feedback(p0, p1);
    }
}

void DynamixelController::seed_shaper(float p0, float p1)
{
    // Start from where the fingers are (clamped into the command range so
    // the first goals are valid), at rest.
    float lo0 = fminf(DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG), hi0 = fmaxf(DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG);
    float lo1 = fminf(DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG), hi1 = fmaxf(DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG);
    _sm_pos[0] = fminf(fmaxf(p0, lo0), hi0);
    _sm_pos[1] = fminf(fmaxf(p1, lo1), hi1);
    _sm_vel[0] = _sm_vel[1] = 0.0f;
    _sm_last_us = micros();
    _sm_ready = true;
}

void DynamixelController::shape_goals(float* id0_deg, float* id1_deg)
{
    unsigned long now = micros();
    float dt = (now - _sm_last_us) * 1e-6f;
    _sm_last_us = now;
    if (dt > 0.05f) dt = 0.05f;          // after a pause, don't take a huge step

    if (!_sm_ready) {                    // shouldn't happen: seeded at engage
        seed_shaper(*id0_deg, *id1_deg);
        return;
    }

    const float V = GRIPPER_MAX_DEG_PER_S, A = GRIPPER_MAX_DEG_PER_S2;
    float target[2] = { *id0_deg, *id1_deg };
    float lo[2] = { fminf(DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG), fminf(DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG) };
    float hi[2] = { fmaxf(DXL_ID0_CLOSED_DEG, DXL_ID0_OPEN_DEG), fmaxf(DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG) };

    for (int i = 0; i < 2; i++) {
        float d = target[i] - _sm_pos[i];
        if (d == 0.0f) { _sm_vel[i] = 0.0f; continue; }

        // Fastest speed from which we can still stop at the target, capped.
        float v_des = sqrtf(2.0f * A * fabsf(d));
        if (v_des > V) v_des = V;
        if (d < 0.0f) v_des = -v_des;

        float dv = v_des - _sm_vel[i], dv_max = A * dt;
        if (dv >  dv_max) dv =  dv_max;
        if (dv < -dv_max) dv = -dv_max;
        _sm_vel[i] += dv;

        float step = _sm_vel[i] * dt;
        if ((d > 0.0f && step >= d) || (d < 0.0f && step <= d)) {
            _sm_pos[i] = target[i];                       // arrive, no overshoot
            _sm_vel[i] = (dt > 0.0f) ? fmaxf(-V, fminf(V, d / dt)) : 0.0f;
        } else {
            _sm_pos[i] += step;
        }
        // Never command outside the range (e.g. while decelerating after the
        // target reversed).
        if (_sm_pos[i] < lo[i]) { _sm_pos[i] = lo[i]; _sm_vel[i] = 0.0f; }
        if (_sm_pos[i] > hi[i]) { _sm_pos[i] = hi[i]; _sm_vel[i] = 0.0f; }
    }
    *id0_deg = _sm_pos[0];
    *id1_deg = _sm_pos[1];
}

bool DynamixelController::pose_guard(bool* engaged)
{
    *engaged = false;
    unsigned long now = millis();
    bool may_engage = _torque_maybe_off || (now - _last_tx_ms) >= HEARTBEAT_SAFE_MS;

    if (may_engage) {
        // Torque may come on with this move: strict, blocking. Two tries,
        // since read_position drops glitch samples; an out-of-range reading
        // is re-read once before refusing.
        float p0 = 0.0f, p1 = 0.0f;
        if (!(read_position(&p0, &p1, 200) || read_position(&p0, &p1, 200))) {
            reject_and_relax("could not read servo positions", false, 0.0f, 0.0f);
            return false;
        }
        if (!pose_in_range(p0, p1)) {
            float q0, q1;
            if (read_position(&q0, &q1, 200) && pose_in_range(q0, q1)) {
                p0 = q0; p1 = q1;
            } else {
                reject_and_relax("present position out of range", true, p0, p1);
                return false;
            }
        }
        _bad_pose_streak = 0;
        _fb_new = false;
        _last_pose_check_ms = millis();
        seed_shaper(p0, p1);
        *engaged = true;
        return true;
    }

    // Already moving: asynchronous, never wait here.
    poll_feedback();
    if (_fb_new) {
        _fb_new = false;
        if (!check_pose_or_relax(_fb0, _fb1))
            return false;
    }
    if (millis() - _fb_ms > POSE_STALE_MS) {
        reject_and_relax("lost servo position feedback (no reply for 500 ms)", false, 0.0f, 0.0f);
        return false;
    }
    return true;
}

bool DynamixelController::check_reply_pose(const Dyn_Response_Packet* r)
{
    float p0, p1;
    memcpy(&p0, &r->left_motor_position,  sizeof(float));
    memcpy(&p1, &r->right_motor_position, sizeof(float));
    if (!sample_valid(p0, p1))
        return true;                     // glitch sample: ignore
    note_feedback(p0, p1);
    _fb_new = false;
    _last_pose_check_ms = millis();
    return check_pose_or_relax(p0, p1);
}

bool DynamixelController::move_servo(float id0_deg, float id1_deg,
                                     float id0_speed, float id1_speed,
                                     Dyn_Response_Packet* response,
                                     unsigned long timeout_ms)
{
    // One-shot move (m, mw): sent as given, no shaping; the servo's profile
    // speed (id*_speed) governs the motion.
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return false;
    }
    bool engaged;
    if (!pose_guard(&engaged)) return false;
    _last_blocked = false;

    // Packet "left" = ID 0, "right" = ID 1 (see dynamixel_control.h).
    uint32_t left_deg_bits, right_deg_bits, left_spd_bits, right_spd_bits;
    memcpy(&left_deg_bits,  &id0_deg,   sizeof(left_deg_bits));
    memcpy(&right_deg_bits, &id1_deg,   sizeof(right_deg_bits));
    memcpy(&left_spd_bits,  &id0_speed, sizeof(left_spd_bits));
    memcpy(&right_spd_bits, &id1_speed, sizeof(right_spd_bits));
    Dyn_Command_Packet packet = {
        DYN_START_BYTE, DYN_MOVE_SERVO,
        left_deg_bits, right_deg_bits, left_spd_bits, right_spd_bits,
        DYN_END_BYTE
    };

    bool ok = send_command(packet, response, timeout_ms);
    if (ok) {
        _torque_maybe_off = false;
        _last_move_tx_ms = millis();
        _sent0 = id0_deg; _sent1 = id1_deg;
        // A later stream continues from this goal rather than jumping.
        _sm_pos[0] = id0_deg; _sm_pos[1] = id1_deg;
        _sm_vel[0] = _sm_vel[1] = 0.0f;
        _sm_last_us = micros();
        _sm_ready = true;
    }
    if (ok && response && !check_reply_pose(response)) return false;
    return ok;
}

void DynamixelController::move_servo_fast(float id0_deg, float id1_deg)
{
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return;
    }
    bool engaged;
    if (!pose_guard(&engaged)) return;
    _last_blocked = false;
    shape_goals(&id0_deg, &id1_deg);

    unsigned long now = millis();
    if (!engaged) {
        // Don't talk over the Mega while it answers a feedback request.
        if (_req_outstanding) {
            if (now - _req_ms < POSE_REQ_TIMEOUT_MS) return;
            _req_outstanding = false;                     // reply lost
        }
        if (now - _last_move_tx_ms < MOVE_MIN_INTERVAL_MS) return;
    }
    bool want_fb = !engaged && (now - _last_pose_check_ms) >= POSE_CHECK_MS;
    bool changed = !(fabsf(id0_deg - _sent0) < MOVE_DEADBAND_DEG) ||
                   !(fabsf(id1_deg - _sent1) < MOVE_DEADBAND_DEG);   // NaN -> changed
    if (!engaged && !want_fb && !changed)
        return;                                           // nothing new to say

    uint32_t left_bits, right_bits, zero = 0;
    memcpy(&left_bits,  &id0_deg, sizeof(left_bits));
    memcpy(&right_bits, &id1_deg, sizeof(right_bits));
    // MOVE_SERVO_LOG = same move, plus a reply carrying positions (feedback).
    Dyn_Command_Packet packet = {
        DYN_START_BYTE, (uint8_t)(want_fb ? DYN_MOVE_SERVO_LOG : DYN_MOVE_SERVO_FAST),
        left_bits, right_bits, zero, zero,
        DYN_END_BYTE
    };
    _serial->write((const uint8_t*)&packet, sizeof(packet));

    _last_tx_ms = _last_move_tx_ms = now;
    _sent0 = id0_deg; _sent1 = id1_deg;
    _torque_maybe_off = false;
    if (want_fb) {
        _req_outstanding = true;
        _req_ms = now;
        _last_pose_check_ms = now;
    }
}

bool DynamixelController::move_servo_log(float id0_deg, float id1_deg,
                                         Dyn_Response_Packet* response,
                                         unsigned long timeout_ms)
{
    // Blocking variant (tml): every move waits for its position reply.
    if (!command_in_range(id0_deg, id1_deg)) {
        reject_and_relax("commanded goal out of range", true, id0_deg, id1_deg);
        return false;
    }
    bool engaged;
    if (!pose_guard(&engaged)) return false;
    _last_blocked = false;
    shape_goals(&id0_deg, &id1_deg);

    uint32_t left_bits, right_bits, zero = 0;
    memcpy(&left_bits,  &id0_deg, sizeof(left_bits));
    memcpy(&right_bits, &id1_deg, sizeof(right_bits));
    Dyn_Command_Packet packet = {
        DYN_START_BYTE, DYN_MOVE_SERVO_LOG,
        left_bits, right_bits, zero, zero,
        DYN_END_BYTE
    };

    bool ok = send_command(packet, response, timeout_ms);
    if (ok) {
        _torque_maybe_off = false;
        _last_move_tx_ms = millis();
        _sent0 = id0_deg; _sent1 = id1_deg;
    }
    if (ok && response && !check_reply_pose(response)) return false;
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

    _torque_maybe_off = true;   // next move re-engages (strict check + reseed)
    _sm_ready = false;
    _sent0 = _sent1 = NAN;
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
    note_feedback(p0, p1);

    if (id0_deg) *id0_deg = p0;
    if (id1_deg) *id1_deg = p1;
    return true;
}
