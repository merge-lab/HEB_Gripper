#include "haptic_motor_control.h"

void HapticMotorController::send_packet(Command_Packet packet)
{
    SerialConnection->write((uint8_t*)&packet, sizeof(packet));
}

bool HapticMotorController::send_command(const Command_Packet& packet, Response_Packet* recieved)
{
    Command_Packet to_send = packet;

    if (temp_sensor != nullptr)
    {
        unsigned long now = millis();
        if (now >= next_thermal_check)
        {
            thermal_paused = temp_sensor->is_temp_exceeded();
            next_thermal_check = now + (thermal_paused ? THERMAL_RECHECK_INTERVAL_MS
                                                        : THERMAL_CHECK_INTERVAL_MS);
        }

        // Override any in-flight force/spring/vibrate/limits request while
        // too hot; RELAX itself passes through unchanged.
        if (thermal_paused && to_send.command != RELAX)
        {
            to_send = Command_Packet{ START_BYTE, RELAX, 0, 0, 0, 0, 0, 0, END_BYTE };
        }
    }

    send_packet(to_send);

    // Fire-and-forget: caller didn't ask for a response.
    if (recieved == NULL)
        return true;

    bool valid_check = false;
    *recieved = read_response(&valid_check, 500);
    return valid_check;
}

Response_Packet HapticMotorController::read_response(bool* valid, unsigned long timeout)
{
    Response_Packet result;
    if (valid != nullptr) *valid = false;

    unsigned long start_time = millis();

    // -----------------------------
    // 1. Find START_BYTE (resync safe)
    // -----------------------------
    bool start_found = false;

    while (millis() - start_time < timeout)
    {
        if (SerialConnection->available())
        {
            char b = SerialConnection->read();

            if (b == START_BYTE)
            {
                result.startByte = START_BYTE;
                start_found = true;
                break;
            }
        }
    }

    if (!start_found)
        return result;

    // -----------------------------
    // 2. Read remaining struct bytes
    // -----------------------------
    size_t remaining = sizeof(Response_Packet) - 1;
    uint8_t* ptr = (uint8_t*)&result;
    size_t index = 1;

    while (remaining > 0 && (millis() - start_time < timeout))
    {
        if (SerialConnection->available())
        {
            ptr[index++] = SerialConnection->read();
            remaining--;
        }
    }

    // timeout or incomplete packet
    if (remaining != 0)
        return result;

    // -----------------------------
    // 3. Validate END_BYTE
    // -----------------------------
    if (result.endByte != END_BYTE)
        return result;

    // -----------------------------
    // 4. Mark valid
    // -----------------------------
    if (valid != nullptr)
        *valid = true;

    return result;
}

bool HapticMotorController::ping_controller(unsigned long wait_time)
{
    // 1. Build ping packet
    Command_Packet packet =
    {
        START_BYTE,
        PING,
        0, 0, 0, 0, 0, 0,
        END_BYTE
    };

    // 2. Send it
    send_packet(packet);

    // 3. Read response
    bool valid = false;
    Response_Packet resp = read_response(&valid, wait_time);

    // 4. Validate response
    if (!valid)
        return false;

    return resp.response == PONG;
}

bool HapticMotorController::set_constant_force(float force, Response_Packet* recieved)
{
    // Torque is a float, but params travel as uint32_t. Bit-cast (not numeric
    // convert) so the fractional part survives the wire.
    uint32_t force_bits;
    memcpy(&force_bits, &force, sizeof(force_bits));

    Command_Packet packet =
    {
        START_BYTE,
        CONSTANT_FORCE,
        force_bits,
        0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::set_constant_newton_force(float force, Response_Packet* recieved)
{
    // Newtons = 3.18x -.122
    float newtons = (force + 0.122) / 3.18;
    return set_constant_force(newtons, recieved);
}

bool HapticMotorController::relax(Response_Packet* recieved)
{
    // RELAX takes no params; end all intentional force.
    Command_Packet packet =
    {
        START_BYTE,
        RELAX,
        0, 0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::vibrate(int frequency, float amplitude, Response_Packet* recieved)
{
    // Vibrate on top of the current base mode.
    // param_1 = frequency (integer Hz); param_2 = amplitude (bit-cast float).
    uint32_t amp_bits;
    memcpy(&amp_bits, &amplitude, sizeof(amp_bits));

    Command_Packet packet =
    {
        START_BYTE,
        VIBRATE,
        (uint32_t)frequency,
        amp_bits,
        0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

bool HapticMotorController::set_spring_force(float spring_constant, Response_Packet* recieved)
{
    // The spring constant is a float, but params are transmitted as uint32_t.
    // Bit-cast (not numeric convert) so the fractional part survives the wire.
    uint32_t spring_bits;
    memcpy(&spring_bits, &spring_constant, sizeof(spring_bits));

    Command_Packet packet =
    {
        START_BYTE,
        SPRING_FORCE,
        spring_bits,
        0, 0, 0, 0, 0,
        END_BYTE
    };

    return send_command(packet, recieved);
}

float HapticMotorController::find_calibration_limit(float force)
{
    const float        TOLERANCE  = 0.5f;
    const int          STABLE_REQ = 5;
    const unsigned long SAMPLE_MS  = 50;
    const unsigned long TIMEOUT_MS = 5000;

    float prev    = 0.0f;
    bool  has_prev = false;
    int   stable  = 0;
    unsigned long t0 = millis();

    while (millis() - t0 < TIMEOUT_MS)
    {
        Response_Packet r;
        if (set_constant_force(force, &r))
        {
            float enc;
            memcpy(&enc, &r.encoder_value, sizeof(enc));

            if (has_prev && fabsf(enc - prev) < TOLERANCE)
            {
                if (++stable >= STABLE_REQ)
                    return enc;
            }
            else
            {
                stable = 0;
            }
            prev     = enc;
            has_prev = true;
        }
        delay(SAMPLE_MS);
    }
    return 999999.0f;  // timeout sentinel
}

bool HapticMotorController::calibrate_trigger(float cal_force)
{
    float lim1 = find_calibration_limit(cal_force);
    if (lim1 == 999999.0f) { relax(nullptr); return false; }

    relax(nullptr);
    delay(300);

    float lim2 = find_calibration_limit(-cal_force);
    if (lim2 == 999999.0f) { relax(nullptr); return false; }

    relax(nullptr);

    LIMIT_OPEN   = (lim1 > lim2) ? lim1 : lim2;
    LIMIT_CLOSE  = (lim1 > lim2) ? lim2 : lim1;
    last_encoder = LIMIT_OPEN;
    calibrated   = true;
    send_limits();
    return true;
}

bool HapticMotorController::set_limit_gate(bool enabled)
{
    if (enabled && calibrated)
        return send_limits();
    // Release limit far beyond any reachable position: never "at limit", and
    // never inside the fade zone.
    const float far = 1.0e9f, zero = 0.0f, ramp = 1.0f, k = 3.0f;
    uint32_t p1, p2, p3, p4, p5;
    memcpy(&p1, &far,  4);
    memcpy(&p2, &zero, 4);
    memcpy(&p3, &zero, 4);
    memcpy(&p4, &ramp, 4);
    memcpy(&p5, &k,    4);
    Command_Packet pkt = { START_BYTE, SET_LIMITS, p1, p2, p3, p4, p5, 0, END_BYTE };
    return send_command(pkt, nullptr);
}

bool HapticMotorController::send_limits()
{
    uint32_t p1, p2, p3, p4, p5;
    memcpy(&p1, &LIMIT_OPEN,       4);
    memcpy(&p2, &LIMIT_THRESHOLD,  4);
    memcpy(&p3, &LIMIT_HYSTERESIS, 4);
    memcpy(&p4, &LIMIT_RAMP_ZONE,  4);
    memcpy(&p5, &LIMIT_RAMP_EXP,   4);
    Command_Packet pkt = { START_BYTE, SET_LIMITS, p1, p2, p3, p4, p5, 0, END_BYTE };
    return send_command(pkt, nullptr);
}

bool HapticMotorController::update_force_slip(float normalized_force, float normalized_slip,
                                              Response_Packet* resp_out)
{
    // Force: [0,1] → [MIN_FORCE_OUT, MAX_FORCE_OUT], sent as resistance (negative)
    float force_out = MIN_FORCE_OUT + normalized_force * (MAX_FORCE_OUT - MIN_FORCE_OUT);

    // Slip: dead zone then inverse-linear from MAX_FREQ down to MIN_FREQ
    float dead_zone_norm = SLIP_DEAD_ZONE / (MAX_SLIP - MIN_SLIP);
    int freq_out;
    if (normalized_slip <= dead_zone_norm) {
        freq_out = 0;
    } else {
        float t = (normalized_slip - dead_zone_norm) / (1.0f - dead_zone_norm);
        freq_out = (int)(MAX_FREQ - t * (MAX_FREQ - MIN_FREQ));
    }

    bool ok = true;

    if (calibrated)
    {
        // Use a local packet buffer if the caller doesn't need the response,
        // so we can always update last_encoder.
        Response_Packet local_resp;
        Response_Packet* rp = resp_out ? resp_out : &local_resp;

        bool at_limit = (last_encoder >= LIMIT_OPEN  - LIMIT_THRESHOLD) ||
                        (last_encoder <= LIMIT_CLOSE + LIMIT_THRESHOLD);

        if (at_limit)
        {
            // Cease actuation; still read encoder to track when trigger moves away.
            ok = set_constant_force(0.0f, rp);
        }
        else
        {
            if (freq_out != current_frequency)
            {
                current_frequency = freq_out;
                ok &= vibrate(freq_out, amplitude, NULL);
            }
            current_force = force_out;
            ok &= set_constant_force(-force_out, rp);
        }

        if (ok)
            memcpy(&last_encoder, &rp->encoder_value, sizeof(last_encoder));

        return ok;
    }

    // Not calibrated — original behavior, no encoder read overhead.
    if (freq_out != current_frequency)
    {
        current_frequency = freq_out;
        ok &= vibrate(freq_out, amplitude, NULL);
    }
    current_force = force_out;
    ok &= set_constant_force(-force_out, resp_out);
    return ok;
}
