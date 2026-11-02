#include "CLI_Control.h"
#include <esp_timer.h>
#include <math.h>

namespace {
constexpr uint8_t SWITCH_40_PIN = 40;
constexpr uint8_t SWITCH_41_PIN = 41;
constexpr uint8_t SWITCH_LED_PIN = 42;

volatile bool ledOn = false;
esp_timer_handle_t ledBlinkTimer = nullptr;

void blinkLed(void*) {
    ledOn = !ledOn;
    digitalWrite(SWITCH_LED_PIN, ledOn ? HIGH : LOW);
}

const char* ledState() {
    return ledOn ? "ON" : "OFF";
}

const char* switchState(uint8_t pin) {
    // GPIO -> switch -> GND: closed is ON, open is pulled HIGH.
    return digitalRead(pin) == LOW ? "ON" : "OFF";
}
}

// Send the current vibration setting and report the ACK.
void applyVibrate(int freq, float amp, HapticMotorController* motor) {
  Response_Packet r;
  if (motor->vibrate(freq, amp, &r))
    Serial.printf("  -> vibrate %d Hz, amp %.3f\n", freq, amp);
  else
    Serial.println("  -> vibrate no ACK (rejected? freq must be 0..400)");
}

// Ramp constant force 0.0 -> 1.5 in 0.1 steps, holding each level for a short
// while (re-sending inside the STM's 500 ms heartbeat window) and printing the
// response. Relaxes the motor on completion. Blocking - returns when done.
void force_ramp_up(HapticMotorController* motor) {
  const int   STEPS       = 15;   // 0.0 .. 1.5 in 0.1 increments (integer count avoids float drift)
  const int   HOLD_CYCLES = 8;    // ~1.6 s per level at 200 ms/cycle (< 500 ms heartbeat per send)

  Serial.println("RAMP UP: constant force 0.00 -> 1.50 in 0.10 steps");

  for (int step = 0; step <= STEPS; step++) {
    float force = step * 0.1f;

    for (int i = 0; i < HOLD_CYCLES; i++) {
      Response_Packet resp;
      if (motor->set_constant_force(-force, &resp)) {
        float pos_deg, torque, motor_t, user_t;
        memcpy(&pos_deg, &resp.encoder_value,   sizeof(pos_deg));
        memcpy(&torque,  &resp.motor_value,     sizeof(torque));
        memcpy(&motor_t, &resp.measured_torque, sizeof(motor_t));
        memcpy(&user_t,  &resp.user_torque,     sizeof(user_t));
        unsigned long raw_enc = (unsigned long)resp.raw_encoder;
        Serial.printf("[RAMP con] set=%+.3f pos=%+.2f enc=%lu cmd=%+.3f motorT=%+.3f userT=%+.3f switch40=%s switch41=%s led42=%s\n",
                      force, pos_deg, raw_enc, torque, motor_t, user_t, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
      } else {
        Serial.printf("no ACK (rejected or timed out) switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
      }
      delay(200);
    }
  }

  motor->relax(NULL);
  Serial.println("RAMP UP complete - relaxed");
}

void trigger_mapping(HapticMotorController* motor, DynamixelController* gripper)
{
    float POS_CLOSE, POS_OPEN;
    float force = -0.75f;
    String input;
    input.reserve(80);
    bool inputOverflow = false;
    unsigned long lastInputMs = 0;

    if (motor->is_calibrated())
    {
        // Use measured mechanical limits; exact range replaces the hardcoded 47°.
        POS_CLOSE = motor->get_limit_close();
        float range = motor->get_limit_open() - motor->get_limit_close();
        POS_OPEN  = POS_CLOSE - range;
        Serial.printf("Trigger mapping ON (calibrated): close=%.2f open=%.2f. Type 'e' + Enter to exit.\n",
                      POS_CLOSE, POS_OPEN);
    }
    else
    {
        // Fallback: read current encoder position with trigger fully closed.
        Response_Packet init_resp;
        while (!motor->relax(&init_resp))
            delay(200);
        memcpy(&POS_CLOSE, &init_resp.encoder_value, sizeof(POS_CLOSE));
        POS_OPEN = POS_CLOSE - 47.0f;
        Serial.printf("Trigger mapping ON (uncalibrated): close=%.2f open=%.2f. Type 'e' + Enter to exit.\n",
                      POS_CLOSE, POS_OPEN);
    }

    Serial.println("TM commands: c <force>, v <hz> <amp> (0 Hz or amp 0 = off), e to exit.");

    while (true)
    {
        // Accept LF, CR, CRLF, or the normal Serial idle timeout when the
        // terminal sends no line ending. Never block mapping while waiting.
        // Bound work and storage to keep mapping responsive.
        for (int bytes = 0; bytes < 64; ++bytes)
        {
            char ch;
            if (Serial.available() > 0)
            {
                int received = Serial.read();
                if (received < 0) break;
                ch = static_cast<char>(received);
                lastInputMs = millis();
            }
            else if ((input.length() > 0 || inputOverflow) &&
                     static_cast<unsigned long>(millis() - lastInputMs) >= Serial.getTimeout())
            {
                // Match readStringUntil()'s fallback without its blocking wait.
                ch = '\n';
            }
            else
                break;

            if (ch != '\n' && ch != '\r')
            {
                if (input.length() < 79 && !inputOverflow)
                    input += ch;
                else
                    inputOverflow = true;
                continue;
            }

            String line = input;
            input = "";
            if (inputOverflow)
            {
                inputOverflow = false;
                Serial.println("[TM] command too long; discarded.");
                continue;
            }
            line.trim();
            if (line.length() == 0) continue;
            if (line.equalsIgnoreCase("e"))
            {
                motor->relax(nullptr);
                if (gripper) gripper->stop(nullptr);
                Serial.println("Trigger mapping OFF.");
                return;
            }

            String arg = line.substring(1);
            char extra;
            if (line.charAt(0) == 'c' || line.charAt(0) == 'C')
            {
                float requestedForce;
                if (sscanf(arg.c_str(), " %f %c", &requestedForce, &extra) == 1 &&
                    isfinite(requestedForce))
                {
                    // Match the normal CLI: positive input applies negative torque.
                    force = -requestedForce;
                    Serial.printf("[TM] base = constant %.3f\n", force);
                }
                else
                    Serial.println("[TM] usage: c <force>");
            }
            else if (line.charAt(0) == 'v' || line.charAt(0) == 'V')
            {
                int frequency;
                float amplitude;
                if (sscanf(arg.c_str(), " %d %f %c", &frequency, &amplitude, &extra) == 2 &&
                    frequency >= 0 && frequency <= 400 && isfinite(amplitude) && amplitude >= 0.0f)
                    applyVibrate(frequency, amplitude, motor);
                else
                    Serial.println("[TM] usage: v <hz 0..400> <amp >= 0>");
            }
            else
                Serial.println("[TM] commands: c <force>, v <hz> <amp>, e");
        }

        // Keep the selected base force and heartbeat active; the STM32 retains
        // the vibration overlay and handles limit gating and ramp at 1 kHz.
        Response_Packet resp;
        bool ok = motor->set_constant_force(force, &resp);

        if (!ok)
        {
            Serial.printf("[TM] no ACK from haptic motor switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
            delay(200);
            continue;
        }

        float pos_deg;
        memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));

        float t = -(pos_deg - POS_CLOSE) / (POS_OPEN - POS_CLOSE);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float left_deg  = 90.0f * (1.0f - t);
        float right_deg = 90.0f + 90.0f * t;

        if (gripper)
            gripper->move_servo_fast(left_deg, right_deg);

        Serial.printf("[TM] pos=%+.2f t=%.2f L=%.1f R=%.1f switch40=%s switch41=%s led42=%s\n",
                      pos_deg, t, left_deg, right_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());

        delay(15);
    }
}

// Same as trigger_mapping(), but reads back and logs actual servo position
// each cycle (slower — move_servo() blocks for the Dynamixel's response).
// Used by response_plot.py to compare expected vs. actual position.
void trigger_mapping_log(HapticMotorController* motor, DynamixelController* gripper)
{
    float POS_CLOSE, POS_OPEN;

    if (motor->is_calibrated())
    {
        POS_CLOSE = motor->get_limit_close();
        float range = motor->get_limit_open() - motor->get_limit_close();
        POS_OPEN  = POS_CLOSE - range;
        Serial.printf("Trigger mapping (logged) ON (calibrated): close=%.2f open=%.2f. Type 'e' + Enter to exit.\n",
                      POS_CLOSE, POS_OPEN);
    }
    else
    {
        Response_Packet init_resp;
        while (!motor->relax(&init_resp))
            delay(200);
        memcpy(&POS_CLOSE, &init_resp.encoder_value, sizeof(POS_CLOSE));
        POS_OPEN = POS_CLOSE - 47.0f;
        Serial.printf("Trigger mapping (logged) ON (uncalibrated): close=%.2f open=%.2f. Type 'e' + Enter to exit.\n",
                      POS_CLOSE, POS_OPEN);
    }

    while (true)
    {
        // Exit check — non-blocking
        if (Serial.available())
        {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.equalsIgnoreCase("e"))
            {
                motor->relax(nullptr);
                if (gripper) gripper->stop(nullptr);
                Serial.println("Trigger mapping OFF.");
                return;
            }
        }

        // STM32 handles limit gating and ramp at 1 kHz — just send full force.
        Response_Packet resp;
        bool ok = motor->set_constant_force(-1.5f, &resp);

        if (!ok)
        {
            Serial.printf("[TM] no ACK from haptic motor switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
            delay(200);
            continue;
        }

        float pos_deg;
        memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));

        float t = -(pos_deg - POS_CLOSE) / (POS_OPEN - POS_CLOSE);
        if (t < 0.0f) t = 0.0f;
        if (t > 1.0f) t = 1.0f;

        float left_deg  = 90.0f * (1.0f - t);
        float right_deg = 90.0f + 90.0f * t;

        float actual_left_deg = 0.0f, actual_right_deg = 0.0f;
        if (gripper)
        {
            Dyn_Response_Packet gripResp;
            // Time the gripper command/response round trip, excluding the
            // STM32 exchange, terminal output, and the loop's 15 ms delay.
            uint32_t latency_start_us = micros();
            bool grip_ok = gripper->move_servo_log(left_deg, right_deg, &gripResp);
            uint32_t latency_us = (uint32_t)(micros() - latency_start_us);
            if (grip_ok)
            {
                memcpy(&actual_left_deg,  &gripResp.left_motor_position,  sizeof(actual_left_deg));
                memcpy(&actual_right_deg, &gripResp.right_motor_position, sizeof(actual_right_deg));
            }
            // Serial.printf("latency = %.3f ms%s\n", latency_us / 1000.0f,
            //               grip_ok ? "" : " (timeout or invalid response)");
        }

        Serial.printf("[TM] pos=%+.2f t=%.2f L=%.1f(act=%.1f) R=%.1f(act=%.1f) switch40=%s switch41=%s led42=%s\n",
                      pos_deg, t, left_deg, actual_left_deg, right_deg, actual_right_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());

        delay(15);
    }
}

void haptic_update_mode(HapticMotorController* motor, DynamixelController* gripper)
{
    const float TRIGGER_RANGE = 47.0f;

    float norm_force = 0.0f;
    float norm_slip  = 0.0f;
    bool  dxl_active = false;
    float POS_CLOSE  = 0.0f;
    float POS_OPEN   = 0.0f;

    Serial.println("HU mode: '<force> <slip>' to set (0-1), 'd' toggle DXL, 'e' exit.");

    while (true)
    {
        if (Serial.available())
        {
            String line = Serial.readStringUntil('\n');
            line.trim();

            if (line.equalsIgnoreCase("e"))
            {
                motor->relax(nullptr);
                if (gripper) gripper->stop(nullptr);
                Serial.println("HU mode OFF.");
                return;
            }
            else if (line.equalsIgnoreCase("d") && gripper)
            {
                dxl_active = !dxl_active;
                if (dxl_active)
                {
                    Response_Packet cal_resp;
                    while (!motor->relax(&cal_resp))
                        delay(200);
                    memcpy(&POS_CLOSE, &cal_resp.encoder_value, sizeof(POS_CLOSE));
                    POS_OPEN = POS_CLOSE - TRIGGER_RANGE;
                    Serial.printf("DXL ON: close=%.2f open=%.2f\n", POS_CLOSE, POS_OPEN);
                }
                else
                {
                    gripper->stop(nullptr);
                    Serial.println("DXL OFF.");
                }
            }
            else
            {
                int sp = line.indexOf(' ');
                if (sp >= 0)
                {
                    norm_force = line.toFloat();
                    norm_slip  = line.substring(sp + 1).toFloat();
                    if (norm_force < 0.0f) norm_force = 0.0f;
                    if (norm_force > 1.0f) norm_force = 1.0f;
                    if (norm_slip  < 0.0f) norm_slip  = 0.0f;
                    if (norm_slip  > 1.0f) norm_slip  = 1.0f;
                    Serial.printf("set: force=%.2f slip=%.2f\n", norm_force, norm_slip);
                }
            }
        }

        if (dxl_active)
        {
            Response_Packet resp;
            bool ok = motor->update_force_slip(norm_force, norm_slip, &resp);
            if (!ok)
            {
                Serial.printf("[HU] no ACK from haptic motor switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
                delay(200);
                continue;
            }
            float pos_deg;
            memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));

            float t = -(pos_deg - POS_CLOSE) / (POS_OPEN - POS_CLOSE);
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;

            float left_deg  = 90.0f * (1.0f - t);
            float right_deg = 90.0f + 90.0f * t;
            gripper->move_servo_fast(left_deg, right_deg);

            Serial.printf("[HU] f=%.2f s=%.2f pos=%+.2f t=%.2f L=%.1f R=%.1f switch40=%s switch41=%s led42=%s\n",
                          norm_force, norm_slip, pos_deg, t, left_deg, right_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
        }
        else
        {
            motor->update_force_slip(norm_force, norm_slip);
            Serial.printf("[HU] f=%.2f s=%.2f switch40=%s switch41=%s led42=%s\n", norm_force, norm_slip, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
        }

        delay(15);
    }
}

void gripper_position_mode(DynamixelController* gripper)
{
    if (!gripper) { Serial.println("no gripper link"); return; }

    // Torque off so the fingers can be moved by hand. Only STOP and PING are
    // sent here, which mean the same thing in every Mega firmware version.
    Dyn_Response_Packet stop_resp;
    if (!gripper->stop(&stop_resp, 300))
        Serial.println("[GP] WARNING: no STOP ACK - torque may still be on");

    Serial.println("Position mode ON: torque OFF - move fingers by hand to each limit.");
    Serial.println("  'r' + Enter resets min/max, 'e' + Enter exits.");

    float l_min =  1e9f, l_max = -1e9f;
    float r_min =  1e9f, r_max = -1e9f;
    int   fails = 0;

    while (true)
    {
        if (Serial.available())
        {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.equalsIgnoreCase("e"))
                break;
            if (line.equalsIgnoreCase("r")) {
                l_min = r_min =  1e9f;
                l_max = r_max = -1e9f;
                Serial.println("[GP] min/max reset");
            }
        }

        float l, r;
        if (gripper->read_position(&l, &r, 200))
        {
            fails = 0;
            if (l < l_min) l_min = l;
            if (l > l_max) l_max = l;
            if (r < r_min) r_min = r;
            if (r > r_max) r_max = r;
            Serial.printf("[GP] L=%7.2f [%7.2f..%7.2f]   R=%7.2f [%7.2f..%7.2f]\n",
                          l, l_min, l_max, r, r_min, r_max);
        }
        else if (++fails % 10 == 1)
        {
            Serial.println("[GP] no PONG from gripper board");
        }

        delay(100);
    }

    Serial.println("Position mode OFF. Measured range:");
    Serial.printf("  Left : %.2f .. %.2f deg (span %.2f)\n", l_min, l_max, l_max - l_min);
    Serial.printf("  Right: %.2f .. %.2f deg (span %.2f)\n", r_min, r_max, r_max - r_min);
}

void CLI_Setup(HapticMotorController* motor, DynamixelController* gripper)
{
    pinMode(SWITCH_40_PIN, INPUT_PULLUP);
    pinMode(SWITCH_41_PIN, INPUT_PULLUP);
    digitalWrite(SWITCH_LED_PIN, LOW);
    pinMode(SWITCH_LED_PIN, OUTPUT);
    // Temporary LED test: toggle every 500 ms, independently of switches/CLI.
    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = blinkLed;
    timerArgs.name = "led42_blink";
    esp_err_t timerResult = esp_timer_create(&timerArgs, &ledBlinkTimer);
    if (timerResult == ESP_OK)
        timerResult = esp_timer_start_periodic(ledBlinkTimer, 500000);
    if (timerResult != ESP_OK)
        Serial.printf("LED blink timer failed: %s\n", esp_err_to_name(timerResult));
    else
        Serial.println("LED test: GPIO 42 flashing, 500 ms ON / 500 ms OFF.");
    Serial.printf("Switches: switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());

    bool pong = motor->ping_controller(500);
    digitalWrite(2, pong ? HIGH : LOW);
    Serial.println(pong ? "haptic link up (PONG)" : "haptic no PONG - check link");

    if (gripper) {
        bool gpong = gripper->ping(300);
        Serial.println(gpong ? "gripper link up (PONG)" : "gripper no PONG - check link");
    }

    Serial.println("Haptic:  s/c/n/f/a/v/sweep/rampup/stop");
    Serial.println("Gripper: m <l> <r> [spd]   gstop   gping   gpos (find range)");
    Serial.println("Modes:   tm (trigger map)   tml (trigger map + log)   hu (haptic update)   cal (calibrate)   sl (send limits)");
    Serial.println("Boots relaxed - send a command to engage.");
}

void CLI(HapticMotorController* motor, DynamixelController* gripper)
{
    static TestMode mode = MODE_SPRING;
    static float value   = 0.0f;
    static int   vibFreq = 0;
    static float vibAmp  = 0.0f;
    static bool  sweep   = false;
    static bool  relaxed = true;          // boot relaxed (safe); engage on command
    static int   sweepIdx = 0, sweepTick = 0;

    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("x") ||
            line.equalsIgnoreCase("relax")) {
            relaxed = true; sweep = false; value = 0.0f; vibAmp = 0.0f; vibFreq = 0;
            Serial.println("RELAXED");
        }
        else if (line.equalsIgnoreCase("rampup")) {
            force_ramp_up(motor);
            // force_ramp_up leaves the motor relaxed; keep the CLI state in sync
            // so it doesn't immediately re-engage the previous base command.
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("sweep")) {
            relaxed = false;
            sweep = !sweep;
            sweepIdx = 0;
            sweepTick = 0;
            if (sweep && vibAmp <= 0.0f) vibAmp = 0.2f;   // need some amplitude to feel it
            if (!sweep) applyVibrate(0, 0.0f, motor);     // stop the buzz when turning off
            Serial.printf("sweep %s (amp %.2f)\n", sweep ? "ON" : "OFF", vibAmp);
        }
        else if (line.equalsIgnoreCase("tm")) {
            trigger_mapping(motor, gripper);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("tml")) {
            trigger_mapping_log(motor, gripper);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("gpos")) {
            gripper_position_mode(gripper);
        }
        else if (line.equalsIgnoreCase("hu")) {
            haptic_update_mode(motor, gripper);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("cal")) {
            Serial.println("Calibrating trigger limits — do not touch the trigger...");
            bool ok = motor->calibrate_trigger();
            if (ok)
                Serial.println("Calibration done.");
            else
                Serial.println("Calibration FAILED (timeout — check haptic link).");
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("sl")) {
            if (motor->is_calibrated()) {
                bool ok = motor->send_limits();
                if (ok)
                    Serial.printf("Limits sent to STM32 (open=%.2f).\n",
                                  motor->get_limit_open());
                else
                    Serial.println("send_limits FAILED.");
            } else {
                Serial.println("Not calibrated — run 'cal' first.");
            }
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.length() > 0) {
            char ch = line.charAt(0);
            String arg = line.substring(1);
            arg.trim();

            if (ch == 's' || ch == 'S') {
                relaxed = false; mode = MODE_SPRING; value = arg.toFloat();
                Serial.printf("base = spring %.3f\n", value);
            } else if (ch == 'c' || ch == 'C') {
                relaxed = false; mode = MODE_CONSTANT; value = -arg.toFloat();
                Serial.printf("base = constant %.3f\n", value);
            } else if (ch == 'n' || ch == 'N') {
                float N = arg.toFloat();
                relaxed = false; mode = MODE_CONSTANT; value = -(N + 0.122f) / 3.18f;
                Serial.printf("base = %.3f N (unitless %+.3f)\n", N, value);
            } else if (ch == 'f' || ch == 'F') {
                relaxed = false; sweep = false; vibFreq = arg.toInt();
                applyVibrate(vibFreq, vibAmp, motor);
            } else if (ch == 'a' || ch == 'A') {
                relaxed = false; vibAmp = arg.toFloat();
                applyVibrate(vibFreq, vibAmp, motor);
            } else if (ch == 'v' || ch == 'V') {
                relaxed = false; sweep = false;
                int sp = arg.indexOf(' ');
                vibFreq = arg.toInt();
                vibAmp  = (sp >= 0) ? arg.substring(sp + 1).toFloat() : vibAmp;
                applyVibrate(vibFreq, vibAmp, motor);
            } else if ((ch == 'm' || ch == 'M') && gripper) {
                // m <left_deg> <right_deg> [speed_rpm]
                int sp1 = arg.indexOf(' ');
                int sp2 = (sp1 >= 0) ? arg.indexOf(' ', sp1 + 1) : -1;
                float l_deg = arg.toFloat();
                float r_deg = (sp1 >= 0) ? arg.substring(sp1 + 1).toFloat() : l_deg;
                float spd   = (sp2 >= 0) ? arg.substring(sp2 + 1).toFloat() : 0.0f;
                Dyn_Response_Packet resp;
                bool ok = gripper->move_servo(l_deg, r_deg, spd, spd, &resp);
                float lmA, rmA;
                memcpy(&lmA, &resp.left_motor_current,  sizeof(lmA));
                memcpy(&rmA, &resp.right_motor_current, sizeof(rmA));
                Serial.printf("move L=%.1f R=%.1f spd=%.0f -> %s (L=%.0fmA R=%.0fmA)\n",
                              l_deg, r_deg, spd,
                              ok ? (resp.response == DYN_ACK ? "ACK" : "FAIL") : "no resp",
                              lmA, rmA);
            } else if (line.equalsIgnoreCase("gstop") && gripper) {
                Dyn_Response_Packet resp;
                bool ok = gripper->stop(&resp);
                Serial.println(ok ? "gripper STOP ACK" : "gripper STOP no resp");
            } else if (line.equalsIgnoreCase("gping") && gripper) {
                bool ok = gripper->ping(300);
                Serial.println(ok ? "gripper PONG" : "gripper no PONG");
            } else {
                Serial.println("unknown command");
            }
        }
    }

    // Auto-sweep: advance through SWEEP_FREQS, holding the current amplitude.
    if (!relaxed && sweep && vibAmp > 0.0f) {
        if (sweepTick == 0) {
            vibFreq = SWEEP_FREQS[sweepIdx];
            applyVibrate(vibFreq, vibAmp, motor);
        }
        if (++sweepTick >= SWEEP_HOLD) {
            sweepTick = 0;
            sweepIdx = (sweepIdx + 1) % SWEEP_COUNT;
        }
    }

    // Each cycle: re-send relax (when stopped) or the active base command. This
    // also serves as the heartbeat that keeps the STM's fail-safe from firing.
    Response_Packet resp;
    bool ok;
    if (relaxed) {
        ok = motor->relax(&resp);
    } else {
        ok = (mode == MODE_SPRING) ? motor->set_spring_force(value, &resp)
                                   : motor->set_constant_force(value, &resp);
    }

    if (ok) {
        float pos_deg, torque, motor_t, user_t;
        memcpy(&pos_deg, &resp.encoder_value,   sizeof(pos_deg));
        memcpy(&torque,  &resp.motor_value,     sizeof(torque));
        memcpy(&motor_t, &resp.measured_torque, sizeof(motor_t));
        memcpy(&user_t,  &resp.user_torque,     sizeof(user_t));
        unsigned long raw_enc = (unsigned long)resp.raw_encoder;   // raw counter, plain integer
        Serial.printf("[%s | %s %.2f | vib %dHz %.2f%s] set=%+.3f pos=%+.2f enc=%lu cmd=%+.3f motorT=%+.3f userT=%+.3f switch40=%s switch41=%s led42=%s\n",
                      relaxed ? "RELAX" : "ON",
                      mode == MODE_SPRING ? "spr" : "con", value,
                      vibFreq, vibAmp, sweep ? " SWEEP" : "",
                      relaxed ? 0.0f : value,
                      pos_deg, raw_enc, torque, motor_t, user_t, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
    } else {
        Serial.printf("no ACK (rejected or timed out) switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
    }

    delay(200);
}
