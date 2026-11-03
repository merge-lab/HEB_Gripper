#include "CLI_Control.h"
#include "tactile_sensors.h"
#include "slip_detector.h"
#include "trigger_control.h"
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

// ---- Slip detection: commands and status output --------------------------------
// Accepted in idle, tac and tmt:  slip_thr <v>   slip_gate <v>   slip?
// Every change is confirmed with a machine-readable "SLIPCFG,<thr>,<gate>".

static void print_slip_config()
{
    const SlipConfig c = slip_config();
    Serial.printf("SLIPCFG,%.3f,%.3f\n", c.threshold, c.contact_gate);
}

static bool handle_slip_command(String line)
{
    line.trim();
    if (line.equalsIgnoreCase("slip?")) {
        print_slip_config();
        const SlipStats st = slip_stats();
        Serial.printf("[SLIP] %.0f updates/s, %.0f us per update (max %.0f), core-0 load %.1f%%, "
                      "%lu samples dropped\n",
                      st.updates_per_s, st.update_us_avg, st.update_us_max,
                      st.core0_load_pct, (unsigned long)st.dropped);
        return true;
    }
    const bool is_thr  = line.startsWith("slip_thr ");
    const bool is_gate = line.startsWith("slip_gate ");
    if (!is_thr && !is_gate)
        return false;

    const char* arg = line.c_str() + line.indexOf(' ') + 1;
    char* end = nullptr;
    const float v = strtof(arg, &end);
    if (end == arg || !(v >= 0.0f) || v > 1000.0f) {
        Serial.println("[SLIP] usage: slip_thr <0..1000>  |  slip_gate <0..1000>");
        return true;
    }
    SlipConfig c = slip_config();
    (is_thr ? c.threshold : c.contact_gate) = v;
    slip_set_config(c);
    print_slip_config();
    return true;
}

// One "SLIP,<avgL>,<avgR>,<slip>" line; slip = 1 if any slip result occurred
// since the previous line (so short events are never missed between lines).
static void print_slip_line()
{
    SlipStatus st;
    slip_take(&st);
    Serial.printf("SLIP,%.3f,%.3f,%d\n", st.result.avgL, st.result.avgR, st.slip_since_last ? 1 : 0);
}

// Tactile -> haptic force (tmt mode). Each finger's three tared channels are
// summed (clamped to 0..TAC_SUM_MAX) and the larger side, s, sets the
// constant force c (same units and sign convention as the CLI 'c' command)
// on a logarithmic curve, for finer resolution at light contact:
//
//     c(s) = TAC_FORCE_MIN + TAC_LOG_K * ln(1 + s / TAC_LOG_S0),
//            clamped to TAC_FORCE_MIN..TAC_FORCE_MAX
//
// K and S0 are fitted so that s = 0.5 -> 2 N and s = 1 -> 3 N (N = 3.18 c):
//     s    0     0.25  0.5   1.0   1.5   >=1.68
//     c    0.25  0.45  0.63  0.94  1.21  1.30
//     N    0.80  1.43  2.00  3.00  3.85  4.13
static const float TAC_SUM_MAX     = 3.0f;
static const float TAC_FORCE_MIN   = 0.25f;
static const float TAC_FORCE_MAX   = 1.30f;
static const float TAC_LOG_K       = 1.85f;
static const float TAC_LOG_S0      = 2.2f;
static const float TRIGGER_N_PER_C = 3.18f;   // c x 3.18 = Newtons at the trigger

static float tactile_to_force(float s)
{
    if (!(s > 0.0f)) s = 0.0f;                    // also catches NaN
    const float c = TAC_FORCE_MIN + TAC_LOG_K * logf(1.0f + s / TAC_LOG_S0);
    return c > TAC_FORCE_MAX ? TAC_FORCE_MAX : c;
}

static float clamp_sum(float s)
{
    if (!(s > 0.0f)) return 0.0f;      // also catches NaN
    return s > TAC_SUM_MAX ? TAC_SUM_MAX : s;
}

// ---- Trigger encoder -> gripper opening -------------------------------------
// t = 1: trigger released (encoder at the HIGH end, where the haptic force
//        pushes it) -> gripper open
// t = 0: trigger fully pulled (encoder lowered) -> gripper closed
//
// Calibrated ('cal'): uses the measured limits.
// Uncalibrated: the released end is taken as the HIGHEST position seen so
// far, with a fixed 47 deg of travel below it. Where the trigger happens to
// sit while relaxed at startup varies, but once the force is on it is pushed
// to the top, so tracking the maximum converges on the real released end.
struct TriggerMap
{
    static constexpr float FALLBACK_RANGE_DEG = 47.0f;
    bool  track_top = false;
    float bottom = 0.0f, top = 0.0f;

    float t(float pos)
    {
        if (track_top && pos > top) {
            top    = pos;
            bottom = top - FALLBACK_RANGE_DEG;
        }
        float span = top - bottom;
        if (!(span > 0.0f)) return 1.0f;
        float x = (pos - bottom) / span;
        if (!(x > 0.0f)) return 0.0f;     // also NaN -> closed
        return x > 1.0f ? 1.0f : x;
    }
};

static TriggerMap make_trigger_map(HapticMotorController* motor, const char* label)
{
    TriggerMap m;
    if (motor->is_calibrated()) {
        m.bottom = motor->get_limit_close();
        m.top    = motor->get_limit_open();
        Serial.printf("%s ON (calibrated): released=%.2f pulled=%.2f. Type 'e' + Enter to exit.\n",
                      label, m.top, m.bottom);
    } else {
        Response_Packet r;
        while (!motor->relax(&r))
            delay(200);
        float pos;
        memcpy(&pos, &r.encoder_value, sizeof(pos));
        m.track_top = true;
        m.top    = pos;
        m.bottom = pos - TriggerMap::FALLBACK_RANGE_DEG;
        Serial.printf("%s ON (uncalibrated): released end = highest position seen (now %.2f), "
                      "47 deg travel. Run 'cal' for exact limits. Type 'e' + Enter to exit.\n",
                      label, pos);
    }
    return m;
}

void trigger_mapping(HapticMotorController* motor, DynamixelController* gripper, bool tactile_force)
{
    float force = -0.75f;
    String input;
    input.reserve(80);
    bool inputOverflow = false;
    unsigned long lastInputMs = 0;

    TriggerMap trig = make_trigger_map(motor, tactile_force ? "Trigger mapping + tactile force" : "Trigger mapping");

    // tmt: the initial tare waits until the haptic motor and servos have been
    // running for a moment. Their current draw shifts the sensors' supply /
    // ground slightly, so a tare taken while everything is still off leaves
    // a constant offset once they engage. c is held at its minimum until then.
    const unsigned long TMT_TARE_DELAY_MS = 500;
    unsigned long tmt_start_ms = millis();
    bool tmt_tared = false;

    // Loop timing. The loop runs on a fixed LOOP_PERIOD_US schedule instead
    // of a fixed delay; the haptic UART exchange (~4.5 ms at 115200 baud)
    // is the floor. Printing is rate-limited so output doesn't scale with
    // the loop rate, and timing stats are reported once per second.
    // Smooth start, friction floor and return-speed limit (trigger_control.h).
    // The force is decided before each exchange, so they use the previous
    // reply's position and speed (one loop, ~5 ms, old).
    const FrictionMap& fmap = trigger_friction_map();
    const bool use_floor = tactile_force && fmap.valid;
    const unsigned long soft_start_ms = millis();
    float last_pos = NAN, trig_vel = 0.0f;          // trig_vel > 0 = toward release
    unsigned long last_pos_us = 0;

    const unsigned long LOOP_PERIOD_US  = 5000;                       // 200 Hz target
    const unsigned long PRINT_PERIOD_US = tactile_force ? 10000 : 50000; // TMT 100 Hz, TM 20 Hz
    unsigned long next_loop_us  = micros();
    unsigned long last_print_us = 0;
    unsigned long stats_start_ms = millis();
    unsigned long stats_loops = 0, stats_haptic_us = 0, stats_haptic_max_us = 0;
    unsigned long stats_bad0 = gripper ? gripper->bad_reply_count() : 0;
    if (tactile_force)
    {
        Serial.println("[TMT] auto-tare in 0.5 s - keep the fingers unloaded.");
        Serial.printf("[TMT] c = %.2f + %.2f ln(1 + s/%.1f), clamped %.2f..%.2f (%.2f..%.2f N); "
                      "s = max(T1+T2+T3, T4+T5+T6): s 0.5 -> %.2f N, s 1 -> %.2f N.\n",
                      TAC_FORCE_MIN, TAC_LOG_K, TAC_LOG_S0, TAC_FORCE_MIN, TAC_FORCE_MAX,
                      TAC_FORCE_MIN * TRIGGER_N_PER_C, TAC_FORCE_MAX * TRIGGER_N_PER_C,
                      tactile_to_force(0.5f) * TRIGGER_N_PER_C, tactile_to_force(1.0f) * TRIGGER_N_PER_C);
        if (trigger_friction_map().valid)
            Serial.printf("[TMT] friction floor ON: return at >= %.0f deg/s everywhere (map: %.1f deg travel, "
                          "floor <= %.2f).\n", TRIGGER_RETURN_DEG_S, trigger_friction_map().travel,
                          FrictionMap::FLOOR_MAX);
        else
            Serial.println("[TMT] friction floor OFF: no friction map - run 'fid' once (hands off the trigger).");
        Serial.println("TMT commands: tare, v <hz> <amp>, slip_thr <v>, slip_gate <v>, slip?, e to exit.");
        Serial.println("TMT,pos,t,id0,id1,sumL,sumR,c,T1,T2,T3,T4,T5,T6,stm_cmd,iq");   // column header
    }
    else
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
            if (line.equalsIgnoreCase("tare"))
            {
                Serial.println(tactile_tare() ? "[TM] tactile tared." : "[TM] tare failed (no samples yet).");
                continue;
            }
            if (handle_slip_command(line))
                continue;

            String arg = line.substring(1);
            char extra;
            if ((line.charAt(0) == 'c' || line.charAt(0) == 'C') && tactile_force)
            {
                Serial.println("[TMT] force follows the tactile sensors in this mode; use 'tm' for manual c.");
            }
            else if (line.charAt(0) == 'c' || line.charAt(0) == 'C')
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
                Serial.println(tactile_force ? "[TMT] commands: tare, v <hz> <amp>, e"
                                             : "[TM] commands: c <force>, v <hz> <amp>, tare, e");
        }

        // tmt: force from the newest tactile sample (shared buffer, no ADC wait).
        float tac[TACTILE_COUNT];
        float sumL = 0.0f, sumR = 0.0f, c_mag = 0.0f;
        if (tactile_force && !tmt_tared && millis() - tmt_start_ms >= TMT_TARE_DELAY_MS)
        {
            tmt_tared = tactile_tare();
            if (tmt_tared) Serial.println("[TMT] tactile tared (motor + servos active).");
        }
        if (tactile_force)
        {
            // Mean of the last ~5 ms of samples: less noise than one sample,
            // still far faster than the loop.
            tactile_average(tac, tactile_nominal_hz() / 200, true);
            sumL = clamp_sum(tac[0] + tac[1] + tac[2]);   // T1..T3: left finger (ID 1)
            sumR = clamp_sum(tac[3] + tac[4] + tac[5]);   // T4..T6: right finger (ID 0)
            float s = sumL > sumR ? sumL : sumR;
            c_mag = tactile_to_force(s);
            if (!tmt_tared) c_mag = TAC_FORCE_MIN;   // stale baseline: don't trust it yet

            if (use_floor && !isnan(last_pos)) {
                // Never below what it takes to return the trigger from here.
                const float floor = fmap.floor_at(trig.top - last_pos,
                                                  TRIGGER_RETURN_DEG_S, TRIGGER_FLOOR_MARGIN);
                if (c_mag < floor) c_mag = floor;
                // Snapping back too fast: ease off the push (never below 0).
                if (trig_vel > TRIGGER_MAX_RETURN_DEG_S) {
                    c_mag -= 0.5f * fmap.b_open * (trig_vel - TRIGGER_MAX_RETURN_DEG_S);
                    if (c_mag < 0.0f) c_mag = 0.0f;
                }
            }
            force = -c_mag;   // same sign convention as 'c'
        }

        // Keep the selected base force and heartbeat active; the STM32 retains
        // the vibration overlay and handles limit gating and ramp at 1 kHz.
        // Smooth start (tm and tmt): fade the force in when the mode begins.
        const float force_cmd = force * trigger_soft_start_gain(millis() - soft_start_ms);

        Response_Packet resp;
        unsigned long haptic_t0 = micros();
        bool ok = motor->set_constant_force(force_cmd, &resp);
        unsigned long haptic_us = micros() - haptic_t0;

        if (!ok)
        {
            Serial.printf("[TM] no ACK from haptic motor switch40=%s switch41=%s led42=%s\n", switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
            delay(200);
            continue;
        }

        float pos_deg;
        memcpy(&pos_deg, &resp.encoder_value, sizeof(pos_deg));
        {
            const unsigned long now_pos_us = micros();
            if (!isnan(last_pos) && now_pos_us != last_pos_us)
                trig_vel += 0.3f * ((pos_deg - last_pos) / ((now_pos_us - last_pos_us) * 1e-6f) - trig_vel);
            last_pos = pos_deg;
            last_pos_us = now_pos_us;
        }

        float t = trig.t(pos_deg);

        // t = 1 (trigger at rest) -> gripper open, t = 0 (fully pulled) -> closed.
        float id0_deg, id1_deg;
        DynamixelController::opening_to_goals(t, &id0_deg, &id1_deg);
        if (gripper)
            gripper->move_servo_fast(id0_deg, id1_deg);

        unsigned long now_us = micros();
        bool print_now = (now_us - last_print_us) >= PRINT_PERIOD_US;
        if (print_now) last_print_us = now_us;

        if (!print_now)
            ;   // rate-limited
        else if (tactile_force)
            // Machine-readable line for heb_console (see header line above).
        {
            // What the STM32 actually applies (after its limit ramp + LPF) and
            // the measured motor current, from the same reply. Sign-flipped so
            // they read in the same positive sense as c.
            float stm_cmd, iq;
            memcpy(&stm_cmd, &resp.motor_value,     sizeof(stm_cmd));
            memcpy(&iq,      &resp.measured_torque, sizeof(iq));
            Serial.printf("TMT,%.2f,%.3f,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                          pos_deg, t, id0_deg, id1_deg, sumL, sumR, -force_cmd,
                          tac[0], tac[1], tac[2], tac[3], tac[4], tac[5],
                          -stm_cmd, -iq);
            print_slip_line();
        }
        else
            Serial.printf("[TM] pos=%+.2f t=%.2f ID0=%.1f ID1=%.1f switch40=%s switch41=%s led42=%s\n",
                          pos_deg, t, id0_deg, id1_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());

        // Once-per-second timing report.
        stats_loops++;
        stats_haptic_us += haptic_us;
        if (haptic_us > stats_haptic_max_us) stats_haptic_max_us = haptic_us;
        unsigned long stats_ms = millis() - stats_start_ms;
        if (stats_ms >= 1000)
        {
            unsigned long fb_age = 0;
            bool have_fb = gripper && gripper->latest_feedback(nullptr, nullptr, &fb_age);
            unsigned long bad_now = gripper ? gripper->bad_reply_count() : 0;
            const SlipStats sst = slip_stats();
            Serial.printf("[%s] loop %.0f Hz, haptic exchange avg %.2f ms / max %.2f ms, "
                          "gripper feedback latency %lu ms, age %ld ms, mega bad replies %lu, "
                          "slip %.0f/s %.0f us core0 %.0f%%\n",
                          tactile_force ? "TMT" : "TM",
                          stats_loops * 1000.0f / stats_ms,
                          stats_haptic_us / 1000.0f / stats_loops,
                          stats_haptic_max_us / 1000.0f,
                          gripper ? gripper->feedback_latency_ms() : 0UL,
                          have_fb ? (long)fb_age : -1L,
                          bad_now - stats_bad0,
                          sst.updates_per_s, sst.update_us_avg, sst.core0_load_pct);
            stats_bad0 = bad_now;
            stats_start_ms = millis();
            stats_loops = stats_haptic_us = stats_haptic_max_us = 0;
        }

        // Fixed-rate schedule: wait out the rest of the period (if any).
        // If a cycle overran by more than a full period, resync rather than
        // bursting to catch up.
        next_loop_us += LOOP_PERIOD_US;
        long wait_us = (long)(next_loop_us - micros());
        if (wait_us > 0)
        {
            // Sleep whole ms (yields to other tasks), spin the remainder.
            if (wait_us >= 1000) delay(wait_us / 1000);
            long rest = (long)(next_loop_us - micros());
            if (rest > 0) delayMicroseconds(rest);
        }
        else if (wait_us < -(long)LOOP_PERIOD_US)
            next_loop_us = micros();
    }
}

// Same as trigger_mapping(), but reads back and logs actual servo position
// each cycle (slower — move_servo() blocks for the Dynamixel's response).
// Used by response_plot.py to compare expected vs. actual position.
void trigger_mapping_log(HapticMotorController* motor, DynamixelController* gripper)
{
    TriggerMap trig = make_trigger_map(motor, "Trigger mapping (logged)");

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

        float t = trig.t(pos_deg);

        // t = 1 (trigger at rest) -> gripper open, t = 0 (fully pulled) -> closed.
        float id0_deg, id1_deg;
        DynamixelController::opening_to_goals(t, &id0_deg, &id1_deg);
        float actual_id0_deg = 0.0f, actual_id1_deg = 0.0f;
        if (gripper)
        {
            Dyn_Response_Packet gripResp;
            // Time the gripper command/response round trip, excluding the
            // STM32 exchange, terminal output, and the loop's 15 ms delay.
            uint32_t latency_start_us = micros();
            bool grip_ok = gripper->move_servo_log(id0_deg, id1_deg, &gripResp);
            uint32_t latency_us = (uint32_t)(micros() - latency_start_us);
            if (grip_ok)
            {
                memcpy(&actual_id0_deg,  &gripResp.left_motor_position,  sizeof(actual_id0_deg));
                memcpy(&actual_id1_deg, &gripResp.right_motor_position, sizeof(actual_id1_deg));
            }
            // Serial.printf("latency = %.3f ms%s\n", latency_us / 1000.0f,
            //               grip_ok ? "" : " (timeout or invalid response)");
        }

        // Keep the L=/R= format: response_plot.py parses it.
        // L = ID 0 (physical right), R = ID 1 (physical left).
        Serial.printf("[TM] pos=%+.2f t=%.2f L=%.1f(act=%.1f) R=%.1f(act=%.1f) switch40=%s switch41=%s led42=%s\n",
                      pos_deg, t, id0_deg, actual_id0_deg, id1_deg, actual_id1_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());

        delay(15);
    }
}

void haptic_update_mode(HapticMotorController* motor, DynamixelController* gripper)
{
    float norm_force = 0.0f;
    float norm_slip  = 0.0f;
    bool  dxl_active = false;
    TriggerMap trig;

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
                    trig = make_trigger_map(motor, "HU gripper mapping");
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

            float t = trig.t(pos_deg);

            // t = 1 (trigger at rest) -> gripper open, t = 0 (fully pulled) -> closed.

            float id0_deg, id1_deg;

            DynamixelController::opening_to_goals(t, &id0_deg, &id1_deg);
            gripper->move_servo_fast(id0_deg, id1_deg);

            Serial.printf("[HU] f=%.2f s=%.2f pos=%+.2f t=%.2f ID0=%.1f ID1=%.1f switch40=%s switch41=%s led42=%s\n",
                          norm_force, norm_slip, pos_deg, t, id0_deg, id1_deg, switchState(SWITCH_40_PIN), switchState(SWITCH_41_PIN), ledState());
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
            Serial.printf("[GP] ID0=%7.2f [%7.2f..%7.2f]   ID1=%7.2f [%7.2f..%7.2f]\n",
                          l, l_min, l_max, r, r_min, r_max);
        }
        else if (++fails % 10 == 1)
        {
            Serial.println("[GP] no PONG from gripper board");
        }

        delay(100);
    }

    Serial.println("Position mode OFF. Measured range:");
    Serial.printf("  ID0 (right): %.2f .. %.2f deg (span %.2f)\n", l_min, l_max, l_max - l_min);
    Serial.printf("  ID1 (left) : %.2f .. %.2f deg (span %.2f)\n", r_min, r_max, r_max - r_min);
}

void gripper_move_watch(DynamixelController* gripper, float id0_deg, float id1_deg, float rpm,
                        bool verbose)
{
    if (!gripper) { Serial.println("no gripper link"); return; }

    // rpm <= 0 means "as fast as the shaper allows" (GRIPPER_MAX_DEG_PER_S).
    const float max_rpm = GRIPPER_MAX_DEG_PER_S / 6.0f;
    if (!(rpm > 0.0f) || rpm > max_rpm) rpm = max_rpm;

    if (!DynamixelController::command_in_range(id0_deg, id1_deg)) {
        Serial.printf("[MW] REJECTED: goal out of range. Allowed ID0 %.0f..%.0f, ID1 %.0f..%.0f\n",
                      DXL_ID0_OPEN_DEG, DXL_ID0_CLOSED_DEG, DXL_ID1_CLOSED_DEG, DXL_ID1_OPEN_DEG);
        gripper->stop(nullptr);
        return;
    }

    float p0 = NAN, p1 = NAN;
    if (!gripper->read_position(&p0, &p1, 200)) {
        Serial.println("[MW] no reply from gripper board - check link / Mega power. Aborting.");
        return;
    }
    const float start0 = p0, start1 = p1;
    if (verbose)
        Serial.printf("[MW] start  ID0=%7.2f ID1=%7.2f -> goal ID0=%7.2f ID1=%7.2f at %.1f rpm "
                      "('e' + Enter = stop, torque off)\n", p0, p1, id0_deg, id1_deg, rpm);

    // Stream the goal through the driver's shaper at the requested speed. The
    // servos' own Profile Velocity is never touched, so nothing slow persists
    // afterwards; pacing, feedback and the range guard are the same as tm/tmt.
    gripper->set_stream_speed_limit(rpm * 6.0f);

    const unsigned long start_ms = millis();
    unsigned long last_print = 0, still_since = 0;
    float last0 = p0, last1 = p1;
    bool reached = false, aborted = false;

    while (millis() - start_ms < 20000UL)
    {
        if (Serial.available()) {
            String l = Serial.readStringUntil('\n');
            l.trim();
            if (l.equalsIgnoreCase("e")) {
                gripper->stop(nullptr);
                Serial.println("[MW] stopped by user (torque off)");
                aborted = true;
                break;
            }
        }

        gripper->move_servo_fast(id0_deg, id1_deg);
        if (gripper->last_move_blocked()) {      // guard relaxed the servos and said why
            Serial.println("[MW] aborted by the range guard.");
            aborted = true;
            break;
        }

        unsigned long age;
        if (gripper->latest_feedback(&p0, &p1, &age) && millis() - last_print >= 100) {
            last_print = millis();
            if (verbose)
                Serial.printf("[MW] actual ID0=%7.2f (err %+6.2f)  ID1=%7.2f (err %+6.2f)\n",
                              p0, p0 - id0_deg, p1, p1 - id1_deg);
            reached = fabsf(p0 - id0_deg) < 1.0f && fabsf(p1 - id1_deg) < 1.0f;
            if (reached) break;
            // Stalled: no movement for 1.5 s (after the first second).
            const bool still = fabsf(p0 - last0) < 0.2f && fabsf(p1 - last1) < 0.2f;
            if (!still || millis() - start_ms < 1000) still_since = millis();
            else if (millis() - still_since > 1500) break;
            last0 = p0; last1 = p1;
        }
        delay(5);
    }
    gripper->set_stream_speed_limit(GRIPPER_MAX_DEG_PER_S);
    if (aborted) return;

    // Per-servo verdict so a single misbehaving servo is obvious.
    const float goals[2]  = { id0_deg, id1_deg };
    const float starts[2] = { start0, start1 };
    const float ends[2]   = { p0, p1 };
    for (int i = 0; i < 2; i++) {
        const float err   = ends[i] - goals[i];
        const float moved = fabsf(ends[i] - starts[i]);
        const char* verdict =
            fabsf(err) < 1.0f                                   ? "reached goal" :
            (moved < 0.5f && fabsf(goals[i] - starts[i]) >= 1.0f) ? "DID NOT MOVE (servo in error, or torque off)" :
                                                                   "STOPPED SHORT (obstruction?)";
        Serial.printf("[MW] ID%d: start %.2f -> end %.2f (goal %.2f, err %+.2f): %s\n",
                      i, starts[i], ends[i], goals[i], err, verdict);
    }
    Serial.println("[MW] done. Torque turns off ~0.5 s after this.");
}

void tactile_mode(HapticMotorController* motor, DynamixelController* gripper, int out_hz)
{
    // Tactile-only: relax the haptic motor and the gripper so nothing moves
    // while we stream sensor data.
    motor->relax(nullptr);
    if (gripper) gripper->stop(nullptr);

    // Output rate: every sample (out_hz >= sample rate) or a block average of
    // `decim` samples per line (a clean low-pass decimation, not a pick).
    const int fs = tactile_nominal_hz();
    if (out_hz <= 0) out_hz = 100;
    int decim = (fs + out_hz / 2) / out_hz;
    if (decim < 1) decim = 1;
    const float actual_hz = (float)fs / decim;

    Serial.printf("Tactile mode ON: T1..T6 (tared) at %.0f Hz (%s; %s).\n",
                  actual_hz, tactile_mode_name(),
                  decim == 1 ? "every sample" : "block average per line");
    Serial.println("Haptic relaxed, gripper torque off. 'tare' re-zeroes, 'e' + Enter exits.");
    print_slip_config();

    const int LINE_BYTES = 64;           // one "T1:...,T6:..." line, rounded up
    unsigned long last_slip_ms = millis();
    char  cmd[32];
    int   cmd_len = 0;
    unsigned long lines = 0, usb_skipped = 0;
    uint32_t dropped = 0;
    uint32_t cursor = tactile_sample_count();   // start with fresh samples
    float buf[64][TACTILE_COUNT];
    float acc[TACTILE_COUNT] = { 0 };
    int   acc_n = 0;

    while (true)
    {
        // Non-blocking line input: readStringUntil() could stall the stream.
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                cmd[cmd_len] = '\0';
                if (cmd_len == 4 && strncasecmp(cmd, "tare", 4) == 0) {
                    Serial.println(tactile_tare() ? "Tactile tared." : "Tare failed (no samples yet).");
                } else if (cmd_len > 0) {
                    handle_slip_command(String(cmd));
                }
                if (cmd_len == 1 && (cmd[0] == 'e' || cmd[0] == 'E')) {
                    Serial.printf("Tactile mode OFF. %lu lines, %lu samples dropped (reader behind), "
                                  "%lu lines skipped (USB busy), ADC rate %.0f Hz.\n",
                                  lines, (unsigned long)dropped, usb_skipped, tactile_measured_hz());
                    return;
                }
                cmd_len = 0;
            } else if (cmd_len < (int)sizeof(cmd) - 1) {
                cmd[cmd_len++] = c;
            }
        }

        int n = tactile_read_new(&cursor, buf, 64, true, &dropped);
        for (int k = 0; k < n; k++) {
            for (int i = 0; i < TACTILE_COUNT; i++) acc[i] += buf[k][i];
            if (++acc_n < decim) continue;

            float m[TACTILE_COUNT];
            for (int i = 0; i < TACTILE_COUNT; i++) { m[i] = acc[i] / acc_n; acc[i] = 0.0f; }
            acc_n = 0;
            // Never block on USB: if the laptop isn't keeping up (or the port
            // is closed), skip the line and count it instead of stalling.
            if (Serial.availableForWrite() < LINE_BYTES) { usb_skipped++; continue; }
            Serial.printf("T1:%.3f,T2:%.3f,T3:%.3f,T4:%.3f,T5:%.3f,T6:%.3f\n",
                          m[0], m[1], m[2], m[3], m[4], m[5]);
            lines++;
        }
        // Slip status at 100 Hz, whatever the tactile line rate.
        if (millis() - last_slip_ms >= 10 && Serial.availableForWrite() >= LINE_BYTES) {
            last_slip_ms = millis();
            print_slip_line();
        }

        delay(1);                        // always yield; ~2 new samples per ms at 2 kHz
    }
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
    Serial.println("Gripper: m <id0> <id1> [rpm]   mw <id0> <id1> [rpm]   gstop   gping   gpos");
    Serial.println("Modes:   tm (trigger map)   tml (trigger map + log)   tac [hz] (tactile stream, 100 default, 2000 max)   tmt (trigger map + tactile force)   tare   hu (haptic update)   cal (calibrate)   sl (send limits)");
    Serial.println("Slip:    slip_thr <v>   slip_gate <v>   slip?   (FORTE detector, on-board at 2 kHz)");
    Serial.println("Trigger: fid (measure friction, hands off)   fid? (print map)   fid_clear");
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
        else if (line.equalsIgnoreCase("tmt")) {
            trigger_mapping(motor, gripper, true);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("tare")) {
            Serial.println(tactile_tare() ? "Tactile tared." : "Tare failed (no samples yet).");
        }
        else if (handle_slip_command(line)) {
            // slip_thr / slip_gate / slip? handled
        }
        else if (line.equalsIgnoreCase("fid")) {
            trigger_friction_id(motor, gripper);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("fid?")) {
            trigger_friction_print();
        }
        else if (line.equalsIgnoreCase("fid_clear")) {
            trigger_friction_clear();
        }
        else if (line.equalsIgnoreCase("tac") || line.startsWith("tac ") || line.startsWith("TAC ")) {
            // tac [hz]: 100 Hz default; 'tac 2000' streams every sample.
            int hz = line.length() > 4 ? line.substring(4).toInt() : 100;
            tactile_mode(motor, gripper, hz);
            relaxed = true; sweep = false; value = 0.0f;
        }
        else if (line.equalsIgnoreCase("gpos")) {
            gripper_position_mode(gripper);
        }
        // Must come before the generic single-letter 'm' handler below.
        else if ((line.startsWith("mw") || line.startsWith("MW") ||
                  line.startsWith("mW") || line.startsWith("Mw")) && gripper) {
            // mw <id0_deg> <id1_deg> [rpm]
            String a = line.substring(2);
            a.trim();
            int sp1 = a.indexOf(' ');
            int sp2 = (sp1 >= 0) ? a.indexOf(' ', sp1 + 1) : -1;
            if (sp1 < 0) {
                Serial.println("usage: mw <id0_deg> <id1_deg> [rpm]   (default 5 rpm)");
            } else {
                float id0 = a.toFloat();
                float id1 = a.substring(sp1 + 1).toFloat();
                float rpm = (sp2 >= 0) ? a.substring(sp2 + 1).toFloat() : 5.0f;
                gripper_move_watch(gripper, id0, id1, rpm, /*verbose=*/true);
            }
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
                // m <id0_deg> <id1_deg> [rpm]: same as mw, only the result is printed.
                int sp1 = arg.indexOf(' ');
                int sp2 = (sp1 >= 0) ? arg.indexOf(' ', sp1 + 1) : -1;
                if (sp1 < 0) {
                    Serial.println("usage: m <id0_deg> <id1_deg> [rpm]");
                } else {
                    float id0 = arg.toFloat();
                    float id1 = arg.substring(sp1 + 1).toFloat();
                    float rpm = (sp2 >= 0) ? arg.substring(sp2 + 1).toFloat() : 0.0f;
                    gripper_move_watch(gripper, id0, id1, rpm, /*verbose=*/false);
                }
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
