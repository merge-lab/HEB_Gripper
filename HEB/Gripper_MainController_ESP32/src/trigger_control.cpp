#include "trigger_control.h"

#include <Preferences.h>

namespace {

FrictionMap g_map;

// ---- Flash storage (NVS) ------------------------------------------------------
constexpr const char* NVS_NAMESPACE = "trigger";
constexpr const char* NVS_KEY       = "fricmap";
constexpr uint32_t    MAP_MAGIC     = 0x46524331;     // "FRC1"

struct StoredMap
{
    uint32_t    magic;
    uint32_t    size;          // sizeof(FrictionMap) when written
    FrictionMap map;
};

bool load_map()
{
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/true)) return false;
    StoredMap st;
    const size_t n = prefs.getBytesLength(NVS_KEY);
    const bool ok = n == sizeof(st) && prefs.getBytes(NVS_KEY, &st, sizeof(st)) == sizeof(st) &&
                    st.magic == MAP_MAGIC && st.size == sizeof(FrictionMap) && st.map.valid &&
                    st.map.n > 0 && st.map.n <= FrictionMap::MAX_BINS;
    prefs.end();
    if (ok) g_map = st.map;
    return ok;
}

bool save_map(const FrictionMap& m)
{
    Preferences prefs;
    if (!prefs.begin(NVS_NAMESPACE, /*readOnly=*/false)) return false;
    StoredMap st = { MAP_MAGIC, (uint32_t)sizeof(FrictionMap), m };
    const bool ok = prefs.putBytes(NVS_KEY, &st, sizeof(st)) == sizeof(st);
    prefs.end();
    return ok;
}

// ---- TriggerIO over the haptic STM32 link -------------------------------------
class HapticTriggerIO : public TriggerIO
{
public:
    explicit HapticTriggerIO(HapticMotorController* motor) : motor_(motor) {}

    bool exchange(float u, float* pos_deg, float* force_meas) override
    {
        Response_Packet r;
        if (!motor_->set_constant_force(-u, &r))      // STM32 command is -u
            return false;
        float pos, iq;
        memcpy(&pos, &r.encoder_value,   sizeof(pos));
        memcpy(&iq,  &r.measured_torque, sizeof(iq));
        *pos_deg = pos;
        *force_meas = -iq;                             // back into u's sense
        return true;
    }

    uint32_t now_us() override { return micros(); }

    bool abort_requested() override
    {
        // Non-blocking line input: 'e' + Enter aborts.
        while (Serial.available()) {
            const char c = (char)Serial.read();
            if (c == '\n' || c == '\r') {
                const bool abort = len_ == 1 && (line_[0] == 'e' || line_[0] == 'E');
                len_ = 0;
                if (abort) return true;
            } else if (len_ < (int)sizeof(line_)) {
                line_[len_++] = c;
            }
        }
        return false;
    }

    void log(const char* msg) override { Serial.println(msg); }

    void sample(float pos_deg, float u, float force_meas) override
    {
        // Live trace for HEB Console at 100 Hz.
        if (millis() - last_sample_ms_ >= 10) {
            last_sample_ms_ = millis();
            Serial.printf("FIDS,%.2f,%.3f,%.3f\n", pos_deg, u, force_meas);
        }
    }

private:
    HapticMotorController* motor_;
    unsigned long last_sample_ms_ = 0;
    char line_[16];
    int  len_ = 0;
};

} // namespace

static float highest_floor(const FrictionMap& m)
{
    float mx = 0.0f;
    for (int i = 0; i < m.n; i++)
        mx = fmaxf(mx, m.floor_at((i + 0.5f) * m.bin_w, TRIGGER_RETURN_DEG_S, TRIGGER_FLOOR_MARGIN));
    return mx;
}

void trigger_control_begin()
{
    if (load_map())
        Serial.printf("[FID] friction map loaded: travel %.1f deg, %d bins, floor up to %.2f.\n",
                      g_map.travel, g_map.n, highest_floor(g_map));
    else
        Serial.println("[FID] no friction map stored - run 'fid' (hands off the trigger).");
}

const FrictionMap& trigger_friction_map() { return g_map; }

bool trigger_friction_id(HapticMotorController* motor, DynamixelController* gripper)
{
    Serial.println("Friction ID ON: hands off the trigger. 'e' + Enter aborts.");
    if (gripper) gripper->stop(nullptr);
    motor->relax(nullptr);
    delay(800);

    if (!motor->is_calibrated()) {
        Serial.println("[FID] finding the trigger's ends (same as 'cal')...");
        if (!motor->calibrate_trigger()) {
            Serial.println("[FID] calibration FAILED (timeout - check the haptic link).");
            Serial.println("Friction ID OFF.");
            return false;
        }
    }
    const float closed = motor->get_limit_close(), release = motor->get_limit_open();
    Serial.printf("[FID] ends: closed %.2f deg, release %.2f deg.\n", closed, release);

    // The STM32's end-of-travel fade would distort the measurement near the
    // release end; the sweeps stop short of both ends by themselves.
    motor->set_limit_gate(false);
    HapticTriggerIO io(motor);
    FrictionIdConfig cfg;
    FrictionMap m;
    const bool ok = run_friction_id(io, closed, release, cfg, &m);
    motor->relax(nullptr);
    motor->set_limit_gate(true);

    if (ok) {
        g_map = m;
        Serial.println(save_map(m) ? "[FID] done; map saved to flash."
                                   : "[FID] done; WARNING: could not save the map to flash.");
        trigger_friction_print();
    }
    Serial.println("Friction ID OFF.");
    return ok;
}

void trigger_friction_print()
{
    if (!g_map.valid) {
        Serial.println("FRICMAP,none");
        return;
    }
    Serial.printf("FRICMAP,%.2f,%d,%.3f,%.5f,%.5f,%.1f,%.3f,%.2f\n",
                  g_map.travel, g_map.n, g_map.bin_w, g_map.b_open, g_map.b_close,
                  TRIGGER_RETURN_DEG_S, TRIGGER_FLOOR_MARGIN, FrictionMap::FLOOR_MAX);
    for (int i = 0; i < g_map.n; i++) {
        const float d = (i + 0.5f) * g_map.bin_w;
        Serial.printf("FRICBIN,%.2f,%.4f,%.4f,%.4f\n", d, g_map.f_open[i], g_map.f_close[i],
                      g_map.floor_at(d, TRIGGER_RETURN_DEG_S, TRIGGER_FLOOR_MARGIN));
    }
    Serial.println("FRICEND");
}

void trigger_friction_clear()
{
    g_map = FrictionMap();
    Preferences prefs;
    if (prefs.begin(NVS_NAMESPACE, false)) {
        prefs.remove(NVS_KEY);
        prefs.end();
    }
    Serial.println("[FID] friction map cleared.");
    trigger_friction_print();
}
