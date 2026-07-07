#include "FORTE_Fingers.h"
#include <cmath>
#include <cstring>

// ── IIR bandpass coefficients (10–50 Hz @ 1 kHz, cascaded 1st-order) ────────
// Derived via bilinear transform of 1st-order Butterworth LP/HP prototypes.
//
// HP stage (fc = 10 Hz):
//   y_hp[n] = HP_B * (x[n] - x[n-1]) + ALPHA_HP * y_hp[n-1]
static constexpr float ALPHA_HP = 0.93906251f;
static constexpr float HP_B     = 0.96953125f;  // (1 + ALPHA_HP) / 2

// LP stage (fc = 50 Hz):
//   y_lp[n] = LP_B0 * (y_hp[n] + y_hp[n-1]) + LP_ALPHA * y_lp[n-1]
static constexpr float LP_B0    = 0.13672874f;
static constexpr float LP_ALPHA = 0.72654253f;

// ── ADC scaling ───────────────────────────────────────────────────────────────
static constexpr float ADC_FACTOR = FF_ADC_REF / static_cast<float>(FF_ADC_RES);
static constexpr float VRANGE     = FF_VMAX - FF_VMIN;

// ── Regressor weights ─────────────────────────────────────────────────────────
// Replace with your trained model weights; the macro in FORTE_Fingers.h
// selects which model compiles in.

// Ridge: force = RIDGE_BIAS + sum(RIDGE_W[i] * x[i])
static constexpr float RIDGE_BIAS   = 0.5f;
static constexpr float RIDGE_W[6]   = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f};

// MLP: 6 → 8 (ReLU) → 1
static constexpr int   MLP_HIDDEN   = 8;
static constexpr float MLP_W1[MLP_HIDDEN][6] = {
    { 1.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f},
    { 0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f,  0.0f,  1.0f,  0.0f},
    { 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f},
    { 0.5f,  0.5f,  0.5f,  0.5f,  0.5f,  0.5f},
    {-0.5f, -0.5f, -0.5f, -0.5f, -0.5f, -0.5f},
};
static constexpr float MLP_B1[MLP_HIDDEN] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.0f, 0.0f};
static constexpr float MLP_W2[MLP_HIDDEN] = {0.4f, 0.6f, 0.8f, 1.0f, 1.2f, 1.4f, 0.5f, -0.3f};
static constexpr float MLP_B2             = 0.25f;

// ── Constructor ───────────────────────────────────────────────────────────────

FORTE_Fingers::FORTE_Fingers(const int pins[6])
    : rms_head_(0), hist_head_(0), hist_count_(0), sample_count_(0), new_data_(false)
{
    for (int i = 0; i < 6; i++) {
        pins_[i]       = pins[i];
        cur_mapped_[i] = 0.f;
        hp_x_prev_[i]  = 0.f;
        hp_y_prev_[i]  = 0.f;
        lp_x_prev_[i]  = 0.f;
        lp_y_prev_[i]  = 0.f;
        rms_sum_[i]    = 0.f;
        for (int j = 0; j < FF_RMS_WINDOW; j++) rms_buf_[i][j]  = 0.f;
        for (int j = 0; j < FF_HIST_LEN;   j++) rms_hist_[i][j] = 0.f;
    }
    latest_   = {{0.f, 0.f, 0.f, 0.f, 0.f, 0.f}, 0.f, false};
    mutex_    = xSemaphoreCreateMutex();
}

// ── IIR bandpass filter ───────────────────────────────────────────────────────

float FORTE_Fingers::apply_bandpass(int ch, float x) {
    // HP: high-pass at 10 Hz
    float hp = HP_B * (x - hp_x_prev_[ch]) + ALPHA_HP * hp_y_prev_[ch];
    hp_x_prev_[ch] = x;
    hp_y_prev_[ch] = hp;

    // LP: low-pass at 50 Hz applied to HP output
    float lp = LP_B0 * (hp + lp_x_prev_[ch]) + LP_ALPHA * lp_y_prev_[ch];
    lp_x_prev_[ch] = hp;
    lp_y_prev_[ch] = lp;

    return lp;
}

// ── Running RMS (O(1) update) ─────────────────────────────────────────────────

void FORTE_Fingers::update_rms(int ch, float sq) {
    rms_sum_[ch] -= rms_buf_[ch][rms_head_];
    rms_buf_[ch][rms_head_] = sq;
    rms_sum_[ch] += sq;
}

// ── Regressor ─────────────────────────────────────────────────────────────────

float FORTE_Fingers::predict_force(const float x[6]) {
#if FF_USE_RIDGE
    float y = RIDGE_BIAS;
    for (int i = 0; i < 6; i++) y += RIDGE_W[i] * x[i];
    return y;
#elif FF_USE_MLP
    float h[MLP_HIDDEN];
    for (int i = 0; i < MLP_HIDDEN; i++) {
        float s = MLP_B1[i];
        for (int j = 0; j < 6; j++) s += MLP_W1[i][j] * x[j];
        h[i] = s > 0.f ? s : 0.f;
    }
    float y = MLP_B2;
    for (int i = 0; i < MLP_HIDDEN; i++) y += MLP_W2[i] * h[i];
    return y;
#else
    (void)x;
    return 0.f;
#endif
}

// ── Regressor + slip detection (100 Hz) ──────────────────────────────────────

void FORTE_Fingers::run_regressor_and_slip() {
    // Append current per-channel RMS to rolling history
    for (int ch = 0; ch < 6; ch++) {
        float r = sqrtf(rms_sum_[ch] / static_cast<float>(FF_RMS_WINDOW));
        rms_hist_[ch][hist_head_] = r;
    }
    if (hist_count_ < FF_HIST_LEN) hist_count_++;
    hist_head_ = (hist_head_ + 1) % FF_HIST_LEN;

    // Slip: monotonic variance gate, averaged per finger side (ch 0-2 = left, 3-5 = right)
    bool slip = false;
    if (hist_count_ == FF_HIST_LEN) {
        for (int side = 0; side < 2 && !slip; side++) {
            float side_var = 0.f;
            int valid = 0;

            for (int ch = side * 3; ch < side * 3 + 3; ch++) {
                // Check that history is monotonically increasing oldest→newest
                bool mono = true;
                for (int k = 0; k < FF_HIST_LEN - 1; k++) {
                    int ia = (hist_head_ + k)     % FF_HIST_LEN;
                    int ib = (hist_head_ + k + 1) % FF_HIST_LEN;
                    if (rms_hist_[ch][ib] - rms_hist_[ch][ia] <= FF_MONO_THRESH) {
                        mono = false;
                        break;
                    }
                }
                if (!mono) continue;

                // Variance of this channel's history
                float sum = 0.f, sum_sq = 0.f;
                for (int k = 0; k < FF_HIST_LEN; k++) {
                    float v = rms_hist_[ch][k];
                    sum    += v;
                    sum_sq += v * v;
                }
                float mean = sum / FF_HIST_LEN;
                side_var += (sum_sq / FF_HIST_LEN) - (mean * mean);
                valid++;
            }

            if (valid > 0 && (side_var / valid) > FF_SLIP_VAR_THRESH)
                slip = true;
        }
    }

    // Force estimate from regressor
    float force = predict_force(cur_mapped_);

    // Write complete result under mutex (minimise lock time — all computation above)
    xSemaphoreTake(mutex_, portMAX_DELAY);
    for (int i = 0; i < 6; i++) latest_.mapped[i] = cur_mapped_[i];
    latest_.force = force;
    latest_.slip  = slip;
    new_data_     = true;
    xSemaphoreGive(mutex_);
}

// ── Main update (1 kHz, called from FreeRTOS task) ───────────────────────────

void FORTE_Fingers::update() {
    // 1. Read and normalize sensors into private working buffer
    for (int i = 0; i < 6; i++) {
        float v = static_cast<float>(analogRead(pins_[i])) * ADC_FACTOR;
        cur_mapped_[i] = ((v - FF_VMIN) / VRANGE) * 4.0f - 2.0f;
    }

    // 2. Bandpass filter + squared value into running RMS buffer
    for (int ch = 0; ch < 6; ch++) {
        float f = apply_bandpass(ch, cur_mapped_[ch]);
        update_rms(ch, f * f);
    }
    rms_head_ = (rms_head_ + 1) % FF_RMS_WINDOW;

    // 3. Every FF_REGRESSOR_EVERY samples: regressor + slip → write latest_
    if (++sample_count_ >= FF_REGRESSOR_EVERY) {
        sample_count_ = 0;
        run_regressor_and_slip();
    }
}

// ── Thread-safe getter (core 1) ───────────────────────────────────────────────

bool FORTE_Fingers::get_latest(FORTE_Reading& out) {
    xSemaphoreTake(mutex_, portMAX_DELAY);
    bool fresh = new_data_;
    if (fresh) {
        out       = latest_;
        new_data_ = false;
    }
    xSemaphoreGive(mutex_);
    return fresh;
}

// ── FreeRTOS task ─────────────────────────────────────────────────────────────

void forte_task(void* arg) {
    FORTE_Fingers* f = static_cast<FORTE_Fingers*>(arg);
    TickType_t last_wake = xTaskGetTickCount();
    for (;;) {
        f->update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1));
    }
}
