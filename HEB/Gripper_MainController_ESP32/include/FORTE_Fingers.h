#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Regressor model select ─────────────────────────────────────────────────
// Set exactly one to 1.
#define FF_USE_RIDGE 1
#define FF_USE_MLP   0

// ── Tunable constants ──────────────────────────────────────────────────────
// ADC / sensor scaling
constexpr float FF_ADC_REF   = 3.3f;
constexpr int   FF_ADC_RES   = 4095;
constexpr float FF_VS        = 3.0f;
constexpr float FF_VMIN      = 0.1f * FF_VS;   // 0.3 V
constexpr float FF_VMAX      = 0.9f * FF_VS;   // 2.7 V

// Slip detection
constexpr int   FF_RMS_WINDOW      = 100;    // samples in RMS window (100 ms @ 1 kHz)
constexpr int   FF_HIST_LEN        = 15;     // RMS history points for variance gate
constexpr int   FF_REGRESSOR_EVERY = 10;     // run regressor+slip every N samples (100 Hz)
constexpr float FF_MONO_THRESH     = 0.05f;  // min RMS delta between consecutive history points
constexpr float FF_SLIP_VAR_THRESH = 0.01f;  // variance threshold to declare slip

// Haptic response (used by CLI_Control)
constexpr int   FF_SLIP_VIB_FREQ = 30;
constexpr float FF_SLIP_VIB_AMP  = 0.3f;

// ── Output struct ──────────────────────────────────────────────────────────
struct FORTE_Reading {
    float mapped[6];  // normalized sensor values in [-2, 2]
    float force;      // regressor force estimate
    bool  slip;       // slip detected this window
};

// ── Class ──────────────────────────────────────────────────────────────────
class FORTE_Fingers {
public:
    explicit FORTE_Fingers(const int pins[6]);

    // Called from core-0 task every 1 ms. Reads sensors, runs bandpass filter
    // and RMS; every FF_REGRESSOR_EVERY samples also runs regressor + slip
    // detection and writes results under mutex.
    void update();

    // Thread-safe read for core 1. Returns true if new data is available
    // since the last call; copies the result into `out` and clears the flag.
    bool get_latest(FORTE_Reading& out);

private:
    int pins_[6];

    // Working sensor buffer — core-0 only, not shared
    float cur_mapped_[6];

    // Cascaded 1st-order IIR: HP (10 Hz) then LP (50 Hz)
    float hp_x_prev_[6], hp_y_prev_[6];
    float lp_x_prev_[6], lp_y_prev_[6];

    // Running RMS: circular buffer of squared filtered values, one per channel
    float rms_buf_[6][FF_RMS_WINDOW];
    float rms_sum_[6];
    int   rms_head_;

    // Slip: rolling per-channel RMS history
    float rms_hist_[6][FF_HIST_LEN];
    int   hist_head_;
    int   hist_count_;

    int   sample_count_;

    // Thread-safe output — written under mutex_ by core 0, read by core 1
    SemaphoreHandle_t mutex_;
    FORTE_Reading     latest_;
    bool              new_data_;

    float apply_bandpass(int ch, float x);
    void  update_rms(int ch, float filtered_sq);
    void  run_regressor_and_slip();
    float predict_force(const float x[6]);
};

// FreeRTOS task entry point — pass FORTE_Fingers* as arg
void forte_task(void* arg);
