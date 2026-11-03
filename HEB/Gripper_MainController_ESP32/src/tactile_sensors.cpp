#include "tactile_sensors.h"

namespace {

// Shared state: written by the sampler task, read by everyone else.
// All access goes through `mux` so readers never see a half-written sample.
float        ring[TACTILE_RING_SIZE][TACTILE_COUNT];
uint32_t     total_samples = 0;                  // next write index = total % size
float        baseline[TACTILE_COUNT] = { 0 };
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t sampler_task = nullptr;

void sampler(void*)
{
    const TickType_t period = pdMS_TO_TICKS(1000 / TACTILE_SAMPLE_HZ) > 0
                            ? pdMS_TO_TICKS(1000 / TACTILE_SAMPLE_HZ) : 1;
    TickType_t last_wake = xTaskGetTickCount();
    float v[TACTILE_COUNT];

    for (;;) {
        tactile_read(v);   // ADC work happens outside the lock

        portENTER_CRITICAL(&mux);
        memcpy(ring[total_samples % TACTILE_RING_SIZE], v, sizeof(v));
        total_samples++;
        portEXIT_CRITICAL(&mux);

        vTaskDelayUntil(&last_wake, period);
    }
}

} // namespace

void tactile_read(float out_mapped[TACTILE_COUNT], uint32_t out_mv[TACTILE_COUNT])
{
    const float span_v = TACTILE_MAX_V - TACTILE_MIN_V;
    for (int i = 0; i < TACTILE_COUNT; i++) {
        // analogReadMilliVolts uses the chip's factory ADC calibration, which
        // is more accurate than raw * 3.3 / 4095 (the S3's ADC is neither
        // 3.3 V full-scale nor perfectly linear).
        uint32_t mv = analogReadMilliVolts(TACTILE_PINS[i]);
        if (out_mv) out_mv[i] = mv;
        float v = mv / 1000.0f;
        out_mapped[i] = ((v - TACTILE_MIN_V) / span_v) * 2.0f - 1.0f;
    }
}

void tactile_begin()
{
    analogReadResolution(12);
    for (int i = 0; i < TACTILE_COUNT; i++) {
        pinMode(TACTILE_PINS[i], INPUT);
        // 11 dB attenuation: full-scale ~3.1 V on the S3, covers the
        // sensors' 0.165..2.805 V output.
        analogSetPinAttenuation(TACTILE_PINS[i], ADC_11db);
    }

    if (sampler_task == nullptr) {
        // Core 0: the Arduino loop (CLI, haptic, gripper) runs on core 1.
        xTaskCreatePinnedToCore(sampler, "tactile", 4096, nullptr, 3, &sampler_task, 0);
    }

    // Initial tare once the buffer has enough data.
    delay(150);
    tactile_tare();
}

uint32_t tactile_latest(float out[TACTILE_COUNT], bool tared)
{
    portENTER_CRITICAL(&mux);
    uint32_t n = total_samples;
    if (n > 0) {
        const float* s = ring[(n - 1) % TACTILE_RING_SIZE];
        for (int i = 0; i < TACTILE_COUNT; i++)
            out[i] = tared ? s[i] - baseline[i] : s[i];
    }
    portEXIT_CRITICAL(&mux);

    if (n == 0)
        for (int i = 0; i < TACTILE_COUNT; i++) out[i] = 0.0f;
    return n;
}

bool tactile_tare(int n)
{
    if (n < 1) n = 1;
    if (n > TACTILE_RING_SIZE) n = TACTILE_RING_SIZE;

    double sum[TACTILE_COUNT] = { 0 };
    bool ok;

    portENTER_CRITICAL(&mux);
    uint32_t total = total_samples;
    ok = total >= (uint32_t)n;
    if (ok) {
        for (int k = 0; k < n; k++) {
            const float* s = ring[(total - 1 - k) % TACTILE_RING_SIZE];
            for (int i = 0; i < TACTILE_COUNT; i++) sum[i] += s[i];
        }
        for (int i = 0; i < TACTILE_COUNT; i++)
            baseline[i] = (float)(sum[i] / n);
    }
    portEXIT_CRITICAL(&mux);
    return ok;
}

void tactile_baseline(float out[TACTILE_COUNT])
{
    portENTER_CRITICAL(&mux);
    memcpy(out, baseline, sizeof(baseline));
    portEXIT_CRITICAL(&mux);
}
