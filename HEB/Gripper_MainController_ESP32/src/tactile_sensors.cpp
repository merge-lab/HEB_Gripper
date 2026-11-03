#include "tactile_sensors.h"

namespace {

// Shared state: written by the sampler task, read by everyone else.
// All access goes through `mux` so readers never see a half-written sample.
float        ring[TACTILE_RING_SIZE][TACTILE_COUNT];
uint32_t     total_samples = 0;                  // next write index = total % size
float        baseline[TACTILE_COUNT] = { 0 };
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

TaskHandle_t sampler_task = nullptr;
bool         continuous   = false;

// Measured rate (updated by the sampler task about once per second).
volatile float measured_hz = 0.0f;
uint32_t       rate_count0 = 0;
unsigned long  rate_t0_ms  = 0;

const uint8_t PINS_U8[TACTILE_COUNT] = { 4, 5, 6, 8, 9, 10 };   // = TACTILE_PINS

inline float mv_to_mapped(float mv)
{
    const float span_v = TACTILE_MAX_V - TACTILE_MIN_V;
    return (((mv / 1000.0f) - TACTILE_MIN_V) / span_v) * 2.0f - 1.0f;
}

void push_sample(const float v[TACTILE_COUNT])
{
    portENTER_CRITICAL(&mux);
    memcpy(ring[total_samples % TACTILE_RING_SIZE], v, sizeof(float) * TACTILE_COUNT);
    total_samples++;
    portEXIT_CRITICAL(&mux);
}

void update_rate()
{
    unsigned long now = millis();
    if (rate_t0_ms == 0) { rate_t0_ms = now; rate_count0 = total_samples; return; }
    unsigned long dt = now - rate_t0_ms;
    if (dt >= 1000) {
        measured_hz = (total_samples - rate_count0) * 1000.0f / dt;
        rate_t0_ms  = now;
        rate_count0 = total_samples;
    }
}

// ---- Continuous (DMA) mode ----------------------------------------------------
// The ADC driver calls this from its ISR once per completed frame (one sample
// of every pin, each averaged over TACTILE_OVERSAMPLE conversions).
void IRAM_ATTR on_adc_frame()
{
    BaseType_t woken = pdFALSE;
    if (sampler_task) vTaskNotifyGiveFromISR(sampler_task, &woken);
    if (woken) portYIELD_FROM_ISR();
}

void continuous_task(void*)
{
    float v[TACTILE_COUNT];
    for (;;) {
        uint32_t frames = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
        if (frames > 2) frames = 2;      // the driver buffers two frames
        for (uint32_t k = 0; k < frames; k++) {
            adc_continuous_result_t* res = nullptr;
            if (!analogContinuousRead(&res, 0) || res == nullptr)
                break;
            for (int i = 0; i < TACTILE_COUNT; i++)
                v[i] = mv_to_mapped((float)res[i].avg_read_mvolts);   // results follow PINS_U8 order
            push_sample(v);
        }
        update_rate();
    }
}

// ---- One-shot fallback (1 kHz, tick-timed) ------------------------------------
void oneshot_task(void*)
{
    TickType_t last_wake = xTaskGetTickCount();
    float v[TACTILE_COUNT];
    for (;;) {
        for (int i = 0; i < TACTILE_COUNT; i++)
            v[i] = mv_to_mapped((float)analogReadMilliVolts(TACTILE_PINS[i]));
        push_sample(v);
        update_rate();
        vTaskDelayUntil(&last_wake, 1);
    }
}

bool start_continuous()
{
    analogContinuousSetWidth(12);
    // 11 dB: full-scale ~3.1 V on the S3, covers the sensors' 0.165..2.805 V.
    analogContinuousSetAtten(ADC_11db);
    const uint32_t adc_hz = (uint32_t)TACTILE_SAMPLE_HZ * TACTILE_COUNT * TACTILE_OVERSAMPLE;
    if (!analogContinuous(PINS_U8, TACTILE_COUNT, TACTILE_OVERSAMPLE, adc_hz, &on_adc_frame))
        return false;
    // Task first, so the ISR has someone to notify once conversions start.
    xTaskCreatePinnedToCore(continuous_task, "tactile", 4096, nullptr, 5, &sampler_task, 0);
    if (!analogContinuousStart()) {
        analogContinuousDeinit();
        vTaskDelete(sampler_task);
        sampler_task = nullptr;
        return false;
    }
    return true;
}

void start_oneshot()
{
    analogReadResolution(12);
    for (int i = 0; i < TACTILE_COUNT; i++) {
        pinMode(TACTILE_PINS[i], INPUT);
        analogSetPinAttenuation(TACTILE_PINS[i], ADC_11db);
    }
    // Core 0: the Arduino loop (CLI, haptic, gripper) runs on core 1.
    xTaskCreatePinnedToCore(oneshot_task, "tactile", 4096, nullptr, 3, &sampler_task, 0);
}

} // namespace

void tactile_begin()
{
    if (sampler_task != nullptr) return;
    continuous = start_continuous();
    if (!continuous) start_oneshot();

    // Initial tare once the buffer has enough data.
    delay(150);
    tactile_tare();
}

int tactile_nominal_hz() { return continuous ? TACTILE_SAMPLE_HZ : TACTILE_FALLBACK_HZ; }

float tactile_measured_hz() { return measured_hz; }

const char* tactile_mode_name()
{
    return continuous ? "continuous ADC (DMA) 2000 Hz, 4x oversampling"
                      : "one-shot 1000 Hz (fallback: continuous ADC failed to start)";
}

uint32_t tactile_sample_count()
{
    portENTER_CRITICAL(&mux);
    uint32_t n = total_samples;
    portEXIT_CRITICAL(&mux);
    return n;
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

int tactile_average(float out[TACTILE_COUNT], int n, bool tared)
{
    float sum[TACTILE_COUNT] = { 0 };
    portENTER_CRITICAL(&mux);
    uint32_t total = total_samples;
    if (n > TACTILE_RING_SIZE) n = TACTILE_RING_SIZE;
    if ((uint32_t)n > total) n = (int)total;
    for (int k = 0; k < n; k++) {
        const float* s = ring[(total - 1 - k) % TACTILE_RING_SIZE];
        for (int i = 0; i < TACTILE_COUNT; i++) sum[i] += s[i];
    }
    for (int i = 0; i < TACTILE_COUNT; i++)
        out[i] = n > 0 ? sum[i] / n - (tared ? baseline[i] : 0.0f) : 0.0f;
    portEXIT_CRITICAL(&mux);
    return n;
}

int tactile_read_new(uint32_t* cursor, float (*out)[TACTILE_COUNT], int max,
                     bool tared, uint32_t* dropped)
{
    portENTER_CRITICAL(&mux);
    uint32_t total = total_samples;
    uint32_t c = *cursor;
    if (total - c > (uint32_t)TACTILE_RING_SIZE) {      // reader fell behind
        if (dropped) *dropped += total - c - TACTILE_RING_SIZE;
        c = total - TACTILE_RING_SIZE;
    }
    int n = (int)(total - c);
    if (n > max) n = max;
    for (int k = 0; k < n; k++) {
        const float* s = ring[(c + k) % TACTILE_RING_SIZE];
        for (int i = 0; i < TACTILE_COUNT; i++)
            out[k][i] = tared ? s[i] - baseline[i] : s[i];
    }
    *cursor = c + n;
    portEXIT_CRITICAL(&mux);
    return n;
}

bool tactile_tare(int n)
{
    if (n < 1) n = 1;
    if (n > TACTILE_RING_SIZE) n = TACTILE_RING_SIZE;
    if (tactile_sample_count() < (uint32_t)n) return false;

    float avg[TACTILE_COUNT];
    tactile_average(avg, n, false);
    portENTER_CRITICAL(&mux);
    memcpy(baseline, avg, sizeof(baseline));
    portEXIT_CRITICAL(&mux);
    return true;
}

void tactile_baseline(float out[TACTILE_COUNT])
{
    portENTER_CRITICAL(&mux);
    memcpy(out, baseline, sizeof(baseline));
    portEXIT_CRITICAL(&mux);
}
