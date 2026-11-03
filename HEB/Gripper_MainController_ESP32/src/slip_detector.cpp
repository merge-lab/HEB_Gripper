#include "slip_detector.h"

#include <esp_timer.h>
#include "tactile_sensors.h"

namespace {

SlipCore     core;                     // ~14 KB of state: static, not on a stack
TaskHandle_t task = nullptr;

// Shared with the CLI (core 1); guarded by mux.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
SlipConfig   config;
SlipResult   latest;
bool         latch = false;
uint32_t     n_updates = 0;
SlipStats    stats;

void detector_task(void*)
{
    static float batch[32][TACTILE_COUNT];
    uint32_t cursor  = tactile_sample_count();
    uint32_t dropped = 0;

    // One-second measurement window for the stats.
    int64_t  win_start = esp_timer_get_time();
    int64_t  busy_us = 0, upd_us_sum = 0, upd_us_max = 0;
    uint32_t win_updates = 0;

    for (;;) {
        SlipConfig cfg;
        portENTER_CRITICAL(&mux);
        cfg = config;
        portEXIT_CRITICAL(&mux);

        const int n = tactile_read_new(&cursor, batch, 32, /*tared=*/false, &dropped);
        for (int k = 0; k < n; k++) {
            SlipResult r;
            const int64_t t0 = esp_timer_get_time();
            const bool updated = core.push(batch[k], cfg, &r);
            const int64_t dt = esp_timer_get_time() - t0;
            busy_us += dt;
            if (!updated) continue;

            portENTER_CRITICAL(&mux);
            latest = r;
            latch |= (r.slip != 0);
            n_updates++;
            portEXIT_CRITICAL(&mux);

            win_updates++;
            upd_us_sum += dt;
            if (dt > upd_us_max) upd_us_max = dt;
        }

        const int64_t now = esp_timer_get_time();
        if (now - win_start >= 1000000) {
            const float secs = (now - win_start) / 1e6f;
            SlipStats s;
            s.updates_per_s  = win_updates / secs;
            s.update_us_avg  = win_updates ? (float)upd_us_sum / win_updates : 0.0f;
            s.update_us_max  = (float)upd_us_max;
            s.core0_load_pct = 100.0f * busy_us / (now - win_start);
            s.dropped        = dropped;
            portENTER_CRITICAL(&mux);
            stats = s;
            portEXIT_CRITICAL(&mux);
            win_start = now;
            busy_us = upd_us_sum = upd_us_max = 0;
            win_updates = 0;
        }

        if (n == 0) vTaskDelay(1);     // ~2 new samples per ms at 2 kHz
    }
}

} // namespace

void slip_begin()
{
    if (task) return;
    // Core 0, just below the tactile ADC task (5) so sampling always wins.
    xTaskCreatePinnedToCore(detector_task, "slip", 4096, nullptr, 4, &task, 0);
}

void slip_take(SlipStatus* out)
{
    portENTER_CRITICAL(&mux);
    out->result          = latest;
    out->slip_since_last = latch;
    out->updates         = n_updates;
    latch = false;
    portEXIT_CRITICAL(&mux);
}

void slip_set_config(const SlipConfig& cfg)
{
    portENTER_CRITICAL(&mux);
    config = cfg;
    portEXIT_CRITICAL(&mux);
}

SlipConfig slip_config()
{
    portENTER_CRITICAL(&mux);
    SlipConfig c = config;
    portEXIT_CRITICAL(&mux);
    return c;
}

SlipStats slip_stats()
{
    portENTER_CRITICAL(&mux);
    SlipStats s = stats;
    portEXIT_CRITICAL(&mux);
    return s;
}
