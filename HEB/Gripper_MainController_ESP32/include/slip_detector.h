#ifndef SLIP_DETECTOR_H
#define SLIP_DETECTOR_H

#include <Arduino.h>
#include "slip_core.h"

// FORTE slip detection running on the ESP32 (algorithm: slip_core.h).
//
// A task on core 0 feeds every tactile sample (2 kHz, raw - not tared, so a
// re-tare can't look like a vibration) through SlipCore and publishes the
// newest result. The thresholds can be changed at any time; HEB Console
// stores them and sends them on connect.

struct SlipStatus
{
    SlipResult result;                  // newest result
    bool       slip_since_last = false; // any slip result since the previous slip_take()
    uint32_t   updates = 0;             // results computed since boot
};

struct SlipStats
{
    float    updates_per_s  = 0.0f;     // ~500 when keeping up
    float    update_us_avg  = 0.0f;     // one 6-channel PSD update
    float    update_us_max  = 0.0f;
    float    core0_load_pct = 0.0f;     // detector's share of core 0
    uint32_t dropped        = 0;        // tactile samples missed (fell behind)
};

// Start the detector task. Call after tactile_begin().
void slip_begin();

// Newest result. Clears the "slip since last" latch, so a short slip between
// two reports is never missed. Intended for a single reader (the CLI loop).
void slip_take(SlipStatus* out);

void       slip_set_config(const SlipConfig& cfg);
SlipConfig slip_config();
SlipStats  slip_stats();

#endif
