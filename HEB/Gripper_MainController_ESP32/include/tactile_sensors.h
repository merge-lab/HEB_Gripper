#ifndef TACTILE_SENSORS_H
#define TACTILE_SENSORS_H

#include <Arduino.h>

// Six analog tactile channels T1..T6, read with the ESP32-S3's ADC1.
// Pins: GPIO 4, 5, 6, 8, 9, 10 (all ADC1).
//   T1..T3: physical LEFT finger  (servo ID 1 side)
//   T4..T6: physical RIGHT finger (servo ID 0 side)
//
// Each reading is mapped linearly from the sensor's output range
// (0.25..4.25 V on a 5 V supply, scaled to 3.3 V: 0.165..2.805 V)
// to -1.0..+1.0. Values are not clamped.
//
// Sampling: the ADC's hardware continuous (DMA) mode converts all six pins
// at TACTILE_SAMPLE_HZ per channel, each sample the average of
// TACTILE_OVERSAMPLE conversions. A task on core 0 moves each sample into a
// ring buffer in shared memory; readers copy from it under a spinlock and
// never touch the ADC. If continuous mode can't start, a 1 kHz one-shot
// fallback is used instead (see tactile_mode_name()).
//
// Tare: a per-channel baseline (average of recent samples) that "tared"
// reads subtract, so an unloaded sensor reads ~0. Taken at boot and on
// tactile_tare().

const int   TACTILE_COUNT = 6;
const int   TACTILE_PINS[TACTILE_COUNT] = { 4, 5, 6, 8, 9, 10 };

const float TACTILE_MIN_V = 0.25f * (3.3f / 5.0f);   // ~0.165 V -> -1.0
const float TACTILE_MAX_V = 4.25f * (3.3f / 5.0f);   // ~2.805 V -> +1.0

const int   TACTILE_SAMPLE_HZ    = 2000;  // per channel (continuous mode)
const int   TACTILE_OVERSAMPLE   = 4;     // conversions averaged per sample
const int   TACTILE_FALLBACK_HZ  = 1000;  // one-shot fallback rate
const int   TACTILE_RING_SIZE    = 1024;  // ~0.5 s of history at 2 kHz
const int   TACTILE_TARE_SAMPLES = 200;   // tare averages the last ~100 ms

// Start sampling (continuous ADC, or the fallback) and take an initial tare.
// Call once from setup(). Blocks ~150 ms.
void tactile_begin();

// Nominal per-channel sample rate of the active mode (2000 or 1000).
int tactile_nominal_hz();

// Measured sample rate over the last second.
float tactile_measured_hz();

// "continuous ADC (DMA) 2000 Hz, 4x oversampling" or the fallback name.
const char* tactile_mode_name();

// Total samples taken since boot (also the index of the next sample).
uint32_t tactile_sample_count();

// Newest sample. tared = true subtracts the baseline.
// Returns the total number of samples taken so far (0 = none yet).
uint32_t tactile_latest(float out[TACTILE_COUNT], bool tared = true);

// Mean of the newest n samples (n is capped to what's available).
// Returns the number of samples averaged.
int tactile_average(float out[TACTILE_COUNT], int n, bool tared = true);

// Copy samples newer than *cursor (a sample index, e.g. from
// tactile_sample_count()) into out, oldest first, up to max. Advances
// *cursor. If the reader fell more than a ring behind, the oldest samples
// are skipped and counted in *dropped (optional). Returns samples copied.
int tactile_read_new(uint32_t* cursor, float (*out)[TACTILE_COUNT], int max,
                     bool tared = true, uint32_t* dropped = nullptr);

// Set the baseline to the average of the last n samples (default ~100 ms).
// Keep the sensors unloaded while taring. Returns false if not enough data.
bool tactile_tare(int n = TACTILE_TARE_SAMPLES);

// Copy of the current baseline.
void tactile_baseline(float out[TACTILE_COUNT]);

#endif
