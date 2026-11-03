#ifndef TACTILE_SENSORS_H
#define TACTILE_SENSORS_H

#include <Arduino.h>

// Six analog tactile channels T1..T6, read with the ESP32-S3's ADC1.
// Pins: GPIO 4, 5, 6, 8, 9, 10 (all ADC1, so they keep working with WiFi on).
//   T1..T3: physical LEFT finger  (servo ID 1 side)
//   T4..T6: physical RIGHT finger (servo ID 0 side)
//
// Each reading is mapped linearly from the sensor's output range
// (0.25..4.25 V on a 5 V supply, scaled to 3.3 V: 0.165..2.805 V)
// to -1.0..+1.0. Values are not clamped.
//
// Sampling runs in a background FreeRTOS task (core 0) at TACTILE_SAMPLE_HZ,
// writing into a ring buffer in shared memory. Readers never touch the ADC:
// they copy the newest sample (or average recent ones) under a spinlock, so
// any loop can get fresh values in microseconds.
//
// Tare: a per-channel baseline (average of recent samples) that "tared"
// reads subtract, so an unloaded sensor reads ~0. Taken at boot and on
// tactile_tare().

const int   TACTILE_COUNT = 6;
const int   TACTILE_PINS[TACTILE_COUNT] = { 4, 5, 6, 8, 9, 10 };

const float TACTILE_MIN_V = 0.25f * (3.3f / 5.0f);   // ~0.165 V -> -1.0
const float TACTILE_MAX_V = 4.25f * (3.3f / 5.0f);   // ~2.805 V -> +1.0

const int   TACTILE_SAMPLE_HZ   = 1000;
const int   TACTILE_RING_SIZE   = 256;    // ~256 ms of history at 1 kHz
const int   TACTILE_TARE_SAMPLES = 100;   // tare averages the last ~100 ms

// Configure the ADC pins, start the sampling task, and take an initial tare.
// Call once from setup(). Blocks ~150 ms.
void tactile_begin();

// Newest sample. tared = true subtracts the baseline.
// Returns the total number of samples taken so far (0 = none yet).
uint32_t tactile_latest(float out[TACTILE_COUNT], bool tared = true);

// Set the baseline to the average of the last n samples (default ~100 ms).
// Keep the sensors unloaded while taring. Returns false if not enough data.
bool tactile_tare(int n = TACTILE_TARE_SAMPLES);

// Copy of the current baseline.
void tactile_baseline(float out[TACTILE_COUNT]);

// Direct, blocking ADC read (used by the sampler task). out_mv optional.
void tactile_read(float out_mapped[TACTILE_COUNT], uint32_t out_mv[TACTILE_COUNT] = nullptr);

#endif
