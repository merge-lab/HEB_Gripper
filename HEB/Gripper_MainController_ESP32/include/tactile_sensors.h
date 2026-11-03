#ifndef TACTILE_SENSORS_H
#define TACTILE_SENSORS_H

#include <Arduino.h>

// Six analog tactile channels T1..T6, read with the ESP32-S3's ADC1.
// Pins: GPIO 4, 5, 6, 8, 9, 10 (all ADC1, so they keep working with WiFi on).
//
// Each reading is mapped linearly from the sensor's output range
// (0.25..4.25 V on a 5 V supply, scaled to 3.3 V: 0.165..2.805 V)
// to -1.0..+1.0. Values are not clamped, so a reading outside the
// calibrated range shows up as < -1 or > +1.

const int   TACTILE_COUNT = 6;
const int   TACTILE_PINS[TACTILE_COUNT] = { 4, 5, 6, 8, 9, 10 };

const float TACTILE_MIN_V = 0.25f * (3.3f / 5.0f);   // ~0.165 V -> -1.0
const float TACTILE_MAX_V = 4.25f * (3.3f / 5.0f);   // ~2.805 V -> +1.0

// Configure the ADC pins. Call once from setup().
void tactile_begin();

// Read all channels. out_mapped gets -1..+1 values; out_mv (optional)
// gets the measured voltage in millivolts.
void tactile_read(float out_mapped[TACTILE_COUNT], uint32_t out_mv[TACTILE_COUNT] = nullptr);

#endif
