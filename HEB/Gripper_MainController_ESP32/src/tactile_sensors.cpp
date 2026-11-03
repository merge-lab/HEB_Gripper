#include "tactile_sensors.h"

void tactile_begin()
{
    analogReadResolution(12);
    for (int i = 0; i < TACTILE_COUNT; i++) {
        pinMode(TACTILE_PINS[i], INPUT);
        // 11 dB attenuation: full-scale ~3.1 V on the S3, covers the
        // sensors' 0.165..2.805 V output.
        analogSetPinAttenuation(TACTILE_PINS[i], ADC_11db);
    }
}

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
