#include <FT_Sensor.h>

float AnalogSensor::get_normalized_reading() const
{
    int raw = analogRead(pin);
    float voltage = raw * (reference_voltage / adc_resolution);
    float normalized = (voltage - min_volt) / (max_volt - min_volt);
    return normalized;
}

float AnalogSensor::get_pressure() const
{
    return (get_normalized_reading() * scale) - offset;
}

float DigitalSensor::get_pressure() const
{
    Wire.beginTransmission(address);
    Wire.write(READ_CMD);
    Wire.endTransmission();

    Wire.requestFrom(address, (uint8_t)7);
    if (Wire.available() < 7) return NAN;

    Wire.read(); // status

    uint32_t raw = 0;
    raw |= ((uint32_t)Wire.read()) << 16;
    raw |= ((uint32_t)Wire.read()) << 8;
    raw |= Wire.read();

    // skip temperature bytes
    Wire.read(); Wire.read(); Wire.read();

    return (raw - offset) * (2.0f * scale / FULL_SCALE);
}