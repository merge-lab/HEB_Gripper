#include "FT_Sensor.h"

// Simple constructor for ESP32 and old analog reference voltages
FT_Sensor::FT_Sensor(int pin, int id, int scale, SENSOR_TYPE sensortype)
    : PIN(pin), ID(id), ADC_REF_VOLT(3.3), ADC_RES(4095), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)), 
    MIN_VOLTAGE(0.25 * (3.3 / 5.0)), MAX_VOLTAGE(4.25 * (3.3 / 5.0)), 
    SCALE(scale), TYPE(sensortype)
    {}

// Main constructor for analog sensors
FT_Sensor::FT_Sensor(int pin, int id, float ref_volt, float offset,
    int res, float minVolt, float maxVolt, int scale, SENSOR_TYPE sensortype) 
    : PIN(pin), ID(id), ADC_REF_VOLT(ref_volt), ADC_RES(res), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)), OFFSET(offset),
    MIN_VOLTAGE(minVolt), MAX_VOLTAGE(maxVolt), SCALE(scale), TYPE(sensortype)
    {}

// Main constructor for digital sensors
FT_Sensor::FT_Sensor(int scl_pin, int sda_pin, int address, int id, int scale,
    SENSOR_TYPE sensortype, float offset) : SCL_PIN(scl_pin), SDA_PIN(sda_pin), 
    ADDRESS(address), ID(id), SCALE(scale), TYPE(sensortype), OFFSET(offset)
{}

// calculates voltage from adc raw value
float FT_Sensor::normalize_adc_val(int adcVal) const
{
    return ((adcVal * ADC_FACTOR) - MIN_VOLTAGE) / 
        (MAX_VOLTAGE - MIN_VOLTAGE) * 2.0 - 1.0;
}

// return the raw ADC value for analog sensors
int FT_Sensor::get_raw_value_analog() const
{
    if(TYPE != ANALOG_SENSOR){return INT_MAX;}
    return analogRead(PIN);
}

// return the calculated voltage for analog sensors
float FT_Sensor::get_normalized_analog() const
{
    if(TYPE != ANALOG_SENSOR){return NAN;}
    return normalize_adc_val(get_raw_value_analog());
}

// return raw value for digital sensor
int FT_Sensor::get_raw_value_digital() const
{
    if(TYPE != DIGITAL_SENSOR){return INT_MAX;}
    
    // send command
    Wire.beginTransmission(ADDRESS);
    Wire.write(0xAA);
    Wire.endTransmission();

    // wait
    delay(10);

    // request data
    Wire.requestFrom(ADDRESS, 7);
    uint8_t status = Wire.read();

    uint32_t p = 0;
    p |= ((uint32_t)Wire.read()) << 16; // MSB
    p |= ((uint32_t)Wire.read()) << 8;
    p |= ((uint32_t)Wire.read());

    Wire.read(); // temp MSB (ignore if not used)
    Wire.read();
    Wire.read();
    return p;
}

// return sthe sensor pressure
float FT_Sensor::get_pressure() const
{
    if(TYPE == ANALOG_SENSOR)
    {
        float normal = get_normalized_analog();
        if(isnan(normal)){return NAN;}
        return (normal * SCALE) - OFFSET;
    }
    if(TYPE == DIGITAL_SENSOR)
    {
        int raw = get_raw_value_digital();
        if(raw == INT_MAX){return NAN;}
        return (raw - OFFSET) * (2.0f * SCALE / FULL_SCALE_COUNTS);
    }
    return NAN;
}

// return sensor type
SENSOR_TYPE FT_Sensor::get_type() const
{
    return TYPE;
}

// return sensor scale
int FT_Sensor::get_scale() const
{
    return SCALE;
}