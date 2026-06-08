#include "FT_Sensor.h"

// Simple constructor for ESP32 and old reference voltages
FT_Sensor::FT_Sensor(int pin, String id)
    : PIN(pin), ID(id), ADC_REF_VOLT(3.3), ADC_RES(4095), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)), 
    MIN_VOLTAGE(0.25 * (3.3 / 5.0)), MAX_VOLTAGE(4.25 * (3.3 / 5.0))
    {}

// Main constructor
FT_Sensor::FT_Sensor(int pin, String id, float ref_volt, 
    int res, float minVolt, float maxVolt) 
    : PIN(pin), ID(id), ADC_REF_VOLT(ref_volt), ADC_RES(res), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)),
    MIN_VOLTAGE(minVolt), MAX_VOLTAGE(maxVolt)
    {}

// calculates voltage from adc raw value
float FT_Sensor::adc_to_voltage(int adcVal)
{
    return ((adcVal * ADC_FACTOR) - MIN_VOLTAGE) / 
        (MAX_VOLTAGE - MIN_VOLTAGE) * 2.0 - 1.0;
}

// return the raw ADC value
int FT_Sensor::get_raw_value()
{
    return analogRead(PIN);
}

// return the calculated voltage
float FT_Sensor::get_sensor_voltage()
{
    return adc_to_voltage(get_raw_value());
}