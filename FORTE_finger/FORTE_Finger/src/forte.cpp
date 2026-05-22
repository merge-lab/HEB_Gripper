#include "Forte.h"

// Simple constructor for ESP32 and references voltages
ForteFinger::ForteFinger(int tip, int mid, int base, String id)
    : TIP_PIN(tip), MIDDLE_PIN(mid), BASE_PIN(base), name(id), 
    ADC_REF_VOLT(3.3), ADC_RES(4095), ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)),
    MIN_VOLTAGE(0.25 * (3.3 / 5.0)), MAX_VOLTAGE(4.25 * (3.3 / 5.0))
    {}

// Fully customizable constructor
ForteFinger::ForteFinger(int tip, int mid, int base, String id, float ref_volt, 
    int res, float minVolt, float maxVolt) 
    : TIP_PIN(tip), MIDDLE_PIN(mid), BASE_PIN(base), name(id), 
    ADC_REF_VOLT(ref_volt), ADC_RES(res), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)),
    MIN_VOLTAGE(minVolt), MAX_VOLTAGE(maxVolt)
    {}

// returns raw finger data
Finger_Raw_ADC ForteFinger::getRawData()
{
    return {
        analogRead(TIP_PIN),
        analogRead(MIDDLE_PIN),
        analogRead(BASE_PIN)
    };
}

// converts from integer adc val to float voltage val
float ForteFinger::adcToVoltage(int adcVal)
{
    return ((adcVal * ADC_FACTOR) - MIN_VOLTAGE) / 
        (MAX_VOLTAGE - MIN_VOLTAGE) * 2.0 - 1.0;
}

// returns finger data in voltages
Finger_Voltages ForteFinger::getFingerVoltages()
{
    Finger_Raw_ADC data = getRawData();
    return {
        adcToVoltage(data.tip),
        adcToVoltage(data.mid),
        adcToVoltage(data.base)
    };
}