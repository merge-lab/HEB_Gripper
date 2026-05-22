#ifndef FORTE_H
#define FORTE_H
#include <Arduino.h>

// stores raw ADC data
struct Finger_Raw_ADC{int tip; int mid; int base;};

// stores converted voltages
struct Finger_Voltages{float tip; float mid; float base;};

/* Wrapper class for a FORTE Finger */
class ForteFinger
{
public:
    /* 
    Simple constructor for ESP32
    assumes: ESP32, min_voltage = ~0.165v, max_voltage = ~2.805v  
    Input: 
        tip         - the pin of the transducer at the tip
        mid         - the pin of the transducer at the middle
        base        - the pin of the transducer at the base
        id          - name of this finger
    */
    ForteFinger(int tip, int mid, int base, String id);

    /* 
    Fully customizable constructor. Assumes nothing
    Can use this with any board and any fingers
    Input:
        tip         - the pin of the transducer at the tip
        mid         - the pin of the transducer at the middle
        base        - the pin of the transducer at the base
        id          - name of this finger
        ref_volt    - reference voltage of this board's ADC
        res         - resolution of this board's ADC
        minVolt     - min possible voltage of the reading
        maxVolt     - max possible voltage of the reading
    */
    ForteFinger(int tip, int mid, int base, String id, 
        float ref_volt, int res, float minVolt, float maxVolt);

    /*
    Gets the raw ADC data. should only be used internally and for debugging
    Returns struct Finger_Raw_ADC {int tip; int mid; int base;};
    */
    Finger_Raw_ADC getRawData();

    /*
    Calculates and returns the ADC voltages for external use.
    Returns struct Finger_Voltages(float tip; float mid; float base;);
    */
    Finger_Voltages getFingerVoltages();


private:
    const int TIP_PIN;          // transducer that senses the tip
    const int MIDDLE_PIN;       // transducer that senses the middle
    const int BASE_PIN;         // transducer that senses the base
    const float ADC_REF_VOLT;   // reference voltage on the ADC, 3.3 for ESP32
    const int ADC_RES;          // ADC resolution, 4095 for ESP32
    const float ADC_FACTOR;     // calculated from ref voltage and resolution
    const float MIN_VOLTAGE;    // obtained via calibration
    const float MAX_VOLTAGE;    // obtained via calibration
    String name;                // name of this finger

    /*
    Converts a raw ADC value to a voltage
    Input: raw adc val in range int:[0, ADC_RES]
    Output: voltage in range float:[min_voltage, max_voltage]
    */
    float adcToVoltage(int adcVal);
};

#endif