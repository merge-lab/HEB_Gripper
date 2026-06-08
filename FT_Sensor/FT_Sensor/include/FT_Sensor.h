#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <Arduino.h>

class FT_Sensor
{
public:
    /*
    Simple constructor for ESP32
    assumes: ESP32, min_voltage = ~0.165v, max_voltage = ~2.805v  
    input:
        pin     - integer pin used for the sensor
        id      - string name of the sensor
    */
    FT_Sensor(int pin, String id);

    /*
    Full constructor for any board
    Input:
    pin         - integer pin used for the sensor
    id          - name of this sensor
    ref_volt    - reference voltage of this board's ADC
    res         - resolution of this board's ADC
    minVolt     - min possible voltage of the reading
    maxVolt     - max possible voltage of the reading
    */
    FT_Sensor(int pin, String id, float ref_volt, int res, float minVolt, float maxVolt);

    /*
    Returns the raw ADC finger value
    Input: None
    Output: integer adc value in range [0, ADC_RES]
    */
    int get_raw_value();

    /*
    Returns the calculated sensor voltage
    Input: None
    Output: float voltage in range [MIN_VOLTAGE, MAX_VOLTAGE]
    */
    float get_sensor_voltage();

private:
    const int PIN;              // pin used for the sensor
    const float ADC_REF_VOLT;   // reference voltage for the ADC, 3.3 for ESP32
    const int ADC_RES;          // adc resolution, 4095 for ESP32
    const float ADC_FACTOR;     // calculated from voltage and resolution
    const float MIN_VOLTAGE;    // obtained via calibration
    const float MAX_VOLTAGE;    // obtained via calibration
    const String ID;            // name / id of this sensor

    /*
    Calculates the sensor voltage from ADC value
    Input: raw ADC value (integer)
    Output: sensor voltage (float)
    */
    float adc_to_voltage(int adcVal);
};

#endif