#ifndef FT_DEVICE_H
#define FT_DEVICE_H

#include <Arduino.h>
#include <vector>
#include <FT_Sensor.h>

class FT_Device
{
public:
    /*
    Simple constructor for ESP32
    assumes: ESP32, min_voltage = ~0.165v, max_voltage = ~2.805v  
    Input:
        sensorPins{}     - sensor pin list
        id               - string name of the device
    */
    FT_Device(std::initializer_list<int> sensorPins, String id);

    /*
    Full constructor for any board
    Input:
        sensorPins{}    - sensor pin list
        id              - name of this sensor
        ref_volt        - reference voltage of this board's ADC
        res             - resolution of this board's ADC
        minVolt         - min possible voltage of the reading
        maxVolt         - max possible voltage of the reading
    */
    FT_Device(std::initializer_list<int> sensorPins, String id,
        float ref_volt, int res, float minVolt, float maxVolt);


    /*
    Returns the pressure of sensor i, accessed in order of creation list
    Input: 
        int i - index of desired sensor in initialization list
    Output: 
        float pressure reading on sensor i in inH20
        -1 if invalid sensor number
    */
    float get_sensor_pressure(int pin_number) const;

    /*
    get pressure of all sensors
    Input: None
    Output:
        vectr<float> of all sensor's pressures in order of initialization in inH20
    */
    std::vector<float> get_all_pressures() const;


private:
    const int NUM_SENSORS;      // number of sensors on board
    const float ADC_REF_VOLT;   // reference voltage for the ADC, 3.3 for ESP32
    const int ADC_RES;          // adc resolution, 4095 for ESP32
    const float ADC_FACTOR;     // calculated from voltage and resolution
    const float MIN_VOLTAGE;    // obtained via calibration
    const float MAX_VOLTAGE;    // obtained via calibration
    const String ID;            // id of this FT_Device
    std::vector<FT_Sensor> sensors;     // vector of sensors

};

#endif