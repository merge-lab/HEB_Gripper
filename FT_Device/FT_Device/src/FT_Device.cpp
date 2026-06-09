#include <FT_Device.h>

// Simple constructor for ESP32
FT_Device::FT_Device(std::initializer_list<int> sensorPins, String id)
    : NUM_SENSORS(sensorPins.size()), ID(id), ADC_REF_VOLT(3.3), ADC_RES(4095), 
    ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)), 
    MIN_VOLTAGE(0.25 * (3.3 / 5.0)), MAX_VOLTAGE(4.25 * (3.3 / 5.0))
{
    sensors.reserve(sensorPins.size());
    int i = 0;
    for(int pin: sensorPins)
    {
        // auto=generate sensor id
        String made_id = "sensor_" + String(i);
        sensors.emplace_back(pin, made_id, ADC_REF_VOLT, 
            ADC_RES, MIN_VOLTAGE, MAX_VOLTAGE);
        i++;
    }
}

// generalized constructor for any board
FT_Device::FT_Device(std::initializer_list<int> sensorPins, String id, 
    float ref_volt, int res, float minVolt, float maxVolt)
    : NUM_SENSORS(sensorPins.size()), ID(id), ADC_REF_VOLT(ref_volt), 
    ADC_RES(res), ADC_FACTOR(ADC_REF_VOLT / float(ADC_RES)),
    MIN_VOLTAGE(minVolt), MAX_VOLTAGE(maxVolt)
{
    sensors.reserve(sensorPins.size());
    int i = 0;
    for(int pin: sensorPins)
    {
        // auto=generate sensor id
        String made_id = "sensor_" + String(i);
        sensors.emplace_back(pin, made_id, ADC_REF_VOLT, 
            ADC_RES, MIN_VOLTAGE, MAX_VOLTAGE);
        i++;
    }
}

float FT_Device::get_sensor_pressure(int pin_number) const
{
    // error if invalid pin number
    if(pin_number < 0 || pin_number >= NUM_SENSORS){return -1;}

    return sensors[pin_number].get_pressure();
}

std::vector<float> FT_Device::get_all_pressures() const
{
    std::vector<float> readings;
    readings.reserve(sensors.size());

    for (const FT_Sensor& sensor : sensors) {
        readings.push_back(sensor.get_pressure());
    }

    return readings;
}