#ifndef FT_DEVICE_H
#define FT_DEVICE_H

#include <Arduino.h>
#include <vector>
#include <FT_Sensor.h>

class FT_Device
{
public:
    /*
    Constructor
    Input: name - the name of this device
    */
    FT_Device(const char* name) : name(name){}
    
    /* Destructor */
    ~FT_Device(){for(FT_Sensor* s : sensors) { delete s; }}

    /* Disable copying */
    FT_Device(const FT_Device&) = delete;
    FT_Device& operator=(const FT_Device&) = delete;

    /*
    Adds a new analog sensor such as L01D
    */
    void add_analog_sensor(int id, int pin, float refV, int adcR, float minV, float maxV, 
        float offset, float scale)
    {
        sensors.push_back(new AnalogSensor(id, pin, refV, adcR, minV, maxV, offset, scale));
    }

    /*
    Adds a new digital sensor such as L10D or L30D
    */

private:
    const char* name;                           // name of this device
    std::vector<FT_Sensor*> sensors;            // sensors on this device
};

#endif