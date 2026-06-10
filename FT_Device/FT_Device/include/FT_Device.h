#ifndef FT_DEVICE_H
#define FT_DEVICE_H

#include <Arduino.h>
#include <vector>
#include <FT_Sensor.h>

/* Can extend this to whatever sensor device you have if needed */
class FT_Device
{
public:
    /*
    Constructor
    Input: name - the name of this device
    */
    FT_Device(const char* name) : name(name){}
    
    /* Destructor */
    ~FT_Device(){for(FT_Sensor* s : sensors) { delete s; } sensors.clear();}

    /* Disable copying */
    FT_Device(const FT_Device&) = delete;
    FT_Device& operator=(const FT_Device&) = delete;

    /* Disable moves */
    FT_Device(FT_Device&&) = delete;
    FT_Device& operator=(FT_Device&&) = delete;

    /*
    Adds a new analog sensor such as L01D
    Assigns next consecutive id value
    Input: see analog_sensor constructor
    Return: True if sensor add successful, false otherwise
    */
    bool add_analog_sensor(int pin, float refV, int adcR, float minV, float maxV, 
        float offset, float scale);

    /*
    Adds a new digital sensor such as L10D or L30D
    Assigns next consectuive id value
    Input: see digital_sensor constructor
    Return: True if sensor add successful, false otherwise
    */
    bool add_digital_sensor(int scale, uint8_t address, float offset);

    /*
    Returns pressure at given sensor
    Input: int id of sensor accessed in order of sensor addition / creation
    Output: float pressure at given sensor
        or NAN if id invalid
    */
    float get_pressure(int id);

    /*
    Return vector of pressure of all sensors
    Input: none
    Output: Vector<float> of sensor pressures
        In order of sensor creation (ie: sensor 0's pressure is at vector[0])
    */
    std::vector<float> get_all_pressures();

private:
    const char* name;                           // name of this device 
    std::vector<FT_Sensor*> sensors;            // sensors on this device
};

#endif