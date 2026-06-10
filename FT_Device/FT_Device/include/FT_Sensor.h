#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <Wire.h>
#include <Arduino.h>
#include <math.h>

/* 
FT Sensor base class
*/
class FT_Sensor
{
protected:
    const int ID;       // sensor ID

public:
    // Default constructor & destructor
    FT_Sensor(int id) : ID(id){}
    virtual ~FT_Sensor() = default;

    /*
    Every sensor must implement
    Returns sensor pressure in inH20
    */
    virtual float get_pressure() const = 0;

    /*
    Returns the sensor id
    Input: None
    Output: int assigned sensor id
    */
    virtual int get_ID() const{ return ID; }
};

/*
Class for Analog Sensors operating under same principles as DLHR-L01D
*/
class AnalogSensor : public FT_Sensor
{
public:
    /*
    Analog sensor constructor
    Input: 
        id              - int id of the sensor
        pin             - pin on the board sensor connected to
        refV            - ADC reference voltage
        adcR            - ADC resolution
        minV            - min sensor voltage
        maxV            - max sensor voltage
        offset          - sensor offset in inH20
        scale           - sensor scaling factor (ex: 10 for L10D)
    */
    AnalogSensor(int id, int pin, float refV, int adcR, float minV, float maxV, 
        float offset, float scale) : FT_Sensor(id), pin(pin), 
        reference_voltage(refV), adc_resolution(adcR), min_volt(minV), 
        max_volt(maxV), offset(offset), scale(scale)
        {}

    /*
    Returns sensor pressure
    Input: None
    Output: Sensor pressure in inH20
    */
    float get_pressure() const override;

private:
    const int pin;                      // pin on the board 
    const float reference_voltage;      // adc ref voltage
    const int adc_resolution;           // adc resolution
    const float min_volt;               // min voltage
    const float max_volt;               // max voltage
    const float offset;                 // offset in inH20
    const float scale;                  // scaling factor for sensor

    /*
    Input: none
    Output: returns a normalized ADC reading
    */
    float get_normalized_reading() const;
};

/*
Class for Digital i2c sensors operating under same principles as DLHR-L10D and DLH-L30D
*/
class DigitalSensor : public FT_Sensor
{
public:
    /*
    Digital sensor constructor
    Input:
        id                  - int id of this sensor
        scale               - scaling factor of this sensor
        address             - address on the i2c bus
        offset              - calibrated sensor offset
    */
    DigitalSensor(int id, int scale, uint8_t address, float offset) : 
        FT_Sensor(id), scale(scale), address(address), offset(offset){}

    /*
    Returns sensor pressure
    Input : None
    Output: Sensor Pressure
    */
    float get_pressure() const;

private:
    const uint8_t address;                              // Address on the i2c bus
    const float offset;                                 // offset obtained from factory or calibration
    const int scale;                                    // sensor scale factor, ex 10 for L10D or 30 for L30D
    static constexpr uint8_t READ_CMD = 0xAA;           // command to send to read
    static constexpr float FULL_SCALE = 16777216.0f;    // 2^24

};

#endif