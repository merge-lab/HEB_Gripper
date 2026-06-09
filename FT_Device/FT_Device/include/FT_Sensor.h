#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <Wire.h>
#include <Arduino.h>
#include <cfloat>
#include <climits>
static constexpr float FULL_SCALE_COUNTS = 16777216.0f; // 2^24

// allows us to use any type of sensors
enum SENSOR_TYPE{
    DIGITAL_SENSOR, // DLHR-L10D and DLH-L30D
    ANALOG_SENSOR   // DLHR-L01D
};

class FT_Sensor
{
public:
    /*
    Simple constructor for ESP32 with analog sensor
    assumes: ESP32, min_voltage = ~0.165v, max_voltage = ~2.805v  
    Input:
        pin     - integer pin used for the sensor
        id      - id of the sensor
    */
    FT_Sensor(int pin, int id, int scale, SENSOR_TYPE sensortype);

    /*
    Full constructor for analog sensor on any board
    Input:
        pin         - integer pin used for the sensor
        id          - id of this sensor
        ref_volt    - reference voltage of this board's ADC
        offset      - pre-calibrated pressure offset
        res         - resolution of this board's ADC
        minVolt     - min possible voltage of the reading
        maxVolt     - max possible voltage of the reading
        scale       - given by the X in the L-X-D of the name
        sensortype  - analog or digital
    */
    FT_Sensor(int pin, int id, float ref_volt, float offset, int res, float minVolt, float maxVolt, int scale, SENSOR_TYPE sensortype);

    /*
    Full constructor for digital i2c on any board
    Input:
        scl_pin     - the pin used for scl
        sda_pin     - the pin used for sda
        address     - device address on the i2c line
        id          - id of the sensor
        scale       - given by the X in the L-X-D of the name
        sensortype  - analog or digital
        offset      - pre-calculated unitless sensor offset
    NOTE: This calibrates the sensor offset, so must be done without pressure on the sensor
    NOTE: Requires the use of wire to work whenever an i2c sensor exists. Should include:
        ''' Wire.begin(SDA_PIN, SCL_PIN)
            calibrate_digital()'''
        in void setup
    */
    FT_Sensor(int scl_pin, int sda_pin, int address, int id, int scale, SENSOR_TYPE sensortype, float offset);

    /*
    Returns calculated pressure value
    Input: None
    Output: float calculated pressure in inH20
        or NAN if something failed
    */
    float get_pressure() const;

    /*
    Returns the type of sensor being used
    Input: None
    Output: SENSOR_TYPE of the sensor
    */
    SENSOR_TYPE get_type() const;

    /*
    Returns the scale of this sensor
    Input: None
    Output: integer SCALE value
    */
    int get_scale() const;


private:
    // For analog sensors
    const int PIN               = 0;    // pin used for the sensor
    const float ADC_REF_VOLT    = 0.0;  // reference voltage for the ADC, 3.3 for ESP32
    const int ADC_RES           = 0;    // adc resolution, 4095 for ESP32
    const float ADC_FACTOR      = 0.0;  // calculated from voltage and resolution
    const float MIN_VOLTAGE     = 0.0;  // obtained via calibration
    const float MAX_VOLTAGE     = 0.0;  // obtained via calibration

    // for digital sensors
    const int SDA_PIN           = 0;    // pin sda is plugged to
    const int SCL_PIN           = 0;    // pin scl is plugged to
    const int ADDRESS           = 0;    // the device number this is on the i2c line
    const float OFFSET          = 0.0;  // factor calibratred device offset

    // for all sensors
    const int ID;               // id of this sensor
    const int SCALE;            // given by the X in the L-X-D of the name
    const SENSOR_TYPE TYPE;     // whether this is a digital or analog sensor

    /*
    Calculates the sensor voltage from ADC value
    Input: raw ADC value (integer)
    Output: sensor voltage (float)
    */
    float normalize_adc_val(int adcVal) const;

    /*
    Returns the raw ADC finger value
    Input: None
    Output: integer adc value in range [0, ADC_RES]
        or INT_MAX if sensor is not analog
    */
    int get_raw_value_analog() const;

    /*
    Returns the calculated sensor voltage
    Input: None
    Output: float voltage in range [MIN_VOLTAGE, MAX_VOLTAGE]
        or NAN if sensor is not analog
    */
    float get_normalized_analog() const;

    /*
    Returns the raw pressure value from the digital sensor
    Input: none
    Output: unscaled sensor reading
        or INT_MAX if sensor is not digital or failure occurs
    */
    int get_raw_value_digital() const;
};

#endif