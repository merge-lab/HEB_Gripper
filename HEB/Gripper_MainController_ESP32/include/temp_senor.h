#ifndef TEMP_SENOR_H
#define TEMP_SENOR_H

#include <Arduino.h>

class CJMCU9908_Temp_Sensor
{
public:
    // Constructor
    CJMCU9908_Temp_Sensor(int scl_pin, int sda_pin, int alert_temp);

    // get the current temperature
    float get_temp();

    // true if the most recent temperature reading is at/above the alert threshold
    bool is_temp_exceeded();

private:
    // Pins
    const int SCL_PIN;
    const int SDA_PIN;

    // temp alert threshold
    const int ALERT_TEMPERATURE;

    bool wire_begun;                // whether Wire.begin() has been called yet

    // Lazily starts the I2C bus on SCL_PIN/SDA_PIN (safe to call repeatedly).
    void ensure_wire_begun();

    // MCP9808 register read helper.
    uint16_t read_register(uint8_t reg);
};

#endif
