#include "temp_senor.h"
#include <Wire.h>

// MCP9808 register map (CJMCU-9808 breakout), default I2C address.
static const uint8_t MCP9808_I2C_ADDR = 0x18;
static const uint8_t REG_AMBIENT_TEMP = 0x05;

CJMCU9908_Temp_Sensor::CJMCU9908_Temp_Sensor(int scl_pin, int sda_pin, int alert_temp)
    : SCL_PIN(scl_pin), SDA_PIN(sda_pin), ALERT_TEMPERATURE(alert_temp), wire_begun(false)
{
}

void CJMCU9908_Temp_Sensor::ensure_wire_begun()
{
    if (wire_begun)
        return;

    Wire.begin(SDA_PIN, SCL_PIN);
    wire_begun = true;
}

uint16_t CJMCU9908_Temp_Sensor::read_register(uint8_t reg)
{
    Wire.beginTransmission(MCP9808_I2C_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(MCP9808_I2C_ADDR, (uint8_t)2);
    uint16_t value = 0;
    if (Wire.available() >= 2)
    {
        value = (uint16_t)Wire.read() << 8;
        value |= Wire.read();
    }
    return value;
}

float CJMCU9908_Temp_Sensor::get_temp()
{
    ensure_wire_begun();

    uint16_t raw = read_register(REG_AMBIENT_TEMP);

    // Top 3 bits are TCRIT/TUPPER/TLOWER flags; bit 12 is sign; value is in 1/16 C.
    raw &= 0x1FFF;
    float temp_c = (raw & 0x0FFF) / 16.0f;
    if (raw & 0x1000)
        temp_c -= 256.0f;

    return temp_c;
}

bool CJMCU9908_Temp_Sensor::is_temp_exceeded()
{
    return get_temp() >= ALERT_TEMPERATURE;
}
