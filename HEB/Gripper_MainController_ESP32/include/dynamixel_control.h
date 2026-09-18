#ifndef DYNAMIXEL_CONTROL_H
#define DYNAMIXEL_CONTROL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include "dynamixel_protocol.h"

// ESP32-side driver for the Arduino Mega / DynamixelShield gripper board.
//
// Wiring (Serial1, default pins — pass RX/TX to begin() to override):
//   ESP32 TX (GPIO21)      -> Mega RX2 (pin 17)
//   ESP32 RX (GPIO35) <- [level shifter] <- Mega TX2 (pin 16)
//   Common GND
//
// The Mega runs at 5V logic and the ESP32 does not tolerate 5V on its GPIOs,
// so the Mega TX2 -> ESP32 RX leg goes through an external level shifter
// down to 3.3V. ESP32 TX -> Mega RX2 needs no shifting (3.3V reads as HIGH).
//
// Avoid on ESP32: GPIO 2 (LED), 4-6/8-10 (analog), 16/17 (haptic), 19/20 (USB D-/D+).

class DynamixelController
{
public:
    DynamixelController(HardwareSerial* serial) : _serial(serial) {}

    // Start the serial port. Default pins GPIO35 (RX) / GPIO21 (TX).
    // Default baud (1,000,000) must match ESP_BAUD on the Mega side.
    void begin(long baud = 1000000, int rx_pin = 35, int tx_pin = 21);

    // Send a MOVE_SERVO command. Degrees are absolute goal positions; speed is
    // in RPM (0 = maximum speed). Blocks up to timeout_ms for a response when
    // response != nullptr; fire-and-forget when nullptr.
    // Returns true if ACK received (or nullptr passed).
    bool move_servo(float left_deg, float right_deg,
                    float left_speed, float right_speed,
                    Dyn_Response_Packet* response = nullptr,
                    unsigned long timeout_ms = 200);

    // Move servos without waiting for a response. Use this in tight control
    // loops — avoids the Arduino's SoftwareSerial TX blocking its RX.
    void move_servo_fast(float left_deg, float right_deg);

    // Stop both servos (torque off).
    bool stop(Dyn_Response_Packet* response = nullptr,
              unsigned long timeout_ms = 200);

    // Ping the Arduino; returns true if PONG received within timeout_ms.
    bool ping(unsigned long timeout_ms = 200);

private:
    HardwareSerial* _serial;

    // Transmit a command packet and optionally wait for a response.
    bool send_command(const Dyn_Command_Packet& packet,
                      Dyn_Response_Packet* response,
                      unsigned long timeout_ms);

    // Hunt for DYN_START_BYTE then read the rest of a response packet.
    // Sets *valid = true only on a fully framed, valid packet.
    Dyn_Response_Packet read_response(bool* valid, unsigned long timeout_ms);
};

#endif
