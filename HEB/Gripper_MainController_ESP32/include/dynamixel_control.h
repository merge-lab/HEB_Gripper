#pragma once

#include <Arduino.h>
#include "Master_Side.h"
#include "dynamixel_protocol.h"

// ESP32-side driver for the Arduino Uno / DynamixelShield gripper board, over
// SPI + a dedicated data-ready interrupt line (see ESP--Arduino-SPI-Controller).
//
// Wiring: mosi/miso/sck are the shared SPI bus lines; ss_pin/data_ready_pin
// are this Arduino's dedicated chip-select and data-ready GPIOs. The Uno is
// 5V and the ESP32 is 3.3V/not 5V-tolerant, so MISO and the data-ready line
// both need a resistor-divider level shift (see ESP--Arduino-SPI-Controller/wiring.txt).
class DynamixelController
{
public:
    struct Telemetry {
        float goal_left_deg, goal_right_deg;
        float actual_left_deg, actual_right_deg;
    };

    DynamixelController(int mosi_pin, int miso_pin, int sck_pin,
                        int ss_pin, int data_ready_pin);

    void begin();     // call once from setup()
    void update();    // call at the top of loop(): drains the ring buffer

    // Send a move command. One single non-blocking SPI transfer (~17 bytes @
    // 1MHz, inherent transfer time only - no artificial delay), safe to call
    // at high rate (e.g. every iteration of a trigger-following loop).
    // left/right_speed_rpm: 0 = no profile-velocity override.
    //
    // Returns whether the *previous* command sent to this slave was
    // acknowledged - a rolling link-health signal (confirms the physical
    // connection/clock are set up correctly), NOT a synchronous confirmation
    // that *this* specific move was received. See dynamixel_protocol.h.
    bool move(float left_deg, float right_deg,
             float left_speed_rpm = 0.0f, float right_speed_rpm = 0.0f);

    bool stop(); // torque off both servos; same rolling-ack semantics as move()
    bool ping(); // link check; same rolling-ack semantics as move()

    // Fills *out with the most recently known settle/telemetry snapshot
    // (always populated, possibly stale/zero if none has arrived yet).
    // Returns true only if this is new since the last call to this function.
    bool get_latest_position(Telemetry* out);

private:
    // Declaration order matters: _ss_pin/_dr_pin must be constructed before
    // _master, since _master's constructor is given their addresses.
    int _ss_pin;
    int _dr_pin;
    Master_Controller _master;

    static const int SLAVE_IDX = 0; // exactly one slave (the gripper Arduino) today

    bool      _has_new_telemetry = false;
    Telemetry _latest{};

    bool send_command(uint8_t command, float d1, float d2, float d3, float d4);
};
