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

// ---- Servo identity & range of motion ---------------------------------------
// The wire protocol (and the Mega firmware) call ID 0 "left" and ID 1 "right",
// but physically it is the other way round:
//   ID 0 = physical RIGHT finger  -> sent in the packet's left_*  fields
//   ID 1 = physical LEFT  finger  -> sent in the packet's right_* fields
// Driver arguments below are therefore named by ID, not by side.
//
// Goal ranges in the same degree convention as goal positions. Every goal the
// ESP32 sends is clamped to these ranges inside the driver.
//
// Measured by hand with 'gpos' (torque off), 2026-11-02:
//   ID 0: closed 436 deg (= 76.07), open 341 deg (= -19)
//   ID 1: closed 91.67,             open 184.92
// Chosen: the original mapping's values, 90 deg of travel on each servo:
//   ID 0: closed 90 -> open 0     ID 1: closed 90 -> open 180
// Note ID 0's closed goal (90) is ~14 deg past its measured closed pose
// (76), so at closed it presses into the other finger / stop. If it trips
// overload protection (servo stops responding, LED blinks), lower this.
const float DXL_ID0_CLOSED_DEG = 90.0f;     // physical right finger
const float DXL_ID0_OPEN_DEG   = 0.0f;
const float DXL_ID1_CLOSED_DEG = 90.0f;     // physical left finger
const float DXL_ID1_OPEN_DEG   = 180.0f;

// SAFETY: every servo motion must go through move_servo(), move_servo_fast()
// or move_servo_log(). Those are the only places that apply the range clamp
// and the ID 0 wrap guard. Never write Dyn_Command_Packet bytes to the
// gripper serial port directly, and don't add a new move path without
// calling clamp_to_range() and engage_check() first.
class DynamixelController
{
public:
    // Clamp goal positions (deg) to each servo's measured range.
    // Returns true if either value had to be changed.
    static bool clamp_to_range(float* id0_deg, float* id1_deg);

    // Map gripper opening g (0 = closed, 1 = fully open, clamped) to goal
    // positions for both servos, linearly between the measured limits.
    static void opening_to_goals(float g, float* id0_deg, float* id1_deg);

    DynamixelController(HardwareSerial* serial) : _serial(serial) {}

    // Start the serial port. Default pins GPIO35 (RX) / GPIO21 (TX).
    // Default baud (1,000,000) must match ESP_BAUD on the Mega side.
    void begin(long baud = 1000000, int rx_pin = 35, int tx_pin = 21);

    // Send a MOVE_SERVO command. Degrees are absolute goal positions (clamped
    // to range); speed is in RPM (0 = maximum speed). Blocks up to timeout_ms
    // for a response when response != nullptr; fire-and-forget when nullptr.
    // Returns true if ACK received (or nullptr passed).
    bool move_servo(float id0_deg, float id1_deg,
                    float id0_speed, float id1_speed,
                    Dyn_Response_Packet* response = nullptr,
                    unsigned long timeout_ms = 200);

    // Move servos (clamped to range) without waiting for a response. Use this
    // in tight control loops.
    void move_servo_fast(float id0_deg, float id1_deg);

    // Send a MOVE_SERVO_LOG command: same as move_servo(), but the Arduino
    // skips its current-draw readback (response only carries position).
    // Lower latency than move_servo() for callers that don't need current —
    // used by trigger_mapping_log()/response_plot.py. Goals are clamped.
    bool move_servo_log(float id0_deg, float id1_deg,
                        Dyn_Response_Packet* response,
                        unsigned long timeout_ms = 200);

    // Stop both servos (torque off).
    bool stop(Dyn_Response_Packet* response = nullptr,
              unsigned long timeout_ms = 200);

    // Ping the Arduino; returns true if PONG received within timeout_ms.
    bool ping(unsigned long timeout_ms = 200);

    // Read both servos' present positions (deg). Uses DYN_PING, whose PONG
    // reply already carries positions, so it never changes torque state and
    // needs no Mega-side changes. Returns true on a valid PONG.
    bool read_position(float* id0_deg, float* id1_deg,
                       unsigned long timeout_ms = 200);

    // True if the most recent move_* call was refused by the wrap guard
    // (see engage_check) rather than sent.
    bool last_move_blocked() const { return _last_blocked; }

private:
    HardwareSerial* _serial;

    // ---- Wrap guard -------------------------------------------------------
    // ID 0's range sits right next to 0/360. If it is below 0 (e.g. -10)
    // when torque comes on, the servo treats it as ~350 and drives the long
    // way (negative direction) into its open stop. The Mega turns torque on
    // at the first move after a STOP or after its 500 ms heartbeat expires,
    // so before any move that may re-enable torque, read positions and
    // refuse unless ID 0 is within [0, 100] deg.
    static constexpr unsigned long HEARTBEAT_SAFE_MS = 400;  // < Mega's 500
    unsigned long _last_tx_ms       = 0;
    bool          _torque_maybe_off = true;   // true at boot / after STOP
    bool          _last_blocked     = false;
    unsigned long _last_warn_ms     = 0;
    bool engage_check();

    // Transmit a command packet and optionally wait for a response.
    bool send_command(const Dyn_Command_Packet& packet,
                      Dyn_Response_Packet* response,
                      unsigned long timeout_ms);

    // Hunt for DYN_START_BYTE then read the rest of a response packet.
    // Sets *valid = true only on a fully framed, valid packet.
    Dyn_Response_Packet read_response(bool* valid, unsigned long timeout_ms);
};

#endif
