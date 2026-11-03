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
// Allowed ranges, in the same degree convention as goal positions.
//
// Measured by hand with 'gpos' (torque off), 2026-11-02:
//   ID 0: closed 436 deg (= 76.07), open 341 deg (= -19)
//   ID 1: closed 91.67,             open 184.92
// Chosen (2026-11-02), kept well inside the mechanical stops:
//   ID 0: closed 60  -> open 1        ID 1: closed 120 -> open 179
// The physical closed pose (ID 0 ~76, ID 1 ~92) is OUTSIDE these ranges on
// purpose, so a gripper resting fully closed must be opened by hand into
// range before the servos will engage.
//
// RULE (enforced in the driver): if a commanded goal is outside its range,
// or either servo's present position is outside its range (+/- the pose
// tolerance), the move is rejected and both servos are relaxed (torque off).
// No clamping: out-of-range input is treated as an error, not corrected.
const float DXL_ID0_CLOSED_DEG = 60.0f;     // physical right finger
const float DXL_ID0_OPEN_DEG   = 1.0f;
const float DXL_ID1_CLOSED_DEG = 120.0f;    // physical left finger
const float DXL_ID1_OPEN_DEG   = 179.0f;
// Slack on the *present position* check only (not on commands), so normal
// settling/overshoot near an end (e.g. 0.8 deg for a 1 deg goal) is accepted.
const float DXL_POSE_TOL_DEG   = 3.0f;

// SAFETY: every servo motion must go through move_servo(), move_servo_fast()
// or move_servo_log(). Those are the only places that enforce the range rule
// above. Never write Dyn_Command_Packet bytes to the gripper serial port
// directly, and don't add a new move path that skips these checks.
class DynamixelController
{
public:
    // True if both goals are inside their allowed ranges (inclusive).
    static bool command_in_range(float id0_deg, float id1_deg);

    // True if both present positions are inside their allowed ranges,
    // widened by DXL_POSE_TOL_DEG.
    static bool pose_in_range(float id0_deg, float id1_deg);

    // Map gripper opening g (0 = closed, 1 = fully open, clamped) to goal
    // positions for both servos, linearly between the range ends. Results
    // are always in range.
    static void opening_to_goals(float g, float* id0_deg, float* id1_deg);

    DynamixelController(HardwareSerial* serial) : _serial(serial) {}

    // Start the serial port. Default pins GPIO35 (RX) / GPIO21 (TX).
    // Default baud (1,000,000) must match ESP_BAUD on the Mega side.
    void begin(long baud = 1000000, int rx_pin = 35, int tx_pin = 21);

    // Send a MOVE_SERVO command. Degrees are absolute goal positions; speed is
    // in RPM (0 = maximum speed). Subject to the range rule: out-of-range goal
    // or pose -> rejected, servos relaxed, returns false. Blocks up to timeout_ms
    // for a response when response != nullptr; fire-and-forget when nullptr.
    // Returns true if ACK received (or nullptr passed).
    bool move_servo(float id0_deg, float id1_deg,
                    float id0_speed, float id1_speed,
                    Dyn_Response_Packet* response = nullptr,
                    unsigned long timeout_ms = 200);

    // Move servos without waiting for a response. Use this in tight control
    // loops. Subject to the range rule (pose re-checked every 100 ms).
    void move_servo_fast(float id0_deg, float id1_deg);

    // Send a MOVE_SERVO_LOG command: same as move_servo(), but the Arduino
    // skips its current-draw readback (response only carries position).
    // Lower latency than move_servo() for callers that don't need current —
    // used by trigger_mapping_log()/response_plot.py. Subject to the range rule.
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

    // True if the most recent move_* call was rejected by the range rule
    // (servos were relaxed).
    bool last_move_blocked() const { return _last_blocked; }

    // For callers that read positions themselves while torque is on (e.g. a
    // watched move): if the pose is out of range, relax the servos, print
    // why, and return false. Glitch samples should be filtered by the caller
    // (read_position already does).
    bool check_pose_or_relax(float id0_deg, float id1_deg);

private:
    HardwareSerial* _serial;

    // ---- Range guard ------------------------------------------------------
    // Pose is checked (via PING, which carries positions) before any move
    // that may re-enable torque -- the Mega turns torque on at the first move
    // after a STOP or after its 500 ms heartbeat expires -- and at least every
    // POSE_CHECK_MS while moving. This also covers ID 0's 0/360 wrap: if it
    // were below 0 (reads ~350) or a turn off (reads ~436), it is out of
    // range and nothing moves.
    static constexpr unsigned long HEARTBEAT_SAFE_MS = 400;  // < Mega's 500
    static constexpr unsigned long POSE_CHECK_MS     = 100;
    unsigned long _last_tx_ms         = 0;
    unsigned long _last_pose_check_ms = 0;
    bool          _torque_maybe_off   = true;   // true at boot / after STOP
    bool          _last_blocked       = false;
    unsigned long _last_warn_ms       = 0;

    bool pose_guard();                          // true = OK to send a move
    void reject_and_relax(const char* why, bool have_pose, float p0, float p1);

    // Transmit a command packet and optionally wait for a response.
    bool send_command(const Dyn_Command_Packet& packet,
                      Dyn_Response_Packet* response,
                      unsigned long timeout_ms);

    // Hunt for DYN_START_BYTE then read the rest of a response packet.
    // Sets *valid = true only on a fully framed, valid packet.
    Dyn_Response_Packet read_response(bool* valid, unsigned long timeout_ms);
};

#endif
