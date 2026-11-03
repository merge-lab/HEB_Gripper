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

// Streamed goals (move_servo_fast / move_servo_log) are shaped in the driver:
// each servo's commanded position follows the requested one with at most
// this speed and acceleration, never overshooting it. The shaper restarts
// from the MEASURED pose every time torque (re)engages (mode start, after a
// relax, after a heartbeat lapse), so the gripper never jumps to a stale goal.
const float GRIPPER_MAX_DEG_PER_S  = 300.0f;    // ~servo top speed
const float GRIPPER_MAX_DEG_PER_S2 = 6000.0f;   // ramp up/down

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
    // watched move): if the pose is out of range on two samples in a row,
    // relax the servos, print why, and return false. (A single out-of-range
    // sample is usually a bad read on the Mega and is ignored once.)
    bool check_pose_or_relax(float id0_deg, float id1_deg);

    // Count of Mega replies that reported a problem (MALFORMED / FAILURE /
    // unknown code) since boot. A rising count means the link is overloaded.
    unsigned long bad_reply_count() const { return _bad_replies; }

    // Most recent valid position feedback (from any reply) and its age.
    // Returns false if none has been received yet.
    bool latest_feedback(float* id0_deg, float* id1_deg, unsigned long* age_ms) const;

    // Round-trip time of the last asynchronous position request (ms).
    unsigned long feedback_latency_ms() const { return _fb_latency_ms; }

    // Speed cap of the streamed-goal shaper (deg/s), clamped to
    // (0, GRIPPER_MAX_DEG_PER_S]. Slow test moves lower it temporarily
    // instead of changing the servos' own Profile Velocity, which would
    // persist on the servos until they are power-cycled.
    void  set_stream_speed_limit(float deg_per_s);
    float stream_speed_limit() const { return _sm_vmax; }

private:
    HardwareSerial* _serial;

    // ---- Link pacing --------------------------------------------------------
    // The Mega is single-threaded and slow to answer: a reply needs servo bus
    // reads, and each failed read blocks it for up to 100 ms. Bytes arriving
    // from the ESP32 while it reads the servos can be lost (and can overflow
    // its 64-byte input buffer). So:
    //  * streamed moves go out at most every MOVE_MIN_INTERVAL_MS, and only
    //    when the goal changed;
    //  * position feedback is requested every POSE_CHECK_MS as a
    //    MOVE_SERVO_LOG (a move whose reply carries positions only - half the
    //    servo reads of a PING), and nothing else is sent until that reply
    //    arrives or POSE_REQ_TIMEOUT_MS passes;
    //  * replies are picked up without blocking (poll_feedback).
    static constexpr unsigned long HEARTBEAT_SAFE_MS    = 400;   // < Mega's 500
    static constexpr unsigned long POSE_CHECK_MS        = 100;
    static constexpr unsigned long POSE_STALE_MS        = 500;   // no feedback -> relax
    static constexpr unsigned long POSE_REQ_TIMEOUT_MS  = 150;   // > one Mega read timeout
    static constexpr unsigned long MOVE_MIN_INTERVAL_MS = 10;    // <= 100 Hz
    static constexpr float         MOVE_DEADBAND_DEG    = 0.05f;

    unsigned long _last_tx_ms         = 0;
    unsigned long _last_move_tx_ms    = 0;
    unsigned long _last_pose_check_ms = 0;
    bool          _torque_maybe_off   = true;   // true at boot / after STOP
    bool          _last_blocked       = false;
    unsigned long _last_warn_ms       = 0;
    float         _sent0 = NAN, _sent1 = NAN;    // last streamed goal
    // After a rejection, wait this long before trying to (re)engage again,
    // so a finger left out of range can't make every control-loop pass block
    // on the strict position check.
    static constexpr unsigned long ENGAGE_BACKOFF_MS = 300;
    unsigned long _engage_retry_ms    = 0;

    // ---- Asynchronous feedback ----------------------------------------------
    uint8_t       _rx_buf[sizeof(Dyn_Response_Packet)];
    size_t        _rx_idx          = 0;
    bool          _req_outstanding = false;
    unsigned long _req_ms          = 0;
    bool          _fb_new          = false;
    float         _fb0 = 0.0f, _fb1 = 0.0f;
    unsigned long _fb_ms           = 0;
    unsigned long _fb_latency_ms   = 0;
    unsigned long _bad_replies     = 0;
    int           _bad_pose_streak = 0;

    // ---- Goal shaper ----------------------------------------------------------
    bool          _sm_ready = false;
    float         _sm_vmax  = GRIPPER_MAX_DEG_PER_S;
    float         _sm_pos[2] = { 0.0f, 0.0f };
    float         _sm_vel[2] = { 0.0f, 0.0f };
    unsigned long _sm_last_us = 0;

    void poll_feedback();
    void note_feedback(float p0, float p1);
    // true = OK to send a move. *engaged = torque (re)engages with this move
    // (strict check passed and the shaper was seeded from the measured pose).
    bool pose_guard(bool* engaged);
    void seed_shaper(float p0, float p1);
    void shape_goals(float* id0_deg, float* id1_deg);
    bool check_reply_pose(const Dyn_Response_Packet* r);
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
