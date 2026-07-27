#ifndef HAPTIC_MOTOR_CONTROL_H
#define HAPTIC_MOTOR_CONTROL_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Wire protocol (command/response packets, framing bytes, enums) is shared
// with the STM32 motor controller.
#include "gripper_protocol.h"


class HapticMotorController
{
public:
    HapticMotorController(HardwareSerial* s2): SerialConnection(s2){}

    /*
    Sends a ping and blocks for wait_time or until response
    Input: wait_time - how long to wait until ending
    Output: true if pong response recieved, false otherwise
    */
    bool ping_controller(unsigned long wait_time);

    /*
    Tell device to use constant force.
    Input:
        float force - commanded torque (sign selects direction); magnitude is
            clamped by the motor controller's saturation limit
        ResponsePacket* recieved - pointer for the response packet
            - if NULL, will not block waiting for a response (fire-and-forget)
            - if not null, filled with the response; only valid when true is returned
    Returns: true if a valid response was received (or NULL was passed,
             meaning we did not wait); false if the response was invalid/timed out
    */
    bool set_constant_force(float force, Response_Packet* recieved);

    /*
    Tell device to use constant force at given newtons.
    Input:
        float force - commanded newtons of force output (sign selects direction)
        ; magnitude is clamped by the motor controller's saturation limit
        ResponsePacket* recieved - pointer for the response packet
            - if NULL, will not block waiting for a response (fire-and-forget)
            - if not null, filled with the response; only valid when true is returned
    Returns: true if a valid response was received (or NULL was passed,
             meaning we did not wait); false if the response was invalid/timed out
    */
    bool set_constant_newton_force(float force, Response_Packet* recieved);

    /*
    Tell device to use spring force
    Input:
        float spring_constant - spring constant to emulate
        ResponsePacket* recieved - pointer for the response packet
            - if NULL, will not block waiting for a response (fire-and-forget)
            - if not null, filled with the response; only valid when true is returned
    Returns: true if a valid response was received (or NULL was passed,
             meaning we did not wait); false if the response was invalid/timed out
    */
    bool set_spring_force(float spring_constant, Response_Packet* recieved);

    /*
    Tell device to relax / apply no force.
    Note: Vibrate cannot be used in this mode
    Input:
        ResponsePacket* recieved - pointer for the response packet
            - if NULL, will ot block waiting for a response
            - if not null, filled with the response, only valid if true is returned
    Returns: true if valid response recieved or NULL was passed in, false if response invalid / timed out
    */
    bool relax(Response_Packet* recieved);

    /*
    Tell device to vibrate on top of the active base mode (constant / spring).
    This is an overlay and does not change the base mode. Send amplitude 0
    (or frequency 0) to stop the vibration.
    Input:
        int frequency   - vibration frequency in Hz
        float amplitude - vibration torque amplitude (0 = off)
        ResponsePacket* recieved - pointer for the response packet
            - if NULL, will not block waiting for a response (fire-and-forget)
            - if not null, filled with the response; only valid when true is returned
    Returns: true if a valid response was received (or NULL was passed,
            meaning we did not wait); false if the response was invalid/timed out
    */
    bool vibrate(int frequency, float amplitude, Response_Packet* recieved);

    /*
    Updates haptic behavior based on normalized finger readings [0,1].
    When calibrate_trigger() has been called, automatically ceases actuation
    at the open/close limits and resumes when the trigger moves back in range.
    resp_out: if non-null, filled with the STM32 response from the force command.
    */
    bool update_force_slip(float normalized_force, float normalized_slip,
                           Response_Packet* resp_out = nullptr);

    /*
    Auto-calibrates the trigger open and close limits by driving the motor
    to each mechanical stop and detecting when the encoder stabilizes.
    cal_force: torque magnitude used during calibration (default 0.5).
    Returns true on success; false if either limit is not reached within 5 s.
    After a successful call, update_force_slip will enforce the limits.
    */
    bool calibrate_trigger(float cal_force = 0.5f);

    bool  is_calibrated()   const { return calibrated; }
    float get_limit_open()  const { return LIMIT_OPEN; }
    float get_limit_close() const { return LIMIT_CLOSE; }
    // Dead zone / ramp / power-off only applies at LIMIT_OPEN (the limit the
    // trigger is driven towards). LIMIT_CLOSE is the natural resting position
    // and needs no protection.
    bool  at_limit(float encoder_pos) const {
        if (!calibrated) return false;
        return encoder_pos >= LIMIT_OPEN - LIMIT_THRESHOLD;
    }
    bool  still_at_limit(float encoder_pos) const {
        if (!calibrated) return false;
        return encoder_pos >= LIMIT_OPEN - (LIMIT_THRESHOLD + LIMIT_HYSTERESIS);
    }
    float limit_approach_scale(float encoder_pos) const {
        if (!calibrated) return 1.0f;
        float dist = (LIMIT_OPEN - LIMIT_THRESHOLD) - encoder_pos;
        if (dist <= 0.0f)            return 0.0f;
        if (dist >= LIMIT_RAMP_ZONE) return 1.0f;
        float t = dist / LIMIT_RAMP_ZONE;
        return (expf(LIMIT_RAMP_EXP * t) - 1.0f) / (expf(LIMIT_RAMP_EXP) - 1.0f);
    }

private:
    HardwareSerial* SerialConnection;         // pointer to serial connection to STM board

    float amplitude = 0.5;
    int current_frequency = 0;
    float current_force = 1.5;

    int MIN_FREQ = 20;
    int MAX_FREQ = 90;
    float MIN_FORCE_OUT = 0.2;
    float MAX_FORCE_OUT = 1.5;

    // kg force measurement
    float MIN_FORCE_IN = 0.0;
    float MAX_FORCE_IN = 46.1;

    // slip rates
    float MIN_SLIP = 0;
    float MAX_SLIP = 50;
    float SLIP_DEAD_ZONE = 10;

    // trigger calibration
    float LIMIT_OPEN      = 0.0f;
    float LIMIT_CLOSE     = 0.0f;
    float LIMIT_THRESHOLD  =  0.01f;  // dead-zone entry size (degrees) at each limit
    float LIMIT_HYSTERESIS =  3.0f;  // extra clearance required to exit the limit state
    float LIMIT_RAMP_ZONE  =  15.0f;  // degrees over which force tapers to 0 before the threshold
    float LIMIT_RAMP_EXP   =   3.0f;  // exponential curve steepness (higher = more convex), max = 88.7
    bool  calibrated      = false;
    float last_encoder    = 0.0f;  // encoder value from the most recent update_force_slip call

    /*
    Casts and sends the given packet with Serial.write()
    Input: packet - the Command Packet to send
    Returns: N/A
    */
    void send_packet(Command_Packet packet);

    /*
    Sends a command packet and optionally waits for a response.
    Shared implementation behind the public command methods.
    Input:
        packet - the Command Packet to send
        recieved - pointer for the response packet
            - if NULL, will not block waiting for a response (fire-and-forget)
            - if not null, filled with the response; only valid when true is returned
    Returns: true if a valid response was received (or NULL was passed,
             meaning we did not wait); false if the response was invalid/timed out
    */
    bool send_command(const Command_Packet& packet, Response_Packet* recieved);

    /*
    Recieves a response packet and casts / returns it to the user
    Input: 
        bool* valid - returns true or false if response packet is valid
            or ignored if set to NULL
        unsigned long timeout - max time to block for
    Returns: response packet that was read
    */
    Response_Packet read_response(bool* valid, unsigned long timeout);

    // Drives the motor at `force` until the encoder stabilizes; returns the
    // stable encoder value, or 999999.0f if the timeout (5 s) is reached.
    float find_calibration_limit(float force);

    /*
    normalizes input force to be between 0 and 1
    as determined by MIN_FORCE_IN and MAX_FORCE_IN
    */
    float normalize_force_in(float force_kg)
    {
        float norm = (force_kg - MIN_FORCE_IN) / (MAX_FORCE_IN - MIN_FORCE_IN);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        return norm;
    }

    /*
    normalize input slip to be between 0 and 1
    as determiened by MIN_SLIP and MAX_SLIP
    */
    float normalize_slip_in(float slip_rate)
    {
        float norm = (slip_rate - MIN_SLIP) / (MAX_SLIP - MIN_SLIP);
        if (norm < 0.0f) norm = 0.0f;
        if (norm > 1.0f) norm = 1.0f;
        return norm;
    }

};



#endif