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
    Updates haptic behavior based on finger readings
    */
    bool update_force_slip(float normalized_force, float normalized_slip)
    {
        // Force: [0,1] → [MIN_FORCE_OUT, MAX_FORCE_OUT], sent as resistance (negative)
        float force_out = MIN_FORCE_OUT + normalized_force * (MAX_FORCE_OUT - MIN_FORCE_OUT);

        // Slip: dead zone then inverse-linear from MAX_FREQ down to MIN_FREQ
        float dead_zone_norm = SLIP_DEAD_ZONE / (MAX_SLIP - MIN_SLIP);
        int freq_out;
        if (normalized_slip <= dead_zone_norm) {
            freq_out = 0;
        } else {
            float t = (normalized_slip - dead_zone_norm) / (1.0f - dead_zone_norm);
            freq_out = (int)(MAX_FREQ - t * (MAX_FREQ - MIN_FREQ));
        }

        bool ok = true;
        if (freq_out != current_frequency) {
            current_frequency = freq_out;
            ok &= vibrate(freq_out, amplitude, NULL);
        }
        current_force = force_out;
        ok &= set_constant_force(-force_out, NULL);
        return ok;
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