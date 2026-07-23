#pragma once

// Wire protocol shared between the ESP32 main controller and the
// Arduino Uno / DynamixelShield gripper actuator board.
//
// Packet framing: every packet begins with DYN_START_BYTE and ends with
// DYN_END_BYTE. Floats are bit-cast through uint32_t via memcpy so the
// fractional bits survive the wire unchanged across compilers.
//
// IMPORTANT: this file must be kept identical in both projects:
//   Gripper_MainController_ESP32/include/dynamixel_protocol.h
//   DynamixelController/DynamixelController/include/dynamixel_protocol.h

#include <stdint.h>

#define DYN_START_BYTE 'S'
#define DYN_END_BYTE   'E'

// Commands sent from the ESP32 to the Arduino Uno.
typedef enum {
    DYN_PING            = 0,
    DYN_MOVE_SERVO      = 1,  // move + send ACK response
    DYN_STOP            = 2,
    DYN_MOVE_SERVO_FAST = 3   // move only, no response — use in high-rate control loops
} Dyn_Command;

// Responses sent from the Arduino Uno back to the ESP32.
typedef enum {
    DYN_ACK       = 0,  // command accepted and forwarded to servos
    DYN_PONG      = 1,  // reply to DYN_PING
    DYN_FAILURE   = 2,  // Dynamixel did not acknowledge the command
    DYN_MALFORMED = 3   // packet framing error
} Dyn_Response;

#pragma pack(push, 1)

// ESP32 -> Arduino Uno  (19 bytes)
typedef struct {
    char     startByte;      // DYN_START_BYTE
    uint8_t  command;        // Dyn_Command
    uint32_t left_degrees;   // float, bit-cast via memcpy
    uint32_t right_degrees;  // float, bit-cast via memcpy
    uint32_t left_speed;     // float (RPM), bit-cast via memcpy; 0 = max speed
    uint32_t right_speed;    // float (RPM), bit-cast via memcpy; 0 = max speed
    char     endByte;        // DYN_END_BYTE
} Dyn_Command_Packet;        // total: 1+1+(4*4)+1 = 19 bytes

// Arduino Uno -> ESP32  (11 bytes)
typedef struct {
    char     startByte;           // DYN_START_BYTE
    uint8_t  response;            // Dyn_Response
    uint32_t left_motor_current;  // float (mA), bit-cast via memcpy
    uint32_t right_motor_current; // float (mA), bit-cast via memcpy
    char     endByte;             // DYN_END_BYTE
} Dyn_Response_Packet;            // total: 1+1+(2*4)+1 = 11 bytes

#pragma pack(pop)
