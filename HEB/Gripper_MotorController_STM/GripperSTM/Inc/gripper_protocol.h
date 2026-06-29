#ifndef GRIPPER_PROTOCOL_H
#define GRIPPER_PROTOCOL_H

// =============================================================================
// Shared wire protocol between the ESP32 main controller and the STM32 motor
// controller. This file is plain C/C++ so BOTH projects can include it.
//
//   ESP32 copy: Gripper_MainController_ESP32/include/gripper_protocol.h
//   STM32 copy: Gripper_MotorController_STM/GripperSTM/Inc/gripper_protocol.h
//
// The two projects build on separate toolchains and don't share an include
// path, so this file is duplicated. KEEP THE TWO COPIES IDENTICAL.
// =============================================================================

#include <stdint.h>

// pre-agreed framing bytes for all communications
#define START_BYTE 'A'
#define END_BYTE   'Z'

// Command types the main controller (ESP32) can send to the motor controller
typedef enum
{
    PING           = 0,   // ping and wait for response
    CONSTANT_FORCE = 1,   // ask motor to output constant force
    SPRING_FORCE   = 2,   // ask motor to act like a spring
    VIBRATE        = 3,   // add vibration on top of the current force
    RELAX          = 4    // end all intentional force
} Command;

// Command packet standard
#pragma pack(push, 1)
typedef struct
{
    char     startByte;   // packet start
    uint8_t  command;     // a Command value (fixed width so layout can't depend on enum size)
    uint32_t param_1;     // params, cast as makes sense for the given command
    uint32_t param_2;     // expect 0 if no param
    uint32_t param_3;
    uint32_t param_4;
    uint32_t param_5;
    uint32_t param_6;
    char     endByte;     // packet end
} Command_Packet;
#pragma pack(pop)

// Possible device (STM32) responses
typedef enum
{
    ACK            = 0,   // received and recognized
    MALFORMED      = 1,   // command packet malformed
    NOT_RECOGNIZED = 2,   // packet well formed but command not recognized
    WRONG_PARAMS   = 3,   // params don't make sense for command or wrong count
    INVALID        = 4,   // recognized but cannot be run in the current state
    PONG           = 5    // ACK for a ping request
} Device_Responses;

#pragma pack(push, 1)
typedef struct
{
    char             startByte;     // packet start
    uint8_t          response;      // a Device_Responses value (fixed width, see above)
    uint32_t         encoder_value; // current encoder reading, cast as needed
    uint32_t         motor_value;   // current value being sent to motor, cast as needed
    char             endByte;       // packet end
} Response_Packet;
#pragma pack(pop)

#endif
