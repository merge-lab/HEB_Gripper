#pragma once

// Dynamixel-gripper command layer, riding on top of the generic SPI
// transfer_packet/local_packet from Communication_Protocol.h (command byte +
// data1..data4 uint32_t fields, 17 bytes on the wire). All numeric fields are
// floats bit-cast through uint32_t via memcpy (both platforms little-endian,
// so a raw copy preserves the value across compilers).
//
// IMPORTANT: this file must be kept identical in both projects:
//   Gripper_MainController_ESP32/include/dynamixel_protocol.h
//   DynamixelController/DynamixelController/include/dynamixel_protocol.h
//
// Inherited convention (Communication_Protocol.h):
//   command == 0       -> no-op / dummy pull, ignore data fields
//   command == CMD_ACK -> bare receipt-confirmation ack; data1..4 unused.
//                         Staged by the slave immediately upon receiving any
//                         real command, so the *next* transfer to that slave
//                         reads it back. It confirms the physical link/clock
//                         are set up correctly and the slave was triggered --
//                         nothing more. It carries no positional data.
// This header adds application command values >= 2.

#include "Communication_Protocol.h"

// ---- ESP32 -> Arduino ----------------------------------------------------

// Move both servos to an absolute goal (degrees). Retargets in-flight if
// already moving toward a previous goal - inherent to Protocol 2.0, no
// explicit "abort" logic needed.
//   data1 = left_degrees    (float)
//   data2 = right_degrees   (float)
//   data3 = left_speed_rpm  (float; 0 = no override, keep current profile velocity)
//   data4 = right_speed_rpm (float; 0 = no override)
#define CMD_DXL_MOVE  2

// Torque off both servos immediately.
//   data1..data4 unused (send 0)
#define CMD_DXL_STOP  3

// Link check / no-op probe. No dedicated reply - the fast ACK (see above) is
// the only response.
//   data1..data4 unused (send 0)
#define CMD_DXL_PING  4

// ---- Arduino -> ESP32 (async, via notify_data / get_data) ----------------

// Settle/telemetry packet. Sent exactly once per settled goal, once both
// servos report MOVING == false. Re-armed whenever a new CMD_DXL_MOVE
// changes the goal, even if the previous goal was never reached.
//   data1 = goal_left_degrees    (float; the goal last commanded, echoed back)
//   data2 = goal_right_degrees   (float)
//   data3 = actual_left_degrees  (float; present position, converted to degrees)
//   data4 = actual_right_degrees (float)
// Motor-current telemetry from the old UART protocol is intentionally
// dropped here in favor of position. Re-adding it later is a new command
// reusing this same generic 4-field shape, not a protocol change.
#define CMD_DXL_SETTLED  5
