#ifndef CLI_CONTROL_H
#define CLI_CONTROL_H

#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"

enum TestMode { MODE_SPRING, MODE_CONSTANT };

// Frequencies (Hz) stepped through in auto-sweep mode.
const int SWEEP_FREQS[] = {10, 20, 30, 50, 75, 100, 150, 200};
const int SWEEP_COUNT   = sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]);
const int SWEEP_HOLD    = 8;   // ~200 ms loop cycles to hold each step (~1.6 s)

/*
Used to setup CLI testing mode, put into void setup
*/
// gripper may be nullptr if the Dynamixel link is not wired up yet.
void CLI_Setup(HapticMotorController* motor, DynamixelController* gripper = nullptr);

// Maps trigger position to gripper open/close angle in a continuous loop.
// Haptic runs at constant force 1.5. Type 'e' + Enter to exit.
void trigger_mapping(HapticMotorController* motor, DynamixelController* gripper);

// Manual haptic update loop. Input '<norm_force> <norm_slip>' to set values.
// 'd' toggles Dynamixel actuation from trigger, 'e' exits.
void haptic_update_mode(HapticMotorController* motor, DynamixelController* gripper);

/*
Serial CLI tool for sending and testing commands

Haptic motor:
  s <k>        spring base (stiffness)
  c <t>        constant-force base (torque, signed)
  n <N>        constant force in Newtons (lateral tip force)
  f <hz>       set vibration frequency
  a <amp>      set vibration amplitude (0 = off)
  v <hz> <amp> set vibration freq + amp together
  sweep        toggle auto frequency sweep
  rampup       test: ramp constant force 0.0 -> 1.5 by 0.1
  stop         relax: zero all force (also: x / relax)

Gripper (Dynamixel):
  m <l> <r>          move to l/r degrees at default speed
  m <l> <r> <spd>    move to l/r degrees at spd RPM
  gstop              torque off both servos
  gping              ping the Arduino gripper board
*/
void CLI(HapticMotorController* motor, DynamixelController* gripper = nullptr);

/*
CLI Tool with debugging prints in case vibrate fails
*/
void applyVibrate(int freq, float amp, HapticMotorController* motor);

/*
Test routine: ramp constant force from 0.0 up to 1.5 in 0.1 steps, holding each
level briefly and printing the response. Triggered by the 'rampup' command.
Leaves the motor relaxed when done.
*/
void force_ramp_up(HapticMotorController* motor);

#endif