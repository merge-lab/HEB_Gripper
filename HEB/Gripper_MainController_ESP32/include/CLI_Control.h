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
// Starts at constant force 0.75 (same sign convention as CLI c).
// While mapping: 'c <force>' changes force, 'v <hz> <amp>' overlays vibration
// (0 Hz or amplitude 0 disables it). Type 'e' + Enter to relax and exit.
// Accepts CR/LF line endings, or processes input after the Serial idle timeout.
// tactile_force = true is 'tmt' mode: same position mapping, but the
// constant force follows the tactile sensors every loop: tared sums
// T1+T2+T3 (left finger) and T4+T5+T6 (right finger), each clamped to 0..3;
// the larger, s, maps logarithmically to c = 0.25 + 1.85 ln(1 + s/2.2),
// clamped 0.25..1.30 (s 0.5 -> 2 N, s 1 -> 3 N). Prints machine-readable
// "TMT,..." lines for heb_console. 'tare' re-zeroes the sensors.
void trigger_mapping(HapticMotorController* motor, DynamixelController* gripper,
                     bool tactile_force = false);

// Same as trigger_mapping(), but reads back and logs actual servo position
// each cycle (slower). Pairs with response_plot.py for expected-vs-actual
// logging/plotting. Type 'e' + Enter to exit.
void trigger_mapping_log(HapticMotorController* motor, DynamixelController* gripper);

// Manual haptic update loop. Input '<norm_force> <norm_slip>' to set values.
// 'd' toggles Dynamixel actuation from trigger, 'e' exits.
void haptic_update_mode(HapticMotorController* motor, DynamixelController* gripper);

// Range-of-motion finder. Turns servo torque off so the fingers can be moved
// by hand, then streams present L/R positions with running min/max.
// 'r' resets min/max, 'e' exits and prints the final range.
void gripper_position_mode(DynamixelController* gripper);

// Tactile-only mode: relaxes the haptic motor and the gripper, then prints
// T1..T6 (tared, -1..+1 scale) at out_hz: every sample when out_hz >= the
// sample rate (2 kHz), otherwise the block average of each group of samples.
// 'tare' re-zeroes, 'e' + Enter exits.
void tactile_mode(HapticMotorController* motor, DynamixelController* gripper, int out_hz = 100);

// Watched move: streams the goals (rejected if out of range) through the
// driver's motion shaper capped at rpm (<= 0: full shaper speed), printing
// actual vs goal every 100 ms when verbose, until reached, stalled or 20 s.
// 'e' stops. The servos' own speed setting is not changed.
void gripper_move_watch(DynamixelController* gripper, float id0_deg, float id1_deg, float rpm,
                        bool verbose = true);

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
  m <id0> <id1> [rpm]  move ID0 (phys. right) / ID1 (phys. left), full speed by default;
                       out of range = rejected + relaxed; prints the result
  mw <id0> <id1> [rpm] same, slow by default (5 rpm), prints actual vs goal
  gstop              torque off both servos
  gping              ping the Arduino gripper board
  gpos               torque off + stream positions (find range of motion)

Tactile / slip (FORTE detector runs on-board at 2 kHz, see slip_core.h):
  tac [hz]           stream T1..T6 (100 Hz default, 2000 = every sample) + SLIP lines
  tmt                trigger map + tactile-driven force (+ SLIP lines)
  tare               re-zero the tactile sensors (any mode)
  slip_thr <v>       slip threshold (FORTE slip_var_threshold, default 2)
  slip_gate <v>      contact gate (both fingers must reach it, default 0.6)
  slip?              print SLIPCFG,<thr>,<gate> and detector load

Trigger friction (trigger_control.h / trigger_friction.h):
  fid                measure friction at descending constant forces (hands off,
                     several minutes); saved to flash; tmt uses a return floor
                     and tm compensates friction in either direction (try c 0)
  fid?               print the stored map (FRICMAP / FRICBIN / FRICEND)
  fid_clear          forget the stored map
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
