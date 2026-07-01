#ifndef CLI_CONTROL_H
#define CLI_CONTROL_H

#include <Arduino.h>
#include "haptic_motor_control.h"

enum TestMode { MODE_SPRING, MODE_CONSTANT };

// Frequencies (Hz) stepped through in auto-sweep mode.
const int SWEEP_FREQS[] = {10, 20, 30, 50, 75, 100, 150, 200};
const int SWEEP_COUNT   = sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]);
const int SWEEP_HOLD    = 8;   // ~200 ms loop cycles to hold each step (~1.6 s)

/*
Used to setup CLI testing mode, put into void setup
*/
void CLI_Setup(HapticMotorController* motor);

/*
Serial CLI tool for sending and testing commands
c 0.3 — set a steady push to feel the buzz against
a 0.2 — turn vibration on at amplitude 0.2
f 20, f 50, f 100, f 150 — step the frequency; amplitude stays put so you're comparing pitch at constant strength
a 0.4 then step f again — compare strength
sweep — hands-free walk through the frequency list; sweep again to stop
f 0 or a 0 — buzz off, base remains
*/
void CLI(HapticMotorController* motor);

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