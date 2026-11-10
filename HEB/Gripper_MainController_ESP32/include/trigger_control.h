#ifndef TRIGGER_CONTROL_H
#define TRIGGER_CONTROL_H

#include <Arduino.h>
#include "haptic_motor_control.h"
#include "dynamixel_control.h"
#include "trigger_friction.h"

// Haptic trigger friction: the 'fid' identification run on the real
// hardware, the stored friction map (kept in flash, loaded at boot), and the
// constants for the friction-aware force floor used by tmt.

// Floor = friction to return the trigger at this speed, plus a margin
// (capped at FrictionMap::FLOOR_MAX).
const float TRIGGER_RETURN_DEG_S = 20.0f;
const float TRIGGER_FLOOR_MARGIN = 0.03f;

// tm/tmt smooth start: the force fades in over this long (smoothstep), so
// the trigger doesn't jump to its release end when a mode starts.
const unsigned long TRIGGER_SOFT_START_MS = 500;

// tmt return-speed limit: above this speed toward release, the push is
// eased off by 0.5 x the measured viscous friction per deg/s (needs a map).
const float TRIGGER_MAX_RETURN_DEG_S = 100.0f;

// 0 -> 1 over TRIGGER_SOFT_START_MS, smooth at both ends.
inline float trigger_soft_start_gain(unsigned long ms_since_start)
{
    const float x = ms_since_start / (float)TRIGGER_SOFT_START_MS;
    return x >= 1.0f ? 1.0f : x * x * (3.0f - 2.0f * x);
}

// Load the stored map (if any). Call once from setup().
void trigger_control_begin();

// The current map (valid == false until an identification has run).
const FrictionMap& trigger_friction_map();

// 'fid': find the ends if needed, measure friction, store and print the map.
// Blocking (several minutes). Hands off the trigger. 'e' + Enter aborts.
bool trigger_friction_id(HapticMotorController* motor, DynamixelController* gripper);

// 'fid?': print the stored map as FRICMAP / FRICBIN / FRICEND lines.
void trigger_friction_print();

// 'fid_clear': forget the stored map.
void trigger_friction_clear();

#endif
