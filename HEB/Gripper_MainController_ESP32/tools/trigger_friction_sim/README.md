# Trigger friction identification - host simulation

Runs the ESP32's friction identification (`src/trigger_friction.cpp`, the
`fid` command) against a simulated trigger with known friction: friction that
varies along the travel, a small pull toward release, static friction 20 %
above sliding, viscous friction, inertia, the STM32's torque filter, the
5.2 ms command exchange, encoder resolution and current noise. Includes a
cases with a sticky spot and position-dependent linkage preload.
Identification uses 12 descending constant-force
levels, each 15% lower than the previous one, with three cycles per level.
Each probe starts with an unrecorded breakaway ramp. Stalled probes recover
at a known working force without adding recovery samples to the map. There
is no speed controller. Expect several minutes for a full calibration.

```bash
cd tools/trigger_friction_sim
g++ -std=c++17 -O2 -I../../include sim_test.cpp ../../src/trigger_friction.cpp -o /tmp/sim_test
/tmp/sim_test
g++ -std=c++17 -O2 -Wall -Wextra -Werror -I../../include linkage_test.cpp -o /tmp/linkage_test
/tmp/linkage_test
```

Passes if the identified friction is within 0.03 (c units) of the true value
away from the ends and step edges, and the force floor never exceeds
`FrictionMap::FLOOR_MAX` (0.3). Also checks descending force levels,
constant force within each probe, final zero force, and bounded signed
compensation through reversal. Synthetic maps check preload cancellation
when the preload exceeds sliding friction in either direction.

After flashing, run `fid` with hands off, then enter `tm` and send `c 0` to
try compensated free motion. Compensation separates the directional maps:
`load = (open - close)/2`, `sliding = (open + close)/2`. The estimated load
is cancelled even at rest; only sliding assistance fades out below 2.3 deg/s,
reaching zero at 0.3 deg/s. Compensation uses 95% of the estimate and is
capped at +/-0.3. Existing stored maps support this decomposition.

Both `tm` and `tmt` use this signed compensation. In `tmt`, the return floor
is applied once to the final motor force while stopped or moving toward
release, fading out smoothly as closing motion starts. The tactile force curve
still supplies its base resistance. Idle `c 0` remains a raw motor command.

This targets repeatable position-dependent loads, including linkage preload.
The decomposition assumes approximately symmetric sliding friction; asymmetric
friction can also appear in the inferred preload.
It does not identify mechanical advantage, backlash or reflected inertia.
`tmt` now separately models the four-bar in `include/trigger_linkage.h`:
motor crank 40 mm, fixed shaft spacing 105.767 mm, rod pin spacing 122.576 mm,
trigger crank 44.265 mm. Rod shape does not affect its pin-to-pin kinematics.
The dimensioned closed motor crank is -108.011 degrees relative to the line
from motor shaft to trigger shaft, with CCW positive. The resulting closed
trigger angle is approximately -84.433 degrees (95.567 degrees interior
angle at the trigger shaft). This angle reference is needed in addition to
the four lengths.

TMT finds motor encoder limits if necessary, anchors the drawing's closed pose
to the measured closed limit, and uses the pictured lower assembly branch.
Increasing motor encoder position is assumed to turn the pictured motor crank
CCW toward release, consistent with the existing control convention.
The measured open motor limit determines trigger travel; 45--60 degrees of
trigger opening is descriptive, not a hardcoded motor travel limit.
For example, 47 motor degrees produce about 48.54 trigger degrees.
Gripper opening is normalized in trigger angle instead of motor angle.
Incompatible ranges/toggles stop TMT with the motor relaxed.

Virtual work gives `tau_motor = tau_trigger * d(trigger)/d(motor)`.
The tactile resistance is multiplied by this derivative relative to its value
at closure. The existing 3.18 N/c conversion is treated as a nominal CLOSED
pose reference; the image alone cannot establish absolute finger force or
the finger's contact lever arm. The final motor command retains its 1.30 cap,
so high desired forces can saturate, and the STM32 still fades near its limit.
Friction/preload compensation is added afterward in motor units, without
geometry scaling. Friction identification, return-speed settings, `tm` and
`tml` remain in motor coordinates. Existing stored friction maps remain usable.
`TMT,pos` remains motor degrees; `t` is now normalized trigger opening and
`c` remains the final commanded motor value (after geometry, compensation,
clamping and startup fade). CSV columns are unchanged.
