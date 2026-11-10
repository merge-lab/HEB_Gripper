# Trigger friction identification - host simulation

Runs the ESP32's friction identification (`src/trigger_friction.cpp`, the
`fid` command) against a simulated trigger with known friction: friction that
varies along the travel, a small pull toward release, static friction 20 %
above sliding, viscous friction, inertia, the STM32's torque filter, the
5.2 ms command exchange, encoder resolution and current noise. Includes a
case with a sticky spot. Identification uses 12 descending constant-force
levels, each 15% lower than the previous one, with three cycles per level.
Each probe starts with an unrecorded breakaway ramp. Stalled probes recover
at a known working force without adding recovery samples to the map. There
is no speed controller. Expect several minutes for a full calibration.

```bash
cd tools/trigger_friction_sim
g++ -std=c++17 -O2 -I../../include sim_test.cpp ../../src/trigger_friction.cpp -o /tmp/sim_test
/tmp/sim_test
```

Passes if the identified friction is within 0.03 (c units) of the true value
away from the ends and step edges, and the force floor never exceeds
`FrictionMap::FLOOR_MAX` (0.3). Also checks descending force levels,
constant force within each probe, final zero force, and bounded directional
compensation with zero assistance at rest.

After flashing, run `fid` with hands off, then enter `tm` and send `c 0` to
try compensated free motion. Assistance follows the measured direction of
motion, uses 95% of estimated friction (capped at 0.3), and fades out below
2.3 deg/s, reaching zero at 0.3 deg/s. This reduces moving drag without a
return bias; it cannot anticipate the intended direction before motion.
Idle `c 0` remains a raw zero motor-force command. `tmt` retains its return
floor behavior.
