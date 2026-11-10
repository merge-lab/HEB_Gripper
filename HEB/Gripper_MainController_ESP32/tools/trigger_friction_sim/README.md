# Trigger friction identification - host simulation

Runs the ESP32's friction identification (`src/trigger_friction.cpp`, the
`fid` command) against a simulated trigger with known friction: friction that
varies along the travel, a small pull toward release, static friction 20 %
above sliding, viscous friction, inertia, the STM32's torque filter, the
5.2 ms command exchange, encoder resolution and current noise. Includes a
case with a sticky spot. Sweeps use speed feedback with targets of 5, 10 and
15 deg/s; warm-up sweeps use 5 deg/s. A 40-degree trigger takes roughly
100 seconds to characterize at these settings.

```bash
cd tools/trigger_friction_sim
g++ -std=c++17 -O2 -I../../include sim_test.cpp ../../src/trigger_friction.cpp -o /tmp/sim_test
/tmp/sim_test
```

Passes if the identified friction is within 0.03 (c units) of the true value
away from the ends and step edges, and the force floor never exceeds
`FrictionMap::FLOOR_MAX` (0.3). Also checks that mean interior speed at each
level is within 25% of its target, including the sticky-spot case.
