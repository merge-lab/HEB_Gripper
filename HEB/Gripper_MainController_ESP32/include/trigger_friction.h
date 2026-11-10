#ifndef TRIGGER_FRICTION_H
#define TRIGGER_FRICTION_H

#include <stdint.h>

// Haptic trigger friction identification and the friction-aware force floor.
// Plain C++ (no Arduino): the same code runs on the ESP32 and in the host
// simulation test (tools/trigger_friction_sim/).
//
// Force convention: u > 0 pushes the trigger toward its RELEASE end (higher
// encoder position), the direction the haptic force normally acts - the same
// sense as the CLI 'c' value (the STM32 command is -u).
//
// Positions in the map are distances from the release end, d = open - pos
// (deg). The trigger encoder is incremental and re-zeroes when the STM32
// powers up, but the release end is found again every session, so a map in
// d stays valid across restarts.
//
// Method:
//   1. In each direction, ramp the force slowly until the trigger starts to
//      move: the breakaway force.
//   2. Sweep end to end (end_margin from each stop) at low target speeds,
//      `cycles` times each. Adjust force from measured speed so low-friction
//      regions do not cause fast sweeps. The force is cut at the far point
//      so friction stops the trigger before the mechanical stop.
//   3. For each direction and fine-grained position bin, fit the measured
//      force as force = f0(pos) + b(pos) * speed. The result is a position x
//      velocity friction map: friction offset and viscous slope vary as the
//      trigger moves through its travel.

struct FrictionMap
{
    static constexpr int   MAX_BINS  = 128;
    // Known upper bound of this trigger's friction (c units): the force floor
    // is never set above it, whatever the measurement says.
    static constexpr float FLOOR_MAX = 0.3f;

    float bin_w   = 1.0f;     // deg per bin
    int   n       = 0;        // bins covering d = 0 .. travel
    float travel  = 0.0f;     // deg between the closed and release ends
    float b_open  = 0.0f;     // viscous part, c per deg/s, moving toward release
    float b_close = 0.0f;     //                            moving toward closed
    float f_open[MAX_BINS];   // force to keep moving toward release as v -> 0 (c)
    float f_close[MAX_BINS];  // force to keep moving toward closed  as v -> 0 (c)
    bool  valid   = false;

    float open_at(float d) const;     // f_open at distance d (interpolated, clamped)
    float close_at(float d) const;

    // Force needed to return the trigger at v_return deg/s, plus margin,
    // limited to 0 .. FLOOR_MAX.
    float floor_at(float d, float v_return, float margin) const;
};

// What the identification needs from the hardware.
class TriggerIO
{
public:
    virtual ~TriggerIO() {}
    // Command force u (see convention); read back position (deg) and the
    // measured motor force (Iq) in the same sign convention. false = link error.
    virtual bool exchange(float u, float* pos_deg, float* force_meas) = 0;
    virtual uint32_t now_us() = 0;
    virtual bool abort_requested() = 0;
    virtual void log(const char* msg) = 0;
    // Called after every exchange (the IO decides how often to stream it).
    virtual void sample(float pos_deg, float u, float force_meas) = 0;
};

struct FrictionIdConfig
{
    float end_margin    = 2.0f;   // keep sweeps this far from each end (deg)
    float u_max         = 0.75f;  // never command more than this
    float breakaway_max = 0.35f;  // friction is <= FLOOR_MAX; no motion by here = blocked
    float ramp_rate     = 0.25f;  // c per second, breakaway search
    float v_break       = 2.0f;   // deg/s that counts as "moving"
    float sweep_speed[3] = { 5.0f, 10.0f, 15.0f }; // deg/s, slow friction measurements
    float speed_kp      = 0.004f; // c per deg/s, speed feedback
    float speed_ki      = 0.04f;  // c per deg, integral speed feedback
    int   cycles        = 3;      // back-and-forth sweeps per level
    float pass_timeout  = 30.0f;  // s minimum per sweep; extended for slow/long travel
};

// Runs the identification between the closed end (close_pos) and the release
// end (open_pos). Blocking; always ends with u = 0. Returns false if aborted
// or it failed; *out is valid only on true.
bool run_friction_id(TriggerIO& io, float close_pos, float open_pos,
                     const FrictionIdConfig& cfg, FrictionMap* out);

#endif
