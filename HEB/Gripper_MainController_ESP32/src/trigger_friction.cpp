#include "trigger_friction.h"

#include <math.h>
#include <stdio.h>

// ---- FrictionMap ------------------------------------------------------------

static float interp(const float* v, int n, float bin_w, float d)
{
    if (n <= 0) return 0.0f;
    const float x = d / bin_w - 0.5f;               // bin centres at (i + 0.5) * bin_w
    if (x <= 0.0f) return v[0];
    if (x >= n - 1) return v[n - 1];
    const int   i = (int)x;
    const float t = x - i;
    return v[i] + t * (v[i + 1] - v[i]);
}

float FrictionMap::open_at(float d) const  { return interp(f_open,  n, bin_w, d); }
float FrictionMap::close_at(float d) const { return interp(f_close, n, bin_w, d); }

float FrictionMap::floor_at(float d, float v_return, float margin) const
{
    if (!valid) return 0.0f;
    const float f = open_at(d) + b_open * v_return + margin;
    return f < 0.0f ? 0.0f : (f > FLOOR_MAX ? FLOOR_MAX : f);
}

float FrictionMap::compensation_at(float d, float velocity) const
{
    if (!valid || !isfinite(d) || !isfinite(velocity)) return 0.0f;
    const float speed = fabsf(velocity);
    // Encoder noise must not choose a direction at rest. Fade assistance in
    // over 2 deg/s, with a small residual drag to avoid overcompensation.
    const float gain = 0.95f * fminf(1.0f, fmaxf(0.0f, (speed - 0.3f) / 2.0f));
    const float friction = velocity > 0.0f ? open_at(d) + b_open * speed
                                           : close_at(d) + b_close * speed;
    const float assist = gain * fminf(FLOOR_MAX, fmaxf(0.0f, friction));
    return velocity > 0.0f ? assist : -assist;
}

// ---- Identification ---------------------------------------------------------

namespace {

// Per-bin sums for the force-vs-speed fit.
struct BinAccum
{
    float n = 0, sv = 0, sf = 0, svv = 0, svf = 0;
    void add(float v, float f) { n += 1; sv += v; sf += f; svv += v * v; svf += v * f; }
};

// [0] = moving toward release, [1] = toward closed. File scope (~2.5 KB)
// rather than on the caller's stack; one identification runs at a time.
BinAccum g_acc[2][FrictionMap::MAX_BINS];

class Runner
{
public:
    Runner(TriggerIO& io, const FrictionIdConfig& cfg, float close_pos, float open_pos)
        : io_(io), cfg_(cfg), close_(close_pos), open_(open_pos) {}

    bool run(FrictionMap* out);

private:
    enum PassResult { REACHED, STALLED, FAILED };

    TriggerIO&              io_;
    const FrictionIdConfig& cfg_;
    const float close_, open_;

    float pos_ = 0, vel_ = 0, force_ = 0;
    uint32_t t_prev_ = 0;
    bool  have_ = false, failed_ = false;

    // Previous two samples, for the lag-free (centred-difference) speed used
    // in the fit. vel_ is low-pass filtered (lags ~15 ms) and only drives the
    // stall / breakaway decisions.
    struct Sample { float pos, force; uint32_t t; bool valid; };
    Sample s1_ = {0, 0, 0, false}, s2_ = {0, 0, 0, false};   // s1_ newest

    BinAccum (&acc_)[2][FrictionMap::MAX_BINS] = g_acc;
    int   n_bins_ = 0;
    float bin_w_  = 1.0f;

    void say(const char* fmt, float a = 0, float b = 0, float c = 0)
    {
        char buf[160];
        snprintf(buf, sizeof(buf), fmt, a, b, c);
        io_.log(buf);
    }

    float elapsed_s(uint32_t since) const { return (uint32_t)(io_.now_us() - since) * 1e-6f; }

    // One command/measurement exchange; updates position and filtered speed.
    bool step(float u)
    {
        if (failed_) return false;
        if (io_.abort_requested()) { io_.log("[FID] aborted."); failed_ = true; return false; }
        float p, f;
        if (!io_.exchange(u, &p, &f)) { io_.log("[FID] haptic link error."); failed_ = true; return false; }
        const uint32_t t = io_.now_us();
        if (have_) {
            const float dt = (uint32_t)(t - t_prev_) * 1e-6f;
            if (dt > 0.0f) vel_ += 0.3f * ((p - pos_) / dt - vel_);
        } else {
            vel_ = 0.0f;
            have_ = true;
        }
        pos_ = p; force_ = f; t_prev_ = t;
        io_.sample(pos_, u, force_);
        return true;
    }

    bool hold(float u, float seconds)
    {
        const uint32_t t0 = io_.now_us();
        while (elapsed_s(t0) < seconds)
            if (!step(u)) return false;
        return true;
    }

    // Ramp the force in direction dir until the trigger moves; returns the
    // breakaway force (> 0), or < 0 on failure. Optionally leave it moving
    // so a lower sliding-force probe can follow without another start.
    float breakaway(int dir, bool stop = true)
    {
        const uint32_t t0 = io_.now_us();
        int moving = 0;
        float u = 0.0f;
        while (true) {
            u = cfg_.ramp_rate * elapsed_s(t0);
            if (u > cfg_.breakaway_max) {
                say("[FID] trigger did not move below force %.2f (friction should be <= %.2f): "
                    "is it blocked or being held?", cfg_.breakaway_max, FrictionMap::FLOOR_MAX);
                failed_ = true;
                return -1.0f;
            }
            if (!step(dir * u)) return -1.0f;
            moving = (dir * vel_ > cfg_.v_break) ? moving + 1 : 0;
            if (moving >= 3) break;
        }
        if (stop && !hold(0.0f, 0.2f)) return -1.0f;
        return u;
    }

    // Constant force throughout the pass: speed is measured, never controlled.
    PassResult pass(int dir, float level, float target, int rec_dir)
    {
        const float start = pos_;
        const uint32_t t0 = io_.now_us();
        uint32_t still_since = 0;
        s1_.valid = s2_.valid = false;
        while (dir > 0 ? pos_ < target : pos_ > target) {
            if (!step(dir * level)) return FAILED;
            const float v = dir * vel_;                // filtered speed in the sweep direction
            // Record the PREVIOUS sample with its centred-difference speed
            // (from the samples either side of it).
            if (rec_dir >= 0 && s1_.valid && s2_.valid) {
                const float dt = (uint32_t)(t_prev_ - s2_.t) * 1e-6f;
                const float vc = dt > 0.0f ? dir * (pos_ - s2_.pos) / dt : 0.0f;
                if (vc > 0.5f && fabsf(s1_.pos - start) > 1.0f) {
                    const int b = (int)((open_ - s1_.pos) / bin_w_);
                    if (b >= 0 && b < n_bins_) acc_[rec_dir][b].add(vc, dir * s1_.force);
                }
            }
            s2_ = s1_;
            s1_ = { pos_, force_, t_prev_, true };
            if (v < 0.5f) {
                if (still_since == 0) still_since = io_.now_us();
                else if (elapsed_s(still_since) > 0.6f) {
                    if (!hold(0.0f, 0.15f)) return FAILED;
                    return STALLED;
                }
            } else {
                still_since = 0;
            }
            if (elapsed_s(t0) > cfg_.pass_timeout) {
                if (rec_dir >= 0) {
                    if (!hold(0.0f, 0.15f)) return FAILED;
                    return STALLED; // a very slow low-force probe is also a limit
                }
                io_.log("[FID] sweep timed out.");
                failed_ = true;
                return FAILED;
            }
        }
        if (!hold(0.0f, 0.15f)) return FAILED;   // coast: friction stops it before the end
        return REACHED;
    }

    // Sweep to target, raising the force a little if it stalls (up to 3 times).
    bool sweep(int dir, float* level, float target, int rec_dir)
    {
        for (int tries = 0; tries < 4; tries++) {
            const PassResult r = pass(dir, *level, target, rec_dir);
            if (r == REACHED) return true;
            if (r == FAILED) return false;
            *level += 0.05f;
            if (*level > cfg_.u_max) break;
            say("[FID] stalled at %.1f deg; force raised to %.2f.", pos_, *level);
        }
        io_.log("[FID] could not complete a sweep.");
        failed_ = true;
        return false;
    }

    bool analyse(FrictionMap* out);
};

bool Runner::run(FrictionMap* out)
{
    if (cfg_.levels < 2 || !(cfg_.force_decay > 0.0f && cfg_.force_decay < 1.0f) ||
        !(cfg_.start_mult >= 1.0f) || !isfinite(cfg_.start_mult) ||
        !(cfg_.force_min > 0.0f && cfg_.force_min <= cfg_.u_max)) {
        io_.log("[FID] invalid descending-force configuration.");
        return false;
    }
    const float travel = open_ - close_;
    const float A = close_ + cfg_.end_margin;       // closed-side turnaround
    const float B = open_  - cfg_.end_margin;       // release-side turnaround
    if (!(travel > 10.0f) || B - A < 6.0f) {
        say("[FID] travel %.1f deg is too short (closed %.1f, release %.1f).", travel, close_, open_);
        return false;
    }
    bin_w_  = travel / FrictionMap::MAX_BINS;
    if (bin_w_ < 0.5f) bin_w_ = 0.5f;
    n_bins_ = (int)ceilf(travel / bin_w_);
    if (n_bins_ > FrictionMap::MAX_BINS) n_bins_ = FrictionMap::MAX_BINS;

    for (int d = 0; d < 2; d++)
        for (int b = 0; b < FrictionMap::MAX_BINS; b++) acc_[d][b] = BinAccum();

    if (!hold(0.0f, 0.3f)) return false;
    say("[FID] travel %.1f deg; sweeping between %.1f and %.1f deg.", travel, A, B);

    // Breakaway forces: always ramp AWAY from the nearer end first (the
    // trigger may be resting against either stop), then warm-up sweeps, so
    // the measured sweeps start at A.
    float ub_open = -1.0f, ub_close = -1.0f, lv;
    float warm_open = 0.0f, warm_close = 0.0f;
    if (pos_ > 0.5f * (A + B)) {
        io_.log("[FID] finding breakaway force toward closed...");
        if ((ub_close = breakaway(-1)) < 0) return false;
        lv = ub_close;
        if (pos_ > A && !sweep(-1, &lv, A, -1)) return false;
        warm_close = lv;
        io_.log("[FID] finding breakaway force toward release...");
        if ((ub_open = breakaway(+1)) < 0) return false;
        lv = ub_open;
        if (!sweep(+1, &lv, B, -1)) return false;
        warm_open = lv;
    } else {
        io_.log("[FID] finding breakaway force toward release...");
        if ((ub_open = breakaway(+1)) < 0) return false;
        lv = ub_open;
        if (pos_ < B && !sweep(+1, &lv, B, -1)) return false;
        warm_open = lv;
        io_.log("[FID] finding breakaway force toward closed...");
        if ((ub_close = breakaway(-1)) < 0) return false;
    }
    lv = ub_close;
    if (!sweep(-1, &lv, A, -1)) return false;
    warm_close = fmaxf(warm_close, lv);
    say("[FID] breakaway: toward release %.3f, toward closed %.3f.", ub_open, ub_close);

    // Descend independently in each direction. A stalled low-force probe is
    // useful data, not a failed calibration. Reposition with a known working
    // force (unrecorded), then try the next, lower measurement force.
    float lo = fmaxf(cfg_.force_min, fminf(cfg_.u_max, warm_open * cfg_.start_mult));
    float lc = fmaxf(cfg_.force_min, fminf(cfg_.u_max, warm_close * cfg_.start_mult));
    for (int k = 0; k < cfg_.levels; k++) {
        say("[FID] descending level %.0f: force %.3f toward release, %.3f toward closed.", k + 1, lo, lc);
        for (int c = 0; c < cfg_.cycles; c++) {
            for (int dir = 1; dir >= -1; dir -= 2) {
                const float target = dir > 0 ? B : A;
                const float level = dir > 0 ? lo : lc;
                // Below breakaway, start moving with an unrecorded force ramp,
                // then immediately switch to the lower, constant test force.
                if (breakaway(dir, false) < 0) return false;
                say("[FID] probe force %.4f.", dir * level);
                const PassResult result = pass(dir, level, target, dir > 0 ? 0 : 1);
                if (result == FAILED) return false;
                if (result == STALLED) {
                    say("[FID] low-force probe stalled at %.1f deg (force %.3f).", pos_, level);
                    float& recovery = dir > 0 ? warm_open : warm_close;
                    if (!sweep(dir, &recovery, target, -1)) return false;
                }
            }
        }
        lo = fmaxf(cfg_.force_min, lo * cfg_.force_decay);
        lc = fmaxf(cfg_.force_min, lc * cfg_.force_decay);
    }
    if (!hold(0.0f, 0.2f)) return false;
    return analyse(out);
}

bool Runner::analyse(FrictionMap* out)
{
    FrictionMap m;
    m.bin_w = bin_w_;
    m.n = n_bins_;
    m.travel = open_ - close_;

    for (int dir = 0; dir < 2; dir++) {
        // Viscous slope pooled over bins (within-bin regression), robust to
        // friction varying with position.
        double sxy = 0, sxx = 0;
        for (int b = 0; b < n_bins_; b++) {
            const BinAccum& a = acc_[dir][b];
            if (a.n < 5) continue;
            sxy += a.svf - (double)a.sv * a.sf / a.n;
            sxx += a.svv - (double)a.sv * a.sv / a.n;
        }
        float slope = sxx > 1e-6 ? (float)(sxy / sxx) : 0.0f;
        if (slope < 0.0f) slope = 0.0f;
        if (slope > 0.05f) slope = 0.05f;
        (dir == 0 ? m.b_open : m.b_close) = slope;

        float* f0 = dir == 0 ? m.f_open : m.f_close;
        bool ok[FrictionMap::MAX_BINS];
        int valid = 0;
        for (int b = 0; b < n_bins_; b++) {
            const BinAccum& a = acc_[dir][b];
            ok[b] = a.n >= 5;
            f0[b] = ok[b] ? (a.sf - slope * a.sv) / a.n : 0.0f;
            valid += ok[b];
        }
        if (valid < 3) {
            say(dir == 0 ? "[FID] too little data moving toward release (%.0f bins)."
                         : "[FID] too little data moving toward closed (%.0f bins).", (float)valid);
            return false;
        }
        // Bins without data (near the ends): copy the nearest measured bin.
        for (int b = 0; b < n_bins_; b++) {
            if (ok[b]) continue;
            int best = -1;
            for (int r = 1; r < n_bins_ && best < 0; r++) {
                if (b - r >= 0 && ok[b - r]) best = b - r;
                else if (b + r < n_bins_ && ok[b + r]) best = b + r;
            }
            f0[b] = f0[best];
        }
    }
    float worst = 0.0f;
    for (int b = 0; b < n_bins_; b++)
        if (m.f_open[b] > worst) worst = m.f_open[b];
    if (worst > FrictionMap::FLOOR_MAX + 0.05f)
        say("[FID] warning: measured friction up to %.2f, above the expected max %.2f - was the "
            "trigger touched? The floor is capped at %.2f anyway.", worst, FrictionMap::FLOOR_MAX,
            FrictionMap::FLOOR_MAX);

    m.valid = true;
    *out = m;
    return true;
}

} // namespace

bool run_friction_id(TriggerIO& io, float close_pos, float open_pos,
                     const FrictionIdConfig& cfg, FrictionMap* out)
{
    Runner r(io, cfg, close_pos, open_pos);
    const bool ok = r.run(out);
    float p, f;
    io.exchange(0.0f, &p, &f);            // always finish with no force
    return ok;
}
