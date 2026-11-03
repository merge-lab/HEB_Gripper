// Host test for run_friction_id(): a simulated trigger with known friction.
// Build (from tools/trigger_friction_sim):
//   g++ -std=c++17 -O2 -I../../include sim_test.cpp ../../src/trigger_friction.cpp -o sim_test
#include <cmath>
#include <cstdio>
#include <random>
#include "trigger_friction.h"

namespace {

constexpr float CLOSE = 10.0f, OPEN = 50.0f;      // ends (deg)
constexpr float B_TRUE = 0.003f;                  // viscous, c per deg/s
constexpr float PULL = 0.02f;                     // constant pull toward release (spring-like)
constexpr float J = B_TRUE * 0.02f;               // inertia: 20 ms mechanical time constant

bool g_sticky = false;                            // add a sticky spot at 30..33 deg

float coulomb(float x)                            // sliding friction along the travel, <= 0.3
{
    const float t = (x - CLOSE) / (OPEN - CLOSE);
    float f = 0.12f + 0.06f * std::sin(2.0f * float(M_PI) * t) + 0.03f * t;
    if (g_sticky && x > 30.0f && x < 33.0f) f += 0.10f;   // well above the first sweep level
    return f;
}

class SimTrigger : public TriggerIO
{
public:
    explicit SimTrigger(float x0) : x_(x0) {}

    bool exchange(float u, float* pos, float* force) override
    {
        advance(0.0026);                // request in flight: old command still applies
        cmd_ = u;
        advance(0.0026);                // reply: state at the end of the exchange
        *pos = std::round(x_ / 0.0055f) * 0.0055f;       // encoder resolution
        *force = filt_ + noise_(rng_);                   // Iq with noise
        return true;
    }
    uint32_t now_us() override { return (uint32_t)(t_ * 1e6); }
    bool abort_requested() override { return false; }
    void log(const char* msg) override { std::printf("  %s\n", msg); }
    void sample(float, float, float) override { samples_++; }

    double t_ = 0;
    long samples_ = 0;

private:
    float x_, v_ = 0, cmd_ = 0, filt_ = 0;
    double next_stm_ = 0;
    std::mt19937 rng_{1};
    std::normal_distribution<float> noise_{0.0f, 0.005f};

    void advance(double dt)
    {
        const double dts = 1e-4;
        for (double e = 0; e < dt; e += dts) {
            t_ += dts;
            if (t_ >= next_stm_) {      // STM32 1 kHz torque LPF
                filt_ += 0.1f * (cmd_ - filt_);
                next_stm_ += 0.001;
            }
            const float drive = filt_ + PULL;
            const float fc = coulomb(x_), fs = 1.2f * fc;     // static 20% above sliding
            if (std::fabs(v_) < 1e-3f && std::fabs(drive) <= fs) { v_ = 0; continue; }
            const float dir = std::fabs(v_) > 1e-3f ? (v_ > 0 ? 1.0f : -1.0f) : (drive > 0 ? 1.0f : -1.0f);
            const float a = (drive - dir * fc - B_TRUE * v_) / J;
            const float v_new = v_ + a * (float)dts;
            v_ = (v_ != 0 && (v_ > 0) != (v_new > 0)) ? 0.0f : v_new;   // friction can't reverse motion
            x_ += v_ * (float)dts;
            if (x_ < CLOSE) { x_ = CLOSE; v_ = 0; }
            if (x_ > OPEN)  { x_ = OPEN;  v_ = 0; }
        }
    }
};

} // namespace

int main()
{
    int failures = 0;
    struct Case { float x0; bool sticky; };
    for (Case cs : { Case{OPEN, false}, Case{CLOSE, false}, Case{0.5f * (CLOSE + OPEN), false},
                     Case{OPEN, true} }) {
        const float x0 = cs.x0;
        g_sticky = cs.sticky;
        std::printf("start at %.1f deg%s:\n", x0, g_sticky ? ", with a sticky spot at 30-33 deg" : "");
        SimTrigger sim(x0);
        FrictionIdConfig cfg;
        FrictionMap m;
        const bool ok = run_friction_id(sim, CLOSE, OPEN, cfg, &m);
        if (!ok) { std::printf("  FAILED\n"); failures++; continue; }

        if (g_sticky) {                 // show the map across the sticky spot
            std::printf("  x(deg)  true->rel  map->rel   true->cl  map->cl\n");
            for (float x = 27.5f; x <= 35.5f; x += 1.0f)
                std::printf("  %5.1f   %.3f      %.3f      %.3f     %.3f\n", x, coulomb(x) - PULL,
                            m.open_at(OPEN - x), coulomb(x) + PULL, m.close_at(OPEN - x));
        }
        float worst_open = 0, worst_close = 0;
        for (float d = cfg.end_margin + 3.5f; d <= OPEN - CLOSE - cfg.end_margin - 3; d += 1.0f) {
            const float x = OPEN - d;
            if (g_sticky && x > 28.5f && x < 34.5f) continue;   // a 1-deg bin can't resolve a step
            worst_open  = std::fmax(worst_open,  std::fabs(m.open_at(d)  - (coulomb(x) - PULL)));
            worst_close = std::fmax(worst_close, std::fabs(m.close_at(d) - (coulomb(x) + PULL)));
        }
        float floor_max = 0;
        for (float d = 0; d <= m.travel; d += 0.5f) floor_max = std::fmax(floor_max, m.floor_at(d, 20, 0.03f));
        std::printf("  took %.1f s, %ld exchanges; b_open %.4f b_close %.4f (true %.4f)\n",
                    sim.t_, sim.samples_, m.b_open, m.b_close, B_TRUE);
        std::printf("  max |friction error| (away from the ends): toward release %.3f, toward closed %.3f\n",
                    worst_open, worst_close);
        std::printf("  floor (20 deg/s + 0.03) max %.3f (cap %.2f)\n", floor_max, FrictionMap::FLOOR_MAX);
        if (worst_open > 0.03f || worst_close > 0.03f || floor_max > FrictionMap::FLOOR_MAX + 1e-6f) failures++;
    }
    std::printf(failures ? "RESULT: %d FAILED\n" : "RESULT: all passed\n", failures);
    return failures;
}
