#include "trigger_linkage.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>

static void near(float actual, float expected, float tolerance)
{
    assert(fabsf(actual - expected) < tolerance);
}

int main()
{
    float beta, j;
    assert(TriggerLinkage::solve(0, &beta, &j));
    // Independent CAD observations, rounded to 0.001 deg in the drawing.
    near(180 + beta, 95.567f, 0.003f);
    const float ax = 40 * cosf(-108.011f * TriggerLinkage::rad);
    const float ay = 40 * sinf(-108.011f * TriggerLinkage::rad);
    const float bx = 105.767f + 44.265f * cosf(beta * TriggerLinkage::rad);
    const float by = 44.265f * sinf(beta * TriggerLinkage::rad);
    const float rod_angle = atan2f(by - ay, bx - ax) / TriggerLinkage::rad;
    near(71.989f - rod_angle, 74.803f, 0.003f);
    near(rod_angle - beta, 81.619f, 0.003f);
    near(hypotf(bx - ax, by - ay), 122.576f, 0.001f);

    for (float travel : {47.0f, 55.0f}) {
        TriggerLinkage m, offset;
        assert(m.begin(-17, -17 + travel));
        assert(offset.begin(500, 500 + travel));
        float prev = -1, t, scale, t2, scale2;
        for (int i = 0; i <= 100; ++i) {
            const float delta = travel * i / 100;
            assert(m.sample(-17 + delta, &t, &scale));
            assert(t > prev && t >= 0 && t <= 1);
            assert(offset.sample(500 + delta, &t2, &scale2));
            near(t, t2, 1e-5f);
            near(scale, scale2, 1e-5f);
            assert(TriggerLinkage::solve(delta, &beta, &j));
            const float a = (TriggerLinkage::closed_motor_deg + delta) * TriggerLinkage::rad;
            // Check rigid rod closure independently of circle construction.
            const float x = 105.767f + 44.265f * cosf(beta * TriggerLinkage::rad) - 40 * cosf(a);
            const float y = 44.265f * sinf(beta * TriggerLinkage::rad) - 40 * sinf(a);
            near(hypotf(x, y), 122.576f, 0.001f);
            float left, right, unused;
            assert(TriggerLinkage::solve(delta - 0.02f, &left, &unused));
            assert(TriggerLinkage::solve(delta + 0.02f, &right, &unused));
            near(j, (right - left) / 0.04f, 0.003f);
            // Constant requested trigger torque gives constant virtual work.
            near(scale / j, 1 / m.j_closed, 1e-5f);
            prev = t;
        }
        assert(m.sample(-1000, &t, &scale)); near(t, 0, 1e-6f); near(scale, 1, 1e-6f);
        assert(m.sample(1000, &t, &scale)); near(t, 1, 1e-6f);
        assert(!m.sample(NAN, &t, &scale));
        assert(!m.sample(INFINITY, &t, &scale));
        std::printf("motor %.2f deg -> trigger %.3f deg; open force scale %.3f\n",
                    travel, m.beta_open - m.beta_closed, scale);
    }
    TriggerLinkage invalid;
    assert(!invalid.begin(10, 10));
    assert(!invalid.begin(10, -10));
    assert(!invalid.begin(0, 100));  // toggle/unreachable branch
    assert(!invalid.begin(NAN, 47));
    assert(!invalid.begin(0, INFINITY));
    assert(!TriggerLinkage::solve(NAN, &beta, &j));
    std::puts("Linkage tests passed.");
}
