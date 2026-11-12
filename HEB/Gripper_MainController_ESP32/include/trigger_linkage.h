#ifndef TRIGGER_LINKAGE_H
#define TRIGGER_LINKAGE_H

#include <math.h>

// Four-bar dimensions in mm, from the closed-position drawing. Coordinates:
// motor shaft (0,0), trigger shaft (ground,0), positive angles CCW.
// Closed crank angle is dimensioned in the supplied CAD image.
struct TriggerLinkage
{
    static constexpr float motor_arm = 40.0f;
    static constexpr float ground = 105.767f;
    static constexpr float rod = 122.576f;
    static constexpr float trigger_arm = 44.265f;
    static constexpr float closed_motor_deg = -108.011f;
    static constexpr float rad = 0.017453292519943295f;

    // Encoder increases toward release; both cranks rotate CCW in the image.
    // Select the lower circle intersection to retain the pictured assembly.
    static bool solve(float motor_delta, float* angle, float* jacobian)
    {
        if (!isfinite(motor_delta)) return false;
        const float a = (closed_motor_deg + motor_delta) * rad;
        const float ax = motor_arm * cosf(a), ay = motor_arm * sinf(a);
        const float dx = ground - ax, dy = -ay;
        const float d = hypotf(dx, dy);
        if (!(d > fabsf(rod - trigger_arm) && d < rod + trigger_arm)) return false;
        const float along = (rod * rod - trigger_arm * trigger_arm + d * d) / (2 * d);
        const float height = sqrtf(fmaxf(0.0f, rod * rod - along * along));
        const float bx = ax + (along * dx + height * dy) / d;
        const float by = ay + (along * dy - height * dx) / d;
        const float beta = atan2f(by, bx - ground);
        const float lx = bx - ax, ly = by - ay;
        const float denom = trigger_arm * (-lx * sinf(beta) + ly * cosf(beta));
        if (fabsf(denom) < 1e-3f) return false;
        const float j = motor_arm * (-lx * sinf(a) + ly * cosf(a)) / denom;
        // Reject toggles, reversed motion and extreme transmission ratios.
        if (!isfinite(j) || j < 0.1f || j > 4.0f) return false;
        *angle = beta / rad;
        *jacobian = j;  // d(trigger angle)/d(motor angle)
        return true;
    }

    float closed = 0, travel = 0, beta_closed = 0, beta_open = 0, j_closed = 1;

    bool begin(float close_pos, float open_pos)
    {
        closed = close_pos;
        travel = open_pos - close_pos;
        if (!isfinite(close_pos) || !isfinite(travel) || travel <= 0) return false;
        float previous = -360, beta = 0, j = 0;
        for (int i = 0; i <= 128; ++i) {
            if (!solve(travel * i / 128, &beta, &j) || beta <= previous) return false;
            if (i == 0) { beta_closed = beta; j_closed = j; }
            previous = beta;
        }
        beta_open = beta;
        return beta_open > beta_closed;
    }

    bool sample(float motor_pos, float* opening, float* force_scale) const
    {
        if (!isfinite(motor_pos)) return false;
        const float delta = fminf(travel, fmaxf(0.0f, motor_pos - closed));
        float beta, j;
        if (!solve(delta, &beta, &j)) return false;
        *opening = fminf(1.0f, fmaxf(0.0f, (beta - beta_closed) / (beta_open - beta_closed)));
        // Virtual work: tau_motor = tau_trigger * d(beta)/d(alpha).
        // Retain the existing c-to-N calibration at the CLOSED pose. This
        // corrects relative transmission; absolute finger force needs a
        // measured reference force/contact lever arm.
        *force_scale = j / j_closed;
        return true;
    }

    // Signed STM32 command: negative requests push toward release. Assistance
    // is in motor units, positive toward release, as in FrictionMap.
    bool motor_command(float requested, float motor_pos, float assistance,
                       float limit, float* command) const
    {
        float opening, scale;
        if (!isfinite(requested) || !isfinite(assistance) ||
            !isfinite(limit) || limit <= 0 || !sample(motor_pos, &opening, &scale))
            return false;
        *command = fminf(limit, fmaxf(-limit, requested * scale - assistance));
        return true;
    }
};

#endif
