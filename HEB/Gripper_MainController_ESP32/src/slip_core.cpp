#include "slip_core.h"

#include <math.h>

namespace {

// Median of n <= 16 values (insertion sort of a copy; n is 11 here).
float median_of(const float* v, int n)
{
    float a[16];
    for (int i = 0; i < n; i++) {
        float x = v[i];
        int j = i - 1;
        while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; }
        a[j + 1] = x;
    }
    return (n & 1) ? a[n / 2] : 0.5f * (a[n / 2 - 1] + a[n / 2]);
}

// |Y[k]|^2 of x[0..n) for three DFT bins at once (Goertzel:
// s[i] = x[i] + c*s[i-1] - s[i-2]). Three independent recurrences side by side
// keep the FPU pipeline busy, and taking two samples per iteration lets the
// two state registers swap roles instead of being copied (no mov.s in the
// loop on the S3). n must be even.
void goertzel3(const float* x, int n, const float* c, float* power)
{
    float a1 = 0, a2 = 0, b1 = 0, b2 = 0, d1 = 0, d2 = 0;   // *1 = s[i-1], *2 = s[i-2]
    const float ca = c[0], cb = c[1], cd = c[2];
    for (int i = 0; i < n; i += 2) {
        const float v0 = x[i], v1 = x[i + 1];
        a2 = v0 + ca * a1 - a2;   b2 = v0 + cb * b1 - b2;   d2 = v0 + cd * d1 - d2;   // s[i]
        a1 = v1 + ca * a2 - a1;   b1 = v1 + cb * b2 - b1;   d1 = v1 + cd * d2 - d1;   // s[i+1]
    }
    power[0] = a1 * a1 + a2 * a2 - ca * a1 * a2;
    power[1] = b1 * b1 + b2 * b2 - cb * b1 * b2;
    power[2] = d1 * d1 + d2 * d2 - cd * d1 * d2;
}

} // namespace

SlipCore::SlipCore()
{
    // np.hanning(N): symmetric, w[n] = 0.5 - 0.5 cos(2 pi n / (N - 1)).
    double u = 0.0;
    for (int n = 0; n < NPERSEG; n++) {
        hann_[n] = (float)(0.5 - 0.5 * cos(2.0 * M_PI * n / (NPERSEG - 1)));
        u += (double)hann_[n] * hann_[n];
    }
    // One-sided PSD for interior bins: 2 * |Y|^2 / (fs * U).
    psd_scale_ = (float)(2.0 / (FS * u));

    // DFT bins k with k * FS / N inside [BAND_LO, BAND_HI], inclusive (FORTE's mask).
    const double df = (double)FS / NPERSEG;
    const int k_lo = (int)ceil(BAND_LO / df - 1e-9);
    const int k_hi = (int)floor(BAND_HI / df + 1e-9);
    n_bins_ = k_hi - k_lo + 1;
    if (n_bins_ > MAX_BINS) n_bins_ = MAX_BINS;
    for (int b = 0; b < MAX_BINS; b++) {
        // Pad the last triple by repeating the top bin (harmless for a max).
        const int k = k_lo + (b < n_bins_ ? b : n_bins_ - 1);
        coef_[b] = (float)(2.0 * cos(2.0 * M_PI * k / NPERSEG));
    }

    reset();
}

void SlipCore::reset()
{
    med_count_ = med_pos_ = 0;
    win_count_ = win_pos_ = 0;
    hist_count_ = hist_pos_ = 0;
    since_update_ = 0;
}

float SlipCore::band_max_db(int ch)
{
    // Chronological window, mean removed, Hann applied. (float, not double:
    // the S3's FPU is single-precision, double math runs in software.)
    const float* w = win_[ch];
    float sum = 0.0f;
    for (int n = 0; n < NPERSEG; n++) sum += w[n];
    const float mean = sum / NPERSEG;
    int idx = win_pos_;                                   // oldest sample
    for (int n = 0; n < NPERSEG; n++) {
        seg_[n] = (w[idx] - mean) * hann_[n];
        if (++idx == NPERSEG) idx = 0;
    }

    // Max power over the band; log10 is monotonic, so take it once at the end.
    float best = 0.0f, p[3];
    for (int b = 0; b < n_bins_; b += 3) {
        goertzel3(seg_, NPERSEG, &coef_[b], p);
        for (int j = 0; j < 3; j++)
            if (p[j] > best) best = p[j];
    }
    return 10.0f * log10f(psd_scale_ * best + 1e-12f);
}

bool SlipCore::push(const float x[CH], const SlipConfig& cfg, SlipResult* out)
{
    // 1. Rolling median over up to the last MEDIAN_LEN samples. While it's
    //    filling, the first med_count_ slots hold exactly the samples so far.
    for (int c = 0; c < CH; c++) med_buf_[c][med_pos_] = x[c];
    if (++med_pos_ == MEDIAN_LEN) med_pos_ = 0;
    if (med_count_ < MEDIAN_LEN) med_count_++;

    for (int c = 0; c < CH; c++)
        win_[c][win_pos_] = median_of(med_buf_[c], med_count_);
    if (++win_pos_ == NPERSEG) win_pos_ = 0;
    if (win_count_ < NPERSEG) win_count_++;

    // 2. PSD update every UPDATE_EVERY samples once the window is full.
    if (win_count_ < NPERSEG) return false;
    if (++since_update_ < UPDATE_EVERY) return false;
    since_update_ = 0;

    for (int c = 0; c < CH; c++) {
        const float db = band_max_db(c);
        hist_[c][hist_pos_] = db;
        if (out) out->band_db[c] = db;
    }
    if (++hist_pos_ == VAR_WINDOW) hist_pos_ = 0;
    if (hist_count_ < VAR_WINDOW) hist_count_++;
    if (hist_count_ < VAR_WINDOW) return false;

    // 3. Per channel: variance of the history, but only if it rose steadily.
    float score[CH];
    for (int c = 0; c < CH; c++) {
        float h[VAR_WINDOW];
        int idx = hist_pos_;                              // oldest
        for (int i = 0; i < VAR_WINDOW; i++) {
            h[i] = hist_[c][idx];
            if (++idx == VAR_WINDOW) idx = 0;
        }
        bool rising = true;
        for (int i = 1; i < VAR_WINDOW && rising; i++)
            rising = (h[i] - h[i - 1]) > MONO_STEP_DB;
        if (!rising) { score[c] = 0.0f; continue; }

        float mean = 0.0f, var = 0.0f;
        for (int i = 0; i < VAR_WINDOW; i++) mean += h[i];
        mean /= VAR_WINDOW;
        for (int i = 0; i < VAR_WINDOW; i++) var += (h[i] - mean) * (h[i] - mean);
        score[c] = var / VAR_WINDOW;
    }

    // 4./5. Per-finger averages, contact gate, threshold.
    float avgL = (score[0] + score[1] + score[2]) / 3.0f;
    float avgR = (score[3] + score[4] + score[5]) / 3.0f;
    if (avgL < cfg.contact_gate || avgR < cfg.contact_gate)
        avgL = avgR = 0.0f;

    if (out) {
        out->avgL = avgL;
        out->avgR = avgR;
        out->slip = (avgL > cfg.threshold || avgR > cfg.threshold) ? 1 : 0;
    }
    return true;
}
