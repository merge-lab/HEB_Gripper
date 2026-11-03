#ifndef SLIP_CORE_H
#define SLIP_CORE_H

// FORTE slip detector, ported from forte/runtime/force_and_slip.py
// (merge-lab/FORTE: sensor_data_updater + slip_predictor). Plain C++ with no
// Arduino dependencies, so the same code runs on the ESP32 and in the host
// equivalence test against FORTE's Python (tools/slip_equivalence/).
//
// Per channel (T1..T6), with samples arriving at FS:
//   1. 11-sample rolling median.
//   2. Every UPDATE_EVERY samples (500 Hz), on the newest NPERSEG samples:
//      remove the mean, apply a Hann window (numpy np.hanning, symmetric),
//      take the one-sided PSD, and keep the max (in dB) over BAND_LO..BAND_HI.
//   3. Keep the last VAR_WINDOW of those values. If they don't rise by more
//      than MONO_STEP_DB every step, the channel scores 0; otherwise it
//      scores their (population) variance.
//   4. avgL = mean score of T1..T3, avgR = mean of T4..T6. If either is below
//      contact_gate, both are zeroed (FORTE: single-finger suppression).
//   5. slip = avgL > threshold || avgR > threshold.
// FORTE's defaults: threshold 2 (slip_var_threshold), contact_gate 0.6.
//
// Cost per update (6 channels): 6 x (400-sample window + 9 Goertzel bins).
// The baseline subtraction FORTE applies before the median is omitted: the
// window mean is removed anyway, so a constant offset can't change the result.

struct SlipConfig
{
    float threshold    = 2.0f;
    float contact_gate = 0.6f;
};

struct SlipResult
{
    float avgL = 0.0f, avgR = 0.0f;
    int   slip = 0;
    float band_db[6] = { 0 };   // newest per-channel band max (dB)
};

class SlipCore
{
public:
    static constexpr int   CH           = 6;
    static constexpr int   FS           = 2000;
    static constexpr int   NPERSEG      = 400;
    static constexpr int   UPDATE_EVERY = 4;      // 2000 / 4 = 500 updates/s
    static constexpr int   VAR_WINDOW   = 15;
    static constexpr int   MEDIAN_LEN   = 11;
    static constexpr float BAND_LO      = 10.0f;
    static constexpr float BAND_HI      = 50.0f;
    static constexpr float MONO_STEP_DB = 0.1f;
    static_assert(NPERSEG % 2 == 0, "goertzel3 processes two samples per step");

    SlipCore();
    void reset();

    // Feed one sample (CH channels). Returns true when a new slip result was
    // computed into *out (every UPDATE_EVERY samples, once the history is full).
    bool push(const float x[CH], const SlipConfig& cfg, SlipResult* out);

private:
    static constexpr int MAX_BINS = 30;           // multiple of 3 (bins run in triples)

    float band_max_db(int ch);

    // Rolling median input
    float med_buf_[CH][MEDIAN_LEN];
    int   med_count_ = 0, med_pos_ = 0;

    // Median-filtered window (ring, oldest at win_pos_)
    float win_[CH][NPERSEG];
    int   win_count_ = 0, win_pos_ = 0;

    // Band-max history (ring, oldest at hist_pos_)
    float hist_[CH][VAR_WINDOW];
    int   hist_count_ = 0, hist_pos_ = 0;

    int   since_update_ = 0;

    // Precomputed constants and scratch
    float hann_[NPERSEG];
    float seg_[NPERSEG];          // windowed segment being analysed
    float psd_scale_;             // 2 / (FS * sum(hann^2))
    int   n_bins_;                // bins inside the band
    float coef_[MAX_BINS];        // Goertzel 2*cos(2*pi*k/N), padded to triples
};

#endif
