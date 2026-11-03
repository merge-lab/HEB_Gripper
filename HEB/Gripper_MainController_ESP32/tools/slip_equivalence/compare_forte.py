#!/usr/bin/env python3
"""Check the ESP32 slip detector (src/slip_core.cpp) against FORTE's Python.

Generates synthetic 2 kHz, 6-channel tactile data (noise, slip-like bursts on
both fingers, a one-finger burst, contact steps), runs it through:
  * the C++ SlipCore via the host harness (slip_harness), and
  * FORTE's own functions from forte/runtime/force_and_slip.py,
fed sample-by-sample exactly like FORTE's sensor_data_updater (11-sample
rolling median) + slip_predictor (newest 400 samples, every 4 samples).
Reports the worst differences.

Usage:
  python3 compare_forte.py --harness ./slip_harness \
      --forte-src /path/to/FORTE/forte/runtime/force_and_slip.py
"""
import argparse
import subprocess
from collections import deque

import numpy as np

FS, NPERSEG, STEP, VAR_WINDOW, MEDIAN_LEN = 2000, 400, 4, 15, 11


def load_forte_functions(path):
    """Exec only FORTE's slip-math section (pure numpy) from its source file."""
    src = open(path).read()
    start = src.index("def welch(")
    end = src.index("# Helpers")
    ns = {"np": np}
    exec(compile(src[start:end], path, "exec"), ns)
    return ns


def synth(seed=0, seconds=12.0):
    rng = np.random.default_rng(seed)
    n = int(seconds * FS)
    t = np.arange(n) / FS
    x = 0.003 * rng.standard_normal((n, 6)) + rng.uniform(-0.2, 0.2, 6)

    def burst(t0, chans, f, amp=0.15, rise=0.08):
        # Vibration whose amplitude grows exponentially, then decays.
        env = np.where(t < t0, 0.0, np.where(t < t0 + rise,
                       np.exp((t - t0 - rise) / (rise / 5)), np.exp(-(t - t0 - rise) / 0.05)))
        for c in chans:
            x[:, c] += amp * env * np.sin(2 * np.pi * (f + 3 * c) * t)

    burst(2.0, range(6), 20)          # slip on both fingers
    burst(4.0, range(6), 35, 0.3)     # stronger slip
    burst(6.0, [3, 4, 5], 25)         # one finger only (FORTE gates this out)
    x[int(8.0 * FS):, :3] += 0.4      # contact step on the left finger
    burst(9.5, range(6), 15, 0.05, rise=0.12)   # weak, slow slip
    return x


def forte_reference(x, f, threshold):
    recent = np.zeros((MEDIAN_LEN, 6))
    r_idx, count = 0, 0
    filtered = np.zeros_like(x)
    psd_history = [deque(maxlen=VAR_WINDOW) for _ in range(6)]
    since = 0
    out = []
    for i, sample in enumerate(x):
        recent[r_idx] = sample
        r_idx = (r_idx + 1) % MEDIAN_LEN
        count += 1
        filtered[i] = np.median(recent[:count] if count < MEDIAN_LEN else recent, axis=0)
        if i + 1 < NPERSEG:
            continue
        since += 1
        if since < STEP:
            continue
        since = 0
        window = filtered[i + 1 - NPERSEG:i + 1]
        f["update_psd_history"](window, FS, NPERSEG, int(0.99 * NPERSEG), 10, 50, psd_history)
        if len(psd_history[0]) == VAR_WINDOW:
            slip, avgL, avgR = f["compute_slip_indicator"](psd_history, VAR_WINDOW, True, False, -72.0, threshold)
            db = [psd_history[c][-1] for c in range(6)]
            out.append((i, db, avgL, avgR, slip))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--harness", required=True)
    ap.add_argument("--forte-src", required=True)
    ap.add_argument("--threshold", type=float, default=2.0)
    args = ap.parse_args()

    f = load_forte_functions(args.forte_src)
    x = synth()
    csv = "\n".join(",".join(f"{v:.7f}" for v in row) for row in x) + "\n"
    res = subprocess.run([args.harness, str(args.threshold), "0.6"], input=csv,
                         capture_output=True, text=True, check=True)
    cpp = [line.split(",") for line in res.stdout.strip().splitlines()]
    ref = forte_reference(x, f, args.threshold)

    assert len(cpp) == len(ref), (len(cpp), len(ref))
    d_db = d_avg = 0.0
    slip_mismatch = 0
    gated_mismatch = 0
    for c, (i, db, aL, aR, s) in zip(cpp, ref):
        assert int(c[0]) == i
        d_db = max(d_db, max(abs(float(c[1 + k]) - db[k]) for k in range(6)))
        d_avg = max(d_avg, abs(float(c[7]) - aL), abs(float(c[8]) - aR))
        slip_mismatch += int(c[9]) != s
        gated_mismatch += (float(c[7]) > 0) != (aL > 0)
    n = len(ref)
    ref_slips = sum(r[4] for r in ref)
    cpp_slips = sum(int(c[9]) for c in cpp)
    print(f"updates compared: {n}  (500 per second of data)")
    print(f"max |band dB diff|: {d_db:.2e} dB")
    print(f"max |avgL/avgR diff|: {d_avg:.2e}")
    print(f"slip updates: FORTE {ref_slips}, ESP32 core {cpp_slips}, mismatched {slip_mismatch}")
    print(f"contact-gate decisions mismatched: {gated_mismatch}")
    # Which bursts were flagged (FORTE reference)
    events = sorted({round(r[0] / FS, 1) for r in ref if r[4]})
    print(f"FORTE flagged slip around t = {events} s")


if __name__ == "__main__":
    main()
