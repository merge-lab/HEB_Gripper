# Slip detector equivalence check (host)

Verifies that the ESP32 slip detector (`src/slip_core.cpp`) produces the same
results as FORTE's Python reference (`forte/runtime/force_and_slip.py` in
[merge-lab/FORTE](https://github.com/merge-lab/FORTE)), so FORTE's slip
threshold means the same thing on the gripper.

```bash
cd tools/slip_equivalence
g++ -O2 -I../../include slip_harness.cpp ../../src/slip_core.cpp -o /tmp/slip_harness
git clone --depth 1 https://github.com/merge-lab/FORTE.git /tmp/FORTE
python3 compare_forte.py --harness /tmp/slip_harness \
    --forte-src /tmp/FORTE/forte/runtime/force_and_slip.py [--threshold 2]
```

Needs `numpy`. The script runs synthetic 2 kHz data (noise, slip-like bursts on
both fingers, a one-finger burst, contact steps) through both and reports the
largest differences and any slip / contact-gate decision mismatches (expected: 0;
band power agrees to ~0.005 dB, the ESP32 using single-precision floats).
