// Host harness for SlipCore: reads samples (6 comma-separated floats per
// line) on stdin, prints one line per slip update:
//   <sample_index>,<db T1..T6>,<avgL>,<avgR>,<slip>
// Build (from tools/slip_equivalence): g++ -O2 -I../../include slip_harness.cpp ../../src/slip_core.cpp -o slip_harness
#include <cstdio>
#include <cstdlib>
#include "slip_core.h"

int main(int argc, char** argv)
{
    SlipConfig cfg;
    if (argc > 1) cfg.threshold    = (float)atof(argv[1]);
    if (argc > 2) cfg.contact_gate = (float)atof(argv[2]);

    static SlipCore core;   // ~25 KB: keep it off the stack
    SlipResult r;
    float x[SlipCore::CH];
    long i = 0;
    while (scanf("%f,%f,%f,%f,%f,%f", &x[0], &x[1], &x[2], &x[3], &x[4], &x[5]) == 6) {
        if (core.push(x, cfg, &r))
            printf("%ld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d\n", i,
                   r.band_db[0], r.band_db[1], r.band_db[2], r.band_db[3], r.band_db[4], r.band_db[5],
                   r.avgL, r.avgR, r.slip);
        i++;
    }
    return 0;
}
