// Runs strack_fusion over a stream of events on stdin, for checking it against a recording:
//   I t ax ay az            an IMU reading (ns; m/s2 in the world, gravity out), as it arrives
//   O t px py pz leds rms   a fit (exposure time in ns), as it arrives
//   R t px py pz leds rms   a better fit for an exposure already given (strack_fusion_refine)
//   P t                     where is it at t? prints "t x y z" or "t -"
//
//   cc -O2 -I src tests/sense_fusion_replay.c src/sense_track.c -lm -o build/sense_fusion_replay

#include "sense_track.h"

#include <stdio.h>
#include <stdlib.h>

int main(void) {
    static strack_fusion f;
    strack_fusion_init(&f);
    if (getenv("BLEND")) f.blend_s = (float)atof(getenv("BLEND"));
    if (getenv("SA")) f.accel_noise = (float)atof(getenv("SA"));
    if (getenv("SSCALE")) f.sigma_scale = (float)atof(getenv("SSCALE"));
    if (getenv("GATE")) f.gate = (float)atof(getenv("GATE"));
    if (getenv("GSIG")) f.gate_sigma = (float)atof(getenv("GSIG"));
    if (getenv("VSIG")) f.start_v_sigma = (float)atof(getenv("VSIG"));
    if (getenv("VMAX")) f.max_speed = (float)atof(getenv("VMAX"));
    if (getenv("LAG")) f.imu_lag_s = (float)atof(getenv("LAG"));
    if (getenv("RFITS")) f.restart_fits = atoi(getenv("RFITS"));
    if (getenv("RMISSES")) f.restart_misses = atoi(getenv("RMISSES"));
    if (getenv("RLEDS")) f.restart_leds = atoi(getenv("RLEDS"));
    if (getenv("COAST")) sscanf(getenv("COAST"), "%f %f %f %f", &f.coast_after_s, &f.coast_tau_s, &f.coast_accel, &f.coast_noise);
    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        long long t;
        float v[3], rms;
        int leds;
        if (line[0] == 'I' && sscanf(line + 1, "%lld %f %f %f", &t, &v[0], &v[1], &v[2]) == 4) {
            strack_fusion_imu(&f, t, v);
        } else if (line[0] == 'O' && sscanf(line + 1, "%lld %f %f %f %d %f", &t, &v[0], &v[1], &v[2], &leds, &rms) == 6) {
            unsigned r0 = f.restarts, a0 = f.accepted;
            bool had = f.main.have;
            float before[3] = {0};
            for (int k = 0; k < 3 && had; k++) before[k] = f.main.x[k][0];
            strack_fusion_optical(&f, t, v, leds, rms);
            if (getenv("VERBOSE") && f.restarts != r0)
                printf("# %s %lld at %.3f %.3f %.3f from %.3f %.3f %.3f leds %d\n", had ? "restart" : "start", t, v[0], v[1],
                       v[2], before[0], before[1], before[2], leds);
            else if (getenv("VERBOSE") && f.accepted == a0)
                printf("# leftout %lld %.3f %.3f %.3f leds %d\n", t, v[0], v[1], v[2], leds);
        } else if (line[0] == 'R' && sscanf(line + 1, "%lld %f %f %f %d %f", &t, &v[0], &v[1], &v[2], &leds, &rms) == 6) {
            strack_fusion_refine(&f, t, v, leds, rms);
        } else if (line[0] == 'P' && sscanf(line + 1, "%lld", &t) == 1) {
            float p[3];
            if (strack_fusion_predict(&f, t, 250000000, p, NULL)) printf("%lld %.5f %.5f %.5f\n", t, p[0], p[1], p[2]);
            else printf("%lld -\n", t);
        }
    }
    fprintf(stderr, "fits taken in %u, left out %u, restarts %u, refined %u\n", f.accepted, f.rejected, f.restarts, f.refined);
    return 0;
}
