// Runs strack_fusion over a stream of events on stdin, for checking it against a recording:
//   I t ax ay az            an IMU reading (ns; m/s2 in the world, gravity out), as it arrives
//   O t px py pz leds rms   a fit (exposure time in ns), as it arrives
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
    if (getenv("LAG")) f.imu_lag_s = (float)atof(getenv("LAG"));
    char line[256];
    while (fgets(line, sizeof line, stdin)) {
        long long t;
        float v[3], rms;
        int leds;
        if (line[0] == 'I' && sscanf(line + 1, "%lld %f %f %f", &t, &v[0], &v[1], &v[2]) == 4) {
            strack_fusion_imu(&f, t, v);
        } else if (line[0] == 'O' && sscanf(line + 1, "%lld %f %f %f %d %f", &t, &v[0], &v[1], &v[2], &leds, &rms) == 6) {
            strack_fusion_optical(&f, t, v, leds, rms);
        } else if (line[0] == 'P' && sscanf(line + 1, "%lld", &t) == 1) {
            float p[3];
            if (strack_fusion_predict(&f, t, 250000000, p, NULL)) printf("%lld %.5f %.5f %.5f\n", t, p[0], p[1], p[2]);
            else printf("%lld -\n", t);
        }
    }
    fprintf(stderr, "fits taken in %u, left out %u, restarts %u\n", f.accepted, f.rejected, f.restarts);
    return 0;
}
