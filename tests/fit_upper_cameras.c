// Fits the upper cameras to a tracking-mode capture: the controllers' poses from the lower pair
// (where it sees them well), and for each upper camera the turn and centre that put the most of
// their LEDs on the dots it saw. A coarse search over every turn, then a finer one.
//
//   cc -O2 -I src tests/fit_upper_cameras.c src/sense_track.c -lm -o build/fit_upper_cameras
//   build/fit_upper_cameras ~/Desktop/"PS VR2 Controller Capture"

#include "sense_track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TW 512
#define TH 508
#define TP (TW * TH)
#define MAXS 2000

static void qrot(const float q[4], const float v[3], float o[3]) {
    float t[3] = {2 * (q[1] * v[2] - q[2] * v[1]), 2 * (q[2] * v[0] - q[0] * v[2]), 2 * (q[0] * v[1] - q[1] * v[0])};
    float r[3] = {v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]), v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]),
                  v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0])};
    memcpy(o, r, sizeof r);
}

static void rotm(const float ax[3], float a, float R[9]) {
    float c = cosf(a), s = sinf(a), v = 1 - c, x = ax[0], y = ax[1], z = ax[2];
    float M[9] = {c + x * x * v, x * y * v - z * s, x * z * v + y * s, y * x * v + z * s, c + y * y * v,
                  y * z * v - x * s, z * x * v - y * s, z * y * v + x * s, c + z * z * v};
    memcpy(R, M, sizeof M);
}

static void mm(const float A[9], const float B[9], float O[9]) {
    float r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    memcpy(O, r, sizeof r);
}

static void camrot(const strack_camera *c, float R[9]) {
    float Ry[9], Rx[9], Rz[9], C[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1}, t[9], y[3] = {0, 1, 0}, x[3] = {1, 0, 0}, z[3] = {0, 0, 1};
    rotm(y, c->yaw_deg * (float)M_PI / 180, Ry);
    rotm(x, c->pitch_deg * (float)M_PI / 180, Rx);
    rotm(z, c->roll_deg * (float)M_PI / 180, Rz);
    mm(Ry, Rx, t);
    mm(t, Rz, t);
    mm(t, C, R);
}

static float jnum(const char *s, const char *key) {
    const char *p = strstr(s, key);
    return p ? strtof(strchr(p, ':') + 1, NULL) : 0;
}

static int jarr(const char *s, const char *key, float *out, int n) {
    const char *p = strstr(s, key);
    if (!p || !(p = strchr(p, '['))) return 0;
    p++;
    for (int i = 0; i < n; i++) {
        char *e;
        out[i] = strtof(p, &e);
        p = e;
        while (*p == ',' || *p == ' ') p++;
    }
    return 1;
}

// A controller seen well by the lower pair: its LEDs and facings in the head, and an upper
// camera's dots that frame.
typedef struct {
    float led[STRACK_LEDS][3], nrm[STRACK_LEDS][3];
    int cam, n;
    float u[STRACK_MAX_BLOBS], v[STRACK_MAX_BLOBS];
} sample;

static sample S[MAXS];
static int NS;

static int score(const strack_camera *c, float gate) {
    float R[9];
    camrot(c, R);
    int total = 0;
    for (int s = 0; s < NS; s++) {
        if (S[s].cam != (c->position[1] > 0 ? (c->position[0] < 0 ? 2 : 3) : -1)) continue;
        unsigned long long used = 0;
        for (int i = 0; i < STRACK_LEDS; i++) {
            float w[3], pc[3], nc[3], u, v;
            for (int k = 0; k < 3; k++) w[k] = S[s].led[i][k] - c->position[k];
            for (int k = 0; k < 3; k++) pc[k] = R[k] * w[0] + R[3 + k] * w[1] + R[6 + k] * w[2];
            for (int k = 0; k < 3; k++) nc[k] = R[k] * S[s].nrm[i][0] + R[3 + k] * S[s].nrm[i][1] + R[6 + k] * S[s].nrm[i][2];
            float d = sqrtf(pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]);
            if (-(nc[0] * pc[0] + nc[1] * pc[1] + nc[2] * pc[2]) / d < 0.2f) continue;
            if (!strack_project(c, pc, &u, &v)) continue;
            int best = -1;
            float bd = gate * gate;
            for (int b = 0; b < S[s].n; b++) {
                if (used >> b & 1) continue;
                float du = S[s].u[b] - u, dv = S[s].v[b] - v, d2 = du * du + dv * dv;
                if (d2 < bd) { bd = d2; best = b; }
            }
            if (best >= 0) { used |= 1ull << best; total++; }
        }
    }
    return total;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s <capture dir>\n", argv[0]); return 1; }
    strack_camera cams[STRACK_CAMERAS], on[STRACK_CAMERAS];
    strack_default_cameras(cams);
    memcpy(on, cams, sizeof on);
    for (int c = 2; c < STRACK_CAMERAS; c++) { on[c].off = false; on[c].max_angle_deg = 85; }
    strack_params prm;
    strack_default_params(&prm);
    prm.calibrate = false;
    strack *t = strack_create(cams, &prm), *det = strack_create(on, &prm);
    char path[1024];
    static char line[1 << 17];
    snprintf(path, sizeof path, "%s/frames.jsonl", argv[1]);
    FILE *fl = fopen(path, "r");
    if (!fl) return 1;
    unsigned char *buf = malloc(2 * (256 + 2 * TP));
    while (fgets(line, sizeof line, fl) && NS < MAXS - 2) {
        char name[64];
        const char *fp = strstr(line, "\"file\": \"");
        if (!fp || sscanf(fp + 9, "%63[^\"]", name) != 1 || !strstr(name, ".m4")) continue;
        snprintf(path, sizeof path, "%s/%s", argv[1], name);
        FILE *fb = fopen(path, "rb");
        if (!fb) continue;
        size_t got = fread(buf, 1, 2 * (256 + 2 * TP), fb);
        fclose(fb);
        if (got != 2 * (256 + 2 * TP)) continue;
        strack_input in = {0};
        in.time_ns = (long long)jnum(line, "time_ns");
        char *hd = strstr(line, "\"head\"");
        jarr(hd, "\"q\"", in.head.q, 4);
        jarr(hd, "\"p\"", in.head.p, 3);
        char *lt = strstr(line, "\"left\""), *rt = strstr(line, "\"right\""), *dl = strstr(line, "\"dots_left\"");
        char *seg[2] = {lt, rt};
        for (int h = 0; h < 2; h++) {
            char tmp[2048];
            char *end = h == 0 ? rt : dl;
            size_t n = end && seg[h] ? (size_t)(end - seg[h]) : 0;
            if (n >= sizeof tmp) n = sizeof tmp - 1;
            memcpy(tmp, seg[h] ? seg[h] : "", n);
            tmp[n] = 0;
            in.prior_valid[h] = strstr(tmp, "\"imu\": true") != NULL;
            if (in.prior_valid[h]) jarr(tmp, "prior_q", in.prior_q[h], 4);
        }
        for (int c = 0; c < STRACK_CAMERAS; c++) {
            const unsigned char *pair = buf + (c / 2) * (256 + 2 * TP);
            const unsigned char *pic = pair + 256 + (c % 2) * TP;
            if (c < 2) strack_detect_half(t, c, pic, TW - 4, TH, TW);
            else strack_no_picture(t, c);
            if (c >= 2) strack_detect_half(det, c, pic, TW - 4, TH, TW);
        }
        strack_result r[2];
        strack_solve(t, &in, r);
        float hqc[4] = {-in.head.q[0], -in.head.q[1], -in.head.q[2], in.head.q[3]};
        for (int h = 0; h < 2; h++) {
            if (!r[h].valid || r[h].leds < 8 || r[h].rms_px > 2) continue;
            for (int c = 2; c < STRACK_CAMERAS; c++) {
                sample *s = &S[NS];
                int n;
                const strack_blob *b = strack_blobs(det, c, &n);
                if (n < 3) continue;
                s->cam = c;
                s->n = n;
                for (int i = 0; i < n; i++) { s->u[i] = b[i].x; s->v[i] = b[i].y; }
                for (int i = 0; i < STRACK_LEDS; i++) {
                    float lp[3], ln[3], w[3], wn[3];
                    strack_led(h, i, lp, ln);
                    qrot(r[h].pose.q, lp, w);
                    for (int k = 0; k < 3; k++) w[k] += r[h].pose.p[k] - in.head.p[k];
                    qrot(hqc, w, s->led[i]);
                    qrot(r[h].pose.q, ln, wn);
                    qrot(hqc, wn, s->nrm[i]);
                }
                NS++;
            }
        }
    }
    printf("%d samples (controller well seen below, three or more dots above)\n", NS);
    for (int c = 2; c < STRACK_CAMERAS; c++) {
        strack_camera best = on[c], k = on[c];
        int bs = -1;
        for (float yaw = -90; yaw <= 90; yaw += 6)
            for (float pitch = -40; pitch <= 90; pitch += 6)
                for (float roll = -90; roll <= 90; roll += 6) {
                    k.yaw_deg = yaw; k.pitch_deg = pitch; k.roll_deg = roll;
                    int sc = score(&k, 14);
                    if (sc > bs) { bs = sc; best = k; }
                }
        printf("camera %d coarse: yaw %.0f pitch %.0f roll %.0f, %d LEDs\n", c, best.yaw_deg, best.pitch_deg, best.roll_deg, bs);
        // Finer, with the centre and focal length.
        float step[6] = {2, 2, 2, 10, 10, 10};
        for (int round = 0; round < 6; round++) {
            for (int p = 0; p < 6; p++)
                for (int sg = -1; sg <= 1; sg += 2)
                    for (;;) {
                        k = best;
                        float *f[6] = {&k.yaw_deg, &k.pitch_deg, &k.roll_deg, &k.cx, &k.cy, &k.f};
                        *f[p] += sg * step[p];
                        int sc = score(&k, 6);
                        int cur = score(&best, 6);
                        if (sc > cur) best = k;
                        else break;
                    }
            for (int p = 0; p < 6; p++) step[p] /= 2;
        }
        printf("camera %d: f %.1f cx %.1f cy %.1f yaw %.1f pitch %.1f roll %.1f, %d LEDs within 6 px (%d within 14)\n", c,
               best.f, best.cx, best.cy, best.yaw_deg, best.pitch_deg, best.roll_deg, score(&best, 6), score(&best, 14));
    }
    return 0;
}
