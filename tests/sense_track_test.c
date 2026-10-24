// Tests sense_track.c on made-up camera frames: both controllers drawn as LED dots into the two
// lower cameras (as BC4, the way the headset sends them), with stray lights, noise and a wrong
// IMU yaw, then found and followed.
//
//   cc -O2 -fsanitize=address,undefined -I src tests/sense_track_test.c src/sense_track.c -lm -o build/sense_track_test
//   build/sense_track_test [frames] [hard]
//
// hard: a third of the LEDs hidden at random, more stray lights, the tracker's camera model a few
// percent and degrees off the real one, the hands crossing, and the left one leaving the view.

#include "sense_track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define W 1024
#define H 1016

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static float frand(void) { return rand() / (float)RAND_MAX; }

static void qmul(const float a[4], const float b[4], float o[4]) {
    float r[4] = {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
                  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
    memcpy(o, r, sizeof r);
}

static void qaxis(float x, float y, float z, float a, float q[4]) {
    float n = sqrtf(x * x + y * y + z * z), s = sinf(a / 2);
    q[0] = x / n * s; q[1] = y / n * s; q[2] = z / n * s; q[3] = cosf(a / 2);
}

static void qrot(const float q[4], const float v[3], float o[3]) {
    float t[3] = {2 * (q[1] * v[2] - q[2] * v[1]), 2 * (q[2] * v[0] - q[0] * v[2]), 2 * (q[0] * v[1] - q[1] * v[0])};
    float r[3] = {v[0] + q[3] * t[0] + (q[1] * t[2] - q[2] * t[1]), v[1] + q[3] * t[1] + (q[2] * t[0] - q[0] * t[2]),
                  v[2] + q[3] * t[2] + (q[0] * t[1] - q[1] * t[0])};
    memcpy(o, r, sizeof r);
}

static float qangle(const float a[4], const float b[4]) {
    float d = fabsf(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
    return 2 * acosf(fminf(1, d)) * 180 / (float)M_PI;
}

// The camera's axes in the head, as sense_track.c builds them, to draw with.
static void cam_axes(const strack_camera *c, float q[4]) {
    float qy[4], qx[4], qz[4], qc[4] = {1, 0, 0, 0}, t[4];   // qc: half turn about x, OpenCV to the head's axes
    qaxis(0, 1, 0, c->yaw_deg * (float)M_PI / 180, qy);
    qaxis(1, 0, 0, c->pitch_deg * (float)M_PI / 180, qx);
    qaxis(0, 0, 1, c->roll_deg * (float)M_PI / 180, qz);
    qmul(qy, qx, t);
    qmul(t, qz, t);
    qmul(t, qc, q);
}

static void splat(float *img, float u, float v, float peak, float sigma) {
    int r = (int)ceilf(sigma * 3);
    for (int y = (int)v - r; y <= (int)v + r; y++)
        for (int x = (int)u - r; x <= (int)u + r; x++) {
            if (x < 0 || y < 0 || x >= W || y >= H) continue;
            float dx = x + 0.5f - u, dy = y + 0.5f - v;
            img[y * W + x] += peak * expf(-(dx * dx + dy * dy) / (2 * sigma * sigma));
        }
}

// A plain BC4 encoder: the block's extremes as end points, eight values between them.
static void encode_bc4(const float *img, uint8_t *out) {
    for (int by = 0; by < H / 4; by++)
        for (int bx = 0; bx < W / 4; bx++) {
            uint8_t px[16], lo = 255, hi = 0;
            for (int i = 0; i < 16; i++) {
                float f = img[(by * 4 + i / 4) * W + bx * 4 + i % 4];
                px[i] = (uint8_t)fminf(255, fmaxf(0, f + 0.5f));
                if (px[i] < lo) lo = px[i];
                if (px[i] > hi) hi = px[i];
            }
            uint8_t *b = out + ((size_t)by * (W / 4) + bx) * 8;
            if (hi == lo) { hi = lo < 255 ? lo + 1 : lo; lo = hi - 1; }
            b[0] = hi;
            b[1] = lo;
            uint8_t pal[8] = {hi, lo};
            for (int i = 1; i < 7; i++) pal[i + 1] = (uint8_t)(((7 - i) * hi + i * lo + 3) / 7);
            uint64_t bits = 0;
            for (int i = 0; i < 16; i++) {
                int best = 0, bd = 1 << 30;
                for (int k = 0; k < 8; k++) {
                    int d = abs(pal[k] - px[i]);
                    if (d < bd) { bd = d; best = k; }
                }
                bits |= (uint64_t)best << (3 * i);
            }
            for (int i = 0; i < 6; i++) b[2 + i] = (uint8_t)(bits >> (8 * i));
        }
}

typedef struct { float q[4], p[3]; } pose;

// Draws both controllers (pose in the head) into each camera's frame.
static bool g_hard;

static void render(const strack_camera cam[2], const pose ctl[2], const bool on[2], uint8_t *bc4[2], int *visible, int seen[2]) {
    static float img[W * H];
    for (int c = 0; c < 2; c++) {
        for (int i = 0; i < W * H; i++) img[i] = 20 + 6 * frand();
        float qc[4], qci[4];
        cam_axes(&cam[c], qc);
        qci[0] = -qc[0]; qci[1] = -qc[1]; qci[2] = -qc[2]; qci[3] = qc[3];
        visible[c] = 0;
        if (c == 0) seen[0] = seen[1] = 0;
        for (int h = 0; h < 2; h++) {
            if (!on[h]) continue;
            for (int i = 0; i < STRACK_LEDS; i++) {
                float p[3], n[3], w[3], wn[3], pc[3], nc[3], u, v;
                strack_led(h, i, p, n);
                qrot(ctl[h].q, p, w);
                qrot(ctl[h].q, n, wn);
                for (int k = 0; k < 3; k++) w[k] += ctl[h].p[k] - cam[c].position[k];
                qrot(qci, w, pc);
                qrot(qci, wn, nc);
                float d = sqrtf(pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]);
                float facing = -(nc[0] * pc[0] + nc[1] * pc[1] + nc[2] * pc[2]) / d;
                if (facing < 0.1f) continue;
                if (g_hard && !getenv("NO_DROP") && frand() < 0.33f) continue;   // behind a finger
                if (!strack_project(&cam[c], pc, &u, &v)) continue;
                // An LED looks bigger close up and dimmer at a slant.
                float sigma = fmaxf(0.9f, 0.006f * cam[c].f / d * 0.5f);
                splat(img, u, v, 400 * facing, sigma);
                if (facing > 0.2f) { visible[c]++; seen[h]++; }
            }
        }
        // Stray lights: a lamp (large) and a couple of small reflections.
        splat(img, 120 + 30 * frand(), 150, 300, 25);
        splat(img, 800 + 40 * frand(), 900, 260, 1.0f);
        for (int k = 0; g_hard && !getenv("NO_STRAY") && k < 6; k++) splat(img, frand() * W, frand() * H, 200 + 100 * frand(), 0.8f + frand());
        encode_bc4(img, bc4[c]);
    }
}

int main(int argc, char **argv) {
    int frames = argc > 1 ? atoi(argv[1]) : 240;
    g_hard = argc > 2 && !strcmp(argv[2], "hard");
    srand(7);
    strack_camera cam[STRACK_CAMERAS], guess[STRACK_CAMERAS];
    strack_default_cameras(cam);
    memcpy(guess, cam, sizeof cam);
    if (g_hard && !getenv("NO_CAL")) {
        // The real cameras differ from what the tracker believes.
        for (int c = 0; c < 2; c++) {
            cam[c].f *= c ? 0.97f : 1.03f;
            cam[c].yaw_deg += c ? -1.5f : 2;
            cam[c].pitch_deg += 2;
            cam[c].cx += 4;
            cam[c].position[1] += 0.005f;
        }
    }
    strack_params prm;
    strack_default_params(&prm);
    strack *t = strack_create(guess, &prm);
    uint8_t *bc4[2] = {malloc((W / 4) * (H / 4) * 8), malloc((W / 4) * (H / 4) * 8)};

    // The decoder against the encoder: a dot's centre comes back within a tenth of a pixel.
    {
        static float img[W * H];
        for (int i = 0; i < W * H; i++) img[i] = 10;
        splat(img, 400.3f, 300.7f, 255, 1.5f);
        encode_bc4(img, bc4[0]);
        int n = strack_detect_bc4(t, 0, bc4[0], W / 4), cnt;
        const strack_blob *b = strack_blobs(t, 0, &cnt);
        printf("one dot: %d found at (%.2f, %.2f), area %d\n", n, n ? b[0].x : 0, n ? b[0].y : 0, n ? b[0].area : 0);
        if (n != 1 || fabsf(b[0].x - 400.3f) > 0.15f || fabsf(b[0].y - 300.7f) > 0.15f) { printf("FAIL: dot\n"); return 1; }
    }

    // The camera model both ways.
    for (int i = 0; i < 1000; i++) {
        float ray[3], u = frand() * 1016, v = frand() * 1016, u2, v2;
        strack_unproject(&cam[0], u, v, ray);
        if (ray[2] < cosf(cam[0].max_angle_deg * (float)M_PI / 180)) continue;
        if (!strack_project(&cam[0], ray, &u2, &v2) || fabsf(u - u2) > 1e-2f || fabsf(v - v2) > 1e-2f) {
            printf("FAIL: camera model (%.3f %.3f) -> (%.3f %.3f)\n", u, v, u2, v2);
            return 1;
        }
    }

    // The head looks a little down and to the right, standing at 1.6 m; the controllers start
    // in front at waist height and move about, turning.
    float head_q[4], head_p[3] = {0.1f, 1.6f, 0.2f};
    {
        float a[4], b[4];
        qaxis(0, 1, 0, -0.3f, a);
        qaxis(1, 0, 0, -0.25f, b);
        qmul(a, b, head_q);
    }
    float yaw_offset[2] = {50 * (float)M_PI / 180, -70 * (float)M_PI / 180};   // the IMU's yaw is way off
    int found_frames[2] = {0}, first[2] = {-1, -1}, n_valid[2] = {0}, n_seen[2] = {0};
    double late_p[2] = {0}, late_worst[2] = {0};
    int late_n[2] = {0};
    double err_p[2] = {0}, err_q[2] = {0}, worst_p[2] = {0}, total = 0, worst_t = 0;
    for (int f = 0; f < frames; f++) {
        float s = f / 60.0f;
        pose ctl_head[2], ctl_world[2];
        bool on[2] = {true, true};
        for (int h = 0; h < 2; h++) {
            float side = h == 0 ? -1 : 1;
            float p[3] = {side * 0.18f + 0.10f * sinf(1.3f * s + h), -0.30f + 0.08f * sinf(0.9f * s + 2 * h),
                          -0.42f + 0.12f * sinf(0.7f * s + h)};
            if (g_hard && !getenv("NO_MOVE")) {
                p[0] = side * 0.16f * cosf(0.9f * s);                    // the hands cross
                if (h == 0 && fmodf(s, 4) > 3) p[2] += 1.0f;             // and the left one goes behind
            }
            // Held as a hand holds it: pointing forward and a bit down, rolled, turning slowly.
            float a[4], b[4], c[4], q[4];
            qaxis(0, 1, 0, 0.5f * sinf(0.8f * s + h) - side * 0.2f, a);
            qaxis(1, 0, 0, -0.5f + 0.3f * sinf(1.1f * s), b);
            qaxis(0, 0, 1, side * 0.3f + 0.2f * sinf(0.6f * s), c);
            qmul(a, b, q);
            qmul(q, c, q);
            memcpy(ctl_head[h].q, q, sizeof q);
            memcpy(ctl_head[h].p, p, sizeof p);
            qmul(head_q, q, ctl_world[h].q);
            qrot(head_q, p, ctl_world[h].p);
            for (int k = 0; k < 3; k++) ctl_world[h].p[k] += head_p[k];
        }
        int visible[2], seen[2];
        render(cam, ctl_head, on, bc4, visible, seen);

        strack_input in = {.time_ns = (int64_t)(f * 1e9 / 60)};
        memcpy(in.head.q, head_q, sizeof head_q);
        memcpy(in.head.p, head_p, sizeof head_p);
        for (int h = 0; h < 2; h++) {
            // The IMU: right about tilt (within a degree), off by yaw_offset, which the runtime
            // takes the tracker's reports in to shrink, a tenth at a time.
            float yq[4], tq[4], q[4];
            qaxis(0, 1, 0, yaw_offset[h], yq);
            qaxis(frand() - 0.5f, 0, frand() - 0.5f, 0.6f * (float)M_PI / 180, tq);
            qmul(yq, ctl_world[h].q, q);
            qmul(tq, q, q);
            in.prior_valid[h] = true;
            memcpy(in.prior_q[h], q, sizeof q);
        }
        double t0 = now_s();
        for (int c = 0; c < 2; c++) strack_detect_bc4(t, c, bc4[c], W / 4);
        for (int c = 2; c < STRACK_CAMERAS; c++) strack_no_picture(t, c);
        strack_result r[2];
        strack_solve(t, &in, r);
        double dt = now_s() - t0;
        total += dt;
        if (dt > worst_t) worst_t = dt;
        for (int h = 0; h < 2; h++) {
            if (r[h].valid && !getenv("NO_ABSORB")) {
                float a = 0.1f * r[h].yaw_error;
                yaw_offset[h] += a;
                strack_shift_yaw(t, h, a);
            }
        }
        for (int h = 0; h < 2; h++) {
            if (seen[h] >= 4) n_seen[h]++;
            if (!r[h].valid) {
                if (getenv("VERBOSE")) printf("frame %3d %s: lost, dots %d/%d\n", f, h ? "right" : "left ", visible[0], visible[1]);
                continue;
            }
            if (first[h] < 0) first[h] = f;
            n_valid[h]++;
            found_frames[h] += r[h].found;
            float d[3];
            for (int k = 0; k < 3; k++) d[k] = r[h].pose.p[k] - ctl_world[h].p[k];
            float e = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            err_p[h] += e;
            if (e > worst_p[h]) worst_p[h] = e;
            err_q[h] += qangle(r[h].pose.q, ctl_world[h].q);
            if (f >= frames / 2) { late_n[h]++; late_p[h] += e; if (e > late_worst[h]) late_worst[h] = e; }
            if (e > 0.05f && getenv("BAD")) printf("frame %d %s: error %.0f mm, %d LEDs, %s\n", f, h ? "right" : "left", e * 1000, r[h].leds, r[h].found ? "found" : "followed");
            if (f < 3 || f % 60 == 0 || getenv("VERBOSE"))
                printf("frame %3d %s: %2d LEDs, rms %.2f px, %s, error %.1f mm, %.1f deg, yaw error %.1f deg, dots %d/%d\n", f,
                       h ? "right" : "left ", r[h].leds, r[h].rms_px, r[h].found ? "found " : "followed", e * 1000,
                       qangle(r[h].pose.q, ctl_world[h].q), r[h].yaw_error * 180 / M_PI, visible[0], visible[1]);
        }
    }
    int ok = 1;
    for (int h = 0; h < 2; h++) {
        double mp = n_valid[h] ? err_p[h] / n_valid[h] * 1000 : 0, mq = n_valid[h] ? err_q[h] / n_valid[h] : 0;
        printf("%s: tracked %d of %d frames it was visible in (first at %d, %d searches), mean error %.1f mm (worst %.1f),"
               " %.2f deg\n", h ? "right" : "left", n_valid[h], n_seen[h], first[h], found_frames[h], mp, worst_p[h] * 1000, mq);
        printf("  second half: %d frames, mean error %.1f mm, worst %.1f\n", late_n[h], late_n[h] ? late_p[h] / late_n[h] * 1000 : 0,
               late_worst[h] * 1000);
        if (n_valid[h] < 0.9 * n_seen[h] || mp > (g_hard ? 25 : 6) || worst_p[h] > (g_hard ? 0.1 : 0.03) || mq > (g_hard ? 5 : 2)) ok = 0;
    }
    {
        float rms;
        strack_camera got[STRACK_CAMERAS];
        strack_get_cameras(t, got);
        int runs = strack_calibration(t, &rms);
        printf("calibrations: %d, fit %.2f px\n", runs, rms);
        for (int c = 0; c < 2; c++)
            printf("camera %d: f %.1f (real %.1f), yaw %.2f (%.2f), pitch %.2f (%.2f), roll %.2f (%.2f)\n", c, got[c].f, cam[c].f,
                   got[c].yaw_deg, cam[c].yaw_deg, got[c].pitch_deg, cam[c].pitch_deg, got[c].roll_deg, cam[c].roll_deg);
    }
    printf("time per frame (both cameras, both controllers): mean %.2f ms, worst %.2f ms\n", total / frames * 1000, worst_t * 1000);
    strack_destroy(t);
    free(bc4[0]);
    free(bc4[1]);
    printf(ok ? "PASS\n" : "FAIL\n");
    return !ok;
}
