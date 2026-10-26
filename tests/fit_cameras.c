// Fits the four tracking cameras' lenses and turns to many fits from recordings: the replay tool's
// matches of fits on many LEDs (MATCHES=1), exported as poses in the head and their matched dots.
// It alternates between each fit's pose with the cameras held, and each camera's focal length,
// centre, two lens terms and turn with the poses held; the cameras' turn as a whole is held where it
// was (the head's frame is the headset's, so a turn of all of them together is the poses' to take).
//
//   cc -O2 -I src tests/fit_cameras.c src/sense_track.c -lm -o build/fit_cameras
//   build/fit_cameras fit.obs [test.obs]   prints the cameras as controller_cameras.json and rms
//
// fit.obs: "F hand R(9, row-major, the LED model in the head) p(3) n", then n lines "cam led x y".

#include "sense_track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct { int cam, led; float x, y; } obs;
typedef struct { int hand, n; float R[9], p[3]; obs *o; } fit;

static fit *load(const char *path, int *count) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); exit(1); }
    int cap = 1024, n = 0;
    fit *fs = malloc(cap * sizeof *fs);
    char tag[4];
    while (fscanf(f, "%3s", tag) == 1) {
        if (strcmp(tag, "F")) continue;
        if (n == cap) fs = realloc(fs, (cap *= 2) * sizeof *fs);
        fit *x = &fs[n];
        if (fscanf(f, "%d", &x->hand) != 1) break;
        for (int i = 0; i < 9; i++) if (fscanf(f, "%f", &x->R[i]) != 1) break;
        for (int i = 0; i < 3; i++) if (fscanf(f, "%f", &x->p[i]) != 1) break;
        if (fscanf(f, "%d", &x->n) != 1) break;
        x->o = malloc(x->n * sizeof *x->o);
        for (int i = 0; i < x->n; i++) if (fscanf(f, "%d %d %f %f", &x->o[i].cam, &x->o[i].led, &x->o[i].x, &x->o[i].y) != 4) break;
        n++;
    }
    fclose(f);
    *count = n;
    return fs;
}

static strack_camera cams[STRACK_CAMERAS];
static float Rhc[STRACK_CAMERAS][9];
static float ledp[2][STRACK_LEDS][3], ledn[2][STRACK_LEDS][3];

static void cam_rot(void) { for (int c = 0; c < STRACK_CAMERAS; c++) strack_camera_rotation(&cams[c], Rhc[c]); }

static void mv(const float *M, const float *v, float *o) {
    for (int i = 0; i < 3; i++) o[i] = M[3 * i] * v[0] + M[3 * i + 1] * v[1] + M[3 * i + 2] * v[2];
}

// Where LED o lands for pose (R, p), with camera c's model and rotation.
static bool project(const strack_camera *c, const float *Rc, int hand, const float *R, const float *p, int led, float *u, float *v) {
    float w[3], pc[3];
    mv(R, ledp[hand][led], w);
    for (int k = 0; k < 3; k++) w[k] += p[k] - c->position[k];
    for (int i = 0; i < 3; i++) pc[i] = Rc[i] * w[0] + Rc[3 + i] * w[1] + Rc[6 + i] * w[2];   // Rc^T w
    if (pc[2] < 1e-3f) return false;
    strack_camera cc = *c;
    cc.width = cc.height = 100000;   // let it land anywhere
    return strack_project(&cc, pc, u, v);
}

static double huber(double e) { return e < 3 ? e * e : 6 * e - 9; }

static void rodrigues(const double w[3], float R[9]) {
    double a = sqrt(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]);
    double x = a > 1e-12 ? w[0] / a : 1, y = a > 1e-12 ? w[1] / a : 0, z = a > 1e-12 ? w[2] / a : 0, c = cos(a), s = sin(a), t = 1 - c;
    float M[9] = {(float)(t * x * x + c), (float)(t * x * y - s * z), (float)(t * x * z + s * y),
                  (float)(t * x * y + s * z), (float)(t * y * y + c), (float)(t * y * z - s * x),
                  (float)(t * x * z - s * y), (float)(t * y * z + s * x), (float)(t * z * z + c)};
    memcpy(R, M, sizeof M);
}

static void mm(const float *A, const float *B, float *O) {
    float T[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) T[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
    memcpy(O, T, sizeof T);
}

static bool solve(int n, double *A, double *b) {
    for (int i = 0; i < n; i++) {
        int p = i;
        for (int r = i + 1; r < n; r++) if (fabs(A[r * n + i]) > fabs(A[p * n + i])) p = r;
        if (fabs(A[p * n + i]) < 1e-15) return false;
        if (p != i) { for (int k = 0; k < n; k++) { double t = A[i * n + k]; A[i * n + k] = A[p * n + k]; A[p * n + k] = t; } double t = b[i]; b[i] = b[p]; b[p] = t; }
        for (int r = 0; r < n; r++) {
            if (r == i) continue;
            double f = A[r * n + i] / A[i * n + i];
            for (int k = i; k < n; k++) A[r * n + k] -= f * A[i * n + k];
            b[r] -= f * b[i];
        }
    }
    for (int i = 0; i < n; i++) b[i] /= A[i * n + i];
    return true;
}

// One fit's residuals (2 per match, Huber-scaled), for its pose moved by x (rotation, then position).
static int pose_res(const fit *f, const double *x, double *r) {
    float dR[9], R[9], p[3];
    rodrigues(x, dR);
    mm(dR, f->R, R);
    for (int k = 0; k < 3; k++) p[k] = f->p[k] + (float)x[3 + k];
    int k = 0;
    for (int i = 0; i < f->n; i++) {
        float u, v;
        if (!project(&cams[f->o[i].cam], Rhc[f->o[i].cam], f->hand, R, p, f->o[i].led, &u, &v)) { u = f->o[i].x + 30; v = f->o[i].y; }
        double du = u - f->o[i].x, dv = v - f->o[i].y, e = sqrt(du * du + dv * dv), s = e > 3 ? sqrt((6 * e - 9) / (e * e)) : 1;
        r[k++] = du * s;
        r[k++] = dv * s;
    }
    return k;
}

static double cost(const double *r, int k) { double s = 0; for (int i = 0; i < k; i++) s += r[i] * r[i]; return s; }

static void refine_pose(fit *f, int its) {
    enum { M = 2 * 64 };
    double r0[M], r1[M], J[M][6];
    if (f->n > 64) f->n = 64;
    double x0[6] = {0};
    int k = pose_res(f, x0, r0);
    double c0 = cost(r0, k), lambda = 1e-3;
    for (int it = 0; it < its; it++) {
        for (int j = 0; j < 6; j++) {
            double x[6] = {0};
            x[j] = j < 3 ? 1e-4 : 1e-5;
            pose_res(f, x, r1);
            for (int i = 0; i < k; i++) J[i][j] = (r1[i] - r0[i]) / x[j];
        }
        double A[36] = {0}, g[6] = {0};
        for (int i = 0; i < k; i++)
            for (int a = 0; a < 6; a++) { g[a] -= J[i][a] * r0[i]; for (int b = 0; b < 6; b++) A[a * 6 + b] += J[i][a] * J[i][b]; }
        bool better = false;
        for (int tries = 0; tries < 5 && !better; tries++) {
            double Mx[36], x[6];
            memcpy(Mx, A, sizeof Mx); memcpy(x, g, sizeof x);
            for (int a = 0; a < 6; a++) Mx[a * 6 + a] += lambda * (A[a * 6 + a] + 1e-9);
            if (!solve(6, Mx, x)) { lambda *= 10; continue; }
            pose_res(f, x, r1);
            double c1 = cost(r1, k);
            if (c1 < c0) {
                float dR[9];
                rodrigues(x, dR);
                mm(dR, f->R, f->R);
                for (int q = 0; q < 3; q++) f->p[q] += (float)x[3 + q];
                memcpy(r0, r1, sizeof(double) * k);
                c0 = c1;
                lambda = fmax(1e-7, lambda / 10);
                better = true;
            } else lambda *= 10;
        }
        if (!better) break;
    }
}

// A camera's parameters as a vector: f, cx, cy, k1, k2, yaw, pitch, roll.
enum { CP = 11 };
static void get_cp(const strack_camera *c, double *v) {
    v[0] = c->f; v[1] = c->cx; v[2] = c->cy; v[3] = c->k[0]; v[4] = c->k[1]; v[5] = c->yaw_deg; v[6] = c->pitch_deg; v[7] = c->roll_deg;
    for (int k = 0; k < 3; k++) v[8 + k] = c->position[k];
}
static void set_cp(strack_camera *c, const double *v) {
    c->f = (float)v[0]; c->cx = (float)v[1]; c->cy = (float)v[2]; c->k[0] = (float)v[3]; c->k[1] = (float)v[4];
    c->yaw_deg = (float)v[5]; c->pitch_deg = (float)v[6]; c->roll_deg = (float)v[7];
    for (int k = 0; k < 3; k++) c->position[k] = (float)v[8 + k];
}
static const double prior_sigma[CP] = {20, 15, 15, 0.05, 0.05, 3, 3, 3, 0.01, 0.01, 0.01};
static strack_camera base[STRACK_CAMERAS];

// Camera c's cost over all fits' matches in it, plus a weak pull toward where it started.
static double cam_cost(int c, const double *v, fit *fs, int nf, int *nobs) {
    strack_camera save = cams[c];
    float Rs[9];
    memcpy(Rs, Rhc[c], sizeof Rs);
    set_cp(&cams[c], v);
    strack_camera_rotation(&cams[c], Rhc[c]);
    double s = 0;
    int n = 0;
    for (int i = 0; i < nf; i++)
        for (int j = 0; j < fs[i].n; j++) {
            if (fs[i].o[j].cam != c) continue;
            float u, w;
            if (!project(&cams[c], Rhc[c], fs[i].hand, fs[i].R, fs[i].p, fs[i].o[j].led, &u, &w)) { s += 900; n++; continue; }
            double du = u - fs[i].o[j].x, dv = w - fs[i].o[j].y;
            s += huber(sqrt(du * du + dv * dv));
            n++;
        }
    double b[CP];
    get_cp(&base[c], b);
    for (int k = 0; k < CP; k++) { double e = (v[k] - b[k]) / prior_sigma[k]; s += e * e; }
    cams[c] = save;
    memcpy(Rhc[c], Rs, sizeof Rs);
    if (nobs) *nobs = n;
    return s;
}

// Gauss-Newton-ish on a camera's 8 numbers with numerical derivatives of the cost (Newton on a
// finite-difference Hessian, damped).
static void refine_cam(int c, fit *fs, int nf) {
    double v[CP], h[CP] = {0.2, 0.2, 0.2, 0.001, 0.001, 0.02, 0.02, 0.02, 0.0002, 0.0002, 0.0002};
    static int free_n = -1;
    if (free_n < 0) free_n = getenv("FREE") ? atoi(getenv("FREE")) : 8;   // the first FREE numbers are fitted
    // the lower left camera's turn and place stay: the head's frame is the headset's, and a turn of
    // all the cameras together is the poses' to take
    int nfree = c == 0 && free_n > 5 ? 5 : free_n;
    get_cp(&cams[c], v);
    double c0 = cam_cost(c, v, fs, nf, NULL), lambda = 1;
    for (int it = 0; it < 6; it++) {
        double g[CP] = {0}, H[CP * CP] = {0};
        for (int a = 0; a < nfree; a++) {
            double vp[CP], vm[CP];
            memcpy(vp, v, sizeof vp); memcpy(vm, v, sizeof vm);
            vp[a] += h[a]; vm[a] -= h[a];
            double cp = cam_cost(c, vp, fs, nf, NULL), cm = cam_cost(c, vm, fs, nf, NULL);
            g[a] = (cp - cm) / (2 * h[a]);
            H[a * CP + a] = (cp - 2 * c0 + cm) / (h[a] * h[a]);
            for (int b = 0; b < a; b++) {
                double vpp[CP], vpm[CP], vmp[CP], vmm[CP];
                memcpy(vpp, v, sizeof v); memcpy(vpm, v, sizeof v); memcpy(vmp, v, sizeof v); memcpy(vmm, v, sizeof v);
                vpp[a] += h[a]; vpp[b] += h[b]; vpm[a] += h[a]; vpm[b] -= h[b];
                vmp[a] -= h[a]; vmp[b] += h[b]; vmm[a] -= h[a]; vmm[b] -= h[b];
                H[a * CP + b] = H[b * CP + a] = (cam_cost(c, vpp, fs, nf, NULL) - cam_cost(c, vpm, fs, nf, NULL) -
                                                 cam_cost(c, vmp, fs, nf, NULL) + cam_cost(c, vmm, fs, nf, NULL)) / (4 * h[a] * h[b]);
            }
        }
        bool better = false;
        for (int tries = 0; tries < 6 && !better; tries++) {
            double M[CP * CP], x[CP];
            memcpy(M, H, sizeof M);
            double Ms[CP * CP], xs[CP];
            for (int a = 0; a < nfree; a++) {
                for (int b = 0; b < nfree; b++) Ms[a * nfree + b] = M[a * CP + b];
                Ms[a * nfree + a] += lambda * fabs(H[a * CP + a]) + 1e-9;
                xs[a] = -g[a];
            }
            if (!solve(nfree, Ms, xs)) { lambda *= 10; continue; }
            for (int a = 0; a < CP; a++) x[a] = a < nfree ? xs[a] : 0;
            double vn[CP];
            for (int a = 0; a < CP; a++) vn[a] = v[a] + x[a];
            double c1 = cam_cost(c, vn, fs, nf, NULL);
            if (c1 < c0) { memcpy(v, vn, sizeof v); c0 = c1; lambda = fmax(1e-4, lambda / 10); better = true; }
            else lambda *= 10;
        }
        if (!better) break;
    }
    set_cp(&cams[c], v);
    strack_camera_rotation(&cams[c], Rhc[c]);
}


// One LED's cost over its matches in all fits, at position q (in the LED model), plus a pull of
// sigma 3 mm toward Monado's.
static float led0[2][STRACK_LEDS][3];
static double led_cost(int hand, int led, const float *q, fit *fs, int nf) {
    float save[3];
    memcpy(save, ledp[hand][led], sizeof save);
    memcpy(ledp[hand][led], q, sizeof save);
    double s = 0;
    for (int i = 0; i < nf; i++) {
        if (fs[i].hand != hand) continue;
        for (int j = 0; j < fs[i].n; j++) {
            if (fs[i].o[j].led != led) continue;
            float u, v;
            int c = fs[i].o[j].cam;
            if (!project(&cams[c], Rhc[c], hand, fs[i].R, fs[i].p, led, &u, &v)) { s += 900; continue; }
            s += huber(hypot(u - fs[i].o[j].x, v - fs[i].o[j].y));
        }
    }
    for (int k = 0; k < 3; k++) { double e = (q[k] - led0[hand][led][k]) / 0.003; s += e * e; }
    memcpy(ledp[hand][led], save, sizeof save);
    return s;
}

static void refine_leds(fit *fs, int nf) {
    for (int hand = 0; hand < 2; hand++)
        for (int led = 0; led < STRACK_LEDS; led++) {
            float q[3];
            memcpy(q, ledp[hand][led], sizeof q);
            double c0 = led_cost(hand, led, q, fs, nf);
            for (int it = 0; it < 4; it++) {
                double g[3], H[9], h = 0.0002;
                for (int a = 0; a < 3; a++) {
                    float qp[3], qm[3];
                    memcpy(qp, q, sizeof q); memcpy(qm, q, sizeof q);
                    qp[a] += h; qm[a] -= h;
                    double cp = led_cost(hand, led, qp, fs, nf), cm = led_cost(hand, led, qm, fs, nf);
                    g[a] = (cp - cm) / (2 * h);
                    H[4 * a] = (cp - 2 * c0 + cm) / (h * h);
                    for (int b = 0; b < a; b++) {
                        float pp[3], pm[3], mp[3], mq[3];
                        memcpy(pp, q, 12); memcpy(pm, q, 12); memcpy(mp, q, 12); memcpy(mq, q, 12);
                        pp[a] += h; pp[b] += h; pm[a] += h; pm[b] -= h; mp[a] -= h; mp[b] += h; mq[a] -= h; mq[b] -= h;
                        H[3 * a + b] = H[3 * b + a] = (led_cost(hand, led, pp, fs, nf) - led_cost(hand, led, pm, fs, nf) -
                                                       led_cost(hand, led, mp, fs, nf) + led_cost(hand, led, mq, fs, nf)) / (4 * h * h);
                    }
                }
                double M[9], x[3];
                memcpy(M, H, sizeof M);
                for (int a = 0; a < 3; a++) { M[4 * a] += 1e-3 * fabs(H[4 * a]) + 1e-9; x[a] = -g[a]; }
                if (!solve(3, M, x)) break;
                float qn[3] = {q[0] + (float)x[0], q[1] + (float)x[1], q[2] + (float)x[2]};
                double c1 = led_cost(hand, led, qn, fs, nf);
                if (c1 >= c0) break;
                memcpy(q, qn, sizeof q);
                c0 = c1;
            }
            memcpy(ledp[hand][led], q, sizeof q);
        }
}

static double rms(fit *fs, int nf) {
    double s = 0;
    long n = 0;
    for (int i = 0; i < nf; i++) {
        double r[128], x[6] = {0};
        int k = pose_res(&fs[i], x, r);
        for (int j = 0; j < k; j++) s += r[j] * r[j];
        n += k / 2;
    }
    return sqrt(s / n);
}

static double median_err(fit *fs, int nf) {
    int cap = 0;
    for (int i = 0; i < nf; i++) cap += fs[i].n;
    double *e = malloc(cap * sizeof *e);
    int n = 0;
    for (int i = 0; i < nf; i++)
        for (int j = 0; j < fs[i].n; j++) {
            float u, v;
            if (!project(&cams[fs[i].o[j].cam], Rhc[fs[i].o[j].cam], fs[i].hand, fs[i].R, fs[i].p, fs[i].o[j].led, &u, &v)) continue;
            e[n++] = hypot(u - fs[i].o[j].x, v - fs[i].o[j].y);
        }
    for (int i = 1; i < n; i++) { double t = e[i]; int j = i; while (j > 0 && e[j - 1] > t) { e[j] = e[j - 1]; j--; } e[j] = t; }
    double m = n ? e[n / 2] : 0;
    free(e);
    return m;
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s fit.obs [test.obs]\n", argv[0]); return 1; }
    strack_default_cameras(cams);
    memcpy(base, cams, sizeof base);
    cam_rot();
    for (int h = 0; h < 2; h++) for (int i = 0; i < STRACK_LEDS; i++) strack_led(h, i, ledp[h][i], ledn[h][i]);
    memcpy(led0, ledp, sizeof led0);
    bool fit_leds = getenv("LEDS") && atoi(getenv("LEDS"));
    int nf, nt = 0;
    fit *fs = load(argv[1], &nf), *ts = argc > 2 ? load(argv[2], &nt) : NULL;
    for (int i = 0; i < nf; i++) refine_pose(&fs[i], 10);
    fprintf(stderr, "%d fits; built-in cameras: rms %.2f px, median %.2f px\n", nf, rms(fs, nf), median_err(fs, nf));
    if (ts) {
        for (int i = 0; i < nt; i++) refine_pose(&ts[i], 10);
        fprintf(stderr, "test %d fits; built-in cameras: rms %.2f px, median %.2f px\n", nt, rms(ts, nt), median_err(ts, nt));
    }
    int rounds = getenv("ROUNDS") ? atoi(getenv("ROUNDS")) : 12;
    for (int round = 0; round < rounds; round++) {
        if (!getenv("NOCAMS")) for (int c = 0; c < STRACK_CAMERAS; c++) refine_cam(c, fs, nf);
        if (fit_leds) refine_leds(fs, nf);
        cam_rot();
        for (int i = 0; i < nf; i++) refine_pose(&fs[i], 6);
        fprintf(stderr, "round %d: rms %.2f px, median %.2f px\n", round + 1, rms(fs, nf), median_err(fs, nf));
    }
    if (ts) {
        for (int i = 0; i < nt; i++) refine_pose(&ts[i], 10);
        fprintf(stderr, "test %d fits; fitted cameras: rms %.2f px, median %.2f px\n", nt, rms(ts, nt), median_err(ts, nt));
    }
    for (int h = 0; h < 2 && fit_leds; h++) {
        fprintf(stderr, "hand %d LEDs moved (mm):", h);
        for (int i = 0; i < STRACK_LEDS; i++)
            fprintf(stderr, " %.1f", 1000 * sqrt(pow(ledp[h][i][0] - led0[h][i][0], 2) + pow(ledp[h][i][1] - led0[h][i][1], 2) + pow(ledp[h][i][2] - led0[h][i][2], 2)));
        fprintf(stderr, "\n");
    }
    printf("[\n");
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        const strack_camera *k = &cams[c];
        printf("  {\"f\": %.2f, \"cx\": %.2f, \"cy\": %.2f, \"k\": [%.5f, %.5f, 0, 0], \"yaw_deg\": %.3f, \"pitch_deg\": %.3f, "
               "\"roll_deg\": %.3f, \"position\": [%.4f, %.4f, %.4f]}%s\n", k->f, k->cx, k->cy, k->k[0], k->k[1], k->yaw_deg,
               k->pitch_deg, k->roll_deg, k->position[0], k->position[1], k->position[2], c < 3 ? "," : "");
    }
    printf("]\n");
    return 0;
}
