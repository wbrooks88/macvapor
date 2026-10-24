// Sense controller positions from the headset's cameras; see sense_track.h.

#include "sense_track.h"

#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PI_F 3.14159265f
#define DEG (PI_F / 180)
#define RING_LEVEL 0.6f   // of the threshold: what counts as bright around a dot

enum { FRAME_W = 1024, FRAME_H = 1016, STACK_CAP = FRAME_W * 8, MAX_MATCHES = 2 * STRACK_LEDS, MAX_PAIRS = 128, HYPOTHESES = 6 };

// ------------------------------------------------------------------------------------ LED model
//
// Monado's pssense_led_model.h (BSL-1.0, Copyright 2026 Beyley Cardellio): positions in metres and
// the way each LED faces, for the left controller; the right one is its mirror image in x.

static const float k_leds_left[STRACK_LEDS][2][3] = {
    {{-0.012054f, -0.055213f, 0.038152f}, {-0.176485f, -0.811967f, 0.556384f}},
    {{-0.033088f, -0.055236f, 0.020190f}, {-0.426232f, -0.891845f, -0.151455f}},
    {{-0.053695f, -0.031009f, 0.028350f}, {-0.788493f, -0.456350f, 0.412339f}},
    {{-0.066986f, -0.010505f, 0.008679f}, {-0.980055f, -0.153898f, 0.125726f}},
    {{-0.063505f, 0.024601f, 0.004185f}, {-0.930796f, 0.360502f, 0.060482f}},
    {{-0.047803f, 0.044966f, 0.018502f}, {-0.701817f, 0.660177f, 0.267618f}},
    {{0.045472f, -0.027042f, 0.043022f}, {0.667492f, -0.396842f, 0.630056f}},
    {{0.059535f, -0.013977f, 0.028536f}, {0.961512f, -0.273488f, -0.026435f}},
    {{0.061060f, 0.014608f, 0.024725f}, {0.984435f, 0.154641f, -0.083507f}},
    {{0.050994f, 0.027255f, 0.036184f}, {0.748123f, 0.399672f, 0.529693f}},
    {{0.048475f, 0.043025f, 0.018776f}, {0.795992f, 0.580182f, -0.172584f}},
    {{0.030569f, 0.053446f, 0.029350f}, {0.448555f, 0.783814f, 0.429458f}},
    {{0.018498f, 0.063913f, 0.011230f}, {0.346695f, 0.893406f, -0.285708f}},
    {{-0.002748f, 0.064260f, 0.022718f}, {-0.040150f, 0.942362f, 0.332178f}},
    {{-0.029078f, 0.013435f, -0.060735f}, {-0.417589f, 0.258817f, -0.870996f}},
    {{0.001067f, 0.013435f, -0.067327f}, {0.014114f, 0.258800f, -0.965828f}},
    {{0.031783f, 0.013435f, -0.059270f}, {0.458628f, 0.258803f, -0.850106f}},
};

void strack_led(int hand, int i, float pos[3], float normal[3]) {
    float s = hand == 1 ? -1 : 1;
    for (int k = 0; k < 3; k++) {
        pos[k] = k_leds_left[i][0][k] * (k == 0 ? s : 1);
        normal[k] = k_leds_left[i][1][k] * (k == 0 ? s : 1);
    }
}

// ----------------------------------------------------------------------------------------- math

static float dot3(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
static float norm3(const float a[3]) { return sqrtf(dot3(a, a)); }

// Row-major 3x3.
static void mv(const float M[9], const float v[3], float o[3]) {
    float r[3] = {M[0] * v[0] + M[1] * v[1] + M[2] * v[2], M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
                  M[6] * v[0] + M[7] * v[1] + M[8] * v[2]};
    memcpy(o, r, sizeof r);
}

static void mtv(const float M[9], const float v[3], float o[3]) {   // transpose times v
    float r[3] = {M[0] * v[0] + M[3] * v[1] + M[6] * v[2], M[1] * v[0] + M[4] * v[1] + M[7] * v[2],
                  M[2] * v[0] + M[5] * v[1] + M[8] * v[2]};
    memcpy(o, r, sizeof r);
}

static void mm(const float A[9], const float B[9], float O[9]) {
    float r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    memcpy(O, r, sizeof r);
}

static void mtm(const float A[9], const float B[9], float O[9]) {   // A^T B
    float r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r[i * 3 + j] = A[i] * B[j] + A[3 + i] * B[3 + j] + A[6 + i] * B[6 + j];
    memcpy(O, r, sizeof r);
}

static void mmt(const float A[9], const float B[9], float O[9]) {   // A B^T
    float r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r[i * 3 + j] = A[i * 3] * B[j * 3] + A[i * 3 + 1] * B[j * 3 + 1] + A[i * 3 + 2] * B[j * 3 + 2];
    memcpy(O, r, sizeof r);
}

static void quat_to_mat(const float q[4], float R[9]) {
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float n = x * x + y * y + z * z + w * w, s = n > 0 ? 2 / n : 0;
    R[0] = 1 - s * (y * y + z * z); R[1] = s * (x * y - z * w);     R[2] = s * (x * z + y * w);
    R[3] = s * (x * y + z * w);     R[4] = 1 - s * (x * x + z * z); R[5] = s * (y * z - x * w);
    R[6] = s * (x * z - y * w);     R[7] = s * (y * z + x * w);     R[8] = 1 - s * (x * x + y * y);
}

static void mat_to_quat(const float R[9], float q[4]) {
    float tr = R[0] + R[4] + R[8];
    if (tr > 0) {
        float s = sqrtf(tr + 1) * 2;
        q[3] = s / 4; q[0] = (R[7] - R[5]) / s; q[1] = (R[2] - R[6]) / s; q[2] = (R[3] - R[1]) / s;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        float s = sqrtf(1 + R[0] - R[4] - R[8]) * 2;
        q[3] = (R[7] - R[5]) / s; q[0] = s / 4; q[1] = (R[1] + R[3]) / s; q[2] = (R[2] + R[6]) / s;
    } else if (R[4] > R[8]) {
        float s = sqrtf(1 + R[4] - R[0] - R[8]) * 2;
        q[3] = (R[2] - R[6]) / s; q[0] = (R[1] + R[3]) / s; q[1] = s / 4; q[2] = (R[5] + R[7]) / s;
    } else {
        float s = sqrtf(1 + R[8] - R[0] - R[4]) * 2;
        q[3] = (R[3] - R[1]) / s; q[0] = (R[2] + R[6]) / s; q[1] = (R[5] + R[7]) / s; q[2] = s / 4;
    }
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (int i = 0; i < 4; i++) q[i] /= n;
}

// The rotation of angle |w| about w.
static void exp_map(const float w[3], float R[9]) {
    float a = norm3(w);
    if (a < 1e-9f) {
        float I[9] = {1, -w[2], w[1], w[2], 1, -w[0], -w[1], w[0], 1};
        memcpy(R, I, sizeof I);
        return;
    }
    float k[3] = {w[0] / a, w[1] / a, w[2] / a}, c = cosf(a), s = sinf(a), v = 1 - c;
    R[0] = c + k[0] * k[0] * v;        R[1] = k[0] * k[1] * v - k[2] * s; R[2] = k[0] * k[2] * v + k[1] * s;
    R[3] = k[1] * k[0] * v + k[2] * s; R[4] = c + k[1] * k[1] * v;        R[5] = k[1] * k[2] * v - k[0] * s;
    R[6] = k[2] * k[0] * v - k[1] * s; R[7] = k[2] * k[1] * v + k[0] * s; R[8] = c + k[2] * k[2] * v;
}

// The axis times angle of R.
static void log_map(const float R[9], float w[3]) {
    float c = fmaxf(-1, fminf(1, (R[0] + R[4] + R[8] - 1) / 2)), a = acosf(c);
    float v[3] = {R[7] - R[5], R[2] - R[6], R[3] - R[1]};
    float s = sinf(a);
    if (s < 1e-4f) {
        if (a < 0.5f) { for (int i = 0; i < 3; i++) w[i] = v[i] / 2; return; }
        // Close to a half turn: the axis from the diagonal.
        float k[3] = {sqrtf(fmaxf(0, (R[0] + 1) / 2)), sqrtf(fmaxf(0, (R[4] + 1) / 2)), sqrtf(fmaxf(0, (R[8] + 1) / 2))};
        if (R[1] < 0) k[1] = -k[1];
        if (R[2] < 0) k[2] = -k[2];
        for (int i = 0; i < 3; i++) w[i] = k[i] * a;
        return;
    }
    for (int i = 0; i < 3; i++) w[i] = v[i] * a / (2 * s);
}

static void axis_angle(const float axis[3], float a, float R[9]) {
    float w[3] = {axis[0] * a, axis[1] * a, axis[2] * a};
    exp_map(w, R);
}

// Solves the n x n system A x = b in place (Gaussian elimination with pivoting). False if singular.
static bool solve_n(int n, double *A, double *b) {
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++) if (fabs(A[r * n + c]) > fabs(A[p * n + c])) p = r;
        if (fabs(A[p * n + c]) < 1e-12) return false;
        if (p != c) {
            for (int k = 0; k < n; k++) { double t = A[c * n + k]; A[c * n + k] = A[p * n + k]; A[p * n + k] = t; }
            double t = b[c]; b[c] = b[p]; b[p] = t;
        }
        for (int r = c + 1; r < n; r++) {
            double f = A[r * n + c] / A[c * n + c];
            for (int k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
            b[r] -= f * b[c];
        }
    }
    for (int c = n - 1; c >= 0; c--) {
        for (int k = c + 1; k < n; k++) b[c] -= A[c * n + k] * b[k];
        b[c] /= A[c * n + c];
    }
    return true;
}

// --------------------------------------------------------------------------------------- camera

void strack_default_cameras(strack_camera out[STRACK_CAMERAS]) {
    // Fitted to a capture from a real headset (both controllers moving in front of the wearer, 60
    // frames): the turn, centre and focal length that make the two cameras agree on where each
    // controller is (to about a centimetre) and put the most LEDs on dots. The lenses are toed in
    // by roll as well as yaw, and look about 28 degrees down.
    static const float k[STRACK_CAMERAS][5] = {
        // cx, cy, yaw, pitch, roll
        {509.6f, 478.0f, 12.8f, -26.0f, -6.8f},
        {500.5f, 508.0f, -11.0f, -30.0f, 6.4f},
    };
    for (int i = 0; i < STRACK_CAMERAS; i++) {
        float side = i == 0 ? -1 : 1;
        out[i] = (strack_camera){
            .f = 410.5f, .cx = k[i][0], .cy = k[i][1], .k = {0.03f, 0, 0, 0}, .max_angle_deg = 78,
            .width = 1016, .height = 1016, .position = {side * 0.060f, -0.030f, -0.102f},
            .yaw_deg = k[i][2], .pitch_deg = k[i][3], .roll_deg = k[i][4],
        };
    }
}

void strack_default_params(strack_params *p) {
    *p = (strack_params){.threshold = 170, .max_area = 600, .gate_px = 5, .min_leds = 5, .max_rms_px = 2.5f, .search_hz = 15};
}

// The camera's axes in the head: OpenCV's x right, y down, z forward, turned by yaw, pitch, roll.
static void camera_rotation(const strack_camera *c, float R[9]) {
    float Ry[9], Rx[9], Rz[9], C[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1}, t[9];
    float y[3] = {0, 1, 0}, x[3] = {1, 0, 0}, z[3] = {0, 0, 1};
    axis_angle(y, c->yaw_deg * DEG, Ry);
    axis_angle(x, c->pitch_deg * DEG, Rx);
    axis_angle(z, c->roll_deg * DEG, Rz);
    mm(Ry, Rx, t);
    mm(t, Rz, t);
    mm(t, C, R);
}

static float distort(const strack_camera *c, float th) {
    float t2 = th * th;
    return th * (1 + t2 * (c->k[0] + t2 * (c->k[1] + t2 * (c->k[2] + t2 * c->k[3]))));
}

bool strack_project(const strack_camera *c, const float p[3], float *u, float *v) {
    float r = sqrtf(p[0] * p[0] + p[1] * p[1]);
    float th = atan2f(r, p[2]);
    if (th > c->max_angle_deg * DEG) return false;
    if (r < 1e-9f) { *u = c->cx; *v = c->cy; return true; }
    float d = c->f * distort(c, th) / r;
    *u = c->cx + d * p[0];
    *v = c->cy + d * p[1];
    return *u >= 0 && *v >= 0 && *u < c->width && *v < c->height;
}

void strack_unproject(const strack_camera *c, float u, float v, float ray[3]) {
    float x = (u - c->cx) / c->f, y = (v - c->cy) / c->f, td = sqrtf(x * x + y * y);
    if (td < 1e-9f) { ray[0] = ray[1] = 0; ray[2] = 1; return; }
    float th = td;
    for (int i = 0; i < 8; i++) {
        float t2 = th * th;
        float g = distort(c, th) - td;
        float dg = 1 + t2 * (3 * c->k[0] + t2 * (5 * c->k[1] + t2 * (7 * c->k[2] + t2 * 9 * c->k[3])));
        th -= g / dg;
    }
    float s = sinf(th) / td;
    ray[0] = x * s;
    ray[1] = y * s;
    ray[2] = cosf(th);
}

// ------------------------------------------------------------------------------------- tracker

typedef struct { int cam, led, blob; float du, dv; } match;

typedef struct {
    bool tracking;
    int64_t t_ns;               // of the last pose
    float R[9], p[3];           // in the world
    float v[3];                 // m/s in the world
    float yaw_error;            // of the prior, last time
    int leds;                   // matched last time
    bool searched;
    int64_t last_search_ns;
    int64_t locked_until_ns;    // the prior's yaw was confirmed recently: search near it only
} hand_state;

enum { CAL_SAMPLES = 40 };

// A pose in the head: the LED model's orientation R and position p.
typedef struct { float R[9], p[3]; } hpose;

// A frame kept for calibrating the cameras: what a controller's LEDs were matched to, and what its
// IMU said about its orientation.
typedef struct {
    int hand, n;
    float prior[9], up[3];
    hpose P;
    struct { int cam, led; float u, v; } o[MAX_MATCHES];
} cal_sample;

struct strack {
    strack_camera base[STRACK_CAMERAS];     // the guess the tracker started from
    float delta[STRACK_CAMERAS][6];         // calibrated away from it: yaw, pitch, roll (rad), log f,
                                            // cx, cy (pixels)
    strack_camera cam[STRACK_CAMERAS];
    float Rhc[STRACK_CAMERAS][9];       // camera axes to head axes
    strack_params prm;
    uint8_t *gray, *seen;               // one camera's picture at a time
    int *stack, *hot;                   // the flood fill's, and the bright blocks
    strack_blob blobs[STRACK_CAMERAS][STRACK_MAX_BLOBS];
    int nblobs[STRACK_CAMERAS];
    bool claimed[STRACK_CAMERAS][STRACK_MAX_BLOBS];
    int pairs[MAX_PAIRS][2], npairs;    // LEDs close enough together to be seen as neighbours
    float led[2][STRACK_LEDS][3], normal[2][STRACK_LEDS][3];
    hand_state hs[2];
    cal_sample cal[CAL_SAMPLES];
    int ncal, cal_next, cal_new, cal_runs;
    int64_t last_sample_ns, last_cal_ns;
    float cal_rms;
};

strack *strack_create(const strack_camera cameras[STRACK_CAMERAS], const strack_params *params) {
    strack *t = calloc(1, sizeof *t);
    if (!t) return NULL;
    t->gray = malloc(FRAME_W * FRAME_H);
    t->seen = malloc(FRAME_W * FRAME_H);
    t->stack = malloc(sizeof(int) * STACK_CAP);
    t->hot = malloc(sizeof(int) * (FRAME_W / 4) * (FRAME_H / 4));
    if (!t->gray || !t->seen || !t->stack || !t->hot) { strack_destroy(t); return NULL; }
    for (int i = 0; i < STRACK_CAMERAS; i++) {
        t->base[i] = t->cam[i] = cameras[i];
        camera_rotation(&cameras[i], t->Rhc[i]);
    }
    t->prm = *params;
    for (int h = 0; h < 2; h++)
        for (int i = 0; i < STRACK_LEDS; i++) strack_led(h, i, t->led[h][i], t->normal[h][i]);
    for (int a = 0; a < STRACK_LEDS; a++)
        for (int b = 0; b < STRACK_LEDS; b++) {
            if (a == b) continue;
            float d[3] = {t->led[0][b][0] - t->led[0][a][0], t->led[0][b][1] - t->led[0][a][1], t->led[0][b][2] - t->led[0][a][2]};
            float na = dot3(t->normal[0][a], t->normal[0][b]);
            // Neighbours that a camera can see at once: close, and facing within 100 degrees.
            if (norm3(d) < 0.048f && na > -0.17f && t->npairs < MAX_PAIRS) {
                t->pairs[t->npairs][0] = a;
                t->pairs[t->npairs][1] = b;
                t->npairs++;
            }
        }
    return t;
}

void strack_destroy(strack *t) {
    if (!t) return;
    free(t->gray);
    free(t->seen);
    free(t->stack);
    free(t->hot);
    free(t);
}

// ------------------------------------------------------------------------------------ the dots

static void add_blob(strack *t, int cam, const strack_blob *b) {
    strack_blob *list = t->blobs[cam];
    int n = t->nblobs[cam];
    if (n == STRACK_MAX_BLOBS && list[n - 1].brightness >= b->brightness) return;
    int i = n < STRACK_MAX_BLOBS ? n++ : n - 1;   // the brightest are kept, in order
    while (i > 0 && list[i - 1].brightness < b->brightness) { list[i] = list[i - 1]; i--; }
    list[i] = *b;
    t->nblobs[cam] = n;
}

// The dot that pixel (x, y) is part of: a flood fill over pixels at the threshold or above.
static void fill_blob(strack *t, int cam, int x0, int y0) {
    int *stack = t->stack, sp = 0, area = 0, minx = x0, maxx = x0, miny = y0, maxy = y0;
    double sw = 0, sx = 0, sy = 0;
    float thr = t->prm.threshold;
    bool overflow = false;
    stack[sp++] = y0 * FRAME_W + x0;
    t->seen[y0 * FRAME_W + x0] = 1;
    while (sp) {
        int i = stack[--sp], x = i % FRAME_W, y = i / FRAME_W;
        float w = t->gray[i] - thr + 1;
        area++;
        sw += w; sx += w * x; sy += w * y;
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
        for (int dy = -1; dy <= 1; dy++)
            for (int dx = -1; dx <= 1; dx++) {
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= FRAME_W || ny >= FRAME_H) continue;
                int j = ny * FRAME_W + nx;
                if (t->seen[j] || t->gray[j] < thr) continue;
                t->seen[j] = 1;
                if (sp < STACK_CAP) stack[sp++] = j;
                else overflow = true;
            }
    }
    int bw = maxx - minx + 1, bh = maxy - miny + 1;
    if (overflow || area > t->prm.max_area) return;
    if (bw > 3 * bh + 2 || bh > 3 * bw + 2) return;              // streaks and edges, not dots
    if (area > 12 && area < 0.35f * bw * bh) return;              // rings and odd shapes
    strack_blob b = {.x = (float)(sx / sw) + 0.5f, .y = (float)(sy / sw) + 0.5f, .brightness = (float)sw, .area = area};
    // An LED is a dot on the dark: around it the picture is dim, or another LED. Bits of a lit
    // screen, a lamp or a window are surrounded by grey that is nearly as bright (in the headset's
    // pictures an LED's surroundings top out near 120 with the threshold at 170; a laptop screen's
    // are all 130 to 170).
    float r = 2 * sqrtf(area / PI_F) + 2, ring = RING_LEVEL * thr;
    int bright = 0, n = 0;
    for (int k = 0; k < 32; k++) {
        int x = (int)(b.x - 0.5f + r * cosf(k * PI_F / 16)), y = (int)(b.y - 0.5f + r * sinf(k * PI_F / 16));
        if (x < 0 || y < 0 || x >= FRAME_W || y >= FRAME_H) continue;
        n++;
        bright += t->gray[y * FRAME_W + x] >= ring && t->gray[y * FRAME_W + x] < thr;
    }
    if (bright * 6 > n) return;
    const strack_camera *c = &t->cam[cam];
    if (b.x >= c->width || b.y >= c->height) return;
    strack_unproject(c, b.x, b.y, b.ray);
    if (b.ray[2] < cosf(c->max_angle_deg * DEG)) return;
    add_blob(t, cam, &b);
}

int strack_detect_gray(strack *t, int cam, const uint8_t *pixels, int stride) {
    t->nblobs[cam] = 0;
    for (int y = 0; y < FRAME_H; y++) memcpy(t->gray + y * FRAME_W, pixels + (size_t)y * stride, FRAME_W);
    memset(t->seen, 0, FRAME_W * FRAME_H);
    uint8_t thr = (uint8_t)fminf(255, fmaxf(1, t->prm.threshold));
    for (int i = 0; i < FRAME_W * FRAME_H; i++)
        if (t->gray[i] >= thr && !t->seen[i]) fill_blob(t, cam, i % FRAME_W, i / FRAME_W);
    return t->nblobs[cam];
}

// One BC4 block's 16 values.
static void bc4_block(const uint8_t *b, uint8_t out[16]) {
    uint8_t p[8];
    int r0 = b[0], r1 = b[1];
    p[0] = r0;
    p[1] = r1;
    if (r0 > r1) {
        for (int i = 1; i < 7; i++) p[i + 1] = (uint8_t)(((7 - i) * r0 + i * r1 + 3) / 7);
    } else {
        for (int i = 1; i < 5; i++) p[i + 1] = (uint8_t)(((5 - i) * r0 + i * r1 + 2) / 5);
        p[6] = 0;
        p[7] = 255;
    }
    uint64_t bits = 0;
    for (int i = 0; i < 6; i++) bits |= (uint64_t)b[2 + i] << (8 * i);
    for (int i = 0; i < 16; i++) out[i] = p[(bits >> (3 * i)) & 7];
}

int strack_detect_bc4(strack *t, int cam, const uint8_t *bc4, int stride_blocks) {
    t->nblobs[cam] = 0;
    memset(t->gray, 0, FRAME_W * FRAME_H);
    memset(t->seen, 0, FRAME_W * FRAME_H);
    int thr = (int)fminf(255, fmaxf(1, t->prm.threshold)), ring = (int)(RING_LEVEL * thr);
    int *hot = t->hot, nhot = 0;
    for (int by = 0; by < FRAME_H / 4; by++)
        for (int bx = 0; bx < FRAME_W / 4 && bx < stride_blocks; bx++) {
            const uint8_t *b = bc4 + ((size_t)by * stride_blocks + bx) * 8;
            // A block's brightest value is one of its end points, or 255 in the six-value mode. Blocks
            // nearly as bright are decoded too, for fill_blob() to see what's around a dot.
            if (b[0] < ring && b[1] < ring && b[0] > b[1]) continue;
            uint8_t px[16];
            bc4_block(b, px);
            bool any = false;
            for (int i = 0; i < 16; i++) {
                t->gray[(by * 4 + i / 4) * FRAME_W + bx * 4 + i % 4] = px[i];
                any |= px[i] >= thr;
            }
            if (any) hot[nhot++] = by * (FRAME_W / 4) + bx;
        }
    for (int k = 0; k < nhot; k++) {
        int bx = hot[k] % (FRAME_W / 4), by = hot[k] / (FRAME_W / 4);
        for (int i = 0; i < 16; i++) {
            int x = bx * 4 + i % 4, y = by * 4 + i / 4;
            if (t->gray[y * FRAME_W + x] >= thr && !t->seen[y * FRAME_W + x]) fill_blob(t, cam, x, y);
        }
    }
    return t->nblobs[cam];
}

const strack_blob *strack_blobs(const strack *t, int cam, int *count) {
    *count = t->nblobs[cam];
    return t->blobs[cam];
}

// --------------------------------------------------------------------------------- pose fitting


typedef struct {
    const strack *t;
    int hand;
    float prior[9];             // the IMU's orientation of the controller, in the head
    float up[3];                // the world's up, in the head
    float w_tilt, w_yaw;        // the prior's weight, in pixels per radian
} problem;

// Where LED i lands in camera c for pose P; false if it faces away or falls outside the picture.
static bool led_pixel(const strack *t, int hand, int c, const hpose *P, int i, float *u, float *v) {
    float w[3], pc[3], n[3], nc[3];
    mv(P->R, t->led[hand][i], w);
    for (int k = 0; k < 3; k++) w[k] += P->p[k] - t->cam[c].position[k];
    mtv(t->Rhc[c], w, pc);
    mv(P->R, t->normal[hand][i], n);
    mtv(t->Rhc[c], n, nc);
    float d = norm3(pc);
    if (d < 0.05f || -dot3(nc, pc) / d < 0.2f) return false;   // within about 78 degrees of facing
    return strack_project(&t->cam[c], pc, u, v);
}

typedef struct { int led, blob; float d2; } cand;

static int cmp_cand(const void *a, const void *b) {
    float x = ((const cand *)a)->d2, y = ((const cand *)b)->d2;
    return x < y ? -1 : x > y;
}

// LEDs matched to the nearest unclaimed dots within gate pixels, each dot and LED used once, in the
// cameras whose bits are set in cams.
static int match_pose(const strack *t, int hand, const hpose *P, float gate, int cams, match *out, float *sse) {
    int n = 0;
    *sse = 0;
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        if (!(cams & 1 << c)) continue;
        cand cs[STRACK_LEDS * 4];
        int nc = 0;
        for (int i = 0; i < STRACK_LEDS; i++) {
            float u, v;
            if (!led_pixel(t, hand, c, P, i, &u, &v)) continue;
            for (int b = 0; b < t->nblobs[c]; b++) {
                if (t->claimed[c][b]) continue;
                float du = t->blobs[c][b].x - u, dv = t->blobs[c][b].y - v, d2 = du * du + dv * dv;
                if (d2 < gate * gate && nc < (int)(sizeof cs / sizeof *cs)) cs[nc++] = (cand){i, b, d2};
            }
        }
        qsort(cs, nc, sizeof *cs, cmp_cand);
        uint32_t used_led = 0;
        uint64_t used_blob = 0;
        for (int k = 0; k < nc; k++) {
            if (used_led & (1u << cs[k].led) || used_blob & (1ull << cs[k].blob)) continue;
            used_led |= 1u << cs[k].led;
            used_blob |= 1ull << cs[k].blob;
            out[n++] = (match){c, cs[k].led, cs[k].blob, 0, 0};
            *sse += cs[k].d2;
        }
    }
    return n;
}

// The residuals: each match's pixel error (softened past 2 pixels), then the prior's tilt and turn.
static int residuals(const problem *pr, const hpose *P, const match *m, int n, double *r) {
    const strack *t = pr->t;
    int k = 0;
    for (int i = 0; i < n; i++) {
        float u, v, w[3], pc[3];
        mv(P->R, t->led[pr->hand][m[i].led], w);
        for (int j = 0; j < 3; j++) w[j] += P->p[j] - t->cam[m[i].cam].position[j];
        mtv(t->Rhc[m[i].cam], w, pc);
        if (pc[2] < 1e-3f) pc[2] = 1e-3f;
        strack_project(&t->cam[m[i].cam], pc, &u, &v);
        double du = u - t->blobs[m[i].cam][m[i].blob].x, dv = v - t->blobs[m[i].cam][m[i].blob].y;
        double e = sqrt(du * du + dv * dv), s = e > 2 ? sqrt(2 / e) : 1;   // Huber
        r[k++] = du * s;
        r[k++] = dv * s;
    }
    float D[9], e[3];
    mmt(P->R, pr->prior, D);
    log_map(D, e);
    float ey = dot3(e, pr->up);
    for (int j = 0; j < 3; j++) r[k++] = (e[j] - ey * pr->up[j]) * pr->w_tilt;
    r[k++] = ey * pr->w_yaw;
    return k;
}

static void apply_step(const hpose *P, const double *x, hpose *out) {
    float w[3] = {(float)x[0], (float)x[1], (float)x[2]}, dR[9];
    exp_map(w, dR);
    mm(dR, P->R, out->R);
    for (int j = 0; j < 3; j++) out->p[j] = P->p[j] + (float)x[3 + j];
}

static double cost(const double *r, int k) {
    double s = 0;
    for (int i = 0; i < k; i++) s += r[i] * r[i];
    return s;
}

// Levenberg-Marquardt over the turn and position, with numerical derivatives.
static void refine(const problem *pr, hpose *P, const match *m, int n, int iterations) {
    enum { MAXR = 2 * MAX_MATCHES + 4 };
    double r0[MAXR], r1[MAXR], J[MAXR][6];
    double lambda = 1e-3;
    int k = residuals(pr, P, m, n, r0);
    double c0 = cost(r0, k);
    for (int it = 0; it < iterations; it++) {
        for (int j = 0; j < 6; j++) {
            double x[6] = {0}, h = j < 3 ? 1e-4 : 1e-5;
            x[j] = h;
            hpose Q;
            apply_step(P, x, &Q);
            residuals(pr, &Q, m, n, r1);
            for (int i = 0; i < k; i++) J[i][j] = (r1[i] - r0[i]) / h;
        }
        double A[36] = {0}, g[6] = {0};
        for (int i = 0; i < k; i++)
            for (int a = 0; a < 6; a++) {
                g[a] -= J[i][a] * r0[i];
                for (int b = 0; b < 6; b++) A[a * 6 + b] += J[i][a] * J[i][b];
            }
        for (int tries = 0; tries < 4; tries++) {
            double M[36], x[6];
            memcpy(M, A, sizeof M);
            memcpy(x, g, sizeof x);
            for (int a = 0; a < 6; a++) M[a * 6 + a] += lambda * (A[a * 6 + a] + 1e-6);
            if (!solve_n(6, M, x)) { lambda *= 10; continue; }
            hpose Q;
            apply_step(P, x, &Q);
            int k1 = residuals(pr, &Q, m, n, r1);
            double c1 = cost(r1, k1);
            if (c1 < c0) {
                *P = Q;
                memcpy(r0, r1, sizeof(double) * k1);
                c0 = c1;
                lambda = fmax(1e-7, lambda / 10);
                break;
            }
            lambda *= 10;
        }
    }
}

typedef struct { hpose P; int n; float sse; } hypothesis;

static void keep_hypothesis(hypothesis *best, int *nbest, const hpose *P, int n, float sse) {
    int i;
    if (*nbest < HYPOTHESES) {
        i = (*nbest)++;
    } else {
        const hypothesis *w = &best[HYPOTHESES - 1];
        if (n < w->n || (n == w->n && sse >= w->sse)) return;
        i = HYPOTHESES - 1;
    }
    while (i > 0 && (best[i - 1].n < n || (best[i - 1].n == n && best[i - 1].sse > sse))) { best[i] = best[i - 1]; i--; }
    best[i] = (hypothesis){*P, n, sse};
}

// Fits a pose starting from P with the cameras in cams: match within a wide gate, refine, match
// again closely, refine. Returns the LEDs matched at the end, the matches in m and the root mean
// square error in rms.
static int polish_with(const problem *pr, hpose *P, float wide_gate, int cams, match *m, float *rms) {
    const strack *t = pr->t;
    float sse;
    int n = match_pose(t, pr->hand, P, wide_gate, cams, m, &sse);
    if (n < 2) return 0;
    refine(pr, P, m, n, 6);
    n = match_pose(t, pr->hand, P, t->prm.gate_px, cams, m, &sse);
    if (n < 2) return 0;
    refine(pr, P, m, n, 4);
    n = match_pose(t, pr->hand, P, t->prm.gate_px, cams, m, &sse);
    *rms = n ? sqrtf(sse / n) : 1e9f;
    return n;
}

// Both cameras together; or, if they disagree (which they do while the guess of where they point
// is off, until calibrate() has put it right), whichever camera alone fits best.
static int polish(const problem *pr, hpose *P, float wide_gate, match *m, float *rms) {
    const strack *t = pr->t;
    hpose start = *P;
    int n = polish_with(pr, P, wide_gate, 3, m, rms);
    if (n >= 2 && *rms <= t->prm.max_rms_px) return n;
    int best = n;
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        hpose Q = start;
        match mc[MAX_MATCHES];
        float r;
        int k = polish_with(pr, &Q, wide_gate, 1 << c, mc, &r);
        if (k < 2 || r > t->prm.max_rms_px) continue;
        if (*rms <= t->prm.max_rms_px * 1.4f && best >= k) continue;   // the joint fit is about as good
        if (best >= 2 && *rms <= t->prm.max_rms_px && best >= k) continue;
        best = k;
        *rms = r;
        *P = Q;
        memcpy(m, mc, sizeof(match) * k);
    }
    return best;
}

// Every dot near another, every pair of neighbouring LEDs, and every turn about the vertical that
// the prior might be off by: the two LEDs on the two dots fix a position (the turn is known), and
// the poses that put the most other LEDs on dots are refined.
static int search(const problem *base, bool locked, hpose *found, match *m, float *rms) {
    const strack *t = base->t;
    hypothesis best[HYPOTHESES];
    int nbest = 0;
    // Unconfirmed, the prior's yaw is still roughly right: the runtime turns a controller to point
    // where the head faces when it connects and when its PS button is pressed.
    int steps = locked ? 3 : 19;
    float step = locked ? 8 * DEG : 10 * DEG;
    for (int s = 0; s < steps; s++) {
        float yaw = (s - steps / 2) * step, Ryaw[9];
        hpose P;
        axis_angle(base->up, yaw, Ryaw);
        mm(Ryaw, base->prior, P.R);
        for (int c = 0; c < STRACK_CAMERAS; c++) {
            float Rc[9];   // the LED model's axes in the camera's
            mtm(t->Rhc[c], P.R, Rc);
            const strack_blob *B = t->blobs[c];
            int nb = t->nblobs[c];
            for (int i = 0; i < nb; i++) {
                if (t->claimed[c][i]) continue;
                // Its four nearest unclaimed dots.
                int near[4], nn = 0;
                float nd[4];
                for (int j = 0; j < nb; j++) {
                    if (j == i || t->claimed[c][j]) continue;
                    float dx = B[j].x - B[i].x, dy = B[j].y - B[i].y, d2 = dx * dx + dy * dy;
                    if (d2 > 220 * 220) continue;
                    int k;
                    if (nn < 4) k = nn++;
                    else if (nd[3] <= d2) continue;
                    else k = 3;
                    while (k > 0 && nd[k - 1] > d2) { near[k] = near[k - 1]; nd[k] = nd[k - 1]; k--; }
                    near[k] = j;
                    nd[k] = d2;
                }
                for (int q = 0; q < nn; q++) {
                    int j = near[q];
                    const float *ri = B[i].ray, *rj = B[j].ray;
                    float cij = dot3(ri, rj), det = 1 - cij * cij;
                    if (det < 1e-7f) continue;
                    for (int pidx = 0; pidx < t->npairs; pidx++) {
                        int a = t->pairs[pidx][0], b = t->pairs[pidx][1];
                        float na[3], nb2[3];
                        mv(Rc, t->normal[base->hand][a], na);
                        if (-dot3(na, ri) < 0.2f) continue;
                        mv(Rc, t->normal[base->hand][b], nb2);
                        if (-dot3(nb2, rj) < 0.2f) continue;
                        float dl[3], d[3];
                        for (int k = 0; k < 3; k++) dl[k] = t->led[base->hand][b][k] - t->led[base->hand][a][k];
                        mv(Rc, dl, d);
                        // Depths si, sj with sj rj - si ri = d, by least squares.
                        float rid = dot3(ri, d), rjd = dot3(rj, d);
                        float si = (-rid + cij * rjd) / det, sj = (rjd - cij * rid) / det;
                        if (si < 0.08f || sj < 0.08f || si > 1.6f || sj > 1.6f) continue;
                        float e[3];
                        for (int k = 0; k < 3; k++) e[k] = sj * rj[k] - si * ri[k] - d[k];
                        if (norm3(e) > 0.2f * norm3(d) + 0.002f) continue;
                        // The model's origin in the camera, then in the head.
                        float la[3], oc[3];
                        mv(Rc, t->led[base->hand][a], la);
                        for (int k = 0; k < 3; k++) oc[k] = si * ri[k] - la[k];
                        mv(t->Rhc[c], oc, P.p);
                        for (int k = 0; k < 3; k++) P.p[k] += t->cam[c].position[k];
                        match mm2[MAX_MATCHES];
                        float sse;
                        int n = match_pose(t, base->hand, &P, t->prm.gate_px * 1.6f, 3, mm2, &sse);
                        if (n >= 3) keep_hypothesis(best, &nbest, &P, n, sse);
                    }
                }
            }
        }
    }
    int best_n = 0;
    float best_rms = 1e9f;
    for (int h = 0; h < nbest; h++) {
        hpose P = best[h].P;
        match mm2[MAX_MATCHES];
        float r;
        int n = polish(base, &P, t->prm.gate_px * 1.6f, mm2, &r);
        if (n > best_n || (n == best_n && r < best_rms)) {
            best_n = n;
            best_rms = r;
            *found = P;
            memcpy(m, mm2, sizeof(match) * n);
        }
    }
    *rms = best_rms;
    return best_n;
}

static void world_from_head(const strack_input *in, const hpose *P, float R[9], float p[3]) {
    float Rh[9];
    quat_to_mat(in->head.q, Rh);
    mm(Rh, P->R, R);
    mv(Rh, P->p, p);
    for (int k = 0; k < 3; k++) p[k] += in->head.p[k];
}

static void head_from_world(const strack_input *in, const float R[9], const float p[3], hpose *P) {
    float Rh[9], d[3];
    quat_to_mat(in->head.q, Rh);
    mtm(Rh, R, P->R);
    for (int k = 0; k < 3; k++) d[k] = p[k] - in->head.p[k];
    mtv(Rh, d, P->p);
}

// --------------------------------------------------------------------------------- calibration
//
// The guess of where the cameras point and how wide they see is only a guess, and a couple of
// degrees off puts a controller centimetres off and makes the two cameras disagree. So frames where
// a controller is well seen are kept, and every few seconds the cameras' turn (yaw, pitch, roll)
// and focal length are fitted to them together with each frame's controller pose: the LEDs' known
// layout fixes the focal length, the IMU's sense of gravity fixes each camera's pitch and roll, and
// a controller both cameras see at once ties their yaws together.

enum { CP = 6, NCP = STRACK_CAMERAS * CP };   // per camera: yaw, pitch, roll, log f, cx, cy

// How far off the starting cameras may be: 3 degrees of turn, 3% of the focal length and 8 pixels of
// the centre. The calibration is held to them by that much; looser, it wandered by 20% and 15
// degrees in the headset, trading focal length against distance, and the controllers jumped.
static const float k_cal_sigma[CP] = {3 * DEG, 3 * DEG, 3 * DEG, 0.03f, 8, 8};

static void camera_with(const strack_camera *base, const float d[CP], strack_camera *out, float R[9]) {
    *out = *base;
    out->yaw_deg += d[0] / DEG;
    out->pitch_deg += d[1] / DEG;
    out->roll_deg += d[2] / DEG;
    out->f *= expf(d[3]);
    out->cx += d[4];
    out->cy += d[5];
    camera_rotation(out, R);
}

// A kept frame's residuals with cameras cams/Rhc and the controller at P: pixel errors, then the
// IMU prior as in residuals().
static int sample_residuals(const strack *t, const strack_camera *cams, const float (*Rhc)[9], const cal_sample *cs,
                            const hpose *P, double *r) {
    int k = 0;
    for (int i = 0; i < cs->n; i++) {
        int c = cs->o[i].cam;
        float w[3], pc[3], u, v;
        mv(P->R, t->led[cs->hand][cs->o[i].led], w);
        for (int j = 0; j < 3; j++) w[j] += P->p[j] - cams[c].position[j];
        mtv(Rhc[c], w, pc);
        if (pc[2] < 1e-3f) pc[2] = 1e-3f;
        strack_project(&cams[c], pc, &u, &v);
        double du = u - cs->o[i].u, dv = v - cs->o[i].v, e = sqrt(du * du + dv * dv), sc = e > 2 ? sqrt(2 / e) : 1;
        r[k++] = du * sc;
        r[k++] = dv * sc;
    }
    float D[9], e[3];
    mmt(P->R, cs->prior, D);
    log_map(D, e);
    float ey = dot3(e, cs->up);
    for (int j = 0; j < 3; j++) r[k++] = (e[j] - ey * cs->up[j]) * 30;
    r[k++] = ey * 2;
    return k;
}

static double calibration_cost(const strack *t, const float d[STRACK_CAMERAS][CP], const hpose *poses, double *rms) {
    strack_camera cams[STRACK_CAMERAS];
    float Rhc[STRACK_CAMERAS][9];
    for (int c = 0; c < STRACK_CAMERAS; c++) camera_with(&t->base[c], d[c], &cams[c], Rhc[c]);
    double r[2 * MAX_MATCHES + 4], total = 0, px = 0;
    int npx = 0;
    for (int i = 0; i < t->ncal; i++) {
        int k = sample_residuals(t, cams, (const float (*)[9])Rhc, &t->cal[i], &poses[i], r);
        for (int j = 0; j < k; j++) total += r[j] * r[j];
        for (int j = 0; j < 2 * t->cal[i].n; j++) px += r[j] * r[j];
        npx += t->cal[i].n;
    }
    for (int c = 0; c < STRACK_CAMERAS; c++)
        for (int j = 0; j < CP; j++) total += (d[c][j] / k_cal_sigma[j]) * (d[c][j] / k_cal_sigma[j]);
    if (rms) *rms = npx ? sqrt(px / npx) : 0;
    return total;
}

// Solves A X = B for m right-hand sides (B is n x m, row-major), in place. False if singular.
static bool solve_multi(int n, double *A, double *B, int m) {
    for (int c = 0; c < n; c++) {
        int p = c;
        for (int r = c + 1; r < n; r++) if (fabs(A[r * n + c]) > fabs(A[p * n + c])) p = r;
        if (fabs(A[p * n + c]) < 1e-12) return false;
        if (p != c) {
            for (int k = 0; k < n; k++) { double x = A[c * n + k]; A[c * n + k] = A[p * n + k]; A[p * n + k] = x; }
            for (int k = 0; k < m; k++) { double x = B[c * m + k]; B[c * m + k] = B[p * m + k]; B[p * m + k] = x; }
        }
        for (int r = c + 1; r < n; r++) {
            double f = A[r * n + c] / A[c * n + c];
            for (int k = c; k < n; k++) A[r * n + k] -= f * A[c * n + k];
            for (int k = 0; k < m; k++) B[r * m + k] -= f * B[c * m + k];
        }
    }
    for (int c = n - 1; c >= 0; c--)
        for (int k = 0; k < m; k++) {
            for (int j = c + 1; j < n; j++) B[c * m + k] -= A[c * n + j] * B[j * m + k];
            B[c * m + k] /= A[c * n + c];
        }
    return true;
}

typedef struct {
    double Acs[NCP][6];     // the cameras' columns against this frame's pose columns
    double Ass[6][6], gs[6];
} cal_block;

// Levenberg-Marquardt over the cameras and every kept frame's pose. The poses only meet through
// the cameras, so each frame's block is eliminated (a Schur complement) and only the cameras'
// small system is solved: milliseconds, so it can run between camera frames.
static void calibrate(strack *t) {
    int N = t->ncal;
    cal_block *blk = malloc(sizeof(cal_block) * N);
    hpose *poses = malloc(sizeof(hpose) * N), *trial = malloc(sizeof(hpose) * N);
    double (*X)[6][NCP + 1] = malloc(sizeof(double) * N * 6 * (NCP + 1));   // each block solved against [Acs^T | gs]
    if (!blk || !poses || !trial || !X) goto out;
    for (int i = 0; i < N; i++) poses[i] = t->cal[i].P;
    float d[STRACK_CAMERAS][CP];
    memcpy(d, t->delta, sizeof d);
    double rms0, rms1, c0 = calibration_cost(t, d, poses, &rms0), lambda = 1e-3;
    for (int it = 0; it < 20; it++) {
        double Acc[NCP][NCP] = {{0}}, gc[NCP] = {0};
        strack_camera cams[STRACK_CAMERAS], cams2[STRACK_CAMERAS];
        float Rhc[STRACK_CAMERAS][9], Rhc2[STRACK_CAMERAS][9];
        for (int c = 0; c < STRACK_CAMERAS; c++) camera_with(&t->base[c], d[c], &cams[c], Rhc[c]);
        for (int i = 0; i < N; i++) {
            const cal_sample *cs = &t->cal[i];
            cal_block *B = &blk[i];
            memset(B, 0, sizeof *B);
            double r0[2 * MAX_MATCHES + 4], r1[2 * MAX_MATCHES + 4], Jc[2 * MAX_MATCHES + 4][NCP] = {{0}},
                                                                     Js[2 * MAX_MATCHES + 4][6];
            int k = sample_residuals(t, cams, (const float (*)[9])Rhc, cs, &poses[i], r0);
            bool uses[STRACK_CAMERAS] = {false};
            for (int o = 0; o < cs->n; o++) uses[cs->o[o].cam] = true;
            for (int q = 0; q < NCP; q++) {
                int c = q / CP;
                if (!uses[c]) continue;
                float d2[CP];
                memcpy(d2, d[c], sizeof d2);
                double h = q % CP >= 4 ? 1e-2 : 1e-4;
                d2[q % CP] += (float)h;
                memcpy(cams2, cams, sizeof cams2);
                memcpy(Rhc2, Rhc, sizeof Rhc2);
                camera_with(&t->base[c], d2, &cams2[c], Rhc2[c]);
                sample_residuals(t, cams2, (const float (*)[9])Rhc2, cs, &poses[i], r1);
                for (int j = 0; j < k; j++) Jc[j][q] = (r1[j] - r0[j]) / h;
            }
            for (int q = 0; q < 6; q++) {
                double x[6] = {0}, h = q < 3 ? 1e-4 : 1e-5;
                x[q] = h;
                hpose Q;
                apply_step(&poses[i], x, &Q);
                sample_residuals(t, cams, (const float (*)[9])Rhc, cs, &Q, r1);
                for (int j = 0; j < k; j++) Js[j][q] = (r1[j] - r0[j]) / h;
            }
            for (int j = 0; j < k; j++) {
                for (int a = 0; a < NCP; a++) {
                    if (Jc[j][a] == 0) continue;
                    gc[a] -= Jc[j][a] * r0[j];
                    for (int b = 0; b < NCP; b++) Acc[a][b] += Jc[j][a] * Jc[j][b];
                    for (int b = 0; b < 6; b++) B->Acs[a][b] += Jc[j][a] * Js[j][b];
                }
                for (int a = 0; a < 6; a++) {
                    B->gs[a] -= Js[j][a] * r0[j];
                    for (int b = 0; b < 6; b++) B->Ass[a][b] += Js[j][a] * Js[j][b];
                }
            }
        }
        for (int q = 0; q < NCP; q++) {
            double jq = 1 / k_cal_sigma[q % CP];
            gc[q] -= jq * (d[q / CP][q % CP] * jq);
            Acc[q][q] += jq * jq;
        }
        bool improved = false;
        for (int tries = 0; tries < 6 && !improved; tries++) {
            double S[NCP * NCP], rc[NCP];
            for (int a = 0; a < NCP; a++) {
                rc[a] = gc[a];
                for (int b = 0; b < NCP; b++) S[a * NCP + b] = Acc[a][b] + (a == b ? lambda * (Acc[a][a] + 1e-6) : 0);
            }
            // Each frame's block, damped, solved against [Acs^T | gs], and taken off the cameras'.
            bool ok = true;
            for (int i = 0; i < N && ok; i++) {
                double M[36];
                for (int a = 0; a < 6; a++)
                    for (int b = 0; b < 6; b++) M[a * 6 + b] = blk[i].Ass[a][b] + (a == b ? lambda * (blk[i].Ass[a][a] + 1e-6) : 0);
                for (int a = 0; a < 6; a++) {
                    for (int q = 0; q < NCP; q++) X[i][a][q] = blk[i].Acs[q][a];
                    X[i][a][NCP] = blk[i].gs[a];
                }
                ok = solve_multi(6, M, &X[i][0][0], NCP + 1);
                for (int a = 0; a < NCP && ok; a++) {
                    for (int b = 0; b < NCP; b++) {
                        double v = 0;
                        for (int q = 0; q < 6; q++) v += blk[i].Acs[a][q] * X[i][q][b];
                        S[a * NCP + b] -= v;
                    }
                    double v = 0;
                    for (int q = 0; q < 6; q++) v += blk[i].Acs[a][q] * X[i][q][NCP];
                    rc[a] -= v;
                }
            }
            if (ok && solve_multi(NCP, S, rc, 1)) {
                float d2[STRACK_CAMERAS][CP];
                for (int q = 0; q < NCP; q++) d2[q / CP][q % CP] = d[q / CP][q % CP] + (float)rc[q];
                for (int i = 0; i < N; i++) {
                    double xs[6];
                    for (int a = 0; a < 6; a++) {
                        xs[a] = X[i][a][NCP];
                        for (int q = 0; q < NCP; q++) xs[a] -= X[i][a][q] * rc[q];
                    }
                    apply_step(&poses[i], xs, &trial[i]);
                }
                double c1 = calibration_cost(t, d2, trial, NULL);
                if (c1 < c0) {
                    memcpy(d, d2, sizeof d);
                    memcpy(poses, trial, sizeof(hpose) * N);
                    c0 = c1;
                    lambda = fmax(1e-7, lambda / 10);
                    improved = true;
                }
            }
            if (!improved) lambda *= 10;
        }
        if (!improved) break;
    }
    calibration_cost(t, d, poses, &rms1);
    // Kept only if the frames now fit well, and better than before.
    if (rms1 < rms0 && rms1 < t->prm.max_rms_px) {
        memcpy(t->delta, d, sizeof d);
        for (int c = 0; c < STRACK_CAMERAS; c++) camera_with(&t->base[c], d[c], &t->cam[c], t->Rhc[c]);
        for (int i = 0; i < N; i++) t->cal[i].P = poses[i];
        t->cal_runs++;
    }
    t->cal_rms = (float)(rms1 < rms0 ? rms1 : rms0);
out:
    free(X);
    free(blk);
    free(poses);
    free(trial);
}

// Keeps a frame for calibration: a controller matched to at least six dots, every so often.
static void keep_sample(strack *t, const strack_input *in, int h, const problem *pr, const hpose *P, const match *m, int n) {
    if (n < 6 || in->time_ns - t->last_sample_ns < 150000000) return;
    t->last_sample_ns = in->time_ns;
    cal_sample *cs = &t->cal[t->cal_next];
    t->cal_next = (t->cal_next + 1) % CAL_SAMPLES;
    if (t->ncal < CAL_SAMPLES) t->ncal++;
    t->cal_new++;
    cs->hand = h;
    cs->n = n;
    memcpy(cs->prior, pr->prior, sizeof cs->prior);
    memcpy(cs->up, pr->up, sizeof cs->up);
    cs->P = *P;
    for (int i = 0; i < n; i++) {
        cs->o[i].cam = m[i].cam;
        cs->o[i].led = m[i].led;
        cs->o[i].u = t->blobs[m[i].cam][m[i].blob].x;
        cs->o[i].v = t->blobs[m[i].cam][m[i].blob].y;
    }
}

void strack_get_cameras(const strack *t, strack_camera out[STRACK_CAMERAS]) {
    memcpy(out, t->cam, sizeof t->cam);
}

int strack_calibration(const strack *t, float *rms_px) {
    if (rms_px) *rms_px = t->cal_rms;
    return t->cal_runs;
}

// Every dot within the controller's reach of where it is, in each camera: its LEDs at a slant
// that weren't matched, glints off it, so that a search for the other controller leaves them be.
static void claim_around(strack *t, const hpose *P) {
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        float w[3], pc[3];
        for (int k = 0; k < 3; k++) w[k] = P->p[k] - t->cam[c].position[k];
        mtv(t->Rhc[c], w, pc);
        float d = norm3(pc);
        if (d < 0.05f) continue;
        float reach = cosf(atanf(0.12f / d));
        for (int b = 0; b < t->nblobs[c]; b++)
            if (dot3(t->blobs[c][b].ray, pc) / d > reach) t->claimed[c][b] = true;
    }
}

typedef struct {
    problem pr;
    bool locked, following;
    float Rw[9], Rc[9];     // the prior as given, and as corrected by the tracker's yaw
    float dt;
} hand_job;

static void accept(strack *t, const strack_input *in, int h, hand_job *j, const hpose *P, const match *m, int n,
                   float rms, bool found, strack_result *o) {
    hand_state *hs = &t->hs[h];
    // For calibration, the LEDs as matched in both cameras, even when the pose came from one.
    match all[MAX_MATCHES];
    float sse;
    int na = match_pose(t, h, P, t->prm.gate_px * 3, 3, all, &sse);
    if (na >= n) keep_sample(t, in, h, &j->pr, P, all, na);
    else keep_sample(t, in, h, &j->pr, P, m, n);
    for (int i = 0; i < n; i++) t->claimed[m[i].cam][m[i].blob] = true;
    claim_around(t, P);
    float R[9], p[3], D[9], e[3];
    world_from_head(in, P, R, p);
    if (j->following && j->dt > 1e-3f) {
        for (int k = 0; k < 3; k++) {
            float v = (p[k] - hs->p[k]) / j->dt;
            hs->v[k] = 0.5f * hs->v[k] + 0.5f * fmaxf(-8, fminf(8, v));
        }
    } else {
        memset(hs->v, 0, sizeof hs->v);
    }
    mmt(R, j->Rw, D);
    log_map(D, e);
    hs->yaw_error = e[1];
    hs->tracking = true;
    hs->leds = n;
    hs->t_ns = in->time_ns;
    memcpy(hs->R, R, sizeof R);
    memcpy(hs->p, p, sizeof p);
    hs->locked_until_ns = in->time_ns + 2000000000LL;
    o->valid = true;
    o->found = found;
    mat_to_quat(R, o->pose.q);
    memcpy(o->pose.p, p, sizeof p);
    o->yaw_error = e[1];
    o->leds = n;
    o->rms_px = rms;
}

static bool search_hand(strack *t, hand_job *j, hpose *P, match *m, int *n, float *rms) {
    *rms = 1e9f;
    *n = search(&j->pr, j->locked, P, m, rms);
    return *n >= (j->locked ? 4 : t->prm.min_leds) && *rms <= t->prm.max_rms_px;
}

void strack_solve(strack *t, const strack_input *in, strack_result out[2]) {
    memset(out, 0, 2 * sizeof *out);
    memset(t->claimed, 0, sizeof t->claimed);
    float Rh[9], upw[3] = {0, 1, 0};
    quat_to_mat(in->head.q, Rh);
    hand_job jobs[2];
    for (int h = 0; h < 2; h++) {
        hand_state *hs = &t->hs[h];
        hand_job *j = &jobs[h];
        if (!in->prior_valid[h]) { hs->tracking = false; continue; }
        // The IMU's orientation, turned by what the last fits found its yaw to be off by while that
        // is fresh; then the fit holds to its yaw nearly as firmly as to its tilt.
        j->locked = in->time_ns < hs->locked_until_ns;
        j->pr = (problem){.t = t, .hand = h, .w_tilt = 30, .w_yaw = j->locked ? 12.0f : 2.0f};
        float Ryaw[9];
        quat_to_mat(in->prior_q[h], j->Rw);
        if (!j->locked) hs->yaw_error = 0;
        axis_angle(upw, hs->yaw_error, Ryaw);
        mm(Ryaw, j->Rw, j->Rc);
        mtm(Rh, j->Rc, j->pr.prior);
        mtv(Rh, upw, j->pr.up);
        j->dt = (in->time_ns - hs->t_ns) / 1e9f;
        j->following = hs->tracking && j->dt > 0 && j->dt < 0.25f;
        if (!j->following) hs->tracking = false;
    }

    // Followed controllers first, so a search for another can't take their dots. A follow looks
    // where the controller should be now: moved on at its last speed, turned as its IMU says. With
    // the orientation known, two LEDs are enough for the position.
    // The one seen better last time goes first.
    int order[2] = {0, 1};
    if (t->hs[1].leds > t->hs[0].leds) { order[0] = 1; order[1] = 0; }
    for (int oi = 0; oi < 2; oi++) {
        int h = order[oi];
        hand_job *j = &jobs[h];
        if (!in->prior_valid[h] || !j->following) continue;
        hpose P;
        match m[MAX_MATCHES];
        float p[3], rms = 1e9f;
        for (int k = 0; k < 3; k++) p[k] = t->hs[h].p[k] + t->hs[h].v[k] * j->dt;
        head_from_world(in, j->Rc, p, &P);
        hpose start = P;
        int n = polish(&j->pr, &P, t->prm.gate_px * 3, m, &rms);
        bool two_ok = t->hs[h].leds >= 4;   // two LEDs only right after seeing plenty
        if (!(n >= 3 || (n == 2 && two_ok)) || rms > t->prm.max_rms_px * (n == 2 ? 0.6f : 1.4f)) continue;
        // A hand doesn't jump: a fit far from where it was going is on someone else's dots.
        float jump[3];
        for (int k = 0; k < 3; k++) jump[k] = P.p[k] - start.p[k];
        if (norm3(jump) > 0.06f + 3 * j->dt) continue;
        accept(t, in, h, j, &P, m, n, rms, false, &out[h]);
    }

    // The rest are searched for among the dots left, a few times a second. One not seen for a
    // moment is kept for a quarter of a second, to be looked for near where it was. Each hand is
    // searched for on the same dots first: the left controller's LEDs are the right one's mirror
    // image, so a hand that can't be seen often fits the other's dots fairly well. Where both fit
    // the same dots, the better fit has them, and the other hand looks again among what's left.
    bool due[2] = {false, false};
    for (int h = 0; h < 2; h++) {
        hand_state *hs = &t->hs[h];
        if (!in->prior_valid[h] || out[h].valid) continue;
        due[h] = !hs->searched || in->time_ns - hs->last_search_ns >= (int64_t)(1e9f / fmaxf(0.5f, t->prm.search_hz));
        if (!due[h]) continue;
        hs->searched = true;
        hs->last_search_ns = in->time_ns;
    }
    hpose sp[2];
    match sm[2][MAX_MATCHES];
    int sn[2] = {0, 0};
    float srms[2] = {1e9f, 1e9f};
    for (int h = 0; h < 2; h++)
        if (due[h] && !search_hand(t, &jobs[h], &sp[h], sm[h], &sn[h], &srms[h])) sn[h] = 0;
    int first = sn[1] > sn[0] || (sn[1] == sn[0] && srms[1] < srms[0]);
    for (int k = 0; k < 2; k++) {
        int h = k ? !first : first;
        if (!sn[h]) continue;
        if (k && sn[!h]) {
            // The better one was taken; if it was on these dots, look again without them.
            float d[3];
            for (int q = 0; q < 3; q++) d[q] = sp[h].p[q] - sp[!h].p[q];
            if (norm3(d) < 0.15f && !search_hand(t, &jobs[h], &sp[h], sm[h], &sn[h], &srms[h])) continue;
        }
        // Dots the other hand fits at least as well are its, out of sight of its own search.
        if (!out[!h].valid && in->prior_valid[!h]) {
            hpose Q = sp[h];
            match mq[MAX_MATCHES];
            float rq;
            mtm(Rh, jobs[!h].Rc, Q.R);   // the other one, turned as its IMU says, here
            int nq = polish(&jobs[!h].pr, &Q, t->prm.gate_px * 3, mq, &rq);
            if (nq >= sn[h] && rq <= t->prm.max_rms_px * 1.5f) continue;
        }
        // Not where the other one is.
        if (out[!h].valid) {
            float R[9], p[3], d[3];
            world_from_head(in, &sp[h], R, p);
            for (int q = 0; q < 3; q++) d[q] = p[q] - out[!h].pose.p[q];
            if (norm3(d) < 0.08f) continue;
        }
        jobs[h].following = false;
        accept(t, in, h, &jobs[h], &sp[h], sm[h], sn[h], srms[h], true, &out[h]);
    }

    if (t->ncal >= 15 && t->cal_new >= 8 && in->time_ns - t->last_cal_ns > 2000000000LL) {
        t->last_cal_ns = in->time_ns;
        t->cal_new = 0;
        calibrate(t);
    }
}

void strack_forget(strack *t) {
    for (int h = 0; h < 2; h++) t->hs[h].tracking = false;
}

void strack_shift_yaw(strack *t, int hand, float angle) {
    if (hand == 0 || hand == 1) t->hs[hand].yaw_error -= angle;
}

// ------------------------------------------------------------------------------------ smoothing

void strack_filter_update(strack_filter *f, const float p[3], int64_t t_ns) {
    float dt = (t_ns - f->t_ns) / 1e9f;
    if (!f->have || dt <= 0 || dt > 0.3f || t_ns - f->last_seen_ns > 300000000) {
        memcpy(f->p, p, sizeof f->p);
        memset(f->v, 0, sizeof f->v);
    } else {
        const float alpha = 0.6f, beta = 0.25f;
        for (int k = 0; k < 3; k++) {
            float pred = f->p[k] + f->v[k] * dt, r = p[k] - pred;
            f->p[k] = pred + alpha * r;
            f->v[k] = fmaxf(-6, fminf(6, f->v[k] + beta * r / dt));
        }
    }
    f->have = true;
    f->t_ns = t_ns;
    f->last_seen_ns = t_ns;
}

bool strack_filter_predict(const strack_filter *f, int64_t t_ns, int64_t max_age_ns, float out[3], float vel[3]) {
    if (!f->have || t_ns - f->last_seen_ns > max_age_ns) return false;
    float dt = fmaxf(-0.05f, fminf(0.08f, (t_ns - f->t_ns) / 1e9f));
    // Unseen for a while, it is less likely to keep going.
    float unseen = (t_ns - f->last_seen_ns) / 1e9f, keep = unseen > 0.05f ? expf(-(unseen - 0.05f) / 0.1f) : 1;
    for (int k = 0; k < 3; k++) {
        out[k] = f->p[k] + f->v[k] * dt * keep;
        if (vel) vel[k] = f->v[k] * keep;
    }
    return true;
}
