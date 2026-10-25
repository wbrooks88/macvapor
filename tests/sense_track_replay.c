// Replays a controller capture (psvr2xr-selftest --capture-controllers) through sense_track.c,
// printing each frame's poses in the head and drawing r000.ppm... into out dir: the camera
// pictures with the dots found (yellow) and the fitted LEDs (cyan left, magenta right). BC4
// frames (frame_NNN.bin) show the lower pair side by side; tracking-mode frames (frame_NNN.m4)
// show the upper pair above the lower, at half size.
//
//   cc -O2 -I src tests/sense_track_replay.c src/sense_track.c -lm -o build/sense_track_replay
//   build/sense_track_replay ~/Desktop/"PS VR2 Controller Capture" /tmp/replay [capture]
//
// The cameras start from the built-in ones, or the capture's controller_cameras.json with
// "capture". CAMS="f cx cy yaw pitch roll" x2 overrides the lower pair, UPPER the same for the
// upper pair (and turns it on), UPOS the same for the upper pair, K1 the lenses' k1 and POS="x y z" the lower pair's position (the
// left one's x is -x); THR sets the dot threshold, CAL turns calibration on, BLIND=c drops camera
// c's picture, NOIMG skips the drawing. SPOTS=1 takes the dots from the headset's spot finder
// (ld_NNN.bin beside each frame) instead of the pictures, where the capture has them. DOTS=1 takes
// the dots the tracker found live, from frames.jsonl (a capture without pictures). With the
// capture's imu.jsonl: IMU=1 takes each controller's orientation from its IMU at the exposure
// (PRIOR_LAG ms after it, default 5) instead of the one the tracker was given live; FUSE=1 runs
// strack_fusion alongside and gives the tracker its prediction (pred_p).

#include "sense_track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 1024
#define H 1016
#define PB (W * H / 2)
#define TW 512
#define TH 508
#define TP (TW * TH)

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

// As sense_track.c's camera_rotation.
static void camrot(const strack_camera *c, float R[9]) {
    float Ry[9], Rx[9], Rz[9], C[9] = {1, 0, 0, 0, -1, 0, 0, 0, -1}, t[9], y[3] = {0, 1, 0}, x[3] = {1, 0, 0}, z[3] = {0, 0, 1};
    rotm(y, c->yaw_deg * (float)M_PI / 180, Ry);
    rotm(x, c->pitch_deg * (float)M_PI / 180, Rx);
    rotm(z, c->roll_deg * (float)M_PI / 180, Rz);
    mm(Ry, Rx, t);
    mm(t, Rz, t);
    mm(t, C, R);
}

// The canvas, and where each camera's picture sits on it at what scale (camera pixels to canvas).
static unsigned char img[H][2 * W][3];
static int canvas_w;
static struct { int x, y; float s; } tile[STRACK_CAMERAS];

static void layout(int half) {
    if (!half) {
        canvas_w = 2 * W;
        for (int c = 0; c < STRACK_CAMERAS; c++) tile[c].x = c % 2 * W, tile[c].y = 0, tile[c].s = c < 2 ? 1 : 0;
    } else {
        canvas_w = W;
        for (int c = 0; c < STRACK_CAMERAS; c++) tile[c].x = c % 2 * TW, tile[c].y = c < 2 ? TH : 0, tile[c].s = 0.5f;
    }
}

static void put(int x, int y, unsigned char r, unsigned char g, unsigned char b) {
    if (x < 0 || y < 0 || x >= canvas_w || y >= H) return;
    img[y][x][0] = r; img[y][x][1] = g; img[y][x][2] = b;
}

// A square of half-size r canvas pixels around camera pixel (u, v).
static void mark(int cam, float u, float v, int r, unsigned char cr, unsigned char cg, unsigned char cb) {
    if (!tile[cam].s) return;
    int cu = tile[cam].x + (int)(u * tile[cam].s), cv = tile[cam].y + (int)(v * tile[cam].s);
    int lim_w = tile[cam].s == 1 ? W : TW, lim_h = tile[cam].s == 1 ? H : TH;
    for (int dy = -r; dy <= r; dy++)
        for (int dx = -r; dx <= r; dx++) {
            if (abs(dx) != r && abs(dy) != r) continue;
            int x = cu + dx, y = cv + dy;
            if (x < tile[cam].x || y < tile[cam].y || x >= tile[cam].x + lim_w || y >= tile[cam].y + lim_h) continue;
            put(x, y, cr, cg, cb);
        }
}

static void bc4(const unsigned char *b, unsigned char *px) {
    int a0 = b[0], a1 = b[1], p[8];
    p[0] = a0; p[1] = a1;
    if (a0 > a1) for (int i = 1; i < 7; i++) p[i + 1] = ((7 - i) * a0 + i * a1) / 7;
    else { for (int i = 1; i < 5; i++) p[i + 1] = ((5 - i) * a0 + i * a1) / 5; p[6] = 0; p[7] = 255; }
    unsigned long long bits = 0;
    for (int i = 0; i < 6; i++) bits |= (unsigned long long)b[2 + i] << (8 * i);
    for (int i = 0; i < 16; i++) px[i] = p[(bits >> (3 * i)) & 7];
}

static void qmul(const float a[4], const float b[4], float o[4]) {
    float r[4] = {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
                  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
    memcpy(o, r, sizeof r);
}

// A capture's IMU readings, per hand, in order.
typedef struct { long long t; float q[4], a[3]; } imu_rec;
static imu_rec *g_imu[2];
static int g_nimu[2];

static void load_imu(const char *dir) {
    char path[1024], l[512];
    snprintf(path, sizeof path, "%s/imu.jsonl", dir);
    FILE *f = fopen(path, "r");
    if (!f) { fprintf(stderr, "no %s\n", path); return; }
    int cap[2] = {0, 0};
    while (fgets(l, sizeof l, f)) {
        int h = strstr(l, "\"hand\": 1") ? 1 : 0;
        const char *tp = strstr(l, "\"t\": ");
        if (!tp) continue;
        if (g_nimu[h] == cap[h]) g_imu[h] = realloc(g_imu[h], (cap[h] = cap[h] ? 2 * cap[h] : 4096) * sizeof(imu_rec));
        imu_rec *r = &g_imu[h][g_nimu[h]++];
        r->t = strtoll(tp + 5, NULL, 10);
        float qa[4], aa[3];
        sscanf(strstr(l, "\"q\": [") + 6, "%f, %f, %f, %f", &qa[0], &qa[1], &qa[2], &qa[3]);
        sscanf(strstr(l, "\"a\": [") + 6, "%f, %f, %f", &aa[0], &aa[1], &aa[2]);
        memcpy(r->q, qa, sizeof qa);
        memcpy(r->a, aa, sizeof aa);
    }
    fclose(f);
}

// The IMU's orientation at t, between the readings either side.
static bool imu_q_at(int h, long long t, float q[4]) {
    int lo = 0, hi = g_nimu[h];
    while (lo < hi) { int m = (lo + hi) / 2; if (g_imu[h][m].t < t) lo = m + 1; else hi = m; }
    if (lo == 0 || lo >= g_nimu[h]) return false;
    const imu_rec *a = &g_imu[h][lo - 1], *b = &g_imu[h][lo];
    if (b->t - a->t > 60000000) return false;
    float w = (float)(t - a->t) / (float)(b->t - a->t), d = 0, n = 0;
    for (int k = 0; k < 4; k++) d += a->q[k] * b->q[k];
    for (int k = 0; k < 4; k++) { q[k] = a->q[k] + w * ((d < 0 ? -b->q[k] : b->q[k]) - a->q[k]); n += q[k] * q[k]; }
    n = sqrtf(n);
    for (int k = 0; k < 4; k++) q[k] /= n;
    return true;
}

// The dots of one camera from a frames.jsonl line: "dots_<name>": [[x, y, area, brightness], ...].
static int parse_dots(const char *line, const char *key, strack_blob *out, int max) {
    const char *p = strstr(line, key);
    if (!p || !(p = strchr(p, '['))) return 0;
    p++;
    int n = 0;
    while (n < max) {
        while (*p == ' ' || *p == ',') p++;
        if (*p != '[') break;
        float x, y, b;
        int a;
        if (sscanf(p, "[%f, %f, %d, %f]", &x, &y, &a, &b) != 4) break;
        out[n++] = (strack_blob){.x = x, .y = y, .area = a, .brightness = b};
        p = strchr(p, ']');
        if (!p) break;
        p++;
    }
    return n;
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

static void set_pair(strack_camera *cams, const char *spec) {
    float v[12];
    if (sscanf(spec, "%f %f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7],
               &v[8], &v[9], &v[10], &v[11]) != 12) return;
    for (int c = 0; c < 2; c++) {
        cams[c].f = v[c * 6]; cams[c].cx = v[c * 6 + 1]; cams[c].cy = v[c * 6 + 2];
        cams[c].yaw_deg = v[c * 6 + 3]; cams[c].pitch_deg = v[c * 6 + 4]; cams[c].roll_deg = v[c * 6 + 5];
    }
}

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: %s <capture dir> <out dir> [capture]\n", argv[0]); return 1; }
    setvbuf(stdout, NULL, _IONBF, 0);
    char path[1024];
    static char line[1 << 17];
    strack_camera cams[STRACK_CAMERAS];
    strack_default_cameras(cams);
    if (argc > 3 && !strcmp(argv[3], "capture")) {
        snprintf(path, sizeof path, "%s/controller_cameras.json", argv[1]);
        FILE *f = fopen(path, "r");
        static char js[16384];
        size_t n = f ? fread(js, 1, sizeof js - 1, f) : 0;
        js[n] = 0;
        if (f) fclose(f);
        char *o = js;
        for (int c = 0; c < STRACK_CAMERAS && o && *o; c++) {
            char *end = strstr(o + 1, "},");
            if (end) *end = 0;
            if (strstr(o, "\"f\"")) {
                cams[c].f = jnum(o, "\"f\""); cams[c].cx = jnum(o, "\"cx\""); cams[c].cy = jnum(o, "\"cy\"");
                cams[c].yaw_deg = jnum(o, "yaw_deg"); cams[c].pitch_deg = jnum(o, "pitch_deg");
                cams[c].roll_deg = jnum(o, "roll_deg");
                jarr(o, "\"k\"", cams[c].k, 4);
                jarr(o, "position", cams[c].position, 3);
            }
            o = end ? end + 2 : NULL;
        }
    }
    if (getenv("CAMS")) set_pair(cams, getenv("CAMS"));
    if (getenv("UPPER")) { set_pair(cams + 2, getenv("UPPER")); cams[2].off = cams[3].off = false; }
    if (getenv("UPOS")) {
        float x, y, z;
        if (sscanf(getenv("UPOS"), "%f %f %f", &x, &y, &z) == 3)
            for (int c = 2; c < 4; c++) { cams[c].position[0] = c == 3 ? x : -x; cams[c].position[1] = y; cams[c].position[2] = z; }
    }
    if (getenv("K1")) for (int c = 0; c < STRACK_CAMERAS; c++) cams[c].k[0] = atof(getenv("K1"));
    if (getenv("POS")) {
        float x, y, z;
        if (sscanf(getenv("POS"), "%f %f %f", &x, &y, &z) == 3)
            for (int c = 0; c < 2; c++) { cams[c].position[0] = c ? x : -x; cams[c].position[1] = y; cams[c].position[2] = z; }
    }
    for (int c = 0; c < STRACK_CAMERAS; c++)
        printf("cam %d: f %.1f c %.1f %.1f yaw %.1f pitch %.1f roll %.1f%s\n", c, cams[c].f, cams[c].cx, cams[c].cy,
               cams[c].yaw_deg, cams[c].pitch_deg, cams[c].roll_deg, cams[c].off ? " (off)" : "");
    strack_params prm;
    strack_default_params(&prm);
    if (getenv("THR")) prm.threshold = atof(getenv("THR"));
    if (getenv("PRED_SIGMA")) prm.pred_max_sigma = atof(getenv("PRED_SIGMA"));
    if (getenv("PRED_JUMP")) prm.pred_jump_m = atof(getenv("PRED_JUMP"));
    if (getenv("PRED_VETO")) prm.pred_veto_m = atof(getenv("PRED_VETO"));
    prm.calibrate = getenv("CAL") != NULL;   // off, as in the headset
    strack *t = strack_create(cams, &prm);
    snprintf(path, sizeof path, "%s/frames.jsonl", argv[1]);
    FILE *fl = fopen(path, "r");
    if (!fl) { fprintf(stderr, "no %s\n", path); return 1; }
    unsigned char *buf = malloc(2 * (256 + 2 * TP) + 256 + 2 * PB);
    const char *bl = getenv("BLIND");
    int blind = bl ? atoi(bl) : -1, frame = 0;
    int use_spots = getenv("SPOTS") && atoi(getenv("SPOTS"));
    int use_dots = getenv("DOTS") && atoi(getenv("DOTS")), use_imu = getenv("IMU") && atoi(getenv("IMU"));
    int use_fuse = getenv("FUSE") && atoi(getenv("FUSE"));
    float prior_lag_ms = getenv("PRIOR_LAG") ? atof(getenv("PRIOR_LAG")) : 5;
    if (use_imu || use_fuse) load_imu(argv[1]);
    static strack_fusion fz[2];
    int fed[2] = {0, 0};
    for (int h = 0; h < 2; h++) strack_fusion_init(&fz[h]);
    static unsigned char ld[36944];
    while (fgets(line, sizeof line, fl)) {
        char name[64];
        const char *fp = strstr(line, "\"file\": \"");
        if (!fp || sscanf(fp + 9, "%63[^\"]", name) != 1) continue;
        int half = strstr(name, ".m4") != NULL;
        snprintf(path, sizeof path, "%s/%s", argv[1], name);
        FILE *fb = fopen(path, "rb");
        size_t want = half ? 2 * (256 + 2 * TP) : 256 + 2 * PB;
        size_t got = fb ? fread(buf, 1, want, fb) : 0;
        if (fb) fclose(fb);
        if (got != want) {
            if (!use_spots && !use_dots) continue;
            memset(buf, 0, want);   // a capture without the pictures
        }
        strack_input in = {0};
        in.time_ns = strtoll(strstr(line, "\"time_ns\": ") + 11, NULL, 10);
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
            float align[4] = {0, 0, 0, 1}, q[4];
            // Live, each turn of the IMU's yaw (g_ctl_yaw, align_q here) was told to the tracker
            // (strack_shift_yaw); the same turns, from one frame's align_q to the next.
            static float last_yaw[2];
            static bool have_yaw[2];
            if (jarr(tmp, "align_q", align, 4)) {
                float yaw = 2 * atan2f(align[1], align[3]);
                if (have_yaw[h]) strack_shift_yaw(t, h, remainderf(yaw - last_yaw[h], 2 * (float)M_PI));
                last_yaw[h] = yaw;
                have_yaw[h] = true;
            }
            // The IMU's orientation at the exposure, turned to the LED model's as backend_psvr2.m does.
            if (in.prior_valid[h] && use_imu && imu_q_at(h, in.time_ns + (long long)(prior_lag_ms * 1e6f), q)) {
                static const float to_grip[4] = {0.42492f, 0, 0, 0.90523f};
                float g[4];
                qmul(align, q, g);
                qmul(g, to_grip, in.prior_q[h]);
            }
            if (use_fuse) {
                // The IMU's readings come in until the tracker gets this frame (about 25 ms after
                // the exposure), less Bluetooth's couple of milliseconds.
                while (fed[h] < g_nimu[h] && g_imu[h][fed[h]].t <= in.time_ns + 23000000) {
                    const imu_rec *r = &g_imu[h][fed[h]++];
                    float w[4], a[3];
                    qmul(align, r->q, w);
                    qrot(w, r->a, a);
                    a[1] -= 9.80665f;
                    strack_fusion_imu(&fz[h], r->t, a);
                }
                in.pred_valid[h] = strack_fusion_estimate(&fz[h], in.time_ns, 250000000, in.pred_p[h], &in.pred_sigma[h]);
            }
        }
        bool spots = false;
        if (use_spots) {
            char ldname[1024];
            snprintf(ldname, sizeof ldname, "%s/ld_%.*s.bin", argv[1], (int)strcspn(name + 6, "."), name + 6);
            FILE *fs = fopen(ldname, "rb");
            spots = fs && fread(ld, 1, sizeof ld, fs) == sizeof ld && ld[0] == 'L' && ld[1] == 'D';
            if (fs) fclose(fs);
        }
        // The pictures: BC4 planes, or each pair's header and two half-size pictures.
        const unsigned char *pic[STRACK_CAMERAS] = {0};
        bool have[STRACK_CAMERAS] = {false};
        for (int c = 0; c < STRACK_CAMERAS; c++) {
            if (half) {
                const unsigned char *pair = buf + (c / 2) * (256 + 2 * TP);
                have[c] = pair[0] == 'V' && pair[1] == 'I';
                pic[c] = pair + 256 + (c % 2) * TP;
            } else if (c < 2) {
                have[c] = true;
                pic[c] = buf + 256 + c * PB;
            }
            static const char *dkey[] = {"\"dots_left\"", "\"dots_right\"", "\"dots_upper_left\"", "\"dots_upper_right\""};
            strack_blob dots[STRACK_MAX_BLOBS];
            if (c == blind) strack_no_picture(t, c);
            else if (use_dots) strack_set_dots(t, c, dots, parse_dots(line, dkey[c], dots, STRACK_MAX_BLOBS));
            else if (spots) strack_detect_spots(t, c, ld + 0x40 + c * 9220);
            else if (!have[c]) strack_no_picture(t, c);
            else if (half) strack_detect_half(t, c, pic[c], TW - 4, TH, TW);
            else strack_detect_bc4(t, c, pic[c], 256);
        }
        strack_result r[2];
        strack_solve(t, &in, r);
        for (int h = 0; h < 2 && use_fuse; h++)
            if (r[h].valid) strack_fusion_optical(&fz[h], in.time_ns, r[h].pose.p, r[h].leds, r[h].rms_px);
        strack_camera cc[STRACK_CAMERAS];
        strack_get_cameras(t, cc);
        printf("%s", name);
        int draw = !getenv("NOIMG");
        if (draw) {
            layout(half);
            memset(img, 0, sizeof img);
            for (int c = 0; c < STRACK_CAMERAS; c++) {
                if (!have[c] || !tile[c].s) continue;
                if (half) {
                    for (int y = 0; y < TH; y++)
                        for (int x = 0; x < TW; x++) {
                            unsigned char g = pic[c][y * TW + x];
                            put(tile[c].x + x, tile[c].y + y, g, g, g);
                        }
                } else {
                    for (int by = 0; by < H / 4; by++)
                        for (int bx = 0; bx < W / 4; bx++) {
                            unsigned char px[16];
                            bc4(pic[c] + (by * 256 + bx) * 8, px);
                            for (int i = 0; i < 16; i++) put(tile[c].x + bx * 4 + i % 4, by * 4 + i / 4, px[i], px[i], px[i]);
                        }
                }
                int n;
                const strack_blob *b = strack_blobs(t, c, &n);
                for (int i = 0; i < n; i++) mark(c, b[i].x, b[i].y, half ? 4 : 6, 255, 255, 0);
            }
        }
        float hqc[4] = {-in.head.q[0], -in.head.q[1], -in.head.q[2], in.head.q[3]};
        for (int h = 0; h < 2; h++) {
            if (!r[h].valid) { printf(" | %s -", h ? "R" : "L"); continue; }
            float ph[3], d[3] = {r[h].pose.p[0] - in.head.p[0], r[h].pose.p[1] - in.head.p[1], r[h].pose.p[2] - in.head.p[2]};
            qrot(hqc, d, ph);
            printf(" | %s head(%.4f %.4f %.4f) leds %d rms %.2f yaw %.0f%s world(%.4f %.4f %.4f) t %lld", h ? "R" : "L", ph[0],
                   ph[1], ph[2], r[h].leds, r[h].rms_px, r[h].yaw_error * 57.3f, r[h].found ? " found" : "", r[h].pose.p[0],
                   r[h].pose.p[1], r[h].pose.p[2], (long long)in.time_ns);
            if (!draw) continue;
            for (int c = 0; c < STRACK_CAMERAS; c++) {
                if (cc[c].off) continue;
                float Rc[9];
                camrot(&cc[c], Rc);
                for (int i = 0; i < STRACK_LEDS; i++) {
                    float lp[3], nrm[3], w[3], hh[3], pc[3], nw[3], nh[3], nc[3], u, v;
                    strack_led(h, i, lp, nrm);
                    qrot(r[h].pose.q, lp, w);
                    for (int k = 0; k < 3; k++) w[k] += r[h].pose.p[k] - in.head.p[k];
                    qrot(hqc, w, hh);
                    for (int k = 0; k < 3; k++) hh[k] -= cc[c].position[k];
                    for (int k = 0; k < 3; k++) pc[k] = Rc[k] * hh[0] + Rc[3 + k] * hh[1] + Rc[6 + k] * hh[2];
                    qrot(r[h].pose.q, nrm, nw);
                    qrot(hqc, nw, nh);
                    for (int k = 0; k < 3; k++) nc[k] = Rc[k] * nh[0] + Rc[3 + k] * nh[1] + Rc[6 + k] * nh[2];
                    float dd = sqrtf(pc[0] * pc[0] + pc[1] * pc[1] + pc[2] * pc[2]);
                    float facing = -(nc[0] * pc[0] + nc[1] * pc[1] + nc[2] * pc[2]) / dd;
                    if (strack_project(&cc[c], pc, &u, &v) && facing > 0.2f)
                        mark(c, u, v, half ? 2 : 3, h ? 255 : 0, h ? 0 : 255, 255);
                }
            }
        }
        float rms;
        int cal = strack_calibration(t, &rms);
        printf(" | cal %d %.2f\n", cal, rms);
        if (!draw) { frame++; continue; }
        snprintf(path, sizeof path, "%s/r%03d.ppm", argv[2], frame++);
        FILE *o = fopen(path, "wb");
        if (!o) continue;
        fprintf(o, "P6 %d %d 255\n", canvas_w, H);
        for (int y = 0; y < H; y++) fwrite(img[y], 1, (size_t)canvas_w * 3, o);
        fclose(o);
    }
    strack_camera cc[STRACK_CAMERAS];
    strack_get_cameras(t, cc);
    for (int c = 0; c < STRACK_CAMERAS; c++)
        printf("final cam %d: f %.1f c %.1f %.1f yaw %.1f pitch %.1f roll %.1f\n", c, cc[c].f, cc[c].cx, cc[c].cy,
               cc[c].yaw_deg, cc[c].pitch_deg, cc[c].roll_deg);
    strack_destroy(t);
    return 0;
}
