// The PS VR2 backend: alvrxr's runtime (runtime.m) shown on a PlayStation VR2 connected through
// Sony's PC adapter, in place of backend.m's stream to ALVR. Built by build-psvr2.sh into
// libpsvr2xr.dylib; see PSVR2.md.
//
// macOS sees the headset as an ordinary 4000x2040 display. A borderless window covers it, and a
// display link on that display composes the newest game frame into it at the panel's rate: lens
// and colour correction from the headset's own calibration, and the image turned by however far
// the head has rotated since the game rendered it, so head turns stay smooth when the game runs
// slower than the panel. Head rotation comes over USB (src/psvr2/, from psvr2-player-mac and
// Monado's driver). Position is a neck model unless config.json asks for the experimental SLAM
// position. The Sense controllers come over Bluetooth (sense.c): their buttons, sticks, triggers
// and orientation. Their position comes from the headset's lower cameras seeing their LED rings
// (sense_track.c), and from an arm model while the cameras can't see them. A double press of the
// headset's button shows the room through its front cameras in place of the game, and another
// goes back.
//
// The headset's eye tracker comes over the same USB (cpsvr2.c): games that ask for eye gaze get it
// (XR_EXT_eye_gaze_interaction), games that render quad views get a sharp inset that follows the
// eyes and a cheap low-resolution rest (foveated rendering), and holding Fn for two seconds runs
// the eye tracker's own calibration, drawn by the compositor over whatever is showing.

#import <AppKit/AppKit.h>
#import <CoreVideo/CoreVideo.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>
#include <mach/mach_time.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <time.h>

#include "alvrxr.h"
#include "psvr2/cpsvr2.h"
#include "sense.h"
#include "sense_track.h"

unsigned axr_stat_wait, axr_stat_end, axr_stat_projection;   // runtime.m's frame and input counts
unsigned axr_stat_syncs, axr_stat_unfocused_syncs, axr_stat_pressed;

// ------------------------------------------------------------------------------------- logging

static FILE *g_logfile;

void axr_log(const char *fmt, ...) {
    static dispatch_once_t once;
    dispatch_once(&once, ^{
        NSString *dir = [NSHomeDirectory() stringByAppendingPathComponent:@"Library/Logs"];
        g_logfile = fopen([[dir stringByAppendingPathComponent:@"psvr2xr.log"] fileSystemRepresentation], "a");
    });
    char msg[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    char line[1100];
    snprintf(line, sizeof line, "%02d:%02d:%02d.%03ld [psvr2xr %d] %s\n", tm.tm_hour, tm.tm_min, tm.tm_sec,
             ts.tv_nsec / 1000000, getpid(), msg);
    fputs(line, stderr);
    if (g_logfile) { fputs(line, g_logfile); fflush(g_logfile); }
}

int64_t axr_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

const char *axr_system_name(void) { return "PlayStation VR2"; }

// The display link stamps vsyncs in host time (mach_absolute_time); XrTime is CLOCK_MONOTONIC.
static mach_timebase_info_data_t g_timebase;

static int64_t host_to_xr(uint64_t host) {
    int64_t host_ns = (int64_t)(host * g_timebase.numer / g_timebase.denom);
    int64_t now_host = (int64_t)(mach_absolute_time() * g_timebase.numer / g_timebase.denom);
    return host_ns + (axr_now_ns() - now_host);
}

// -------------------------------------------------------------------------------------- config

// <config>/config.json, read at start; a default one is written on first run.
typedef struct {
    float render_scale;   // of a 2000x1906 eye image, which matches the panel at the centre
    float eye_height;     // metres above the floor, for the neck model
    float lookahead_ms;   // added to the pose prediction, to tune for the felt latency
    bool chromatic;       // per-colour lens correction
    bool reprojection;    // turn each frame by the head's rotation since it was rendered
    bool positional;      // experimental: the headset's SLAM position instead of the neck model
    bool controllers;     // the Sense controllers over Bluetooth
    bool controller_tracking;      // their position from the headset's cameras
    float controller_latency_ms;   // from a camera frame's exposure to its arrival over USB
    float controller_threshold;    // 0..255: how bright an LED's dot is in the camera picture
    float controller_smoothing;    // 0 off, 1 as tuned, more to smooth harder (its jitter vs its lag)
    bool eye_tracking;    // the headset's eye tracker
    bool show_gaze;       // a ring where the eyes look, to check the eye tracker
    bool foveated;        // offer quad views (a gaze-following sharp inset) to games that use them
    float foveated_area_deg;     // width and height of the inset
    float foveated_outer_scale;  // the rest of the view's resolution, as a share of render_scale's
    float foveated_inset_scale;  // the inset's resolution, as a share of the panel's at its centre
} config;

static NSString *g_config_dir;
static config g_cfg = {.render_scale = 0.75f, .eye_height = 1.6f, .chromatic = true, .reprojection = true,
                       .controllers = true, .controller_tracking = true, .controller_latency_ms = 25,
                       .controller_threshold = 170, .controller_smoothing = 1, .eye_tracking = true, .foveated = true, .foveated_area_deg = 40,
                       .foveated_outer_scale = 0.5f, .foveated_inset_scale = 1.0f};

static void read_config(void) {
    NSString *path = [g_config_dir stringByAppendingPathComponent:@"config.json"];
    NSFileManager *fm = NSFileManager.defaultManager;
    [fm createDirectoryAtPath:g_config_dir withIntermediateDirectories:YES attributes:nil error:nil];
    if (![fm fileExistsAtPath:path]) {
        NSDictionary *d = @{
            @"render_scale" : @(g_cfg.render_scale), @"eye_height" : @(g_cfg.eye_height),
            @"lookahead_ms" : @(g_cfg.lookahead_ms), @"chromatic" : @(g_cfg.chromatic),
            @"reprojection" : @(g_cfg.reprojection), @"positional" : @(g_cfg.positional),
            @"controllers" : @(g_cfg.controllers), @"eye_tracking" : @(g_cfg.eye_tracking),
            @"controller_tracking" : @(g_cfg.controller_tracking),
            @"controller_latency_ms" : @(g_cfg.controller_latency_ms),
            @"controller_threshold" : @(g_cfg.controller_threshold),
            @"controller_smoothing" : @(g_cfg.controller_smoothing),
            @"show_gaze" : @(g_cfg.show_gaze), @"foveated" : @(g_cfg.foveated),
            @"foveated_area_deg" : @(g_cfg.foveated_area_deg), @"foveated_outer_scale" : @(g_cfg.foveated_outer_scale),
            @"foveated_inset_scale" : @(g_cfg.foveated_inset_scale),
        };
        [[NSJSONSerialization dataWithJSONObject:d options:NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys error:nil]
            writeToFile:path atomically:YES];
    }
    NSData *data = [NSData dataWithContentsOfFile:path];
    NSDictionary *c = data ? [NSJSONSerialization JSONObjectWithData:data options:0 error:nil] : nil;
    if (![c isKindOfClass:NSDictionary.class]) { axr_log("config.json unreadable, using defaults"); return; }
    if (c[@"render_scale"]) g_cfg.render_scale = fmaxf(0.3f, fminf(1.5f, [c[@"render_scale"] floatValue]));
    if (c[@"eye_height"]) g_cfg.eye_height = [c[@"eye_height"] floatValue];
    if (c[@"lookahead_ms"]) g_cfg.lookahead_ms = [c[@"lookahead_ms"] floatValue];
    if (c[@"chromatic"]) g_cfg.chromatic = [c[@"chromatic"] boolValue];
    if (c[@"reprojection"]) g_cfg.reprojection = [c[@"reprojection"] boolValue];
    if (c[@"positional"]) g_cfg.positional = [c[@"positional"] boolValue];
    if (c[@"controllers"]) g_cfg.controllers = [c[@"controllers"] boolValue];
    if (c[@"controller_tracking"]) g_cfg.controller_tracking = [c[@"controller_tracking"] boolValue];
    if (c[@"controller_latency_ms"])
        g_cfg.controller_latency_ms = fmaxf(0, fminf(100, [c[@"controller_latency_ms"] floatValue]));
    if (c[@"controller_threshold"])
        g_cfg.controller_threshold = fmaxf(40, fminf(254, [c[@"controller_threshold"] floatValue]));
    if (c[@"controller_smoothing"])
        g_cfg.controller_smoothing = fmaxf(0, fminf(4, [c[@"controller_smoothing"] floatValue]));
    if (c[@"eye_tracking"]) g_cfg.eye_tracking = [c[@"eye_tracking"] boolValue];
    if (c[@"show_gaze"]) g_cfg.show_gaze = [c[@"show_gaze"] boolValue];
    if (c[@"foveated"]) g_cfg.foveated = [c[@"foveated"] boolValue];
    if (c[@"foveated_area_deg"]) g_cfg.foveated_area_deg = fmaxf(15, fminf(80, [c[@"foveated_area_deg"] floatValue]));
    if (c[@"foveated_outer_scale"])
        g_cfg.foveated_outer_scale = fmaxf(0.2f, fminf(1, [c[@"foveated_outer_scale"] floatValue]));
    if (c[@"foveated_inset_scale"])
        g_cfg.foveated_inset_scale = fmaxf(0.3f, fminf(1.5f, [c[@"foveated_inset_scale"] floatValue]));
    axr_log("config: render scale %.2f, eye height %.2f m, lookahead %+.1f ms, chromatic %s, reprojection %s, %s,"
            " controllers %s",
            g_cfg.render_scale, g_cfg.eye_height, g_cfg.lookahead_ms, g_cfg.chromatic ? "on" : "off",
            g_cfg.reprojection ? "on" : "off", g_cfg.positional ? "SLAM position (experimental)" : "neck model",
            g_cfg.controllers ? (g_cfg.controller_tracking ? "on, tracked by the headset's cameras" : "on, arm model only") : "off");
    axr_log("config: eye tracking %s, gaze ring %s, foveated rendering %s (inset %.0f deg at %.2f, rest at %.2f)",
            g_cfg.eye_tracking ? "on" : "off", g_cfg.show_gaze ? "on" : "off", g_cfg.foveated ? "offered" : "off",
            g_cfg.foveated_area_deg, g_cfg.foveated_inset_scale, g_cfg.foveated_outer_scale);
}

// --------------------------------------------------------------------------------- quaternions

typedef struct { float x, y, z, w; } quat;

static const quat QID = {0, 0, 0, 1};

static quat qmul(quat a, quat b) {
    return (quat){a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                  a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

static quat qconj(quat q) { return (quat){-q.x, -q.y, -q.z, q.w}; }

static quat qnorm(quat q) {
    float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return n > 0 ? (quat){q.x / n, q.y / n, q.z / n, q.w / n} : QID;
}

static void qrot(quat q, const float v[3], float out[3]) {
    float tx = 2 * (q.y * v[2] - q.z * v[1]), ty = 2 * (q.z * v[0] - q.x * v[2]), tz = 2 * (q.x * v[1] - q.y * v[0]);
    out[0] = v[0] + q.w * tx + (q.y * tz - q.z * ty);
    out[1] = v[1] + q.w * ty + (q.z * tx - q.x * tz);
    out[2] = v[2] + q.w * tz + (q.x * ty - q.y * tx);
}

static quat qyaw(float a) { return (quat){0, sinf(a / 2), 0, cosf(a / 2)}; }

// Column-major, as Metal's float4x4.
static void qmatrix(quat q, float m[16]) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z, xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    float r[16] = {1 - 2 * (yy + zz), 2 * (xy + wz),     2 * (xz - wy),     0,
                   2 * (xy - wz),     1 - 2 * (xx + zz), 2 * (yz + wx),     0,
                   2 * (xz + wy),     2 * (yz - wx),     1 - 2 * (xx + yy), 0,
                   0,                 0,                 0,                 1};
    memcpy(m, r, sizeof r);
}

// ------------------------------------------------------------------------------------- state

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_started, g_usb;
static float g_calib[8];

// Head orientation: the fused, predicted SLAM/gyro quaternion in Monado's axes, turned upright as
// psvr2-player-mac does, then g_recenter's yaw so that forward at the last recenter is -Z.
static const quat k_upright = {0, 0, (float)M_SQRT1_2, (float)M_SQRT1_2};
static quat g_recenter = {0, 0, 0, 1};
static float g_origin[3];          // SLAM position at the last recenter (experimental positional)
static bool g_tracking_ready;      // recentered once, so LOCAL is taken from a settled pose

// Panel timing.
static double g_refresh = 90;
static int64_t g_period_ns = 11111111;
static int64_t g_vsync_ns;         // a recent vsync of the headset display, XrTime
static int64_t g_last_wait, g_last_display;

// The head orientation each frame was predicted with, by display time, so xrLocateViews and the
// compositor agree on what the game rendered for.
enum { POSE_RING = 32 };
static struct { int64_t time; quat q; } g_poses[POSE_RING];
static unsigned g_pose_next;

// Copies of the game's eye images. The game's swapchain images go back to it after xrEndFrame,
// so axr_submit copies each frame here on the game's queue; the compositor reads the newest.
enum { SLOTS = 3 };
static struct {
    id<MTLTexture> eye[4];         // left, right, and with quad views the left and right insets
    bool quad;
    float inset_fov[2][4];         // the insets' field of view (left, right, up, down)
    quat q;                        // the head orientation the frame was rendered with
    int readers;                   // compositor passes in flight
    bool pending;                  // copy in flight
    uint64_t seq;
} g_slots[SLOTS];
static int g_latest = -1;
static uint64_t g_seq;

// Display.
static CGDirectDisplayID g_display;
static NSWindow *g_window;
void (*axr_on_button_hold)(void);
static CAMetalLayer *g_layer;
static CVDisplayLinkRef g_link;
static id<MTLDevice> g_device;
static id<MTLCommandQueue> g_queue;
static id<MTLRenderPipelineState> g_pipeline;
static id<MTLBuffer> g_lut;
static id<MTLTexture> g_blank;     // bound before the first game frame, which the shader then skips

// Passthrough: the headset's cameras in place of the game (double press of the headset button).
// The status thread powers them; the compositor uploads their frames as two BC4 textures.
static volatile bool g_passthrough;
static id<MTLTexture> g_cam[2];
static unsigned char *g_cam_buf[2];

static unsigned g_stat_submits, g_stat_dropped, g_stat_composed, g_stat_repeated, g_stat_no_drawable;
static uint64_t g_last_composed_seq;

// compose.metal's PS VR2 counterpart, embedded by build-psvr2.sh as a C string.
#include "psvr2_compose_metal.h"

typedef struct {
    float calib_l[4], calib_r[4];
    float delta[16];
    float p0[4];          // a frame, chromatic, quad insets, gaze ring
    float p1[4];          // passthrough
    float cal[4];         // calibration target in the head (m), mode: 0 off, 1 dark, 2 light
    float cal2[4];        // half the IPD (m), seconds since the target appeared
    float gaze[4];        // gaze direction in the head, valid
    float inset_l[4];     // tangents of the insets' field of view: left, right, up, down
    float inset_r[4];
} compose_uniform;
_Static_assert(sizeof(compose_uniform) == 208, "Uni layout");

id<MTLDevice> axr_metal_device(void) {
    static dispatch_once_t once;
    dispatch_once(&once, ^{ g_device = MTLCreateSystemDefaultDevice(); });
    return g_device;
}

// -------------------------------------------------------------------------------------- poses

// The head orientation ahead_s seconds from now, with the recenter yaw. False without tracking.
static bool head_orientation(float ahead_s, quat *out) {
    float q[4];
    if (!g_usb || psvr2_get_predicted_quat(ahead_s, q) != 1) return false;
    quat o = qnorm(qmul(k_upright, (quat){q[1], q[2], q[3], q[0]}));
    pthread_mutex_lock(&g_lock);
    quat r = g_recenter;
    pthread_mutex_unlock(&g_lock);
    *out = qnorm(qmul(r, o));
    return true;
}

// Experimental: the SLAM position, in the same axes as the orientation. The orientation's mapping
// is psvr2-player-mac's; that the position takes the same one is a guess to check against the log.
static bool slam_position(float out[3]) {
    float q[4], p[3];
    if (!g_usb || psvr2_get_pose(q, p) != 1) return false;
    float mapped[3] = {-p[1], -p[0], p[2]}, up[3];
    qrot(k_upright, mapped, up);
    pthread_mutex_lock(&g_lock);
    quat r = g_recenter;
    pthread_mutex_unlock(&g_lock);
    qrot(r, up, out);
    return true;
}

// ------------------------------------------------------------------------ controller tracking
//
// The headset's lower cameras see the controllers' LEDs, which sense.c keeps lit while this runs.
// The camera thread hands each frame to tracking_thread, which finds the controllers in it
// (sense_track.c) and feeds each fit into g_fusion, which carries the position on between fits
// with the controller's own accelerometer and leaves out fits that jump. hand_motion uses that
// while it is fresh, and the arm model otherwise, easing from one to the other. The tracker also
// tells how far each controller's IMU yaw and tilt have drifted, and g_ctl_align takes that in bit
// by bit.

static strack *g_tracker;                   // the tracking thread's
static strack_fusion g_fusion[2];          // under g_lock
static int64_t g_fusion_imu_after[2];       // the last IMU reading fed to it
static int64_t g_optical_seen[2];           // exposure time of the last frame each was seen in
static volatile bool g_tracker_reset;       // the world turned (recentering): start following afresh
static struct {
    bool optical;            // what the position was based on last time
    float from[3];           // the offset being eased out after a switch
    int64_t since;
    float last[3];
    bool have_last;
} g_blend[2];                // under g_lock

static quat g_ctl_align[2];
// The controller's new IMU readings into its fusion, as accelerations in the room: turned by its
// orientation then and by g_ctl_align, gravity taken out. Under g_lock.
static void feed_imu(int hand) {
    sense_sample ss[64];
    int n = sense_samples(hand, g_fusion_imu_after[hand], ss, 64);
    for (int i = 0; i < n; i++) {
        quat q = qmul(g_ctl_align[hand], (quat){ss[i].q[0], ss[i].q[1], ss[i].q[2], ss[i].q[3]});
        float a[3];
        qrot(q, ss[i].accel, a);
        a[1] -= 9.80665f;
        strack_fusion_imu(&g_fusion[hand], ss[i].t_ns, a);
    }
    if (n) g_fusion_imu_after[hand] = ss[n - 1].t_ns;
}

static bool optically_tracked(int hand, int64_t within_ns) {
    pthread_mutex_lock(&g_lock);
    bool seen = g_optical_seen[hand] && axr_now_ns() - g_optical_seen[hand] < within_ns;
    pthread_mutex_unlock(&g_lock);
    return seen;
}

// ------------------------------------------------------------------------------- controllers
//
// A controller's orientation comes from its own IMU, level with gravity but with a yaw that is
// wherever it started and drifts slowly; g_ctl_align turns it to match the room. It is set when a
// controller connects, when the headset recenters, and when the controller's PS button is pressed,
// each time taking the controller to be pointing the way the head faces. The cameras' fits then
// correct its yaw and its tilt (the IMU's sense of level is often 5 to 15 degrees off) bit by bit.

static quat g_ctl_align[2] = {{0, 0, 0, 1}, {0, 0, 0, 1}};
static bool g_ctl_aligned[2];
static float g_torso_yaw;          // the arm model's shoulders, which follow the head's yaw loosely

static float yaw_of(quat q) {
    float f[3] = {0, 0, -1}, fwd[3];
    qrot(q, f, fwd);
    return atan2f(-fwd[0], -fwd[2]);
}

// The grip's orientation in the room, before g_ctl_align.
static quat raw_grip(const sense_state *s) {
    quat imu = {s->orientation[0], s->orientation[1], s->orientation[2], s->orientation[3]};
    quat to_grip = {sense_imu_to_grip[0], sense_imu_to_grip[1], sense_imu_to_grip[2], sense_imu_to_grip[3]};
    return qnorm(qmul(imu, to_grip));
}

// Turns a controller to point where the head faces, head_yaw radians from -Z.
static void align_controller(int hand, float head_yaw) {
    sense_state s;
    if (!sense_get(hand, &s)) return;
    float yaw = yaw_of(raw_grip(&s));
    pthread_mutex_lock(&g_lock);
    g_ctl_align[hand] = qyaw(head_yaw - yaw);
    g_ctl_aligned[hand] = true;
    pthread_mutex_unlock(&g_lock);
}

static void recenter(void) {
    quat o;
    float q[4];
    if (!g_usb || psvr2_get_predicted_quat(0, q) != 1) return;
    o = qnorm(qmul(k_upright, (quat){q[1], q[2], q[3], q[0]}));
    float fwd[3], f[3] = {0, 0, -1};
    qrot(o, f, fwd);
    float yaw = atan2f(-fwd[0], -fwd[2]);
    pthread_mutex_lock(&g_lock);
    quat turn = qnorm(qmul(qyaw(-yaw), qconj(g_recenter)));   // how the room turns
    g_recenter = qyaw(-yaw);
    pthread_mutex_unlock(&g_lock);
    float pos[3];
    if (slam_position(pos)) {
        pthread_mutex_lock(&g_lock);
        memcpy(g_origin, pos, sizeof pos);
        pthread_mutex_unlock(&g_lock);
    }
    axr_log("recentered (turned %.0f deg)", yaw * 180 / M_PI);
    // A controller the cameras follow keeps its place in the room, which turns with the head's
    // forward; one they don't points where the head now faces.
    for (int h = 0; h < 2; h++) {
        if (optically_tracked(h, 2000000000LL)) {
            pthread_mutex_lock(&g_lock);
            g_ctl_align[h] = qnorm(qmul(turn, g_ctl_align[h]));
            strack_fusion_reset(&g_fusion[h]);
            pthread_mutex_unlock(&g_lock);
        } else {
            align_controller(h, 0);
        }
    }
    g_tracker_reset = true;
}

// The orientation a frame for display time t is rendered with: the one predicted for it at
// xrWaitFrame if there is one, else a fresh prediction.
static quat pose_for(int64_t t) {
    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < POSE_RING; i++)
        if (g_poses[i].time && llabs(g_poses[i].time - t) < 500000) {
            quat q = g_poses[i].q;
            pthread_mutex_unlock(&g_lock);
            return q;
        }
    pthread_mutex_unlock(&g_lock);
    quat q = QID;
    head_orientation(((float)(t - axr_now_ns()) + g_cfg.lookahead_ms * 1e6f) / 1e9f, &q);
    return q;
}

static axr_pose head_pose(quat q) {
    axr_pose p = {{q.x, q.y, q.z, q.w}, {0, g_cfg.eye_height, 0}};
    float slam[3];
    if (g_cfg.positional && slam_position(slam)) {
        pthread_mutex_lock(&g_lock);
        for (int i = 0; i < 3; i++) p.position[i] += slam[i] - g_origin[i];
        pthread_mutex_unlock(&g_lock);
        return p;
    }
    // Neck model: the eyes 7.5 cm above and 8 cm in front of a pivot that stays put.
    float eyes[3] = {0, 0.075f, -0.08f}, r[3];
    qrot(q, eyes, r);
    p.position[0] = r[0];
    p.position[1] = g_cfg.eye_height - 0.075f + r[1];
    p.position[2] = 0.08f + r[2];
    return p;
}

// Where a hand is: an elbow hanging below the shoulder on its side, and a forearm along the
// controller. The shoulders turn with the head only once it looks more than 35 degrees away, so
// looking around doesn't swing the arms.
static axr_motion hand_motion(int hand, int64_t time) {
    axr_motion m = {0};
    sense_state s;
    if (!g_cfg.controllers || !sense_get(hand, &s) || !s.updated_ns) return m;
    pthread_mutex_lock(&g_lock);
    quat align = g_ctl_align[hand];
    bool aligned = g_ctl_aligned[hand];
    pthread_mutex_unlock(&g_lock);
    if (!aligned) return m;
    quat head = pose_for(time);
    axr_pose hp = head_pose(head);
    float head_yaw = yaw_of(head);
    pthread_mutex_lock(&g_lock);
    float d = remainderf(head_yaw - g_torso_yaw, 2 * (float)M_PI);
    if (fabsf(d) > 35 * (float)M_PI / 180) g_torso_yaw += d - copysignf(35 * (float)M_PI / 180, d);
    quat torso = qyaw(g_torso_yaw);
    pthread_mutex_unlock(&g_lock);

    quat grip = qnorm(qmul(align, raw_grip(&s)));
    float side = hand == SENSE_LEFT ? -1 : 1;
    // The elbow 18 cm out to the side of and 52 cm below the eyes (a shoulder 22 cm below them,
    // an upper arm of 30 cm hanging down), then 32 cm of forearm and hand along the controller.
    float neck[3] = {hp.position[0], hp.position[1], hp.position[2]};
    float elbow_off[3] = {side * 0.18f, -0.52f, 0.02f}, elbow[3], fore[3] = {0, 0, -0.32f}, hand_off[3];
    qrot(torso, elbow_off, elbow);
    qrot(grip, fore, hand_off);
    for (int i = 0; i < 3; i++) {
        elbow[i] += neck[i];
        m.pose.position[i] = elbow[i] + hand_off[i];
    }
    m.pose.orientation[0] = grip.x; m.pose.orientation[1] = grip.y;
    m.pose.orientation[2] = grip.z; m.pose.orientation[3] = grip.w;
    float w[3];
    qrot(align, s.angular_velocity, w);
    memcpy(m.angular_velocity, w, sizeof w);
    // The hand moves with the forearm's turn about the elbow.
    m.linear_velocity[0] = w[1] * hand_off[2] - w[2] * hand_off[1];
    m.linear_velocity[1] = w[2] * hand_off[0] - w[0] * hand_off[2];
    m.linear_velocity[2] = w[0] * hand_off[1] - w[1] * hand_off[0];
    m.valid = true;

    // Where the cameras saw it, carried forward to this time, if they saw it lately. A switch
    // between that and the arm model eases over a fraction of a second instead of jumping.
    float seen[3], vel[3], base[3];
    pthread_mutex_lock(&g_lock);
    feed_imu(hand);
    bool optical = g_cfg.controller_tracking && strack_fusion_predict(&g_fusion[hand], time, 250000000, seen, vel);
    memcpy(base, optical ? seen : m.pose.position, sizeof base);
    if (optical != g_blend[hand].optical) {
        for (int i = 0; i < 3; i++) g_blend[hand].from[i] = g_blend[hand].have_last ? g_blend[hand].last[i] - base[i] : 0;
        g_blend[hand].since = time;
        g_blend[hand].optical = optical;
    }
    float age = (time - g_blend[hand].since) / 1e9f, ease = age > 0 ? expf(-age / (optical ? 0.12f : 0.25f)) : 1;
    for (int i = 0; i < 3; i++) m.pose.position[i] = base[i] + g_blend[hand].from[i] * ease;
    memcpy(g_blend[hand].last, m.pose.position, sizeof g_blend[hand].last);
    g_blend[hand].have_last = true;
    pthread_mutex_unlock(&g_lock);
    if (optical) memcpy(m.linear_velocity, vel, sizeof vel);
    return m;
}

// The camera thread's newest frame, waiting for tracking_thread: the lower pair as BC4 while
// passthrough has the cameras in that mode, or else all four half-size pictures.
static pthread_mutex_t g_track_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_track_cond = PTHREAD_COND_INITIALIZER;
static unsigned char *g_track_planes[2], g_track_header[256];
static unsigned char *g_track_half[STRACK_CAMERAS], g_track_half_header[2][256];
static bool g_track_has[STRACK_CAMERAS];
static double g_track_arrived;
static bool g_track_pending, g_track_is_half;
// The headset's spot finder (cpsvr2.h): its newest packet, for captures; and, while the cameras are
// in the tracking mode, the dots from it, 19 ms sooner than the pictures come
// (PSVR2XR_TRACK_SPOTS=0: from the pictures). At 32 it runs LEDs close together into one spot,
// which cost a controller lying side-on half a metre away a third of its LEDs on the bench; but
// on a recording of play its fits had as many LEDs as the pictures', and with the fusion the
// sooner fits put the controllers 20 to 30% closer to where they were when shown.
static unsigned char g_ld_packet[4][PSVR2_LD_PACKET_BYTES], g_track_ld[PSVR2_LD_PACKET_BYTES];   // the last 4
static unsigned g_ld_count;
static double g_stat_ld_ahead_s; static unsigned g_stat_ld_ahead_n;   // under g_track_lock
static bool g_ld_have, g_track_is_spots, g_use_spots;
static double g_ld_arrived, g_half_arrived, g_ld_packet_arrived[4];
// The lower pair waits here for the upper pair taken at the same moment.
static unsigned char *g_lower_wait[2], g_lower_wait_header[256];
static double g_lower_wait_arrived;
static bool g_lower_waiting;

// Statistics for the status line, under g_lock.
static unsigned g_stat_track_frames, g_stat_track_dots[STRACK_CAMERAS], g_stat_track_seen[2], g_stat_track_leds[2],
    g_stat_track_left_out[2];
static float g_stat_track_rms[2];

static void camera_frame(const unsigned char *header, const unsigned char *left, const unsigned char *right,
                         double arrived_s, void *ctx) {
    (void)ctx;
    sense_led_timing(0, 0, 0);   // the passthrough mode's long exposures see steady pulses
    pthread_mutex_lock(&g_track_lock);
    memcpy(g_track_planes[0], left, PSVR2_CAM_PLANE_BYTES);
    memcpy(g_track_planes[1], right, PSVR2_CAM_PLANE_BYTES);
    memcpy(g_track_header, header, sizeof g_track_header);
    g_track_arrived = arrived_s;
    g_track_is_half = false;
    g_track_is_spots = false;
    g_track_pending = true;
    pthread_cond_signal(&g_track_cond);
    pthread_mutex_unlock(&g_track_lock);
}

// The controllers' LEDs flash once on each exposure of the tracking mode (sense_led_timing). The
// headers carry the exposure time in the headset's microseconds (bytes 8 to 11, 16683 apart); the
// quickest arrival over the last couple of seconds ties that to our clock, and where in the frame
// to flash, for the latencies in between, tracking_thread finds by trying (g_led_shift_ns).
static int64_t g_expose_us64, g_expose_arrival_off[128];
static uint32_t g_expose_last_us;
static int g_expose_n;
static double g_expose_period_us = 16683;
static _Atomic int64_t g_led_shift_ns = -1;   // -1: not searched yet
static _Atomic int64_t g_expose_off_ns = INT64_MIN;   // our clock less the headset's, at the quickest arrival

static void led_timing(const unsigned char *header, double arrived_s) {
    uint32_t us = (uint32_t)header[8] | (uint32_t)header[9] << 8 | (uint32_t)header[10] << 16 | (uint32_t)header[11] << 24;
    uint32_t d = us - g_expose_last_us;
    if (g_expose_n && d > 5000 && d < 100000) {
        g_expose_us64 += d;
        double frames = round(d / g_expose_period_us);
        if (frames >= 1) g_expose_period_us += 0.02 * (d / frames - g_expose_period_us);
    } else {
        g_expose_us64 = us;   // the first, or a jump: start again
        g_expose_n = 0;
    }
    g_expose_last_us = us;
    int64_t arrival = (int64_t)(arrived_s * 1e9);
    g_expose_arrival_off[g_expose_n++ % 128] = arrival - g_expose_us64 * 1000;
    int n = g_expose_n < 128 ? g_expose_n : 128;
    int64_t off = INT64_MAX;
    for (int i = 0; i < n; i++)
        if (g_expose_arrival_off[i] < off) off = g_expose_arrival_off[i];
    if (n >= 30) g_expose_off_ns = off;
    int64_t shift = g_led_shift_ns;
    if (n >= 30 && shift >= 0)
        sense_led_timing(g_expose_us64 * 1000 + off, (int64_t)(g_expose_period_us * 1000), shift);
}

// The two pairs come one after the other with the same exposure time in their headers (bytes 8 to
// 11); both go to the tracker together. An upper pair without its lower one goes alone.
static void tracking_frame(const unsigned char *header, int upper, const unsigned char *left,
                           const unsigned char *right, double arrived_s, void *ctx) {
    (void)ctx;
    if (!upper) led_timing(header, arrived_s);
    pthread_mutex_lock(&g_track_lock);
    if (!upper) {
        memcpy(g_lower_wait[0], left, PSVR2_TRACK_PICTURE_BYTES);
        memcpy(g_lower_wait[1], right, PSVR2_TRACK_PICTURE_BYTES);
        memcpy(g_lower_wait_header, header, 256);
        g_lower_wait_arrived = arrived_s;
        g_lower_waiting = true;
    } else {
        bool pair = g_lower_waiting && !memcmp(g_lower_wait_header + 8, header + 8, 4);
        if (pair) {
            memcpy(g_track_half[0], g_lower_wait[0], PSVR2_TRACK_PICTURE_BYTES);
            memcpy(g_track_half[1], g_lower_wait[1], PSVR2_TRACK_PICTURE_BYTES);
            memcpy(g_track_half_header[0], g_lower_wait_header, 256);
        }
        memcpy(g_track_half[2], left, PSVR2_TRACK_PICTURE_BYTES);
        memcpy(g_track_half[3], right, PSVR2_TRACK_PICTURE_BYTES);
        memcpy(g_track_half_header[1], header, 256);
        g_track_has[0] = g_track_has[1] = pair;
        g_track_has[2] = g_track_has[3] = true;
        g_track_arrived = pair ? g_lower_wait_arrived : arrived_s;
        g_lower_waiting = false;
        g_half_arrived = arrived_s;
        // How much sooner the spot finder brought this exposure.
        for (int i = 0; i < 4 && g_ld_have; i++)
            if (!memcmp(g_ld_packet[i] + 8, header + 8, 4)) {
                g_stat_ld_ahead_s += arrived_s - g_ld_packet_arrived[i];
                g_stat_ld_ahead_n++;
            }
        // The pictures stay for captures while the spot finder drives the tracker.
        if (!(g_use_spots && g_ld_have && arrived_s - g_ld_arrived < 0.2)) {
            g_track_is_half = true;
            g_track_is_spots = false;
            g_track_pending = true;
            pthread_cond_signal(&g_track_cond);
        }
    }
    pthread_mutex_unlock(&g_track_lock);
}

// Each spot finder packet: the exposure it covers, in our clock as a lower pair arriving with it
// would have come (led_timing's tie), so tracking_thread takes off the same latency.
static void led_detector_frame(const unsigned char *packet, double arrived_s, void *ctx) {
    (void)ctx;
    uint32_t us = (uint32_t)packet[8] | (uint32_t)packet[9] << 8 | (uint32_t)packet[10] << 16 | (uint32_t)packet[11] << 24;
    int64_t off = g_expose_off_ns;
    double exposed_arrival = arrived_s;
    if (off != INT64_MIN && g_expose_n) {
        int64_t us64 = g_expose_us64 + (int32_t)(us - g_expose_last_us);
        exposed_arrival = (us64 * 1000 + off) / 1e9;
    }
    pthread_mutex_lock(&g_track_lock);
    g_ld_packet_arrived[g_ld_count % 4] = arrived_s;
    memcpy(g_ld_packet[g_ld_count++ % 4], packet, PSVR2_LD_PACKET_BYTES);
    g_ld_have = true;
    g_ld_arrived = arrived_s;
    // Only in the tracking mode (pictures coming half-size lately), not passthrough's.
    if (g_use_spots && arrived_s - g_half_arrived < 0.5) {
        memcpy(g_track_ld, packet, PSVR2_LD_PACKET_BYTES);
        g_track_arrived = exposed_arrival;
        g_track_is_spots = true;
        g_track_is_half = false;
        g_track_pending = true;
        pthread_cond_signal(&g_track_cond);
    }
    pthread_mutex_unlock(&g_track_lock);
}

// <config>/controller_cameras.json: where the cameras sit and point and how their lenses bend, in
// sense_track.h's terms. Each run starts from the built-in cameras and calibrates a little way from them; what it found is
// saved to look at. Starting from the last run's calibration let it wander further every run. A
// file whose cameras say "pinned": true is used instead, and left alone.
static bool g_cameras_pinned;

static NSString *cameras_path(void) { return [g_config_dir stringByAppendingPathComponent:@"controller_cameras.json"]; }

static void save_cameras(const strack_camera cams[STRACK_CAMERAS], int calibrations) {
    if (g_cameras_pinned) return;
    NSMutableArray *list = [NSMutableArray array];
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        const strack_camera *k = &cams[c];
        [list addObject:@{
            @"camera" : @[ @"lower left", @"lower right", @"upper left", @"upper right" ][c],
            @"f" : @(k->f), @"cx" : @(k->cx), @"cy" : @(k->cy),
            @"k" : @[ @(k->k[0]), @(k->k[1]), @(k->k[2]), @(k->k[3]) ],
            @"max_angle_deg" : @(k->max_angle_deg), @"width" : @(k->width), @"height" : @(k->height),
            @"position" : @[ @(k->position[0]), @(k->position[1]), @(k->position[2]) ],
            @"yaw_deg" : @(k->yaw_deg), @"pitch_deg" : @(k->pitch_deg), @"roll_deg" : @(k->roll_deg),
            @"calibrations" : @(calibrations), @"pinned" : @NO, @"off" : @(k->off),
        }];
    }
    [[NSJSONSerialization dataWithJSONObject:list options:NSJSONWritingPrettyPrinted | NSJSONWritingSortedKeys error:nil]
        writeToFile:cameras_path() atomically:YES];
}

static void load_cameras(strack_camera cams[STRACK_CAMERAS]) {
    strack_default_cameras(cams);
    NSData *data = [NSData dataWithContentsOfFile:cameras_path()];
    NSArray *list = data ? [NSJSONSerialization JSONObjectWithData:data options:0 error:nil] : nil;
    if (![list isKindOfClass:NSArray.class] || list.count != STRACK_CAMERAS) return;
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        NSDictionary *d = list[c];
        if (![d isKindOfClass:NSDictionary.class] || ![d[@"pinned"] boolValue]) continue;
        g_cameras_pinned = true;
        strack_camera *k = &cams[c];
        if (d[@"f"]) k->f = fmaxf(100, fminf(1000, [d[@"f"] floatValue]));
        if (d[@"cx"]) k->cx = [d[@"cx"] floatValue];
        if (d[@"cy"]) k->cy = [d[@"cy"] floatValue];
        NSArray *kk = d[@"k"];
        if ([kk isKindOfClass:NSArray.class] && kk.count == 4)
            for (int i = 0; i < 4; i++) k->k[i] = [kk[i] floatValue];
        if (d[@"max_angle_deg"]) k->max_angle_deg = fmaxf(30, fminf(89, [d[@"max_angle_deg"] floatValue]));
        NSArray *pos = d[@"position"];
        if ([pos isKindOfClass:NSArray.class] && pos.count == 3)
            for (int i = 0; i < 3; i++) k->position[i] = [pos[i] floatValue];
        if (d[@"yaw_deg"]) k->yaw_deg = [d[@"yaw_deg"] floatValue];
        if (d[@"pitch_deg"]) k->pitch_deg = [d[@"pitch_deg"] floatValue];
        if (d[@"roll_deg"]) k->roll_deg = [d[@"roll_deg"] floatValue];
        if (d[@"off"]) k->off = [d[@"off"] boolValue];
    }
}

// --capture-controllers (PSVR2XR_CAPTURE_DIR): every 4th camera frame, up to 300 (20 s), as it came
// (frame_NNN.bin: the 256-byte header, then the left and right BC4 planes; or frame_NNN.m4 in the
// tracking mode: the lower pair's header and two pictures, then the upper pair's, zeros for a pair
// that didn't come), with a line per frame in frames.jsonl of what the tracker was given and found,
// to work out from them what's wrong.
// PSVR2XR_CAPTURE_PICTURES=0 leaves the pictures out (the spot finder's packets stay), so
// PSVR2XR_CAPTURE_EVERY=1 and PSVR2XR_CAPTURE_FRAMES=3600 can keep a minute at the full rate; the
// controllers' IMU readings go to imu.jsonl.
static FILE *g_capture, *g_capture_imu;
static NSString *g_capture_dir;
static unsigned g_capture_frames, g_capture_saved;
static int64_t g_capture_imu_after[2];

static void capture_frame(bool half, const unsigned char *header, unsigned char *const planes[2],
                          unsigned char headers[2][256], unsigned char *const pictures[STRACK_CAMERAS],
                          const bool has[STRACK_CAMERAS], const strack_input *in, const strack_result r[2],
                          const unsigned char *ld_used) {
    static int every, limit = -1, with_pictures = -1;
    if (!every) every = getenv("PSVR2XR_CAPTURE_EVERY") ? atoi(getenv("PSVR2XR_CAPTURE_EVERY")) : 4;
    if (every < 1) every = 1;
    if (limit < 0) limit = getenv("PSVR2XR_CAPTURE_FRAMES") ? atoi(getenv("PSVR2XR_CAPTURE_FRAMES")) : 300;
    if (with_pictures < 0) with_pictures = !getenv("PSVR2XR_CAPTURE_PICTURES") || atoi(getenv("PSVR2XR_CAPTURE_PICTURES"));
    if (!g_capture || g_capture_saved >= (unsigned)limit) return;
    for (int h = 0; h < 2 && g_capture_imu; h++) {
        sense_sample ss[512];
        int n = sense_samples(h, g_capture_imu_after[h], ss, 512);
        for (int i = 0; i < n; i++)
            fprintf(g_capture_imu, "{\"hand\": %d, \"t\": %lld, \"q\": [%.6f, %.6f, %.6f, %.6f], \"a\": [%.4f, %.4f, %.4f], \"g\": [%.4f, %.4f, %.4f]}\n",
                    h, (long long)ss[i].t_ns, ss[i].q[0], ss[i].q[1], ss[i].q[2], ss[i].q[3], ss[i].accel[0], ss[i].accel[1],
                    ss[i].accel[2], ss[i].gyro[0], ss[i].gyro[1], ss[i].gyro[2]);
        if (n) g_capture_imu_after[h] = ss[n - 1].t_ns;
    }
    if (g_capture_frames++ % every) return;
    NSString *name = [NSString stringWithFormat:half ? @"frame_%03u.m4" : @"frame_%03u.bin", g_capture_saved++];
    FILE *f = with_pictures ? fopen([g_capture_dir stringByAppendingPathComponent:name].fileSystemRepresentation, "wb") : NULL;
    if (f && !half) {
        fwrite(header, 1, 256, f);
        fwrite(planes[0], 1, PSVR2_CAM_PLANE_BYTES, f);
        fwrite(planes[1], 1, PSVR2_CAM_PLANE_BYTES, f);
    } else if (f) {
        static unsigned char zeros[PSVR2_TRACK_PICTURE_BYTES];
        for (int pair = 0; pair < 2; pair++) {
            bool ok = has[2 * pair];
            fwrite(ok ? headers[pair] : zeros, 1, 256, f);
            fwrite(ok ? pictures[2 * pair] : zeros, 1, PSVR2_TRACK_PICTURE_BYTES, f);
            fwrite(ok ? pictures[2 * pair + 1] : zeros, 1, PSVR2_TRACK_PICTURE_BYTES, f);
        }
    }
    if (f) fclose(f);
    // The spot finder's packet for the same exposure beside it (ld_NNN.bin), or its newest.
    pthread_mutex_lock(&g_track_lock);
    bool ld = g_ld_have;
    static unsigned char ld_copy[PSVR2_LD_PACKET_BYTES];
    if (ld_used) memcpy(ld_copy, ld_used, sizeof ld_copy);
    else if (ld) {
        int pick = (g_ld_count - 1) % 4;
        for (int i = 0; i < 4 && half; i++)
            if (!memcmp(g_ld_packet[i] + 8, headers[1] + 8, 4)) pick = i;
        memcpy(ld_copy, g_ld_packet[pick], sizeof ld_copy);
    }
    pthread_mutex_unlock(&g_track_lock);
    if (ld || ld_used) {
        NSString *ld_name = [NSString stringWithFormat:@"ld_%03u.bin", g_capture_saved - 1];
        FILE *lf = fopen([g_capture_dir stringByAppendingPathComponent:ld_name].fileSystemRepresentation, "wb");
        if (lf) { fwrite(ld_copy, 1, sizeof ld_copy, lf); fclose(lf); }
    }
    unsigned lph, lpe, lbl;
    if (sense_led_test(&lph, &lpe, &lbl)) fprintf(g_capture, "{\"led_test\": [%u, %u, %u], ", lph, lpe, lbl);
    else fprintf(g_capture, "{");
    fprintf(g_capture, "\"file\": \"%s\", \"time_ns\": %lld, \"head\": {\"q\": [%.6f, %.6f, %.6f, %.6f], \"p\": [%.4f, %.4f, %.4f]}",
            name.UTF8String, (long long)in->time_ns, in->head.q[0], in->head.q[1], in->head.q[2], in->head.q[3],
            in->head.p[0], in->head.p[1], in->head.p[2]);
    for (int h = 0; h < 2; h++) {
        fprintf(g_capture, ", \"%s\": {\"imu\": %s", h ? "right" : "left", in->prior_valid[h] ? "true" : "false");
        pthread_mutex_lock(&g_lock);
        quat al = g_ctl_align[h];
        pthread_mutex_unlock(&g_lock);
        fprintf(g_capture, ", \"align_q\": [%.6f, %.6f, %.6f, %.6f]", al.x, al.y, al.z, al.w);
        if (in->prior_valid[h])
            fprintf(g_capture, ", \"prior_q\": [%.6f, %.6f, %.6f, %.6f]", in->prior_q[h][0], in->prior_q[h][1],
                    in->prior_q[h][2], in->prior_q[h][3]);
        if (r[h].valid)
            fprintf(g_capture, ", \"q\": [%.6f, %.6f, %.6f, %.6f], \"p\": [%.4f, %.4f, %.4f], \"leds\": %d, \"rms_px\": %.2f",
                    r[h].pose.q[0], r[h].pose.q[1], r[h].pose.q[2], r[h].pose.q[3], r[h].pose.p[0], r[h].pose.p[1],
                    r[h].pose.p[2], r[h].leds, r[h].rms_px);
        fprintf(g_capture, "}");
    }
    for (int c = 0; c < STRACK_CAMERAS; c++) {
        int n;
        const strack_blob *b = strack_blobs(g_tracker, c, &n);
        static const char *names[] = {"left", "right", "upper_left", "upper_right"};
        fprintf(g_capture, ", \"dots_%s\": [", names[c]);
        for (int i = 0; i < n; i++)
            fprintf(g_capture, "%s[%.2f, %.2f, %d, %.0f]", i ? ", " : "", b[i].x, b[i].y, b[i].area, b[i].brightness);
        fprintf(g_capture, "]");
    }
    fprintf(g_capture, "}\n");
    fflush(g_capture);
    if (g_capture_imu) fflush(g_capture_imu);
    if (g_capture_saved == (unsigned)limit) axr_log("controller capture done: %s", g_capture_dir.UTF8String);
}

// Finds where in the frame the LEDs must flash to be in the pictures: each shift in turn (1.5 ms
// apart, the flash being 2.1 ms long) for a few frames, counting the dots; the most dots win, then
// half a step either side. Searched again when the dots fall away for a while.
static void led_search(int dots, bool spots) {
    enum { STEP_NS = 1500000 };
    // in frames: the pictures' four cameras come 30 times a second, the spot finder's 60
    const int SETTLE = spots ? 8 : 4, COUNT = spots ? 12 : 6;
    static int step = -1, nsteps, frame, refine;
    static double sum, means[16], best, dark, level;
    static int64_t base, coarse, last_search_ns;
    static int recent_n;
    static double recent;
    int64_t period = (int64_t)(g_expose_period_us * 1000);
    if (step < 0 && g_led_shift_ns >= 0) {
        // Locked: watch the level.
        recent += dots;
        if (++recent_n < 120) return;
        double mean = recent / recent_n;
        recent = 0;
        recent_n = 0;
        if (mean > dark + 0.5 * (level - dark) || axr_now_ns() - last_search_ns < 5000000000LL) return;
        axr_log("controller LEDs: %.1f dots a frame, was %.1f; finding their timing again", mean, level);
    }
    if (step < 0) {   // start
        nsteps = (int)(period / STEP_NS);
        if (nsteps > 16) nsteps = 16;
        step = 0; frame = 0; sum = 0; refine = 0; base = 0;
        last_search_ns = axr_now_ns();
        g_led_shift_ns = 0;
        return;
    }
    if (++frame <= SETTLE) return;
    sum += dots;
    if (frame < SETTLE + COUNT) return;
    double mean = sum / COUNT;
    frame = 0;
    sum = 0;
    if (!refine) {
        means[step] = mean;
        if (++step < nsteps) { g_led_shift_ns = (int64_t)step * STEP_NS; return; }
        int b = 0;
        dark = means[0];
        for (int i = 1; i < nsteps; i++) {
            if (means[i] > means[b]) b = i;
            if (means[i] < dark) dark = means[i];
        }
        best = means[b];
        base = coarse = (int64_t)b * STEP_NS;
        refine = 1;
        g_led_shift_ns = (coarse - STEP_NS / 2 + period) % period;
        return;
    }
    if (mean > best) { best = mean; base = g_led_shift_ns; }
    if (refine == 1) {
        refine = 2;
        g_led_shift_ns = (coarse + STEP_NS / 2) % period;
        return;
    }
    g_led_shift_ns = base;
    level = best;
    step = -1;
    recent = 0;
    recent_n = 0;
    axr_log("controller LEDs: flashing %.1f ms into each %.2f ms frame (%.1f dots a frame there, %.1f at the worst)",
            base / 1e6, period / 1e6, best, dark);
}

static void *tracking_thread(void *arg) {
    (void)arg;
    unsigned char *planes[2] = {malloc(PSVR2_CAM_PLANE_BYTES), malloc(PSVR2_CAM_PLANE_BYTES)}, header[256];
    unsigned char *pictures[STRACK_CAMERAS], headers[2][256];
    bool has[STRACK_CAMERAS];
    static unsigned char ld[PSVR2_LD_PACKET_BYTES];
    for (int c = 0; c < STRACK_CAMERAS; c++)
        if (!(pictures[c] = malloc(PSVR2_TRACK_PICTURE_BYTES))) return NULL;
    if (!planes[0] || !planes[1]) return NULL;
    int saved_calibrations = 0;
    int64_t last_save = 0;
    for (;;) {
        pthread_mutex_lock(&g_track_lock);
        while (!g_track_pending) pthread_cond_wait(&g_track_cond, &g_track_lock);
        g_track_pending = false;
        bool half = g_track_is_half, spots = g_track_is_spots;
        if (spots) {
            memcpy(ld, g_track_ld, sizeof ld);
            // and the newest pictures, for a capture
            for (int c = 0; c < STRACK_CAMERAS; c++) {
                has[c] = g_track_has[c];
                if (g_capture && has[c]) memcpy(pictures[c], g_track_half[c], PSVR2_TRACK_PICTURE_BYTES);
            }
            if (g_capture) memcpy(headers, g_track_half_header, sizeof headers);
        } else if (half) {
            for (int c = 0; c < STRACK_CAMERAS; c++) {
                has[c] = g_track_has[c];
                if (has[c]) memcpy(pictures[c], g_track_half[c], PSVR2_TRACK_PICTURE_BYTES);
            }
            memcpy(headers, g_track_half_header, sizeof headers);
        } else {
            memcpy(planes[0], g_track_planes[0], PSVR2_CAM_PLANE_BYTES);
            memcpy(planes[1], g_track_planes[1], PSVR2_CAM_PLANE_BYTES);
            memcpy(header, g_track_header, sizeof header);
        }
        int64_t exposed = (int64_t)(g_track_arrived * 1e9) - (int64_t)(g_cfg.controller_latency_ms * 1e6f);
        pthread_mutex_unlock(&g_track_lock);

        if (g_tracker_reset) {
            g_tracker_reset = false;
            strack_forget(g_tracker);
        }
        int dots[STRACK_CAMERAS] = {0};
        for (int c = 0; c < STRACK_CAMERAS; c++) {
            if (spots)
                dots[c] = strack_detect_spots(g_tracker, c, ld + 0x40 + c * PSVR2_LD_CAMERA_BYTES);
            else if (half && has[c])   // the last 4 columns are padding
                dots[c] = strack_detect_half(g_tracker, c, pictures[c], PSVR2_TRACK_WIDTH - 4, PSVR2_TRACK_HEIGHT,
                                             PSVR2_TRACK_WIDTH);
            else if (!half && c < 2)
                dots[c] = strack_detect_bc4(g_tracker, c, planes[c], PSVR2_CAM_WIDTH / 4);
            else
                strack_no_picture(g_tracker, c);
        }
        if (half || spots) led_search(dots[0] + dots[1] + dots[2] + dots[3], spots);

        // The head then, and each controller's orientation from its IMU.
        strack_input in = {.time_ns = exposed};
        quat hq = QID;
        head_orientation((exposed - axr_now_ns()) / 1e9f, &hq);
        axr_pose hp = head_pose(hq);
        in.head.q[0] = hq.x; in.head.q[1] = hq.y; in.head.q[2] = hq.z; in.head.q[3] = hq.w;
        memcpy(in.head.p, hp.position, sizeof in.head.p);
        for (int h = 0; h < 2; h++) {
            sense_state st;
            if (!sense_get(h, &st) || !st.updated_ns || axr_now_ns() - (int64_t)st.updated_ns > 200000000 || !sense_leds_on(h))
                continue;
            pthread_mutex_lock(&g_lock);
            bool aligned = g_ctl_aligned[h];
            quat g = qnorm(qmul(g_ctl_align[h], raw_grip(&st)));
            pthread_mutex_unlock(&g_lock);
            if (!aligned) continue;
            in.prior_valid[h] = true;
            in.prior_q[h][0] = g.x; in.prior_q[h][1] = g.y; in.prior_q[h][2] = g.z; in.prior_q[h][3] = g.w;
        }
        strack_result r[2];
        strack_solve(g_tracker, &in, r);

        pthread_mutex_lock(&g_lock);
        g_stat_track_frames++;
        for (int c = 0; c < STRACK_CAMERAS; c++) g_stat_track_dots[c] += dots[c];
        for (int h = 0; h < 2; h++) {
            if (!r[h].valid) continue;
            feed_imu(h);
            // A fit the fusion leaves out (far from where the accelerometer says the controller
            // went: on the wrong dots) doesn't correct the IMU's yaw either.
            if (!strack_fusion_optical(&g_fusion[h], exposed, r[h].pose.p, r[h].leds, r[h].rms_px)) {
                g_stat_track_left_out[h]++;
                continue;
            }
            g_optical_seen[h] = exposed;
            // The IMU's yaw toward the cameras', a little at a time, and only from fits on plenty of
            // LEDs: weak fits fed back here at a tenth a frame spun a controller half a turn in ten
            // seconds while it sat still. (Held to under 1.5 px as well, it hardly ever took any in,
            // and the IMU's drift showed.)
            // Overhead a controller is often seen on only four or five LEDs, while its IMU's turn
            // about its own length (the world's vertical, pointing up) goes unchecked by gravity
            // and drifts; such fits correct it at half the rate, if they fit well.
            float gain = r[h].leds >= 6 && r[h].rms_px <= 2.5f ? 0.03f : r[h].leds >= 4 && r[h].rms_px <= 1.5f ? 0.015f : 0;
            if (gain > 0) {
                float a = gain * r[h].yaw_error;
                g_ctl_align[h] = qnorm(qmul(qyaw(a), g_ctl_align[h]));
                strack_shift_yaw(g_tracker, h, a);
            }
            // Its tilt too, from fits on 8 LEDs or more: the IMU's level is often 5 to 15 degrees off
            // what the cameras see, and a fit on a few LEDs then holds to a wrong tilt and lands on
            // the wrong dots. (On recordings of play this halved the jumps of tens of centimetres.)
            if (r[h].leds >= 8 && r[h].rms_px <= 2.5f) {
                float ex = 0.03f * r[h].tilt_error[0], ez = 0.03f * r[h].tilt_error[1], a = sqrtf(ex * ex + ez * ez);
                if (a > 1e-7f) {
                    quat dq = {ex / a * sinf(a / 2), 0, ez / a * sinf(a / 2), cosf(a / 2)};
                    g_ctl_align[h] = qnorm(qmul(dq, g_ctl_align[h]));
                }
            }
            g_stat_track_seen[h]++;
            g_stat_track_leds[h] += r[h].leds;
            g_stat_track_rms[h] += r[h].rms_px;
        }
        pthread_mutex_unlock(&g_lock);

        capture_frame(half || spots, header, planes, headers, pictures, has, &in, r, spots ? ld : NULL);
        float rms;
        int calibrations = strack_calibration(g_tracker, &rms);
        if (calibrations > saved_calibrations && axr_now_ns() - last_save > 20000000000LL) {
            strack_camera cams[STRACK_CAMERAS];
            strack_get_cameras(g_tracker, cams);
            save_cameras(cams, calibrations);
            axr_log("controller cameras calibrated (%d times, fit %.2f px): left f %.1f yaw %.1f pitch %.1f roll %.1f,"
                    " right f %.1f yaw %.1f pitch %.1f roll %.1f; saved to %s", calibrations, rms, cams[0].f,
                    cams[0].yaw_deg, cams[0].pitch_deg, cams[0].roll_deg, cams[1].f, cams[1].yaw_deg, cams[1].pitch_deg,
                    cams[1].roll_deg, cameras_path().UTF8String);
            saved_calibrations = calibrations;
            last_save = axr_now_ns();
        }
    }
    return NULL;
}

static void start_controller_tracking(void) {
    for (int h = 0; h < 2; h++) {
        strack_fusion_init(&g_fusion[h]);
        g_fusion[h].blend_s *= g_cfg.controller_smoothing;   // how long a correction takes to ease in
    }
    strack_camera cams[STRACK_CAMERAS];
    load_cameras(cams);
    strack_params prm;
    strack_default_params(&prm);
    // In the headset the calibration wandered (12 degrees and 15% in a minute), trading focal
    // length and pitch against where the controllers are, and they jumped with it. The built-in
    // cameras, fitted to a capture, do better left alone; PSVR2XR_CALIBRATE_CAMERAS=1 turns it on.
    const char *cal = getenv("PSVR2XR_CALIBRATE_CAMERAS");
    prm.calibrate = cal && atoi(cal);
    prm.threshold = g_cfg.controller_threshold;
    g_tracker = strack_create(cams, &prm);
    g_track_planes[0] = malloc(PSVR2_CAM_PLANE_BYTES);
    g_track_planes[1] = malloc(PSVR2_CAM_PLANE_BYTES);
    bool ok = g_tracker && g_track_planes[0] && g_track_planes[1];
    for (int c = 0; c < STRACK_CAMERAS; c++) ok = ok && (g_track_half[c] = malloc(PSVR2_TRACK_PICTURE_BYTES));
    for (int i = 0; i < 2; i++) ok = ok && (g_lower_wait[i] = malloc(PSVR2_TRACK_PICTURE_BYTES));
    if (!ok) { axr_log("controller tracking: out of memory"); return; }
    const char *capture = getenv("PSVR2XR_CAPTURE_DIR");
    if (capture) {
        g_capture_dir = @(capture);
        [NSFileManager.defaultManager createDirectoryAtPath:g_capture_dir withIntermediateDirectories:YES attributes:nil error:nil];
        g_capture = fopen([g_capture_dir stringByAppendingPathComponent:@"frames.jsonl"].fileSystemRepresentation, "w");
        g_capture_imu = fopen([g_capture_dir stringByAppendingPathComponent:@"imu.jsonl"].fileSystemRepresentation, "w");
        save_cameras(cams, 0);
        [NSFileManager.defaultManager copyItemAtPath:cameras_path()
                                              toPath:[g_capture_dir stringByAppendingPathComponent:@"controller_cameras.json"]
                                               error:nil];
        axr_log("controller capture: saving camera frames to %s", capture);
    }
    psvr2_camera_set_listener(camera_frame, NULL);
    psvr2_camera_set_tracking_listener(tracking_frame, NULL);
    const char *spots = getenv("PSVR2XR_TRACK_SPOTS");
    g_use_spots = !spots || atoi(spots);
    psvr2_camera_set_led_detector_listener(led_detector_frame, NULL);
    if (psvr2_camera_start_tracking() != 0) {
        axr_log("controller tracking: the headset's cameras did not start; using the arm model");
        psvr2_camera_set_listener(NULL, NULL);
        psvr2_camera_set_tracking_listener(NULL, NULL);
        psvr2_camera_set_led_detector_listener(NULL, NULL);
        return;
    }
    sense_tracking_leds(true);
    pthread_t t;
    pthread_create(&t, NULL, tracking_thread, NULL);
    pthread_detach(t);
    axr_log("controller tracking on: lower cameras f %.0f, turned %.0f/%.0f deg out and %.0f down; upper cameras %s (%s)",
            cams[0].f, cams[0].yaw_deg, -cams[1].yaw_deg, -cams[0].pitch_deg, cams[2].off ? "off" : "on",
            cameras_path().UTF8String);
}

// ------------------------------------------------------------------------------------------ eyes
//
// The eye tracker's combined gaze, smoothed by a one-euro filter (little smoothing while the eyes
// move fast, so the inset keeps up with a glance, more while they rest, so it doesn't shimmer),
// and held through a blink for a moment.

static pthread_mutex_t g_eye_lock = PTHREAD_MUTEX_INITIALIZER;
static bool g_eyes_on;             // the eye tracker started
static NSString *g_eye_calibration_path;
static struct {
    unsigned frame;                // the last gaze frame filtered
    bool have;
    float dir[3], rate[3];         // filtered direction, and its filtered rate of change
    double t, valid_t;             // when that frame arrived; the last valid one
} g_eye;

// Calibration targets, drawn by the compositor while the calibration runs (calibration_thread).
static struct {
    int mode;                      // 0 none, 1 dark background, 2 light background
    float target[3];               // in the head, metres (x right, y up, -z forward)
    int64_t shown_ns;
    bool running;
} g_cal;

static float euro_alpha(float cutoff_hz, float dt) {
    float tau = 1.0f / (2 * (float)M_PI * cutoff_hz);
    return 1.0f / (1.0f + tau / dt);
}

// Where the eyes look, as a unit vector in the head (x right, y up, -z forward). False without
// a recent valid sample.
static bool gaze_in_head(float out[3]) {
    if (!g_eyes_on) return false;
    psvr2_gaze g;
    if (psvr2_get_gaze(&g) != 1) return false;
    double now = axr_now_ns() / 1e9;
    pthread_mutex_lock(&g_eye_lock);
    if (g.valid && g.frame != g_eye.frame) {
        double t = now - g.age_s;
        if (!g_eye.have || t - g_eye.t > 0.25) {
            memcpy(g_eye.dir, g.dir, sizeof g_eye.dir);
            memset(g_eye.rate, 0, sizeof g_eye.rate);
        } else {
            float dt = fmaxf(0.001f, (float)(t - g_eye.t)), speed = 0, n = 0;
            float ar = euro_alpha(1.0f, dt);
            for (int i = 0; i < 3; i++) {
                g_eye.rate[i] += ar * ((g.dir[i] - g_eye.dir[i]) / dt - g_eye.rate[i]);
                speed += g_eye.rate[i] * g_eye.rate[i];
            }
            float a = euro_alpha(2.0f + 2.0f * sqrtf(speed), dt);   // 2 Hz at rest, faster when moving
            for (int i = 0; i < 3; i++) {
                g_eye.dir[i] += a * (g.dir[i] - g_eye.dir[i]);
                n += g_eye.dir[i] * g_eye.dir[i];
            }
            n = sqrtf(n);
            for (int i = 0; i < 3 && n > 0; i++) g_eye.dir[i] /= n;
        }
        g_eye.frame = g.frame;
        g_eye.t = g_eye.valid_t = t;
        g_eye.have = true;
    }
    bool ok = g_eye.have && now - g_eye.valid_t < 0.3;
    if (ok) memcpy(out, g_eye.dir, sizeof g_eye.dir);
    pthread_mutex_unlock(&g_eye_lock);
    return ok;
}

// The rotation that turns -Z to look along d, without roll.
static quat look_along(const float d[3]) {
    float yaw = atan2f(-d[0], -d[2]), pitch = asinf(fmaxf(-1, fminf(1, d[1])));
    return qnorm(qmul(qyaw(yaw), (quat){sinf(pitch / 2), 0, 0, cosf(pitch / 2)}));
}

bool axr_backend_has_eye_tracker(void) { return true; }

bool axr_eye_tracking(void) { return g_eyes_on; }

// The eye tracker's calibration, as PSVR2Toolkit.UnitySetup runs it: 8 targets on a dark
// background (5 far, 3 near), the headset computes, then 5 on a light one, and it computes again.
// Targets are in millimetres in the eye tracker's axes (x left, y up, +z ahead). At the end the
// headset is asked to report the result, which cpsvr2.c saves to send again on the next start.
static const float k_cal_points[][3] = {
    {0, 0, 2000},    {-400, 400, 2000}, {400, -400, 2000}, {400, 400, 2000}, {-400, -400, 2000},
    {0, 0, 300},     {-100, 0, 300},    {100, 0, 300},
    {0, 0, 2000},    {-1000, 0, 2000},  {1000, 0, 2000},   {0, 1000, 2000},  {0, -1000, 2000},
};
enum { CAL_POINTS = sizeof k_cal_points / sizeof k_cal_points[0], CAL_LIGHT_FROM = 8 };

static void cal_show(int mode, const float mm[3]) {
    pthread_mutex_lock(&g_eye_lock);
    g_cal.mode = mode;
    if (mm) {
        g_cal.target[0] = -mm[0] / 1000;
        g_cal.target[1] = mm[1] / 1000;
        g_cal.target[2] = -mm[2] / 1000;
    }
    g_cal.shown_ns = axr_now_ns();
    pthread_mutex_unlock(&g_eye_lock);
}

static void sleep_s(double s) {
    struct timespec d = {(time_t)s, (long)((s - (time_t)s) * 1e9)};
    nanosleep(&d, NULL);
}

// Polls the eye tracker for up to timeout_s: a collected point (result OK), or a finished compute
// (status OK or failed). Returns 1 on success, 0 on failure, -1 on timeout.
static int cal_wait(bool compute, double timeout_s) {
    for (double t = 0; t < timeout_s; t += 0.05) {
        sleep_s(0.05);
        int status = 0, result = 0;
        if (psvr2_gaze_calibration_status(&status, &result) != 0) continue;
        if (!compute && result == PSVR2_GAZE_RESULT_OK) return 1;
        if (compute && status == PSVR2_GAZE_STATUS_COMPUTE_OK) return 1;
        if (compute && status == PSVR2_GAZE_STATUS_COMPUTE_FAILED) return 0;
    }
    return -1;
}

static void *calibration_thread(void *arg) {
    (void)arg;
    axr_log("eye calibration: look at each dot until the next one appears");
    psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_SET_EYE, 0, 0, 0, 2);
    psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_START, 0, 0, 0, 0);
    bool ok = true;
    for (int i = 0; i < CAL_POINTS && ok; i++) {
        const float *pt = k_cal_points[i];
        cal_show(i < CAL_LIGHT_FROM ? 1 : 2, pt);
        sleep_s(1.0);   // time to find the dot and settle on it
        psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_COLLECT, pt[0], pt[1], pt[2], 0);
        int r = cal_wait(false, 10);
        if (r != 1) {
            axr_log("eye calibration: point %d %s", i + 1, r < 0 ? "timed out" : "failed");
            ok = false;
            break;
        }
        bool stage_end = i == CAL_LIGHT_FROM - 1 || i == CAL_POINTS - 1;
        if (!stage_end) continue;
        psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_COMPUTE, 0, 0, 0, 0);
        r = cal_wait(true, 10);
        if (r != 1) {
            axr_log("eye calibration: the headset's compute after point %d %s", i + 1, r < 0 ? "timed out" : "failed");
            ok = false;
            break;
        }
        if (i == CAL_LIGHT_FROM - 1) {
            axr_log("eye calibration: dark stage done");
            cal_show(2, k_cal_points[CAL_LIGHT_FROM]);   // the eyes adjust to the light first
            sleep_s(2.0);
        }
    }
    cal_show(0, NULL);
    psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_STOP, 0, 0, 0, 0);
    psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_SET_EYE, 0, 0, 0, 2);
    psvr2_gaze_calibration_command(PSVR2_GAZE_CAL_RETRIEVE, 0, 0, 0, 0);
    if (ok) {
        double age = -1;
        for (int t = 0; t < 30; t++) {
            sleep_s(0.1);
            age = psvr2_gaze_calibration_saved_age();
            if (age >= 0 && age < 4) break;
        }
        axr_log("eye calibration succeeded%s", age >= 0 && age < 4 ? "; saved, and sent again on every start"
                                                                    : ", but the headset did not report it to save;"
                                                                      " it holds until the headset turns off");
    } else {
        axr_log("eye calibration did not finish; hold Fn for 2 seconds to try again");
    }
    pthread_mutex_lock(&g_eye_lock);
    g_cal.running = false;
    pthread_mutex_unlock(&g_eye_lock);
    return NULL;
}

// Starts the calibration unless it is running or there is no eye tracker.
void axr_eye_calibrate(void) {
    pthread_mutex_lock(&g_eye_lock);
    bool start = g_eyes_on && !g_cal.running;
    if (start) g_cal.running = true;
    pthread_mutex_unlock(&g_eye_lock);
    if (!start) {
        if (!g_eyes_on) axr_log("eye calibration: the eye tracker is off or not found");
        return;
    }
    pthread_t t;
    pthread_create(&t, NULL, calibration_thread, NULL);
    pthread_detach(t);
}

// --------------------------------------------------------------------------- foveated rendering

// The game's eye images cover this tan-space rectangle (see axr_current_views).
static const float k_FX = 0.3585564f, k_FY = 0.3762281f, k_CX = 0.6603788f;

bool axr_foveated_supported(void) { return g_cfg.foveated; }

void axr_foveated_sizes(uint32_t outer[2], uint32_t inset[2]) {
    float s = g_cfg.render_scale * g_cfg.foveated_outer_scale;
    float t = 2 * tanf(g_cfg.foveated_area_deg / 2 * (float)M_PI / 180) * g_cfg.foveated_inset_scale;
    outer[0] = (uint32_t)MAX(16, lroundf(2000 * s / 16) * 16);
    outer[1] = (uint32_t)MAX(16, lroundf(1906 * s / 16) * 16);
    // The panel has 2000 * FX pixels per unit of tangent at its centre, across and down.
    inset[0] = (uint32_t)MAX(16, lroundf(2000 * k_FX * t / 16) * 16);
    inset[1] = (uint32_t)MAX(16, lroundf(1906 * k_FY * t / 16) * 16);
}

void axr_foveated_inset(int64_t time, bool gaze, float fov[2][4]) {
    (void)time;   // the newest gaze: the eyes move too fast and too suddenly to predict
    axr_views v = axr_current_views();
    float half = g_cfg.foveated_area_deg / 2 * (float)M_PI / 180, ax = 0, ay = 0, d[3];
    if (gaze && gaze_in_head(d) && d[2] < -0.2f) {
        ax = atanf(d[0] / -d[2]);
        ay = atanf(d[1] / -d[2]);
    }
    // Moved, not shrunk, to stay inside the eye's view.
    for (int e = 0; e < 2; e++) {
        float cx = fmaxf(v.fov[e][0] + half, fminf(v.fov[e][1] - half, ax));
        float cy = fmaxf(v.fov[e][3] + half, fminf(v.fov[e][2] - half, ay));
        fov[e][0] = cx - half;
        fov[e][1] = cx + half;
        fov[e][2] = cy + half;
        fov[e][3] = cy - half;
    }
}

axr_motion axr_device_motion(int device, int64_t time) {
    if (device == AXR_EYES) {
        axr_motion m = {0};
        float d[3];
        if (!gaze_in_head(d)) return m;
        quat q = pose_for(time);
        m.pose = head_pose(q);
        quat o = qnorm(qmul(q, look_along(d)));
        m.pose.orientation[0] = o.x; m.pose.orientation[1] = o.y;
        m.pose.orientation[2] = o.z; m.pose.orientation[3] = o.w;
        m.valid = true;
        return m;
    }
    if (device == AXR_LEFT) return hand_motion(SENSE_LEFT, time);
    if (device == AXR_RIGHT) return hand_motion(SENSE_RIGHT, time);
    axr_motion m = {0};
    if (device != AXR_HEAD) return m;
    quat q = pose_for(time);
    m.pose = head_pose(q);
    float gyro[3];
    double age;
    if (g_usb && psvr2_get_motion(gyro, &age) == 1) qrot(q, gyro, m.angular_velocity);
    m.valid = true;
    return m;
}

// The game's eye images cover the tan-space rectangle the compositor reads (psvr2_compose.metal):
// x from -cx / FX to (1 - cx) / FX, y from -0.5 / FY to 0.5 / FY, with the wider side outwards.
axr_views axr_current_views(void) {
    const float FX = k_FX, FY = k_FY, cx = k_CX;
    int prox = 0, ipd_mm = 63;
    if (g_usb) psvr2_get_status(&prox, &ipd_mm);
    if (ipd_mm < 55 || ipd_mm > 75) ipd_mm = 63;
    float half = ipd_mm / 2000.0f, up = atanf(0.5f / FY);
    float outer = atanf(cx / FX), inner = atanf((1 - cx) / FX);
    return (axr_views){
        .eye_in_head = {{{0, 0, 0, 1}, {-half, 0, 0}}, {{0, 0, 0, 1}, {half, 0, 0}}},
        .fov = {{-outer, inner, up, -up}, {-inner, outer, up, -up}},
    };
}

// ------------------------------------------------------------------------------------- input

// runtime.m binds games to the Quest Touch profile and asks for its OpenXR input paths, as ALVR
// names them. The Sense controllers map onto it by position: square/cross are X/A, triangle/circle
// Y/B, L1/R1 the grip, create/options the menu button.
bool axr_input(const char *path, float *value) {
    int hand;
    const char *rest;
    if (!strncmp(path, "/user/hand/left/input/", 22)) hand = SENSE_LEFT, rest = path + 22;
    else if (!strncmp(path, "/user/hand/right/input/", 23)) hand = SENSE_RIGHT, rest = path + 23;
    else return false;
    sense_state s;
    if (!g_cfg.controllers || !sense_get(hand, &s)) return false;
    uint32_t b = s.buttons;
    bool left = hand == SENSE_LEFT;
    float stick[2] = {s.stick[0], s.stick[1]};
    float r = hypotf(stick[0], stick[1]);   // a small round dead zone, rescaled to reach 1
    float k = r < 0.08f ? 0 : fminf(1, (r - 0.08f) / 0.92f) / r;
    stick[0] *= k;
    stick[1] *= k;
#define IS(x) (!strcmp(rest, x))
#define BIT(m) ((b & (m)) ? 1.0f : 0.0f)
    float v;
    if (IS(left ? "x/click" : "a/click")) v = BIT(SENSE_FACE_LOWER);
    else if (IS(left ? "x/touch" : "a/touch")) v = BIT(SENSE_FACE_LOWER_TOUCH);
    else if (IS(left ? "y/click" : "b/click")) v = BIT(SENSE_FACE_UPPER);
    else if (IS(left ? "y/touch" : "b/touch")) v = BIT(SENSE_FACE_UPPER_TOUCH);
    else if (IS("menu/click") || IS("system/click")) v = BIT(SENSE_MENU);
    else if (IS("squeeze/value") || IS("squeeze/click")) v = BIT(SENSE_GRIP);
    else if (IS("squeeze/touch")) v = BIT(SENSE_GRIP | SENSE_GRIP_TOUCH);
    else if (IS("trigger/value")) v = s.trigger;
    else if (IS("trigger/click")) v = s.trigger > 0.75f ? 1 : 0;
    else if (IS("trigger/touch")) v = (b & SENSE_TRIGGER_TOUCH) || s.trigger > 0.02f ? 1 : 0;
    else if (IS("thumbstick/x")) v = stick[0];
    else if (IS("thumbstick/y")) v = stick[1];
    else if (IS("thumbstick/click")) v = BIT(SENSE_STICK_CLICK);
    else if (IS("thumbstick/touch")) v = BIT(SENSE_STICK_TOUCH | SENSE_STICK_CLICK);
    else return false;
#undef IS
#undef BIT
    *value = v;
    return true;
}

uint64_t axr_input_generation(void) { return sense_generation(); }

void axr_haptics(int hand, float duration_s, float frequency, float amplitude) {
    if (!g_cfg.controllers) return;
    if (hand == AXR_LEFT) sense_vibrate(SENSE_LEFT, duration_s, frequency, amplitude);
    if (hand == AXR_RIGHT) sense_vibrate(SENSE_RIGHT, duration_s, frequency, amplitude);
}

// ------------------------------------------------------------------------------ frame timing

bool axr_backend_connected(void) { return g_usb && psvr2_connected(); }

bool axr_backend_tracking(void) {
    pthread_mutex_lock(&g_lock);
    bool ready = g_tracking_ready;
    pthread_mutex_unlock(&g_lock);
    return ready && axr_backend_connected();
}

double axr_refresh_hz(void) {
    pthread_mutex_lock(&g_lock);
    double r = g_refresh;
    pthread_mutex_unlock(&g_lock);
    return r;
}

void axr_recommended_eye_size(uint32_t *w, uint32_t *h) {
    *w = (uint32_t)(lroundf(2000 * g_cfg.render_scale / 16) * 16);
    *h = (uint32_t)(lroundf(1906 * g_cfg.render_scale / 16) * 16);
}

// The first vsync at or after t, on the grid of a known one (which may be before or after t).
static int64_t vsync_at_or_after(int64_t vsync, int64_t t, int64_t period) {
    int64_t d = t - vsync;
    int64_t k = d >= 0 ? (d + period - 1) / period : -((-d) / period);
    return vsync + k * period;
}

// A frame starts on a vsync of the headset display, no sooner than a period after the last one.
// It is composed at the next vsync and scanned out at the one after, which is its display time;
// the compositor turns it by whatever the prediction got wrong.
int64_t axr_wait_frame(void) {
    pthread_mutex_lock(&g_lock);
    int64_t period = g_period_ns, vsync = g_vsync_ns, last = g_last_wait;
    pthread_mutex_unlock(&g_lock);
    int64_t now = axr_now_ns();
    if (!vsync) vsync = now;
    int64_t earliest = last ? last + period - period / 4 : now;
    int64_t start = vsync_at_or_after(vsync, earliest > now ? earliest : now, period);
    if (start > now) {
        struct timespec d = {(start - now) / 1000000000, (start - now) % 1000000000};
        nanosleep(&d, NULL);
    }
    int64_t display = start + 2 * period;
    float ahead = ((float)(display - axr_now_ns()) + g_cfg.lookahead_ms * 1e6f) / 1e9f;
    quat q = QID;
    head_orientation(ahead, &q);
    pthread_mutex_lock(&g_lock);
    g_last_wait = start;
    if (display <= g_last_display) display = g_last_display + 1;   // display times must rise
    g_last_display = display;
    g_poses[g_pose_next % POSE_RING].time = display;
    g_poses[g_pose_next % POSE_RING].q = q;
    g_pose_next++;
    pthread_mutex_unlock(&g_lock);
    return display;
}

// ---------------------------------------------------------------------------------- submission

// Copies n eye images (2, or 4 with quad views) into a free slot.
static void submit_views(int64_t display_time, const axr_eye_image *eyes, int n, const float inset_fov[2][4],
                         id<MTLCommandQueue> queue) {
    quat q = pose_for(display_time);
    pthread_mutex_lock(&g_lock);
    g_stat_submits++;
    int s = -1;
    for (int i = 0; i < SLOTS; i++)
        if (i != g_latest && !g_slots[i].readers && !g_slots[i].pending) { s = i; break; }
    if (s < 0) { g_stat_dropped++; pthread_mutex_unlock(&g_lock); return; }
    g_slots[s].pending = true;
    pthread_mutex_unlock(&g_lock);

    // Only this thread touches a pending slot's textures.
    g_slots[s].quad = n == 4;
    if (inset_fov) memcpy(g_slots[s].inset_fov, inset_fov, sizeof g_slots[s].inset_fov);
    for (int e = 0; e < n; e++) {
        id<MTLTexture> src = eyes[e].texture, dst = g_slots[s].eye[e];
        if (!dst || dst.width != (NSUInteger)eyes[e].width || dst.height != (NSUInteger)eyes[e].height ||
            dst.pixelFormat != src.pixelFormat) {
            MTLTextureDescriptor *d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:src.pixelFormat
                                                                                         width:eyes[e].width
                                                                                        height:eyes[e].height
                                                                                     mipmapped:NO];
            d.usage = MTLTextureUsageShaderRead;
            d.storageMode = MTLStorageModePrivate;
            g_slots[s].eye[e] = [axr_metal_device() newTextureWithDescriptor:d];
            axr_log("%s %d images: %dx%d, format %lu", e < 2 ? "eye" : "inset", e % 2, eyes[e].width, eyes[e].height,
                    (unsigned long)src.pixelFormat);
        }
    }
    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    for (int e = 0; e < n; e++)
        [blit copyFromTexture:eyes[e].texture sourceSlice:eyes[e].slice sourceLevel:0
                 sourceOrigin:MTLOriginMake(eyes[e].x, eyes[e].y, 0)
                   sourceSize:MTLSizeMake(eyes[e].width, eyes[e].height, 1)
                    toTexture:g_slots[s].eye[e] destinationSlice:0 destinationLevel:0
            destinationOrigin:MTLOriginMake(0, 0, 0)];
    [blit endEncoding];
    [cmd addCompletedHandler:^(id<MTLCommandBuffer> done) {
        pthread_mutex_lock(&g_lock);
        g_slots[s].pending = false;
        if (done.status == MTLCommandBufferStatusCompleted) {
            g_slots[s].q = q;
            g_slots[s].seq = ++g_seq;
            g_latest = s;
        }
        pthread_mutex_unlock(&g_lock);
    }];
    [cmd commit];
}

void axr_submit(int64_t display_time, const axr_eye_image eyes[2], id<MTLCommandQueue> queue) {
    submit_views(display_time, eyes, 2, NULL, queue);
}

void axr_submit_quad(int64_t display_time, const axr_eye_image eyes[4], const float inset_fov[2][4],
                     id<MTLCommandQueue> queue) {
    submit_views(display_time, eyes, 4, inset_fov, queue);
}

// ---------------------------------------------------------------------------------- compositor

// The newest camera frames, in new textures each time so a frame still on the GPU is left alone,
// as psvr2-player-mac does. False until the first frame after the cameras come on.
static bool update_camera_textures(void) {
    if (!g_passthrough) {
        g_cam[0] = g_cam[1] = nil;
        return false;
    }
    if (!g_cam_buf[0]) {
        g_cam_buf[0] = malloc(PSVR2_CAM_PLANE_BYTES);
        g_cam_buf[1] = malloc(PSVR2_CAM_PLANE_BYTES);
        if (!g_cam_buf[0] || !g_cam_buf[1]) return false;
    }
    if (psvr2_camera_get_frame(g_cam_buf[0], g_cam_buf[1]) == 1) {
        MTLTextureDescriptor *d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBC4_RUnorm
                                                                                     width:PSVR2_CAM_WIDTH
                                                                                    height:PSVR2_CAM_HEIGHT
                                                                                 mipmapped:NO];
        d.usage = MTLTextureUsageShaderRead;
        d.storageMode = MTLStorageModeShared;
        for (int i = 0; i < 2; i++) {
            id<MTLTexture> t = [g_device newTextureWithDescriptor:d];
            // BC4: a 4x4 block is 8 bytes, so a row of blocks is width * 2 bytes
            [t replaceRegion:MTLRegionMake2D(0, 0, PSVR2_CAM_WIDTH, PSVR2_CAM_HEIGHT) mipmapLevel:0
                   withBytes:g_cam_buf[i] bytesPerRow:PSVR2_CAM_WIDTH * 2];
            g_cam[i] = t;
        }
    }
    return g_cam[0] && g_cam[1];
}

static void compose(int64_t scanout) {
    CAMetalLayer *layer = g_layer;
    if (!layer || !g_pipeline) return;
    pthread_mutex_lock(&g_lock);
    int s = g_latest;
    quat rendered = QID;
    id<MTLTexture> left = nil, right = nil, inset_left = nil, inset_right = nil;
    float inset_fov[2][4] = {{0}};
    if (s >= 0) {
        g_slots[s].readers++;
        rendered = g_slots[s].q;
        left = g_slots[s].eye[0];
        right = g_slots[s].eye[1];
        if (g_slots[s].quad) {
            inset_left = g_slots[s].eye[2];
            inset_right = g_slots[s].eye[3];
            memcpy(inset_fov, g_slots[s].inset_fov, sizeof inset_fov);
        }
        if (g_slots[s].seq == g_last_composed_seq) g_stat_repeated++;
        g_last_composed_seq = g_slots[s].seq;
    }
    double refresh = g_refresh;
    pthread_mutex_unlock(&g_lock);

    // The head at the middle of this refresh's scan-out.
    float ahead = ((float)(scanout - axr_now_ns()) + g_cfg.lookahead_ms * 1e6f) / 1e9f + 0.5f / (float)refresh;
    quat now = rendered;
    bool turned = g_cfg.reprojection && s >= 0 && head_orientation(ahead, &now);
    bool cameras = update_camera_textures();
    // Lens angle, brightness and convergence: psvr2-player-mac's defaults, tuned there by eye
    compose_uniform u = {
        .calib_l = {g_calib[0], g_calib[1], g_calib[4], g_calib[5]},
        .calib_r = {g_calib[2], g_calib[3], g_calib[6], g_calib[7]},
        .p0 = {s >= 0 ? 1 : 0, g_cfg.chromatic ? 1 : 0, inset_left && inset_right ? 1 : 0, 0},
        .p1 = {cameras ? 1 : 0, 150.0f * (float)M_PI / 180.0f, 1.6f, 0.100f},
    };
    qmatrix(turned ? qnorm(qmul(qconj(rendered), now)) : QID, u.delta);
    for (int k = 0; k < 4; k++) {
        u.inset_l[k] = tanf(inset_fov[0][k]);
        u.inset_r[k] = tanf(inset_fov[1][k]);
    }
    float gaze[3];
    if (g_cfg.show_gaze && gaze_in_head(gaze)) {
        memcpy(u.gaze, gaze, sizeof gaze);
        u.p0[3] = 1;
    }
    pthread_mutex_lock(&g_eye_lock);
    if (g_cal.mode) {
        memcpy(u.cal, g_cal.target, sizeof g_cal.target);
        u.cal[3] = g_cal.mode;
        u.cal2[1] = (axr_now_ns() - g_cal.shown_ns) / 1e9f;
    }
    pthread_mutex_unlock(&g_eye_lock);
    if (u.cal[3] > 0) {
        int prox = 0, ipd_mm = 63;
        if (g_usb) psvr2_get_status(&prox, &ipd_mm);
        u.cal2[0] = (ipd_mm < 55 || ipd_mm > 75 ? 63 : ipd_mm) / 2000.0f;
    }

    id<CAMetalDrawable> drawable = [layer nextDrawable];
    id<MTLCommandBuffer> cmd = drawable ? [g_queue commandBuffer] : nil;
    if (!cmd) {
        pthread_mutex_lock(&g_lock);
        if (s >= 0) g_slots[s].readers--;
        g_stat_no_drawable++;
        pthread_mutex_unlock(&g_lock);
        return;
    }
    MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
    pass.colorAttachments[0].texture = drawable.texture;
    pass.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    pass.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];
    [enc setRenderPipelineState:g_pipeline];
    [enc setFragmentBytes:&u length:sizeof u atIndex:0];
    [enc setFragmentBuffer:g_lut offset:0 atIndex:1];
    [enc setFragmentTexture:left ?: g_blank atIndex:0];
    [enc setFragmentTexture:right ?: g_blank atIndex:1];
    [enc setFragmentTexture:cameras ? g_cam[0] : g_blank atIndex:2];
    [enc setFragmentTexture:cameras ? g_cam[1] : g_blank atIndex:3];
    [enc setFragmentTexture:inset_left ?: g_blank atIndex:4];
    [enc setFragmentTexture:inset_right ?: g_blank atIndex:5];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc endEncoding];
    [cmd presentDrawable:drawable];
    [cmd addCompletedHandler:^(id<MTLCommandBuffer> _) {
        pthread_mutex_lock(&g_lock);
        if (s >= 0) g_slots[s].readers--;
        g_stat_composed++;
        pthread_mutex_unlock(&g_lock);
    }];
    [cmd commit];
}

static CVReturn link_callback(CVDisplayLinkRef link, const CVTimeStamp *now, const CVTimeStamp *out,
                              CVOptionFlags in_flags, CVOptionFlags *out_flags, void *ctx) {
    (void)now; (void)in_flags; (void)out_flags; (void)ctx;
    int64_t scanout = host_to_xr(out->hostTime);
    double period = CVDisplayLinkGetActualOutputVideoRefreshPeriod(link);
    pthread_mutex_lock(&g_lock);
    g_vsync_ns = scanout;
    if (period > 0.004 && period < 0.02) {
        g_period_ns = (int64_t)(period * 1e9);
        g_refresh = 1 / period;
    }
    pthread_mutex_unlock(&g_lock);
    @autoreleasepool { compose(scanout); }
    return kCVReturnSuccess;
}

// ------------------------------------------------------------------------------------- display

// The PS VR2 among the displays: Sony's EDID vendor id, or a 4000-pixel-wide mode.
static CGDirectDisplayID find_headset(bool *mirrored) {
    CGDirectDisplayID ids[16];
    uint32_t n = 0;
    *mirrored = false;
    if (CGGetOnlineDisplayList(16, ids, &n) != kCGErrorSuccess) return 0;
    for (uint32_t i = 0; i < n; i++) {
        bool sony = CGDisplayVendorNumber(ids[i]) == 0x4DD9;
        if (!sony) {
            NSArray *modes = (__bridge_transfer NSArray *)CGDisplayCopyAllDisplayModes(ids[i], NULL);
            for (id m in modes)
                if (CGDisplayModeGetPixelWidth((__bridge CGDisplayModeRef)m) == 4000) sony = true;
        }
        if (!sony) continue;
        if (CGDisplayMirrorsDisplay(ids[i]) != kCGNullDirectDisplay) { *mirrored = true; continue; }
        return ids[i];
    }
    return 0;
}

// A window that never takes the keyboard or the mouse, which stay with the game.
@interface PSVR2Window : NSWindow
@end
@implementation PSVR2Window
- (BOOL)canBecomeKeyWindow { return NO; }
- (BOOL)canBecomeMainWindow { return NO; }
@end

static void make_window(void) {
    NSScreen *screen = nil;
    for (NSScreen *sc in NSScreen.screens)
        if ([sc.deviceDescription[@"NSScreenNumber"] unsignedIntValue] == g_display) screen = sc;
    if (!screen) { axr_log("the headset display has no NSScreen; is it set to extend the desktop?"); return; }
    PSVR2Window *w = [[PSVR2Window alloc] initWithContentRect:screen.frame styleMask:NSWindowStyleMaskBorderless
                                                      backing:NSBackingStoreBuffered defer:NO screen:screen];
    w.releasedWhenClosed = NO;
    w.level = NSMainMenuWindowLevel + 1;
    w.ignoresMouseEvents = YES;
    w.hasShadow = NO;
    w.opaque = YES;
    w.backgroundColor = NSColor.blackColor;
    w.collectionBehavior = NSWindowCollectionBehaviorCanJoinAllSpaces | NSWindowCollectionBehaviorStationary |
                           NSWindowCollectionBehaviorIgnoresCycle | NSWindowCollectionBehaviorFullScreenAuxiliary;
    [w setFrame:screen.frame display:YES];
    CAMetalLayer *layer = [CAMetalLayer layer];
    layer.device = axr_metal_device();
    layer.pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
    layer.framebufferOnly = YES;
    layer.displaySyncEnabled = YES;
    layer.maximumDrawableCount = 3;
    layer.contentsScale = screen.backingScaleFactor;
    // The panel's own resolution, whatever the desktop is scaled to.
    layer.drawableSize = CGSizeMake(4000, 2040);
    NSView *view = [[NSView alloc] initWithFrame:(NSRect){NSZeroPoint, screen.frame.size}];
    view.layer = layer;
    view.wantsLayer = YES;
    w.contentView = view;
    [w orderFrontRegardless];
    g_window = w;
    g_layer = layer;
    NSSize px = [screen convertRectToBacking:screen.frame].size;
    // A headset that offers no panel-sized mode is in its flat cinema mode: it shows the whole
    // 1080p frame, both eyes' halves, on one virtual screen, and no setting in Displays fixes it.
    bool vr_modes = false;
    NSArray *modes = (__bridge_transfer NSArray *)CGDisplayCopyAllDisplayModes(g_display, NULL);
    for (id m in modes)
        if (CGDisplayModeGetPixelWidth((__bridge CGDisplayModeRef)m) >= 2000) vr_modes = true;
    axr_log("headset window up: desktop %.0fx%.0f (%.0fx%.0f px)%s", screen.frame.size.width,
            screen.frame.size.height, px.width, px.height,
            !vr_modes ? "; the headset offers only 1080p modes, so it is in cinema mode and both eyes see "
                        "one flat screen: power-cycle the PC adapter, or try a 4-lane DisplayPort 1.4 cable"
            : px.width == 4000 || px.width == 2000 ? "" : "; set PS VR2 to 2000x1020 or 4000x2040 in Displays for a sharper image");
}

// The window has to be made on the main thread. Under Wine that thread runs the Cocoa event loop
// for the Mac driver; a plain command-line app with no event loop never gets there.
static void on_main(void (^block)(void)) {
    if (NSThread.isMainThread) { block(); return; }
    dispatch_semaphore_t done = dispatch_semaphore_create(0);
    dispatch_async(dispatch_get_main_queue(), ^{
        block();
        dispatch_semaphore_signal(done);
    });
    if (dispatch_semaphore_wait(done, dispatch_time(DISPATCH_TIME_NOW, 3 * NSEC_PER_SEC)) != 0)
        axr_log("the main thread has not made the headset window after 3 s; it will appear when it does");
}

// -------------------------------------------------------------------------------------- status

// The room through the headset's cameras in place of the game, or back to the game. The game keeps
// running underneath.
static void toggle_passthrough(void) {
    if (g_passthrough) {
        g_passthrough = false;
        psvr2_camera_stop();
        axr_log("passthrough off, back to the game");
    } else if (psvr2_camera_start() == 0) {
        g_passthrough = true;
        axr_log("passthrough on: the headset's cameras (double press the headset button again to go back)");
    } else {
        axr_log("passthrough: the cameras did not start");
    }
}

static void *status_thread(void *arg) {
    (void)arg;
    bool was_down = false, recentered = false, ps_down[2] = {false, false}, cal_asked = false, hold_asked = false;
    int64_t connected_since = 0, last_stats = axr_now_ns(), last_press = 0, down_since = 0;
    for (;;) {
        struct timespec d = {0, 50000000};
        nanosleep(&d, NULL);
        int64_t now = axr_now_ns();
        if (g_usb && psvr2_connected()) {
            if (!connected_since) connected_since = now;
            // Let the fusion settle for half a second, then face forward once, before the
            // runtime takes LOCAL from the head.
            if (!recentered && now - connected_since > 500000000) {
                recenter();
                recentered = true;
                pthread_mutex_lock(&g_lock);
                g_tracking_ready = true;
                pthread_mutex_unlock(&g_lock);
            }
            // A press recenters; a second within 0.45 s also turns the cameras on or off, and
            // holding it for two seconds calibrates the eye tracker, and for five calls
            // axr_on_button_hold (the self-test quits).
            bool down = psvr2_get_button() == 1;
            if (down && !was_down) {
                recenter();
                if (now - last_press < 450000000) {
                    toggle_passthrough();
                    last_press = 0;
                } else {
                    last_press = now;
                }
                down_since = now;
                cal_asked = false;
                hold_asked = false;
            }
            if (down && !cal_asked && now - down_since > 2000000000LL) {
                cal_asked = true;
                axr_eye_calibrate();
            }
            if (down && !hold_asked && now - down_since > 5000000000LL && axr_on_button_hold) {
                hold_asked = true;
                axr_on_button_hold();
            }
            was_down = down;
        }
        // A controller that has just connected, or whose PS button was pressed, points where the
        // head faces.
        for (int h = 0; h < 2 && g_cfg.controllers; h++) {
            sense_state s;
            bool on = sense_get(h, &s) && s.updated_ns;
            quat head = QID;
            head_orientation(0, &head);
            pthread_mutex_lock(&g_lock);
            bool aligned = g_ctl_aligned[h];
            if (!on) g_ctl_aligned[h] = false;
            pthread_mutex_unlock(&g_lock);
            bool ps = on && (s.buttons & SENSE_PS);
            if (on && (!aligned || (ps && !ps_down[h]))) {
                // The cameras keep a controller they see pointing the right way by themselves.
                if (aligned && optically_tracked(h, 1000000000)) {
                    axr_log("%s controller: PS pressed, but the cameras see it, so it keeps its direction",
                            h == SENSE_LEFT ? "left" : "right");
                } else {
                    align_controller(h, yaw_of(head));
                    if (aligned) axr_log("%s controller recentered", h == SENSE_LEFT ? "left" : "right");
                }
            }
            ps_down[h] = ps;
        }
        if (now - last_stats > 5000000000LL) {
            double dt = (now - last_stats) / 1e9;
            last_stats = now;
            pthread_mutex_lock(&g_lock);
            unsigned submits = g_stat_submits, dropped = g_stat_dropped, composed = g_stat_composed,
                     repeated = g_stat_repeated, no_drawable = g_stat_no_drawable;
            g_stat_submits = g_stat_dropped = g_stat_composed = g_stat_repeated = g_stat_no_drawable = 0;
            double refresh = g_refresh;
            pthread_mutex_unlock(&g_lock);
            int prox = 0, ipd = 0;
            float bias = 0, corr = 0, q[4] = {0}, p[3] = {0};
            if (g_usb) {
                psvr2_get_status(&prox, &ipd);
                psvr2_get_fusion_status(&bias, &corr);
                psvr2_get_pose(q, p);
            }
            if (submits || composed)
                axr_log("game %.0f fps (%u dropped) | panel %.0f Hz: %.0f composed/s, %u repeats, %u without a drawable"
                        " | app: %u waits, %u ends, %u projection layers | head: %s, worn %d, IPD %d mm,"
                        " gyro bias %.1f deg/s, SLAM correction %.1f deg, raw SLAM position (%.3f, %.3f, %.3f)",
                        submits / dt, dropped, refresh, composed / dt, repeated, no_drawable, axr_stat_wait,
                        axr_stat_end, axr_stat_projection, axr_backend_connected() ? "tracking" : "NOT tracking",
                        prox, ipd, bias, corr, p[0], p[1], p[2]);
            for (int h = 0; h < 2 && g_cfg.controllers; h++) {
                sense_state s;
                if (!sense_get(h, &s)) continue;
                float e[3] = {0}, f[3] = {0, 0, -1};
                qrot(raw_grip(&s), f, e);
                axr_log("%s controller: %s, battery %.0f%%%s, buttons %03x, stick (%.2f, %.2f), trigger %.2f,"
                        " pointing (%.2f, %.2f, %.2f) before recentering, %s",
                        h == SENSE_LEFT ? "left" : "right", s.bluetooth ? "Bluetooth" : "USB", s.battery * 100,
                        s.charging ? " charging" : "", s.buttons, s.stick[0], s.stick[1], s.trigger, e[0], e[1], e[2],
                        s.updated_ns && axr_now_ns() - (int64_t)s.updated_ns < 1000000000 ? "reporting" : "NOT reporting");
            }
            if (g_tracker) {
                pthread_mutex_lock(&g_lock);
                unsigned frames = g_stat_track_frames, dots[STRACK_CAMERAS],
                         seen[2] = {g_stat_track_seen[0], g_stat_track_seen[1]},
                         leds[2] = {g_stat_track_leds[0], g_stat_track_leds[1]},
                         left_out[2] = {g_stat_track_left_out[0], g_stat_track_left_out[1]};
                float rms[2] = {g_stat_track_rms[0], g_stat_track_rms[1]};
                memcpy(dots, g_stat_track_dots, sizeof dots);
                g_stat_track_frames = 0;
                memset(g_stat_track_dots, 0, sizeof g_stat_track_dots);
                memset(g_stat_track_seen, 0, sizeof g_stat_track_seen);
                memset(g_stat_track_left_out, 0, sizeof g_stat_track_left_out);
                memset(g_stat_track_leds, 0, sizeof g_stat_track_leds);
                memset(g_stat_track_rms, 0, sizeof g_stat_track_rms);
                pthread_mutex_unlock(&g_lock);
                float cal_rms;
                int cal = strack_calibration(g_tracker, &cal_rms);
                char side[2][128];
                for (int h = 0; h < 2; h++) {
                    if (seen[h])
                        snprintf(side[h], sizeof side[h], "seen in %.0f%% of frames (%.1f LEDs, %.2f px; %u fits left out)",
                                 frames ? 100.0 * seen[h] / frames : 0, (double)leds[h] / seen[h], rms[h] / seen[h],
                                 left_out[h]);
                    else
                        snprintf(side[h], sizeof side[h], "not seen%s",
                                 sense_leds_on(h) ? "" : " (its LEDs are off: is it connected?)");
                }
                double fr = frames ? frames : 1;
                pthread_mutex_lock(&g_track_lock);
                bool spots = g_track_is_spots;
                double ahead = g_stat_ld_ahead_n ? g_stat_ld_ahead_s / g_stat_ld_ahead_n : 0;
                g_stat_ld_ahead_s = 0;
                g_stat_ld_ahead_n = 0;
                pthread_mutex_unlock(&g_track_lock);
                axr_log("controller tracking: %.0f camera frames/s, %.1f/%.1f/%.1f/%.1f dots a frame (lower left/right,"
                        " upper left/right camera, %s; the spot finder's come %.1f ms before the pictures); left %s;"
                        " right %s; cameras calibrated %d times%s",
                        frames / dt, dots[0] / fr, dots[1] / fr, dots[2] / fr, dots[3] / fr,
                        spots ? "from the headset's spot finder" : "found in the pictures", ahead * 1e3, side[0], side[1],
                        cal, cal ? [NSString stringWithFormat:@" (fit %.2f px)", cal_rms].UTF8String : "");
            }
            if (g_eyes_on) {
                psvr2_gaze g;
                unsigned packets = psvr2_gaze_packets();
                float d[3] = {0};
                bool looking = gaze_in_head(d);
                if (psvr2_get_gaze(&g) == 1)
                    axr_log("eyes: %.0f samples/s, %s, looking (%.2f, %.2f, %.2f), pupils %.1f/%.1f mm, blink %d/%d,"
                            " user calibration %u%s",
                            packets / dt, looking ? "tracked" : "NOT tracked", d[0], d[1], d[2], g.pupil_mm[0],
                            g.pupil_mm[1], g.blink[0], g.blink[1], g.calibration_id,
                            g.calibration_id ? "" : " (none: hold Fn for 2 s to calibrate)");
                else
                    axr_log("eyes: no samples from the eye tracker yet");
            }
            axr_stat_wait = axr_stat_end = axr_stat_projection = 0;
            axr_stat_syncs = axr_stat_unfocused_syncs = axr_stat_pressed = 0;
        }
    }
    return NULL;
}

// ---------------------------------------------------------------------------------------- start

void axr_backend_start(void) {
    pthread_mutex_lock(&g_lock);
    if (g_started) { pthread_mutex_unlock(&g_lock); return; }
    g_started = true;
    pthread_mutex_unlock(&g_lock);

    mach_timebase_info(&g_timebase);
    NSString *env = NSProcessInfo.processInfo.environment[@"PSVR2XR_CONFIG_DIR"];
    g_config_dir = env ?: [NSHomeDirectory() stringByAppendingPathComponent:@"Library/Application Support/psvr2xr"];
    read_config();
    if (getenv("PSVR2XR_SHOW_GAZE")) g_cfg.show_gaze = true;   // the self-test's eye checks

    if (psvr2_start() == 0) {
        g_usb = true;
        psvr2_get_distortion_calibration(g_calib);
        psvr2_set_brightness(1.0f);
        axr_log("PS VR2 on USB; distortion calibration %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f", g_calib[0],
                g_calib[1], g_calib[2], g_calib[3], g_calib[4], g_calib[5], g_calib[6], g_calib[7]);
        if (g_cfg.eye_tracking) {
            g_eye_calibration_path = [g_config_dir stringByAppendingPathComponent:@"eye_calibration.bin"];
            bool saved = [NSFileManager.defaultManager fileExistsAtPath:g_eye_calibration_path];
            g_eyes_on = psvr2_gaze_start(g_eye_calibration_path.fileSystemRepresentation) == 0;
            axr_log(g_eyes_on ? "eye tracker on%s" : "eye tracker not available%s",
                    !g_eyes_on ? "" : saved ? ", with your saved calibration" : "; hold Fn for 2 s to calibrate it");
        }
    } else {
        axr_log("PS VR2 not found on USB (is the adapter's USB cable in, and no other app using the headset?);"
                " showing frames without head tracking");
        float fallback[8] = {-0.09919293f, 0, 0.09919293f, 0, 1, 0, 1, 0};
        memcpy(g_calib, fallback, sizeof g_calib);
    }
    // An old calibration version leaves the per-eye rotation out: no rotation.
    if (g_calib[4] == 0 && g_calib[6] == 0) g_calib[4] = g_calib[6] = 1;

    bool mirrored;
    g_display = find_headset(&mirrored);
    if (!g_display) {
        axr_log(mirrored ? "the PS VR2 display mirrors another one; set it to extend the desktop in Displays"
                         : "no PS VR2 display; is the DisplayPort cable in and the headset on?");
    } else {
        CGDisplayModeRef mode = CGDisplayCopyDisplayMode(g_display);
        double hz = mode ? CGDisplayModeGetRefreshRate(mode) : 0;
        if (mode) CGDisplayModeRelease(mode);
        if (hz > 0) {
            g_refresh = hz;
            g_period_ns = (int64_t)(1e9 / hz);
        }
        axr_log("PS VR2 display %u at %.0f Hz", g_display, g_refresh);
    }

    id<MTLDevice> dev = axr_metal_device();
    g_queue = [dev newCommandQueue];
    NSError *err = nil;
    id<MTLLibrary> lib = [dev newLibraryWithSource:@(psvr2_compose_metal) options:nil error:&err];
    if (!lib) axr_log("compositor shader: %s", err.localizedDescription.UTF8String);
    MTLRenderPipelineDescriptor *pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [lib newFunctionWithName:@"vs"];
    pd.fragmentFunction = [lib newFunctionWithName:@"fs"];
    pd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
    g_pipeline = [dev newRenderPipelineStateWithDescriptor:pd error:&err];
    if (!g_pipeline) axr_log("compositor pipeline: %s", err.localizedDescription.UTF8String);
    g_lut = [dev newBufferWithBytes:psvr2_distortion_lut() length:1024 * 3 * sizeof(float)
                            options:MTLResourceStorageModeShared];
    g_blank = [dev newTextureWithDescriptor:[MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                            width:1 height:1 mipmapped:NO]];

    if (g_display) {
        on_main(^{ make_window(); });
        if (CVDisplayLinkCreateWithCGDisplay(g_display, &g_link) == kCVReturnSuccess) {
            CVDisplayLinkSetOutputCallback(g_link, link_callback, NULL);
            CVDisplayLinkStart(g_link);
        } else {
            axr_log("no display link for the headset display; nothing will be shown");
        }
    }

    if (g_cfg.controllers) sense_start();
    if (g_cfg.controllers && g_cfg.controller_tracking && g_usb) start_controller_tracking();

    pthread_t t;
    pthread_create(&t, NULL, status_thread, NULL);
    pthread_detach(t);
    axr_log("started (config %s)", g_config_dir.UTF8String);
}
