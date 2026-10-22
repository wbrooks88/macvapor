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
// position; there are no controllers yet.

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
} config;

static NSString *g_config_dir;
static config g_cfg = {.render_scale = 0.75f, .eye_height = 1.6f, .chromatic = true, .reprojection = true};

static void read_config(void) {
    NSString *path = [g_config_dir stringByAppendingPathComponent:@"config.json"];
    NSFileManager *fm = NSFileManager.defaultManager;
    [fm createDirectoryAtPath:g_config_dir withIntermediateDirectories:YES attributes:nil error:nil];
    if (![fm fileExistsAtPath:path]) {
        NSDictionary *d = @{
            @"render_scale" : @(g_cfg.render_scale), @"eye_height" : @(g_cfg.eye_height),
            @"lookahead_ms" : @(g_cfg.lookahead_ms), @"chromatic" : @(g_cfg.chromatic),
            @"reprojection" : @(g_cfg.reprojection), @"positional" : @(g_cfg.positional),
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
    axr_log("config: render scale %.2f, eye height %.2f m, lookahead %+.1f ms, chromatic %s, reprojection %s, %s",
            g_cfg.render_scale, g_cfg.eye_height, g_cfg.lookahead_ms, g_cfg.chromatic ? "on" : "off",
            g_cfg.reprojection ? "on" : "off", g_cfg.positional ? "SLAM position (experimental)" : "neck model");
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
    id<MTLTexture> eye[2];
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
static CAMetalLayer *g_layer;
static CVDisplayLinkRef g_link;
static id<MTLDevice> g_device;
static id<MTLCommandQueue> g_queue;
static id<MTLRenderPipelineState> g_pipeline;
static id<MTLBuffer> g_lut;
static id<MTLTexture> g_blank;     // bound before the first game frame, which the shader then skips

static unsigned g_stat_submits, g_stat_dropped, g_stat_composed, g_stat_repeated, g_stat_no_drawable;
static uint64_t g_last_composed_seq;

// compose.metal's PS VR2 counterpart, embedded by build-psvr2.sh as a C string.
#include "psvr2_compose_metal.h"

typedef struct {
    float calib_l[4], calib_r[4];
    float delta[16];
    float p0[4];
} compose_uniform;
_Static_assert(sizeof(compose_uniform) == 112, "Uni layout");

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

static void recenter(void) {
    quat o;
    float q[4];
    if (!g_usb || psvr2_get_predicted_quat(0, q) != 1) return;
    o = qnorm(qmul(k_upright, (quat){q[1], q[2], q[3], q[0]}));
    float fwd[3], f[3] = {0, 0, -1};
    qrot(o, f, fwd);
    float yaw = atan2f(-fwd[0], -fwd[2]);
    pthread_mutex_lock(&g_lock);
    g_recenter = qyaw(-yaw);
    pthread_mutex_unlock(&g_lock);
    float pos[3];
    if (slam_position(pos)) {
        pthread_mutex_lock(&g_lock);
        memcpy(g_origin, pos, sizeof pos);
        pthread_mutex_unlock(&g_lock);
    }
    axr_log("recentered (turned %.0f deg)", yaw * 180 / M_PI);
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

axr_motion axr_device_motion(int device, int64_t time) {
    axr_motion m = {0};
    if (device != AXR_HEAD) return m;   // no controllers yet
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
    const float FX = 0.3585564f, FY = 0.3762281f, cx = 0.6603788f;
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

// Stage 1 has no controllers; games are played with a gamepad, which Wine passes through itself.
bool axr_input(const char *path, float *value) { (void)path; (void)value; return false; }
uint64_t axr_input_generation(void) { return 0; }
void axr_haptics(int hand, float duration_s, float frequency, float amplitude) {
    (void)hand; (void)duration_s; (void)frequency; (void)amplitude;
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

void axr_submit(int64_t display_time, const axr_eye_image eyes[2], id<MTLCommandQueue> queue) {
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
    for (int e = 0; e < 2; e++) {
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
            axr_log("eye %d images: %dx%d, format %lu", e, eyes[e].width, eyes[e].height, (unsigned long)src.pixelFormat);
        }
    }
    id<MTLCommandBuffer> cmd = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmd blitCommandEncoder];
    for (int e = 0; e < 2; e++)
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

// ---------------------------------------------------------------------------------- compositor

static void compose(int64_t scanout) {
    CAMetalLayer *layer = g_layer;
    if (!layer || !g_pipeline) return;
    pthread_mutex_lock(&g_lock);
    int s = g_latest;
    quat rendered = QID;
    id<MTLTexture> left = nil, right = nil;
    if (s >= 0) {
        g_slots[s].readers++;
        rendered = g_slots[s].q;
        left = g_slots[s].eye[0];
        right = g_slots[s].eye[1];
        if (g_slots[s].seq == g_last_composed_seq) g_stat_repeated++;
        g_last_composed_seq = g_slots[s].seq;
    }
    double refresh = g_refresh;
    pthread_mutex_unlock(&g_lock);

    // The head at the middle of this refresh's scan-out.
    float ahead = ((float)(scanout - axr_now_ns()) + g_cfg.lookahead_ms * 1e6f) / 1e9f + 0.5f / (float)refresh;
    quat now = rendered;
    bool turned = g_cfg.reprojection && s >= 0 && head_orientation(ahead, &now);
    compose_uniform u = {
        .calib_l = {g_calib[0], g_calib[1], g_calib[4], g_calib[5]},
        .calib_r = {g_calib[2], g_calib[3], g_calib[6], g_calib[7]},
        .p0 = {s >= 0 ? 1 : 0, g_cfg.chromatic ? 1 : 0, 0, 0},
    };
    qmatrix(turned ? qnorm(qmul(qconj(rendered), now)) : QID, u.delta);

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
    axr_log("headset window up: desktop %.0fx%.0f (%.0fx%.0f px)%s", screen.frame.size.width,
            screen.frame.size.height, px.width, px.height,
            px.width == 4000 || px.width == 2000 ? "" : "; set PS VR2 to 2000x1020 or 4000x2040 in Displays for a sharper image");
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

static void *status_thread(void *arg) {
    (void)arg;
    bool was_down = false, recentered = false;
    int64_t connected_since = 0, last_stats = axr_now_ns();
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
            bool down = psvr2_get_button() == 1;
            if (down && !was_down) recenter();
            was_down = down;
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

    if (psvr2_start() == 0) {
        g_usb = true;
        psvr2_get_distortion_calibration(g_calib);
        psvr2_set_brightness(1.0f);
        axr_log("PS VR2 on USB; distortion calibration %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f", g_calib[0],
                g_calib[1], g_calib[2], g_calib[3], g_calib[4], g_calib[5], g_calib[6], g_calib[7]);
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

    pthread_t t;
    pthread_create(&t, NULL, status_thread, NULL);
    pthread_detach(t);
    axr_log("started (config %s)", g_config_dir.UTF8String);
}
