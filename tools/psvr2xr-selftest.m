// psvr2xr-selftest: the PS VR2 backend on its own, with no OpenXR, Wine or game. It renders a
// test room through the same calls runtime.m makes (axr_wait_frame, axr_device_motion,
// axr_current_views, axr_submit), so what it shows checks the display, the lens correction, head
// tracking and reprojection before any game is involved.
//
//   out/psvr2xr-selftest            render every frame
//   out/psvr2xr-selftest --slow N   render only every Nth frame, as a slow game would; with
//                                   reprojection on, turning your head should still look smooth
//   out/psvr2xr-selftest --shaders-only   compile the test room's shader and exit (for CI)
//   out/psvr2xr-selftest --calibrate-eyes calibrate the eye tracker first (13 dots), then the room
//   out/psvr2xr-selftest --foveated       render quad views as a foveated game would: a low-
//                                   resolution room with a sharp inset, tinted orange, that
//                                   follows your eyes; a yellow ring shows where you look
//   out/psvr2xr-selftest --capture-controllers   the room, while saving what controller tracking
//                                   sees (camera frames, dots, poses) to ~/Desktop/PS VR2 Controller
//                                   Capture, to send for a look when the controllers aren't found
//
// The room: a floor grid with 1 m squares, a horizon, and four pillars 3 m away: red ahead,
// green to the right, blue behind and yellow to the left. Straight grid lines and a level horizon
// mean the lens correction is right; pillars that stay put when you turn mean tracking is.
//
// Sense controllers, when connected over Bluetooth, show as a ball at each hand (cyan left,
// magenta right) with a smaller one 20 cm along where the controller points; the trigger lights the
// ball up and vibrates the controller. Where the headset's cameras see them, the balls follow your
// hands; elsewhere an arm model places them. Each points where your head faces when it connects
// until the cameras find it; press its PS button to point it forward again.
#import <AppKit/AppKit.h>
#import <Metal/Metal.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alvrxr.h"

static const char *scene_source =
    "#include <metal_stdlib>\n"
    "using namespace metal;\n"
    "struct VOut { float4 pos [[position]]; float2 uv; };\n"
    "vertex VOut vs(uint vid [[vertex_id]]) {\n"
    "    float2 p[3] = { float2(-1.0, -1.0), float2(3.0, -1.0), float2(-1.0, 3.0) };\n"
    "    VOut o; o.pos = float4(p[vid], 0.0, 1.0);\n"
    "    o.uv = float2(p[vid].x * 0.5 + 0.5, 1.0 - (p[vid].y * 0.5 + 0.5));\n"
    "    return o;\n"
    "}\n"
    "struct Eye { float4x4 rot; float4 origin; float4 tans; float4 hand[2]; float4 tip[2]; };\n"   // tans: tan of left, right, up, down; hand/tip w: shown, trigger; origin w: 2 tints an inset
    "static float pillar(float3 o, float3 d, float2 c, float r) {\n"   // distance along d to a vertical cylinder
    "    float2 oc = o.xz - c; float2 dd = d.xz;\n"
    "    float a = dot(dd, dd), b = dot(oc, dd), k = dot(oc, oc) - r * r;\n"
    "    float h = b * b - a * k;\n"
    "    if (a < 1e-6 || h < 0.0) return -1.0;\n"
    "    float t = (-b - sqrt(h)) / a;\n"
    "    float y = o.y + t * d.y;\n"
    "    return (t > 0.0 && y > 0.0 && y < 2.5) ? t : -1.0;\n"
    "}\n"
    "static float ball(float3 o, float3 d, float3 c, float r) {\n"   // distance along d to a sphere
    "    float3 oc = o - c; float b = dot(oc, d), k = dot(oc, oc) - r * r, h = b * b - k;\n"
    "    if (h < 0.0) return -1.0;\n"
    "    float t = -b - sqrt(h);\n"
    "    return t > 0.0 ? t : -1.0;\n"
    "}\n"
    "fragment float4 fs(VOut in [[stage_in]], constant Eye &e [[buffer(0)]]) {\n"
    "    float tx = mix(e.tans.x, e.tans.y, in.uv.x);\n"
    "    float ty = mix(e.tans.z, e.tans.w, in.uv.y);\n"
    "    float3 d = normalize((e.rot * float4(tx, ty, -1.0, 0.0)).xyz);\n"
    "    float3 o = e.origin.xyz;\n"
    "    float best = 1e9; float3 col = mix(float3(0.55, 0.7, 0.9), float3(0.15, 0.3, 0.6), saturate(d.y * 2.0));\n"
    "    if (d.y < 0.0) {\n"
    "        float t = -o.y / d.y; float3 p = o + t * d;\n"
    "        float2 g = abs(fract(p.xz + 0.5) - 0.5);\n"
    "        float line = 1.0 - smoothstep(0.0, 0.02 + 0.002 * t, min(g.x, g.y));\n"
    "        float fade = exp(-t * 0.04);\n"
    "        col = mix(float3(0.35), mix(float3(0.25), float3(0.95), line), fade);\n"
    "        best = t;\n"
    "    }\n"
    "    float2 at[4] = { float2(0.0, -3.0), float2(3.0, 0.0), float2(0.0, 3.0), float2(-3.0, 0.0) };\n"
    "    float3 pc[4] = { float3(0.9, 0.15, 0.1), float3(0.1, 0.8, 0.2), float3(0.15, 0.3, 0.95), float3(0.95, 0.85, 0.1) };\n"
    "    for (int i = 0; i < 4; i++) {\n"
    "        float t = pillar(o, d, at[i], 0.25);\n"
    "        if (t > 0.0 && t < best) {\n"
    "            float3 p = o + t * d; float3 n = normalize(float3(p.x - at[i].x, 0.0, p.z - at[i].y));\n"
    "            float band = step(0.5, fract(p.y * 2.0));\n"
    "            col = pc[i] * (0.45 + 0.55 * saturate(dot(n, normalize(float3(0.4, 0.6, 0.7))))) * (0.8 + 0.2 * band);\n"
    "            best = t;\n"
    "        }\n"
    "    }\n"
    "    float3 hc[2] = { float3(0.1, 0.85, 0.95), float3(0.95, 0.2, 0.85) };\n"
    "    for (int i = 0; i < 2; i++) {\n"
    "        if (e.hand[i].w == 0.0) continue;\n"
    "        float glow = 0.35 + 0.65 * e.tip[i].w;\n"
    "        float t = ball(o, d, e.hand[i].xyz, 0.04);\n"
    "        if (t > 0.0 && t < best) { best = t; col = hc[i] * glow * (0.6 + 0.4 * saturate(dot(normalize(o + t * d - e.hand[i].xyz), float3(0.4, 0.6, 0.7)))); }\n"
    "        t = ball(o, d, e.tip[i].xyz, 0.015);\n"
    "        if (t > 0.0 && t < best) { best = t; col = float3(1.0); }\n"
    "    }\n"
    "    if (e.origin.w > 1.5) col = mix(col, float3(1.0, 0.55, 0.1), 0.25);\n"
    "    return float4(col, 1.0);\n"
    "}\n";

typedef struct {
    float rot[16];
    float origin[4];
    float tans[4];
    float hand[2][4];   // position, shown
    float tip[2][4];    // 20 cm along the controller, trigger
} eye_uniform;

static int g_slow = 1;
static bool g_shaders_only, g_calibrate_eyes, g_foveated;
static id<MTLRenderPipelineState> g_scene;
static id<MTLCommandQueue> g_queue;

static void matrix_from(const float q[4], float m[16]) {   // x, y, z, w; column-major
    float x = q[0], y = q[1], z = q[2], w = q[3];
    float r[16] = {1 - 2 * (y * y + z * z), 2 * (x * y + w * z),     2 * (x * z - w * y),     0,
                   2 * (x * y - w * z),     1 - 2 * (x * x + z * z), 2 * (y * z + w * x),     0,
                   2 * (x * z + w * y),     2 * (y * z - w * x),     1 - 2 * (x * x + y * y), 0,
                   0,                       0,                       0,                       1};
    memcpy(m, r, sizeof r);
}

static void *render_thread(void *arg) {
    (void)arg;
    uint32_t size[4][2];   // per view: the eyes, and with --foveated the insets
    axr_recommended_eye_size(&size[0][0], &size[0][1]);
    int views = g_foveated ? 4 : 2;
    if (g_foveated) axr_foveated_sizes(size[0], size[2]);
    memcpy(size[1], size[0], sizeof size[0]);
    memcpy(size[3], size[2], sizeof size[2]);
    uint32_t w = size[0][0], h = size[0][1];
    id<MTLDevice> dev = axr_metal_device();
    id<MTLTexture> tex[3][4];   // a small swapchain, as the runtime gives a game
    for (int i = 0; i < 3; i++)
        for (int e = 0; e < views; e++) {
            MTLTextureDescriptor *d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                         width:size[e][0] height:size[e][1]
                                                                                     mipmapped:NO];
            d.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
            d.storageMode = MTLStorageModePrivate;
            tex[i][e] = [dev newTextureWithDescriptor:d];
        }
    printf("rendering %ux%u per eye%s\n", w, h, g_slow > 1 ? " (slowed down)" : "");
    if (g_foveated) printf("and a %ux%u inset per eye that follows your eyes\n", size[2][0], size[2][1]);
    unsigned frame = 0;
    for (;;) {
        @autoreleasepool {
            int64_t t = axr_wait_frame();
            for (int skip = 1; skip < g_slow; skip++) t = axr_wait_frame();
            axr_motion head = axr_device_motion(AXR_HEAD, t);
            axr_views v = axr_current_views();
            float inset[2][4];
            if (g_foveated) axr_foveated_inset(t, true, inset);
            float hands[2][4] = {{0}}, tips[2][4] = {{0}};
            for (int k = 0; k < 2; k++) {
                axr_motion m = axr_device_motion(k == 0 ? AXR_LEFT : AXR_RIGHT, t);
                if (!m.valid) continue;
                float trig = 0, rot[16], along[3];
                axr_input(k == 0 ? "/user/hand/left/input/trigger/value" : "/user/hand/right/input/trigger/value", &trig);
                if (trig > 0.1f) axr_haptics(k == 0 ? AXR_LEFT : AXR_RIGHT, 0.05f, 160, trig);
                matrix_from(m.pose.orientation, rot);
                for (int j = 0; j < 3; j++) along[j] = -rot[8 + j] * 0.2f;   // -Z column
                for (int j = 0; j < 3; j++) {
                    hands[k][j] = m.pose.position[j];
                    tips[k][j] = m.pose.position[j] + along[j];
                }
                hands[k][3] = 1;
                tips[k][3] = trig;
            }
            int i = frame++ % 3;
            id<MTLCommandBuffer> cmd = [g_queue commandBuffer];
            for (int e = 0; e < views; e++) {
                eye_uniform u;
                matrix_from(head.pose.orientation, u.rot);
                // The eye's offset in the head, turned with the head.
                const float *off = v.eye_in_head[e % 2].position;
                for (int k = 0; k < 3; k++)
                    u.origin[k] = head.pose.position[k] + u.rot[k] * off[0] + u.rot[4 + k] * off[1] + u.rot[8 + k] * off[2];
                u.origin[3] = e < 2 ? 1 : 2;
                for (int k = 0; k < 4; k++) u.tans[k] = tanf(e < 2 ? v.fov[e][k] : inset[e - 2][k]);
                memcpy(u.hand, hands, sizeof hands);
                memcpy(u.tip, tips, sizeof tips);
                MTLRenderPassDescriptor *pass = [MTLRenderPassDescriptor renderPassDescriptor];
                pass.colorAttachments[0].texture = tex[i][e];
                pass.colorAttachments[0].loadAction = MTLLoadActionDontCare;
                pass.colorAttachments[0].storeAction = MTLStoreActionStore;
                id<MTLRenderCommandEncoder> enc = [cmd renderCommandEncoderWithDescriptor:pass];
                [enc setRenderPipelineState:g_scene];
                [enc setFragmentBytes:&u length:sizeof u atIndex:0];
                [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
                [enc endEncoding];
            }
            [cmd commit];
            axr_eye_image eyes[4];
            for (int e = 0; e < views; e++)
                eyes[e] = (axr_eye_image){tex[i][e], 0, 0, 0, (int32_t)size[e][0], (int32_t)size[e][1]};
            if (g_foveated) axr_submit_quad(t, eyes, inset, g_queue);
            else axr_submit(t, eyes, g_queue);
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    @autoreleasepool {
        for (int i = 1; i < argc; i++)
            if (!strcmp(argv[i], "--slow") && i + 1 < argc) g_slow = atoi(argv[++i]) > 1 ? atoi(argv[i]) : 1;
            else if (!strcmp(argv[i], "--shaders-only")) g_shaders_only = true;
            else if (!strcmp(argv[i], "--calibrate-eyes")) g_calibrate_eyes = true;
            else if (!strcmp(argv[i], "--foveated")) g_foveated = true;
            else if (!strcmp(argv[i], "--capture-controllers")) {
                NSString *dir = [NSHomeDirectory() stringByAppendingPathComponent:@"Desktop/PS VR2 Controller Capture"];
                setenv("PSVR2XR_CAPTURE_DIR", dir.fileSystemRepresentation, 1);
                printf("Saving what controller tracking sees to %s\n", dir.fileSystemRepresentation);
            }
        if (g_shaders_only) {
            id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
            if (!dev) { puts("no Metal device; shader not checked"); return 0; }
            NSError *err = nil;
            id<MTLLibrary> lib = [dev newLibraryWithSource:@(scene_source) options:nil error:&err];
            if (!lib) { fprintf(stderr, "scene shader: %s\n", err.localizedDescription.UTF8String); return 1; }
            puts("scene shader compiles");
            return 0;
        }
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        if (g_foveated || g_calibrate_eyes) setenv("PSVR2XR_SHOW_GAZE", "1", 1);   // the gaze ring
        axr_backend_start();   // on the main thread, so the headset window is made right away

        id<MTLDevice> dev = axr_metal_device();
        g_queue = [dev newCommandQueue];
        NSError *err = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:@(scene_source) options:nil error:&err];
        if (!lib) { fprintf(stderr, "scene shader: %s\n", err.localizedDescription.UTF8String); return 1; }
        MTLRenderPipelineDescriptor *pd = [MTLRenderPipelineDescriptor new];
        pd.vertexFunction = [lib newFunctionWithName:@"vs"];
        pd.fragmentFunction = [lib newFunctionWithName:@"fs"];
        pd.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
        g_scene = [dev newRenderPipelineStateWithDescriptor:pd error:&err];
        if (!g_scene) { fprintf(stderr, "scene pipeline: %s\n", err.localizedDescription.UTF8String); return 1; }

        printf("Put the headset on. You should stand in a grey room with a floor grid and four pillars:\n"
               "red ahead, green right, blue behind, yellow left. Press the headset's Fn button to face\n"
               "the red pillar again. Sense controllers show as cyan (left) and magenta (right) balls; pull\n"
               "a trigger to light one and feel it buzz, press PS to point it forward again.\n"
               "Status every 5 s here and in ~/Library/Logs/psvr2xr.log.\n"
               "Quit with Ctrl-C.\n");
        if (g_calibrate_eyes)
            printf("Eye calibration starts as soon as the headset tracks: look at each dot until the next one\n"
                   "appears. Afterwards the room comes back with a yellow ring where you look.\n");
        if (g_foveated && !axr_eye_tracking())
            printf("The eye tracker is off or not found, so the sharp inset stays in the middle.\n");
        pthread_t t;
        pthread_create(&t, NULL, render_thread, NULL);
        pthread_detach(t);
        if (g_calibrate_eyes) {
            dispatch_async(dispatch_get_global_queue(QOS_CLASS_DEFAULT, 0), ^{
                for (int i = 0; i < 300 && !axr_backend_tracking(); i++) usleep(100000);
                axr_eye_calibrate();
            });
        }
        [NSApp run];
    }
    return 0;
}
