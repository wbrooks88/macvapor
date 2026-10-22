// psvr2xr-selftest: the PS VR2 backend on its own, with no OpenXR, Wine or game. It renders a
// test room through the same calls runtime.m makes (axr_wait_frame, axr_device_motion,
// axr_current_views, axr_submit), so what it shows checks the display, the lens correction, head
// tracking and reprojection before any game is involved.
//
//   out/psvr2xr-selftest            render every frame
//   out/psvr2xr-selftest --slow N   render only every Nth frame, as a slow game would; with
//                                   reprojection on, turning your head should still look smooth
//
// The room: a floor grid with 1 m squares, a horizon, and four pillars 3 m away: red ahead,
// green to the right, blue behind and yellow to the left. Straight grid lines and a level horizon
// mean the lens correction is right; pillars that stay put when you turn mean tracking is.
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
    "struct Eye { float4x4 rot; float4 origin; float4 tans; };\n"   // tans: tan of left, right, up, down
    "static float pillar(float3 o, float3 d, float2 c, float r) {\n"   // distance along d to a vertical cylinder
    "    float2 oc = o.xz - c; float2 dd = d.xz;\n"
    "    float a = dot(dd, dd), b = dot(oc, dd), k = dot(oc, oc) - r * r;\n"
    "    float h = b * b - a * k;\n"
    "    if (a < 1e-6 || h < 0.0) return -1.0;\n"
    "    float t = (-b - sqrt(h)) / a;\n"
    "    float y = o.y + t * d.y;\n"
    "    return (t > 0.0 && y > 0.0 && y < 2.5) ? t : -1.0;\n"
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
    "    return float4(col, 1.0);\n"
    "}\n";

typedef struct {
    float rot[16];
    float origin[4];
    float tans[4];
} eye_uniform;

static int g_slow = 1;
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
    uint32_t w, h;
    axr_recommended_eye_size(&w, &h);
    id<MTLDevice> dev = axr_metal_device();
    id<MTLTexture> tex[3][2];   // a small swapchain, as the runtime gives a game
    for (int i = 0; i < 3; i++)
        for (int e = 0; e < 2; e++) {
            MTLTextureDescriptor *d = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB
                                                                                         width:w height:h mipmapped:NO];
            d.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
            d.storageMode = MTLStorageModePrivate;
            tex[i][e] = [dev newTextureWithDescriptor:d];
        }
    printf("rendering %ux%u per eye%s\n", w, h, g_slow > 1 ? " (slowed down)" : "");
    unsigned frame = 0;
    for (;;) {
        @autoreleasepool {
            int64_t t = axr_wait_frame();
            for (int skip = 1; skip < g_slow; skip++) t = axr_wait_frame();
            axr_motion head = axr_device_motion(AXR_HEAD, t);
            axr_views v = axr_current_views();
            int i = frame++ % 3;
            id<MTLCommandBuffer> cmd = [g_queue commandBuffer];
            for (int e = 0; e < 2; e++) {
                eye_uniform u;
                matrix_from(head.pose.orientation, u.rot);
                // The eye's offset in the head, turned with the head.
                const float *off = v.eye_in_head[e].position;
                for (int k = 0; k < 3; k++)
                    u.origin[k] = head.pose.position[k] + u.rot[k] * off[0] + u.rot[4 + k] * off[1] + u.rot[8 + k] * off[2];
                u.origin[3] = 1;
                for (int k = 0; k < 4; k++) u.tans[k] = tanf(v.fov[e][k]);
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
            axr_eye_image eyes[2] = {{tex[i][0], 0, 0, 0, (int32_t)w, (int32_t)h}, {tex[i][1], 0, 0, 0, (int32_t)w, (int32_t)h}};
            axr_submit(t, eyes, g_queue);
        }
    }
    return NULL;
}

int main(int argc, char **argv) {
    @autoreleasepool {
        for (int i = 1; i < argc; i++)
            if (!strcmp(argv[i], "--slow") && i + 1 < argc) g_slow = atoi(argv[++i]) > 1 ? atoi(argv[i]) : 1;
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
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
               "the red pillar again. Status every 5 s here and in ~/Library/Logs/psvr2xr.log.\n"
               "Quit with Ctrl-C.\n");
        pthread_t t;
        pthread_create(&t, NULL, render_thread, NULL);
        pthread_detach(t);
        [NSApp run];
    }
    return 0;
}
