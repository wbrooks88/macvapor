// alvrxr: an OpenXR runtime for macOS that displays through ALVR's server_core, so the ALVR
// visionOS client shows the frames and supplies head, controller and button input.
//
// runtime.m is the OpenXR surface; backend.m owns ALVR, the compositor and the encoder.
// backend_psvr2.m is the other implementation of this interface: a PS VR2 on the Mac's own
// display output (build-psvr2.sh, PSVR2.md).
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __OBJC__
#import <Metal/Metal.h>
#endif

// Devices, in the order ALVR's /user/head, /user/hand/left, /user/hand/right. AXR_EYES, past the
// count, is where the eyes look: at the head, turned along the gaze (XR_EXT_eye_gaze_interaction).
enum { AXR_HEAD = 0, AXR_LEFT = 1, AXR_RIGHT = 2, AXR_DEVICE_COUNT = 3, AXR_EYES = 3 };

typedef struct {
    float orientation[4];      // x, y, z, w
    float position[3];
} axr_pose;

typedef struct {
    axr_pose pose;
    float linear_velocity[3];
    float angular_velocity[3];
    bool valid;
} axr_motion;

typedef struct {
    axr_pose eye_in_head[2];   // left, right
    float fov[2][4];           // left, right, up, down (radians, left/down negative)
} axr_views;

void axr_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

const char *axr_system_name(void);     // what xrGetSystemProperties reports

int64_t axr_now_ns(void);              // XrTime: CLOCK_MONOTONIC nanoseconds

// Starts ALVR (idempotent). The runtime works with no headset connected: frames are composed
// and dropped, and poses fall back to a standing default.
void axr_backend_start(void);

// The game's audio, tapped by MacVapoR Audio and handed to ALVR (audio.m).
bool axr_audio_available(void);
void axr_audio_start(void);
void axr_audio_stats(uint64_t *frames, float *peak);   // since the last call
bool axr_backend_connected(void);
bool axr_backend_tracking(void);     // connected and past the client's placeholder poses
double axr_refresh_hz(void);
void axr_recommended_eye_size(uint32_t *w, uint32_t *h);

// Blocks until the next frame is due and returns its predicted display time. Each frame is
// tied to the newest ALVR tracking sample, so poses and the encoded frame agree on the time.
int64_t axr_wait_frame(void);

// Motion of a device at an XrTime, in ALVR's stage space (floor origin, -Z forward).
axr_motion axr_device_motion(int device, int64_t time);
axr_views axr_current_views(void);

// Current value of an ALVR input path (e.g. "/user/hand/left/input/trigger/value").
bool axr_input(const char *path, float *value);
uint64_t axr_input_generation(void);   // bumps whenever any input changes
void axr_haptics(int hand, float duration_s, float frequency, float amplitude);

// Eye tracking. axr_backend_has_eye_tracker says whether this backend's headset has one at all
// (fixed, so the runtime can list the extensions before anything starts); axr_eye_tracking says
// whether it is on and reporting now.
bool axr_backend_has_eye_tracker(void);
bool axr_eye_tracking(void);

// Foveated rendering with quad views (XR_VARJO_quad_views, XR_VARJO_foveated_rendering): the game
// renders each eye twice, the whole view at a low resolution and a small inset at a high one,
// and the compositor lays the inset over the rest. Sizes are per view (w, h) for the outer and
// the inset views. The inset's field of view (left, right, up, down, radians) follows the eyes
// when gaze is true and they are tracked, and stays in the middle otherwise.
bool axr_foveated_supported(void);
void axr_eye_calibrate(void);   // PS VR2 only: runs the eye tracker's calibration in the headset
void axr_foveated_sizes(uint32_t outer[2], uint32_t inset[2]);
void axr_foveated_inset(int64_t time, bool gaze, float fov[2][4]);

#ifdef __OBJC__
id<MTLDevice> axr_metal_device(void);

typedef struct {
    id<MTLTexture> texture;    // 2D or 2D array
    uint32_t slice;
    int32_t x, y, width, height;
} axr_eye_image;

// Composes the two eye images into ALVR's (foveated) stream layout on the app's queue, after
// the app's own rendering, and encodes and sends the result for that frame's display time.
void axr_submit(int64_t display_time, const axr_eye_image eyes[2], id<MTLCommandQueue> queue);

// The same with quad views: eyes[0..1] the whole view, eyes[2..3] the insets, rendered with the
// field of view inset_fov (as axr_foveated_inset gives it).
void axr_submit_quad(int64_t display_time, const axr_eye_image eyes[4], const float inset_fov[2][4],
                     id<MTLCommandQueue> queue);
#endif
