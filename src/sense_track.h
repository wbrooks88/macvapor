// Position tracking of the PlayStation VR2 Sense controllers from the headset's cameras.
//
// Each controller has 17 infrared LEDs on its ring and grip (positions and facing from Monado's
// pssense_led_model.h, BSL-1.0, data extracted from Sony's PC driver). sense.c keeps them lit
// continuously, the way psvr2-controller-ir-lights-enable (MIT) does: PRESCAN pulses of 2.1 ms
// every 2 ms. The headset's two lower cameras, the ones passthrough shows, see them as bright dots.
//
// This file is the part that needs no hardware: finding the dots in a camera frame, and finding
// the pose of each controller that puts its LEDs on those dots. The controller's own IMU gives its
// orientation, level with gravity, so the search only has to find its turn about the vertical and
// its position; two LEDs fix a hypothesis, and the rest confirm it. Once a controller is found, its
// next pose is looked for near the last one. backend_psvr2.m feeds frames in and blends the
// result with the arm model when a controller can't be seen.
//
// Conventions: the head's axes are x right, y up, -z forward; the world's are the same with y up
// along gravity. A camera's own axes are OpenCV's: x right, y down, z forward through the lens.
// Quaternions are x, y, z, w.
#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Cameras: lower left, lower right, upper left, upper right.
enum { STRACK_CAMERAS = 4, STRACK_MAX_BLOBS = 48, STRACK_LEDS = 17 };

// A fisheye camera: Kannala-Brandt's model, r = f (t + k1 t^3 + k2 t^5 + k3 t^7 + k4 t^9) pixels
// from (cx, cy) for a ray t radians off the lens axis. Pixels are those of the full 1016-pixel frame.
typedef struct {
    float f, cx, cy, k[4];
    float max_angle_deg;    // rays further off the axis than this are outside the picture
    int width, height;
    float position[3];      // the lens in the head, metres from between the eyes
    float yaw_deg;          // + turns the lens to the left
    float pitch_deg;        // + tilts it up
    float roll_deg;         // + turns the picture counterclockwise
    bool off;               // not used (no model of it yet)
} strack_camera;

// The cameras as fitted to captures from a real headset: the lower pair at the front corners of
// the visor, turned about 12 degrees out and 28 down, the upper pair 30 out and 10 up, each seeing
// about 150 degrees across.
void strack_default_cameras(strack_camera out[STRACK_CAMERAS]);

typedef struct {
    float threshold;        // 0..255: a pixel this bright can be part of an LED's dot
    int max_area;           // dots bigger than this many pixels aren't LEDs (lamps, windows)
    float gate_px;          // how far a predicted LED may be from a dot to count as seeing it
    int min_leds;           // LEDs that must be matched for a pose found from scratch
    float max_rms_px;       // and how well, on average
    float search_hz;        // how often to search for a controller that isn't being followed
    bool calibrate;         // fit the cameras' turn, centre and focal length to what it sees
} strack_params;

void strack_default_params(strack_params *out);

typedef struct {
    float q[4];             // x, y, z, w
    float p[3];
} strack_pose;

typedef struct {
    float x, y;             // pixels
    float ray[3];           // unit, in the camera's axes
    float brightness;       // sum over the dot of how much it is above the threshold
    int area;
} strack_blob;

typedef struct {
    int64_t time_ns;               // when the frame was exposed
    strack_pose head;              // the head in the world then
    bool prior_valid[2];           // left, right: the controller is on and its IMU reports
    float prior_q[2][4];           // its LED model's orientation in the world, from its IMU; the
                                   // turn about the vertical may be off
} strack_input;

typedef struct {
    bool valid;
    bool found;                    // found by a search this frame, not followed from the last
    strack_pose pose;              // the LED model (Monado's frame, which sense.c calls the grip)
                                   // in the world
    float yaw_error;               // radians the prior's turn about the vertical is off by: turn
                                   // the prior by this about the world's up to match
    int leds;                      // LEDs matched to dots, over both cameras
    float rms_px;
} strack_result;

typedef struct strack strack;

strack *strack_create(const strack_camera cameras[STRACK_CAMERAS], const strack_params *params);
void strack_destroy(strack *t);

// Finds the dots in a frame. bc4 is one camera's picture as the headset sends it: BC4 blocks,
// row by row, stride_blocks to a row (256 for the 1024-wide planes). Only blocks with a bright
// end point are decoded, so this is quick on a frame of LEDs in a dim picture. Returns the number
// of dots kept.
int strack_detect_bc4(strack *t, int cam, const uint8_t *bc4, int stride_blocks);
// The same from 8-bit grey pixels (tests, captures).
int strack_detect_gray(strack *t, int cam, const uint8_t *pixels, int stride);
// From a half-size 8-bit picture (the headset's tracking mode, width by height); the dots come out
// in the full frame's pixels, as the camera model has them. Nothing for a camera that is off.
int strack_detect_half(strack *t, int cam, const uint8_t *pixels, int width, int height, int stride);
// From the headset's own spot finder: block is one camera's part of an LED detector packet (a
// count and 36-byte spots; see cpsvr2.h), the spots in the half-size picture's pixels. Spots go
// through the same tests as dots found in a picture, bar the look at what's around them.
int strack_detect_spots(strack *t, int cam, const uint8_t *block);
// No picture from this camera this frame.
void strack_no_picture(strack *t, int cam);
const strack_blob *strack_blobs(const strack *t, int cam, int *count);

// The controllers' poses from the dots found in this frame's cameras.
void strack_solve(strack *t, const strack_input *in, strack_result out[2]);

// The cameras as calibrated so far (or as given, until the first calibration), to save and start
// from next time; and how many calibrations have been taken in, with the last one's fit in pixels.
void strack_get_cameras(const strack *t, strack_camera out[STRACK_CAMERAS]);
int strack_calibration(const strack *t, float *rms_px);

// The world has moved under the tracker (recentering): stop following from the last poses, and
// search afresh. What it has learnt of the IMUs' yaw and the cameras stays.
void strack_forget(strack *t);

// The caller has turned controller hand's prior by angle radians about the vertical (taking in
// some of a yaw_error it was told), so the tracker's own idea of the error shrinks by as much.
void strack_shift_yaw(strack *t, int hand, float angle);

// The camera model, exposed for tests and the capture tool.
bool strack_project(const strack_camera *c, const float p_cam[3], float *u, float *v);
void strack_unproject(const strack_camera *c, float u, float v, float ray[3]);

// The LED model: left (0) or right (1), LED i, position (m) and facing (unit).
void strack_led(int hand, int i, float pos[3], float normal[3]);

// --------------------------------------------------------------------------- smoothing

// The optical positions in, smoothed and carried forward to the time a frame is shown: an
// alpha-beta filter on the position, its velocity damped while it isn't seen.
typedef struct {
    bool have;
    int64_t t_ns;           // of the state
    float p[3], v[3];
    int64_t last_seen_ns;
    // A one-euro filter ahead of it (Casiez et al.): the position smoothed hard while the hand is
    // still and hardly at all while it moves fast, so fit jitter of a few millimetres goes and
    // swings keep up. min_hz 0 turns it off.
    float min_hz, beta;     // cutoff at rest, and how much it rises per m/s
    bool e_have;
    int64_t e_t_ns;
    float e_p[3], e_dp[3];
} strack_filter;

void strack_filter_update(strack_filter *f, const float p[3], int64_t t_ns);
// The one-euro filter's settings: a cutoff of 1.5 Hz at rest, rising 8 Hz per m/s.
void strack_filter_init(strack_filter *f);
// The position at t_ns; false if it hasn't been seen for max_age_ns.
bool strack_filter_predict(const strack_filter *f, int64_t t_ns, int64_t max_age_ns, float out[3], float vel[3]);

#ifdef __cplusplus
}
#endif
