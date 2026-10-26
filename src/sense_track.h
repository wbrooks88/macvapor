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
    // With the fusion's prediction (strack_input's pred_p) sure to within pred_max_sigma (0: not
    // used), a follow starts there and may move pred_jump_m from it, and a search's find further
    // than pred_veto_m from it (0: any) is left out. Off: on the 80 s recording of fast moves
    // (0.03, 0.06, 0.15) found the controllers in 2% more frames but was no closer once fused.
    float pred_max_sigma, pred_jump_m, pred_veto_m;
    int poor_dots;          // this many dots left unexplained near a fit on under 10 LEDs: search afresh
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
    bool pred_valid[2];            // where the fusion (strack_fusion) has the controller at this
    float pred_p[2][3];            // exposure, from its fits and its accelerometer since, and how
    float pred_sigma[2];           // sure it is (m)
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
    int expected;                  // LEDs the pose turns squarely to a camera, in its picture
    float tilt_error[2];           // radians the prior is tilted off by, about the world's x and z
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
// Dots found earlier (a capture's), in the full frame's pixels, brightest first: x, y, brightness
// and area as strack_blobs() gave them. Replaces this camera's.
int strack_set_dots(strack *t, int cam, const strack_blob *dots, int n);
// No picture from this camera this frame.
void strack_no_picture(strack *t, int cam);
const strack_blob *strack_blobs(const strack *t, int cam, int *count);

// The controllers' poses from the dots found in this frame's cameras.
void strack_solve(strack *t, const strack_input *in, strack_result out[2]);
// The last fit of a hand's matches: camera, LED and dot (strack_blobs' index), up to max.
int strack_last_matches(const strack *t, int hand, int (*out)[3], int max);

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
// The camera's axes (x right, y down, z forward) in the head's, as a row-major 3x3.
void strack_camera_rotation(const strack_camera *c, float R[9]);

// The LED model: left (0) or right (1), LED i, position (m) and facing (unit).
void strack_led(int hand, int i, float pos[3], float normal[3]);

// --------------------------------------------------------------------------- fusion
//
// Each controller's position from its fits and its own accelerometer: a Kalman filter on position,
// velocity and the accelerometer's bias (each axis on its own), driven between fits by the IMU's
// acceleration in the world and corrected by each fit at its exposure time, however late that
// arrives. A fit far from where the IMU says the controller went is left out (the tracker's few-LED
// fits are now and then on the wrong dots, tens of centimetres off); if the fits keep agreeing with
// each other and not with it, the filter starts again from them. Each correction is eased in over
// blend_s, so the position doesn't step at the cameras' 60 Hz. On a recording of fast moves this
// was 2 to 3 times closer to the best fits than the smoothing before it, at a tenth of the wobble.

enum { STRACK_IMU_RING = 256 };

typedef struct {
    bool have;
    int64_t t_ns;              // of the state
    float x[3][3];             // per axis: position (m), velocity (m/s), accelerometer bias (m/s2)
    float P[3][3][3];          // and its covariance
} strack_track;

typedef struct {
    // Settings (strack_fusion_init's are the ones tuned on the recording).
    float accel_noise;         // m/s2: how far the accelerometer is trusted
    float bias_walk;           // m/s2 per root second
    float sigma_scale;         // on a fit's error by its LEDs (3 mm on 13 or more, to 2 cm on 3 to 5)
    float gate;                // a fit further than this many deviations from the prediction is left out
    float gate_sigma;          // m: the most a fit's own error counts for in that (0: as weighted)
    float start_v_sigma;       // m/s: a track starting less sure of its velocity than this starts still
    float max_speed;           // m/s
    float blend_s;             // how long a correction takes to ease in; 0 shows it at once
    float imu_lag_s;           // the IMU's readings are this much later than they say against the cameras
    float coast_after_s;       // with no fit for this long it coasts: the accelerometer counted at
    float coast_accel;         // this much, the velocity fading over coast_tau_s, and where it is
    float coast_tau_s;         // growing unsure by coast_noise m per root second
    float coast_noise;
    int restart_fits;          // fits agreeing with each other, and not with the filter, to restart it
    int restart_misses;        // after the filter has left out this many in a row
    int restart_leds;          // and one of them on at least this many LEDs
    // State.
    struct { int64_t t_ns; float a[3]; } imu[STRACK_IMU_RING];
    unsigned nimu;
    strack_track main, challenger;
    struct { int64_t t_ns; float p[3], sigma; strack_track before; } fix[8];   // the last fits taken in
    unsigned nfix;
    int64_t last_ns;           // the last fit taken in
    int rejects, ch_n, ch_best;
    float off[3], off_v[3];    // the correction still being eased in, as of off_ns, and its rate
    int64_t off_ns;
    unsigned accepted, rejected, restarts, refined;   // counts, for the log
} strack_fusion;

void strack_fusion_init(strack_fusion *f);
// Forget the position (recentering); the IMU's readings stay.
void strack_fusion_reset(strack_fusion *f);
// An IMU reading: when, and the acceleration in the world (m/s2), gravity taken out. In order.
void strack_fusion_imu(strack_fusion *f, int64_t t_ns, const float accel[3]);
// A fit: the exposure time, the LED model's position in the world, how many LEDs and how well.
// Returns whether it was taken in.
bool strack_fusion_optical(strack_fusion *f, int64_t t_ns, const float p[3], int leds, float rms_px);
// A better fit for an exposure already gone by (the pictures' fit, after the spot finder's): it
// takes the place of the fit taken in for that exposure, or goes in among the later ones, and those
// are taken in again after it. False if it's too old (more than 8 fits back) or far from the rest.
bool strack_fusion_refine(strack_fusion *f, int64_t t_ns, const float p[3], int leds, float rms_px);
// The position and velocity at t_ns, carried on by the IMU's readings since the last fit; false if
// no fit was taken in for max_age_ns.
bool strack_fusion_predict(const strack_fusion *f, int64_t t_ns, int64_t max_age_ns, float out[3], float vel[3]);
// The filter's own estimate at t_ns, without the easing, and how sure it is (m, one deviation): for
// the tracker to look there (strack_input's pred_p). False as strack_fusion_predict.
bool strack_fusion_estimate(const strack_fusion *f, int64_t t_ns, int64_t max_age_ns, float out[3], float *sigma);

#ifdef __cplusplus
}
#endif
