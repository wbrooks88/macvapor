/* PSVR2 C core: reading headset SLAM poses via libusb (protocol from the Monado driver). */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Start: opens the device, claims the interfaces, starts the read thread.
 * Returns 0 on success. */
int psvr2_start(void);

void psvr2_stop(void);

/* 1 — the headset is connected and poses are coming in. */
int psvr2_connected(void);

/* Last raw SLAM quaternion and position.
 * quat_wxyz: w, x, y, z as in the USB packet; pos_xyz: meters. Returns 1 if the pose is valid. */
int psvr2_get_pose(float quat_wxyz[4], float pos_xyz[3]);

/* Proximity sensor (headset is worn) and IPD in mm. */
int psvr2_get_status(int *proximity, int *ipd_mm);

/* Bias-corrected angular velocity (rad/s, axes match the quaternion after
 * Monado mapping) and the age of the last SLAM pose in seconds. */
int psvr2_get_motion(float gyro_radps[3], double *slam_age_s);

/* Predicted quaternion: the gyro-integrated orientation, corrected toward
 * SLAM poses (fusion.c), extrapolated lookahead_s ahead. The quaternion is
 * ALREADY in Monado-mapped axes (x-right, y-up, -z-forward before the
 * 90-degree correction); no remap needed in the caller. Returns 1 if the
 * pose is valid. */
int psvr2_get_predicted_quat(float lookahead_s, float out_wxyz[4]);

/* Estimated gyro bias (deg/s) and the largest SLAM correction (deg) since
 * the previous call. Returns 1 once fusion has a pose. */
int psvr2_get_fusion_status(float *bias_dps, float *max_correction_deg);

/* Fn button on the headset: 1 — pressed. */
int psvr2_get_button(void);

/* Display brightness 0..1. */
int psvr2_set_brightness(float brightness);

/* 8 distortion parameters computed from the headset calibration (as in Monado). */
int psvr2_get_distortion_calibration(float out[8]);

/* Pointer to the 1024x3 float LUT (r,g,b). */
const float *psvr2_distortion_lut(void);

/* --- Headset cameras (passthrough) ---
 * Frames arrive as a pair of 1024x1016 BC4 textures (left and right cameras),
 * 60 Hz. Protocol from PSVR2Toolkit: enabled with vendor command 0x0B,
 * stream on interface 6, EP 0x87, packet signature 'V','I'. */

#define PSVR2_CAM_WIDTH 1024
#define PSVR2_CAM_HEIGHT 1016
/* BC4: 4 bits per pixel */
#define PSVR2_CAM_PLANE_BYTES (PSVR2_CAM_WIDTH * PSVR2_CAM_HEIGHT / 2)

/* Powers on the cameras and starts the read thread for passthrough. 0 — success. Counted:
 * passthrough and controller tracking each start and stop them, and they stay on while either
 * wants them. */
int psvr2_camera_start(void);

void psvr2_camera_stop(void);

/* The same for controller tracking. While nothing wants passthrough, the headset sends the
 * tracking mode instead of the BC4 frames: all four cameras at half size (512x508 8-bit, the last few
 * columns padding), short exposure, the lower pair and then the upper pair,
 * taken at the same moment, 60 times a second each. */
#define PSVR2_TRACK_WIDTH 512
#define PSVR2_TRACK_HEIGHT 508
#define PSVR2_TRACK_PICTURE_BYTES (PSVR2_TRACK_WIDTH * PSVR2_TRACK_HEIGHT)
int psvr2_camera_start_tracking(void);
void psvr2_camera_stop_tracking(void);

/* Called on the camera thread with each tracking-mode packet: its header, which pair (0 lower, 1 upper), and its left and
 * right pictures. Only valid during the call. NULL stops it. */
typedef void (*psvr2_tracking_listener)(const unsigned char *header, int upper, const unsigned char *left,
                                        const unsigned char *right, double arrived_s, void *ctx);
void psvr2_camera_set_tracking_listener(psvr2_tracking_listener fn, void *ctx);

/* The headset's own spot finder ('L','D' packets on interface 8, EP 0x89), streaming while the
 * cameras do, 60 times a second. Each packet covers one exposure of all four cameras:
 *   0x00  'L','D'; bytes 8..11 the exposure time in headset microseconds, as in the cameras'
 *         headers; bytes 0x10, 0x12 the pictures' width and height (508)
 *   0x40  four cameras (lower left, lower right, upper left, upper right), PSVR2_LD_CAMERA_BYTES
 *         each: a uint32 count, then up to 256 spots of 36 bytes:
 *           u16 kind (1; 2 for some faint ones beside a bigger spot), u8 peak value, u8 ?,
 *           u16 x0, x1, y0, y1 (the spot's box in the half-size picture's pixels, inclusive),
 *           u32 sums over the box of w, w*dx, w*dy, w*dx*dy, w*dx*dx, w*dy*dy, where w is a
 *           pixel's value less 32 (pixels above it only) and dx, dy are from x0, y0.
 * Called on its own thread with each packet; only valid during the call. NULL stops it. */
#define PSVR2_LD_PACKET_BYTES 36944
#define PSVR2_LD_CAMERA_BYTES 9220
#define PSVR2_LD_SPOT_BYTES 36
typedef void (*psvr2_led_detector_listener)(const unsigned char *packet, double arrived_s, void *ctx);
void psvr2_camera_set_led_detector_listener(psvr2_led_detector_listener fn, void *ctx);

/* Called on the camera thread with every frame as it arrives: the packet's 256-byte header, the
 * two BC4 planes (left camera, right camera; PSVR2_CAM_PLANE_BYTES each, 256 blocks to a row), and
 * the CLOCK_MONOTONIC second it arrived. The planes are only valid during the call. NULL stops it. */
typedef void (*psvr2_camera_listener)(const unsigned char *header, const unsigned char *left,
                                      const unsigned char *right, double arrived_s, void *ctx);
void psvr2_camera_set_listener(psvr2_camera_listener fn, void *ctx);

/* Copies the last frame: left and right are PSVR2_CAM_PLANE_BYTES buffers each.
 * Returns 1 if the frame is new (since the previous call), 0 otherwise. */
int psvr2_camera_get_frame(unsigned char *left, unsigned char *right);

/* --- Eye tracking ---
 * The headset's own eye tracker sends a 'G','S' gaze state on interface 5, EP 0x85, about 120
 * times a second, while vendor command 0x0C (subcommand 1) keeps it on. The layout is the one
 * PSVR2Toolkit (MIT) and Monado's driver (BSL-1.0) agree on. Directions are mapped into the
 * head's axes as both do: x right, y up, -z forward. */

typedef struct {
	int valid;               /* the combined direction is valid */
	float dir[3];            /* combined gaze direction, unit vector in head axes */
	int origin_valid;
	float origin_m[3];       /* combined gaze origin in metres from the headset's centre */
	int eye_valid[2];        /* left, right */
	float eye_dir[2][3];
	float eye_origin_m[2][3];
	float pupil_mm[2];       /* 0 when unknown */
	int blink[2];            /* 1: that eye is closed */
	unsigned calibration_id; /* 0: the headset holds no user calibration */
	unsigned frame;          /* the headset's gaze frame counter */
	double age_s;            /* since the packet arrived; large when none has */
} psvr2_gaze;

/* Turns the eye tracker on and starts reading it. calibration_path is a user calibration to send
 * first if the file exists, and where a new one is saved when the headset reports it (after a
 * calibration, interface 10's 'V','D' data, item 3, as PSVR2Toolkit saves it). NULL for neither.
 * 0: success. */
int psvr2_gaze_start(const char *calibration_path);

void psvr2_gaze_stop(void);

/* The newest gaze state. Returns 1 if one has arrived. */
int psvr2_get_gaze(psvr2_gaze *out);

/* Gaze packets received since the previous call. */
unsigned psvr2_gaze_packets(void);

/* Calibration (report 0x0D to set, 0x8D to read back; the sequence is PSVR2Toolkit.UnitySetup's):
 * SET_EYE (arg: 0 left, 1 right, 2 both), START, then per target COLLECT with the target's
 * position in millimetres (x left, y up, +z ahead of the eyes) and poll until result is 0,
 * COMPUTE after each stage and poll until status is COMPUTE_OK or COMPUTE_FAILED, then STOP,
 * SET_EYE both, and RETRIEVE, which makes the headset report the new calibration on VD. */
enum {
	PSVR2_GAZE_CAL_START = 1,
	PSVR2_GAZE_CAL_COLLECT = 2,
	PSVR2_GAZE_CAL_DISCARD = 3,
	PSVR2_GAZE_CAL_COMPUTE = 4,
	PSVR2_GAZE_CAL_RETRIEVE = 5,
	PSVR2_GAZE_CAL_STOP = 6,
	PSVR2_GAZE_CAL_SET_EYE = 7,
};
enum { PSVR2_GAZE_STATUS_COMPUTE_OK = 5, PSVR2_GAZE_STATUS_COMPUTE_FAILED = 6 };
enum { PSVR2_GAZE_RESULT_OK = 0, PSVR2_GAZE_RESULT_FAILED = 1, PSVR2_GAZE_RESULT_WAITING = 3 };

int psvr2_gaze_calibration_command(int mode, float x_mm, float y_mm, float z_mm, int arg);

/* The calibration status (enum above, or 1 for tracking) and the last point's result.
 * 0: success. */
int psvr2_gaze_calibration_status(int *status, int *result);

/* Seconds since a calibration blob last arrived on VD and was saved; negative if none has. */
double psvr2_gaze_calibration_saved_age(void);

#ifdef __cplusplus
}
#endif
