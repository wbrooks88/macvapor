/*
 * PSVR2 pose reading via libusb. The protocol is documented in the
 * Monado driver (BSL-1.0): src/xrt/drivers/psvr2.
 */
#include "cpsvr2.h"
#include "fusion.h"

#include <libusb.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define PSVR2_VID 0x054C
#define PSVR2_PID 0x0CDE

#define SLAM_INTERFACE 3
#define SLAM_ENDPOINT 0x83
#define STATUS_INTERFACE 7
#define STATUS_ENDPOINT 0x88
#define CAMERA_INTERFACE 6
#define CAMERA_ENDPOINT 0x87
/* Camera frame header ('V','I'), followed by two BC4 planes */
#define CAMERA_HEADER_BYTES 256

#define GYRO_SCALE (2000.0f / 32767.0f)
#define DEG_TO_RAD(d) ((d) * (float)M_PI / 180.0f)

#pragma pack(push, 1)
struct imu_usb_record {
	uint32_t vts_us;
	int16_t accel[3];
	int16_t gyro[3];
	uint16_t dp_frame_cnt;
	uint16_t dp_line_cnt;
	uint16_t imu_ts_us;
	uint16_t status;
};

struct status_record_hdr {
	uint8_t dprx_status;
	uint8_t prox_sensor_flag;
	uint8_t function_button;
	uint8_t empty0[2];
	uint8_t ipd_dial_mm;
	uint8_t remainder[26];
};

struct slam_usb_record {
	char hdr[3];
	uint8_t const1;
	uint32_t pkt_size;
	uint32_t vts_ts_us;
	uint32_t unknown1;
	float pos[3];
	float orient[4];
	uint8_t remainder[468];
};

struct sie_ctrl_pkt {
	uint16_t report_id;
	uint16_t subcmd;
	uint32_t len;
	uint8_t data[512 - 8];
};
#pragma pack(pop)

static libusb_context *g_ctx;
static libusb_device_handle *g_dev;
static pthread_t g_slam_thread, g_status_thread;
static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int g_running;
static int g_have_pose;
static float g_quat[4];
static float g_pos[3];
static float g_gyro[3];
static double g_slam_time;
static double g_imu_time;
static int g_proximity;
static int g_function_button;
static int g_ipd_mm = 63;

/* Cameras: separate mutex — copying a frame (1 MB) must not delay
 * pose and IMU reads under the shared g_lock */
static pthread_mutex_t g_camera_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_camera_thread;
static volatile int g_camera_running;
static unsigned char *g_camera_frame; /* two BC4 planes back to back */
static int g_camera_seq;              /* grows with every received frame */
static int g_camera_seq_taken;

/* Gyro integration corrected toward SLAM poses (see fusion.c) */
#define IMU_DT 0.0005f
static struct fusion_state g_fusion;
/* Largest SLAM difference since the last status read, radians */
static float g_max_correction;

static double monotonic_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void *slam_thread_fn(void *arg)
{
	(void)arg;
	uint8_t buf[1024];

	while (g_running) {
		int transferred = 0;
		int ret = libusb_bulk_transfer(g_dev, SLAM_ENDPOINT, buf, sizeof(buf), &transferred, 500);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			continue;
		}
		if (ret != 0) {
			if (g_running) {
				fprintf(stderr, "psvr2: slam read error: %s\n", libusb_error_name(ret));
			}
			break;
		}
		if (transferred < (int)sizeof(struct slam_usb_record)) {
			continue;
		}

		struct slam_usb_record rec;
		memcpy(&rec, buf, sizeof(rec));

		pthread_mutex_lock(&g_lock);
		/* Quaternion continuity: pick the closer of q/-q */
		float dot = g_quat[0] * rec.orient[0] + g_quat[1] * rec.orient[1] +
		            g_quat[2] * rec.orient[2] + g_quat[3] * rec.orient[3];
		float sign = (g_have_pose && dot < 0.0f) ? -1.0f : 1.0f;
		for (int i = 0; i < 4; i++) {
			g_quat[i] = sign * rec.orient[i];
		}
		memcpy(g_pos, rec.pos, sizeof(g_pos));
		g_slam_time = monotonic_s();
		/* Bring the SLAM quaternion into the gyro's Monado-mapped axes
		 * (as in process_slam_record) */
		float mapped[4] = {g_quat[0], -g_quat[2], -g_quat[1], g_quat[3]};
		fusion_slam(&g_fusion, rec.vts_ts_us, mapped);
		if (g_fusion.last_error > g_max_correction) {
			g_max_correction = g_fusion.last_error;
		}
		g_have_pose = 1;
		pthread_mutex_unlock(&g_lock);
	}
	return NULL;
}

static void *status_thread_fn(void *arg)
{
	(void)arg;
	uint8_t buf[1024];

	while (g_running) {
		int transferred = 0;
		int ret = libusb_interrupt_transfer(g_dev, STATUS_ENDPOINT, buf, sizeof(buf), &transferred, 500);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			continue;
		}
		if (ret != 0) {
			break;
		}
		if (transferred < (int)sizeof(struct status_record_hdr)) {
			continue;
		}
		struct status_record_hdr *hdr = (struct status_record_hdr *)buf;

		int n_imu = (transferred - (int)sizeof(*hdr)) / (int)sizeof(struct imu_usb_record);

		pthread_mutex_lock(&g_lock);
		g_proximity = hdr->prox_sensor_flag;
		g_function_button = hdr->function_button;
		g_ipd_mm = hdr->ipd_dial_mm;
		for (int i = 0; i < n_imu; i++) {
			struct imu_usb_record imu;
			memcpy(&imu, buf + sizeof(*hdr) + i * sizeof(imu), sizeof(imu));

			/* Axis mapping as in Monado process_imu_record */
			float gyro[3] = {
				-DEG_TO_RAD(imu.gyro[1] * GYRO_SCALE),
				DEG_TO_RAD(imu.gyro[2] * GYRO_SCALE),
				-DEG_TO_RAD(imu.gyro[0] * GYRO_SCALE),
			};
			fusion_imu(&g_fusion, imu.vts_us, gyro, IMU_DT);
			memcpy(g_gyro, g_fusion.rate, sizeof(g_gyro));
		}
		if (n_imu > 0) {
			g_imu_time = monotonic_s();
		}
		pthread_mutex_unlock(&g_lock);
	}
	return NULL;
}

static int get_control(uint16_t report_id, uint8_t subcmd, uint8_t *out, uint32_t size, uint16_t *subcmd_out)
{
	struct sie_ctrl_pkt pkt = {0};
	pkt.report_id = report_id;
	pkt.subcmd = subcmd;
	pkt.len = size;

	int ret = libusb_control_transfer(g_dev,
	    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT | LIBUSB_ENDPOINT_IN,
	    0x1, report_id, 0x0, (unsigned char *)&pkt, size + 8, 1000);
	if (ret < 0) {
		return ret;
	}
	memcpy(out, pkt.data, size);
	if (subcmd_out != NULL) {
		*subcmd_out = pkt.subcmd; /* the gaze report answers in this field */
	}
	return 0;
}

static int send_control(uint16_t report_id, uint8_t subcmd, const uint8_t *data, uint32_t len)
{
	struct sie_ctrl_pkt pkt = {0};
	pkt.report_id = report_id;
	pkt.subcmd = subcmd;
	pkt.len = len;
	if (len > 0) {
		memcpy(pkt.data, data, len);
	}

	int ret = libusb_control_transfer(g_dev,
	    LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_ENDPOINT,
	    0x9, report_id, 0x0, (unsigned char *)&pkt, len + 8, 1000);
	return ret < 0 ? ret : 0;
}

int psvr2_set_brightness(float brightness)
{
	if (g_dev == NULL) {
		return -1;
	}
	if (brightness < 0.0f) brightness = 0.0f;
	if (brightness > 1.0f) brightness = 1.0f;
	uint8_t b = (uint8_t)(brightness * 31.0f);
	return send_control(0x12, 1, &b, 1);
}

int psvr2_get_distortion_calibration(float out[8])
{
	memset(out, 0, 8 * sizeof(float));
	if (g_dev == NULL) {
		return -1;
	}

	uint8_t buf[0x100];
	if (get_control(0x8f, 1, buf, sizeof(buf), NULL) != 0) {
		return -1;
	}

	uint8_t version = buf[0];
	float p[8];
	memcpy(p, buf + 8, sizeof(p));

	/* Conversion from psvr2_setup_distortion_and_fovs (Monado) */
	if (version < 4) {
		out[0] = -0.09919293f;
		out[2] = 0.09919293f;
	} else {
		out[0] = (((-p[0] - p[6]) * 29.9f + 14.95f) / 1000.0f - 3.22f) / 32.46199f;
		out[1] = (((-p[1] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;
		out[2] = (((p[6] - p[2]) * 29.9f + 14.95f) / 1000.0f + 3.22f) / 32.46199f;
		out[3] = (((-p[3] * 29.9f) + 14.95f) / 1000.0f) / 32.46199f;

		float left = -p[4] * (float)M_PI / 180.0f;
		out[4] = cosf(left);
		out[5] = sinf(left);

		float right = -p[5] * (float)M_PI / 180.0f;
		out[6] = cosf(right);
		out[7] = sinf(right);
	}
	return 0;
}

int psvr2_start(void)
{
	if (g_running) {
		return 0;
	}

	int ret = libusb_init(&g_ctx);
	if (ret < 0) {
		fprintf(stderr, "psvr2: libusb_init: %s\n", libusb_error_name(ret));
		return -1;
	}

	g_dev = libusb_open_device_with_vid_pid(g_ctx, PSVR2_VID, PSVR2_PID);
	if (g_dev == NULL) {
		fprintf(stderr, "psvr2: device %04x:%04x not found\n", PSVR2_VID, PSVR2_PID);
		libusb_exit(g_ctx);
		g_ctx = NULL;
		return -1;
	}

	ret = libusb_claim_interface(g_dev, STATUS_INTERFACE);
	if (ret == 0) {
		ret = libusb_set_interface_alt_setting(g_dev, STATUS_INTERFACE, 1);
	}
	if (ret == 0) {
		ret = libusb_claim_interface(g_dev, SLAM_INTERFACE);
	}
	if (ret != 0) {
		fprintf(stderr, "psvr2: failed to claim interfaces: %s\n", libusb_error_name(ret));
		libusb_close(g_dev);
		g_dev = NULL;
		libusb_exit(g_ctx);
		g_ctx = NULL;
		return -1;
	}

	g_have_pose = 0;
	fusion_reset(&g_fusion);
	g_max_correction = 0;
	g_running = 1;
	pthread_create(&g_slam_thread, NULL, slam_thread_fn, NULL);
	pthread_create(&g_status_thread, NULL, status_thread_fn, NULL);
	return 0;
}

void psvr2_stop(void)
{
	if (!g_running) {
		return;
	}
	/* The camera and gaze threads use g_dev — shut them down before closing the
	 * device, also powering off the headset cameras and eye tracker */
	psvr2_camera_stop();
	psvr2_gaze_stop();
	g_running = 0;
	pthread_join(g_slam_thread, NULL);
	pthread_join(g_status_thread, NULL);
	libusb_release_interface(g_dev, SLAM_INTERFACE);
	libusb_release_interface(g_dev, STATUS_INTERFACE);
	libusb_close(g_dev);
	g_dev = NULL;
	libusb_exit(g_ctx);
	g_ctx = NULL;
}

int psvr2_connected(void)
{
	return g_running && g_have_pose;
}

int psvr2_get_pose(float quat_wxyz[4], float pos_xyz[3])
{
	pthread_mutex_lock(&g_lock);
	memcpy(quat_wxyz, g_quat, sizeof(g_quat));
	memcpy(pos_xyz, g_pos, sizeof(g_pos));
	int have = g_have_pose;
	pthread_mutex_unlock(&g_lock);
	return have;
}

int psvr2_get_status(int *proximity, int *ipd_mm)
{
	pthread_mutex_lock(&g_lock);
	*proximity = g_proximity;
	*ipd_mm = g_ipd_mm;
	pthread_mutex_unlock(&g_lock);
	return 0;
}

int psvr2_get_motion(float gyro_radps[3], double *slam_age_s)
{
	pthread_mutex_lock(&g_lock);
	memcpy(gyro_radps, g_gyro, sizeof(g_gyro));
	*slam_age_s = g_have_pose ? monotonic_s() - g_slam_time : 0.0;
	int have = g_have_pose;
	pthread_mutex_unlock(&g_lock);
	return have;
}

int psvr2_get_predicted_quat(float lookahead_s, float out_wxyz[4])
{
	pthread_mutex_lock(&g_lock);
	if (!g_have_pose || !g_fusion.initialized) {
		pthread_mutex_unlock(&g_lock);
		return 0;
	}
	/* From the newest IMU sample to the requested time */
	double ahead = monotonic_s() - g_imu_time + (double)lookahead_s;
	if (ahead < 0.0) ahead = 0.0;
	if (ahead > 0.1) ahead = 0.1;
	fusion_predict(&g_fusion, (float)ahead, out_wxyz);
	pthread_mutex_unlock(&g_lock);
	return 1;
}

int psvr2_get_fusion_status(float *bias_dps, float *max_correction_deg)
{
	pthread_mutex_lock(&g_lock);
	const float *b = g_fusion.bias;
	*bias_dps = sqrtf(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]) * 180.0f / (float)M_PI;
	*max_correction_deg = g_max_correction * 180.0f / (float)M_PI;
	g_max_correction = 0;
	int ready = g_fusion.initialized;
	pthread_mutex_unlock(&g_lock);
	return ready;
}

int psvr2_get_button(void)
{
	pthread_mutex_lock(&g_lock);
	int b = g_function_button;
	pthread_mutex_unlock(&g_lock);
	return b;
}

/* --- Headset cameras (passthrough) ---
 * Protocol from PSVR2Toolkit (https://github.com/BnuuySolutions/PSVR2Toolkit):
 * cameras are enabled with vendor command 0x0B, frames arrive as 'V','I'
 * packets (256-byte header + two BC4 planes). The command does not "stick":
 * send it after claiming the interface and repeat it when the stream idles.
 */

static int camera_power(int on)
{
	uint8_t data[8] = {1, 0, 0, 0, (uint8_t)(on ? 0x10 : 0x05), 0, 0, 0};
	return send_control(0x0B, 1, data, sizeof(data));
}

static void *camera_thread_fn(void *arg)
{
	(void)arg;
	const int frame_bytes = CAMERA_HEADER_BYTES + 2 * PSVR2_CAM_PLANE_BYTES;
	unsigned char *buf = malloc(frame_bytes + 4096);
	if (buf == NULL) {
		return NULL;
	}

	while (g_camera_running) {
		int transferred = 0;
		int ret = libusb_bulk_transfer(g_dev, CAMERA_ENDPOINT, buf,
		                               frame_bytes + 4096, &transferred, 300);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			camera_power(1); /* the stream may have stalled — re-request it */
			continue;
		}
		if (ret != 0) {
			break;
		}
		if (transferred < frame_bytes || buf[0] != 'V' || buf[1] != 'I') {
			continue;
		}

		pthread_mutex_lock(&g_camera_lock);
		if (g_camera_frame != NULL) {
			memcpy(g_camera_frame, buf + CAMERA_HEADER_BYTES, 2 * PSVR2_CAM_PLANE_BYTES);
			g_camera_seq++;
		}
		pthread_mutex_unlock(&g_camera_lock);
	}

	free(buf);
	return NULL;
}

int psvr2_camera_start(void)
{
	if (g_dev == NULL) {
		return -1;
	}
	if (g_camera_running) {
		return 0;
	}

	int ret = libusb_claim_interface(g_dev, CAMERA_INTERFACE);
	if (ret != 0) {
		fprintf(stderr, "psvr2: camera interface busy: %s\n", libusb_error_name(ret));
		return -1;
	}

	pthread_mutex_lock(&g_camera_lock);
	if (g_camera_frame == NULL) {
		g_camera_frame = malloc(2 * PSVR2_CAM_PLANE_BYTES);
	}
	g_camera_seq = 0;
	g_camera_seq_taken = 0;
	pthread_mutex_unlock(&g_camera_lock);

	if (g_camera_frame == NULL) {
		libusb_release_interface(g_dev, CAMERA_INTERFACE);
		return -1;
	}

	/* The power-on command must come after claiming the interface */
	camera_power(1);

	g_camera_running = 1;
	if (pthread_create(&g_camera_thread, NULL, camera_thread_fn, NULL) != 0) {
		g_camera_running = 0;
		camera_power(0);
		libusb_release_interface(g_dev, CAMERA_INTERFACE);
		return -1;
	}
	return 0;
}

void psvr2_camera_stop(void)
{
	if (!g_camera_running) {
		return;
	}
	g_camera_running = 0;
	pthread_join(g_camera_thread, NULL);
	camera_power(0);
	libusb_release_interface(g_dev, CAMERA_INTERFACE);
}

int psvr2_camera_get_frame(unsigned char *left, unsigned char *right)
{
	int fresh = 0;
	pthread_mutex_lock(&g_camera_lock);
	if (g_camera_frame != NULL && g_camera_seq != g_camera_seq_taken) {
		g_camera_seq_taken = g_camera_seq;
		memcpy(left, g_camera_frame, PSVR2_CAM_PLANE_BYTES);
		memcpy(right, g_camera_frame + PSVR2_CAM_PLANE_BYTES, PSVR2_CAM_PLANE_BYTES);
		fresh = 1;
	}
	pthread_mutex_unlock(&g_camera_lock);
	return fresh;
}

/* --- Eye tracking ---
 * Gaze state layout and the enable command from PSVR2Toolkit (usb_thread_gaze.cpp,
 * hmd2_gaze.h, MIT) and Monado (psvr2_eye.c, psvr2_protocol.h, BSL-1.0), which agree:
 * interface 5, EP 0x85, 'G','S' packets of 0x148 bytes; report 0x0C subcommand 1 turns the
 * stream on and has to be repeated, since it lapses (and stops on display mode changes).
 * The user calibration goes to EP 0x05 before that, and comes back on interface 10 (VD).
 */

#define GAZE_INTERFACE 5
#define GAZE_ENDPOINT 0x85
#define GAZE_CALIBRATION_ENDPOINT 0x05
#define VD_INTERFACE 10
#define VD_ENDPOINT 0x8B
#define GAZE_XFER_SIZE 32768
#define VD_XFER_SIZE 32768

#define REPORT_GAZE_STREAM 0x0C
#define REPORT_GAZE_CALIBRATION_SET 0x0D
#define REPORT_GAZE_CALIBRATION_GET 0x8D

#pragma pack(push, 1)
struct gaze_eye_record {
	uint32_t origin_valid;
	float origin_mm[3];
	uint32_t dir_valid;
	float dir[3];
	uint32_t pupil_valid;
	float pupil_mm;
	uint32_t pupil_pos_valid;
	float pupil_pos[2];
	uint32_t pos_guide_valid;
	float pos_guide[2];
	uint32_t blink_valid;
	uint32_t blink;
};

struct gaze_state_record {
	uint8_t magic[2]; /* 'G','S' */
	uint16_t version;
	uint32_t size;
	float exposure[2];
	uint32_t led_status;
	uint32_t exposure_counter[2];
	uint32_t led_counter;
	int32_t dsp_return_code;
	float lens_config[6];
	uint32_t user_calibration_id;
	float fr_gaze_origin[3];
	uint8_t enabled_eye;
	uint8_t motor_sequence;
	uint8_t motor_strength;
	uint8_t pad0;
	/* "wearable" data */
	int64_t timestamp;
	uint32_t frame_counter;
	struct gaze_eye_record eye[2];
	uint32_t combined_origin_valid;
	float combined_origin_mm[3];
	uint32_t combined_dir_valid;
	float combined_dir[3];
	uint32_t convergence_valid;
	/* "foveated" data */
	int64_t foveated_timestamp;
	uint32_t foveated_frame_counter;
	uint32_t foveated_tracking_state;
	float foveated_dir[3][3];
	float foveated_convergence;
};

struct gaze_calibration_packet {
	float x, y, z;
	uint8_t arg; /* result, or the eye for SET_EYE */
};

struct vd_header {
	char magic[2]; /* 'V','D' */
	uint16_t min_size;
	uint32_t size;
	uint16_t num_items;
	uint8_t pad[22];
};

struct vd_item {
	uint16_t id;
	uint16_t unk;
	uint32_t size;
	uint32_t offset;
};

struct gaze_calibration_header {
	char magic[2];
	uint16_t struct_version;
	uint16_t data_version;
	uint16_t data_id;
	uint32_t payload_size;
	uint32_t calibration_id;
	uint8_t calibration_eye;
	uint8_t pad[12];
};
#pragma pack(pop)

_Static_assert(sizeof(struct gaze_eye_record) == 72, "gaze eye layout");
_Static_assert(sizeof(struct gaze_state_record) == 0x148, "gaze state layout");

static pthread_mutex_t g_gaze_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_t g_gaze_thread, g_vd_thread;
static volatile int g_gaze_running;
static int g_gaze_claimed, g_vd_claimed;
static struct gaze_state_record g_gaze;
static int g_have_gaze;
static double g_gaze_time;
static unsigned g_gaze_packets;
static char g_gaze_calibration_path[1024];
static double g_gaze_calibration_saved;

static int gaze_stream(int on)
{
	return send_control(REPORT_GAZE_STREAM, on ? 1 : 2, NULL, 0);
}

/* Sends a saved user calibration to the eye tracker, if there is one. */
static void gaze_upload_calibration(void)
{
	if (g_gaze_calibration_path[0] == 0) {
		return;
	}
	FILE *f = fopen(g_gaze_calibration_path, "rb");
	if (f == NULL) {
		return;
	}
	unsigned char buf[16384];
	size_t n = fread(buf, 1, sizeof(buf), f);
	fclose(f);
	if (n == 0) {
		return;
	}
	int transferred = 0;
	int ret = libusb_bulk_transfer(g_dev, GAZE_CALIBRATION_ENDPOINT, buf, (int)n, &transferred, 1000);
	if (ret != 0) {
		fprintf(stderr, "psvr2: could not send the eye calibration: %s\n", libusb_error_name(ret));
	} else {
		fprintf(stderr, "psvr2: sent the eye calibration (%d bytes)\n", transferred);
	}
}

static void *gaze_thread_fn(void *arg)
{
	(void)arg;
	unsigned char *buf = malloc(GAZE_XFER_SIZE);
	if (buf == NULL) {
		return NULL;
	}
	double last_enable = monotonic_s();

	while (g_gaze_running) {
		/* The enable lapses after a few seconds: renew it every second, as Monado does */
		if (monotonic_s() - last_enable > 1.0) {
			gaze_stream(1);
			last_enable = monotonic_s();
		}
		int transferred = 0;
		int ret = libusb_bulk_transfer(g_dev, GAZE_ENDPOINT, buf, GAZE_XFER_SIZE, &transferred, 500);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			gaze_stream(1); /* stopped, e.g. by a display change — ask again */
			last_enable = monotonic_s();
			continue;
		}
		if (ret != 0) {
			if (g_gaze_running) {
				fprintf(stderr, "psvr2: gaze read error: %s\n", libusb_error_name(ret));
			}
			break;
		}
		if (transferred < (int)sizeof(struct gaze_state_record) || buf[0] != 'G' || buf[1] != 'S') {
			continue;
		}
		pthread_mutex_lock(&g_gaze_lock);
		memcpy(&g_gaze, buf, sizeof(g_gaze));
		g_have_gaze = 1;
		g_gaze_time = monotonic_s();
		g_gaze_packets++;
		pthread_mutex_unlock(&g_gaze_lock);
	}
	free(buf);
	return NULL;
}

/* Keeps a user calibration from a VD packet, if the packet carries a complete one. */
static void vd_save_calibration(const unsigned char *data, uint32_t size)
{
	if (size < sizeof(struct gaze_calibration_header) || g_gaze_calibration_path[0] == 0) {
		return;
	}
	struct gaze_calibration_header h;
	memcpy(&h, data, sizeof(h));
	/* Only a calibration of both eyes with an id (PSVR2Toolkit skips the blank ones) */
	if (h.calibration_eye != 2 || h.calibration_id == 0) {
		return;
	}
	/* Unchanged: nothing to write */
	FILE *f = fopen(g_gaze_calibration_path, "rb");
	if (f != NULL) {
		unsigned char old[16384];
		size_t n = fread(old, 1, sizeof(old), f);
		fclose(f);
		if (n == size && memcmp(old, data, size) == 0) {
			return;
		}
	}
	char tmp[1100];
	snprintf(tmp, sizeof(tmp), "%s.new", g_gaze_calibration_path);
	f = fopen(tmp, "wb");
	if (f == NULL) {
		fprintf(stderr, "psvr2: cannot write %s\n", tmp);
		return;
	}
	size_t written = fwrite(data, 1, size, f);
	fclose(f);
	if (written != size || rename(tmp, g_gaze_calibration_path) != 0) {
		fprintf(stderr, "psvr2: cannot save the eye calibration to %s\n", g_gaze_calibration_path);
		return;
	}
	pthread_mutex_lock(&g_gaze_lock);
	g_gaze_calibration_saved = monotonic_s();
	pthread_mutex_unlock(&g_gaze_lock);
	fprintf(stderr, "psvr2: saved eye calibration %u (%u bytes)\n", h.calibration_id, size);
}

/* VD packets ("general data", Hmd2GenData in PSVR2Toolkit): a 32-byte header and a table of
 * items, several packets back to back. Item 3 is the eye tracker's user calibration. */
static void vd_parse(const unsigned char *buf, int len)
{
	int at = 0;
	while (len - at >= 0x200) {
		struct vd_header h;
		memcpy(&h, buf + at, sizeof(h));
		if (h.magic[0] != 'V' || h.magic[1] != 'D' || h.min_size != 0x200 || h.size < sizeof(h) ||
		    h.size > (uint32_t)(len - at)) {
			return;
		}
		for (uint16_t i = 0; i < h.num_items; i++) {
			size_t item_at = sizeof(h) + i * sizeof(struct vd_item);
			if (item_at + sizeof(struct vd_item) > h.size) {
				break;
			}
			struct vd_item item;
			memcpy(&item, buf + at + item_at, sizeof(item));
			if ((uint64_t)item.offset + item.size > h.size) {
				break;
			}
			if (item.id == 3) {
				vd_save_calibration(buf + at + item.offset, item.size);
			}
		}
		at += (int)h.size;
	}
}

static void *vd_thread_fn(void *arg)
{
	(void)arg;
	unsigned char *buf = malloc(VD_XFER_SIZE);
	if (buf == NULL) {
		return NULL;
	}
	while (g_gaze_running) {
		int transferred = 0;
		int ret = libusb_bulk_transfer(g_dev, VD_ENDPOINT, buf, VD_XFER_SIZE, &transferred, 500);
		if (ret == LIBUSB_ERROR_TIMEOUT) {
			continue;
		}
		if (ret != 0) {
			if (g_gaze_running) {
				fprintf(stderr, "psvr2: VD read error: %s\n", libusb_error_name(ret));
			}
			break;
		}
		vd_parse(buf, transferred);
	}
	free(buf);
	return NULL;
}

int psvr2_gaze_start(const char *calibration_path)
{
	if (g_dev == NULL) {
		return -1;
	}
	if (g_gaze_running) {
		return 0;
	}
	snprintf(g_gaze_calibration_path, sizeof(g_gaze_calibration_path), "%s",
	         calibration_path != NULL ? calibration_path : "");

	int ret = libusb_claim_interface(g_dev, GAZE_INTERFACE);
	if (ret != 0) {
		fprintf(stderr, "psvr2: gaze interface busy: %s\n", libusb_error_name(ret));
		return -1;
	}
	g_gaze_claimed = 1;
	/* VD only brings new calibrations back; gaze works without it */
	g_vd_claimed = libusb_claim_interface(g_dev, VD_INTERFACE) == 0;
	if (!g_vd_claimed) {
		fprintf(stderr, "psvr2: VD interface busy; a new eye calibration will not be saved\n");
	}

	pthread_mutex_lock(&g_gaze_lock);
	g_have_gaze = 0;
	g_gaze_packets = 0;
	pthread_mutex_unlock(&g_gaze_lock);

	gaze_upload_calibration();
	gaze_stream(1);

	g_gaze_running = 1;
	if (pthread_create(&g_gaze_thread, NULL, gaze_thread_fn, NULL) != 0) {
		g_gaze_running = 0;
		gaze_stream(0);
		libusb_release_interface(g_dev, GAZE_INTERFACE);
		g_gaze_claimed = 0;
		if (g_vd_claimed) {
			libusb_release_interface(g_dev, VD_INTERFACE);
			g_vd_claimed = 0;
		}
		return -1;
	}
	if (g_vd_claimed && pthread_create(&g_vd_thread, NULL, vd_thread_fn, NULL) != 0) {
		libusb_release_interface(g_dev, VD_INTERFACE);
		g_vd_claimed = 0;
	}
	return 0;
}

void psvr2_gaze_stop(void)
{
	if (!g_gaze_running) {
		return;
	}
	g_gaze_running = 0;
	pthread_join(g_gaze_thread, NULL);
	if (g_vd_claimed) {
		pthread_join(g_vd_thread, NULL);
		libusb_release_interface(g_dev, VD_INTERFACE);
		g_vd_claimed = 0;
	}
	gaze_stream(0);
	libusb_release_interface(g_dev, GAZE_INTERFACE);
	g_gaze_claimed = 0;
}

/* Headset axes to the runtime's, as PSVR2Toolkit and Monado both flip them: x and z negated */
static void gaze_axes(const float in[3], float scale, float out[3])
{
	out[0] = -in[0] * scale;
	out[1] = in[1] * scale;
	out[2] = -in[2] * scale;
}

int psvr2_get_gaze(psvr2_gaze *out)
{
	memset(out, 0, sizeof(*out));
	pthread_mutex_lock(&g_gaze_lock);
	struct gaze_state_record g = g_gaze;
	int have = g_have_gaze;
	double t = g_gaze_time;
	pthread_mutex_unlock(&g_gaze_lock);
	out->age_s = have ? monotonic_s() - t : 1e9;
	if (!have) {
		return 0;
	}
	out->valid = g.combined_dir_valid != 0;
	gaze_axes(g.combined_dir, 1.0f, out->dir);
	out->origin_valid = g.combined_origin_valid != 0;
	gaze_axes(g.combined_origin_mm, 0.001f, out->origin_m);
	for (int e = 0; e < 2; e++) {
		out->eye_valid[e] = g.eye[e].dir_valid != 0;
		gaze_axes(g.eye[e].dir, 1.0f, out->eye_dir[e]);
		gaze_axes(g.eye[e].origin_mm, 0.001f, out->eye_origin_m[e]);
		out->pupil_mm[e] = g.eye[e].pupil_valid ? g.eye[e].pupil_mm : 0.0f;
		out->blink[e] = g.eye[e].blink_valid && g.eye[e].blink;
	}
	/* A direction that is not unit length is not a real sample */
	float n = sqrtf(out->dir[0] * out->dir[0] + out->dir[1] * out->dir[1] + out->dir[2] * out->dir[2]);
	if (n < 0.5f || n > 1.5f) {
		out->valid = 0;
	} else {
		for (int i = 0; i < 3; i++) {
			out->dir[i] /= n;
		}
	}
	out->calibration_id = g.user_calibration_id;
	out->frame = g.frame_counter;
	return 1;
}

unsigned psvr2_gaze_packets(void)
{
	pthread_mutex_lock(&g_gaze_lock);
	unsigned n = g_gaze_packets;
	g_gaze_packets = 0;
	pthread_mutex_unlock(&g_gaze_lock);
	return n;
}

int psvr2_gaze_calibration_command(int mode, float x_mm, float y_mm, float z_mm, int arg)
{
	if (g_dev == NULL) {
		return -1;
	}
	struct gaze_calibration_packet p = {x_mm, y_mm, z_mm, (uint8_t)arg};
	return send_control(REPORT_GAZE_CALIBRATION_SET, (uint8_t)mode, (const uint8_t *)&p, sizeof(p));
}

int psvr2_gaze_calibration_status(int *status, int *result)
{
	if (g_dev == NULL) {
		return -1;
	}
	struct gaze_calibration_packet p;
	uint16_t st = 0;
	if (get_control(REPORT_GAZE_CALIBRATION_GET, 0, (uint8_t *)&p, sizeof(p), &st) != 0) {
		return -1;
	}
	*status = st;
	*result = p.arg;
	return 0;
}

double psvr2_gaze_calibration_saved_age(void)
{
	pthread_mutex_lock(&g_gaze_lock);
	double t = g_gaze_calibration_saved;
	pthread_mutex_unlock(&g_gaze_lock);
	return t > 0 ? monotonic_s() - t : -1.0;
}
