// PlayStation VR2 Sense controllers through IOKit's HID manager; see sense.h.
//
// The protocol is Monado's (src/xrt/drivers/pssense, BSL-1.0, Copyright 2023 Collabora, Ltd. and
// Jarett Millard), checked against PSVR2Toolkit's sense_controller.h (MIT, Copyright 2026 Bnuuy
// Solutions):
//   - Reading feature report 0x05 (the IMU calibration, in two 64-byte parts) switches a
//     controller from its compatibility mode into full mode, where it sends report 0x31 over
//     Bluetooth (78 bytes) or 0x01 over USB (64 bytes).
//   - Bluetooth reports end in a CRC-32 of a seed byte followed by the report: 0xa1 for input,
//     0xa2 for output, 0xa3 for features.
//   - Output report 0x31 carries 38 bytes of settings, vibration among them; Monado and Sony's
//     driver send one every 32/3000 s (the PCM haptics rate), which this does too.
// Orientation is fused here from the gyro and accelerometer (Mahony's filter: the gyro, pulled
// toward the accelerometer's gravity), as Monado's m_imu_3dof does, so it holds pitch and roll;
// yaw drifts slowly and is set by recentering.

#include "sense.h"

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/hid/IOHIDManager.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

void axr_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

enum { SONY = 0x054C, PID_LEFT = 0x0E45, PID_RIGHT = 0x0E46 };
enum { BT_INPUT_ID = 0x31, BT_INPUT_LEN = 78, USB_INPUT_ID = 0x01, USB_INPUT_LEN = 64 };
enum { BT_OUTPUT_ID = 0x31, BT_OUTPUT_LEN = 78, USB_OUTPUT_LEN = 39, OUTPUT_TAG = 0x10 };
enum { CALIB_ID = 0x05, FEATURE_LEN = 64, CALIB_LEN = 116 };
enum { LED_PRESCAN = 1, LED_ALL_OFF = 5 };
// The tracking LEDs lit steadily, as psvr2-controller-ir-lights-enable (MIT) found works and checked
// with a camera: PRESCAN pulses of period 42 (42 x 150 ticks of a third of a microsecond, 2.1 ms)
// every 2 ms, so each overlaps the next, starting 50 ms after the controller's clock now reads.
enum { LED_STEADY_PERIOD = 42, LED_STEADY_CYCLE = 2000000 * 3, LED_LEAD_TICKS = 50000 * 3 };
enum { VIBRATE_ENABLE = 0x03, VIBRATE_HIGH_120HZ = 0x00, VIBRATE_LOW_60HZ = 0x20, VIBRATE_CLASSIC = 0x40 };
static const int64_t OUTPUT_PERIOD_NS = 10666666;   // 32 samples at 3000 Hz

// The IMU sits turned 50.27 degrees about X from the controller's LED model (Monado's
// pssense_led_model.h), whose -Z runs along the controller.
const float sense_imu_to_grip[4] = {0.42492f, 0, 0, 0.90523f};   // sin, cos of 25.135 degrees

typedef struct {
    IOHIDDeviceRef dev;
    bool bluetooth, calibrated;
    uint8_t report[128];
    // Calibration, Monado's parse_pssense_calibration.
    float gyro_bias[3], gyro_scale[3], accel_bias[3], accel_scale[3];
    // Fusion.
    bool fused;
    float q[4];             // x, y, z, w
    float integral[3];
    uint32_t imu_ticks;
    int64_t first_ns;
    // Output.
    uint8_t out_seq, out_counter;
    uint8_t vib_amplitude, vib_mode;
    // The tracking LEDs: the controller's clock as last read (Monado's device_timestamp_ticks,
    // thirds of a microsecond) and when; the LED settings' sequence number, bumped on each change;
    // and the pulses' start, fixed when they were turned on.
    uint32_t device_ticks;
    int64_t device_ticks_ns;
    bool leds_on;
    uint8_t led_seq;
    uint32_t led_start;
    int64_t vib_end_ns;
    bool vib_dirty;
    unsigned crc_errors, reports;
    sense_state state;
} controller;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static controller g_ctl[2];
static uint64_t g_generation;
static bool g_started;
static bool g_leds_wanted;

static int64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

static uint32_t crc32_seeded(uint8_t seed, const uint8_t *p, size_t len) {
    uint32_t crc = 0xffffffff;
    for (size_t i = 0; i <= len; i++) {
        crc ^= i == 0 ? seed : p[i - 1];
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320 : 0);
    }
    return crc ^ 0xffffffff;
}

static int16_t le16(const uint8_t *p) { return (int16_t)(p[0] | p[1] << 8); }
static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

// ----------------------------------------------------------------------------------- calibration

static bool read_calibration(controller *c) {
    uint8_t blob[CALIB_LEN] = {0};
    bool have[2] = {false, false};
    for (int tries = 0; tries < 8 && !(have[0] && have[1]); tries++) {
        uint8_t buf[FEATURE_LEN] = {0};
        CFIndex len = sizeof buf;
        IOReturn r = IOHIDDeviceGetReport(c->dev, kIOHIDReportTypeFeature, CALIB_ID, buf, &len);
        if (r != kIOReturnSuccess) {
            axr_log("sense: reading calibration failed (0x%x)", r);
            return false;
        }
        if (len == FEATURE_LEN - 1 && buf[0] != CALIB_ID) {   // delivered without its report ID
            memmove(buf + 1, buf, FEATURE_LEN - 1);
            buf[0] = CALIB_ID;
            len = FEATURE_LEN;
        }
        if (len != FEATURE_LEN) { axr_log("sense: calibration report is %ld bytes", (long)len); continue; }
        if (c->bluetooth && crc32_seeded(0xa3, buf, FEATURE_LEN - 4) != le32(buf + FEATURE_LEN - 4)) {
            axr_log("sense: calibration CRC mismatch, reading again");
            continue;
        }
        if (buf[1] == 0x00) { memcpy(blob, buf + 2, 58); have[0] = true; }
        else if (buf[1] == 0x81) { memcpy(blob + 58, buf + 2, 58); have[1] = true; }
    }
    if (!(have[0] && have[1])) { axr_log("sense: incomplete calibration"); return false; }

    // Offsets in the 116-byte blob, from Monado's struct pssense_calibration_data.
    float accel_plus[3] = {le16(blob + 0x00), le16(blob + 0x0E), le16(blob + 0x1C)};
    float accel_minus[3] = {le16(blob + 0x06), le16(blob + 0x14), le16(blob + 0x22)};
    float gyro_plus[3] = {le16(blob + 0x46), le16(blob + 0x2A), le16(blob + 0x38)};
    float gyro_minus[3] = {le16(blob + 0x4C), le16(blob + 0x30), le16(blob + 0x3E)};
    float bias[3] = {le16(blob + 0x40), le16(blob + 0x42), le16(blob + 0x44)};
    const float rad_per_sec_ref = (float)M_PI * 3;   // Monado's reference rate
    for (int i = 0; i < 3; i++) {
        float span = (fabsf(gyro_plus[i] - bias[i]) + fabsf(gyro_minus[i] - bias[i])) / 2;
        c->gyro_bias[i] = bias[i];
        c->gyro_scale[i] = span > 1 ? rad_per_sec_ref / span : 0;
        c->accel_bias[i] = (accel_plus[i] + accel_minus[i]) / 2;
        float half = accel_plus[i] - c->accel_bias[i];
        c->accel_scale[i] = fabsf(half) > 1 ? 9.80665f / half : 0;
    }
    axr_log("sense: %s calibration: gyro bias %.0f %.0f %.0f, scale %.3g %.3g %.3g rad/s per unit; "
            "accel bias %.0f %.0f %.0f, scale %.3g %.3g %.3g m/s2 per unit",
            c == &g_ctl[SENSE_LEFT] ? "left" : "right", c->gyro_bias[0], c->gyro_bias[1], c->gyro_bias[2],
            c->gyro_scale[0], c->gyro_scale[1], c->gyro_scale[2], c->accel_bias[0], c->accel_bias[1],
            c->accel_bias[2], c->accel_scale[0], c->accel_scale[1], c->accel_scale[2]);
    return c->gyro_scale[0] && c->gyro_scale[1] && c->gyro_scale[2] && c->accel_scale[0] && c->accel_scale[1] &&
           c->accel_scale[2];
}

// ---------------------------------------------------------------------------------------- fusion

static void qmul(const float a[4], const float b[4], float o[4]) {
    float r[4] = {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
                  a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
                  a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
                  a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
    memcpy(o, r, sizeof r);
}

static void qrot(const float q[4], const float v[3], float o[3]) {
    float tx = 2 * (q[1] * v[2] - q[2] * v[1]), ty = 2 * (q[2] * v[0] - q[0] * v[2]), tz = 2 * (q[0] * v[1] - q[1] * v[0]);
    o[0] = v[0] + q[3] * tx + (q[1] * tz - q[2] * ty);
    o[1] = v[1] + q[3] * ty + (q[2] * tx - q[0] * tz);
    o[2] = v[2] + q[3] * tz + (q[0] * ty - q[1] * tx);
}

static void qnormalize(float q[4]) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0) for (int i = 0; i < 4; i++) q[i] /= n;
}

// gyro in rad/s and accel in m/s2, both in the IMU's axes (+Y up when the controller is level).
static void fuse(controller *c, const float gyro[3], const float accel[3], float dt, int64_t t) {
    float an = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
    if (!c->fused) {
        if (an < 4) return;
        // Start level with gravity, facing wherever: the shortest turn from the measured up to +Y.
        float a[3] = {accel[0] / an, accel[1] / an, accel[2] / an};
        float q[4] = {-a[2], 0, a[0], 1 + a[1]};   // a x +Y, 1 + a.+Y: rotates a onto +Y
        if (q[3] < 1e-4f) { q[0] = 1; q[1] = q[2] = q[3] = 0; }
        qnormalize(q);
        memcpy(c->q, q, sizeof q);
        memset(c->integral, 0, sizeof c->integral);
        c->first_ns = t;
        c->fused = true;
        return;
    }
    float w[3] = {gyro[0], gyro[1], gyro[2]};
    // Pull toward gravity only while the accelerometer reads close to 1 g, so swings don't tilt it.
    if (an > 0.6f * 9.80665f && an < 1.4f * 9.80665f) {
        float a[3] = {accel[0] / an, accel[1] / an, accel[2] / an};
        float inv[4] = {-c->q[0], -c->q[1], -c->q[2], c->q[3]}, up[3] = {0, 1, 0}, v[3];
        qrot(inv, up, v);   // where the filter thinks up is, in the IMU's axes
        float e[3] = {a[1] * v[2] - a[2] * v[1], a[2] * v[0] - a[0] * v[2], a[0] * v[1] - a[1] * v[0]};
        float kp = t - c->first_ns < 1500000000 ? 10.0f : 1.0f, ki = 0.02f;
        for (int i = 0; i < 3; i++) {
            c->integral[i] += ki * e[i] * dt;
            w[i] += kp * e[i] + c->integral[i];
        }
    }
    float half = dt / 2;
    float d[4] = {w[0] * half, w[1] * half, w[2] * half, 1};
    qmul(c->q, d, c->q);
    qnormalize(c->q);
}

// ----------------------------------------------------------------------------------------- input

static void handle_report(controller *c, int hand, const uint8_t *r, CFIndex len) {
    int base;
    if (len == BT_INPUT_LEN && r[0] == BT_INPUT_ID) {
        base = 2;
        c->reports++;
        if (crc32_seeded(0xa1, r, BT_INPUT_LEN - 4) != le32(r + BT_INPUT_LEN - 4)) {
            if (c->crc_errors++ % 1000 == 0) axr_log("sense: input CRC mismatches: %u of %u", c->crc_errors, c->reports);
            return;
        }
    } else if (len == USB_INPUT_LEN && r[0] == USB_INPUT_ID && !c->bluetooth) {
        base = 1;
    } else {
        return;   // the compatibility mode's short reports, until calibration has been read
    }
    const uint8_t *p = r + base;   // Monado's struct pssense_input_report_common
    int64_t t = now_ns();
    uint32_t b = 0;
    uint8_t b0 = p[7], b1 = p[8], b2 = p[9];
    if (b1 & 16) b |= SENSE_PS;
    if (b1 & 128) b |= SENSE_TRIGGER_TOUCH;
    if (b2 & 1) b |= SENSE_FACE_UPPER_TOUCH;
    if (b2 & 2) b |= SENSE_FACE_LOWER_TOUCH;
    if (b2 & 4) b |= SENSE_STICK_TOUCH;
    if (b2 & 8) b |= SENSE_GRIP_TOUCH;
    if (hand == SENSE_LEFT) {
        if (b0 & 8) b |= SENSE_FACE_UPPER;      // triangle
        if (b0 & 1) b |= SENSE_FACE_LOWER;      // square
        if (b1 & 1) b |= SENSE_MENU;            // create
        if (b0 & 16) b |= SENSE_GRIP;           // L1
        if (b0 & 64) b |= SENSE_TRIGGER_CLICK;
        if (b1 & 4) b |= SENSE_STICK_CLICK;
    } else {
        if (b0 & 4) b |= SENSE_FACE_UPPER;      // circle
        if (b0 & 2) b |= SENSE_FACE_LOWER;      // cross
        if (b1 & 2) b |= SENSE_MENU;            // options
        if (b0 & 32) b |= SENSE_GRIP;           // R1
        if (b0 & 128) b |= SENSE_TRIGGER_CLICK;
        if (b1 & 8) b |= SENSE_STICK_CLICK;
    }

    pthread_mutex_lock(&g_lock);
    sense_state *s = &c->state;
    s->buttons = b;
    s->stick[0] = (p[0] - 128) / 128.0f;
    s->stick[1] = (p[1] - 128) / -128.0f;
    s->trigger = p[2] / 255.0f;
    s->trigger_proximity = p[3] / 255.0f;
    s->grip_proximity = p[4] / 255.0f;
    uint8_t bat = p[41];
    int charge_state = bat >> 4;
    s->battery = charge_state == 2 ? 1.0f : charge_state <= 1 ? fminf(1.0f, (bat & 15) * 0.1f + 0.05f) : -1;
    s->charging = charge_state == 1 || charge_state == 2;

    c->device_ticks = le32(p + 47);
    c->device_ticks_ns = t;
    uint32_t ticks = le32(p + 27);   // thirds of a microsecond
    if (c->calibrated) {
        float gyro[3], accel[3];
        for (int i = 0; i < 3; i++) {
            gyro[i] = (le16(p + 15 + 2 * i) - c->gyro_bias[i]) * c->gyro_scale[i];
            accel[i] = (le16(p + 21 + 2 * i) - c->accel_bias[i]) * c->accel_scale[i];
        }
        float dt = c->imu_ticks ? (uint32_t)(ticks - c->imu_ticks) / 3e6f : 0;
        if (dt <= 0 || dt > 0.1f) dt = 0;
        fuse(c, gyro, accel, dt, t);
        memcpy(s->orientation, c->q, sizeof c->q);
        qrot(c->q, gyro, s->angular_velocity);
    }
    c->imu_ticks = ticks;
    s->updated_ns = t;
    g_generation++;
    pthread_mutex_unlock(&g_lock);
}

static void input_callback(void *ctx, IOReturn result, void *sender, IOHIDReportType type, uint32_t id,
                           uint8_t *report, CFIndex len) {
    (void)sender; (void)type; (void)id;
    if (result != kIOReturnSuccess) return;
    int hand = (int)(intptr_t)ctx;
    uint8_t fixed[BT_INPUT_LEN];
    if (len == BT_INPUT_LEN - 1 && g_ctl[hand].bluetooth) {   // delivered without its report ID
        fixed[0] = BT_INPUT_ID;
        memcpy(fixed + 1, report, len);
        report = fixed;
        len = BT_INPUT_LEN;
    }
    handle_report(&g_ctl[hand], hand, report, len);
}

// ---------------------------------------------------------------------------------------- output

// The 38 bytes of settings that both report forms carry (Monado's struct pssense_output_settings).
static void fill_settings(controller *c, uint8_t *s, int64_t t) {
    if (c->vib_amplitude && t >= c->vib_end_ns) { c->vib_amplitude = 0; c->vib_dirty = true; }
    if (c->vib_dirty) {
        s[0] = VIBRATE_ENABLE | c->vib_mode;
        s[2] = c->vib_amplitude;
        c->vib_dirty = c->vib_amplitude != 0;   // keep restating it while it runs, then once at 0
    }
    put32(s + 15, (uint32_t)(t / 1000));        // host time of sending, microseconds
    // The tracking LEDs (Monado's struct pssense_led_settings at byte 19): on steadily while the
    // headset's cameras look for them, off otherwise.
    bool on = g_leds_wanted && c->device_ticks_ns;
    if (on != c->leds_on) {
        c->leds_on = on;
        c->led_seq++;
        if (on) c->led_start = c->device_ticks + (uint32_t)((t - c->device_ticks_ns) * 3 / 1000) + LED_LEAD_TICKS;
    }
    s[19] = on ? LED_PRESCAN : LED_ALL_OFF;
    s[20] = c->led_seq;
    if (on) {
        s[21] = LED_STEADY_PERIOD;
        put32(s + 22, c->led_start);
        put32(s + 26, LED_STEADY_CYCLE);
        memset(s + 30, 0xff, 4);                // every LED of the four groups
    }
}

// The next output report, in r; returns its length. Called under g_lock.
static CFIndex build_output(controller *c, uint8_t r[BT_OUTPUT_LEN]) {
    int64_t t = now_ns();
    memset(r, 0, BT_OUTPUT_LEN);
    CFIndex len;
    if (c->bluetooth) {
        r[0] = BT_OUTPUT_ID;
        r[1] = (uint8_t)(c->out_seq << 4);       // low bits 0: the PS5 report form
        r[2] = OUTPUT_TAG;
        fill_settings(c, r + 3, t);
        r[41] = c->out_counter++;
        put32(r + 74, crc32_seeded(0xa2, r, BT_OUTPUT_LEN - 4));
        len = BT_OUTPUT_LEN;
    } else {
        r[0] = (uint8_t)(c->out_seq << 4 | 2);   // Monado: any non-zero report ID works over USB
        fill_settings(c, r + 1, t);
        len = USB_OUTPUT_LEN;
    }
    c->out_seq = (c->out_seq + 1) & 15;
    return len;
}

static void *output_thread(void *arg) {
    (void)arg;
    int64_t next = now_ns();
    for (;;) {
        for (int h = 0; h < 2; h++) {
            uint8_t r[BT_OUTPUT_LEN];
            CFIndex len = 0;
            IOHIDDeviceRef dev = NULL;
            pthread_mutex_lock(&g_lock);
            controller *c = &g_ctl[h];
            if (c->dev && c->calibrated) {
                dev = (IOHIDDeviceRef)CFRetain(c->dev);
                len = build_output(c, r);
            }
            pthread_mutex_unlock(&g_lock);
            // Sent outside the lock: a Bluetooth write can take a while.
            if (dev) {
                IOReturn res = IOHIDDeviceSetReport(dev, kIOHIDReportTypeOutput, r[0], r, len);
                if (res != kIOReturnSuccess) {
                    static unsigned failures;
                    if (failures++ % 500 == 0) axr_log("sense: output report failed (0x%x)", res);
                }
                CFRelease(dev);
            }
        }
        next += OUTPUT_PERIOD_NS;
        int64_t wait = next - now_ns();
        if (wait < -OUTPUT_PERIOD_NS) { next = now_ns(); wait = 0; }
        if (wait > 0) {
            struct timespec ts = {(time_t)(wait / 1000000000), (long)(wait % 1000000000)};
            nanosleep(&ts, NULL);
        }
    }
    return NULL;
}

// ------------------------------------------------------------------------------------ discovery

static int hand_of(IOHIDDeviceRef dev) {
    CFNumberRef n = IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDProductIDKey));
    int pid = 0;
    if (n) CFNumberGetValue(n, kCFNumberIntType, &pid);
    return pid == PID_LEFT ? SENSE_LEFT : pid == PID_RIGHT ? SENSE_RIGHT : -1;
}

static void matched(void *ctx, IOReturn result, void *sender, IOHIDDeviceRef dev) {
    (void)ctx; (void)result; (void)sender;
    int hand = hand_of(dev);
    if (hand < 0) return;
    controller *c = &g_ctl[hand];
    pthread_mutex_lock(&g_lock);
    bool busy = c->dev != NULL;
    pthread_mutex_unlock(&g_lock);
    if (busy) return;
    if (IOHIDDeviceOpen(dev, kIOHIDOptionsTypeNone) != kIOReturnSuccess) {
        axr_log("sense: could not open the %s controller", hand == SENSE_LEFT ? "left" : "right");
        return;
    }
    CFStringRef transport = IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDTransportKey));
    bool bt = transport && CFStringFind(transport, CFSTR("Bluetooth"), kCFCompareCaseInsensitive).location != kCFNotFound;
    controller fresh = {.dev = dev, .bluetooth = bt};
    pthread_mutex_lock(&g_lock);
    *c = fresh;
    pthread_mutex_unlock(&g_lock);
    bool calibrated = read_calibration(c);   // also switches it into full mode
    pthread_mutex_lock(&g_lock);
    c->calibrated = calibrated;
    c->state.connected = true;
    c->state.bluetooth = bt;
    c->state.battery = -1;
    c->state.orientation[3] = 1;
    pthread_mutex_unlock(&g_lock);
    CFRetain(dev);
    IOHIDDeviceRegisterInputReportCallback(dev, c->report, sizeof c->report, input_callback, (void *)(intptr_t)hand);
    axr_log("sense: %s controller connected over %s%s", hand == SENSE_LEFT ? "left" : "right", bt ? "Bluetooth" : "USB",
            calibrated ? "" : ", but its calibration could not be read, so it will not report; turn it off and on");
}

static void removed(void *ctx, IOReturn result, void *sender, IOHIDDeviceRef dev) {
    (void)ctx; (void)result; (void)sender;
    for (int h = 0; h < 2; h++) {
        pthread_mutex_lock(&g_lock);
        bool mine = g_ctl[h].dev == dev;
        if (mine) {
            g_ctl[h].dev = NULL;
            g_ctl[h].calibrated = false;
            g_ctl[h].state.connected = false;
            g_generation++;
        }
        pthread_mutex_unlock(&g_lock);
        if (mine) {
            axr_log("sense: %s controller disconnected", h == SENSE_LEFT ? "left" : "right");
            CFRelease(dev);
        }
    }
}

static CFDictionaryRef match_pid(int pid) {
    int vid = SONY;
    CFNumberRef v = CFNumberCreate(NULL, kCFNumberIntType, &vid), p = CFNumberCreate(NULL, kCFNumberIntType, &pid);
    const void *keys[] = {CFSTR(kIOHIDVendorIDKey), CFSTR(kIOHIDProductIDKey)}, *vals[] = {v, p};
    CFDictionaryRef d = CFDictionaryCreate(NULL, keys, vals, 2, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFRelease(v);
    CFRelease(p);
    return d;
}

static void *hid_thread(void *arg) {
    (void)arg;
    IOHIDManagerRef m = IOHIDManagerCreate(NULL, kIOHIDOptionsTypeNone);
    CFDictionaryRef l = match_pid(PID_LEFT), r = match_pid(PID_RIGHT);
    const void *both[] = {l, r};
    CFArrayRef list = CFArrayCreate(NULL, both, 2, &kCFTypeArrayCallBacks);
    IOHIDManagerSetDeviceMatchingMultiple(m, list);
    CFRelease(list);
    CFRelease(l);
    CFRelease(r);
    IOHIDManagerRegisterDeviceMatchingCallback(m, matched, NULL);
    IOHIDManagerRegisterDeviceRemovalCallback(m, removed, NULL);
    IOHIDManagerScheduleWithRunLoop(m, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    IOReturn res = IOHIDManagerOpen(m, kIOHIDOptionsTypeNone);
    if (res != kIOReturnSuccess) axr_log("sense: the HID manager would not open (0x%x)", res);
    CFRunLoopRun();
    return NULL;
}

// ------------------------------------------------------------------------------------------- API

void sense_start(void) {
    pthread_mutex_lock(&g_lock);
    bool started = g_started;
    g_started = true;
    pthread_mutex_unlock(&g_lock);
    if (started) return;
    pthread_t t;
    pthread_create(&t, NULL, hid_thread, NULL);
    pthread_detach(t);
    pthread_create(&t, NULL, output_thread, NULL);
    pthread_detach(t);
}

bool sense_get(int hand, sense_state *out) {
    if (hand != SENSE_LEFT && hand != SENSE_RIGHT) return false;
    pthread_mutex_lock(&g_lock);
    *out = g_ctl[hand].state;
    pthread_mutex_unlock(&g_lock);
    return out->connected;
}

uint64_t sense_generation(void) {
    pthread_mutex_lock(&g_lock);
    uint64_t g = g_generation;
    pthread_mutex_unlock(&g_lock);
    return g;
}

void sense_tracking_leds(bool on) {
    pthread_mutex_lock(&g_lock);
    g_leds_wanted = on;
    pthread_mutex_unlock(&g_lock);
}

bool sense_leds_on(int hand) {
    if (hand != SENSE_LEFT && hand != SENSE_RIGHT) return false;
    pthread_mutex_lock(&g_lock);
    bool on = g_ctl[hand].leds_on;
    pthread_mutex_unlock(&g_lock);
    return on;
}

void sense_vibrate(int hand, float seconds, float frequency, float amplitude) {
    if (hand != SENSE_LEFT && hand != SENSE_RIGHT) return;
    uint8_t amp = (uint8_t)fminf(255, fmaxf(0, amplitude * 255));
    // Monado's choice of the controller's vibration modes by frequency.
    uint8_t mode = frequency <= 70 ? VIBRATE_LOW_60HZ : frequency >= 110 ? VIBRATE_HIGH_120HZ : VIBRATE_CLASSIC;
    pthread_mutex_lock(&g_lock);
    controller *c = &g_ctl[hand];
    if (amp != c->vib_amplitude || mode != c->vib_mode || amp) {
        c->vib_amplitude = amp;
        c->vib_mode = mode;
        c->vib_end_ns = now_ns() + (int64_t)(fminf(seconds, 10) * 1e9f);
        c->vib_dirty = true;
    }
    pthread_mutex_unlock(&g_lock);
}
