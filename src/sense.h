// PlayStation VR2 Sense controllers over Bluetooth (or USB), through IOKit's HID manager: buttons,
// sticks, triggers, finger sensors, orientation from the controller's own IMU, battery and
// vibration. The protocol (report layouts, CRCs, calibration) follows Monado's pssense driver
// (BSL-1.0, by Jarett Millard and others) and PSVR2Toolkit (MIT); see src/sense.c.
//
// Orientation from here; position from the headset's cameras seeing the LED rings, which
// sense_tracking_leds() lights (sense_track.c finds them). backend_psvr2.m places a controller the
// cameras can't see with an arm model.
#pragma once
#include <stdbool.h>
#include <stdint.h>

enum { SENSE_LEFT = 0, SENSE_RIGHT = 1 };

// Buttons, the same bit for both hands where the hands have the same control.
enum {
    SENSE_FACE_UPPER = 1 << 0,        // triangle (left), circle (right)
    SENSE_FACE_LOWER = 1 << 1,        // square (left), cross (right)
    SENSE_FACE_UPPER_TOUCH = 1 << 2,
    SENSE_FACE_LOWER_TOUCH = 1 << 3,
    SENSE_MENU = 1 << 4,              // create (left), options (right)
    SENSE_PS = 1 << 5,
    SENSE_GRIP = 1 << 6,              // L1 / R1
    SENSE_GRIP_TOUCH = 1 << 7,
    SENSE_TRIGGER_CLICK = 1 << 8,     // L2 / R2 past the middle
    SENSE_TRIGGER_TOUCH = 1 << 9,
    SENSE_STICK_CLICK = 1 << 10,
    SENSE_STICK_TOUCH = 1 << 11,
};

typedef struct {
    bool connected;
    bool bluetooth;
    uint32_t buttons;
    float stick[2];         // -1..1, +x right, +y up
    float trigger;          // 0..1
    float trigger_proximity, grip_proximity;   // 0..1, how close a finger is
    float orientation[4];   // x, y, z, w: the IMU's frame in a gravity-aligned world (+Y up), yaw
                            // wherever it started; turned by sense_imu_to_grip for the grip pose
    float angular_velocity[3];   // rad/s, world
    float battery;          // 0..1, or -1 unknown
    bool charging;
    uint64_t updated_ns;    // CLOCK_MONOTONIC of the newest report
} sense_state;

// The rotation from the IMU's axes to the controller's grip pose (-Z along the controller, +Y up
// out of its top), from Monado's LED model: 50.27 degrees about X.
extern const float sense_imu_to_grip[4];

void sense_start(void);                       // idempotent; finds controllers as they connect
bool sense_get(int hand, sense_state *out);   // false if that controller isn't connected
uint64_t sense_generation(void);              // bumps on every input report
void sense_vibrate(int hand, float seconds, float frequency, float amplitude);
// The infrared tracking LEDs, steadily on or off (off by default: they show in passthrough).
void sense_tracking_leds(bool on);
bool sense_leds_on(int hand);       // the last output report to that controller asked for them on
