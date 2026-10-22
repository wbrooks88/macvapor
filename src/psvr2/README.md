# PS VR2 core

Headset access for the PS VR2 backend (`src/backend_psvr2.m`), taken unchanged from
[psvr2-player-mac](https://github.com/ttkalec/psvr2-player-mac) at commit 9efc5c6:

- `cpsvr2.c`, `cpsvr2.h`: the USB side through libusb (SLAM poses, IMU, status, Fn button,
  distortion calibration, brightness).
- `fusion.c`, `fusion.h`: gyro integration corrected toward the SLAM poses.
- `lut.c`: the lens distortion and chromatic aberration table.

The protocol, distortion formulas and table are Monado's (Boost Software License 1.0); the
rest is psvr2-player-mac's (MIT). See `LICENSE` and `THIRD-PARTY.md` here.
