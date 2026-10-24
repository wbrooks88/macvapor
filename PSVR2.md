# MacVapoR on a PlayStation VR2

**Status: written but not yet run on hardware.** Expect to fix things on the first tries; the
logs below are what to send back.

This branch adds a second backend to alvrxr. In place of streaming to an Apple Vision Pro over
ALVR, it drives a PS VR2 plugged into the Mac through Sony's PC adapter. Games run as in
MacVapoR (NotProton, CrossOver, DXMT), and only the OpenXR runtime they load changes.

What works in this first stage:

- The game's frames on the headset, lens-corrected with the headset's own calibration.
- Head rotation over USB, with a neck model for position.
- Reprojection: each refresh shows the newest game frame turned by however far the head has moved
  since it was rendered, so turning your head stays smooth when the game runs below the panel's
  rate. That matters on a MacBook Air.
- Recentering with the headset's Fn button.
- Passthrough: double-press the Fn button to see the room through the headset's front cameras
  (grayscale) in place of the game, and double-press again to go back. The game keeps running.
- The Sense controllers over Bluetooth: buttons, sticks, triggers, finger sensors, vibration, and
  orientation from each controller's own motion sensor. Games see them as Quest Touch controllers.
- Eye tracking from the headset's own eye tracker: games that ask for eye gaze get it, games that
  render quad views get foveated rendering, and holding Fn for 2 seconds calibrates it (see
  [Eye tracking](#eye-tracking)).

Not yet: the controllers' position. Tracking where they are needs the headset's cameras to watch
their light rings, and no open code does that on a Mac yet. Until then each hand is placed by an
arm model: an elbow at your side and a forearm that turns with the controller. Pointing, aiming,
turning a wheel and pressing buttons work; reaching out to grab things only partly does. Room-scale
position for the head is experimental (see `positional` below).

The headset code (USB protocol, lens correction, head tracking) comes from
[psvr2-player-mac](https://github.com/ttkalec/psvr2-player-mac), which ports it from Monado's
driver; see `src/psvr2/`.

## The quick way: neutron and the installer

No CrossOver is needed. [neutron](https://github.com/lixsldotnet/neutron) is a free Steam Play tool
(arm64 Wine with FEX and DXMT) that runs 64-bit Windows games from Mac Steam, and MacVapoR sets
its games up when it is installed. `tools/install-psvr2.command`, in a folder with this branch as
`macvapor.bundle` and the `wineopenxr-neutron` kit, installs everything else and builds the app:
with Steam logged in once and [Homebrew](https://brew.sh) installed, double-click it (right-click >
Open the first time). Then in MacVapoR press Install, pick PlayStation VR2, and per game: start it
once from Steam, quit, press Set Up, then Play. 32-bit games don't run under neutron.

The kit holds wineopenxr built for neutron's Wine 11.19 (its Windows half) and the sources and
headers of its Mac half, which `tools/neutron-vr-setup.sh` builds against neutron's own ntdll.so
the first time a game is set up. `build-psvr2.sh` bundles it when `NEUTRON_KIT` (or
`./wineopenxr-neutron`) has it.

## What you need

- An Apple silicon Mac with macOS 15 or later.
- Sony's PS VR2 PC adapter with its power brick, and a USB-C to DisplayPort cable (DP 1.4).
- MacVapoR installed and a game set up with it, as in the main README. You do not need a Vision
  Pro for this; skip the ALVR steps.
- Command Line Tools (`xcode-select --install`), Homebrew, and:

      brew install libusb vulkan-headers
      git clone https://github.com/KhronosGroup/OpenXR-SDK ~/Source/OpenXR-SDK

## Hook the headset up

1. Adapter power brick into the wall. DisplayPort cable from the adapter to the Mac. The adapter's
   USB cable to the Mac's other port. The headset into the adapter.
2. Turn the headset on. A display called "PS VR2" should appear in System Settings > Displays.
3. Set it to **extend** the desktop (not mirror), at **90 Hz**, and pick **2000x1020** or
   **4000x2040**.

## Pair the controllers

Once per controller: hold its PS button and Create (left) or Options (right) until its light
blinks, then pick it under System Settings > Bluetooth. After that, pressing PS wakes it and it
reconnects.

When a game starts, each controller points where your head faces, so hold them pointing forward.
Press a controller's PS button to point it forward again whenever it drifts; the headset's Fn
button recenters your view and both controllers at once.

## Build

    ./build-psvr2.sh

This makes `out/libpsvr2xr.dylib` (the runtime) and `out/psvr2xr-selftest`. If MacVapoR is in
Applications (from its release DMG), it also makes `out/MacVapoR.app`: the same app with the PS VR2
added, so you don't need to build ALVR or NotProton. Set `MACVAPOR_APP` if your MacVapoR.app is
somewhere else. `macvapor/make-release.sh` puts the PS VR2 runtime in the apps it builds as well.

## The MacVapoR app

Drag `out/MacVapoR.app` to Applications in place of the old one, open it and press **Reinstall**.
Then:

1. Under **Runtime**, set **Show games on** to **PlayStation VR2**. This switches every set-up game
   at once; choose Apple Vision Pro to go back.
2. The **PlayStation VR2** section shows whether the headset is on USB and its display is extended,
   and at what rate. It runs the self-test (Run, Run at half rate, Stop) and has the settings
   below as sliders and switches.
3. Under **Games**, set a game up as usual. Set-up games get a **Play** button, which starts the
   game through Steam, so it runs with NotProton.

**Runtime Log** in the toolbar opens the PS VR2 log while the PS VR2 is chosen. The steps below do
the same things from Terminal.

## Step 1: the self-test

    out/psvr2xr-selftest

Put the headset on. You should stand in a grey room with a 1 m floor grid and four pillars 3 m
away: red ahead, green right, blue behind, yellow left. Check:

- Grid lines look straight all the way to the edges of the view. Curved lines mean the lens
  correction is off.
- Turning your head moves the view the right way, and the pillars stay where they are.
- Fn recenters you to face the red pillar.
- With the controllers on, a cyan ball (left) and a magenta ball (right) sit near your hands, with a
  small white one 20 cm ahead of each. Point a controller around: the white ball should follow
  where it points. Pull a trigger: its ball lights up and the controller buzzes. If a controller
  points the wrong way after you press its PS button, send me the log.
- Both eyes fuse into one 3D picture. If it looks doubled or uncomfortable, close it right away
  and send me the log.

Then try `out/psvr2xr-selftest --slow 2`, which renders only every other frame, like a game at
45 fps. Head turns should still look smooth; the pillars may look slightly doubled when you move.

## Step 2: a game

    tools/psvr2-runtime.sh on

Games set up by MacVapoR now load the PS VR2 runtime. Launch one from NotProton with the headset
on (or choose PlayStation VR2 in the MacVapoR app and press Play). `tools/psvr2-runtime.sh off` switches back to the Vision Pro runtime.

Start with a light, seated game that works with a gamepad. Games that need motion controllers
will load but can't be played yet.

## Settings

`~/Library/Application Support/psvr2xr/config.json` is written with these defaults on the first
run, and read when a game starts:

| Key | Default | What it does |
| --- | --- | --- |
| `render_scale` | 0.75 | Eye image size as a share of 2000x1906. Lower it if the game stutters. |
| `eye_height` | 1.6 | Metres from the floor to your eyes, for the neck model. |
| `reprojection` | true | Turns each frame with your head. Turn it off only to compare. |
| `chromatic` | true | Per-colour lens correction. |
| `lookahead_ms` | 0 | Adds to the head prediction. Raise it if the world seems to lag your head when you turn. |
| `controllers` | true | Use the Sense controllers over Bluetooth. |
| `eye_tracking` | true | Use the headset's eye tracker. |
| `show_gaze` | false | Draw a yellow ring where you look, to check the eye tracker. |
| `foveated` | true | Offer foveated rendering (quad views) to games that support it. |
| `foveated_area_deg` | 40 | Width and height of the sharp area that follows your eyes, in degrees. |
| `foveated_outer_scale` | 0.5 | Resolution of the rest of the view, as a share of `render_scale`'s. |
| `foveated_inset_scale` | 1.0 | Resolution of the sharp area, as a share of the panel's at its centre. |
| `positional` | false | Experimental: use the headset's own position tracking instead of the neck model. The axes are a guess; if moving your head moves the world the wrong way, turn it back off and send me the log. |

## Eye tracking

The PS VR2 tracks both eyes with its own cameras and works out where you look itself; the Mac only
reads the result over USB, about 120 times a second. No jailbreak is needed for this (PSVR2Toolkit
needs one only for the headset's outside cameras).

**Calibrate once.** In MacVapoR press **Calibrate eyes** (or run
`out/psvr2xr-selftest --calibrate-eyes`), or hold the headset's **Fn** button for 2 seconds during
any game. Look at each dot until the next one appears: 8 on black, then 5 on light grey, about a
minute. The headset reports the result, which is saved to
`~/Library/Application Support/psvr2xr/eye_calibration.bin` and sent to the headset every time a game
starts. A calibration you made on a PS5 may already be in the headset; the log says which one it
holds (`user calibration` on the `eyes:` line, 0 for none).

**Check it.** **Foveated test** (or `out/psvr2xr-selftest --foveated`) shows the test room with a
yellow ring where you look. If the ring sits off to one side of what you look at, calibrate again.
`show_gaze` puts the ring in games too.

**In games.** Two things use it:

- Eye gaze (`XR_EXT_eye_gaze_interaction`): games and apps that read where you look (aiming,
  menus, social apps' eyes) get it from the headset.
- Foveated rendering (`XR_VARJO_quad_views` with `XR_VARJO_foveated_rendering`): the game renders
  the whole view at a low resolution and a small sharp area where you look. With the defaults that
  is about a third of the pixels of a normal frame, which matters most on a MacBook Air. Only games
  that support quad views can do this (DCS World, for example, and some Unreal Engine games); a
  game that doesn't renders as before.
  The log says `session uses quad views` when a game takes it. Most SteamVR games, which run
  through OpenComposite, don't.

If the ring jumps around or the log says `NOT tracked` while
your eyes are open, adjust the headset so it sits level and the IPD dial matches your eyes.

## Logs

`~/Library/Logs/psvr2xr.log` gets a line every 5 seconds: the game's frame rate, how many frames
the panel composed and repeated, whether the head is tracked, the IPD dial, and raw tracking
numbers. With the eye tracker on, an `eyes:` line gives its sample rate, where you look, pupil size,
blinks and the calibration it holds. Each connected controller gets a line too (battery, buttons, stick, trigger, where it
points), and its calibration is logged when it connects. Send it along with what you saw.

## How it works

`src/backend_psvr2.m` implements the interface `runtime.m` uses (`src/alvrxr.h`), as `backend.m`
does for ALVR. A borderless window covers the headset's display, and a display link on that
display composes the newest game frame at the panel's rate (`src/psvr2_compose.metal`). The
runtime reports the field of view the lens correction expects, so the game renders exactly the
area the lenses show. Each submitted frame is copied off the game's swapchain on the game's own
queue, so the compositor can show it again on refreshes the game misses.

`src/sense.c` talks to the Sense controllers through macOS's HID manager. Reading their
calibration switches them into full reporting; their reports and vibration commands follow
Monado's PlayStation Sense driver and PSVR2Toolkit. Orientation is fused from each controller's
gyro and accelerometer, which holds pitch and roll steady; yaw drifts slowly, which the PS button
fixes.
