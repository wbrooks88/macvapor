#!/bin/sh
# Builds the PS VR2 variant of the runtime and its self-test into out/ (see PSVR2.md):
#   out/libpsvr2xr.dylib, out/psvr2xr.json   the OpenXR runtime, runtime.m on backend_psvr2.m
#   out/psvr2xr-selftest                      the backend alone, showing a test room
#   out/MacVapoR.app                          MacVapoR with the PS VR2 in it, made from the one in
#                                             /Applications (see the end); NO_APP=1 skips it
# No ALVR is needed. Needs the Command Line Tools and Homebrew's libusb and vulkan-headers, and
# the OpenXR headers: OPENXR points at a clone of KhronosGroup/OpenXR-SDK (include/) or a built
# OpenXR-SDK-Source (build/include/).
set -eu
here=$(cd "$(dirname "$0")" && pwd)
OPENXR=${OPENXR:-$HOME/Source/OpenXR-SDK}
MIN_VER=${MIN_VER:-15.0}
export MACOSX_DEPLOYMENT_TARGET=$MIN_VER
BREW=$(brew --prefix)
out="$here/out"
mkdir -p "$out" "$here/build"

xr_include=
for d in "$OPENXR/include" "$OPENXR/build/include"; do
    [ -f "$d/openxr/openxr.h" ] && { xr_include=$d; break; }
done
[ -n "$xr_include" ] || {
    echo "no OpenXR headers under $OPENXR; git clone https://github.com/KhronosGroup/OpenXR-SDK ~/Source/OpenXR-SDK" >&2
    exit 1
}
[ -f "$BREW/include/vulkan/vulkan.h" ] || { echo "brew install vulkan-headers" >&2; exit 1; }
[ -f "$BREW/include/libusb-1.0/libusb.h" ] || { echo "brew install libusb" >&2; exit 1; }

# libusb linked in statically when Homebrew has the archive, so the runtime loads inside a game's
# Wine process without reaching for a Homebrew dylib.
if [ -f "$BREW/lib/libusb-1.0.a" ]; then
    usb="$BREW/lib/libusb-1.0.a -framework IOKit -framework CoreFoundation -framework Security"
else
    usb="-L$BREW/lib -lusb-1.0"
fi

{ printf 'static const char psvr2_compose_metal[] =\n'; sed 's/\\/\\\\/g; s/"/\\"/g; s/^/"/; s/$/\\n"/' "$here/src/psvr2_compose.metal"; printf ';\n'; } \
    > "$here/build/psvr2_compose_metal.h"

cflags="-mmacosx-version-min=$MIN_VER -O2 -g -Wall -Wno-unused-function -Wno-deprecated-declarations"
for f in cpsvr2 fusion lut; do
    clang $cflags -I "$BREW/include/libusb-1.0" -c "$here/src/psvr2/$f.c" -o "$here/build/$f.o"
done
clang $cflags -c "$here/src/sense.c" -o "$here/build/sense.o"   # the Sense controllers
core="$here/build/cpsvr2.o $here/build/fusion.o $here/build/lut.o $here/build/sense.o"
frameworks="-framework Foundation -framework AppKit -framework Metal -framework QuartzCore -framework CoreVideo -framework CoreGraphics -framework IOKit"

# shellcheck disable=SC2086
clang -dynamiclib $cflags -fobjc-arc -fvisibility=hidden \
    -I "$here/src" -I "$here/build" -I "$xr_include" -I "$BREW/include" \
    "$here/src/runtime.m" "$here/src/vulkan.m" "$here/src/backend_psvr2.m" $core $usb $frameworks \
    -install_name @rpath/libpsvr2xr.dylib -o "$out/libpsvr2xr.dylib"
codesign -f -s - "$out/libpsvr2xr.dylib" 2>/dev/null

# shellcheck disable=SC2086
clang $cflags -fobjc-arc -I "$here/src" -I "$here/build" \
    "$here/tools/psvr2xr-selftest.m" "$here/src/backend_psvr2.m" $core $usb $frameworks \
    -o "$out/psvr2xr-selftest"
codesign -f -s - "$out/psvr2xr-selftest" 2>/dev/null

cat > "$out/psvr2xr.json" <<JSON
{
    "file_format_version": "1.0.0",
    "runtime": {
        "name": "psvr2xr",
        "library_path": "./libpsvr2xr.dylib"
    }
}
JSON
echo "built $out/libpsvr2xr.dylib and $out/psvr2xr-selftest"
[ -n "${NO_APP:-}" ] && exit 0

# MacVapoR.app with the PS VR2 in it. It is made from a MacVapoR.app you already have (from a
# release DMG, or from macvapor/make-release.sh), so ALVR and NotProton need not be built here:
# the app itself is rebuilt from macvapor/Sources, and the PS VR2 runtime and self-test are added
# beside alvrxr. MACVAPOR_APP picks the app to start from.
base=${MACVAPOR_APP:-/Applications/MacVapoR.app}
if [ ! -x "$base/Contents/MacOS/MacVapoR" ]; then
    echo "no MacVapoR.app at $base, so only the runtime was built. Install MacVapoR from its release DMG" >&2
    echo "(or set MACVAPOR_APP) and run this again to get MacVapoR.app with the PS VR2 in it." >&2
    exit 0
fi
app="$out/MacVapoR.app"
rm -rf "$app"
ditto "$base" "$app"
xattr -dr com.apple.quarantine "$app" 2>/dev/null || true
swiftc -O -parse-as-library -target "arm64-apple-macos$MIN_VER" "$here/macvapor/Sources/MacVapoR.swift" \
    -o "$app/Contents/MacOS/MacVapoR"
cp "$out/libpsvr2xr.dylib" "$out/psvr2xr-selftest" "$app/Contents/Library/MacVapoR/runtime/"
cp "$here/src/psvr2/LICENSE" "$app/Contents/Resources/LICENSES/psvr2-player-mac-MIT.txt"
# neutron support (games through the free neutron Steam Play tool instead of NotProton): the
# wineopenxr-neutron kit, its prebuilt Windows half and the sources of its Mac half, with this
# tree's setup script. NEUTRON_KIT picks the unpacked kit; without one the app is NotProton-only.
kit=${NEUTRON_KIT:-$here/wineopenxr-neutron}
if [ -f "$kit/x86_64-windows/wineopenxr.dll" ]; then
    rm -rf "$app/Contents/Resources/wineopenxr-neutron"
    mkdir -p "$app/Contents/Resources/wineopenxr-neutron"
    cp -R "$kit/x86_64-windows" "$kit/src" "$kit/include" "$kit/wineopenxr64.json" "$app/Contents/Resources/wineopenxr-neutron/"
    cp "$here/tools/neutron-vr-setup.sh" "$app/Contents/Resources/wineopenxr-neutron/"
    echo "added neutron support from $kit"
else
    echo "no wineopenxr-neutron kit at $kit, so this app is NotProton-only (set NEUTRON_KIT)" >&2
fi
codesign -f -s - "$app/Contents/Library/MacVapoR/runtime/libpsvr2xr.dylib" "$app/Contents/Library/MacVapoR/runtime/psvr2xr-selftest"
codesign -f -s - "$app"
echo "built $app: drag it to Applications in place of the old one, open it, press Reinstall, then"
echo "choose PlayStation VR2 under Show games on"
