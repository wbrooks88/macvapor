#!/bin/sh
# Builds the PS VR2 variant of the runtime and its self-test into out/ (see PSVR2.md):
#   out/libpsvr2xr.dylib, out/psvr2xr.json   the OpenXR runtime, runtime.m on backend_psvr2.m
#   out/psvr2xr-selftest                      the backend alone, showing a test room
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
core="$here/build/cpsvr2.o $here/build/fusion.o $here/build/lut.o"
frameworks="-framework Foundation -framework AppKit -framework Metal -framework QuartzCore -framework CoreVideo -framework CoreGraphics"

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
echo "try the headset: $out/psvr2xr-selftest; then switch games to it: tools/psvr2-runtime.sh on"
