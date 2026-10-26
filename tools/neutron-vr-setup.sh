#!/bin/bash
# Sets up one neutron game prefix for OpenXR through wineopenxr and MacVapoR's runtime (the PS VR2
# one by default), with no CrossOver or NotProton. neutron (github.com/lixsldotnet/neutron) is a
# free Steam Play tool: arm64 Wine 11.19 with ARM64EC system DLLs and FEX for the game's x86_64
# code. Run it with the game closed, after the game has been launched once with neutron so its
# prefix exists.
#
#   neutron-vr-setup.sh APPID 'Game.exe'          e.g. 620980 'Beat Saber.exe'
#       (the executable's path inside the game folder, e.g. 450390 TheLab/win64/TheLab.exe)
#   neutron-vr-setup.sh --undo APPID 'Game.exe'
#   neutron-vr-setup.sh --build                   only build wineopenxr's Mac half
#   TARGET=alvr neutron-vr-setup.sh ...           the Vision Pro runtime instead of the PS VR2
#   OC=/path/to/openvr_api.dll neutron-vr-setup.sh APPID 'Game.exe'
#       an OpenVR game: OpenComposite's 64-bit openvr_api.dll replaces the game's own, which is
#       kept as openvr_api.dll.orig and put back by --undo
#
# This script sits in the wineopenxr-neutron kit, which carries:
#   - x86_64-windows/wineopenxr.dll, Proton's wineopenxr with NotProton's crossover.patch (D3D11
#     on DXMT), built against neutron's Wine 11.19. A game's x86_64 code runs in FEX, and so does
#     this DLL, as NotProton's FEX runner already did.
#   - src/ and include/, from which the first run builds the arm64 unix half (wineopenxr.so)
#     against neutron's own ntdll.so, with neutron's TEB-in-TSD defines. That takes seconds and
#     needs only the Command Line Tools.
#
# What goes where, and why (neutron's tool/neutron run script):
#   - neutron sets no WINEDLLPATH, so wineopenxr goes into neutron's own lib/wine/aarch64-windows
#     and aarch64-unix, beside its other builtins, with the native OpenXR loader beside the .so.
#     A neutron update drops them; run this again after one.
#   - The Khronos loader in the game opens the runtime by its system32 path, so the DLL and
#     wineopenxr64.json go into the prefix's system32 too, and HKLM's ActiveRuntime names it.
#   - HKCU\Software\Wine\VR is written as a persistent key, with the values wineopenxr reads.
#   - D3D11 is DXMT in neutron, which is what wineopenxr's Metal path needs. Vulkan games don't
#     work this way.
#   - The native OpenXR loader finds the runtime through ~/.config/openxr/1/active_runtime.json,
#     which the MacVapoR app writes when you pick a headset.
#   - Leave NEUTRON_SANDBOX off for VR games: the sandbox has no USB or MacVapoR access.
#
# Overridable: NEUTRON (neutron's files folder), LOADER (an arm64 libopenxr_loader.dylib),
# MACVAPOR (MacVapoR's Application Support folder), STEAM, TARGET
set -eu
kit=$(cd "$(dirname "$0")" && pwd)

NEUTRON=${NEUTRON:-$HOME/Library/Application Support/neutron/compatibilitytools.d/neutron_proton/files}
MACVAPOR=${MACVAPOR:-$HOME/Library/Application Support/MacVapoR}
STEAM=${STEAM:-$HOME/Library/Application Support/Steam}
TARGET=${TARGET:-psvr2}
wine_pe="$NEUTRON/lib/wine/aarch64-windows"
wine_unix="$NEUTRON/lib/wine/aarch64-unix"
out="$kit/aarch64-unix"

die() { echo "$*" >&2; exit 1; }

[ "$(uname -m)" = arm64 ] || die "Apple Silicon only."
[ -f "$wine_unix/ntdll.so" ] || die "neutron not found at $NEUTRON; install it (./install.sh from its release)"
"$NEUTRON/bin/wine" --version 2>/dev/null | grep -q "^wine-11\.19" \
    || echo "note: neutron's Wine is $("$NEUTRON/bin/wine" --version 2>/dev/null), the kit was built for 11.19" >&2

build() {
    command -v clang >/dev/null || die "no clang: xcode-select --install"
    # The arm64 OpenXR loader that ships with MacVapoR's wineopenxr.
    if [ -z "${LOADER:-}" ]; then
        for l in "$MACVAPOR/wineopenxr/aarch64-unix/libopenxr_loader.dylib" \
                 /Applications/MacVapoR.app/Contents/Library/MacVapoR/wineopenxr/aarch64-unix/libopenxr_loader.dylib \
                 "$out/libopenxr_loader.dylib"; do
            [ -f "$l" ] && { LOADER=$l; break; }
        done
    fi
    [ -n "${LOADER:-}" ] && [ -f "$LOADER" ] || die "no libopenxr_loader.dylib; install MacVapoR.app, or set LOADER"
    lipo -archs "$LOADER" | grep -qw arm64 || die "$LOADER has no arm64"
    xattr -dr com.apple.quarantine "$kit" 2>/dev/null || true
    local tmp o objs=""
    tmp=$(mktemp -d)
    echo "building wineopenxr.so against $wine_unix/ntdll.so"
    for c in openxr.c openxr_thunks.c; do
        o="$tmp/${c%.c}.o"
        clang -arch arm64 -mmacosx-version-min=15.0 -std=gnu2x -c -o "$o" "$kit/src/$c" \
            -I"$kit/src" -I"$kit/include" -D__WINESRC__ -DWINE_UNIX_LIB -DWINE_NO_LONG_TYPES \
            -DWINE_TEB_IN_TSD -DWINE_USER_SHARED_DATA_VA=0x7ffffdfe0000ull \
            -fPIC -fasynchronous-unwind-tables -g -O2 -w
        objs="$objs $o"
    done
    # D3D11 on DXMT's Metal half, which stays clear of Wine's headers.
    clang -arch arm64 -mmacosx-version-min=15.0 -fobjc-arc -c -o "$tmp/metal.o" "$kit/src/metal.m" -fPIC -g -O2
    mkdir -p "$out"
    # shellcheck disable=SC2086
    clang -arch arm64 -mmacosx-version-min=15.0 -dynamiclib -o "$out/wineopenxr.so" $objs "$tmp/metal.o" \
        "$wine_unix/ntdll.so" "$LOADER" -framework Metal -framework Foundation \
        -Wl,-rpath,@loader_path -install_name @rpath/wineopenxr.so 2>&1 | grep -v "was built for newer" || true
    rm -rf "$tmp"
    [ -f "$out/wineopenxr.so" ] || die "link failed"
    nm -u "$out/wineopenxr.so" | grep -qE "_xrGetInstanceProcAddr$" || die "wineopenxr.so does not import the OpenXR loader"
    [ "$LOADER" -ef "$out/libopenxr_loader.dylib" ] || cp -f "$LOADER" "$out/libopenxr_loader.dylib"
    codesign -s - -f "$out/wineopenxr.so" >/dev/null 2>&1 || true
    echo "built $out/wineopenxr.so"
}

if [ "${1:-}" = "--build" ]; then
    build
    shasum -a 256 "$wine_unix/ntdll.so" | cut -d' ' -f1 > "$out/ntdll.sha256"
    exit 0
fi

undo=0
[ "${1:-}" = "--undo" ] && { undo=1; shift; }
[ $# -eq 2 ] || { sed -n 8,16p "$0" >&2; exit 2; }
appid=$1
exe=$2
case "$appid" in ''|*[!0-9]*) die "not an app id: $appid" ;; esac

case "$TARGET" in
    psvr2) runtime_name=psvr2xr runtime_lib="$MACVAPOR/runtime/libpsvr2xr.dylib" ;;
    alvr) runtime_name=alvrxr runtime_lib="$MACVAPOR/runtime/libalvrxr.dylib" ;;
    *) die "TARGET is psvr2 or alvr" ;;
esac

# The game may be in any Steam library; its prefix is in that library's compatdata or the main one's.
library=
for lib in "$STEAM" $(sed -n 's/.*"path"[[:space:]]*"\(.*\)".*/\1/p' "$STEAM/steamapps/libraryfolders.vdf" | tr ' ' '\001'); do
    lib=$(printf '%s' "$lib" | tr '\001' ' ')
    [ -f "$lib/steamapps/appmanifest_$appid.acf" ] && { library=$lib; break; }
done
[ -n "$library" ] || die "app $appid is not installed in any Steam library"
prefix="$library/steamapps/compatdata/$appid/pfx"
[ -f "$prefix/system.reg" ] || prefix="$STEAM/steamapps/compatdata/$appid/pfx"
[ -f "$prefix/system.reg" ] || die "no prefix for $appid; launch the game once with neutron first"
game_dir=$(sed -n 's/.*"installdir"[[:space:]]*"\(.*\)".*/\1/p' "$library/steamapps/appmanifest_$appid.acf")
game="$library/steamapps/common/$game_dir"
[ -f "$game/$exe" ] || die "no $exe in $game"
exe_name=$(basename "$exe")
sys32="$prefix/drive_c/windows/system32"

export WINEPREFIX="$prefix" WINEDEBUG=-all
WINESERVER="$NEUTRON/bin/wineserver"
if pgrep -fi "$exe_name" >/dev/null 2>&1; then
    die "close the game first"
fi
reg() { "$NEUTRON/bin/wine" reg "$@" >/dev/null 2>&1; }

# The game's own 64-bit openvr_api.dlls, or their kept originals.
openvr_dlls() {
    find "$game" \( -iname openvr_api.dll -o -iname openvr_api.dll.orig \) | sed 's/\.orig$//' | sort -u |
        while IFS= read -r f; do
            orig=$f.orig
            [ -f "$orig" ] || orig=$f
            file "$orig" | grep -q "PE32+" && printf '%s\n' "$f"
        done
}

if [ $undo -eq 1 ]; then
    openvr_dlls | while IFS= read -r f; do
        if [ -f "$f.orig" ]; then
            mv -f "$f.orig" "$f"
            rm -f "$(dirname "$f")/opencomposite.ini"
        fi
    done
    rm -f "$sys32/wineopenxr.dll" "$sys32/wineopenxr64.json"
    reg delete 'HKCU\Software\Wine\VR' /f || true
    reg delete 'HKLM\Software\Khronos\OpenXR\1' /v ActiveRuntime /f || true
    "$WINESERVER" -w
    echo "removed from $prefix (wineopenxr stays in neutron's lib/wine, where nothing loads it unasked)"
    exit 0
fi

[ -f "$runtime_lib" ] || die "no $runtime_lib; open MacVapoR and install its runtime first"

# A prebuilt Mac half (make-psvr2-dmg.sh puts one in the app) is kept while it was linked against
# this very ntdll.so; otherwise, or after a neutron update, it is built here.
stamp="$out/ntdll.sha256"
now=$(shasum -a 256 "$wine_unix/ntdll.so" | cut -d' ' -f1)
if [ ! -f "$out/wineopenxr.so" ] || [ ! -f "$out/libopenxr_loader.dylib" ] || [ "$(cat "$stamp" 2>/dev/null)" != "$now" ]; then
    [ -w "$kit" ] || die "neutron changed since this MacVapoR was made; reinstall MacVapoR's runtime (Install Everything)"
    command -v clang >/dev/null || die "neutron is a different version from the one this MacVapoR was made for; install the Command Line Tools (xcode-select --install) so it can adapt, then set the game up again"
    build
    printf '%s' "$now" > "$stamp"
fi

cp -f "$kit/x86_64-windows/wineopenxr.dll" "$wine_pe/"
cp -f "$out/wineopenxr.so" "$out/libopenxr_loader.dylib" "$wine_unix/"
cp -f "$kit/x86_64-windows/wineopenxr.dll" "$kit/wineopenxr64.json" "$sys32/"

if [ -n "${OC:-}" ]; then
    file "$OC" | grep -q "PE32+" || die "$OC is not a 64-bit DLL"
    grep -aq "IVRCompositor_" "$OC" || die "$OC does not look like OpenComposite"
    list=$(mktemp)
    openvr_dlls > "$list"
    [ -s "$list" ] || { rm -f "$list"; die "no 64-bit openvr_api.dll in $game"; }
    while IFS= read -r f; do
        [ -f "$f.orig" ] || cp -p "$f" "$f.orig"
        cp -f "$OC" "$f"
        printf 'initUsingVulkan=false\n' > "$(dirname "$f")/opencomposite.ini"
        echo "OpenComposite in place of ${f#"$game"/}"
    done < "$list"
    rm -f "$list"
fi

reg add 'HKLM\Software\Khronos\OpenXR\1' /v ActiveRuntime /t REG_SZ /d 'C:\windows\system32\wineopenxr64.json' /f
reg add 'HKCU\Software\Wine\WineDbg' /v ShowCrashDialog /t REG_DWORD /d 0 /f
# The values wineopenxr_init_registry records for MacVapoR's runtimes (see alvrxr's vulkan.m).
reg add 'HKCU\Software\Wine\VR' /v state /t REG_DWORD /d 1 /f
reg add 'HKCU\Software\Wine\VR' /v openxr_vulkan_instance_extensions /t REG_SZ \
    /d 'VK_KHR_get_physical_device_properties2 VK_KHR_surface VK_KHR_win32_surface' /f
reg add 'HKCU\Software\Wine\VR' /v openxr_vulkan_device_extensions /t REG_SZ /d 'VK_KHR_maintenance1' /f
reg add 'HKCU\Software\Wine\VR' /v openxr_vulkan_device_vid /t REG_DWORD /d 0x106b /f
reg add 'HKCU\Software\Wine\VR' /v openxr_vulkan_device_pid /t REG_DWORD /d 0x1b0003f1 /f
"$WINESERVER" -w

mkdir -p "$HOME/.config/openxr/1"
active="$HOME/.config/openxr/1/active_runtime.json"
# Another runtime's manifest is left alone; one of MacVapoR's is pointed at TARGET's runtime.
if [ -e "$active" ] && ! grep -qE "libalvrxr.dylib|libpsvr2xr.dylib" "$active"; then
    echo "note: $active names another runtime; leaving it" >&2
else
    cat > "$active" <<JSON
{
    "file_format_version": "1.0.0",
    "runtime": {
        "name": "$runtime_name",
        "library_path": "$runtime_lib"
    }
}
JSON
fi
echo "set up $prefix for OpenXR ($exe, $runtime_name)"
