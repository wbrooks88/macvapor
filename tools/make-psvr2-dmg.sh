#!/bin/bash
# Makes out/MacVapoR-PSVR2.dmg: MacVapoR with the PS VR2 runtime and everything neutron needs
# inside, so a Mac that receives it needs only Steam. No Homebrew, Command Line Tools or Terminal
# there: the app's Install Everything button installs neutron with the node bundled here, and the
# Mac half of wineopenxr comes prebuilt against neutron 0.1.0's ntdll.so (a later neutron makes
# the setup script rebuild it, which then does need the Command Line Tools).
#
# Run it on an Apple silicon Mac that can run build-psvr2.sh (Command Line Tools, Homebrew's
# libusb and vulkan-headers, OpenXR-SDK in ~/Source/OpenXR-SDK). It downloads what else it needs
# into build/dmg. The app is signed ad hoc, so people open it the first time with right-click >
# Open.
#
#   tools/make-psvr2-dmg.sh                 the kit from ./wineopenxr-neutron
#   NEUTRON_KIT=/path/to/kit tools/make-psvr2-dmg.sh
#
# Overridable: NEUTRON_KIT, MACVAPOR_APP (the release app to start from), NEUTRON_URL, NODE_URL
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
NEUTRON_KIT=${NEUTRON_KIT:-$here/wineopenxr-neutron}
NEUTRON_URL=${NEUTRON_URL:-https://github.com/lixsldotnet/neutron/releases/download/v0.1.0/neutron-macos-arm64.tar.xz}
NODE_URL=${NODE_URL:-https://nodejs.org/dist/v22.12.0/node-v22.12.0-darwin-arm64.tar.xz}
MACVAPOR_RELEASE=https://github.com/fayep/MacVapoR/releases/download/v0.1.0/MacVapoR-0.1.0.dmg
work=$here/build/dmg
out=$here/out
mkdir -p "$work" "$out"

die() { echo "$*" >&2; exit 1; }
[ -f "$NEUTRON_KIT/x86_64-windows/wineopenxr.dll" ] || die "no wineopenxr-neutron kit at $NEUTRON_KIT"

# The MacVapoR release the app is made from (alvrxr, the audio helper, NotProton's wineopenxr).
base=${MACVAPOR_APP:-}
if [ -z "$base" ]; then
    base="$work/release/MacVapoR.app"
    if [ ! -x "$base/Contents/MacOS/MacVapoR" ]; then
        echo "==> downloading the MacVapoR release"
        curl -fL --progress-bar -o "$work/MacVapoR.dmg" "$MACVAPOR_RELEASE"
        mnt=$(mktemp -d)
        hdiutil attach -quiet -nobrowse -readonly -mountpoint "$mnt" "$work/MacVapoR.dmg"
        app=$(find "$mnt" -maxdepth 1 -name 'MacVapoR*.app' | head -1)
        mkdir -p "$work/release"
        [ -n "$app" ] && ditto "$app" "$base"
        hdiutil detach -quiet "$mnt"
        [ -x "$base/Contents/MacOS/MacVapoR" ] || die "the MacVapoR release has no app in it"
    fi
fi
loader="$base/Contents/Library/MacVapoR/wineopenxr/aarch64-unix/libopenxr_loader.dylib"
[ -f "$loader" ] || die "no arm64 OpenXR loader in $base"

# neutron 0.1.0, the one Install Everything installs, for its ntdll.so.
if [ ! -f "$work/neutron/dist/neutron/files/lib/wine/aarch64-unix/ntdll.so" ]; then
    echo "==> downloading neutron"
    rm -rf "$work/neutron"
    curl -fL --progress-bar "$NEUTRON_URL" | tar xJ -C "$work"
    xattr -dr com.apple.quarantine "$work/neutron" 2>/dev/null || true
fi
neutron_files=$work/neutron/dist/neutron/files
[ -f "$neutron_files/lib/wine/aarch64-unix/ntdll.so" ] || die "the neutron download has no ntdll.so"

echo "==> prebuilding wineopenxr's Mac half against neutron's ntdll.so"
kit=$work/wineopenxr-neutron
rm -rf "$kit"
mkdir -p "$kit"
cp -R "$NEUTRON_KIT/x86_64-windows" "$NEUTRON_KIT/src" "$NEUTRON_KIT/include" "$NEUTRON_KIT/wineopenxr64.json" "$kit/"
cp "$here/tools/neutron-vr-setup.sh" "$kit/"
NEUTRON="$neutron_files" LOADER="$loader" bash "$kit/neutron-vr-setup.sh" --build

if [ ! -x "$work/node/bin/node" ]; then
    echo "==> downloading node"
    rm -rf "$work/node"
    mkdir -p "$work/node"
    curl -fL --progress-bar "$NODE_URL" | tar xJ -C "$work/node" --strip-components 1
fi

echo "==> building MacVapoR"
MACVAPOR_APP="$base" NEUTRON_KIT="$kit" NODE_DIST="$work/node" "$here/build-psvr2.sh"
app=$out/MacVapoR.app
for f in Contents/Resources/install-neutron.sh Contents/Resources/node/bin/node \
         Contents/Resources/wineopenxr-neutron/aarch64-unix/wineopenxr.so \
         Contents/Library/MacVapoR/runtime/libpsvr2xr.dylib; do
    [ -e "$app/$f" ] || die "the app is missing $f"
done
if otool -L "$app/Contents/Library/MacVapoR/runtime/libpsvr2xr.dylib" | grep -q /opt/homebrew; then
    die "libpsvr2xr.dylib links a Homebrew library, so it would not load on other Macs"
fi

echo "==> making the DMG"
stage=$work/stage
rm -rf "$stage"
mkdir -p "$stage"
ditto "$app" "$stage/MacVapoR.app"
ln -s /Applications "$stage/Applications"
cat > "$stage/Read Me.txt" <<'TXT'
MacVapoR with the PS VR2: Windows VR games from Steam on a PlayStation VR2, on an Apple silicon Mac.

1. Install Steam (store.steampowered.com) and log in once.
2. Drag MacVapoR to Applications. The first time, right-click it and choose Open, then Open.
3. Press Install Everything. Steam restarts once.
4. Connect the PS VR2 through Sony's PC adapter (DisplayPort and USB into the Mac) and pair the
   Sense controllers under System Settings > Bluetooth.
5. Install a 64-bit Windows VR game in Steam, then press Play next to it in MacVapoR. The first
   time, the game starts once without VR so it can be prepared: quit it and press Play again.
TXT
dmg=$out/MacVapoR-PSVR2.dmg
rm -f "$dmg"
hdiutil create -quiet -volname "MacVapoR PS VR2" -srcfolder "$stage" -format UDZO -ov "$dmg"
echo "==> made $dmg ($(du -h "$dmg" | cut -f1))"
