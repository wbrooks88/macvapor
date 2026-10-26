#!/bin/bash
# Double-click installer for MacVapoR with the PS VR2 and neutron, from a folder that holds this
# file, macvapor.bundle (this repo's psvr2-backend branch) and wineopenxr-neutron/ (the kit).
# It installs what is missing and builds MacVapoR on this Mac, so nothing needs signing:
#   - Apple's Command Line Tools (asks, then stops: run it again once they are in)
#   - Homebrew's libusb, vulkan-headers and node (Homebrew itself has to be there: brew.sh)
#   - Khronos' OpenXR-SDK headers, in ~/Source/OpenXR-SDK
#   - the MacVapoR release app the build starts from (fayep/MacVapoR v0.1.0), kept in
#     ~/Library/Application Support/MacVapoR/release
#   - neutron (lixsldotnet/neutron v0.1.0), whose installer quits and restarts Steam
#   - MacVapoR built from this branch into /Applications/MacVapoR.app
# Safe to run again: steps that are done are skipped, and the app is rebuilt.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
SRC=$HOME/Source
SUPPORT="$HOME/Library/Application Support/MacVapoR"
MACVAPOR_RELEASE=https://github.com/fayep/MacVapoR/releases/download/v0.1.0/MacVapoR-0.1.0.dmg
NEUTRON_RELEASE=https://github.com/lixsldotnet/neutron/releases/download/v0.1.0/neutron-macos-arm64.tar.xz
NEUTRON_FILES="$HOME/Library/Application Support/neutron/compatibilitytools.d/neutron_proton/files"

step() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
finish() {
    printf '\n%s\n' "$1"
    read -r -n 1 -p "Press any key to close this window." _ || true
    echo
    exit "${2:-0}"
}
fail() { finish "$(printf '\033[1;31m%s\033[0m' "$*")" 1; }
trap 'fail "Something went wrong above. Send Claude the last 20 lines of this window."' ERR

[ "$(uname -m)" = arm64 ] || fail "This needs a Mac with Apple Silicon (M1 or newer)."
[ -f "$here/macvapor.bundle" ] && [ -d "$here/wineopenxr-neutron" ] || fail "Keep this file in its folder, next to macvapor.bundle and wineopenxr-neutron."
[ -d /Applications/Steam.app ] || fail "Install Steam first (store.steampowered.com), log in once, then run this again."
[ -d "$HOME/Library/Application Support/Steam/Steam.AppBundle" ] || fail "Open Steam and log in once, then run this again."

step "Command Line Tools"
if ! xcode-select -p >/dev/null 2>&1; then
    xcode-select --install 2>/dev/null || true
    fail "A window asks to install the Command Line Tools: click Install, wait for it to finish, then double-click this again."
fi

step "Homebrew packages"
BREW=/opt/homebrew/bin/brew
[ -x "$BREW" ] || fail "Homebrew is needed. Open Terminal, paste the line from https://brew.sh, then double-click this again."
eval "$("$BREW" shellenv)"
for pkg in libusb vulkan-headers node; do
    brew list --versions "$pkg" >/dev/null 2>&1 || brew install "$pkg"
done

step "OpenXR headers"
mkdir -p "$SRC"
[ -f "$SRC/OpenXR-SDK/include/openxr/openxr.h" ] || git clone -q --depth 1 https://github.com/KhronosGroup/OpenXR-SDK "$SRC/OpenXR-SDK"

step "MacVapoR source"
repo=$SRC/macvapor-psvr2
if [ -d "$repo/.git" ]; then
    git -C "$repo" fetch -q "$here/macvapor.bundle" psvr2-backend
    git -C "$repo" checkout -q -f -B psvr2-backend FETCH_HEAD
else
    git clone -q -b psvr2-backend "$here/macvapor.bundle" "$repo"
fi

step "MacVapoR release to build from"
base="$SUPPORT/release/MacVapoR.app"
if [ ! -x "$base/Contents/MacOS/MacVapoR" ]; then
    mkdir -p "$SUPPORT/release"
    if [ -x /Applications/MacVapoR.app/Contents/MacOS/MacVapoR ] \
       && [ ! -f /Applications/MacVapoR.app/Contents/Library/MacVapoR/runtime/libpsvr2xr.dylib ]; then
        ditto /Applications/MacVapoR.app "$base"
    else
        dmg=$(mktemp -d)/MacVapoR.dmg
        curl -fL --progress-bar -o "$dmg" "$MACVAPOR_RELEASE"
        mnt=$(mktemp -d)
        hdiutil attach -quiet -nobrowse -readonly -mountpoint "$mnt" "$dmg"
        app=$(find "$mnt" -maxdepth 1 -name 'MacVapoR*.app' | head -1)
        [ -n "$app" ] || fail "The MacVapoR download has no app in it."
        ditto "$app" "$base"
        hdiutil detach -quiet "$mnt"
        rm -f "$dmg"
    fi
fi

step "neutron"
if [ ! -x "$NEUTRON_FILES/bin/wine" ] \
   || [ "$(plutil -extract CFBundleExecutable raw /Applications/Steam.app/Contents/Info.plist)" != steam_neutron ]; then
    nsrc="$SUPPORT/neutron-release"
    if [ ! -x "$nsrc/neutron/install.sh" ]; then
        mkdir -p "$nsrc"
        curl -fL --progress-bar "$NEUTRON_RELEASE" | tar xJ -C "$nsrc"
    fi
    echo "neutron's installer quits Steam, sets it up and starts it again."
    (cd "$nsrc/neutron" && ./install.sh)
fi

step "Building MacVapoR (a minute or two)"
osascript -e 'quit app "MacVapoR"' >/dev/null 2>&1 || true
(cd "$repo" && MACVAPOR_APP="$base" NEUTRON_KIT="$here/wineopenxr-neutron" OPENXR="$SRC/OpenXR-SDK" ./build-psvr2.sh)
[ -x "$repo/out/MacVapoR.app/Contents/MacOS/MacVapoR" ] || fail "The build did not make MacVapoR.app."

step "Installing MacVapoR"
rm -rf /Applications/MacVapoR.app
ditto "$repo/out/MacVapoR.app" /Applications/MacVapoR.app
open /Applications/MacVapoR.app

trap - ERR
finish "$(printf '\033[1;32mDone.\033[0m In MacVapoR: press Install (or Reinstall), pick PlayStation VR2,
then for each game: start it once from Steam and quit, press Set Up, then Play.')"
