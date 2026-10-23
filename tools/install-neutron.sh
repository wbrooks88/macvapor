#!/bin/bash
# Installs neutron (github.com/lixsldotnet/neutron), the free Steam Play tool MacVapoR sets games up
# for, or puts its Steam hook back after a Steam update replaced Steam.app. MacVapoR runs it from
# its Install Everything button; it works from Terminal too.
#
# neutron's installer quits Steam, points Steam.app at a start script that turns Steam Play on, and
# starts Steam again. That start script runs a Node helper, so a node must be on PATH: Homebrew's,
# or the one MacVapoR bundles (NODE_DIR).
#
# Overridable: NEUTRON_URL, NODE_DIR, WORK
set -eu
NEUTRON_URL=${NEUTRON_URL:-https://github.com/lixsldotnet/neutron/releases/download/v0.1.0/neutron-macos-arm64.tar.xz}
WORK=${WORK:-$HOME/Library/Application Support/MacVapoR/neutron-release}
files="$HOME/Library/Application Support/neutron/compatibilitytools.d/neutron_proton/files"
PATH="${NODE_DIR:+$NODE_DIR:}/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

die() { echo "$*" >&2; exit 1; }
[ -d /Applications/Steam.app ] || die "Install Steam first (store.steampowered.com), log in once, then try again."
[ -d "$HOME/Library/Application Support/Steam/Steam.AppBundle" ] || die "Open Steam and log in once, then try again."
command -v node >/dev/null || die "No node found for neutron's Steam helper."

hooked=$(plutil -extract CFBundleExecutable raw /Applications/Steam.app/Contents/Info.plist 2>/dev/null || true)
if [ -x "$files/bin/wine" ] && [ "$hooked" = steam_neutron ]; then
    echo "neutron is installed and hooked into Steam."
    exit 0
fi

if [ ! -x "$WORK/neutron/install.sh" ]; then
    echo "Downloading neutron (about 260 MB)…"
    rm -rf "$WORK"
    mkdir -p "$WORK"
    curl -fsSL "$NEUTRON_URL" | tar xJ -C "$WORK"
fi
[ -x "$WORK/neutron/install.sh" ] || die "The neutron download has no install.sh."
echo "Installing neutron; Steam quits and starts again."
cd "$WORK/neutron"
./install.sh
echo "neutron installed. Windows-only games in Steam now run through it."
