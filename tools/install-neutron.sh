#!/bin/bash
# Installs neutron (github.com/lixsldotnet/neutron), the free Steam Play tool MacVapoR sets games up
# for, or puts its Steam hook back after a Steam update replaced Steam.app. MacVapoR runs it from
# its Install Everything button; it works from Terminal too.
#
# neutron's installer quits Steam, points Steam.app at a start script that turns Steam Play on, and
# starts Steam again. That start script runs a Node helper, so a node must be on PATH: Homebrew's,
# or the one MacVapoR bundles (NODE_DIR).
#
# It also turns Steam Play on for all games with neutron (Steam's global compat tool mapping,
# AppID 0 in config.vdf), which neutron's Steam helper does through Steam's UI but no longer can
# with Steam's newer settings (GetGlobalCompatTools is gone); a Steam update can undo either. Steam
# writes config.vdf when it quits, so it is quit for the edit and started again.
#
# --check exits 0 when everything is in place, 1 otherwise, and changes nothing.
#
# Overridable: NEUTRON_URL, NODE_DIR, WORK
set -eu
NEUTRON_URL=${NEUTRON_URL:-https://github.com/lixsldotnet/neutron/releases/download/v0.1.0/neutron-macos-arm64.tar.xz}
WORK=${WORK:-$HOME/Library/Application Support/MacVapoR/neutron-release}
files="$HOME/Library/Application Support/neutron/compatibilitytools.d/neutron_proton/files"
PATH="${NODE_DIR:+$NODE_DIR:}/opt/homebrew/bin:/usr/bin:/bin:/usr/sbin:/sbin"
export PATH

die() { echo "$*" >&2; exit 1; }
config=${STEAM_CONFIG:-"$HOME/Library/Application Support/Steam/config/config.vdf"}

# Steam Play on for all games, with neutron: a "0" entry under CompatToolMapping.
steam_play_all() {
    [ -f "$config" ] && /usr/bin/perl -0ne 'exit(/"CompatToolMapping"\s*\{[^}]*?"0"\s*\{\s*"name"\s*"neutron_proton"/s ? 0 : 1)' "$config"
}
steam_running() { pgrep -x steam_osx >/dev/null; }
quit_steam() {
    open "steam://exit" 2>/dev/null || true
    for _ in $(seq 60); do steam_running || return 0; sleep 1; done
    die "Steam did not quit; quit it and try again."
}
turn_steam_play_on() {
    steam_play_all && return 0
    [ -f "$config" ] || die "No Steam settings found; open Steam and log in once, then try again."
    local was=0
    if steam_running; then was=1; echo "Turning Steam Play on for all games; Steam quits and starts again."; quit_steam; fi
    cp "$config" "$config.before-neutron"
    /usr/bin/perl -0pi -e 's/("CompatToolMapping"\s*\{)/$1\n\t\t\t\t\t"0"\n\t\t\t\t\t{\n\t\t\t\t\t\t"name"\t\t"neutron_proton"\n\t\t\t\t\t\t"config"\t\t""\n\t\t\t\t\t\t"priority"\t\t"75"\n\t\t\t\t\t}/' "$config"
    if ! steam_play_all; then
        # No CompatToolMapping section yet: add one inside the Steam section.
        /usr/bin/perl -0pi -e 's/("Steam"\s*\{)/$1\n\t\t\t\t"CompatToolMapping"\n\t\t\t\t{\n\t\t\t\t\t"0"\n\t\t\t\t\t{\n\t\t\t\t\t\t"name"\t\t"neutron_proton"\n\t\t\t\t\t\t"config"\t\t""\n\t\t\t\t\t\t"priority"\t\t"75"\n\t\t\t\t\t}\n\t\t\t\t}/' "$config"
    fi
    steam_play_all || die "Could not turn Steam Play on in $config."
    echo "Steam Play is on for all games, with neutron."
    [ $was = 1 ] && open -a /Applications/Steam.app
    return 0
}
[ "${1:-}" = --steam-play ] && { turn_steam_play_on; exit; }   # only that
[ -d /Applications/Steam.app ] || die "Install Steam first (store.steampowered.com), log in once, then try again."
[ -d "$HOME/Library/Application Support/Steam/Steam.AppBundle" ] || die "Open Steam and log in once, then try again."
command -v node >/dev/null || die "No node found for neutron's Steam helper."

hooked=$(plutil -extract CFBundleExecutable raw /Applications/Steam.app/Contents/Info.plist 2>/dev/null || true)
if [ "${1:-}" = --check ]; then
    [ -x "$files/bin/wine" ] && [ "$hooked" = steam_neutron ] && steam_play_all
    exit $?
fi
if [ -x "$files/bin/wine" ] && [ "$hooked" = steam_neutron ]; then
    turn_steam_play_on
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
hooked=$(plutil -extract CFBundleExecutable raw /Applications/Steam.app/Contents/Info.plist 2>/dev/null || true)
[ "$hooked" = steam_neutron ] || die "neutron could not hook into Steam: macOS kept it from changing Steam.app. Allow MacVapoR (or Terminal) under System Settings > Privacy & Security > App Management, then try again."
turn_steam_play_on
echo "neutron installed. Windows-only games in Steam now run through it."
