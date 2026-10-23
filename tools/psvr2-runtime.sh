#!/bin/sh
# Points the OpenXR loader at the PS VR2 runtime (out/libpsvr2xr.dylib) or back at MacVapoR's.
# Games set up by MacVapoR find the runtime through ~/.config/openxr/1/active_runtime.json, so
# this is all a game needs to switch headsets.
#
#   tools/psvr2-runtime.sh on       the PS VR2 runtime (the current manifest is kept beside it)
#   tools/psvr2-runtime.sh off      back to what was there before
#   tools/psvr2-runtime.sh status
set -eu
here=$(cd "$(dirname "$0")/.." && pwd)
dir="$HOME/.config/openxr/1"
active="$dir/active_runtime.json"
kept="$dir/active_runtime.json.before-psvr2xr"
lib="$here/out/libpsvr2xr.dylib"
case "${1:-status}" in
on)
    [ -f "$lib" ] || { echo "no $lib; run ./build-psvr2.sh first" >&2; exit 1; }
    mkdir -p "$dir"
    if [ -f "$active" ] && ! grep -q libpsvr2xr.dylib "$active"; then cp "$active" "$kept"; fi
    cat > "$active" <<JSON
{
    "file_format_version": "1.0.0",
    "runtime": {
        "name": "psvr2xr",
        "library_path": "$lib"
    }
}
JSON
    echo "games now use the PS VR2 runtime ($lib)"
    ;;
off)
    if [ -f "$kept" ]; then mv -f "$kept" "$active"; echo "restored the previous runtime"
    else rm -f "$active"; echo "removed the PS VR2 runtime; open MacVapoR and click Install to set its runtime again"; fi
    ;;
status)
    if [ -f "$active" ]; then sed -n 's/.*"library_path": *"\(.*\)".*/active runtime: \1/p' "$active"
    else echo "no active OpenXR runtime"; fi
    ;;
*) sed -n 2,9p "$0" >&2; exit 2 ;;
esac
