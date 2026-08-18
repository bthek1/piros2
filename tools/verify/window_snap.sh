#!/usr/bin/env bash
# Capture every X11 window whose title matches a pattern to PNG files.
#
# The dev box session is Wayland, but rviz2/rqt run through Xwayland (the
# QT_QPA_PLATFORM=xcb pin), so their windows are X windows and `xwd`
# (installed) can dump them — the *root* window cannot be dumped under
# rootless Xwayland (BadMatch), so this walks the window tree and grabs
# each matching top-level window by id. ffmpeg decodes XWD → PNG; no
# ImageMagick here. A window that is fully occluded or minimised dumps
# whatever the X server has, which may be stale — read the picture as
# "what the window held", not proof it was on screen.
#
#   window_snap.sh OUTDIR [TITLE_REGEX]      default regex: rviz|rqt|Open3D|\.ply (the mesh viewer titles its window with the file path)
#
# Prints one line per capture; exits 0 even when nothing matched (an
# honest "no window" is a result, not a failure).
set -uo pipefail
out=${1:?usage: window_snap.sh OUTDIR [TITLE_REGEX]}
pattern=${2:-'rviz|rqt|Open3D|\.ply'}
mkdir -p "$out"
if ! command -v xwininfo >/dev/null || ! command -v xwd >/dev/null; then
    echo "window_snap: xwininfo/xwd missing (apt install x11-utils x11-apps)" >&2
    exit 0
fi
n=0
# Top-level frames come from mutter-x11-frames and carry the title; the
# client window sits one level below with the same title — keep the
# outermost (largest) match per title so each window is captured once.
while read -r id title; do
    [ -n "$id" ] || continue
    safe=$(printf '%s' "$title" | tr -c 'A-Za-z0-9._-' '_' | cut -c1-60)
    file="$out/window_${safe}.png"
    if xwd -id "$id" -silent 2>/dev/null \
        | ffmpeg -hide_banner -loglevel error -y -i - -frames:v 1 "$file" 2>/dev/null; then
        echo "window: $title → $file"
        n=$((n + 1))
    else
        echo "window: $title ($id) — capture failed" >&2
    fi
done < <(xwininfo -root -tree 2>/dev/null \
    | grep -E "^ {5}0x[0-9a-f]+ \"[^\"]*(${pattern})[^\"]*\"" \
    | awk '{ if (match($0, /[0-9]+x[0-9]+\+/)) { split(substr($0, RSTART, RLENGTH), g, /[x+]/); if (g[1] < 50 || g[2] < 50) next } print }' \
    | sed -E 's/^ +(0x[0-9a-f]+) "([^"]*)".*/\1 \2/' \
    | awk '{t=$0; sub(/^[^ ]+ /, "", t); if (!seen[t]++) print}')
[ "$n" -gt 0 ] || echo "window: no window matching /${pattern}/ (session not up, or not an X window — see docs/info/verification.md)"
