#!/bin/bash
# Build a release and publish it to GitHub Pages.
#
# Pages hosts what the dashboard's "Update from GitHub" button and the USB
# recovery flasher need:
#   firmware.bin          — the app image (also flashed at 0x10000 on recovery)
#   bootloader.bin        — recovery: flashed at 0x0
#   partitions.bin        — recovery: flashed at 0x8000
#   boot_app0.bin         — recovery: flashed at 0xe000 (otadata reset)
#   manifest.json         — {version, sha256, bin} for the OTA update check
#   recovery-manifest.json— ESP Web Tools install manifest
#   index.html            — the recovery flasher page (site root)
#
# These are committed to the `gh-pages` branch via a git worktree, so the
# main working tree is never touched.  First run bootstraps the branch.
#
# Usage:
#   bash tools/release.sh           # interactive — prompts before pushing
#   bash tools/release.sh --yes     # non-interactive — pushes without prompt
#                                   # (used after the OTA flash has been tested)

set -e
cd "$(dirname "$0")/.."

AUTO_YES=0
[ "${1:-}" = "--yes" ] || [ "${1:-}" = "-y" ] && AUTO_YES=1

BUILD_DIR="build/esp32.esp32.adafruit_feather_esp32s3"
PAGES_URL="https://toldxls.github.io/AutoFOV-V12"

# ── Step 1: build (embeds HTML + compiles) ───────────────────────────────────
echo "=== Step 1: build ==="
bash tools/build.sh

# ── Step 2: gather artifacts ─────────────────────────────────────────────────
echo ""
echo "=== Step 2: collect artifacts ==="

VERSION=$(sed -n 's/.*BUILD_VERSION "\([^"]*\)".*/\1/p' data/web_ui.h)
[ -n "$VERSION" ] || { echo "ERROR: could not read BUILD_VERSION from data/web_ui.h"; exit 1; }

APP_BIN="$BUILD_DIR/AutoFOV_V12.ino.bin"
BOOT_BIN="$BUILD_DIR/AutoFOV_V12.ino.bootloader.bin"
PART_BIN="$BUILD_DIR/AutoFOV_V12.ino.partitions.bin"
# boot_app0.bin ships with the ESP32 Arduino core — pick the newest installed.
BOOTAPP0=$(ls -1 "$HOME"/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin 2>/dev/null | sort -V | tail -1)

for f in "$APP_BIN" "$BOOT_BIN" "$PART_BIN" "$BOOTAPP0"; do
    [ -f "$f" ] || { echo "ERROR: missing build artifact: $f"; exit 1; }
done

SHA=$(shasum -a 256 "$APP_BIN" | awk '{print $1}')
APP_SIZE=$(wc -c < "$APP_BIN" | tr -d ' ')

echo "version : v$VERSION"
echo "app bin : $APP_SIZE bytes"
echo "sha256  : $SHA"

# ── Step 3: stage the gh-pages worktree ──────────────────────────────────────
echo ""
echo "=== Step 3: stage gh-pages ==="

WT=$(mktemp -d)
cleanup() { git worktree remove --force "$WT" 2>/dev/null || true; rm -rf "$WT"; }
trap cleanup EXIT

git fetch origin gh-pages:refs/remotes/origin/gh-pages 2>/dev/null || true
if git show-ref --verify --quiet refs/remotes/origin/gh-pages; then
    # Remote branch exists — reset the local gh-pages to match it (the main
    # tree never commits there, so a hard reset is always correct).
    git worktree add -B gh-pages "$WT" refs/remotes/origin/gh-pages
elif git show-ref --verify --quiet refs/heads/gh-pages; then
    git worktree add "$WT" gh-pages
else
    echo "gh-pages branch not found — bootstrapping a new one."
    git worktree add --detach "$WT"
    ( cd "$WT" && git checkout --orphan gh-pages )
fi
# Always clear the worktree before staging the release, so files dropped from
# the published set (e.g. a renamed page) actually disappear from gh-pages
# rather than lingering forever.
( cd "$WT" && git rm -rf --quiet . 2>/dev/null || true )

cp "$APP_BIN"  "$WT/firmware.bin"
cp "$BOOT_BIN" "$WT/bootloader.bin"
cp "$PART_BIN" "$WT/partitions.bin"
cp "$BOOTAPP0" "$WT/boot_app0.bin"
cp web/recovery.html "$WT/index.html"   # served at the Pages site root
: > "$WT/.nojekyll"   # serve files verbatim — no Jekyll processing

# V12.3: publish the owner-curated calibration library (dashboard CAL I/O →
# LOAD FROM GITHUB reads calibrations/index.json). Copy ONLY the top-level
# library files (*.json, README) — NOT photo subfolders that may live under
# calibrations/ locally as test backups (gitignored on main; would otherwise
# bloat gh-pages by hundreds of MB).
[ -d calibrations ] && mkdir -p "$WT/calibrations" && \
  find calibrations -maxdepth 1 -type f \( -name '*.json' -o -name '*.md' \) \
    -exec cp {} "$WT/calibrations/" \;

cat > "$WT/manifest.json" <<EOF
{ "version": "$VERSION", "sha256": "$SHA", "bin": "firmware.bin" }
EOF

# ESP Web Tools manifest — multi-part flash that leaves the NVS region (0x9000)
# untouched, so a recovery flash preserves calibration and WiFi settings.
cat > "$WT/recovery-manifest.json" <<EOF
{
  "name": "AutoFOV V12",
  "version": "$VERSION",
  "builds": [
    {
      "chipFamily": "ESP32-S3",
      "parts": [
        { "path": "bootloader.bin", "offset": 0 },
        { "path": "partitions.bin", "offset": 32768 },
        { "path": "boot_app0.bin",  "offset": 57344 },
        { "path": "firmware.bin",   "offset": 65536 }
      ]
    }
  ]
}
EOF

# ── Step 4: commit + push ────────────────────────────────────────────────────
echo ""
echo "=== Step 4: publish ==="
( cd "$WT" && git add -A && git status --short )

if [ "$AUTO_YES" = 1 ]; then
    ans=y
    echo ""
    echo "--yes given — pushing v$VERSION without prompting."
else
    printf "\nPush v%s to the gh-pages branch on origin? [y/N] " "$VERSION"
    read -r ans
fi
if [ "$ans" = "y" ] || [ "$ans" = "Y" ]; then
    ( cd "$WT" \
      && git commit -m "release v$VERSION" \
      && git push -u origin gh-pages )
    echo ""
    echo "=== Published ==="
    echo "Live at: $PAGES_URL/"
    echo "Manifest: $PAGES_URL/manifest.json"
    echo "Recovery: $PAGES_URL/"
    echo ""
    echo "First release? Enable Pages once:"
    echo "  GitHub → repo Settings → Pages → Source: branch 'gh-pages' / root"
else
    echo "Aborted — nothing pushed. Staged files left in: $WT"
    trap - EXIT   # keep the worktree for inspection
fi
