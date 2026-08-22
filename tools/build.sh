#!/bin/bash
# One-command build: regenerates web_ui.h then compiles the sketch.
# Run after committing all changes; OTA flash the resulting .bin.
# Usage: bash tools/build.sh [--force]
#   --force   build even when the .bin is at/over 97 % of the OTA slot

set -e
cd "$(dirname "$0")/.."
FORCE=0
for a in "$@"; do [ "$a" = "--force" ] && FORCE=1; done

ARDUINO_CLI="$HOME/bin/arduino-cli"
# esp32 core 3.3.x dropped the `PartitionScheme=custom` menu option for the
# adafruit_feather_esp32s3 board, so the old FQBN now fails FQBN validation.
# The sketch's own partitions.csv (symlink -> tools/partitions.csv) still defines
# the real dual-OTA layout — the core copies it into the build dir and it wins
# over any built-in scheme. We select `default` only to satisfy FQBN validation
# with a STANDARD bootloader (the board default, tinyuf2_noota, would flash a
# TinyUF2 bootloader and change boot behavior — do not use it), then reproduce
# the old `custom` behavior with two build-property overrides below:
#   build.partitions=            empty, mirroring how `custom` was defined
#   upload.maximum_size=<app0>   the real app0/app1 slot, so the build-time size
#                                guard reads e.g. "80% / 1900544" instead of
#                                `default`'s 1.2 MB.
# USBMode=hwcdc: Serial on the hardware USB-Serial/JTAG peripheral instead of
# TinyUSB CDC (the board default) — drops ~45 KB flash + ~31 KB internal RAM
# of unused USB-class buffers. Same D+/D- pins; the ROM download mode the USB
# recovery flasher relies on is independent of this setting.
FQBN="esp32:esp32:adafruit_feather_esp32s3:PSRAM=enabled,PartitionScheme=default,LoopCore=1,EventsCore=1,USBMode=hwcdc"
BUILD_DIR="build/esp32.esp32.adafruit_feather_esp32s3"

# Derive the app-slot size from partitions.csv (field 5 of the app0 row) so the
# size guard tracks the real layout instead of a hardcoded copy — the slot was
# already resized once (commit a1759ef) and a stale literal would let an
# oversized build pass the check and then fail every OTA on the device.
APP_SLOT_HEX="$(awk -F',' '$1=="app0"{gsub(/[ \t]/,"",$5); print $5}' tools/partitions.csv | head -1)"
APP_MAX_SIZE=$(( ${APP_SLOT_HEX:-0} ))   # bash arithmetic parses the 0x… hex
if [ "$APP_MAX_SIZE" -le 0 ]; then
    APP_MAX_SIZE=1900544
    echo "WARNING: couldn't read app0 size from tools/partitions.csv — using fallback $APP_MAX_SIZE"
fi

# Minify deps are optional — embed_html.py reports which it used on its status
# line ([js terser|rjsmin|off] [ws on] [gz zopfli|gzip]); an un-minified embed
# is ~100 KB gz LARGER (≈5 % of the OTA slot). Install once with:
#   cd tools && npm install          (terser — the real JS minifier)
#   pip3 install -r tools/requirements.txt   (rjsmin fallback, rcssmin, zopfli)
if [ ! -x tools/node_modules/terser/bin/terser ] && ! python3 -c "import rjsmin" 2>/dev/null; then
    echo "WARNING: no JS minifier found (terser or rjsmin) — HTML will embed un-minified (~100 KB gz larger)."
fi

echo "=== Step 1: embed HTML ==="
EMBED_OUT="$(python3 tools/embed_html.py)"
echo "$EMBED_OUT"
case "$EMBED_OUT" in *"[js off"*)
    if [ -t 1 ]; then printf '\033[31m%s\033[0m\n' "WARNING: dashboard JS embedded UN-minified — see the embed line above"
    else echo "WARNING: dashboard JS embedded UN-minified — see the embed line above"; fi;;
esac
# Pull the "vMAJOR.MINOR.PATCH" embed_html.py just stamped so we can name the
# output .bin after it — the OTA picker shows the filename, so a versioned copy
# makes it obvious which build is being uploaded. (|| true: grep miss must not
# trip set -e.)
VERSION="$(printf '%s\n' "$EMBED_OUT" | grep -oE 'v[0-9]+\.[0-9]+\.[0-9]+' | head -1 || true)"

echo "=== Step 2: compile ==="
"$ARDUINO_CLI" compile \
    --fqbn "$FQBN" \
    --build-property "build.partitions=" \
    --build-property "upload.maximum_size=$APP_MAX_SIZE" \
    --build-path "$(pwd)/$BUILD_DIR" \
    --libraries "$HOME/Documents/Arduino/libraries" \
    --warnings none \
    .

echo ""
echo "=== Done ==="
BIN="$BUILD_DIR/AutoFOV_V12.ino.bin"
# Flash-usage gate. arduino-cli only errors at 100 % of the slot and
# drift-check only warns at push time; a build that quietly creeps to the
# ceiling (e.g. an un-minified embed) would then fail every OTA on-device.
if [ -f "$BIN" ]; then
    BIN_SIZE=$(wc -c < "$BIN" | tr -d ' ')
    PCT=$(( BIN_SIZE * 100 / APP_MAX_SIZE ))
    FREE_KB=$(( (APP_MAX_SIZE - BIN_SIZE) / 1024 ))
    if [ -t 1 ]; then RED=$'\033[31m'; YEL=$'\033[33m'; RST=$'\033[0m'; else RED=""; YEL=""; RST=""; fi
    echo "flash: ${PCT}% of OTA slot (${BIN_SIZE} / ${APP_MAX_SIZE} B, ${FREE_KB} KB free)"
    if [ "$PCT" -ge 97 ] && [ "$FORCE" != 1 ]; then
        echo "${RED}ERROR: binary is at ${PCT}% of the OTA slot — refusing to produce a near-full image.${RST}"
        echo "       Trim (check the embed line for [js off]) or rerun with: bash tools/build.sh --force"
        exit 1
    elif [ "$PCT" -ge 90 ]; then
        echo "${YEL}WARNING: binary is at ${PCT}% of the OTA slot — headroom is getting thin.${RST}"
    fi
fi
if [ -n "$VERSION" ] && [ -f "$BIN" ]; then
    VERSIONED="$BUILD_DIR/AutoFOV_V12_${VERSION}.bin"
    rm -f "$BUILD_DIR"/AutoFOV_V12_v*.bin   # only the current build's versioned copy is kept
    cp "$BIN" "$VERSIONED"
    echo "Binary: $VERSIONED"
    echo "        (also $BIN)"
else
    echo "Binary: $BIN"
fi
echo "OTA flash via web UI → WiFi Info → FIRMWARE UPDATE"
