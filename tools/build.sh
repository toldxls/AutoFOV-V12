#!/bin/bash
# One-command build: regenerates web_ui.h then compiles the sketch.
# Run after committing all changes; OTA flash the resulting .bin.
# Usage: bash tools/build.sh

set -e
cd "$(dirname "$0")/.."

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
#   upload.maximum_size=1900544  the real app0/app1 slot (0x1D0000 = 1856 KB), so
#                                the build-time size guard reads "80% / 1900544"
#                                exactly as before instead of `default`'s 1.2 MB.
FQBN="esp32:esp32:adafruit_feather_esp32s3:PSRAM=enabled,PartitionScheme=default,LoopCore=1,EventsCore=1"
BUILD_DIR="build/esp32.esp32.adafruit_feather_esp32s3"

# Optional minify deps — warn (non-fatal) if absent so the ~46 KB JS/CSS/HTML
# minify in embed_html.py doesn't silently fall back to an un-minified embed.
if ! python3 -c "import rjsmin, rcssmin" 2>/dev/null; then
    echo "WARNING: rjsmin/rcssmin missing — HTML will embed un-minified (~46 KB larger)."
    echo "         install with: pip3 install -r tools/requirements.txt"
fi

echo "=== Step 1: embed HTML ==="
python3 tools/embed_html.py

echo "=== Step 2: compile ==="
"$ARDUINO_CLI" compile \
    --fqbn "$FQBN" \
    --build-property "build.partitions=" \
    --build-property "upload.maximum_size=1900544" \
    --build-path "$(pwd)/$BUILD_DIR" \
    --libraries "$HOME/Documents/Arduino/libraries" \
    --warnings none \
    .

echo ""
echo "=== Done ==="
echo "Binary: $BUILD_DIR/AutoFOV_V12.ino.bin"
echo "OTA flash via web UI → WiFi Info → FIRMWARE UPDATE"
