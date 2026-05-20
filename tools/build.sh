#!/bin/bash
# One-command build: regenerates web_ui.h then compiles the sketch.
# Run after committing all changes; OTA flash the resulting .bin.
# Usage: bash tools/build.sh

set -e
cd "$(dirname "$0")/.."

ARDUINO_CLI="$HOME/bin/arduino-cli"
FQBN="esp32:esp32:adafruit_feather_esp32s3:PSRAM=enabled,PartitionScheme=custom,LoopCore=1,EventsCore=1"
BUILD_DIR="build/esp32.esp32.adafruit_feather_esp32s3"

echo "=== Step 1: embed HTML ==="
python3 tools/embed_html.py

echo "=== Step 2: compile ==="
"$ARDUINO_CLI" compile \
    --fqbn "$FQBN" \
    --build-path "$(pwd)/$BUILD_DIR" \
    --libraries "$HOME/Documents/Arduino/libraries" \
    --warnings none \
    .

echo ""
echo "=== Done ==="
echo "Binary: $BUILD_DIR/AutoFOV_V12.ino.bin"
echo "OTA flash via web UI → WiFi Info → FIRMWARE UPDATE"
