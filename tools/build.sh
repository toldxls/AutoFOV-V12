#!/bin/bash
# One-command build: regenerates web_ui.h then compiles the sketch.
# Run after committing all changes; OTA flash the resulting .bin.
# Usage: bash tools/build.sh

set -e
cd "$(dirname "$0")/.."

ARDUINO_CLI="$HOME/bin/arduino-cli"
FQBN="esp32:esp32:adafruit_feather_esp32s3:PSRAM=enabled,PartitionScheme=custom,LoopCore=1,EventsCore=1"
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
    --build-path "$(pwd)/$BUILD_DIR" \
    --libraries "$HOME/Documents/Arduino/libraries" \
    --warnings none \
    .

echo ""
echo "=== Done ==="
echo "Binary: $BUILD_DIR/AutoFOV_V12.ino.bin"
echo "OTA flash via web UI → WiFi Info → FIRMWARE UPDATE"
