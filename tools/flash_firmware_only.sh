#!/bin/bash
# Flash only the firmware binary at 0x10000 — does NOT touch the partition
# table at 0x8000. Use this instead of Arduino IDE Upload to preserve the
# dual-OTA partition layout.
#
# Usage: bash tools/flash_firmware_only.sh [port]
# Default port: /dev/cu.usbmodem2101

ESPTOOL="/Users/travis/Library/Arduino15/packages/esp32/tools/esptool_py/5.2.0/esptool"
PORT="${1:-/dev/cu.usbmodem2101}"
BIN="build/esp32.esp32.adafruit_feather_esp32s3/AutoFOV_V12.ino.bin"

if [ ! -f "$BIN" ]; then
    echo "ERROR: $BIN not found — compile in Arduino IDE first."
    exit 1
fi

set -e
echo "Flashing $BIN -> $PORT (0x10000 only, partition table preserved)"
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 921600 \
    --before default-reset --after hard-reset \
    write-flash 0x10000 "$BIN"
