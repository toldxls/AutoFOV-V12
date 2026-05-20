#!/bin/bash
# Full USB recovery flash — partition table + otadata reset + firmware.
# Use this instead of Arduino IDE Upload, or to recover from a bootloop.
# Hold BOOT, press RESET, release BOOT to enter bootloader mode first.
#
# Usage: bash tools/flash_firmware_only.sh [port]
# Default port: /dev/cu.usbmodem101

ESPTOOL="/Users/travis/Library/Arduino15/packages/esp32/tools/esptool_py/5.2.0/esptool"
BOOT_APP0="/Users/travis/Library/Arduino15/packages/esp32/hardware/esp32/3.3.8/tools/partitions/boot_app0.bin"
PORT="${1:-/dev/cu.usbmodem101}"
BUILD="build/esp32.esp32.adafruit_feather_esp32s3"

cd "$(dirname "$0")/.."

if [ ! -f "$BUILD/AutoFOV_V12.ino.bin" ]; then
    echo "ERROR: firmware binary not found — run bash tools/build.sh first."
    exit 1
fi

set -e
echo "Flashing partition table + otadata + firmware -> $PORT"
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 921600 \
    --before no-reset --after hard-reset \
    write-flash \
    0xe000  "$BOOT_APP0" \
    0x8000  "$BUILD/AutoFOV_V12.ino.partitions.bin" \
    0x10000 "$BUILD/AutoFOV_V12.ino.bin"
