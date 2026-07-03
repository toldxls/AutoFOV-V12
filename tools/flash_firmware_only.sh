#!/bin/bash
# Full USB recovery flash — bootloader + partition table + otadata reset + firmware.
# Use this instead of Arduino IDE Upload, or to recover from a bootloop.
# Leaves the NVS region (0x9000) untouched: calibration + WiFi creds survive.
# Hold BOOT, press RESET, release BOOT to enter bootloader mode first.
#
# Usage: bash tools/flash_firmware_only.sh [port]
# Default port: /dev/cu.usbmodem101

# Newest installed esp32 core / esptool — hardcoded version paths broke the
# moment the core was updated, i.e. exactly when a recovery was needed.
ESPTOOL=$(ls -1 "$HOME"/Library/Arduino15/packages/esp32/tools/esptool_py/*/esptool 2>/dev/null | sort -V | tail -1)
BOOT_APP0=$(ls -1 "$HOME"/Library/Arduino15/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin 2>/dev/null | sort -V | tail -1)
PORT="${1:-/dev/cu.usbmodem101}"
BUILD="build/esp32.esp32.adafruit_feather_esp32s3"

cd "$(dirname "$0")/.."

[ -n "$ESPTOOL" ]   || { echo "ERROR: esptool not found under ~/Library/Arduino15 — install the esp32 core."; exit 1; }
[ -n "$BOOT_APP0" ] || { echo "ERROR: boot_app0.bin not found under ~/Library/Arduino15 — install the esp32 core."; exit 1; }

for f in "$BUILD/AutoFOV_V12.ino.bin" "$BUILD/AutoFOV_V12.ino.bootloader.bin" "$BUILD/AutoFOV_V12.ino.partitions.bin"; do
    [ -f "$f" ] || { echo "ERROR: $f not found — run bash tools/build.sh first."; exit 1; }
done

set -e
echo "Flashing bootloader + partition table + otadata + firmware -> $PORT"
"$ESPTOOL" --chip esp32s3 --port "$PORT" --baud 921600 \
    --before no-reset --after hard-reset \
    write-flash \
    0x0     "$BUILD/AutoFOV_V12.ino.bootloader.bin" \
    0x8000  "$BUILD/AutoFOV_V12.ino.partitions.bin" \
    0xe000  "$BOOT_APP0" \
    0x10000 "$BUILD/AutoFOV_V12.ino.bin"
