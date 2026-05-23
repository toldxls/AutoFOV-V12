# AutoFOV V12

AutoFOV is a specialized photomicroscopy field-of-view (FOV) calculator and automated focus stacking assistant with wireless camera trigger. Powered by an ESP32-S3, it uses a Time-of-Flight (ToF) sensor to measure bellows separation distance and calculates a real-time FOV based on a calibrated linear regression model for any bellows/magnification.

V12 adds a full real-time vibration monitoring and analysis system using the onboard LSM6DSOX IMU, with per-axis FFT spectral analysis, a settle-time estimator, and a high-resolution web analyzer panel.

## Features

* **Real-Time FOV Calculation:** Uses a VL53L4CX ToF sensor to measure distance and computes FOV for 5×, 10×, and 20× objectives.
* **Custom Calibration:** Calibrate the device to your specific optical setup using a scaled micrometer. Calculates standard error (RMSE) and R² of the calibration fit.
* **Stack Calculator:** Automatically calculates image overlap % and total depth from step size, and required image count based on objective NA and depth of field.
* **Stack-Complete Notifications:** Pushes a `stackDone` WebSocket event to every connected dashboard (browser Notification + audible alert beeps) and, optionally, an ntfy.sh push so a phone away from the bench can hear the stack finish.
* **Web Dashboard & Telemetry:** Connects to WiFi to provide a ~30 Hz live data stream (distance, FOV, ToF signal rate, vibration) and full remote control of settings.
* **Captive Portal Setup:** First-time WiFi setup via a built-in access point (AutoFOV-Setup).
* **PSRAM-Backed UI:** Fluid, flicker-free UI using off-screen 16-bit sprites buffered in PSRAM.
* **Customization:** Adjustable screen brightness, sleep timeouts, and multiple color themes (Classic, Midnight, Forest, Daylight) with adjustable tint.
* **Vibration Monitor (V12):** Dual-core DSP pipeline on the LSM6DSOX. Separates acceleration into vertical and horizontal-plane channels by projecting out gravity, runs a 512-point FFT on each, and tracks dominant frequency, per-band RMS, and a Goertzel-fit settle-time estimate after each shutter pulse. Vibration signatures can be captured and saved to LittleFS for comparison.

## Hardware Requirements

* **Microcontroller:** ESP32-S3 (requires 2 MB+ PSRAM and 4 MB+ Flash)
* **Display:** 2.8" ILI9341 TFT SPI Display
* **Touch Controller:** FT6206 Capacitive Touch (I2C)
* **Sensor:** VL53L4CX Time-of-Flight Sensor (I2C)
* **IMU:** LSM6DSOX 6-axis accelerometer/gyro (I2C)
* **Trigger Interface:** Optocoupler sensing the stepper controller's shutter release

### Pin Configuration

| Component | Pin (ESP32-S3) | Notes |
| :--- | :--- | :--- |
| **TFT CS** | 9 | SPI Chip Select |
| **TFT DC** | 10 | SPI Data/Command |
| **TFT RST** | 6 | SPI Reset |
| **TFT Backlight** | A2 | PWM brightness control |
| **Touch Int** | 11 | Active-low interrupt |
| **Camera Trigger** | A4 | Input pullup |
| **Trigger LED** | A3 | Active-low PWM output (3.3 V → LED → A3) |
| **IR LED** | 12 | NEC IR transmit to MJKZZ controller (GPIO12 → 100 Ω → IR LED → GND) |
| **I2C SDA / SCL** | Default | Shared by FT6206, VL53L4CX, and LSM6DSOX (0x6A/0x6B) |

## Software Dependencies

Install via the Arduino Library Manager or manually:

* `Adafruit_GFX`
* `Adafruit_ILI9341`
* `Adafruit_FT6206`
* `vl53l4cx_class` (STM32duino)
* `ESPAsyncWebServer` (me-no-dev)
* `AsyncTCP` (me-no-dev)
* `ArduinoJson` v6.x
* `IRremoteESP8266`
* `Adafruit LSM6DS` + `Adafruit Unified Sensor`
* `arduinoFFT`

## Installation & Flashing

Firmware is built with `arduino-cli` via `tools/build.sh` (the Arduino IDE is
only needed for the Serial Monitor and editing). The web dashboard in
`data/index.html` is gzip-embedded into the firmware binary — there is **no**
separate LittleFS upload for it. The two `.ino` files compile as a single
translation unit (the build concatenates them alphabetically, so
`AutoFOV_V12.ino` precedes `AutoFOV_V12_wifi.ino`).

### Build settings

The build targets the Adafruit Feather ESP32-S3 with a **custom dual-OTA
partition table** (`tools/partitions.csv`). `tools/build.sh` passes the correct
FQBN automatically; if building from the Arduino IDE instead, match:

| Setting | Value |
| :--- | :--- |
| Board | Adafruit Feather ESP32-S3 (4 MB Flash, 2 MB PSRAM) |
| PSRAM | Enabled (QSPI PSRAM) |
| Partition Scheme | Custom — `tools/partitions.csv` (dual-OTA) |
| Arduino Runs On | Core 1 |
| Events Run On | Core 1 |

### Building and flashing

```
git add … && git commit
bash tools/build.sh
```

Flash the resulting `.bin` through the web dashboard (WiFi Info → FIRMWARE →
CHOOSE .bin), or publish a release with `bash tools/release.sh` and use the
dashboard's **Update from GitHub** button. Do **not** use the Arduino IDE's
Upload button — it overwrites the partition table and breaks the dual-OTA
layout.

### Recovery

If a device won't boot or its dashboard is unreachable, re-flash it over USB
with the browser-based recovery flasher at
<https://toldxls.github.io/AutoFOV-V12/> (Chrome or Edge, plus a
data-capable USB-C cable). It rewrites the bootloader, partition table and
firmware while keeping calibration and WiFi settings.

## Usage

### WiFi & Web Dashboard

1. **Initial boot:** No credentials saved → device starts in Captive Portal mode.
2. **Connect:** Join the `AutoFOV-Setup` network; the setup page appears automatically.
3. **Configure:** Enter your WiFi credentials. The device saves them, restarts, and connects. Assign a static high IP (e.g. 192.168.1.250) via your router for easy access.
4. **Dashboard:** Enter the device IP in any browser. The full remote control UI loads from LittleFS.

### Stack-Complete Notification

The web dashboard subscribes to a `stackDone` WebSocket event the moment the device declares a stack finished. The browser fires a desktop Notification (if granted) and plays three audible beeps via WebAudio.

Optionally, set an [ntfy.sh](https://ntfy.sh) topic from the dashboard's **Auto Remote** screen — when a stack finishes, the device POSTs a high-priority push notification to that topic so any phone subscribed in the ntfy app gets pinged without needing a tab open.

### Stack-Complete Thresholds

| Limit | Value | Role |
| :--- | :--- | :--- |
| Pulse debounce | 15 ms | A4 must read LOW for >15 ms to count as one shutter pulse. |
| `MIN_ACTIVE_DURATION` | 20 s | Stack must be active at least this long (first pulse → last pulse) or the completion event is suppressed. |
| `SILENCE_DURATION` | 5 s | This much silence after the last A4 pulse declares the stack complete. Raise if your inter-shot interval is longer. |

### Vibration Monitor

Access via **SETTINGS → VIBRATION** on the device or web dashboard. The state badge shows **CALM** (green) / **MODERATE** (yellow) / **STRONG** (red).

**Basic screen** — dominant frequency, vertical and horizontal RMS (mg), recommended settle wait, and a spectrum thumbnail. Tap the thumbnail (or the **▶ ANALYZE** hint) to open the full analyzer.

**Expanded web analyzer panel:**
* *Spectrum* — 256-bin FFT for vertical (cyan) and horizontal (pink) channels. mg mode shows peak-hold envelope + live overlay + running-average. µm mode uses running-average as primary to prevent 1/f² transient amplification.
* *Waterfall* — vertical spectrum vs time, newest row on top. Adjustable sensitivity gain slider.
* *RMS Trend* — 90-second rolling plot of vertical and horizontal RMS with a 10-second moving-average marker on the y-axis.
* *Wait Analysis* — 90-second timeline of the firmware's recommended settle wait vs the configured wait, with a cut/short/ok delta indicator.

**Vibration Signatures** — capture a reference spectrum (averaged over 8 FFT frames) and save it to LittleFS (`/vibsig/*.bin`). Signatures persist across power cycles and are listed in the web dashboard on connect.

**Settle Time** — after each A4 shutter pulse, a Goertzel analysis fits the ring-down decay to estimate how long the bench needs to settle. The result is shown as the recommended wait time and can be used to tune the configured inter-shot delay.

### Calibration

1. Tap **CALIBRATE** on the main screen.
2. Set photo width (pixels) and demarcation distance (default 0.4 mm).
3. Capture 3–20 points at varying distances.
4. The device fits a linear regression and saves slope, intercept, R², and RMSE to NVS.
