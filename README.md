# AutoFOV V12

AutoFOV is a specialized photomicroscopy field-of-view (FOV) calculator and automated focus stacking assistant with wireless camera trigger. Powered by an ESP32-S3, it uses a Time-of-Flight (ToF) sensor to measure bellows separation distance and calculates a real-time FOV based on a calibrated linear regression model for any bellows/magnification.

V12 adds a full real-time vibration monitoring and analysis system using the onboard LSM6DSOX IMU, with per-axis FFT spectral analysis, a settle-time estimator, and a high-resolution web analyzer panel.

V12.3 adds **photo-assisted calibration** — drop a stage-micrometer JPEG on the web dashboard and it measures the pixel count across the demarcation window automatically (periodic tick detection, sub-pixel centroids, and image deskew), then auto-fills the calibration point. Calibrations can also be exported/imported and shared as `.json`/`.txt`.

V12.4 adds an **MTF / resolution analyzer** — drop one or more USAF-1951 target shots on the dashboard and it measures the system's slanted-edge MTF, plots it against the objective's diffraction limit, fits the **equivalent NA** the system actually performs at, and reports MTF50 / MTF10 / MTF5 in lp/mm or µm. Accepts JPEG, PNG, and baseline TIFF; scores multiple shots for focus and picks the sharpest automatically.

## Features

* **Real-Time FOV Calculation:** Uses a VL53L4CX ToF sensor to measure distance and computes FOV for 5×, 10×, and 20× objectives.
* **Custom Calibration:** Calibrate the device to your specific optical setup using a stage micrometer. Count the pixels by hand, or — on the web dashboard — drop a micrometer JPEG and **Measure from Photo** finds the periodic ticks, deskews any stage tilt, and fills the pixel count automatically (refusing any result it can't trust). Calculates standard error (RMSE) and R² of the fit, and points can be exported/imported as `.json`/`.txt`.
* **Stack Calculator:** Automatically calculates image overlap % and total depth from step size, and required image count based on objective NA and depth of field.
* **Stack-Complete Notifications:** Pushes a `stackDone` WebSocket event to every connected dashboard (browser Notification + audible alert beeps) and, optionally, an ntfy.sh push so a phone away from the bench can hear the stack finish. A dashboard toast also offers a one-click **Save PNG** summary card with run stats, vibration RMS, and per-axis estimated pixel blur.
* **Web Dashboard & Telemetry:** Connects to WiFi to provide a ~30 Hz live data stream (distance, FOV, ToF signal rate, vibration) and full remote control of settings.
* **Captive Portal Setup:** First-time WiFi setup via a built-in access point (AutoFOV-Setup).
* **PSRAM-Backed UI:** Fluid, flicker-free UI using off-screen 16-bit sprites buffered in PSRAM.
* **Customization:** Adjustable screen brightness, sleep timeouts, and multiple color themes (Classic, Midnight, Forest, Daylight) with adjustable tint.
* **MTF / Resolution Analyzer (V12.4):** Slanted-edge MTF from a USAF-1951 target shot, on the web dashboard. Auto-scales from the known bar pitch, measures both edges of a bar and averages them, plots measured vs diffraction-limited ideal, fits an equivalent NA, subtracts the measured noise floor, and marks MTF50 / MTF10 (practical limit) / MTF5 (eye extinction).
* **Lens Centering Check (V12.3):** Drop a stage-micrometer shot and the dashboard measures lateral chromatic-aberration shift across the field to verify the lens is centered.
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
4. **Dashboard:** Enter the device IP in any browser. The full remote-control UI is served straight from the firmware binary (gzip-embedded — no LittleFS upload).

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

**Vibration Signatures** — capture a reference spectrum over a 10-second window using peak-hold (not averaging, so transient events still register) with the ambient noise floor subtracted, and save it to LittleFS (`/vibsig/*.bin`). Signatures persist across power cycles and are listed in the web dashboard on connect.

**Settle Time** — after each A4 shutter pulse, a Goertzel analysis fits the ring-down decay to estimate how long the bench needs to settle. The result is shown as the recommended wait time and can be used to tune the configured inter-shot delay.

### Calibration

Calibration fits the distance→FOV curve to your bellows/camera from a few points
across the focus range. (Both the device and the dashboard CALIBRATOR page have an
**i** button with in-app help.)

1. **CALIBRATE** on the main screen (or the dashboard's CALIBRATOR page).
2. Set **Photo Width** (your camera's pixel width, e.g. 6960) and **Demarc Dist** —
   the real width in mm you measure pixels across (default 0.4 mm).
3. **START CAL**, then at each bellows distance fill in the pixel count spanning the
   demarcation on a stage-micrometer shot: type it, or on the dashboard tap
   **📷 MEASURE FROM PHOTO** and drop a micrometer JPEG to measure it automatically.
   **CAPTURE & SAVE** pairs that pixel count with the live ToF distance.
4. Move the bellows (~10 mm) and repeat across the range. Stop anytime and **FINISH**
   (3–20 points); **REVIEW** to check or RETAKE a point.
5. The device fits a pixel-space linear regression (`pixels = slope·dist + intercept`)
   and saves slope, intercept, R², and RMSE to NVS.

**Measure from Photo (dashboard).** Decodes a JPEG, detects the periodic micrometer
ticks, deskews any stage tilt, measures the pixels across the demarcation window, and
auto-fills the pixel field — pairing with the device's live ToF reading at capture.
It refuses to apply a measurement it can't trust (too few ticks for the declared span,
or high spacing error). JPEG only (browsers can't decode RAW): shoot JPEG or RAW+JPEG,
a full **uncropped** frame; export RAW→JPEG first if needed. The dashboard
**CALIBRATION POINTS** view lists the active/factory points and exports them as `.txt`,
and **CAL I/O** backs up, restores, or shares a calibration via `.json`/`.txt`.

### MTF / Resolution (dashboard)

Open **MTF** from the dashboard's calibration tools. The tool measures the real
resolving power of the whole imaging train (objective + bellows + relay + sensor)
from a photo of a USAF-1951 resolution target, using the slanted-edge method.

**Shooting.** Pick an element whose bars fill most of the frame (e.g. Group 4
Element 2 at 20×), tilt the target ~5° so the edges are slanted, best focus,
lowest ISO. JPEG, PNG, and baseline TIFF (uncompressed / LZW / PackBits, 8- or
16-bit) all work — for honest high-frequency numbers shoot RAW and export an
unsharpened TIFF, since in-camera JPEG sharpening inflates the MTF tail.

**Measuring.** Set **Grp/Elem** to the element photographed — its known bar
pitch calibrates µm/px automatically (needs ≥2 bars in frame). Drop one or
several shots: each is focus-scored, thumbnails appear with sharp/uniform
metrics, the sharpest is starred and pre-selected, and a green heatmap shows
where the shot is sharpest — the measurement box is pre-placed there. Drag the
box across one **whole bar** (both edges are measured and averaged) or onto a
single edge, then **MEASURE**. Bad placements are rejected with a specific hint
rather than a wrong number.

**Reading the results.**
* **MTF50** — perceived-sharpness benchmark; the headline number.
* **MTF10** — practical resolution limit (≈ Rayleigh 9% contrast); shown also
  as *min sep* in µm (1000/f — one line-pair period, the same convention as
  objective-datasheet "resolving power").
* **MTF5** — approximate by-eye extinction contrast.
* **equiv NA** — the diffraction-limited NA whose ideal curve best fits the
  measurement: what the system *performs like*. A lower bound on the true
  aperture, and the right value for the calculator's effective-NA input.
* The plot draws the measured curve against the active objective's ideal (blue),
  the effective-NA ideal (grey), and the equiv-NA fit (green); the x-axis is
  capped at the objective's diffraction limit and toggles between lp/mm and µm.
  The measured noise floor (read beyond the cutoff, where true MTF must be zero)
  is subtracted before the markers are placed, and a ⚠ warning flags any
  physically impossible reading (wrong Grp/Elem or objective selected).

## License

Copyright © 2026 Travis Olds.

AutoFOV is released as **source-available** under the
[PolyForm Noncommercial License 1.0.0](LICENSE.md). You are free to use, study,
modify, and share it for any **noncommercial** purpose — personal projects,
hobby and amateur use, education, research, and noncommercial organizations.

**Commercial use is not permitted without a separate license from the author.**
That includes selling devices that run this firmware, bundling it into a product
or paid service, or otherwise using it to generate revenue. To use AutoFOV
commercially, contact the author to arrange a commercial license.

Third-party libraries (Arduino-ESP32 core, Adafruit GFX/sensor libraries,
ESPAsyncWebServer, AsyncTCP, etc.) remain under their own respective licenses;
this notice covers only the original AutoFOV source in this repository.
