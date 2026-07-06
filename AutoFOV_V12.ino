// =============================================================================
// AutoFOV — photomicroscopy FOV calculator & focus-stacking assistant
// Copyright © 2026 Travis Olds
//
// Licensed under the PolyForm Noncommercial License 1.0.0 — noncommercial use
// only; commercial use requires a separate license from the author.
// Full terms: see LICENSE.md or https://polyformproject.org/licenses/noncommercial/1.0.0
// =============================================================================
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_FT6206.h>
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <Preferences.h>
#include <atomic>
#include <new>          // V12.3: std::nothrow for the transient info-overlay canvas

// WiFi / web-server headers pulled in HERE (not in the _wifi tab) so that the
// Arduino preprocessor's auto-generated forward declarations can see types like
// AsyncWebSocket / AsyncWebSocketClient that appear in the wifi tab's signatures.
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <esp_ota_ops.h>   // esp_ota_get_running_partition() — real app-slot size for the Flash readout
#include <esp_system.h>    // V12.5: esp_reset_reason() — boot diagnostics + boot-loop rollback
#include <esp_task_wdt.h>  // V12.5: loopTask hardware watchdog (Core-1 freeze backstop)
#include <esp_core_dump.h> // V12.5.x: read the last panic's PC/task/backtrace (flash coredump)
#include <esp_app_desc.h>  // V12.5.x: esp_app_get_elf_sha256() — flag a crash from a different build
#include <LittleFS.h>
#include <ArduinoJson.h>

#include <IRremoteESP8266.h>   // patched3: IR LED transmit for the MJKZZ remote
#include <IRsend.h>

#include <Adafruit_LSM6DSOX.h>  // V12: LSM6DSOX 6-axis IMU — bench-vibration monitor
#include <arduinoFFT.h>         // V12: 512-pt FFT for the vibration spectrum (v2.x API)

#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include "data/FreeSans7pt7b.h"   // local — used for the main-screen CALIBRATE button
// V12.5: pulled in here (main) as well as the wifi tab so BUILD_VERSION is
// defined for setup()'s early diag block. #pragma once makes the wifi tab's
// include (data/web_ui.h) a no-op — the gzip PROGMEM array is emitted once.
#include "data/web_ui.h"

#define TFT_BLACK         0x0000
#define TFT_WHITE         0xFFFF
#define COLOR_LIGHTGREY   0xC618
#define COLOR_DARKGREY    0x7BEF
#define COLOR_GREENYELLOW 0xAFE5
#define COLOR_PUREGREEN   0x07E0 
#define COLOR_DARKGREEN   0x0320
#define COLOR_YELLOW      0xFFE0 
#define COLOR_ORANGE      0xFD20 
#define COLOR_RED         0xF800 
#define COLOR_MAROON      0x7800
#define COLOR_BLUEGREEN   0x0452 
#define COLOR_DARKBLUE    0x0019 
#define COLOR_PURPLE      0xA81F 
#define COLOR_TEAL        0x04B1 
#define COLOR_DARKTEAL    0x0248 

#define MUTED_5X          0x5000 
#define MUTED_10X         0x39E7
#define MUTED_20X         0x000A

// Centralized firmware version string. Update once per release; every screen
// that prints the build label reads from here so they can never drift apart.
#define FIRMWARE_VERSION  "Auto FOV V12"

namespace Config {
  // V12.3 pixel-space calibration: pixels = CTRL_X*dist + CTRL_Y, then
  // FOV = demarcation * sensorWidth / pixels. Pixels are linear in distance, so
  // this straight-line fit is accurate (FOV-vs-distance is the resulting 1/x
  // curve). Defaults are the least-squares pixel-space fit of the reference
  // dataset below (FACTORY_DIST/FACTORY_FOV), R²≈0.9991.
  // 12.3: re-derived from a 14-point photo-measured set (22–157 mm, 0.4 mm
  // demarcation). The prior 6.3725/1227.14 fit was a 0.3 mm-demarcation dataset
  // labelled 0.4 mm, so the factory default read ~33% high; this corrects it.
  constexpr float DEFAULT_CTRL_X = 8.40325f;      // px per mm (pixel slope)
  constexpr float DEFAULT_CTRL_Y = 1652.202f;     // px at distance 0 (pixel intercept)
  constexpr float DEFAULT_CALIB_ERROR = 0.0060f;  // calibration RMSE in FOV mm
  constexpr float DEFAULT_SENSOR_WIDTH_PX = 6960.0f;
  constexpr float DEFAULT_DEMARCATION_MM = 0.4f;
  constexpr int   DEFAULT_TEMP_PIXELS = 1800;
  constexpr int   MIN_CALIB_POINTS = 3;
  constexpr int   MAX_CALIB_POINTS = 20;
  constexpr int   DEFAULT_BRIGHTNESS = 128;
}

// --- Optical Constants for Stack Calculator ---
const float NA_5X  = 0.14f;
const float NA_10X = 0.28f;
const float NA_20X = 0.42f;
const float WAVELENGTH_UM = 0.55f;

// --- Stack Calculator Motor Constants ---
const float BASE_STEP_UM = 0.1562f;

const float STEP_TABLE[] = {
  0.156f, 0.312f, 0.468f, 0.625f, 0.781f,
  0.937f, 1.093f, 1.250f, 1.406f, 1.562f,
  1.718f, 1.875f, 2.031f, 2.187f, 2.344f, 2.500f, 2.656f, 
  2.812f, 2.968f, 3.125f, 3.281f, 3.437f, 3.593f,
};
const int STEP_TABLE_LEN = 23;

// Factory reference calibration (14 points) used for the graph/seed when no
// custom calibration has been run. Stored as (distance mm, FOV mm); FOV here is
// 0.4*6960/pixels from the measured pixel counts. DEFAULT_CTRL_X/Y above are the
// pixel-space least-squares fit of these points.
// 12.3: photo-measured set (auto tick detection + deskew), 0.4 mm demarcation.
const int   FACTORY_N = 14;
const float FACTORY_DIST[14] = {
  22.0f, 32.0f, 42.0f, 52.0f, 62.0f, 72.0f, 82.0f,
  92.0f, 102.0f, 112.0f, 122.0f, 132.0f, 142.0f, 157.0f
};
const float FACTORY_FOV[14] = {
  1.5341f, 1.4434f, 1.3883f, 1.3315f, 1.2788f, 1.2323f, 1.1856f,
  1.1491f, 1.1058f, 1.0720f, 1.0385f, 1.0075f, 0.9744f, 0.9452f
};

int stackStepIndex = 2;

// --- Sleep / Dim Constants ---
// V11: timeouts are now runtime-adjustable from the SCREEN TIMEOUTS submenu.
// Defaults match the original const values. Persisted under a separate
// Preferences key so the existing CalibData magic doesn't have to change.
const unsigned long DIM_TIMEOUT_DEFAULT   = 240000UL;
const unsigned long SLEEP_TIMEOUT_DEFAULT = 600000UL;
unsigned long dimTimeoutMs   = DIM_TIMEOUT_DEFAULT;
unsigned long sleepTimeoutMs = SLEEP_TIMEOUT_DEFAULT;
const int DIM_BRIGHTNESS = 5;

// --- V11: Color Themes ---
// Lightweight theming: a theme overrides the screen background and the
// title-box accent color. Most other accents stay as their compiled defines —
// this keeps the diff small while still giving each theme a distinctive feel.
// Apply by replacing tft.fillScreen(THEME_BG) with tft.fillScreen(themeBg)
// and passing themeTitleBg into drawLeftBoxedText.
struct Theme {
  const char* name;
  uint16_t bg;          // screen background (replaces TFT_BLACK fills)
  uint16_t titleBg;     // accent stripe behind the screen title
  uint16_t swatch;      // small color shown on the picker button
};

// V12.3: one paragraph of a scrollable info overlay (SECURITY TIPS / RECOVERY
// FLASH). Declared up here — ahead of the Arduino auto-prototype insertion point
// (~line 178) — so the generated buildInfoLines() prototype sees the type. The
// content arrays + renderer live down by handleWifiInfoTouch().
#define INFO_BODY 0
#define INFO_HEAD 1
#define INFO_WARN 2
struct InfoBlock { const char* text; uint8_t kind; };

// Index aligns with the order the buttons appear on the THEME picker.
const Theme THEMES[] = {
  // CLASSIC: pitch-black, deep-blue title — original look unchanged.
  { "CLASSIC",  0x0000, 0x0019, 0x041F },
  // MIDNIGHT: dark indigo bg, royal-blue title.
  { "MIDNIGHT", 0x0009, 0x18B2, 0x18B2 },
  // FOREST: dark forest-green bg, bright-green title accent.
  { "FOREST",   0x0120, 0x0320, COLOR_PUREGREEN },
  // DAYLIGHT: medium light-grey bg with dark text. The bg is intentionally
  // not pure white — going a notch grey gives existing light accents more
  // breathing room and reduces eye-strain for prolonged use.
  { "DAYLIGHT", 0xBDF7, 0x0019, 0xBDF7 }
};
const int NUM_THEMES = sizeof(THEMES) / sizeof(THEMES[0]);
const int THEME_DAYLIGHT_IDX = 3;
int currentThemeIndex = 0;
// V11: Theme intensity. 50 = no shift; 0 = bg lightened toward white; 100 =
// bg darkened toward black. Lets the user dim a too-bright theme (DAYLIGHT)
// or boost a too-dark one without changing themes. Persisted in prefs.
int themeIntensity = 50;
// V11 deferred-save state — tint drags happen at slider speed (every 20ms);
// writing NVS that fast caused visible UI hitches and unnecessary flash wear.
// We mark dirty during drags and saveDisplayPrefs() runs from the main loop
// ~500ms after the last drag sample, or immediately on theme tap / GO BACK.
bool displayPrefsDirty = false;
unsigned long lastTintDragMs = 0;
// patched3: WS commands (brightness / LED duty / secStep) used to write the
// full CalibData blob to NVS on every message — a web slider drag is dozens
// of flash writes. Mark the "calib" namespace dirty instead; loop() flushes
// once ~500 ms after the last edit (mirrors the displayPrefsDirty pattern).
bool calibPrefsDirty = false;
unsigned long lastCalibEditMs = 0;
// patched3: when a WS-driven theme/tint change arrives we defer the TFT
// repaint by the same 500 ms window used for NVS save.  A rapid tint-drag
// stream from the HTML thus collapses into a single redraw on drag-end,
// avoiding the multi-Hz full-screen flicker a per-message redraw would cause.
bool displayNeedsRedraw = false;
// V11: tintDragActive is set while the user's finger is on the tint slider.
// During the drag we ONLY repaint the slider thumb (so it tracks the
// finger) — we don't touch the rest of the page, because partial repaints
// during a fast drag create banding/flicker that's worse than just waiting.
// On touch lift (detected in loop), the page is fully redrawn once with
// the new tint and prefs are saved.
bool tintDragActive = false;

// Apply intensity shift to an RGB565 color. Intensity is a 0..100 dial:
//   50 → identity (no change)
//   < 50 → blend toward white (lighten)
//   > 50 → blend toward black (darken)
// The shift is per-channel linear interpolation. We scale by a fraction
// (50-x)/50 toward 0 (black) when x>50, or (50-x)/50 toward 31/63/31 (white
// in 565 space) when x<50.
inline uint16_t applyIntensity(uint16_t color) {
  if (themeIntensity == 50) return color;
  uint8_t r = (color >> 11) & 0x1F;
  uint8_t g = (color >> 5)  & 0x3F;
  uint8_t b =  color        & 0x1F;
  if (themeIntensity > 50) {
    // Darken: scale each channel toward 0 by ((100 - x) / 50) — i.e., at
    // x=100 the bg is fully black, at x=50 it's unchanged.
    int darken = (100 - themeIntensity); // 0..50
    r = (r * darken) / 50;
    g = (g * darken) / 50;
    b = (b * darken) / 50;
  } else {
    // Lighten: blend toward max channels.
    int lighten = (50 - themeIntensity); // 0..50
    r = r + ((31 - r) * lighten) / 50;
    g = g + ((63 - g) * lighten) / 50;
    b = b + ((31 - b) * lighten) / 50;
  }
  return (r << 11) | (g << 5) | b;
}

// Convenience aliases — read these throughout drawing code.
// V11: THEME_BG previously called applyIntensity() on every reference; that
// macro ran 6 muls / 3 divs / branches and was hot on refresh paths (refresh
// fns blit 5+ sprite rows each, tint drags fire every 20 ms). cachedThemeBg
// holds the precomputed result and is refreshed whenever currentThemeIndex or
// themeIntensity changes (applyTheme, end-of-tint-drag, loadDisplayPrefs).
uint16_t cachedThemeBg = 0x0000;
inline void refreshCachedThemeBg() {
  cachedThemeBg = applyIntensity(THEMES[currentThemeIndex].bg);
}
#define THEME_BG       cachedThemeBg
#define THEME_TITLE_BG (THEMES[currentThemeIndex].titleBg)

// V11: light → dark color remap for the DAYLIGHT theme. When a screen needs
// text or an icon-accent color that's normally light (TFT_WHITE, GREENYELLOW,
// LIGHTGREY, etc.), it asks themedText() instead of using the literal.
// On all dark themes this is a pass-through; on DAYLIGHT it swaps for a
// dark equivalent that reads on the light bg. We don't try to remap
// boxed-button fills — those are designed to be solid contrast colors and
// already read on either bg.
inline uint16_t themedText(uint16_t orig) {
  if (currentThemeIndex != THEME_DAYLIGHT_IDX) return orig;
  switch (orig) {
    case TFT_WHITE:         return TFT_BLACK;          // primary text
    case COLOR_LIGHTGREY:   return COLOR_DARKGREY;     // muted text
    case COLOR_GREENYELLOW: return 0x0320;             // accent text → dark green
    case COLOR_PUREGREEN:   return 0x03E0;             // icon accent → mid-dark green
    case 0x07FF:            return 0x0019;             // cyan (title text) → dark blue
    case COLOR_DARKGREY:    return 0x4208;             // disabled tone → still-readable dark grey
    case COLOR_YELLOW:      return 0x82A0;             // bright yellow → dark mustard
    case 0xC81F:            return 0x780F;             // pink/magenta → dark plum
    default:                return orig;
  }
}

// --- Hardware Pins ---
const int CAMERA_TRIGGER_PIN = A4;
const int TOUCH_INT_PIN = 11; 
#define LITE_PIN A2
#define TRIGGER_LED_PIN  A3    // V17: LED that lights with the shutter trigger indicator

// patched3: IR remote emulation — the device drives an IR LED (GPIO 12, via a
// 100 Ω series resistor) to send the MJKZZ stacking-controller's NEC remote
// codes. Codes captured from the physical remote 2026-05-16; all NEC, addr 0x00.
#define IR_LED_PIN 12
constexpr uint64_t IR_MJKZZ_START   = 0xFFB04F;  // NEC cmd 0x0D
constexpr uint64_t IR_MJKZZ_STOP    = 0xFF906F;  // NEC cmd 0x09
constexpr uint64_t IR_MJKZZ_UP      = 0xFFE21D;  // NEC cmd 0x47
constexpr uint64_t IR_MJKZZ_DOWN    = 0xFFC23D;  // NEC cmd 0x43
constexpr uint64_t IR_MJKZZ_CAPTURE = 0xFF10EF;  // NEC cmd 0x08
IRsend irLed(IR_LED_PIN);
// PWM duty for the trigger LED (0-255). ESP32 GPIOs are fixed 3.3V; analogWrite
// uses PWM to reduce average power to a 3V LED without a current-limiting resistor.
// 180 ≈ 71% duty, giving a comfortable brightness well below the 3.3V peak.
// Increase toward 255 for brighter, decrease toward 80 for dim. Min ~50 is visible.
// Active-low wiring: 3.3V → LED → A3. A3 HIGH = off, A3 LOW = on.
// TRIGGER_LED_DUTY is the ON-duty (0=off, 255=full-on).
// Sent as analogWrite(255 - TRIGGER_LED_DUTY) because wiring is active-low.
// 60 ≈ 24% brightness. Raise for brighter, lower for dimmer.
#define TRIGGER_LED_DUTY 60
#define TFT_CS   9
#define TFT_DC   10
#define TFT_RST  6

Adafruit_FT6206 touch = Adafruit_FT6206();
VL53L4CX sensor(&Wire, -1);
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// V12: LSM6DSOX IMU for the bench-vibration monitor. Shares the I²C bus with
// the VL53L4CX (0x29) and the FT6206 touch controller (0x38); its own address
// is 0x6A (SA0/SDO low) or 0x6B (high). The Adafruit driver handles reset and
// the WHO_AM_I check; FIFO config + draining use direct register I/O so they
// do not depend on the library exposing a FIFO API. See vibTask below.
Adafruit_LSM6DSOX lsm;

// Forward decl so PSRAMCanvas16::begin() (defined above the metrics block) can
// count a hard allocation failure. Defined with the other health metrics below.
extern std::atomic<uint32_t> allocFailCount;

// ─── PSRAMCanvas16 — drop-in GFXcanvas16 replacement using PSRAM ─────────────
// On ESP32-S3 with 2 MB PSRAM the 6 sprites below occupy ~91 KB.  Moving them
// to PSRAM frees that internal heap for the WiFi / AsyncTCP / lwIP stack.
// ps_malloc() falls back to malloc() automatically if PSRAM is absent or full.
//
// Why not subclass GFXcanvas16?  Its pixel buffer is declared *private*, so a
// subclass cannot redirect the allocation.  We re-implement Adafruit_GFX
// directly with the same drawPixel / getBuffer interface.
// ─────────────────────────────────────────────────────────────────────────────
class PSRAMCanvas16 : public Adafruit_GFX {
public:
  // Constructor is allocation-free: ps_malloc() called during static init
  // (before app_main on several ESP32-S3 core versions) silently falls back to
  // malloc(), permanently trapping ~92 KB of internal heap.  Call begin() from
  // setup() after the PSRAM interface is fully active.
  PSRAMCanvas16(uint16_t w, uint16_t h)
      : Adafruit_GFX(w, h), _buf(nullptr) {}

  void begin() {
    if (_buf) return;
    size_t bytes = (size_t)_width * _height * 2;
    _buf = (uint16_t*)ps_malloc(bytes);
    if (!_buf) {
      Serial.printf("[PSRAM] ps_malloc(%u B) failed — using DRAM\n", (unsigned)bytes);
      _buf = (uint16_t*)malloc(bytes);
    }
    if (_buf) memset(_buf, 0, bytes);
    else allocFailCount.fetch_add(1, std::memory_order_relaxed);   // #5 hard fail (sprite disabled)
  }
  ~PSRAMCanvas16() { free(_buf); }

  // Required virtual (per-pixel write)
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (!_buf || (unsigned)x >= (unsigned)_width || (unsigned)y >= (unsigned)_height) return;
    _buf[(int32_t)y * _width + x] = color;
  }

  // Hot-path bulk fills — avoid per-pixel overhead
  void fillScreen(uint16_t color) override {
    if (!_buf) return;
    uint32_t n = (uint32_t)_width * _height;
    for (uint32_t i = 0; i < n; i++) _buf[i] = color;
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    if (!_buf || w <= 0 || h <= 0) return;
    int16_t x1 = (x < 0) ? 0 : x,       y1 = (y < 0) ? 0 : y;
    int16_t x2 = (x + w > _width)  ? _width  : (x + w);
    int16_t y2 = (y + h > _height) ? _height : (y + h);
    for (int16_t row = y1; row < y2; row++) {
      uint16_t* p = _buf + (int32_t)row * _width + x1;
      for (int16_t col = x1; col < x2; col++) *p++ = color;
    }
  }
  void writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    if (!_buf || y < 0 || y >= _height || w <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > _width) w = _width - x;
    if (w <= 0) return;
    uint16_t* p = _buf + (int32_t)y * _width + x;
    for (int16_t i = 0; i < w; i++) p[i] = color;
  }
  void writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    if (!_buf || x < 0 || x >= _width || h <= 0) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > _height) h = _height - y;
    if (h <= 0) return;
    uint16_t* p = _buf + (int32_t)y * _width + x;
    for (int16_t i = 0; i < h; i++, p += _width) *p = color;
  }

  uint16_t* getBuffer() const { return _buf; }
  bool      allocated()  const { return _buf != nullptr; }

private:
  uint16_t* _buf;
};

// patched3: sprites now backed by PSRAM when available (see PSRAMCanvas16 above).
// All call sites remain identical — getBuffer() / tft.drawRGBBitmap() unchanged.
// V11: fovSprite shrunk from 240×60 to 240×48 — the FOV value text only
// occupies sprite-y ≈ 18..45 (FreeSans24 glyphs at baseline=42), so the
// extra 12 rows were always empty. Saves ~5.6 KB of internal heap.
PSRAMCanvas16 fovSprite(240, 48);
PSRAMCanvas16 distSprite(240, 45);
PSRAMCanvas16 menuSprite(80, 40);
PSRAMCanvas16 barSprite(44, 12);
// V11 SCREEN_TIMEOUT value strip — 120 wide × 40 tall, sized to span the
// FULL height of the dim/sleep button rows (y=68..104 and y=140..176, each
// 36px tall) plus a tiny margin. Blitted in the gap between the ± buttons
// (x=60..180) so no part of the bg between the buttons is left at a stale
// shade when tint changes. Earlier 26-tall version left 5px slivers above
// and below the value text that stayed at the previous tint, visible as
// horizontal grey/black bands. Memory: 120*40*2 = 9600 bytes.
PSRAMCanvas16 valSprite(120, 40);
// Objective-button strip: covers TFT rows Y_POS-1..Y_POS+BOX_SIZE (199..265)
// so the ±1 selection ring fits entirely inside the blit region.
// 240 wide (full screen) × 67 tall = 32 160 bytes.  Eliminates the visible
// fill→text→ring flicker that occurred on every 30 ms main-loop redraw.
PSRAMCanvas16 objSprite(240, 67);

// ─── V12: vibration-monitor DSP + UI state ──────────────────────────────────
// The IMU FIFO feeds a raw-sample ring; every VIB_HOP samples vibTask runs a
// Hann-windowed 512-pt FFT and publishes the spectrum through a lock-free
// ping-pong. See vibTask / vibBegin further down.
#define VIB_FFT_SIZE     512                          // FFT length
#define VIB_FFT_BINS     (VIB_FFT_SIZE / 2)           // 256 usable bins (0..255)
#define VIB_SAMPLE_RATE  416.0f                       // accel ODR, Hz (see imuSetup)
#define VIB_BIN_HZ       (VIB_SAMPLE_RATE / VIB_FFT_SIZE)  // 0.8125 Hz per bin
// Accelerometer sensitivity — must match the setAccelRange() call in imuSetup.
// ±2 g → 0.061 mg/LSB (±4 g would be 0.122). The ±2 g range halves the
// quantization step for finer spectral resolution; bench vibration is
// single-digit mg so the reduced full-scale headroom is never reached.
#define VIB_MG_PER_LSB   0.061f
#define VIB_HOP          128                          // FFT every 128 new samples (75% overlap)
#define VIB_RAW_LEN      8192                         // raw ring — ~19.7 s look-back (Phase 2)
// Band-RMS warning thresholds (mg). MODERATE was 2.0, tried 1.0 (too twitchy)
// and 1.4 (still flagging quiet benches); 1.7 sits between the old 2.0 and
// typical ambient, catching real disturbances without flipping on hum.
// STRONG kept at 8 mg — the "stop, this run is wasted" line.
#define VIB_RMS_MODERATE 1.7f
#define VIB_RMS_STRONG   8.0f
// Narrowband-tonal promotion. A persistent tonal — a dryer at 26 Hz, a stepper
// holding torque, a desk fan — can push the rig 1-3 µm without ever crossing
// the wide-band RMS thresholds (1 µm @ 26 Hz ≈ 2.7 mg peak in a single bin,
// roughly 1.9 mg time-domain RMS once spread over the FFT window).  The
// wide-band test still rules MODERATE/STRONG, but if the dominant bin holds
// >= VIB_TONAL_PEAK_MG of *peak* amplitude AND the dominant frequency stays
// within +-VIB_TONAL_BIN_TOL bins for VIB_TONAL_FRAMES consecutive frames,
// promote CALM -> MODERATE.  Hann coherent gain + the 4/N FFT scaling already
// applied below mean dst[domBin] in deci-mg IS the peak sinusoid amplitude.
#define VIB_TONAL_PEAK_MG  0.5f    // dominant-bin peak amplitude, mg
#define VIB_TONAL_FRAMES   3       // consecutive FFT frames of stability needed
#define VIB_TONAL_BIN_TOL  1       // allow +-1 bin (~+-0.8 Hz) drift per frame
// VIB_SPECTRUM screen geometry.
#define VIB_PLOT_X       4
#define VIB_PLOT_Y       82
#define VIB_WF_X         6
#define VIB_WF_Y         188                          // pulled up from 192; sits 1 px below the plot border
#define VIB_WF_W         228                          // waterfall width  (frequency)
#define VIB_WF_H         131                          // grown from 124 → fills to screen bottom (y=319, border at 320 clipped)

// FFT working buffers + Hann window — .bss (fast internal RAM), ~6 KB.
static float vibFftReal[VIB_FFT_SIZE];
static float vibFftImag[VIB_FFT_SIZE];
static float vibHann[VIB_FFT_SIZE];
ArduinoFFT<float> vibFFT(vibFftReal, vibFftImag, VIB_FFT_SIZE, VIB_SAMPLE_RATE);

// V12: horizontal-plane (e1/e2) projections for the combined-horizontal FFT —
// a VIB_FFT_SIZE-deep .bss ring, written and read only by vibTask.
static int16_t  vibHx[VIB_FFT_SIZE];
static int16_t  vibHy[VIB_FFT_SIZE];
static uint32_t vibHxHead = 0;

// Spectrum ping-pong (lock-free SPSC). vibTask writes vibSpec[seq & 1] then
// publishes vibSpecSeq; readers copy vibSpec[seq & 1] and re-check seq.
// Magnitudes stored in deci-mg (0.1 mg units).
uint16_t vibSpec[2][VIB_FFT_BINS];
// V12: combined horizontal-plane spectrum — second ping-pong, filled in the
// same FFT pass as vibSpec so one vibSpecSeq covers both.
uint16_t vibSpecH[2][VIB_FFT_BINS];

// Per-bin noise-floor EMAs (deci-mg) — track stationary IMU/ambient hum so the
// blur calculation only counts excursions above the floor. Mirrors the web
// analyzer's vibBaseV/H (τ ≈ 8 s). Updated by vibTask each FFT hop only when
// !inStack, so the floor freezes for the duration of a stack and the
// subtraction can't drift into the stack's own motion. .bss arrays — 2 KB.
static float vibBaseV[VIB_FFT_BINS] = { 0 };
static float vibBaseH[VIB_FFT_BINS] = { 0 };
static bool  vibBasePrimed = false;

// PSRAM-backed buffers, allocated by vibBegin(). vibRaw: AC accel ring (LSB,
// gravity removed). vibWF: waterfall RGB565 image, owned by the UI thread.
int16_t  *vibRaw = nullptr;
uint16_t *vibWF  = nullptr;
volatile uint32_t vibRawHead = 0;   // next write index into vibRaw (vibTask only)
bool vibReady = false;              // true once vibBegin() allocations succeeded

// Off-screen canvas for the spectrum plot — drawn by the UI thread then blitted
// in one shot (mirrors the six sprites above) to keep the plot flicker-free.
PSRAMCanvas16 vibPlotSprite(232, 104);

// ─── V12 Phase 2: settle-time analysis ──────────────────────────────────────
// The MJKZZ cycle is: move -> WAIT -> SNAP(shutter) -> HOLD -> move. The A4
// optocoupler senses SNAP; the motor step lands WAIT ms (~1.5 s by default)
// before it. So on each shutter pulse, vibTask Goertzel-filters the window of
// ring buffer *preceding* the pulse — it already holds the move and its full
// ring-down through the WAIT settle period — analysed immediately, no waiting.
// It builds a short-block RMS envelope, fits an exponential decay -> tau + a
// settle time; per-stack settle times pool into a 95th-percentile "suggested
// wait". Passive only — the A4 watcher's behaviour is unchanged.
//
// VIB_SETTLE_BLOCKS sizes the window. Keep it a bit larger than the controller
// WAIT (+ the move's own duration), but short enough not to reach back into the
// previous step's SNAP (~WAIT+HOLD before this one). 2.5 s suits WAIT=1500 ms.
#define VIB_BLOCK          32       // Goertzel block: 32 samples ≈ 77 ms at 416 Hz
#define VIB_SETTLE_BLOCKS  32       // analysis window ≈ 2.5 s (WAIT 1.5 s + margin)
#define VIB_SETTLE_SAMPLES (VIB_BLOCK * VIB_SETTLE_BLOCKS)
#define VIB_RES_MIN_HZ     8.0f
#define VIB_RES_MAX_HZ     140.0f
#define VIB_RES_DEFAULT    55.0f
#define VIB_SETTLE_LOG_MAX 400      // per-stack settle records (percentile pool)
// Per-stack blur stats only integrate FFT hops within this window after a
// shutter pulse (pulse → exposure → ring-down). Without the gate the ~16 idle
// hops of the 5 s stack-done silence tail owned the min and diluted the avg —
// the "per-stack" stats described the quiet bench, not the captures.
#define VIB_BLUR_GATE_MS   1500

float    vibResonanceHz   = VIB_RES_DEFAULT;  // Goertzel lock frequency (NVS-persisted)
int      vibCurrentWaitMs = 1500;             // controller's WAIT setting, user-entered (NVS)
// Shutter denominator (T = 1/N s) used to weight blur bins by |sin(πfT)|.
// NVS-persisted; matches the web analyzer's vibShutterDenom so the firmware-
// computed V/H blur stats line up with what the analyzer panel shows live.
uint16_t vibShutterDenom  = 60;
// Lowest bin included in the broadband blur sum. Matches the web's
// VIB_DISP_MIN_BIN (= 4) — bin 4 ≈ 3.25 Hz. Skipping bins 1-3 stops the 1/ω²
// weighting from blowing up the sub-3-Hz IMU noise.
#define VIB_DISP_MIN_BIN  4
float    vibEnvPlot[VIB_SETTLE_BLOCKS];       // last decay envelope (vibTask writes, UI reads)
float    vibEnvPeak       = 0.0f;             // peak envelope value of the last analysis
uint16_t vibSettleLog[VIB_SETTLE_LOG_MAX];    // per-stack settle times, ms (vibTask only)
int      vibSettleLogCount = 0;
bool          vibPrefsDirty = false;          // deferred "vib" NVS save flag
unsigned long lastVibEditMs = 0;
volatile uint32_t vibPulseRingPos = 0;        // vibRaw write index at the last A4 pulse

// ─── V12 Phase 3: signature library ─────────────────────────────────────────
#define VIB_SIG_MAX      12         // max reference signatures listed
#define VIB_SIG_CACHE    1          // signatures held decoded in RAM (the selected one)
struct VibSignature {
  char     name[16];
  uint16_t mag[VIB_FFT_BINS];       // averaged spectrum, deci-mg
  uint16_t odrHz;
  uint16_t frameCount;
  uint8_t  isTransient;
  uint8_t  pad;
  uint32_t epoch;
};
// Fixed on-device name palette (custom renaming is deferred to the web panel).
const char* const VIB_SIG_NAMES[8] = {
  "HVAC", "Fan", "Washer", "Dryer", "Dishwasher", "Footsteps", "CatJump", "Car"
};
#define VIB_SIG_LIST_TOP  42
#define VIB_SIG_ROW_H     26
#define VIB_SIG_VIS_ROWS  4
// FFT frames peak-held per CAPTURE. At 128-sample hop / 416 Hz ODR each frame
// is 308 ms of new data on a 512-sample (1.23 s) Hann window — 75 % overlap.
// 32 frames covers ~10 s. Peak-hold (vs mean) preserves narrow tonal peaks
// even when they drift across bins during the capture, so the saved signature
// resembles the bright vertical bands in the waterfall instead of a smoothed
// time-average. More frames = more chances for each drifting tone to hit
// every bin in its range at full amplitude — bump for slow-drifting sources;
// transients now bake their peak in, so a single bump during capture will
// contaminate the signature and warrants a re-capture.
#define VIB_SIG_FRAMES    32
// Max basename length for /vibsig/*.bin (excluding the ".bin" extension).
// Capped at 15 to match VibSignature.name[16] and vibSigCaptureName[16] —
// every buffer in the pipeline (capture staging, on-disk struct, list cache,
// cleanup pass, and the JS input) honours this single number. Earlier
// firmware accepted names up to 30 chars from the network, but truncated
// them silently into the 15-char buffers on save, and a separate boot-time
// cleanup pass deleted any file whose name didn't fit, so renames to long
// names were lost on the next reboot.
#define VIB_SIG_NAME_MAX  15

char     vibSigList[VIB_SIG_MAX][24];  // base filenames of saved signatures
int      vibSigCount   = 0;
int      vibSigSel     = -1;           // selected list index, -1 = none
int      vibSigScroll  = 0;
bool     vibSigPicker  = false;        // name-picker overlay open
bool     vibSigCapturing = false;      // peak-holding frames for a CAPTURE
int      vibSigFrames  = 0;
uint32_t vibSigAccum[VIB_FFT_BINS];    // per-bin peak hold (deci-mg) for capture
// Per-bin noise-floor snapshot taken at capture arm. Subtracted from each
// frame before the peak-hold so the stored signature is "max excursion above
// ambient" rather than "max excursion" — a long-running fan baked into the
// living-room hum no longer paints a smooth dome across the signature plot,
// only the source's actual narrowband peaks survive.
float    vibSigBaseSnap[VIB_FFT_BINS];
char     vibSigCaptureName[16];
VibSignature vibSigLoaded;             // currently-selected decoded signature
bool     vibSigLoadedValid = false;
uint32_t vibSigSpecSeqSeen = 0;        // last spectrum seq consumed by the plot
uint32_t vibCapSeqSeen = 0;            // last spectrum seq consumed by a capture
bool     vibSigUiDirty = false;        // list/selection changed → redraw list

// Total PSRAM consumed by the off-screen sprites plus the V12 vibration DSP
// buffers (2 bytes per pixel / per sample). Covers the six UI sprites above,
// vibPlotSprite, the vibRaw raw-sample ring, and the vibWF waterfall image.
// Referenced by both drawMemInfoUI() and the wifi slow-telemetry path.
const uint32_t SPRITE_BYTES = (240*48 + 240*45 + 80*40 + 44*12 + 120*40 + 240*67
                               + 232*104 + VIB_RAW_LEN + VIB_WF_W*VIB_WF_H) * 2;

void setSmoothFont(uint8_t size);

// patched3: forward declarations for WiFi tab accessors (defined in patched3_wifi.ino).
// They are used by drawWifiIndicator() / drawWifiInfoUI() in this file.
bool        wifiIsConnected();
bool        wifiIsPortal();
int         wifiGetRSSI();
String      wifiGetIP();
String      wifiGetSSID();
const char* wifiGetOtaPassword();
bool        wifiHasCustomPassword();
void   drawWifiIndicator();   // defined below — forward-decl needed by wifi tab calls

// V11 polish: corner radius for buttons and slider rails.
// Change this one value to adjust roundness globally.
#define UI_RADIUS 3

// patched3 UI polish: scale an RGB565 colour by num/den per channel (clamped).
// num>den brightens, num<den darkens — used for the button/title bevel where a
// brighter top edge and darker bottom edge fake a raised, shaded surface.
static inline uint16_t scaleColor565(uint16_t c, uint16_t num, uint16_t den) {
  uint32_t r = (uint32_t)((c >> 11) & 0x1F) * num / den;
  uint32_t g = (uint32_t)((c >> 5)  & 0x3F) * num / den;
  uint32_t b = (uint32_t)( c        & 0x1F) * num / den;
  if (r > 31) r = 31;
  if (g > 63) g = 63;
  if (b > 31) b = 31;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

class Button {
public:
  int x, y, w, h;
  const char* label;
  uint16_t boxCol, textCol;
  uint8_t textSize;
  bool isBold;

  Button(int x, int y, int w, int h, const char* l = "", uint16_t bc = TFT_BLACK, uint16_t tc = TFT_WHITE, uint8_t ts = 1, bool bold = false)
    : x(x), y(y), w(w), h(h), label(l), boxCol(bc), textCol(tc), textSize(ts), isBold(bold) {}

  bool contains(int px, int py) const {
    int pad = 5; 
    return (px >= (x - pad) && px <= (x + w + pad) && py >= (y - pad) && py <= (y + h + pad));
  }

  void draw(Adafruit_ILI9341& display, const char* overrideLabel = nullptr, uint16_t overrideBoxCol = 0x0001, uint16_t overrideTextCol = 0x0001) {
    const char* l = overrideLabel ? overrideLabel : label;
    uint16_t bc = (overrideBoxCol != 0x0001) ? overrideBoxCol : boxCol;
    uint16_t tc = (overrideTextCol != 0x0001) ? overrideTextCol : textCol;

    display.fillRoundRect(x, y, w, h, UI_RADIUS, bc);

    // patched3 UI polish: 1px bevel — a brighter line inside the top edge and a
    // darker line inside the bottom edge give the flat fill a raised, shaded
    // look.  Lines start UI_RADIUS in from each side to clear the round corners.
    if (w > 2 * UI_RADIUS + 2 && h > 6) {
      display.drawFastHLine(x + UI_RADIUS, y + 1,     w - 2 * UI_RADIUS, scaleColor565(bc, 9, 5));
      display.drawFastHLine(x + UI_RADIUS, y + h - 2, w - 2 * UI_RADIUS, scaleColor565(bc, 1, 2));
    }

    if (l && strlen(l) > 0) {
      setSmoothFont(textSize);
      display.setTextColor(tc);
      int16_t x1, y1; uint16_t bw, bh;
      display.getTextBounds(l, 0, 0, &x1, &y1, &bw, &bh);
      int cx = x + (w - bw) / 2 - x1; 
      int cy = y + (h - bh) / 2 - y1;
      display.setCursor(cx, cy); display.print(l);
      if (isBold) {
        display.setCursor(cx + 1, cy); display.print(l);
      }
    }
  }
};

#define BOX_SIZE 64
#define BUTTON_PADDING 20
#define Y_POS 200 
#define X_POS_5X 2
#define X_POS_10X ((X_POS_5X - 2) + BOX_SIZE + BUTTON_PADDING)
#define X_POS_20X ((X_POS_10X - 2) + BOX_SIZE + BUTTON_PADDING)

Button btnMainCalibrate(139, 280, 92, 35, "CALIBRATE", COLOR_BLUEGREEN, TFT_WHITE, 1, false);
Button btnSettingsGear(190, 0, 50, 50);

// Settings screen layout: brightness slider sits at y≈82..106; spacing below
// pushes buttons down so users can drag the slider without grazing STACK CALC.
Button btnStackCalc(20, 127, 200, 35, "STACK CALC",    COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnBrightness(20, 172, 200, 35, "BRIGHTNESS",   COLOR_DARKBLUE,  TFT_WHITE, 1, true);
Button btnSettingsClose(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true); // X close for settings (steps back to MAIN)
Button btnSettingsMem(20, 217, 200, 38, "MEMORY", COLOR_BLUEGREEN, TFT_BLACK, 1, true);
Button btnMemClose(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);      // X close for mem info

Button btnStartCal(10, 215, 105, 55, "START CAL", 0x001F, TFT_WHITE, 1, true);
Button btnResetAll(125, 215, 105, 55, "RESET ALL", 0x7800, TFT_WHITE, 1, true);
Button btnCancelCal(60, 280, 120, 35, "CANCEL", 0xF800, TFT_WHITE, 1, true);
// Top-right "i" on CAL_SETTINGS — opens the scrollable calibration how-to
// overlay (mirrors the WiFi/Recovery info buttons).
Button btnCalInfo(200, 4, 32, 30, "i", 0x0A28, COLOR_TEAL, 2, true);

// --- CAL_RUN buttons reorganized to make room for REVIEW ---
// Top row unchanged: CAPTURE&SAVE + GO BACK at y=190 h=65
Button btnCapSave(20, 190, 95, 65);
Button btnGoBackPt(125, 190, 95, 65, "GO BACK", TFT_BLACK, TFT_WHITE, 1, true);
// Bottom row: REVIEW + FINISH + CANCEL, three across at y=265 h=40. FINISH
// (btnFinalize, below) computes the fit early from however many points are
// captured so far (>= MIN_CALIB_POINTS), so the bellows range and step count
// are entirely up to the user — no need to pre-pick a point count.
Button btnReviewPts(5, 265, 70, 40, "REVIEW", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnCancelPt(165, 265, 70, 40, "CANCEL", COLOR_MAROON, TFT_WHITE, 1, true);
// Shared by the CAL_RUN and CAL_REVIEW bottom rows (identical geometry on both).
Button btnFinalize(85, 265, 70, 40, "FINISH", COLOR_DARKGREEN, TFT_WHITE, 1, true);

Button btnYesDel(20, 160, 200, 50, "YES, DELETE", COLOR_MAROON, TFT_WHITE, 1, false);
Button btnNoKeep(20, 230, 200, 50, "NO, KEEP GOING", COLOR_DARKGREEN, TFT_WHITE, 1, false);

Button btnFinishCal(70, 230, 100, 40, "FINISH", 0x001F, TFT_WHITE, 1, false);

Button btnTimeCalc(40, 258, 160, 50, "TIME CALC", 0x001F, TFT_WHITE, 1, true);
// patched3: X close (was the "GO BACK" button) — steps back to APP_SETTINGS.
Button btnCalcBack(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);

// patched3: X close (was the "GO BACK" button) — steps back to STACK_CALC.
Button btnTimeBack(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);

Button btnObj5x(X_POS_5X, Y_POS, BOX_SIZE, BOX_SIZE, "5x", MUTED_5X, TFT_WHITE, 2, false);
Button btnObj10x(X_POS_10X, Y_POS, BOX_SIZE, BOX_SIZE, "10x", MUTED_10X, TFT_WHITE, 2, false);
Button btnObj20x(X_POS_20X, Y_POS, BOX_SIZE, BOX_SIZE, "20x", MUTED_20X, TFT_WHITE, 2, false);

// --- FOV INFO screen buttons ---
// btnFovInfo sits in the top strip to the left of the gear icon (gear cx=217).
// Drawn manually in drawMainScreen: dark-grey box, bright purple two-line text,
// matching the tiny built-in 6x8 font used by "TOF int".
// x=155, y=2, w=44, h=40 leaves a 7px gap before the gear's left cog (~x=206).
Button btnFovInfo(152, 2, 44, 40);   // label drawn manually
// CALIB INFO bottom row: GRAPH (left) + POINTS (right). POINTS opens the
// read-only calibration-point list (CAL_REVIEW with calReviewReadOnly).
Button btnInfoBack(15, 270, 100, 40, "GRAPH", TFT_BLACK, 0xF81F, 1, true);
Button btnInfoPoints(125, 270, 100, 40, "POINTS", TFT_BLACK, COLOR_BLUEGREEN, 1, true);
// Centered BACK for the read-only point list (drawn only when calReviewReadOnly).
Button btnPointsBack(60, 265, 120, 40, "BACK", TFT_BLACK, TFT_WHITE, 1, true);
Button btnInfoClose(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true); // X close calib info
Button btnInfoGraph(170, 4, 62, 26, "GRAPH", TFT_BLACK, 0xF81F, 1, true); // top-right of calib info
Button btnGraphBack(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);  // X close, top-right

// --- BRIGHTNESS_SETTINGS screen ---
// V11: layout compressed in the lower half to make room for a new
// SCREEN TIMEOUTS button just above GO BACK. The LED toggle moved from
// y=162 to y=148 (-14) and the LED slider blit from y=210 to y=192 (-18)
// so the "TRIGGER LED" label and ON/OFF toggle remain visually aligned and
// centered with the slider beneath them.
// patched3: lower half restacked again — an IR CONTROL button now sits above
// SCREEN TIMEOUTS, so both menu buttons shrank to h=31 and GO BACK moved down.
// patched3: X close (was the "GO BACK" button) — steps back to APP_SETTINGS.
Button btnBrightBack(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);
Button btnLedToggle(155, 136, 75, 26, "OFF", COLOR_MAROON, TFT_WHITE, 1, true);
Button btnIrControl(20, 210, 200, 31, "IR CONTROL", COLOR_PURPLE, TFT_WHITE, 1, true);
Button btnScreenTimeouts(20, 248, 200, 31, "SCREEN TIMEOUTS", COLOR_TEAL, TFT_WHITE, 1, true);

// --- SCREEN_TIMEOUT screen (V11) — adjustable dim/sleep + theme picker ---
// V11 polish: dim row pulled up to y=68 and sleep row to y=140 so labels
// have clear breathing room above each row of -/+ buttons.
Button btnDimMinus  ( 20, 68, 40, 36, "-",   COLOR_MAROON,    TFT_WHITE, 2, true);
Button btnDimPlus   (180, 68, 40, 36, "+",   COLOR_DARKGREEN, TFT_WHITE, 2, true);
Button btnSleepMinus( 20, 140, 40, 36, "-",  COLOR_MAROON,    TFT_WHITE, 2, true);
Button btnSleepPlus (180, 140, 40, 36, "+",  COLOR_DARKGREEN, TFT_WHITE, 2, true);
// patched3: X close (was the "GO BACK" button) — steps back to BRIGHTNESS_SETTINGS.
Button btnThemeBack (205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);
// Four theme swatches sized to fit one row across the 240px screen.
Button btnTheme0( 12, 235, 50, 35, "");
Button btnTheme1( 68, 235, 50, 35, "");
Button btnTheme2(124, 235, 50, 35, "");
Button btnTheme3(180, 235, 50, 35, "");

// --- SENSOR_INFO screen ---
Button btnSensorToggle(10, 162, 220, 40, "SLEEP SENSOR",          COLOR_MAROON,     TFT_WHITE, 1, true);
Button btnSensorHiRef (10, 210, 220, 38, "HIGH REFLECTIVITY: OFF", COLOR_DARKGREY,   TFT_WHITE, 1, false);
// patched3: X close (was the "GO BACK" button) — steps back to MAIN.
Button btnSensorBack  (205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);


// --- WIFI_INFO screen (patched3) — opened from WiFi zone in header (x=93..151, y=0..42) ---
Button btnWifiInfoForget(20, 275, 200, 36, "FORGET WiFi",  COLOR_MAROON, TFT_WHITE, 1, true);
Button btnWifiInfoBack  (205,  2,  33, 33, "X",            0x4208,       COLOR_RED, 2, true);
// V12.3: (i) on the WiFi title bar → scrollable SECURITY TIPS (mirrors the web).
Button btnWifiInfoTips  (123,  4,  32, 30, "i",            0x0A28,       COLOR_TEAL, 2, true);

// --- ABOUT screen (V11) — opened from the CALIB / version label on main screen ---
Button btnAboutClose(205, 2, 33, 33,    "X",             0x4208,       COLOR_RED, 2, true);
// V12.3: (i) on the ABOUT title bar → scrollable RECOVERY FLASH help (mirrors the web).
Button btnAboutInfo (123, 5, 32, 28,    "i",             0x0A28,       COLOR_TEAL, 2, true);

// V12.3: shared drag-scrolled info overlay (SECURITY TIPS / RECOVERY FLASH).
// Just an X close in the title bar — the body is dragged with a finger.
Button btnInfoTextClose(205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);

// --- CAL_REVIEW screen buttons ---
Button btnReviewUp(195, 45, 35, 40, "UP", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnReviewDown(195, 195, 35, 40, "DN", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnRetake(5, 265, 70, 40, "RETAKE", 0x001F, TFT_WHITE, 1, true);
Button btnReviewBack(165, 265, 70, 40, "BACK", TFT_BLACK, TFT_WHITE, 1, true);

// --- IR_CONTROL screen (patched3) — MJKZZ NEC remote buttons ---
Button btnIrStart  ( 20,  55, 200, 40, "START",         COLOR_DARKGREEN, TFT_WHITE, 1, true);
Button btnIrStop   ( 20, 105, 200, 40, "STOP",          COLOR_MAROON,    TFT_WHITE, 1, true);
Button btnIrUp     ( 20, 155,  95, 40, "UP",            COLOR_DARKBLUE,  TFT_WHITE, 1, true);
Button btnIrDown   (125, 155,  95, 40, "DOWN",          COLOR_DARKBLUE,  TFT_WHITE, 1, true);
Button btnIrCapture( 20, 205, 200, 40, "CAPTURE PHOTO", COLOR_PURPLE,    TFT_WHITE, 1, true);
// patched3: X close (was the "GO BACK" button) — steps back to BRIGHTNESS_SETTINGS.
Button btnIrBack   (205, 2, 33, 33, "X", 0x4208, COLOR_RED, 2, true);

// --- V12: VIBRATION monitor — hub (VIB_HOME) + live spectrum (VIB_SPECTRUM) ---
Button btnVibSpectrum  ( 20,  70, 200, 50, "SPECTRUM",    COLOR_DARKBLUE,  TFT_WHITE, 1, true);
Button btnVibSettle    ( 20, 138, 200, 50, "SETTLE TIME", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnVibSignatures( 20, 206, 200, 50, "SIGNATURES",  COLOR_PURPLE,    TFT_WHITE, 1, true);
Button btnVibHomeClose (205,   2,  33, 33, "X", 0x4208, COLOR_RED, 2, true);
Button btnVibSpecClose (205,   2,  33, 33, "X", 0x4208, COLOR_RED, 2, true);
// Tile on the SETTINGS screen that opens the vibration hub.
Button btnVibMonitor   ( 20, 265, 200, 34, "VIBRATION",   COLOR_TEAL,     TFT_WHITE,       1, true);
// VIB_SETTLE — motor-step settle-time analysis.
Button btnVibRunTest   ( 16, 252, 100, 30, "RUN TEST",  COLOR_DARKGREEN, TFT_WHITE, 1, true);
Button btnVibReLock    (124, 252, 100, 30, "RE-LOCK",   COLOR_PURPLE,    TFT_WHITE, 1, true);
Button btnVibWaitMinus ( 16, 286, 100, 28, "WAIT -100", COLOR_MAROON,    TFT_WHITE, 1, true);
Button btnVibWaitPlus  (124, 286, 100, 28, "WAIT +100", COLOR_DARKGREEN, TFT_WHITE, 1, true);
Button btnVibSettleClose(205,  2,  33, 33, "X", 0x4208, COLOR_RED, 2, true);
// VIB_SIGNATURES — reference-spectrum library.
Button btnVibSigCapture( 10, 262, 100, 40, "CAPTURE", COLOR_DARKGREEN, TFT_WHITE, 1, true);
Button btnVibSigDelete (130, 262, 100, 40, "DELETE",  COLOR_MAROON,    TFT_WHITE, 1, true);
Button btnVibSigClose  (205,   2,  33, 33, "X", 0x4208, COLOR_RED, 2, true);
Button btnVibSigUp     (198,  46,  37, 34, "UP", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
Button btnVibSigDown   (198, 124,  37, 34, "DN", COLOR_BLUEGREEN, TFT_WHITE, 1, true);
// IR screen buttons indexed for the draw/touch loops. irCodes[] aligns with the
// first five (the NEC-sending buttons); btnIrBack is index 5 (no code).
Button* const  irButtons[6] = { &btnIrStart, &btnIrStop, &btnIrUp,
                                &btnIrDown, &btnIrCapture, &btnIrBack };
const uint64_t irCodes[5]   = { IR_MJKZZ_START, IR_MJKZZ_STOP, IR_MJKZZ_UP,
                                IR_MJKZZ_DOWN, IR_MJKZZ_CAPTURE };

struct __attribute__((packed)) CalibData {
  uint32_t magic; 
  float ctrlX, ctrlY, sensorWidth, demarcation, calibError;
  int brightness;
  float stackStepSize;
  float stackTotalDepth;
  float stackTimePerStep;
  uint8_t isCustom;
  uint8_t ledEnabled;   // V17b: 1 = trigger LED enabled, 0 = disabled
  uint8_t ledDuty;      // V17b: LED PWM duty 0-255 (active-low: higher = dimmer)
  uint8_t _pad;
  float calibR2;        // V17: persisted R²
  // V11 fix: persist the calibration scatter points so the CAL_GRAPH plot
  // (and the web graph) survive a reboot. Without these, a custom-calibrated
  // device reloaded distPoints[]/fovPoints[] as all-zero after a power-cycle.
  int32_t calNPoints;        // nPoints used for the stored fit
  int32_t calPointsCaptured; // valid slot count in the two arrays below
  float   calDistPoints[20];
  float   calFovPoints[20];
};
CalibData settings;

// V17: Magic bumped because CalibData layout grew (calibR2 added).
// Devices upgrading from V16 will see a one-time return to defaults.
#define CALIB_MAGIC 0x544F4C4D  // V12.3: bumped — ctrlX/ctrlY are now pixel-space coefficients (old FOV-coefficient calibs reset to factory)

// V12.5 (F4): calibration backup so a friend's calibration survives a firmware
// update that bumps CALIB_MAGIC. Stored in its OWN NVS namespace "calibbak"
// with its OWN format magic — DECOUPLED from CALIB_MAGIC, so bumping the latter
// never invalidates the backup. Holds only the layout-stable inputs (width,
// demarcation, point pairs); the fit is always re-derived on restore
// (finalizeCalibration), never trusted from stored coefficients. NVS survives
// OTA (writes only the app slot), power cycles, and USB recovery.
#define CALIBBAK_MAGIC 0x43424B31UL   // "CBK1" — bump only if THIS struct changes
struct CalibBackup {
  uint32_t magic;
  float    width;    // sensorWidthPixels
  float    demarc;   // demarcationDist (mm)
  int32_t  n;        // point count (2..20)
  float    dist[20];
  float    fov[20];
};

// ─── V12.5: field-resilience state (boot-loop rollback + reset diagnostics) ───
// RTC_NOINIT RAM survives a warm reset (SW reset, panic, task/interrupt WDT,
// brownout — the RTC domain stays powered) but is GARBAGE after a cold
// power-on. `sentinel` is the discriminator: on a mismatch we zero the struct,
// so a cold boot correctly starts bootCount=0. A deep brownout that also sags
// the RTC domain simply looks like a cold boot (counter reset → never a false
// rollback), which is the safe failure direction.
// Bump the sentinel whenever RtcResilience GROWS. But do NOT blindly memset on a
// bumped sentinel — MIGRATE instead (zero only the appended fields), so the
// F1 anti-ping-pong guard (rollbackAttempted) and the AUTO-ROLLBACK marker
// survive a rollback across the version boundary. Only a truly unknown sentinel
// (cold-boot garbage) gets a full wipe. Keep the prior sentinel value(s) here.
#define RTC_RESIL_SENTINEL    0xAF0FB008UL   // current: +lastMinStack/lastMaxAllocKB
#define RTC_RESIL_SENTINEL_V1 0xAF0FB007UL   // pre-.54 (24-byte layout, no stack/frag tail)
struct RtcResilience {
  uint32_t sentinel;          // == RTC_RESIL_SENTINEL once initialised
  uint32_t bootCount;         // F1: consecutive un-healthy boots (reset after 30 s uptime)
  uint8_t  rollbackAttempted; // F1: partition already flipped this streak (anti-ping-pong)
  uint8_t  _pad[3];
  uint32_t lastUptimeSec;     // F2: snapshot updated ~5 s in loop(), read next boot
  uint32_t lastMinHeap;       // F2: snapshot (KB) updated with lastUptimeSec
  uint32_t pendingReasonCode; // F2: synthetic reason to fold into the ring next boot (0 = none)
  uint32_t lastMinStack;      // smallest free task stack (bytes) seen this session
  uint32_t lastMaxAllocKB;    // smallest largest-contiguous-free block (KB) — fragmentation
};
RTC_NOINIT_ATTR RtcResilience g_rtc;

// Synthetic reason codes (>= 100) that our own code stashes in
// g_rtc.pendingReasonCode before a self-initiated restart, so the post-reboot
// diag ring shows WHY rather than a generic SW reset. Codes < 100 are raw
// esp_reset_reason_t values.
#define DIAG_REASON_ROLLBACK    100   // F1 auto-rollback flipped the boot partition
#define DIAG_REASON_SENSOR_WDT  101   // F3 sensor-heartbeat stall → self-heal reboot

// F2: persisted ring of recent boot/crash records (NVS namespace "diag").
// NOTE: growing this struct changes sizeof — the reader skips size-mismatched
// (older-format) entries, so the ring self-heals over the next few boots.
struct DiagEntry {
  uint8_t  reason;      // esp_reset_reason() code, or a DIAG_REASON_* synthetic
  uint8_t  _pad;
  uint16_t bootSlot;    // running OTA subtype at the time (0 = ota_0, 1 = ota_1)
  uint32_t uptimeSec;   // g_rtc.lastUptimeSec captured just before the reset
  uint32_t minHeap;     // g_rtc.lastMinHeap (KB) captured just before the reset
  char     version[12]; // BUILD_VERSION of the image that was running
  uint16_t minStack;    // smallest free task stack (bytes) before the reset — a
                        //   near-zero value alongside a PANIC implicates a stack overflow
  uint16_t maxAllocKB;  // smallest largest-free block (KB) — fragmentation before the reset
};
#define DIAG_RING_LEN 8

// F2: compact "last reset" line for the device MEM_INFO screen, built once in
// the early-setup block (e.g. "BROWNOUT 8h/40K"). Global so drawMemInfoUI can
// show it without re-reading NVS.
char g_bootReasonLine[28] = {0};

// Map an esp_reset_reason_t (or a DIAG_REASON_* synthetic) to a short label for
// the diag ring / dashboard. Kept compact — these strings ship in the binary.
static const char* diagReasonStr(uint8_t r) {
  switch (r) {
    case DIAG_REASON_ROLLBACK:   return "AUTO-ROLLBACK";
    case DIAG_REASON_SENSOR_WDT: return "SENSOR-STALL WDT";
    case ESP_RST_POWERON:  return "POWERON";
    case ESP_RST_EXT:      return "EXT-RESET";
    case ESP_RST_SW:       return "SW-RESTART";
    case ESP_RST_PANIC:    return "PANIC";
    case ESP_RST_INT_WDT:  return "INT-WDT";
    case ESP_RST_TASK_WDT: return "TASK-WDT";
    case ESP_RST_WDT:      return "OTHER-WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO:     return "SDIO";
    // IDF-5.x additions (this build is IDF 5.1.x). CPU_LOCKUP is a genuine crash;
    // PWR_GLITCH is a power fault (brownout-class). Without these they'd fall to
    // "UNKNOWN" and read as benign on the dashboard.
#ifdef ESP_RST_USB
    case ESP_RST_USB:      return "USB-RESET";
#endif
#ifdef ESP_RST_JTAG
    case ESP_RST_JTAG:     return "JTAG-RESET";
#endif
#ifdef ESP_RST_EFUSE
    case ESP_RST_EFUSE:    return "EFUSE-ERR";
#endif
#ifdef ESP_RST_PWR_GLITCH
    case ESP_RST_PWR_GLITCH: return "PWR-GLITCH";
#endif
#ifdef ESP_RST_CPU_LOCKUP
    case ESP_RST_CPU_LOCKUP: return "CPU-LOCKUP";
#endif
    default:               return "UNKNOWN";
  }
}

// V12.5 (F1): does this reset reason indicate an UNEXPECTED failure (crash /
// watchdog / power fault) — as opposed to a deliberate restart (our own
// ESP.restart(): OTA, forget-wifi, F3 self-heal, F1 rollback → ESP_RST_SW) or a
// cold/button boot (POWERON/EXT)? Only unexpected failures accumulate the
// boot-loop counter, so a burst of intentional reboots (e.g. OTA then
// forget-wifi within 30 s) can never trigger a spurious rollback, and a
// persistent I²C wedge that F3 keeps healing (SW resets) never downgrades the
// firmware. A cold power-on can't accumulate anyway (RTC garbage → sentinel
// reset). UNKNOWN counts, conservatively — an unclassifiable reset might be a
// real crash loop, and the rollback is fully guarded (validated slot, one
// attempt, never bricks).
static bool diagIsCrash(esp_reset_reason_t rr) {
  switch (rr) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_UNKNOWN:
#ifdef ESP_RST_CPU_LOCKUP
    case ESP_RST_CPU_LOCKUP:
#endif
      return true;   // firmware faults — a rollback to older firmware can help
    default:
      // M2: BROWNOUT / PWR_GLITCH are POWER faults, deliberately NOT counted —
      // rolling back firmware can't fix a failing PSU, and counting them would
      // let a brownouting supply trigger a spurious downgrade (and, across the
      // sentinel boundary, a slot ping-pong). Still logged in the ring as
      // BROWNOUT via diagReasonStr, so remote diagnosis is unaffected.
      // Also POWERON / EXT / SW (deliberate) / DEEPSLEEP / SDIO / USB / JTAG / EFUSE.
      return false;
  }
}

// NOTE: appending to the ring is inlined in setup()'s early block rather than a
// helper — a free function taking `const DiagEntry&` trips the Arduino
// auto-prototype generator (the generated prototype lands before DiagEntry is
// defined). Reading the ring happens in the wifi tab's GET /diag handler, which
// is compiled after this struct so a local DiagEntry there is fine.

bool isCustomCalib = false;
int currentBrightness = Config::DEFAULT_BRIGHTNESS;
int currentLedDuty = TRIGGER_LED_DUTY;  // V17b: persisted LED brightness (0=off,255=full on-duty)
bool ledEnabled = false;                // V17b: whether trigger LED fires at all
// Atomic: written by loop()/wifiLoop() (Core 1), read by sensorTask (Core 0) —
// the task must stop polling while asleep, or a data-ready latched just before
// StopMeasurement() makes its next poll call ClearInterruptAndStartMeasurement()
// and silently restart ranging (laser back on while the UI says "USER SLEEP").
std::atomic<bool> sensorSleeping{false};   // V17b: TOF sensor user-sleep state
bool highReflMode   = false;            // High-Reflectivity mode: 8ms timing budget (vs 33ms default)
// Sensor auto-power-off (1 h idle). sensorsAutoOff = both sensors currently
// auto-powered-down; tofAutoSlept = WE slept the TOF (vs the user having it
// manually slept already, which we must not auto-wake). See loop().
bool sensorsAutoOff = false;
bool tofAutoSlept   = false;
Preferences preferences;

float distPoints[20]; 
float fovPoints[20];  

int nPoints = 10;      
float sensorWidthPixels = Config::DEFAULT_SENSOR_WIDTH_PX; 
float demarcationDist = Config::DEFAULT_DEMARCATION_MM; 

int currentCalIndex = 0;
int pointsCaptured = 0;        // V17: tracks how many calibration slots hold valid data
bool isRetakeMode = false;     // V17: true when CAL_RUN was entered to retake a single existing point
float CTRLX = Config::DEFAULT_CTRL_X, CTRLY = Config::DEFAULT_CTRL_Y;
float CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; 
float CALIB_R2 = 0.9991f;      // V17: factory-default R² (0.994 empirically verified)
// V12.3: cached pixel-space fit statistics for the distance-dependent prediction
// interval (recomputed from the cal points on load/finalize — no new persisted
// fields). gCalSpx = pixel residual SD, gCalXbar = mean cal distance,
// gCalSxx = Σ(dist-mean)², gCalN = point count.
float gCalSpx = 0.0f, gCalXbar = 0.0f, gCalSxx = 0.0f; int gCalN = 0;
float mul_5x = 4.0, mul_10x = 2.0, mul_20x = 1.0;
int tempPixels = Config::DEFAULT_TEMP_PIXELS;

float stackStepSize = 0.468f;
int   stackTotalImgs = 128;
float stackTimePerStep = 3.3f;
int stackCalcSelection = 0;

enum DisplayMode { 
  MAIN, CAL_SETTINGS, CAL_RUN, CAL_SAMPLING, CAL_SUCCESS, 
  CAL_CONFIRM, APP_SETTINGS, STACK_CALC, STACK_TIME,
  FOV_INFO,           // V17:  FOV calibration info screen
  CAL_REVIEW,         // V17:  review/edit captured calibration points
  BRIGHTNESS_SETTINGS,// V17b: screen+LED brightness controls
  SENSOR_INFO,        // V17b: TOF sensor details + sleep toggle
  MEM_INFO,           // V17b: memory and system info
  CAL_GRAPH,          // V17b: calibration scatter plot + regression line
  ABOUT,              // V11:  about / version / developer info, opened from main-screen calib label
  SCREEN_TIMEOUT,     // V11:  screen timeouts + color-theme picker
  WIFI_INFO,          // patched3: WiFi status / SSID / forget-wifi screen
  IR_CONTROL,         // patched3: MJKZZ IR remote button panel
  VIB_HOME,           // V12: vibration-monitor hub (3 nav buttons)
  VIB_SPECTRUM,       // V12: live FFT spectrum + waterfall
  VIB_SETTLE,         // V12: motor-step settle-time analysis
  VIB_SIGNATURES,     // V12: reference-spectrum library / compare
  WIFI_TIPS,          // V12.3: scrollable WiFi security-tips overlay (from WIFI_INFO)
  RECOVERY_HELP,      // V12.3: scrollable USB-recovery help overlay (from ABOUT)
  CALIB_HELP          // scrollable calibration how-to overlay (from CAL_SETTINGS)
};
DisplayMode currentMode = MAIN;
DisplayMode preSleepMode = MAIN;
int calibSelection = 0; 

// V17: review screen state
int reviewSelected = -1;
int reviewScrollOffset = 0;
// When true, CAL_REVIEW is a read-only viewer opened from the CALIB INFO screen
// (no active calibration session): RETAKE/FINISH are hidden and BACK returns to
// CALIB INFO instead of resuming capture.
bool calReviewReadOnly = false;
const int REVIEW_VISIBLE_ROWS = 6;
const int REVIEW_ROW_HEIGHT = 26;
const int REVIEW_LIST_TOP = 45;

std::atomic<uint32_t> sensorState{0};
std::atomic<uint32_t> sensorHealth{0};
std::atomic<uint32_t> sensorAmbient{0};   // V17b: ambient Mcps * 1000 (24 bits)
std::atomic<uint32_t> sensorErrInt{1};    // 2σ FOV error bound × 100 (centimillimeters); written by updateDisplay()
std::atomic<uint32_t> sensorAvgDist{0};  // 5-sample rolling-avg distance × 10 (tenths of mm); written by updateDisplay()
// Distance EMA in tenths of a mm (round(emaDistance*10)), written by sensorTask
// alongside sensorState. sensorState carries whole-mm, which quantises the
// calibration capture to 1 mm; the calibration sampler averages THIS instead so
// capture points keep sub-mm resolution. Bit 31 = valid (mirrors sensorState).
std::atomic<uint32_t> sensorDistTenths{0};
std::atomic<bool>     sensorEmaReset{false}; // V17b: request EMA reset after wake
// V12.5 (F3): sensorTask liveness heartbeat — stores millis() at the TOP of
// every poll iteration (the sleep path still loops ~100 ms, so a fresh value
// means alive-or-idle, NOT necessarily ranging). loop() (Core 1) watches this;
// staleness means the shared I²C bus is wedged (any task stuck holding
// i2cMutex), which a reboot heals via recoverI2CBus() at boot.
std::atomic<uint32_t> sensorHeartbeatMs{0};
// V12.5: worst sensorTask heartbeat stall observed this session (ms). Recorded
// but NOT acted on — see the F3 note in loop(). Surfaced in /diag to diagnose
// the intermittent real-hardware I²C stall without rebooting.
std::atomic<uint32_t> sensorMaxStallMs{0};

// V12.5.x: session-level health metrics (all reset each boot; surfaced in /diag).
// #1 per-task free-stack high-water (bytes); a value near 0 = stack-overflow risk.
// 0 = not yet sampled. Written by loop()'s 5 s snapshot (Core 1), read by /diag.
std::atomic<uint32_t> stackFreeSensor{0};
std::atomic<uint32_t> stackFreeVib{0};
std::atomic<uint32_t> stackFreeLoop{0};
// #3 smallest largest-contiguous-free internal-heap block seen (KB) — fragmentation.
std::atomic<uint32_t> minMaxAllocKB{0xFFFFFFFFUL};
// #5 count of ps_malloc/malloc failures: the 7 sprites + any transient info
//    canvas (all via PSRAMCanvas16::begin), the vib DSP buffers, and the OTA
//    PSRAM buffer. Non-zero = memory pressure degraded a feature.
std::atomic<uint32_t> allocFailCount{0};
// #6 count of IMU/Wire I²C transaction errors (imuReadRegs/imuWriteReg failures) —
//    a bus-health proxy that complements sensorMaxStallMs.
std::atomic<uint32_t> i2cErrCount{0};

// V12: vibration-monitor cross-core scalars. vibTask (Core 0) writes them;
// loop() (Core 1) and the wifi tab read them. Plain scalars — no struct, no
// lock — each is a single 32-bit atomic. The full FFT spectrum array (added
// in Phase 1) uses a separate lock-free ping-pong, not these.
std::atomic<uint32_t> vibDominant{0};      // dominant frequency, Hz × 10
std::atomic<uint32_t> vibBandRms{0};       // in-band VERTICAL RMS accel, mg × 100
std::atomic<uint32_t> vibHorizRms{0};      // V12: horizontal-plane RMS accel, mg × 100
std::atomic<uint32_t> vibState{0};         // 0 = calm, 1 = moderate, 2 = strong
std::atomic<uint32_t> vibSuggestedWait{0}; // advisory settle wait, ms
std::atomic<uint32_t> vibSpecSeq{0};       // bumped each time a new FFT spectrum is published
std::atomic<uint32_t> vibPulseSeq{0};      // bumped by loop() / RUN TEST on each shutter pulse
std::atomic<uint32_t> vibStackStartSeq{0}; // bumped by loop() when a focus stack begins
std::atomic<uint32_t> vibStackDoneSeq{0};  // bumped by loop() at stack-complete
// Bumped when a pulse sequence ends WITHOUT reaching MIN_ACTIVE_DURATION — a
// single shot or an aborted run. vibTask clears inStack without computing the
// suggested-wait aggregate. Without this signal, one lone shutter pulse left
// inStack stuck true, freezing the vibBaseV/H noise-floor EMAs (stale ambient
// subtraction for signature captures and blur stats) until the next real stack.
std::atomic<uint32_t> vibStackAbortSeq{0};
// Per-stack blur stats — shutter-weighted broadband displacement, with the
// per-bin noise floor subtracted. Mirrors what vibDispStats() computes on the
// web side, so the PNG card's V/H pixel-blur numbers match the analyzer's
// "broadband" row line-for-line. µm × 100 (centi-µm).
std::atomic<uint32_t> vibStackBlurVAvg{0};
std::atomic<uint32_t> vibStackBlurVMin{0};
std::atomic<uint32_t> vibStackBlurVMax{0};
std::atomic<uint32_t> vibStackBlurHAvg{0};
std::atomic<uint32_t> vibStackBlurHMin{0};
std::atomic<uint32_t> vibStackBlurHMax{0};
std::atomic<uint32_t> vibSettleSeq{0};     // bumped when a decay analysis is published
std::atomic<uint32_t> vibSettleMs{0};      // last single-pulse settle time, ms
std::atomic<uint32_t> vibSettleTauMs{0};   // last fitted decay constant τ, ms
// Auto-power-off request for the accelerometer. loop() (Core 1) sets it after
// 1 h idle; vibTask (Core 0) owns the actual LSM6DSOX power-down/up so all IMU
// register access stays on one core (see vibTask).
std::atomic<bool> vibAutoOff{false};
bool imuPresent = false;                   // set true once the LSM6DSOX answers
uint8_t imuAddr = 0x6A;                    // resolved to 0x6A or 0x6B at init

TaskHandle_t sensorTaskHandle;
TaskHandle_t vibTaskHandle = nullptr;      // V12: vibration-monitor DSP task
SemaphoreHandle_t i2cMutex;

unsigned long samplingStartTime = 0;
long samplingSum = 0;
int samplingCount = 0;

const unsigned long MIN_ACTIVE_DURATION = 20000;
const unsigned long SILENCE_DURATION = 5000;
bool isSequenceActive = false;
// patched3: file-scope so the wifi tab can push it to fast-telemetry JSON.
// Was previously a function-local `static bool lastCircleState` inside the
// trigger-watcher; semantics identical — true while the optocoupler reads
// active and the on-screen indicator / LED are lit.
bool shutterActive = false;
unsigned long firstPulseTime = 0;
unsigned long lastPulseTime = 0;

// V12.3: measured stack time. The trigger watcher already times the real
// photo-pulse sequence (firstPulseTime..lastPulseTime); counting the shutter
// edges turns that into a measured sec/step for the just-finished MJKZZ stack.
// When the projected total at the configured image count (measuredPerStep x
// stackTotalImgs) drifts >= MEASURED_DRIFT_SEC from the manual stackTimePerStep,
// the measured value auto-replaces it so Sec/Step rarely needs hand-tuning.
// Advisory feedback only — this never gates or delays capture.
uint32_t stackPulseCount = 0;            // shutter edges in the active sequence
float    measuredPerStep = 0.0f;         // last measured sec/step (0 = none yet)
uint32_t measuredTotalSec = 0;           // last measured total stack duration, s
uint32_t measuredPulses = 0;             // shots counted in the last measured stack
const float MEASURED_DRIFT_SEC = 30.0f;  // projected-total drift that auto-applies

// V12.3: single source of truth for "is an MJKZZ stack running". Set true on any
// START (web IR cmd, device IR screen, or the bezel STACK toggle) and false on
// STOP or when the camera-trigger watcher sees the pulse sequence end. Broadcast
// in telemetry so every web control (bezel STACK button + IR-screen buttons)
// reflects reality regardless of where start/stop was issued.
bool mjkzzStackActive = false;

// V12.3: name of the calibration shown on the CAL_SUCCESS screen. Set by a
// named import (POST /calib → calApply); cleared right before a device/web
// point-calibration finalizes so those show a generic "CALIB SUCCESS".
char lastCalibName[32] = "";

// patched3: deferred WiFi-forget timer. When the user taps FORGET WiFi we want
// to show a "CLEARING..." flash for ~600 ms before rebooting, but blocking the
// loop with delay() also blocks touch and any other loop-driven refresh.  We
// stamp `wifiForgetAtMs = millis() + delay_ms` instead and loop() polls it.
// 0 = no pending forget.
uint32_t wifiForgetAtMs = 0;

// patched3: WIFI_INFO live-refresh timer.
unsigned long lastWifiInfoUpdate = 0;

// patched3: header WiFi-indicator RSSI poll timer (MAIN screen only).
unsigned long lastWifiHeaderCheckMs = 0;

bool isScreenSleep = false;
bool isScreenDim = false;
unsigned long lastActivityTime = 0;
// Set to true whenever the finger lifts; consumed by the adj-button acceleration
// logic to unconditionally reset holdStartTime on the very next press.
// Prevents rapid successive single taps from accumulating into the hold-acceleration
// threshold (which only resets on a time-gap, not on an actual finger-up event).
bool adjFingerLifted = false;

const uint8_t keyboardReportMap[] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x85, 0x01, 
    0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 
    0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x06, 
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 
    0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0                
};

const int numReadings = 5;
int readings[numReadings] = {0};
int readIndex = 0;

// V12.3: rolling ~5 s window of the displayed AVG FOV, sampled ~20 Hz, for the
// empirical error (2σ of the FOV's actual variation — sensor jitter + slow drift
// — not a model estimate). Timestamped so the window is a true 5 s at any rate.
#define FOV_ERR_WIN 128
float    fovErrWin[FOV_ERR_WIN];
uint32_t fovErrWinT[FOV_ERR_WIN];
int      fovErrHead = 0, fovErrCount = 0;
uint32_t fovErrLastSample = 0;
long totalDist = 0;
float averageDist = 0;
bool bufferFilled = false;

const int avgFovLabelY = 52;     
const int avgFovValueY = 73;     
const int distanceLabelY = 132;  
const int distanceValueY = 147; 

int currentobj = 3; 
unsigned long lastDisplayUpdate = 0, lastTouchTime = 0, lastCalibDistUpdate = 0;
unsigned long lastModeChangeMs = 0;  // menu-transition guard timestamp
// Separate cadences so the three live-refresh sites don't reset each other's
// timer when the user navigates between screens. Names match the screen they
// drive; lastCalibDistUpdate (above) is kept for the top-strip distance update.
unsigned long lastSensorInfoUpdate = 0;
unsigned long lastVibSpecUpdate = 0;
float lastAvgFov = -1;
uint16_t lastDistance = 0xFFFF;
// #1: cache for drawSignalHealthBar's MAIN (non-sprite) path. The 30 Hz refresh
// calls it every frame, but the quantised bar (≤40 px, 5 colours) rarely moves —
// skip the sprite rebuild + SPI blit when filled/colour are unchanged. Reset to
// -1 in drawMainScreen() so the bar always paints once on screen entry.
int      lastBarFilled = -1;
uint16_t lastBarColor  = 0;

// Forward declarations
void drawMainScreen();
void drawCalSettingsUI();
void drawAppSettingsUI();
void drawOldBrightnessBar();          // V17b
void drawPointEntryUI();
void drawConfirmUI();


void drawBrightnessSettingsUI();
void drawSensorInfoUI();
void refreshBrightnessSettingsValues(bool force);
void handleBrightnessSettingsTouch(TS_Point p, int adj);
void handleSensorInfoTouch(TS_Point p);
void refreshSensorInfoValues();
void drawMemInfoUI();
void handleMemInfoTouch(TS_Point p);
void drawAboutUI();                        // V11:  about / developer screen
void handleAboutTouch(TS_Point p);         // V11
void drawScreenTimeoutUI();                // V11:  timeouts + theme picker
void refreshScreenTimeoutValues(bool force); // V11
void handleScreenTimeoutTouch(TS_Point p); // V11
void drawIrControlUI();                    // patched3: MJKZZ IR remote screen
void handleIrControlTouch(TS_Point p);     // patched3
void vibBegin();                           // V12: PSRAM alloc + Hann table
void drawVibHomeUI();                      // V12: vibration-monitor hub
void handleVibHomeTouch(TS_Point p);       // V12
void drawVibSpectrumUI();                  // V12: live spectrum + waterfall
void refreshVibSpectrumValues(bool force); // V12
void handleVibSpectrumTouch(TS_Point p);   // V12
void drawVibSettleUI();                    // V12: settle-time analysis screen
void refreshVibSettleValues(bool force);   // V12
void handleVibSettleTouch(TS_Point p);     // V12
void drawVibSignaturesUI();                // V12: signature library screen
void refreshVibSignaturesValues(bool force); // V12
void handleVibSignaturesTouch(TS_Point p); // V12
void loadVibPrefs();                       // V12: load "vib" NVS namespace
void saveVibPrefs();                       // V12: persist "vib" NVS namespace
void vibCapturePoll();                     // V12: drives an armed signature capture
void wifiPushVibSigList();                 // V12: wifi tab — push signature list to web
void applyTheme(int idx);                  // V11
void loadDisplayPrefs();                   // V11: load timeouts + theme from NVS
void saveDisplayPrefs();                   // V11: persist timeouts + theme to NVS
void fireTriggerLed(bool on);              // V17b: centralised active-low LED drive
void drawWifiInfoUI();                     // patched3: WiFi status screen
void refreshWifiInfoValues();              // patched3: live row refresh
void handleWifiInfoTouch(TS_Point p);      // patched3
void openInfoScreen(DisplayMode mode);     // V12.3: enter a drag-scrolled info overlay
void drawInfoTextScreen();                 // V12.3: full repaint of the active info overlay
void infoScrollReset();                    // V12.3: disable HW vertical scroll (restore normal)
void handleInfoTextTouch(TS_Point p);      // V12.3: drag-scroll / close for info overlays
void wifiForgetAndRestart();               // patched3: defined in patched3_wifi.ino
const char* wifiGetPortalCode();           // patched3: random WPA2 code shown on TFT during setup
void wifiNotifyStackComplete();            // stack-done WebSocket event
void wifiNotifyTestAlert();                // pings webview beep on TEST ALERT press
void redrawCurrentScreen();                // full repaint of currentMode
void wifiPushSettings();                   // push buildSettingsJson to all WS clients


// ─────────────────────────────────────────────────────────────────────────────
// V17b: fireTriggerLed — centralised active-low LED drive
// Active-low wiring: 3.3V → LED → A3. HIGH=off, LOW=on.
// Uses currentLedDuty and ledEnabled.
// ─────────────────────────────────────────────────────────────────────────────

// ─────────────────────────────────────────────────────────────────────────────
// V17b: BRIGHTNESS_SETTINGS screen
// Two rows: Screen Brightness, LED Brightness (with LED enable toggle).
// Adj buttons at y=175 control the selected row.
// Horizontal slider per row: drag to set value.
// ─────────────────────────────────────────────────────────────────────────────




// ─────────────────────────────────────────────────────────────────────────────
// V17b: SENSOR_INFO screen
// Shows MCPS, signal %, status description, on/off toggle for TOF sensor.
// ─────────────────────────────────────────────────────────────────────────────



// ─────────────────────────────────────────────────────────────────────────────
// Forward reference needed: handleCalGraphTouch calls drawFovInfoUI before its fwd decl
void drawFovInfoUI();
void centerStaticText(const char* txt, int y, uint8_t size);

// V17b: CAL_GRAPH — scatter plot of calibration points + regression line
// Layout:
//   x=0..8:   rotated Y-axis title "FOV(mm)" (chars stacked vertically)
//   x=9..28:  Y-axis tick numbers (5 chars * 6px)
//   x=29:     Y axis line
//   x=30..191: plot area
//   x=192..239: legend (stats)
//   top-right (x=205..238, y=2..35): X close button (drawn first so plot doesn't overwrite)
//   bottom (pB+4..pB+20): X axis title "Distance (mm)"
//   pT=40 (room above for close btn), pB=290
// ─────────────────────────────────────────────────────────────────────────────
void drawCalGraphUI() {
  tft.fillScreen(THEME_BG);

  // Guard: with no points, the array reads below would be uninitialised.
  // Show a friendly placeholder and bail out instead of plotting garbage.
  // V11 fix: bound the plot by pointsCaptured (slots actually filled), NOT
  // nPoints — the latter is the target count for the *next* calibration and
  // can exceed the filled slots if the user edits "Cal Points" in CAL_SETTINGS.
  int n = constrain(pointsCaptured, 0, 20);
  if (n <= 0) {
    btnGraphBack.draw(tft);
    setSmoothFont(1);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    centerStaticText("No calibration points", 150, 1);
    centerStaticText("to plot.", 170, 1);
    return;
  }

  // ── Plot area margins ── full width; legend overlays top-right corner of plot
  const int pL = 45, pR = 234, pT = 40, pB = 282;
  const int pW = pR - pL;   // 204
  const int pH = pB - pT;   // 250

  // ── Close button (X) — top-right, drawn first ──
  btnGraphBack.draw(tft);

  // ── Y-axis title "FOV(mm)" rotated 90° CCW via pixel-level transpose ──
  {
    const char* yTitle = "FOV(mm)";
    const int cw = 6, ch = 8;
    int tw = strlen(yTitle) * cw;   // = 42px
    GFXcanvas16 tc(tw, ch);
    tc.fillScreen(TFT_BLACK);
    tc.setFont(); tc.setTextSize(1); tc.setTextColor(themedText(COLOR_LIGHTGREY));
    tc.setCursor(0, 0); tc.print(yTitle);
    // Rotate 90° CCW: (px, py) -> screen (drawX + py, drawY + tw-1-px)
    int drawX = 1;
    int drawY = pT + (pH - tw) / 2;
    tft.fillRect(0, drawY, drawX + ch, tw, THEME_BG);
    for (int px = 0; px < tw; px++)
      for (int py = 0; py < ch; py++)
        if (tc.getPixel(px, py) != TFT_BLACK)
          tft.drawPixel(drawX + py, drawY + (tw - 1 - px), COLOR_LIGHTGREY);
  }

  // ── Data range ──
  float xMin = distPoints[0], xMax = distPoints[0];
  float yMin = fovPoints[0],  yMax = fovPoints[0];
  for (int i = 1; i < n; i++) {
    if (distPoints[i] < xMin) xMin = distPoints[i];
    if (distPoints[i] > xMax) xMax = distPoints[i];
    if (fovPoints[i]  < yMin) yMin = fovPoints[i];
    if (fovPoints[i]  > yMax) yMax = fovPoints[i];
  }
  float xRange = xMax - xMin; if (xRange < 1.0f) xRange = 1.0f;
  float yRange = yMax - yMin; if (yRange < 0.01f) yRange = 0.01f;
  float xLo = xMin - xRange * 0.1f, xHi = xMax + xRange * 0.1f;
  float yLo = yMin - yRange * 0.1f, yHi = yMax + yRange * 0.1f;

  auto toScreenX = [&](float x) -> int { return pL + (int)((x - xLo) / (xHi - xLo) * pW); };
  auto toScreenY = [&](float y) -> int { return pB - (int)((y - yLo) / (yHi - yLo) * pH); };

  // ── Axes ──
  tft.drawLine(pL, pT, pL, pB, TFT_WHITE);
  tft.drawLine(pL, pB, pR, pB, TFT_WHITE);

  // ── Tick marks + labels (built-in 6x8 font) ──
  tft.setFont(); tft.setTextSize(1); tft.setTextColor(themedText(TFT_WHITE));
  // X major ticks: 4; 3 minor ticks between each pair
  for (int t = 0; t <= 3; t++) {
    float xv = xLo + (xHi - xLo) * t / 3.0f;
    int sx = toScreenX(xv);
    tft.drawLine(sx, pB, sx, pB + 4, TFT_WHITE);
    char lbl[8]; snprintf(lbl, sizeof(lbl), "%d", (int)roundf(xv));
    int lw = strlen(lbl) * 6;
    int lblX = constrain(sx - lw / 2, pL, pR - lw);  // clamp so label stays on screen
    tft.setCursor(lblX, pB + 6); tft.print(lbl);
    if (t < 3) {
      for (int m = 1; m <= 3; m++) {
        int msx = toScreenX(xv + (xHi - xLo) * m / 12.0f);
        tft.drawLine(msx, pB, msx, pB + 2, TFT_WHITE);
      }
    }
  }
  // Y major ticks: 5; 2 minor ticks between each pair
  for (int t = 0; t <= 4; t++) {
    float yv = yLo + (yHi - yLo) * t / 4.0f;
    int sy = toScreenY(yv);
    tft.drawLine(pL - 4, sy, pL, sy, TFT_WHITE);
    char lbl[8]; snprintf(lbl, sizeof(lbl), "%.2f", yv);
    int lw = strlen(lbl) * 6;
    tft.setCursor(pL - 5 - lw, sy - 3); tft.print(lbl);
    if (t < 4) {
      for (int m = 1; m <= 2; m++) {
        int msy = toScreenY(yv + (yHi - yLo) * m / 12.0f);
        tft.drawLine(pL - 2, msy, pL, msy, TFT_WHITE);
      }
    }
  }

  // ── X-axis title "Distance (mm)" — centred below ticks ──
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  const char* xTitle = "Distance (mm)";
  int xTitleW = strlen(xTitle) * 6;
  tft.setCursor(pL + (pW - xTitleW) / 2, pB + 22); tft.print(xTitle);

  // ── Regression curve (pixel-space cal → FOV = fovAt() is a 1/x curve) ──
  {
    int prevx = -1, prevy = 0;
    const int SAMPLES = 24;
    for (int s = 0; s <= SAMPLES; s++) {
      float d  = xLo + (xHi - xLo) * s / (float)SAMPLES;
      int   cx = toScreenX(d);
      int   cy = constrain(toScreenY(fovAt(d)), pT, pB);
      if (prevx >= 0) tft.drawLine(prevx, prevy, cx, cy, COLOR_BLUEGREEN);
      prevx = cx; prevy = cy;
    }
  }

  // ── Data points — colour by residual magnitude ──
  for (int i = 0; i < n; i++) {
    float predicted = fovAt(distPoints[i]);
    float resid = fovPoints[i] - predicted;
    bool within = fabsf(resid) < CALIB_ERROR;
    int sx = toScreenX(distPoints[i]);
    int sy = toScreenY(fovPoints[i]);
    tft.fillCircle(sx, sy, 3, within ? COLOR_PUREGREEN : COLOR_RED);
    tft.drawCircle(sx, sy, 4, within ? TFT_WHITE : COLOR_DARKGREY);
  }

  // ── Legend — top-right corner overlay on the plot (semi-opaque dark box) ──
  // Box: x=pR-100..pR, y=pT..pT+118
  setSmoothFont(1);
  char buf[32];
  const int lbx = pR - 75, lby = pT + 10;
  const int lbw = 75,       lbh = 100;
  tft.fillRect(lbx, lby, lbw, lbh, 0x0841);
  tft.setFont(); tft.setTextSize(1);           // tiny 6x8 font for legend

  // Legend: label + value on same line, 3 sig figs, trailing zeros stripped
  auto fmt3 = [&](char* b, int bsz, float v) {
    snprintf(b, bsz, "%.3f", v);
    // Strip trailing zeros (keep at least one decimal)
    char* dot = strchr(b, '.');
    if (dot) {
      int len = strlen(b);
      while (len > (dot - b + 2) && b[len-1] == '0') b[--len] = 0;
    }
  };
  auto legendRow = [&](int y, const char* label, const char* val, uint16_t col) {
    tft.fillRect(lbx + 1, y, lbw - 2, 12, 0x0841); // inset so border not erased
    char combined[24]; snprintf(combined, sizeof(combined), "%s%s", label, val);
    int startX = lbx + lbw - (int)strlen(combined) * 6 - 3;
    tft.setTextColor(themedText(COLOR_LIGHTGREY)); tft.setCursor(startX, y + 2); tft.print(label);
    tft.setTextColor(col); tft.setCursor(startX + (int)strlen(label) * 6, y + 2); tft.print(val);
  };

  uint16_t r2Col   = (CALIB_R2 > 0.995f)   ? COLOR_PUREGREEN : (CALIB_R2 > 0.98f)   ? COLOR_YELLOW : COLOR_ORANGE;
  uint16_t rmseCol = (CALIB_ERROR < 0.02f)  ? COLOR_PUREGREEN : (CALIB_ERROR < 0.05f) ? COLOR_YELLOW : COLOR_ORANGE;

  snprintf(buf, sizeof(buf), "%.4f", CALIB_R2);   // R² at 4 dp (≈1.000 hides a near-perfect fit)
  {
    tft.fillRect(lbx + 1, lby + 4, lbw - 2, 12, 0x0841);
    char combined[20]; snprintf(combined, sizeof(combined), "R :%s", buf); // space placeholder for superscript
    int startX = lbx + lbw - (int)strlen(combined) * 6 - 3 + 6; // shift right 1 char for raised 2
    // Print "R" at normal baseline
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(startX - 6, lby + 6); tft.print("R");
    // Print "2" raised 3px (superscript)
    tft.setCursor(startX, lby + 3); tft.print("2");
    // Print ":"
    tft.setCursor(startX + 6, lby + 6); tft.print(":");
    // Print value
    tft.setTextColor(r2Col);
    tft.setCursor(startX + 12, lby + 6); tft.print(buf);
  }
  fmt3(buf, sizeof(buf), CTRLX);       legendRow(lby + 18, "m:", buf, TFT_WHITE);
  fmt3(buf, sizeof(buf), CTRLY);       legendRow(lby + 32, "b:", buf, TFT_WHITE);
  fmt3(buf, sizeof(buf), CALIB_ERROR); legendRow(lby + 46, "RMSE:", buf, rmseCol);
// Dot rows right-aligned: circle on left, text immediately to its right.
  { int ty = lby + 66;
    int textW = 5 * 6;                    // "<RMSE" = 5 chars * 6px
    int textX = lbx + lbw - 3 - textW;    // text right-aligned to legend edge (3px inset)
    int cxDot = textX - 8;                // circle 8px left of text start (4px radius + 4px gap)
    tft.fillCircle(cxDot, ty + 4, 3, COLOR_PUREGREEN);
    tft.drawCircle(cxDot, ty + 4, 4, TFT_WHITE);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(textX, ty); tft.print("<RMSE");
  }
  { int ty = lby + 78;
    int textW = 5 * 6;                    // ">RMSE"
    int textX = lbx + lbw - 3 - textW;
    int cxDot = textX - 8;
    tft.fillCircle(cxDot, ty + 4, 3, COLOR_RED);
    tft.drawCircle(cxDot, ty + 4, 4, COLOR_DARKGREY);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(textX, ty); tft.print(">RMSE");
  }
  tft.drawRect(lbx, lby, lbw, lbh, 0x4208); // border last so rows don't erase it
}

void handleCalGraphTouch(TS_Point p) {
  if (btnGraphBack.contains(p.x, p.y)) {
    currentMode = FOV_INFO; drawFovInfoUI();
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// V17b: MEM_INFO screen — heap usage and misc system info
// ─────────────────────────────────────────────────────────────────────────────
void drawLeftBoxedText(const char*, int, int, uint16_t);  // fwd ref
void setSmoothFont(uint8_t);  // fwd ref
void drawMemInfoUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("MEMORY & INFO", 5, 5, COLOR_DARKGREEN);
  btnMemClose.draw(tft);

  const uint32_t spriteBytes = SPRITE_BYTES;

  uint32_t freeHeap   = ESP.getFreeHeap();
  uint32_t totalHeap  = ESP.getHeapSize();
  uint32_t minFree    = ESP.getMinFreeHeap();
  uint32_t psramFree  = ESP.getFreePsram();
  uint32_t psramTotal = ESP.getPsramSize();
  uint32_t usedHeap   = totalHeap > freeHeap ? totalHeap - freeHeap : 0;

  setSmoothFont(1);
  char buf[40];

  // ── Bar color palette (RGB565) ────────────────────────────────────────
  const uint16_t C_HEAP_USED  = 0xFD40;  // warm amber  — heap used
  const uint16_t C_HEAP_FREE  = 0x03A0;  // dark green  — heap free bg
  const uint16_t C_PSRAM_SPR  = 0x5011;  // purple      — sprite alloc
  const uint16_t C_PSRAM_USE  = 0x2BBF;  // slate blue  — other PSRAM used
  const uint16_t C_PSRAM_FREE = 0x03EF;  // dark teal   — free PSRAM

  const int BX = 15, BW = 210, BH = 14, IW = BW - 2;

  // ── Stacked horizontal usage bar ─────────────────────────────────────
  // s1 = leftmost segment, s2 overlaid on far-left of s1, bgCol fills rest.
  // lowestMark draws a white 2px tick at the historical high-water mark for used.
  auto drawBar = [&](int by, uint32_t total,
                     uint32_t s1, uint16_t c1,
                     uint32_t s2, uint16_t c2,
                     uint16_t bgCol, uint32_t lowestMark) {
    if (total == 0) return;
    tft.fillRoundRect(BX, by, BW, BH, 3, 0x1082);
    tft.fillRect(BX+1, by+1, IW, BH-2, bgCol);
    int w1 = constrain((int)((uint64_t)s1 * IW / total), 0, IW);
    if (w1 > 0) tft.fillRect(BX+1, by+1, w1, BH-2, c1);
    int w2 = constrain((int)((uint64_t)s2 * IW / total), 0, w1);
    if (w2 > 0) tft.fillRect(BX+1, by+1, w2, BH-2, c2);
    tft.drawRoundRect(BX, by, BW, BH, 3, 0x528A);
    if (lowestMark > 0 && lowestMark < total) {
      int tx = BX + 1 + constrain((int)((uint64_t)(total - lowestMark) * IW / total), 0, IW-2);
      tft.fillRect(tx, by-3, 2, BH+6, TFT_WHITE);
    }
  };

  // ── Heap bar (y=58) — "HEAP" label sits 7 px above ──────────────────
  tft.setFont(NULL); tft.setTextSize(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(BX + 2, 43); tft.print("HEAP");
  setSmoothFont(1);
  drawBar(58, totalHeap, usedHeap, C_HEAP_USED, 0, 0, C_HEAP_FREE, minFree);

  // ── Heap legend — 3 stacked lines: [indicator] label   value ─────────
  // Bar bottom = y=71.  FreeSans9 ascenders ~10 px above baseline; baseline
  // at 96 puts cap-tops at 86 — 15 px clear of bar bottom.  Lines 20 px apart.
  auto heapLeg = [&](int y, const char* label, const char* val,
                      uint16_t valCol, bool isTick) {
    if (isTick) tft.fillRect(15, y-10, 2, 11, TFT_WHITE);
    else        tft.fillRect(15, y-9,  8,  8, valCol);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(27, y); tft.print(label);
    int16_t bx1, by1; uint16_t btw, bth;
    tft.getTextBounds(val, 0, 0, &bx1, &by1, &btw, &bth);
    tft.setTextColor(valCol);
    tft.setCursor(228 - (int)btw - bx1, y); tft.print(val);
  };

  snprintf(buf, sizeof(buf), "%luK", usedHeap / 1024);
  heapLeg( 96, "used", buf, C_HEAP_USED,    false);

  snprintf(buf, sizeof(buf), "%luK", minFree / 1024);
  heapLeg(116, "low",  buf, TFT_WHITE,       true);

  snprintf(buf, sizeof(buf), "%luK/%luK", freeHeap/1024, totalHeap/1024);
  heapLeg(136, "free", buf, C_HEAP_FREE,     false);

  // ── PSRAM bar + legend (y=164, only when PSRAM present) ──────────────
  // Default nextY (PSRAM absent): 8 px below "free" descent (~139).
  int nextY = 147;
  if (psramTotal > 0) {
    uint32_t psramUsed = psramTotal > psramFree ? psramTotal - psramFree : 0;
    uint32_t sprClamp  = min(spriteBytes, psramUsed);
    tft.setFont(NULL); tft.setTextSize(1);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(BX + 2, 149); tft.print("PSRAM");
    setSmoothFont(1);
    drawBar(164, psramTotal, psramUsed, C_PSRAM_USE, sprClamp, C_PSRAM_SPR, C_PSRAM_FREE, 0);

    // bar bottom = y=178; pLY=196 puts ascenders at 186 — 8 px clear of bar
    int pLY = 196;
    snprintf(buf, sizeof(buf), "%luK buf", spriteBytes / 1024);
    tft.fillRect(15, pLY-9, 8, 8, C_PSRAM_SPR);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(27, pLY); tft.print(buf);

    snprintf(buf, sizeof(buf), "%luK/%luK", psramFree/1024, psramTotal/1024);
    int16_t x1, y1; uint16_t tw, th;
    tft.getTextBounds(buf, 0, 0, &x1, &y1, &tw, &th);
    int rX = 225 - (int)tw;
    tft.fillRect(rX-12, pLY-9, 8, 8, C_PSRAM_FREE);
    tft.setTextColor(C_PSRAM_FREE);
    tft.setCursor(rX, pLY); tft.print(buf);

    nextY = 203;
  }

  // ── Separator ─────────────────────────────────────────────────────────
  tft.drawFastHLine(10, nextY+4, 220, 0x2965);

  // ── System info ───────────────────────────────────────────────────────
  int ly = nextY + 18;
  auto infoRow = [&](const char* label, const char* val, uint16_t col) {
    tft.setTextColor(themedText(COLOR_LIGHTGREY)); tft.setCursor(15, ly); tft.print(label);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(val, 0, 0, &x1, &y1, &w, &h);
    tft.setTextColor(col); tft.setCursor(225 - (int)w - x1, ly); tft.print(val);
    ly += 26;
  };

  infoRow("Firmware:", FIRMWARE_VERSION, COLOR_GREENYELLOW);
  {
    char sdk[11]; snprintf(sdk, sizeof(sdk), "%s", ESP.getSdkVersion());
    snprintf(buf, sizeof(buf), "%lu MHz-%s", (unsigned long)getCpuFrequencyMhz(), sdk);
    infoRow("CPU:", buf, TFT_WHITE);
  }
  {
    // used / running-slot capacity — NOT sketch + getFreeSketchSpace(), which
    // adds the identically-sized standby OTA (rollback) slot and misreads as
    // ~1.8 MB of free app space the firmware can't actually grow into.
    uint32_t sk = ESP.getSketchSize();
    const esp_partition_t* run = esp_ota_get_running_partition();
    uint32_t slot = run ? run->size : 0x1D0000;
    snprintf(buf, sizeof(buf), "%luK/%luK",
             (unsigned long)(sk/1024), (unsigned long)(slot/1024));
    infoRow("Flash:", buf, themedText(COLOR_LIGHTGREY));
  }
  // V12.5 (F2): why the previous session ended (see g_bootReasonLine). Amber for
  // crash-class resets (WDT / panic / brownout / rollback), grey for clean ones.
  if (g_bootReasonLine[0]) {
    bool clean = strncmp(g_bootReasonLine, "POWERON", 7) == 0 ||
                 strncmp(g_bootReasonLine, "SW-RESTART", 10) == 0 ||
                 strncmp(g_bootReasonLine, "EXT-RESET", 9) == 0;
    infoRow("Last:", g_bootReasonLine, clean ? themedText(COLOR_LIGHTGREY) : 0xFD40 /* amber */);
  }
}

void handleMemInfoTouch(TS_Point p) {
  if (btnMemClose.contains(p.x, p.y)) {
    currentMode = APP_SETTINGS; drawAppSettingsUI();
  }
}

// V11: ABOUT screen — opened by tapping the calib/version block on the main
// screen. Shows firmware build info, sketch footprint, and developer credit.
void drawAboutUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("ABOUT", 5, 5, COLOR_DARKBLUE);
  btnAboutClose.draw(tft);
  btnAboutInfo.draw(tft);

  // Big firmware banner — same FreeSans18 used elsewhere for emphasis.
  setSmoothFont(3);
  tft.setTextColor(themedText(COLOR_GREENYELLOW));
  int16_t bx, by; uint16_t bw, bh;
  const char* banner = FIRMWARE_VERSION;
  tft.getTextBounds(banner, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor((240 - bw) / 2 - bx, 75);
  tft.print(banner);

  // Subtitle under banner
  setSmoothFont(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  const char* sub = "Field-of-view calculator";
  tft.getTextBounds(sub, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor((240 - bw) / 2 - bx, 100);
  tft.print(sub);

  // ── Detail rows (label left, value right, same pattern as MEM_INFO) ──
  char buf[40];
  int ly = 132;
  const int lStep = 22;

  auto row = [&](const char* label, const char* val, uint16_t valCol) {
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    tft.setCursor(15, ly); tft.print(label);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(val, 0, 0, &x1, &y1, &w, &h);
    tft.setTextColor(valCol);
    tft.setCursor(225 - w - x1, ly); tft.print(val);
    ly += lStep;
  };

  // Build date+time come from __DATE__/__TIME__ macros — fixed at compile time,
  // so this stamp tells the user exactly which build is on the device.
  snprintf(buf, sizeof(buf), "%s", __DATE__);
  row("Built:", buf, TFT_WHITE);

  snprintf(buf, sizeof(buf), "%s", __TIME__);
  row("Time:", buf, TFT_WHITE);

  // Sketch footprint — actual binary size flashed to the chip.
  uint32_t sketchKB = ESP.getSketchSize() / 1024;
  snprintf(buf, sizeof(buf), "%lu KB", (unsigned long)sketchKB);
  row("Sketch size:", buf, COLOR_GREENYELLOW);

  // Free OTA space tells the user how much room is left for an over-the-air update.
  uint32_t freeKB = ESP.getFreeSketchSpace() / 1024;
  snprintf(buf, sizeof(buf), "%lu KB", (unsigned long)freeKB);
  row("Free flash:", buf, COLOR_LIGHTGREY);

  // Chip identity — useful when comparing units / debugging in the field.
  snprintf(buf, sizeof(buf), "%s rev%d", ESP.getChipModel(), ESP.getChipRevision());
  // Truncate if needed to fit
  if (strlen(buf) > 16) buf[16] = 0;
  row("Chip:", buf, COLOR_LIGHTGREY);

  // ── Developer credit, set apart visually with a divider line ──
  // Pull divider up close to the last row so the Developed by / Travis Olds
  // block has enough room between it and the screen bottom.
  ly -= 4;
  tft.drawFastHLine(30, ly, 180, 0x39E7);  // muted grey divider
  // Gap is generous — FreeSans9 ascenders reach ~10px above the cursor
  // baseline, so 18px keeps the line clear of the "Developed by" cap-tops.
  ly += 18;

  setSmoothFont(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  const char* devLine = "Developed by";
  tft.getTextBounds(devLine, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor((240 - bw) / 2 - bx, ly);
  tft.print(devLine);
  // FreeSans12 ascenders reach ~13px above the cursor baseline, so 28px
  // between the two baselines keeps "Travis Olds" comfortably clear of the
  // "Developed by" descenders rather than touching them.
  ly += 28;

  setSmoothFont(2);
  tft.setTextColor(themedText(0xC81F));  // bright purple — matches FOV Info accent
  const char* devName = "Travis Olds";
  tft.getTextBounds(devName, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor((240 - bw) / 2 - bx, ly);
  tft.print(devName);
}

void handleAboutTouch(TS_Point p) {
  if (btnAboutInfo.contains(p.x, p.y)) {
    openInfoScreen(RECOVERY_HELP);
    return;
  }
  if (btnAboutClose.contains(p.x, p.y)) {
    currentMode = MAIN; drawMainScreen();
  }
}

// ────────────────────────────────────────────────────────────────────────────
// V11: SCREEN TIMEOUTS + COLOR THEMES screen
// ────────────────────────────────────────────────────────────────────────────
// Reached from BRIGHTNESS_SETTINGS via the new SCREEN TIMEOUTS button.
// Top half: dim and sleep timeout adjusters (-/+ buttons step in 30s slices).
// Bottom half: 4-swatch theme picker; tapping a swatch applies and persists.

// Step sizes for the timeout buttons. Dim moves in 30s steps, sleep in 60s
// steps so users can dial in coarse durations without dozens of taps.
const unsigned long DIM_STEP_MS   = 30UL * 1000UL;
const unsigned long SLEEP_STEP_MS = 60UL * 1000UL;
const unsigned long DIM_MIN_MS    = 30UL * 1000UL;     // 30s minimum
const unsigned long DIM_MAX_MS    = 30UL * 60UL * 1000UL;   // 30 min max
const unsigned long SLEEP_MIN_MS  = 60UL * 1000UL;     // 1 min minimum
const unsigned long SLEEP_MAX_MS  = 60UL * 60UL * 1000UL;   // 60 min max

// Sensor auto-power-off: after this long with no activity at all (no device
// touch, no camera-trigger pulse, no web menu/settings change — the 1 Hz web
// latency ping does NOT count), the TOF sensor and the accelerometer are
// powered down. The next interaction brings them back. Independent of, and
// much longer than, the screen dim/sleep timeouts above.
const unsigned long SENSOR_IDLE_OFF_MS = 30UL * 60UL * 1000UL;   // 30 minutes

// Helper: format milliseconds as "Mm Ss" (or "Ss" if under a minute).
static void formatDurationMs(unsigned long ms, char* out, size_t outSize) {
  unsigned long totalS = ms / 1000UL;
  unsigned long m = totalS / 60UL;
  unsigned long s = totalS % 60UL;
  if (m == 0) snprintf(out, outSize, "%lus", s);
  else if (s == 0) snprintf(out, outSize, "%lum", m);
  else snprintf(out, outSize, "%lum %lus", m, s);
}

void refreshScreenTimeoutValues(bool force) {
  // V11: every dynamic strip is rendered into distSprite first and blitted in
  // one shot. Doing fillRect-then-print directly on the TFT meant each strip
  // briefly flashed bg-only between the clear and the text — visible as a
  // bar of flicker on every tint drag sample. Sprite-built rows blit
  // atomically: the new bg AND the text appear together, in one DMA pass.
  // The sprite size (240x45) covers each row's full width and height with
  // room to spare; we reuse the same buffer for all three strips.
  //
  // Repaint triggers also expanded: tint changes (lastIntensity diff) now
  // refresh the dim and sleep value strips too. Without that, the strip
  // backgrounds kept the OLD bg shade until next full redraw — visible on
  // CLASSIC at max tint as patchy "the bg around buttons went black but
  // the strips behind values are still grey."
  static unsigned long lastDim = 0, lastSleep = 0;
  static int lastTheme = -1;
  static int lastIntensity = -1;
  if (force) { lastDim = 0; lastSleep = 0; lastTheme = -1; lastIntensity = -1; }

  bool tintChanged = (lastIntensity != themeIntensity);
  bool themeChanged = (lastTheme != currentThemeIndex);

  // ── Dim value strip ── valSprite is 120w×40h, blitted at TFT (60, 68) so
  // it spans the full vertical extent of the dim button row (y=68..104).
  // This overpaints any stale bg pixels above and below the value text that
  // would otherwise stay at the previous tint shade. Atomic blit = no flash.
  if (force || lastDim != dimTimeoutMs || tintChanged || themeChanged) {
    char buf[16]; formatDurationMs(dimTimeoutMs, buf, sizeof(buf));
    valSprite.fillScreen(THEME_BG);
    valSprite.setFont(&FreeSans12pt7b);
    valSprite.setTextSize(1);
    valSprite.setTextColor(themedText(COLOR_GREENYELLOW));
    int16_t bx, by; uint16_t bw, bh;
    valSprite.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    // Sprite-x=60 = TFT-x=120 (screen midline). Sprite-y=27 puts the
    // FreeSans12 baseline near sprite mid (sprite is 40 tall).
    valSprite.setCursor(60 - bw / 2 - bx, 27);
    valSprite.print(buf);
    tft.drawRGBBitmap(60, 68, valSprite.getBuffer(), 120, 40);
    lastDim = dimTimeoutMs;
  }

  // ── Sleep value strip ── same pattern, blitted at TFT (60, 140) covering
  // the full sleep button row (y=140..176).
  if (force || lastSleep != sleepTimeoutMs || tintChanged || themeChanged) {
    char buf[16]; formatDurationMs(sleepTimeoutMs, buf, sizeof(buf));
    valSprite.fillScreen(THEME_BG);
    valSprite.setFont(&FreeSans12pt7b);
    valSprite.setTextSize(1);
    valSprite.setTextColor(COLOR_PURPLE);
    int16_t bx, by; uint16_t bw, bh;
    valSprite.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    valSprite.setCursor(60 - bw / 2 - bx, 27);
    valSprite.print(buf);
    tft.drawRGBBitmap(60, 140, valSprite.getBuffer(), 120, 40);
    lastSleep = sleepTimeoutMs;
  }

  // ── TINT row ── label + slider, y=188..210
  // Repaint when intensity changes OR theme changes (latter shifts label
  // color via themedText, and shifts slider fill color).
  if (force || tintChanged || themeChanged) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextSize(1);
    distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
    distSprite.setCursor(15, 17); distSprite.print("TINT:");
    // Slider in sprite coords: bar at sprite (70, 5), width 155, height 12.
    // Center marker tick spans sprite y=3..16.
    int sliderX = 70, sliderY = 5, sliderW = 155, sliderH = 12;
    int filled = (themeIntensity * sliderW) / 100;
    // Rounded slider: full track in dark, active pill on top, rounded border last.
    distSprite.fillRoundRect(sliderX, sliderY, sliderW, sliderH, 2, COLOR_DARKGREY);
    if (filled > 4) {
      distSprite.fillRoundRect(sliderX, sliderY, filled, sliderH, 2, themedText(COLOR_PUREGREEN));
    }
    distSprite.drawRoundRect(sliderX, sliderY, sliderW, sliderH, 2, themedText(COLOR_LIGHTGREY));
    // Center marker (intensity=50 = no shift) — short tick above and below.
    int centerX = sliderX + sliderW / 2;
    distSprite.drawFastVLine(centerX, sliderY - 2, sliderH + 4, themedText(TFT_WHITE));
    tft.drawRGBBitmap(0, 188, distSprite.getBuffer(), 240, 22);
    lastTheme = currentThemeIndex;
    lastIntensity = themeIntensity;
  }
}

void drawScreenTimeoutUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("TIMEOUTS", 5, 5, COLOR_TEAL);

  // ── Dim row ──
  // Both labels MUST set font before printing — the Button::draw calls below
  // change the font to whatever the buttons use (FreeSans12 for the big -/+).
  // Without setSmoothFont(1) before each label print, "AUTO SLEEP" rendered
  // in the bigger font and didn't match "SCREEN DIM".
  setSmoothFont(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(15, 58); tft.print("SCREEN DIM");
  btnDimMinus.draw(tft);
  btnDimPlus.draw(tft);

  // ── Sleep row ──
  setSmoothFont(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(15, 130); tft.print("AUTO SLEEP");
  btnSleepMinus.draw(tft);
  btnSleepPlus.draw(tft);

  // ── Theme picker — 4 swatches in a row ──
  // Each swatch: filled with its theme's accent color, 2px white border on
  // the active theme so the current selection reads at a glance. The first
  // letter of the theme name is drawn black on bright swatches and white on
  // dark ones (computed per-swatch from RGB565 luminance) so the legend is
  // always readable regardless of the active theme.
  Button* swatches[NUM_THEMES] = { &btnTheme0, &btnTheme1, &btnTheme2, &btnTheme3 };
  for (int i = 0; i < NUM_THEMES; i++) {
    Button* b = swatches[i];
    uint16_t sw = THEMES[i].swatch;
    tft.fillRoundRect(b->x, b->y, b->w, b->h, UI_RADIUS, sw);
    if (i == currentThemeIndex) {
      // Bold white border to mark active. Inner border uses radius-1 so the
      // two stroke lines stay parallel through the curve rather than diverging.
      tft.drawRoundRect(b->x,     b->y,     b->w,     b->h,     UI_RADIUS,     TFT_WHITE);
      tft.drawRoundRect(b->x + 1, b->y + 1, b->w - 2, b->h - 2, UI_RADIUS - 1, TFT_WHITE);
    } else {
      tft.drawRoundRect(b->x, b->y, b->w, b->h, UI_RADIUS, COLOR_DARKGREY);
    }
    // Compute approximate brightness of the swatch fill: extract 5/6/5 RGB
    // channels from the RGB565 value and weight green more heavily (eye is
    // most sensitive to green). Threshold ~120 is the rough boundary where
    // white text starts losing contrast against the bg.
    uint8_t r5 = (sw >> 11) & 0x1F;
    uint8_t g6 = (sw >> 5)  & 0x3F;
    uint8_t b5 =  sw        & 0x1F;
    // Scale to 0-255 (5-bit r,b * 8; 6-bit g * 4) and weight (~30/59/11).
    int luma = (r5 * 8 * 30 + g6 * 4 * 59 + b5 * 8 * 11) / 100;
    uint16_t letterCol = (luma > 120) ? TFT_BLACK : TFT_WHITE;

    setSmoothFont(2);
    tft.setTextColor(letterCol);
    char letter[2] = { THEMES[i].name[0], 0 };
    int16_t lx, ly; uint16_t lw, lh;
    tft.getTextBounds(letter, 0, 0, &lx, &ly, &lw, &lh);
    tft.setCursor(b->x + (b->w - lw) / 2 - lx, b->y + (b->h + lh) / 2 - 2);
    tft.print(letter);
  }

  btnThemeBack.draw(tft);
  refreshScreenTimeoutValues(true);
}

void applyTheme(int idx) {
  if (idx < 0 || idx >= NUM_THEMES) return;
  if (idx == currentThemeIndex) return;
  currentThemeIndex = idx;
  refreshCachedThemeBg();
  saveDisplayPrefs();
  // V11: full redraw is the right call here — a theme tap is a single
  // discrete event (one tap = one repaint), not a continuous drag, so
  // fillScreen once produces a clean snap to the new theme without
  // perceptible flicker. Earlier attempts at partial repaints created
  // banding/coverage bugs and offered no real speed advantage for a
  // one-shot event.
  tintDragActive = false;
  drawScreenTimeoutUI();
}

void handleScreenTimeoutTouch(TS_Point p) {
  // Dim ±
  // V11 fix: defer NVS writes via displayPrefsDirty (same pattern as tint
  // drag). Tap-and-hold no longer hammers flash; loop() flushes ~500 ms
  // after the last edit, and GO BACK flushes immediately.
  if (btnDimMinus.contains(p.x, p.y)) {
    if (dimTimeoutMs > DIM_MIN_MS + DIM_STEP_MS / 2) dimTimeoutMs -= DIM_STEP_MS;
    else dimTimeoutMs = DIM_MIN_MS;
    displayPrefsDirty = true; lastTintDragMs = millis();
    refreshScreenTimeoutValues(false);
    return;
  }
  if (btnDimPlus.contains(p.x, p.y)) {
    if (dimTimeoutMs + DIM_STEP_MS <= DIM_MAX_MS) dimTimeoutMs += DIM_STEP_MS;
    else dimTimeoutMs = DIM_MAX_MS;
    // Keep dim < sleep so the screen actually has a chance to dim before
    // sleeping. If +step would equal/exceed sleep, push sleep up too.
    if (dimTimeoutMs >= sleepTimeoutMs) sleepTimeoutMs = dimTimeoutMs + SLEEP_STEP_MS;
    if (sleepTimeoutMs > SLEEP_MAX_MS) sleepTimeoutMs = SLEEP_MAX_MS;
    displayPrefsDirty = true; lastTintDragMs = millis();
    refreshScreenTimeoutValues(false);
    return;
  }
  // Sleep ±
  if (btnSleepMinus.contains(p.x, p.y)) {
    if (sleepTimeoutMs > SLEEP_MIN_MS + SLEEP_STEP_MS / 2) sleepTimeoutMs -= SLEEP_STEP_MS;
    else sleepTimeoutMs = SLEEP_MIN_MS;
    // Keep sleep > dim — if user shrinks sleep below dim, drag dim down too.
    if (sleepTimeoutMs <= dimTimeoutMs && dimTimeoutMs > DIM_MIN_MS) {
      dimTimeoutMs = (sleepTimeoutMs > DIM_STEP_MS) ? sleepTimeoutMs - DIM_STEP_MS : DIM_MIN_MS;
    }
    displayPrefsDirty = true; lastTintDragMs = millis();
    refreshScreenTimeoutValues(false);
    return;
  }
  if (btnSleepPlus.contains(p.x, p.y)) {
    if (sleepTimeoutMs + SLEEP_STEP_MS <= SLEEP_MAX_MS) sleepTimeoutMs += SLEEP_STEP_MS;
    else sleepTimeoutMs = SLEEP_MAX_MS;
    displayPrefsDirty = true; lastTintDragMs = millis();
    refreshScreenTimeoutValues(false);
    return;
  }

  // Theme swatches
  Button* swatches[NUM_THEMES] = { &btnTheme0, &btnTheme1, &btnTheme2, &btnTheme3 };
  for (int i = 0; i < NUM_THEMES; i++) {
    if (swatches[i]->contains(p.x, p.y)) {
      applyTheme(i);
      return;
    }
  }

  // V11: Tint slider drag — slider at x=70..225, y=193..205 on the THEME row.
  // Touch zone y=183..215.
  //
  // Behavior: during the drag we ONLY repaint the slider thumb position (so
  // the user sees their finger tracked). The rest of the page — including
  // the slider strip's BG — stays at the OLD tint. The new tint does NOT
  // apply anywhere yet; doing partial repaints with the new tint while the
  // rest of the page is still at the old tint produced visible bands of
  // different shades, which is what the user reported. On finger lift
  // (detected in loop()) we do ONE clean drawScreenTimeoutUI() with the
  // final tint — fillScreen + redraw, all at the new shade, atomic.
  //
  // Tradeoff: no live preview of the bg shade while dragging. User sees
  // the slider thumb move, releases finger, screen snaps to new shade. This
  // is the standard pattern for intensity sliders on unbuffered displays.
  if (p.x >= 65 && p.x <= 230 && p.y >= 183 && p.y <= 215) {
    int sliderX_touch = 70, sliderW_touch = 155;
    int valX = constrain((int)p.x, sliderX_touch, sliderX_touch + sliderW_touch);
    int newIntensity = ((valX - sliderX_touch) * 100) / sliderW_touch;
    if (newIntensity != themeIntensity) {
      // Compute the slider's bg using the OLD intensity (so it matches the
      // rest of the screen). Then set the new intensity and only redraw
      // the slider rectangle's fill portion, NOT the bg behind it.
      // This is intentional: themeIntensity must update so the value is
      // remembered for the lift-time full redraw and for save_prefs, but we
      // don't want THEME_BG (which depends on themeIntensity) to apply to
      // anything we draw mid-drag.
      themeIntensity = newIntensity;
      // Slider geometry on the TFT (NOT in any sprite):
      //   bar at x=70..225, y=193..204 (12 tall), drawn directly to TFT.
      // We draw fill + remainder + center marker. We do NOT clear the
      // bg around the slider, so anything the user can see outside the
      // slider rectangle stays at the old tint.
      int sX = 70, sY = 193, sW = 155, sH = 12;
      int filled = (themeIntensity * sW) / 100;
      // Rounded slider: full dark track, rounded active pill, border last.
      tft.fillRoundRect(sX, sY, sW, sH, 2, COLOR_DARKGREY);
      if (filled > 4) {
        tft.fillRoundRect(sX, sY, filled, sH, 2, themedText(COLOR_PUREGREEN));
      }
      tft.drawRoundRect(sX, sY, sW, sH, 2, themedText(COLOR_LIGHTGREY));
      // Center marker tick at intensity=50.
      int centerX = sX + sW / 2;
      tft.drawFastVLine(centerX, sY - 2, sH + 4, themedText(TFT_WHITE));
    }
    tintDragActive = true;
    displayPrefsDirty = true;
    lastTintDragMs = millis();
    return;
  }

  if (btnThemeBack.contains(p.x, p.y)) {
    if (displayPrefsDirty) saveDisplayPrefs();   // flush any pending tint change
    tintDragActive = false;                      // already going to redraw a different screen
    currentMode = BRIGHTNESS_SETTINGS;
    drawBrightnessSettingsUI();
  }
}

// Persistence — stored in a small dedicated NVS namespace so changes here
// can never invalidate the calibration blob in the "calib" namespace.
void loadDisplayPrefs() {
  preferences.begin("display", true);
  dimTimeoutMs      = preferences.getULong("dimMs",   DIM_TIMEOUT_DEFAULT);
  sleepTimeoutMs    = preferences.getULong("sleepMs", SLEEP_TIMEOUT_DEFAULT);
  currentThemeIndex = preferences.getInt  ("theme",   0);
  themeIntensity    = preferences.getInt  ("tint",    50);
  preferences.end();
  // Sanity: clamp loaded values to defined bounds in case the on-disk values
  // were set by an older build with different limits.
  if (dimTimeoutMs   < DIM_MIN_MS)   dimTimeoutMs   = DIM_MIN_MS;
  if (dimTimeoutMs   > DIM_MAX_MS)   dimTimeoutMs   = DIM_MAX_MS;
  if (sleepTimeoutMs < SLEEP_MIN_MS) sleepTimeoutMs = SLEEP_MIN_MS;
  if (sleepTimeoutMs > SLEEP_MAX_MS) sleepTimeoutMs = SLEEP_MAX_MS;
  // Keep dim < sleep — a corrupt or legacy NVS pair could otherwise sleep the
  // screen before it ever dims. DIM_MAX + SLEEP_STEP is always <= SLEEP_MAX,
  // so this push can't exceed the sleep ceiling.
  if (dimTimeoutMs >= sleepTimeoutMs) sleepTimeoutMs = dimTimeoutMs + SLEEP_STEP_MS;
  if (currentThemeIndex < 0 || currentThemeIndex >= NUM_THEMES) currentThemeIndex = 0;
  if (themeIntensity < 0 || themeIntensity > 100) themeIntensity = 50;
}

void saveDisplayPrefs() {
  preferences.begin("display", false);
  preferences.putULong("dimMs",   dimTimeoutMs);
  preferences.putULong("sleepMs", sleepTimeoutMs);
  preferences.putInt  ("theme",   currentThemeIndex);
  preferences.putInt  ("tint",    themeIntensity);
  preferences.end();
  displayPrefsDirty = false;
}

// V12: "vib" NVS namespace — separate from CalibData so adding the vibration
// monitor never forces a calibration reset. Holds the Goertzel lock frequency
// and the last aggregate suggested-wait. Mirrors loadDisplayPrefs/saveDisplayPrefs.
void loadVibPrefs() {
  preferences.begin("vib", true);
  vibResonanceHz   = preferences.getFloat("resHz", VIB_RES_DEFAULT);
  vibSuggestedWait.store(preferences.getUInt("wait", 0));
  vibCurrentWaitMs = preferences.getInt("curWait", 1500);
  vibShutterDenom  = preferences.getUShort("shut", 60);
  preferences.end();
  if (vibResonanceHz < VIB_RES_MIN_HZ || vibResonanceHz > VIB_RES_MAX_HZ)
    vibResonanceHz = VIB_RES_DEFAULT;
  if (vibCurrentWaitMs < 0)     vibCurrentWaitMs = 0;
  if (vibCurrentWaitMs > 20000) vibCurrentWaitMs = 20000;
  if (vibShutterDenom < 1 || vibShutterDenom > 2000) vibShutterDenom = 60;
}

void saveVibPrefs() {
  preferences.begin("vib", false);
  preferences.putFloat (  "resHz",   vibResonanceHz);
  preferences.putUInt  (  "wait",    vibSuggestedWait.load());
  preferences.putInt   (  "curWait", vibCurrentWaitMs);
  preferences.putUShort(  "shut",    vibShutterDenom);
  preferences.end();
  vibPrefsDirty = false;
}

void drawSuccessScreen();
void drawStackCalcUI();
void drawStackTimeUI();
void drawFovInfoUI();        // V17
void drawCalReviewUI();      // V17
void refreshCalSettingsValues(bool force = false);
void refreshPixelValue(int p, bool force = false);
void refreshStackCalcValues(bool force = false);
void refreshStackTimeValues(bool force = false);
void updateObjectiveButtons();
void drawAdjButtons(int y);
void drawSignalHealthBar(uint8_t status, float mcps, int x, int y, bool toSprite = false);
void drawLeftBoxedText(const char* txt, int x, int y, uint16_t bgCol);
void drawCaptureSaveButton(Button& btn, uint16_t boxCol, uint16_t textCol);
void centerStaticText(const char* txt, int y, uint8_t size);
void finalizeCalibration();
void finalizeEarly();
void resetToFactory();
void updateDisplay();
void updateSensorAverages();
void drawGearIcon(int cx, int cy, uint16_t color, uint16_t bgCol);
void wakeScreen();
void registerActivity();
void saveStackSettings();
void saveAllSettings();

void handleMainTouch(TS_Point p);
void handleAppSettingsTouch(TS_Point p);
void handleCalSettingsTouch(TS_Point p, int adj);
void handleCalRunTouch(TS_Point p, int adj);
void handleCalConfirmTouch(TS_Point p);
void handleCalSuccessTouch(TS_Point p);
void handleStackCalcTouch(TS_Point p, int adj);
void handleStackTimeTouch(TS_Point p, int adj);
void handleFovInfoTouch(TS_Point p);       // V17
void drawCalGraphUI();                     // V17b: calibration scatter plot
void handleCalGraphTouch(TS_Point p);      // V17b
void handleCalReviewTouch(TS_Point p);     // V17
void fireTriggerLed(bool on) {
  // LED pulses with the A4 trigger when ledEnabled is true.
  // on=true: trigger active → LED on at currentLedDuty
  // on=false: trigger ended → LED off
  // ledEnabled=false: always off
  if (!ledEnabled || !on) {
    analogWrite(TRIGGER_LED_PIN, 255);             // active-low: off
  } else {
    analogWrite(TRIGGER_LED_PIN, 255 - currentLedDuty); // active-low: on
  }
}


void drawBrightnessSettingsUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("BRIGHTNESS", 5, 5, COLOR_DARKBLUE);
  // Static labels (drawn once)
  setSmoothFont(1);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(15, 72); tft.print("SCREEN");
  // V11: "TRIGGER LED" + LED %% are now drawn inline by refreshBrightness-
  // SettingsValues — see comment there for why the label moved into the
  // dynamic refresh path.
  // V11: LED on/off toggle — when ON, the bright COLOR_PUREGREEN box can
  // bloom against the light DAYLIGHT bg. themedText() pulls it down to a
  // mid-dark green so the boundary stays crisp without losing the green
  // identity. Same for the OFF (maroon) case, although maroon already has
  // good contrast on either bg.
  btnLedToggle.draw(tft, ledEnabled ? "ON" : "OFF",
    ledEnabled ? themedText(COLOR_PUREGREEN) : COLOR_MAROON, themedText(TFT_WHITE));
  refreshBrightnessSettingsValues(true);
  // patched3: IR CONTROL button — opens the MJKZZ IR remote panel.
  btnIrControl.draw(tft);
  // V11: SCREEN TIMEOUTS button — opens the timeouts/theme sub-screen.
  btnScreenTimeouts.draw(tft);
  btnBrightBack.draw(tft);
}

void refreshBrightnessSettingsValues(bool force) {
  static int lastBr = -1, lastLed = -1;
  static bool lastLedEn = true;
  if (force) { lastBr = -1; lastLed = -1; lastLedEn = !ledEnabled; }

  // ── Screen slider (y=60..85, sprite-blitted) ──
  if (force || lastBr != currentBrightness) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextSize(1);
    // Filled bar
    int barW = 210, barH = 22;
    int filled = map(currentBrightness, 1, 255, 0, barW);
    distSprite.fillRoundRect(15, 0, barW, barH, 2, COLOR_DARKGREY);
    if (filled > 4) distSprite.fillRoundRect(15, 0, filled, barH, 2, COLOR_GREENYELLOW);
    distSprite.drawRoundRect(15, 0, barW, barH, 2, COLOR_LIGHTGREY);
    distSprite.fillRect(15 + filled - 2, 0, 4, barH, TFT_WHITE); // thumb
    // Percentage label right of bar
    char buf[8]; snprintf(buf, sizeof(buf), "%d%%", map(currentBrightness, 1, 255, 1, 100));
    distSprite.setTextColor(themedText(COLOR_GREENYELLOW));
    int16_t bx, by; uint16_t bw, bh;
    distSprite.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    distSprite.setCursor((240 - bw) / 2 - bx, 40);
    distSprite.print(buf);
    tft.drawRGBBitmap(0, 80, distSprite.getBuffer(), 240, 45);
    lastBr = currentBrightness;
  }

  // ── LED slider (V11 polish): glitch-free single-pass redraw ──
  // Earlier attempts left two related bugs: (a) %% text fragments persisted
  // on slider drags because the clear strip didn't cover the FULL height of
  // the FreeSans9 glyph (ascenders reach ~13px above the baseline; the strip
  // only covered ~9), and (b) when toggling ON↔OFF, the disabled branch left
  // the previous %% text behind because it never painted over that region.
  // Both fixed below by clearing a tall-enough strip on every refresh and
  // ALWAYS painting the right-side text — either the %% (when ON) or a
  // muted "—" placeholder (when OFF). Strip is x=0..150 so the LED toggle
  // button (x=155..230) is never touched.
  if (force || lastLed != currentLedDuty || lastLedEn != ledEnabled) {
    // LED label + %% value — blitted via valSprite (120px wide) so the draw
    // is a single atomic blit with no fillRect-then-print flicker. valSprite
    // is only 120px wide so the blit ends at x=119, well clear of the toggle
    // button at x=155. Right-align %% to sprite-x=110.
    char buf[8];
    valSprite.fillScreen(THEME_BG);
    valSprite.setFont(&FreeSans9pt7b); valSprite.setTextSize(1);
    valSprite.setTextColor(themedText(COLOR_LIGHTGREY));
    valSprite.setCursor(15, 18); valSprite.print("LED");
    if (ledEnabled) {
      snprintf(buf, sizeof(buf), "%d%%", map(currentLedDuty, 0, 127, 0, 100));
      valSprite.setTextColor(themedText(COLOR_PURPLE));
    } else {
      snprintf(buf, sizeof(buf), "--");
      valSprite.setTextColor(themedText(COLOR_DARKGREY));
    }
    int16_t bx, by; uint16_t bw, bh;
    valSprite.getTextBounds(buf, 0, 0, &bx, &by, &bw, &bh);
    valSprite.setCursor(110 - bw - bx, 18);
    valSprite.print(buf);
    tft.drawRGBBitmap(0, 138, valSprite.getBuffer(), 120, 22);

    // Bar — built in distSprite, blitted in one shot.
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextSize(1);
    if (ledEnabled) {
      int barW = 210, barH = 22;
      int filled = map(currentLedDuty, 0, 127, 0, barW);
      distSprite.fillRoundRect(15, 0, barW, barH, 2, COLOR_DARKGREY);
      if (filled > 4) distSprite.fillRoundRect(15, 0, filled, barH, 2, COLOR_PURPLE);
      distSprite.drawRoundRect(15, 0, barW, barH, 2, COLOR_LIGHTGREY);
      distSprite.fillRect(15 + filled - 2, 0, 4, barH, TFT_WHITE);
    } else {
      // Disabled: muted rounded bar with "DISABLED" text inside.
      distSprite.fillRoundRect(15, 0, 210, 22, 2, 0x1082);
      distSprite.drawRoundRect(15, 0, 210, 22, 2, COLOR_DARKGREY);
      distSprite.setTextColor(themedText(COLOR_DARKGREY));
      const char* dis = "DISABLED";
      int16_t dx, dy; uint16_t dw, dh;
      distSprite.getTextBounds(dis, 0, 0, &dx, &dy, &dw, &dh);
      distSprite.setCursor((240 - dw) / 2 - dx, 16);
      distSprite.print(dis);
    }
    tft.drawRGBBitmap(0, 172, distSprite.getBuffer(), 240, 22);

    // Toggle is only repainted when its STATE actually changes (ledEnabled
    // flipped), not on every duty-cycle tick. This eliminates the toggle
    // re-flash that used to show on every slider movement.
    if (force || lastLedEn != ledEnabled) {
      btnLedToggle.draw(tft, ledEnabled ? "ON" : "OFF",
        ledEnabled ? themedText(COLOR_PUREGREEN) : COLOR_MAROON, themedText(TFT_WHITE));
    }

    lastLed = currentLedDuty; lastLedEn = ledEnabled;
  }
}

void handleBrightnessSettingsTouch(TS_Point p, int adj) {
  // adj is always 0 here (adj buttons removed); called from slider zone or normal touch

  // LED enable toggle
  if (btnLedToggle.contains(p.x, p.y)) {
    ledEnabled = !ledEnabled;
    // When enabled, light LED at current duty so user sees immediate feedback
    // When disabled, ensure LED is off
    if (ledEnabled) {
      analogWrite(TRIGGER_LED_PIN, 255 - currentLedDuty);  // active-low: on
    } else {
      analogWrite(TRIGGER_LED_PIN, 255);                    // active-low: off
    }
    refreshBrightnessSettingsValues(false);
    return;
  }

  // patched3: IR CONTROL button — opens the MJKZZ IR remote panel. Persists
  // the brightness/LED settings on the way out, same as the SCREEN TIMEOUTS path.
  if (btnIrControl.contains(p.x, p.y)) {
    settings.brightness = currentBrightness;
    settings.ledEnabled = ledEnabled ? 1 : 0;
    settings.ledDuty = (uint8_t)currentLedDuty;
    preferences.begin("calib", false);
    preferences.putBytes("settings", &settings, sizeof(CalibData));
    preferences.end();
    currentMode = IR_CONTROL;
    drawIrControlUI();
    return;
  }

  // V11: SCREEN TIMEOUTS button — opens the timeouts/theme sub-screen.
  // Hit-tested early so the slider drag zone (which is forgiving) doesn't
  // accidentally swallow taps on the button.
  if (btnScreenTimeouts.contains(p.x, p.y)) {
    settings.brightness = currentBrightness;
    settings.ledEnabled = ledEnabled ? 1 : 0;
    settings.ledDuty = (uint8_t)currentLedDuty;
    preferences.begin("calib", false);
    preferences.putBytes("settings", &settings, sizeof(CalibData));
    preferences.end();
    currentMode = SCREEN_TIMEOUT;
    drawScreenTimeoutUI();
    return;
  }

  // Screen slider drag: y=60..105
  if (p.y >= 80 && p.y <= 125) {
    int valX = constrain(p.x, 15, 225);
    currentBrightness = max(1, (int)map(valX, 15, 225, 1, 255));
    analogWrite(LITE_PIN, currentBrightness);
    refreshBrightnessSettingsValues(false);
    return;
  }

  // LED slider drag — y=166..196 (patched3: LED group moved up 12 px to open
  // breathing room between the bar at y=172..194 and the IR CONTROL button).
  if (ledEnabled && p.y >= 166 && p.y <= 196) {
    int valX = constrain(p.x, 15, 225);
    currentLedDuty = (int)map(valX, 15, 225, 0, 127); // 127 = ~50% duty = max safe
    fireTriggerLed(ledEnabled);  // apply to LED immediately
    refreshBrightnessSettingsValues(false);
    return;
  }

  if (btnBrightBack.contains(p.x, p.y)) {
    settings.brightness = currentBrightness;
    settings.ledEnabled = ledEnabled ? 1 : 0;
    settings.ledDuty = (uint8_t)currentLedDuty;
    preferences.begin("calib", false);
    preferences.putBytes("settings", &settings, sizeof(CalibData));
    preferences.end();
    currentMode = APP_SETTINGS; drawAppSettingsUI();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// IR_CONTROL screen (patched3) — sends the MJKZZ stacking-controller's NEC
// remote codes over the IR LED. Reached from the BRIGHTNESS screen.
// ─────────────────────────────────────────────────────────────────────────────

// Send the NEC code for IR button `idx` (0-4) and give that button a brief
// green flash — IR is invisible, so the flash is the only press feedback.
static void irFlashSend(int idx) {
  irButtons[idx]->draw(tft, nullptr, COLOR_GREENYELLOW, TFT_BLACK);
  irLed.sendNEC(irCodes[idx]);
  if      (idx == 0) mjkzzStackActive = true;    // START → web buttons sync via telemetry
  else if (idx == 1) mjkzzStackActive = false;   // STOP
  delay(120);
  irButtons[idx]->draw(tft);
}

void drawIrControlUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("IR REMOTE", 5, 5, COLOR_PURPLE);
  for (int i = 0; i < 6; i++) irButtons[i]->draw(tft);
}

void handleIrControlTouch(TS_Point p) {
  for (int i = 0; i < 5; i++) {
    if (irButtons[i]->contains(p.x, p.y)) { irFlashSend(i); return; }
  }
  if (btnIrBack.contains(p.x, p.y)) {
    currentMode = BRIGHTNESS_SETTINGS;
    drawBrightnessSettingsUI();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V12: VIBRATION monitor — hub (VIB_HOME) + live spectrum (VIB_SPECTRUM).
// A passive bench-vibration monitor: it measures and advises only, and never
// gates or delays camera capture. vibTask (Core 0) owns the DSP; these UI
// functions run on Core 1 and only read the published spectrum / scalars.
// ─────────────────────────────────────────────────────────────────────────────

// Magnitude fraction (0..1) → heat-ramp RGB565: black→blue→cyan→green→yellow→red.
static uint16_t vibHeatColor(float frac) {
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  uint8_t r, g, b;
  if      (frac < 0.25f) { float t = frac / 0.25f;          r = 0;                  g = 0;                  b = (uint8_t)(t * 255); }
  else if (frac < 0.50f) { float t = (frac - 0.25f) / 0.25f;r = 0;                  g = (uint8_t)(t * 255); b = 255; }
  else if (frac < 0.75f) { float t = (frac - 0.50f) / 0.25f;r = (uint8_t)(t * 255); g = 255;                b = (uint8_t)((1 - t) * 255); }
  else                   { float t = (frac - 0.75f) / 0.25f;r = 255;                g = (uint8_t)((1 - t) * 255); b = 0; }
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

void drawVibHomeUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("VIBRATION", 5, 5, COLOR_TEAL);
  btnVibHomeClose.draw(tft);

  btnVibSpectrum.draw(tft);
  btnVibSettle.draw(tft);
  btnVibSignatures.draw(tft);

  setSmoothFont(1);

  if (imuPresent) {
    tft.setTextColor(themedText(COLOR_PUREGREEN));
    centerStaticText("IMU ready", 300, 1);
  } else {
    tft.setTextColor(COLOR_RED);
    centerStaticText("IMU not detected", 300, 1);
  }
}

void handleVibHomeTouch(TS_Point p) {
  if (btnVibHomeClose.contains(p.x, p.y)) {
    currentMode = APP_SETTINGS; drawAppSettingsUI(); return;
  }
  if (btnVibSpectrum.contains(p.x, p.y)) {
    currentMode = VIB_SPECTRUM; drawVibSpectrumUI(); return;
  }
  if (btnVibSettle.contains(p.x, p.y)) {
    currentMode = VIB_SETTLE; drawVibSettleUI(); return;
  }
  if (btnVibSignatures.contains(p.x, p.y)) {
    currentMode = VIB_SIGNATURES; drawVibSignaturesUI(); return;
  }
}

void drawVibSpectrumUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("VIB SPECTRUM", 5, 5, COLOR_TEAL);
  btnVibSpecClose.draw(tft);
  // Waterfall border — the image itself is blitted by the refresh.
  tft.drawRect(VIB_WF_X - 1, VIB_WF_Y - 1, VIB_WF_W + 2, VIB_WF_H + 2,
               themedText(COLOR_DARKGREY));
  refreshVibSpectrumValues(true);
}

// Reads the lock-free spectrum ping-pong; redraws the readout row, the spectrum
// plot (off-screen sprite → one blit) and scrolls the waterfall.
void refreshVibSpectrumValues(bool force) {
  static uint32_t lastSeq    = 0xFFFFFFFF;
  static float    yMaxSmooth = 100.0f;   // deci-mg, smoothed plot ceiling

  if (!imuPresent || !vibReady) {
    if (force) {
      distSprite.fillScreen(THEME_BG);
      distSprite.setFont(&FreeSans9pt7b); distSprite.setTextSize(1);
      distSprite.setTextColor(COLOR_RED);
      distSprite.setCursor(12, 28);
      distSprite.print(imuPresent ? "Vibration buffers failed" : "IMU not detected");
      tft.drawRGBBitmap(0, 34, distSprite.getBuffer(), 240, 45);
    }
    return;
  }

  // Snapshot the spectrum — lock-free SPSC: read seq, copy, re-check seq.
  uint16_t spec[VIB_FFT_BINS], specH[VIB_FFT_BINS];
  uint32_t seq = 0;
  for (int tries = 0; tries < 3; tries++) {
    seq = vibSpecSeq.load();
    memcpy(spec,  vibSpec[seq & 1],  sizeof(spec));
    memcpy(specH, vibSpecH[seq & 1], sizeof(specH));
    if (vibSpecSeq.load() == seq) break;
  }
  bool isNewFrame = (seq != lastSeq);
  lastSeq = seq;

  // ── Per-bin temporal smoothing for the plotted live spectrum ──
  // Each FFT frame carries its own noise realisation; a single-frame plot
  // strobes bin-to-bin in a way the eye reads as jitter. A 2-frame EMA
  // (smooth = (old + live + 1) / 2, i.e. α = 0.5) softens that without
  // adding perceptible lag — transients still appear within one frame.
  // Only the spectrum plot + magenta H-polyline read these; the waterfall
  // keeps using the raw snapshot so it still captures per-frame detail.
  static uint16_t smoothV[VIB_FFT_BINS] = {0};
  static uint16_t smoothH[VIB_FFT_BINS] = {0};
  static bool     smoothInit = false;
  if (isNewFrame) {
    if (!smoothInit) {
      memcpy(smoothV, spec,  sizeof(smoothV));
      memcpy(smoothH, specH, sizeof(smoothH));
      smoothInit = true;
    } else {
      for (int b = 0; b < VIB_FFT_BINS; b++) {
        smoothV[b] = (uint16_t)(((uint32_t)smoothV[b] + spec[b]  + 1) >> 1);
        // α = 0.25 (4-frame effective) for H — reduces the dense multi-peak
        // pattern that appears at moderate/strong vibration levels.
        smoothH[b] = (uint16_t)((3u*(uint32_t)smoothH[b] + specH[b] + 2u) >> 2);
      }
    }
  }

  // ── Readout (always): line 1 = dominant Hz + state, line 2 = V/H RMS ──
  float    domHz   = vibDominant.load() / 10.0f;
  float    vertMg  = vibBandRms.load()  / 100.0f;
  float    horizMg = vibHorizRms.load() / 100.0f;
  uint32_t st      = vibState.load();
  const char* stTxt = (st == 2) ? "STRONG" : (st == 1) ? "MODERATE" : "CALM";
  uint16_t    stCol = (st == 2) ? COLOR_RED : (st == 1) ? COLOR_ORANGE : COLOR_PUREGREEN;

  distSprite.fillScreen(THEME_BG);
  distSprite.setFont(&FreeSans9pt7b); distSprite.setTextSize(1);
  char buf[40];

  // Line 1 — dominant frequency (left) + state, right-aligned so the longest
  // label ("MODERATE") can't run past the 240 px sprite edge.
  distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  snprintf(buf, sizeof(buf), "%.1f Hz", domHz);
  distSprite.setCursor(8, 18); distSprite.print(buf);
  distSprite.setTextColor(stCol);
  int16_t stx1, sty1; uint16_t stw, sth;
  distSprite.getTextBounds(stTxt, 0, 0, &stx1, &sty1, &stw, &sth);
  distSprite.setCursor(185 - (int)stw - stx1, 18); distSprite.print(stTxt);

  // Line 2 — vertical vs horizontal RMS. Horizontal turns orange when it
  // exceeds vertical: that is the "horizontal mess" the old magnitude DSP hid.
  distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  snprintf(buf, sizeof(buf), "V %.1f mg", vertMg);
  distSprite.setCursor(8, 40); distSprite.print(buf);
  distSprite.setTextColor((horizMg > vertMg) ? COLOR_ORANGE : themedText(COLOR_LIGHTGREY));
  snprintf(buf, sizeof(buf), "H %.1f mg", horizMg);
  distSprite.setCursor(124, 40); distSprite.print(buf);
  tft.drawRGBBitmap(0, 34, distSprite.getBuffer(), 240, 45);

  if (!isNewFrame && !force) return;   // nothing new — skip the heavy redraw

  // ── Y auto-scale (rise fast, decay slow; floored so a quiet bench is calm) ──
  uint16_t curMax = 1;
  for (int b = 2; b < VIB_FFT_BINS; b++) {
    if (spec[b]  > curMax) curMax = spec[b];
    if (specH[b] > curMax) curMax = specH[b];
  }
  float target = (curMax < 30) ? 30.0f : (float)curMax;
  if (target > yMaxSmooth) yMaxSmooth = target;
  else                     yMaxSmooth += (target - yMaxSmooth) * 0.04f;
  float yMax = yMaxSmooth;

  // ── Spectrum plot — drawn into vibPlotSprite, blitted in one shot ──
  const int pL = 2, pR = 230, pT = 2, pB = 86;
  const int plotW = pR - pL, plotH = pB - pT;
  vibPlotSprite.fillScreen(THEME_BG);
  vibPlotSprite.drawFastVLine(pL, pT, plotH, themedText(COLOR_DARKGREY));
  vibPlotSprite.drawFastHLine(pL, pB, plotW, themedText(COLOR_DARKGREY));
  for (int px = 0; px < plotW; px++) {
    int b0 = px * VIB_FFT_BINS / plotW;
    int b1 = (px + 1) * VIB_FFT_BINS / plotW;
    if (b1 <= b0) b1 = b0 + 1;
    uint16_t mx = 0;
    for (int b = b0; b < b1 && b < VIB_FFT_BINS; b++)
      if (b >= 2 && smoothV[b] > mx) mx = smoothV[b];
    int h = (int)((float)mx / yMax * plotH);
    if (h > plotH) h = plotH;
    if (h > 0)
      vibPlotSprite.drawFastVLine(pL + px, pB - h, h, vibHeatColor((float)h / plotH));
  }
  // Frequency ticks + labels (built-in 6x8 font).
  vibPlotSprite.setFont(); vibPlotSprite.setTextSize(1);
  vibPlotSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  for (int hz = 50; hz <= 200; hz += 50) {
    int bin = (int)(hz / VIB_BIN_HZ);
    int px  = pL + bin * plotW / VIB_FFT_BINS;
    if (px > pR) continue;
    vibPlotSprite.drawFastVLine(px, pB, 3, themedText(COLOR_DARKGREY));
    char lbl[6]; snprintf(lbl, sizeof(lbl), "%d", hz);
    int lw = (int)strlen(lbl) * 6;
    int lx = px - lw / 2;
    if (lx < pL) lx = pL;
    if (lx > pR - lw) lx = pR - lw;
    vibPlotSprite.setCursor(lx, pB + 5); vibPlotSprite.print(lbl);
  }
  vibPlotSprite.setCursor(pR - 12, pB + 5); vibPlotSprite.print("Hz");
  // Dominant-frequency marker — a small triangle at the plot top.
  if (vibDominant.load() > 0) {
    int bin = (int)(domHz / VIB_BIN_HZ);
    int px  = pL + bin * plotW / VIB_FFT_BINS;
    if (px >= pL && px <= pR)
      vibPlotSprite.fillTriangle(px - 3, pT, px + 3, pT, px, pT + 6, COLOR_YELLOW);
  }
  // Horizontal-plane spectrum overlaid as a magenta polyline — the heat bars
  // are the vertical channel, this line is the combined horizontal channel.
  {
    int prevX = -1, prevY = 0;
    for (int px = 0; px < plotW; px++) {
      int b0 = px * VIB_FFT_BINS / plotW;
      int b1 = (px + 1) * VIB_FFT_BINS / plotW;
      if (b1 <= b0) b1 = b0 + 1;
      uint16_t mx = 0;
      for (int b = b0; b < b1 && b < VIB_FFT_BINS; b++)
        if (b >= 2 && smoothH[b] > mx) mx = smoothH[b];
      int hh = (int)((float)mx / yMax * plotH);
      if (hh > plotH) hh = plotH;
      int y = pB - hh;
      if (prevX >= 0) vibPlotSprite.drawLine(prevX, prevY, pL + px, y, 0xF81F);
      prevX = pL + px; prevY = y;
    }
    vibPlotSprite.setTextColor(0xF81F);
    vibPlotSprite.setCursor(pR - 44, pT + 1); vibPlotSprite.print("H");
  }
  tft.drawRGBBitmap(VIB_PLOT_X, VIB_PLOT_Y, vibPlotSprite.getBuffer(), 232, 104);

  // ── Waterfall — scroll down one row on a new frame, then blit ──
  // Color is mapped through sqrt(frac) so moderate-energy rows still register
  // visibly when yMax has been pushed up by a brief strong peak. Without the
  // gamma the slow-decay yMax keeps the waterfall colors saturated near the
  // floor for seconds after every spike, and moderate vibrations read as black.
  if (isNewFrame && vibWF) {
    memmove(vibWF + VIB_WF_W, vibWF,
            (size_t)(VIB_WF_H - 1) * VIB_WF_W * sizeof(uint16_t));
    for (int px = 0; px < VIB_WF_W; px++) {
      int b0 = px * VIB_FFT_BINS / VIB_WF_W;
      int b1 = (px + 1) * VIB_FFT_BINS / VIB_WF_W;
      if (b1 <= b0) b1 = b0 + 1;
      uint16_t mx = 0;
      for (int b = b0; b < b1 && b < VIB_FFT_BINS; b++)
        if (b >= 2 && spec[b] > mx) mx = spec[b];
      float frac = (float)mx / yMax;
      if (frac < 0.0f) frac = 0.0f;
      vibWF[px] = vibHeatColor(sqrtf(frac));
    }
  }
  if (vibWF)
    tft.drawRGBBitmap(VIB_WF_X, VIB_WF_Y, vibWF, VIB_WF_W, VIB_WF_H);
}

void handleVibSpectrumTouch(TS_Point p) {
  if (btnVibSpecClose.contains(p.x, p.y)) {
    currentMode = VIB_HOME; drawVibHomeUI();
  }
}

// ── VIB_SETTLE — motor-step settle-time analysis ────────────────────────────
static bool vibSettleWaiting = false;   // a RUN TEST capture is in flight

void drawVibSettleUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("VIB SETTLE", 5, 5, COLOR_TEAL);
  btnVibSettleClose.draw(tft);
  btnVibRunTest.draw(tft);
  btnVibReLock.draw(tft);
  btnVibWaitMinus.draw(tft);
  btnVibWaitPlus.draw(tft);
  refreshVibSettleValues(true);
}

// Recommended WAIT — the post-stack 95th-percentile aggregate, or a provisional
// value derived from the last single RUN TEST until a stack has run. Returns 0
// when there is no data yet. `provisional` is set true for the single-test case.
static uint32_t vibRecommendedWait(bool* provisional) {
  uint32_t rec = vibSuggestedWait.load();
  if (provisional) *provisional = false;
  if (rec == 0) {
    uint32_t s = vibSettleMs.load();
    if (s > 0) { rec = (uint32_t)(s * 1.15f); if (provisional) *provisional = true; }
  }
  return rec;
}

void refreshVibSettleValues(bool force) {
  static uint32_t lastSeq = 0xFFFFFFFF;
  if (!imuPresent || !vibReady) {
    if (force) {
      setSmoothFont(1);
      tft.setTextColor(COLOR_RED);
      centerStaticText("IMU not detected", 150, 1);
    }
    return;
  }
  uint32_t seq   = vibSettleSeq.load();
  bool     isNew = (seq != lastSeq);
  if (isNew) { lastSeq = seq; vibSettleWaiting = false; }
  if (!isNew && !force) return;

  bool     provisional = false;
  uint32_t recWait = vibRecommendedWait(&provisional);

  // ── Key:value rows + verdict ──
  // Each row/verdict is composited into distSprite and blitted in one shot —
  // a refresh lands on a new settle analysis (seconds apart), so a bare
  // fillRect-then-print would blink the row band each time. The four 24-px
  // strips tile the band y=38..134; the decay plot below starts at y=142.
  char buf[44];
  auto blitRow = [&](int screenY, const char* label, const char* val, uint16_t vc) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b); distSprite.setTextSize(1);
    distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
    distSprite.setCursor(14, 14); distSprite.print(label);
    int16_t x1, y1; uint16_t w, h;
    distSprite.getTextBounds(val, 0, 0, &x1, &y1, &w, &h);
    distSprite.setTextColor(vc);
    distSprite.setCursor(226 - w - x1, 14); distSprite.print(val);
    tft.drawRGBBitmap(0, screenY, distSprite.getBuffer(), 240, 24);
  };
  snprintf(buf, sizeof(buf), "%.1f Hz", vibResonanceHz);
  blitRow(38, "Resonance lock:", buf, themedText(TFT_WHITE));
  if (recWait > 0) {
    snprintf(buf, sizeof(buf), "%lu ms%s", (unsigned long)recWait,
             provisional ? " (1 test)" : "");
    blitRow(62, "Recommended WAIT:", buf, COLOR_GREENYELLOW);
  } else {
    blitRow(62, "Recommended WAIT:", "--", themedText(COLOR_DARKGREY));
  }
  snprintf(buf, sizeof(buf), "%d ms", vibCurrentWaitMs);
  blitRow(86, "Current WAIT:", buf, COLOR_TEAL);

  // ── Verdict line — also sprite-blitted (strip y=110..134, baseline 124) ──
  const char* verdict;
  uint16_t    verdictCol;
  if (vibSettleWaiting) {
    verdict = "analyzing...";                       verdictCol = COLOR_YELLOW;
  } else if (recWait == 0) {
    verdict = "Run a stack or RUN TEST for advice"; verdictCol = themedText(COLOR_DARKGREY);
  } else {
    int delta = vibCurrentWaitMs - (int)recWait;
    if (delta > 150) {
      snprintf(buf, sizeof(buf), "WAIT can drop ~%d ms", delta);
      verdictCol = COLOR_PUREGREEN;
    } else if (delta < -150) {
      snprintf(buf, sizeof(buf), "WAIT too short by ~%d ms", -delta);
      verdictCol = COLOR_RED;
    } else {
      snprintf(buf, sizeof(buf), "WAIT is about right");
      verdictCol = COLOR_PUREGREEN;
    }
    verdict = buf;
  }
  {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b); distSprite.setTextSize(1);
    distSprite.setTextColor(verdictCol);
    int16_t x1, y1; uint16_t w, h;
    distSprite.getTextBounds(verdict, 0, 0, &x1, &y1, &w, &h);
    distSprite.setCursor((240 - (int)w) / 2 - x1, 14); distSprite.print(verdict);
    tft.drawRGBBitmap(0, 110, distSprite.getBuffer(), 240, 24);
  }

  // ── Decay-envelope plot (vibPlotSprite → one blit) ──
  const int pL = 4, pR = 228, pT = 4, pB = 86;
  const int plotW = pR - pL, plotH = pB - pT;
  vibPlotSprite.fillScreen(THEME_BG);
  vibPlotSprite.drawFastVLine(pL, pT, plotH, themedText(COLOR_DARKGREY));
  vibPlotSprite.drawFastHLine(pL, pB, plotW, themedText(COLOR_DARKGREY));
  float emax = (vibEnvPeak > 1.0f) ? vibEnvPeak : 1.0f;
  int prevx = -1, prevy = 0;
  for (int b = 0; b < VIB_SETTLE_BLOCKS; b++) {
    int x = pL + b * plotW / (VIB_SETTLE_BLOCKS - 1);
    int y = pB - (int)(vibEnvPlot[b] / emax * plotH);
    if (y < pT) y = pT;
    if (y > pB) y = pB;
    if (prevx >= 0)
      vibPlotSprite.drawLine(prevx, prevy, x, y, themedText(COLOR_BLUEGREEN));
    vibPlotSprite.fillCircle(x, y, 2, COLOR_GREENYELLOW);
    prevx = x; prevy = y;
  }
  // X axis: window start (oldest) on the left, the shutter pulse on the right.
  vibPlotSprite.setFont(); vibPlotSprite.setTextSize(1);
  vibPlotSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  char tb[16];
  snprintf(tb, sizeof(tb), "-%d ms",
           (int)(VIB_SETTLE_BLOCKS * VIB_BLOCK * 1000.0f / VIB_SAMPLE_RATE));
  vibPlotSprite.setCursor(pL, pB + 5); vibPlotSprite.print(tb);
  vibPlotSprite.setCursor(pR - 4 * 6, pB + 5); vibPlotSprite.print("SNAP");
  tft.drawRGBBitmap(4, 142, vibPlotSprite.getBuffer(), 232, 104);
}

void handleVibSettleTouch(TS_Point p) {
  if (btnVibSettleClose.contains(p.x, p.y)) {
    currentMode = VIB_HOME; drawVibHomeUI(); return;
  }
  if (btnVibRunTest.contains(p.x, p.y)) {
    // Synthesise a pulse — vibTask analyses the preceding ~2.5 s immediately,
    // so the result appears on the next refresh tick. Disturb the rig (or run a
    // motor step) within ~2.5 s before tapping.
    vibPulseRingPos = vibRawHead;
    // Release on the seq bump pairs with acquire in vibTask so the
    // ringPos write above is visible there before settle analysis runs.
    vibPulseSeq.fetch_add(1, std::memory_order_release);
    vibSettleWaiting = true;
    refreshVibSettleValues(true);
    return;
  }
  if (btnVibReLock.contains(p.x, p.y)) {
    float d = vibDominant.load() / 10.0f;
    if (d < VIB_RES_MIN_HZ) d = VIB_RES_MIN_HZ;
    if (d > VIB_RES_MAX_HZ) d = VIB_RES_MAX_HZ;
    vibResonanceHz = d;
    vibPrefsDirty  = true;
    lastVibEditMs  = millis();
    refreshVibSettleValues(true);
    return;
  }
  // Enter the controller's WAIT setting so the verdict can advise a delta.
  if (btnVibWaitMinus.contains(p.x, p.y)) {
    vibCurrentWaitMs -= 100;
    if (vibCurrentWaitMs < 0) vibCurrentWaitMs = 0;
    vibPrefsDirty = true; lastVibEditMs = millis();
    refreshVibSettleValues(true);
    return;
  }
  if (btnVibWaitPlus.contains(p.x, p.y)) {
    vibCurrentWaitMs += 100;
    if (vibCurrentWaitMs > 20000) vibCurrentWaitMs = 20000;
    vibPrefsDirty = true; lastVibEditMs = millis();
    refreshVibSettleValues(true);
    return;
  }
}

// ── VIB_SIGNATURES — reference-spectrum library (LittleFS /vibsig) ───────────

// Scan /vibsig for saved .bin signatures into vibSigList.
static void vibSigScan() {
  vibSigCount = 0;
  File dir = LittleFS.open("/vibsig");
  if (dir && dir.isDirectory()) {
    for (;;) {
      File e = dir.openNextFile();
      if (!e) break;
      if (!e.isDirectory() && vibSigCount < VIB_SIG_MAX) {
        const char* fn = e.name();
        const char* slash = strrchr(fn, '/');
        const char* base = slash ? slash + 1 : fn;
        strncpy(vibSigList[vibSigCount], base, 23);
        vibSigList[vibSigCount][23] = 0;
        vibSigCount++;
      }
      e.close();
    }
  }
  if (dir) dir.close();
  if (vibSigSel >= vibSigCount) { vibSigSel = -1; vibSigLoadedValid = false; }
}

static bool vibSigLoad(const char* base) {
  char path[44];
  snprintf(path, sizeof(path), "/vibsig/%s", base);
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = (f.read((uint8_t*)&vibSigLoaded, sizeof(VibSignature)) == (int)sizeof(VibSignature));
  f.close();
  return ok;
}

// Finalise an in-flight CAPTURE: copy the peak-hold accumulator out, write
// the .bin file. The on-disk layout is unchanged (uint16 per bin); only the
// semantics flipped from "mean magnitude over N frames" to "peak magnitude
// observed during the capture window". Cosine similarity is scale-invariant
// so existing live-match math still works; a re-capture is recommended for
// pre-12.1.2 signatures so library entries are comparable to each other.
static void vibSigFinishCapture() {
  VibSignature sig;
  memset(&sig, 0, sizeof(sig));
  strncpy(sig.name, vibSigCaptureName, sizeof(sig.name) - 1);
  sig.odrHz      = (uint16_t)VIB_SAMPLE_RATE;
  sig.frameCount = (uint16_t)vibSigFrames;
  sig.epoch      = millis();
  for (int b = 0; b < VIB_FFT_BINS; b++)
    sig.mag[b] = (uint16_t)vibSigAccum[b];
  LittleFS.mkdir("/vibsig");
  char path[48], tmp[52];
  snprintf(path, sizeof(path), "/vibsig/%s.bin", vibSigCaptureName);
  snprintf(tmp,  sizeof(tmp),  "%s.tmp", path);
  // Atomic write: fill a .tmp sibling, then rename over the target. A
  // partial write or power loss leaves the previous signature intact.
  bool ok = false;
  File f = LittleFS.open(tmp, "w");
  if (f) {
    size_t wrote = f.write((const uint8_t*)&sig, sizeof(sig));
    f.close();
    ok = (wrote == sizeof(sig));
  }
  if (ok) ok = LittleFS.rename(tmp, path);
  if (!ok) {
    LittleFS.remove(tmp);
    Serial.printf("[vib] save failed: %s\n", path);
  }
  vibSigCapturing = false;
  vibSigScan();
  vibSigUiDirty = true;
}

// Drives an armed CAPTURE to completion from loop(), independent of the active
// screen, so a web-initiated capture works wherever the device happens to be.
void vibCapturePoll() {
  if (!vibSigCapturing || !vibReady) return;
  uint32_t seq = vibSpecSeq.load();
  if (seq == vibCapSeqSeen) return;          // wait for a fresh FFT frame
  uint16_t spec[VIB_FFT_BINS]; uint32_t s = seq;
  for (int t = 0; t < 3; t++) {
    s = vibSpecSeq.load();
    memcpy(spec, vibSpec[s & 1], sizeof(spec));
    if (vibSpecSeq.load() == s) break;
  }
  vibCapSeqSeen = s;
  // Peak-hold above the noise-floor snapshot — vibSigAccum[b] retains the
  // strongest *excursion above ambient* seen at bin b during the capture
  // window. Mean-averaging (the old behaviour) smeared slowly-drifting tones
  // into a broad hump because the same tone wandering across ±1 bin over
  // 10 s shows up at half-amplitude in three bins; peak-hold keeps full
  // amplitude in each bin the tone visited. Subtracting the floor snapshot
  // additionally strips the constant ambient hum that was inflating every
  // bin's baseline, so the saved signature reads as a sparse "fingerprint"
  // of just the source's tones rather than tones-plus-room.
  for (int b = 0; b < VIB_FFT_BINS; b++) {
    float adj = (float)spec[b] - vibSigBaseSnap[b];
    if (adj < 0) adj = 0;
    uint32_t a = (uint32_t)lroundf(adj);
    if (a > vibSigAccum[b]) vibSigAccum[b] = a;
  }
  vibSigFrames++;
  if (vibSigFrames >= VIB_SIG_FRAMES) {
    vibSigFinishCapture();
    if (currentMode == VIB_SIGNATURES) drawVibSignaturesUI();
    wifiPushVibSigList();
  }
}

// Cosine similarity (bins 2..255) of the loaded signature vs a live spectrum.
static float vibSigCosine(const uint16_t* live) {
  double dot = 0, na = 0, nb = 0;
  for (int b = 2; b < VIB_FFT_BINS; b++) {
    double a = vibSigLoaded.mag[b], c = live[b];
    dot += a * c; na += a * a; nb += c * c;
  }
  if (na <= 0 || nb <= 0) return 0.0f;
  return (float)(dot / (sqrt(na) * sqrt(nb)));
}

void drawVibSignaturesUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("SIGNATURES", 5, 5, COLOR_TEAL);
  btnVibSigClose.draw(tft);

  if (vibSigPicker) {
    setSmoothFont(1);
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    centerStaticText("Tap a source to capture:", 50, 1);
    for (int i = 0; i < 8; i++) {
      int bx = 18 + (i & 1) * 108, by = 72 + (i >> 1) * 48;
      tft.fillRoundRect(bx, by, 100, 38, UI_RADIUS, COLOR_DARKBLUE);
      setSmoothFont(1); tft.setTextColor(TFT_WHITE);
      int16_t x1, y1; uint16_t w, h;
      tft.getTextBounds(VIB_SIG_NAMES[i], 0, 0, &x1, &y1, &w, &h);
      tft.setCursor(bx + (100 - w) / 2 - x1, by + (38 - h) / 2 - y1);
      tft.print(VIB_SIG_NAMES[i]);
    }
    tft.setTextColor(themedText(COLOR_DARKGREY));
    centerStaticText("tap X to cancel", 282, 1);
    return;
  }

  vibSigScan();
  vibSigUiDirty = true;
  btnVibSigCapture.draw(tft);
  btnVibSigDelete.draw(tft);
  refreshVibSignaturesValues(true);
}

void refreshVibSignaturesValues(bool force) {
  if (vibSigPicker) return;
  if (!imuPresent || !vibReady) {
    if (force) {
      setSmoothFont(1); tft.setTextColor(COLOR_RED);
      centerStaticText("IMU not detected", 160, 1);
    }
    return;
  }

  // Snapshot the live spectrum (lock-free SPSC).
  uint16_t spec[VIB_FFT_BINS]; uint32_t seq = 0;
  for (int t = 0; t < 3; t++) {
    seq = vibSpecSeq.load();
    memcpy(spec, vibSpec[seq & 1], sizeof(spec));
    if (vibSpecSeq.load() == seq) break;
  }
  bool newFrame = (seq != vibSigSpecSeqSeen);
  vibSigSpecSeqSeen = seq;
  // CAPTURE frame accumulation runs in vibCapturePoll() from loop(), so a
  // web-initiated capture completes even when this screen isn't open.

  // ── Signature list — each row built in distSprite and blitted in one shot.
  //    The 240-wide strips also paint over the scroll-button column, so the
  //    UP/DN buttons are redrawn on top once the rows are down.
  if (force || vibSigUiDirty) {
    vibSigUiDirty = false;
    for (int r = 0; r < VIB_SIG_VIS_ROWS; r++) {
      int idx  = vibSigScroll + r;
      int rowY = VIB_SIG_LIST_TOP + r * VIB_SIG_ROW_H;
      bool sel = (idx == vibSigSel);
      distSprite.fillScreen(THEME_BG);
      if (sel) distSprite.fillRect(8, 0, 182, VIB_SIG_ROW_H - 2, COLOR_DARKBLUE);
      distSprite.setFont(&FreeSans9pt7b); distSprite.setTextSize(1);
      if (vibSigCount == 0 && r == 0) {
        distSprite.setTextColor(themedText(COLOR_DARKGREY));
        distSprite.setCursor(14, 18); distSprite.print("no signatures saved");
      } else if (idx < vibSigCount) {
        char nm[24]; strncpy(nm, vibSigList[idx], sizeof(nm) - 1); nm[sizeof(nm) - 1] = 0;
        char* dot = strstr(nm, ".bin"); if (dot) *dot = 0;
        distSprite.setTextColor(sel ? COLOR_GREENYELLOW : themedText(TFT_WHITE));
        distSprite.setCursor(14, 18); distSprite.print(nm);
      }
      tft.drawRGBBitmap(0, rowY, distSprite.getBuffer(), 240, VIB_SIG_ROW_H);
    }
    bool canUp = vibSigScroll > 0;
    bool canDn = vibSigScroll + VIB_SIG_VIS_ROWS < vibSigCount;
    btnVibSigUp.draw(tft, nullptr, canUp ? COLOR_BLUEGREEN : COLOR_DARKGREY,
                     canUp ? TFT_WHITE : COLOR_LIGHTGREY);
    btnVibSigDown.draw(tft, nullptr, canDn ? COLOR_BLUEGREEN : COLOR_DARKGREY,
                       canDn ? TFT_WHITE : COLOR_LIGHTGREY);
  }

  // ── Comparison plot — live spectrum + selected signature overlay ──
  if (newFrame || force) {
    const int pL = 4, pR = 228, pT = 4, pB = 80;
    const int plotW = pR - pL, plotH = pB - pT;
    vibPlotSprite.fillScreen(THEME_BG);
    vibPlotSprite.drawFastVLine(pL, pT, plotH, themedText(COLOR_DARKGREY));
    vibPlotSprite.drawFastHLine(pL, pB, plotW, themedText(COLOR_DARKGREY));
    uint16_t mx = 30;
    for (int b = 2; b < VIB_FFT_BINS; b++) {
      if (spec[b] > mx) mx = spec[b];
      if (vibSigLoadedValid && vibSigLoaded.mag[b] > mx) mx = vibSigLoaded.mag[b];
    }
    for (int px = 0; px < plotW; px++) {
      int b0 = px * VIB_FFT_BINS / plotW, b1 = (px + 1) * VIB_FFT_BINS / plotW;
      if (b1 <= b0) b1 = b0 + 1;
      uint16_t v = 0;
      for (int b = b0; b < b1 && b < VIB_FFT_BINS; b++)
        if (b >= 2 && spec[b] > v) v = spec[b];
      int h = (int)((float)v / mx * plotH);
      if (h > 0)
        vibPlotSprite.drawFastVLine(pL + px, pB - h, h, vibHeatColor((float)h / plotH));
    }
    if (vibSigLoadedValid) {
      int prevx = -1, prevy = 0;
      for (int px = 0; px < plotW; px++) {
        int b = px * VIB_FFT_BINS / plotW;
        int h = (int)((float)vibSigLoaded.mag[b] / mx * plotH);
        int y = pB - h;
        if (y < pT) y = pT;
        if (prevx >= 0) vibPlotSprite.drawLine(prevx, prevy, pL + px, y, COLOR_YELLOW);
        prevx = pL + px; prevy = y;
      }
    }
    vibPlotSprite.setFont(); vibPlotSprite.setTextSize(1);
    char st[44];
    if (vibSigCapturing) {
      snprintf(st, sizeof(st), "Capturing %s  %d/%d",
               vibSigCaptureName, vibSigFrames, VIB_SIG_FRAMES);
      vibPlotSprite.setTextColor(COLOR_YELLOW);
    } else if (vibSigLoadedValid) {
      int pct = (int)(vibSigCosine(spec) * 100.0f + 0.5f);
      snprintf(st, sizeof(st), "%s  (yellow)   live match %d%%", vibSigLoaded.name, pct);
      vibPlotSprite.setTextColor(themedText(COLOR_GREENYELLOW));
    } else {
      snprintf(st, sizeof(st), "Tap a row to compare - CAPTURE to save");
      vibPlotSprite.setTextColor(themedText(COLOR_DARKGREY));
    }
    vibPlotSprite.setCursor(pL, 92); vibPlotSprite.print(st);
    tft.drawRGBBitmap(4, 150, vibPlotSprite.getBuffer(), 232, 104);
  }
}

void handleVibSignaturesTouch(TS_Point p) {
  // Name-picker overlay swallows all touches.
  if (vibSigPicker) {
    if (btnVibSigClose.contains(p.x, p.y)) {     // X cancels the picker
      vibSigPicker = false; drawVibSignaturesUI(); return;
    }
    for (int i = 0; i < 8; i++) {
      int bx = 18 + (i & 1) * 108, by = 72 + (i >> 1) * 48;
      if (p.x >= bx && p.x <= bx + 100 && p.y >= by && p.y <= by + 38) {
        strncpy(vibSigCaptureName, VIB_SIG_NAMES[i], sizeof(vibSigCaptureName) - 1);
        vibSigCaptureName[sizeof(vibSigCaptureName) - 1] = 0;
        for (int b = 0; b < VIB_FFT_BINS; b++) {
          vibSigAccum[b]    = 0;
          vibSigBaseSnap[b] = vibBaseV[b];
        }
        vibSigFrames    = 0;
        vibCapSeqSeen   = vibSpecSeq.load();
        vibSigCapturing = true;
        vibSigPicker    = false;
        drawVibSignaturesUI();
        return;
      }
    }
    return;
  }

  if (btnVibSigClose.contains(p.x, p.y)) {
    currentMode = VIB_HOME; drawVibHomeUI(); return;
  }
  if (btnVibSigCapture.contains(p.x, p.y)) {
    vibSigPicker = true; drawVibSignaturesUI(); return;
  }
  if (btnVibSigDelete.contains(p.x, p.y)) {
    if (vibSigSel >= 0 && vibSigSel < vibSigCount) {
      char path[48];
      snprintf(path, sizeof(path), "/vibsig/%s", vibSigList[vibSigSel]);
      LittleFS.remove(path);
      vibSigSel = -1; vibSigLoadedValid = false;
      vibSigScan();
      drawVibSignaturesUI();
    }
    return;
  }
  if (btnVibSigUp.contains(p.x, p.y)) {
    if (vibSigScroll > 0) { vibSigScroll--; vibSigUiDirty = true; }
    return;
  }
  if (btnVibSigDown.contains(p.x, p.y)) {
    if (vibSigScroll + VIB_SIG_VIS_ROWS < vibSigCount) { vibSigScroll++; vibSigUiDirty = true; }
    return;
  }
  // List-row tap.
  if (p.x >= 8 && p.x <= 190 &&
      p.y >= VIB_SIG_LIST_TOP &&
      p.y <  VIB_SIG_LIST_TOP + VIB_SIG_VIS_ROWS * VIB_SIG_ROW_H) {
    int idx = vibSigScroll + (p.y - VIB_SIG_LIST_TOP) / VIB_SIG_ROW_H;
    if (idx < vibSigCount) {
      if (idx == vibSigSel) { vibSigSel = -1; vibSigLoadedValid = false; }
      else { vibSigSel = idx; vibSigLoadedValid = vibSigLoad(vibSigList[idx]); }
      vibSigUiDirty = true;
    }
  }
}

// Helper: draw one key:value row into distSprite and blit to screen at given y
static void sensorRow(const char* label, const char* val, uint16_t valCol, int screenY) {
  distSprite.fillScreen(THEME_BG);
  distSprite.setTextSize(1);
  distSprite.setFont(&FreeSans9pt7b);
  distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  distSprite.setCursor(10, 24); distSprite.print(label);
  distSprite.setTextColor(valCol);
  int16_t x1, y1; uint16_t w, h;
  distSprite.getTextBounds(val, 0, 0, &x1, &y1, &w, &h);
  distSprite.setCursor(230 - w - x1, 24); distSprite.print(val);
  tft.drawRGBBitmap(0, screenY, distSprite.getBuffer(), 240, 30);
}

void refreshSensorInfoValues() {
  uint32_t hState   = sensorHealth.load(std::memory_order_acquire);
  uint32_t ambState = sensorAmbient.load(std::memory_order_acquire);
  // sensorSigma not used (fields unavailable in this driver version)
  uint8_t  status   = (hState >> 24) & 0xFF;
  float    mcps     = (hState & 0xFFFFFF) / 1000.0f;
  float    ambient  = (ambState & 0xFFFFFF) / 1000.0f;


  char buf[32];
  // 50 Mcps = 100% on the display scale (Spectralon ~30 Mcps ≈ 60% — healthy).
  // Real VL53L4CX saturation is 150-200+ Mcps; flag OVLD only above 200 Mcps.
  float sigRaw = mcps / 50.0f * 100.0f;
  float sigPct = constrain(sigRaw, 0.0f, 100.0f);   // for bar fill only
  bool  sigOvld = (mcps > 200.0f);
  uint16_t sigCol;
  if (sigOvld) {
    sigCol = COLOR_RED;
    snprintf(buf, sizeof(buf), "%.1f Mcps OVLD", mcps);
  } else {
    sigCol = (sigPct >= 60) ? COLOR_PUREGREEN : (sigPct >= 30) ? COLOR_YELLOW : COLOR_ORANGE;
    snprintf(buf, sizeof(buf), "%.1f Mcps (%.0f%%)", mcps, sigPct);
  }

  // Row 0: Signal rate — y=42
  sensorRow("Signal:", buf, sigCol, 42);

  // Signal bar — y=73
  distSprite.fillScreen(THEME_BG);
  if (sigOvld) {
    // Full red bar to visually flag saturation
    distSprite.fillRect(15, 4, 210, 12, COLOR_RED);
  } else {
    uint16_t barCol;
    int segs = constrain((int)(mcps * 0.35f), 0, 10);
    if      (segs >= 9) barCol = COLOR_DARKGREEN;
    else if (segs >= 7) barCol = COLOR_PUREGREEN;
    else if (segs >= 5) barCol = COLOR_YELLOW;
    else if (segs >= 3) barCol = COLOR_ORANGE;
    else                barCol = COLOR_RED;
    int barFill = (int)(sigPct / 100.0f * 210.0f);
    distSprite.fillRect(15, 4, barFill, 12, barCol);
    distSprite.fillRect(15 + barFill, 4, 210 - barFill, 12, COLOR_DARKGREY);
  }
  distSprite.drawRect(15, 4, 210, 12, COLOR_LIGHTGREY);
  tft.drawRGBBitmap(0, 73, distSprite.getBuffer(), 240, 20);

  // Row 1: Ambient — y=92
  snprintf(buf, sizeof(buf), "%.3f Mcps", ambient);
  sensorRow("Ambient:", buf, ambient > 5.0f ? COLOR_ORANGE : COLOR_LIGHTGREY, 92);

  // Row 2: Status — y=124 (sigma/reflectance unavailable in this driver version)
  const char* statusDesc;
  uint16_t    statusCol;
  if (sensorSleeping) {
    statusDesc = "USER SLEEP";    statusCol = COLOR_DARKGREY;
  } else if (status == 0) {
    statusDesc = "OK";            statusCol = COLOR_PUREGREEN;
  } else if (status == 1) {
    statusDesc = "LOW S/N";       statusCol = COLOR_ORANGE;
  } else if (status == 2) {
    statusDesc = "LOW SIGNAL";    statusCol = COLOR_ORANGE;
  } else if (status == 3) {
    // RANGE_VALID_MIN_RANGE_CLIPPED — valid, target near sensor floor
    statusDesc = "MIN CLIPPED";   statusCol = COLOR_YELLOW;
  } else if (status == 4) {
    // OUTOFBOUNDS_FAIL — phase out of unambiguous range
    statusDesc = "OUT OF RANGE";  statusCol = COLOR_RED;
  } else if (status == 6) {
    // RANGE_VALID_NO_WRAP_CHECK_FAIL — valid distance; wrap check skipped
    // because signal was too high for the check to be reliable.  Common
    // with specular / high-reflectivity targets at close range.
    statusDesc = "HIGH SIG OK";   statusCol = COLOR_YELLOW;
  } else if (status == 7) {
    // WRAP_TARGET_FAIL — signal saturated the phase counter; at microscope
    // distances (<200 mm) this is a false alarm.  Enable High Reflectivity
    // mode to shorten the timing budget and resolve the overload.
    statusDesc = "SATURATED";     statusCol = COLOR_RED;
  } else if (status == 9) {
    statusDesc = "XTALK FAIL";    statusCol = COLOR_RED;
  } else if (status == 11) {
    statusDesc = "MULTI-OBJECT";  statusCol = COLOR_YELLOW;
  } else if (status == 13) {
    statusDesc = "TOO CLOSE";     statusCol = COLOR_ORANGE;
  } else if (status == 255) {
    statusDesc = "NO DATA";       statusCol = COLOR_DARKGREY;
  } else {
    snprintf(buf, sizeof(buf), "ERR %d", status); statusDesc = buf; statusCol = COLOR_RED;
  }
  sensorRow("Status:", statusDesc, statusCol, 124);
}

void drawSensorInfoUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("TOF SENSOR", 5, 5, COLOR_DARKBLUE);
  refreshSensorInfoValues();   // draws all live rows via sprites
  btnSensorToggle.draw(tft,
    sensorSleeping ? "WAKE SENSOR" : "SLEEP SENSOR",
    sensorSleeping ? COLOR_DARKGREEN : COLOR_MAROON, TFT_WHITE);
  btnSensorHiRef.draw(tft,
    highReflMode ? "HIGH REFLECTIVITY: ON" : "HIGH REFLECTIVITY: OFF",
    highReflMode ? 0x0340 : COLOR_DARKGREY,   // dark teal when active
    TFT_WHITE);
  btnSensorBack.draw(tft);
}

void handleSensorInfoTouch(TS_Point p) {
  if (btnSensorToggle.contains(p.x, p.y)) {
    // State flags flip only when the guarded I2C work actually ran — on a
    // mutex timeout (wedged bus) the tap is a no-op instead of leaving the
    // UI claiming a state the hardware never reached.
    if (sensorSleeping) {
      // Wake: re-apply config (timing budget + ROI) then restart.
      // Sensor is already stopped; applyHighReflConfig() only sets parameters.
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
        applyHighReflConfig();
        sensor.VL53L4CX_StartMeasurement();
        xSemaphoreGive(i2cMutex);
        sensorEmaReset.store(true, std::memory_order_release);
        sensorSleeping = false;
        tofAutoSlept = false;   // manually woken — the idle-resume branch must
                                // not re-configure the now-ranging sensor
      }
    } else {
      // Sleep: flag FIRST so sensorTask stops polling (see sensorSleeping
      // decl — a latched data-ready would otherwise restart ranging), then
      // stop; revert the flag if the stop never ran.
      sensorSleeping = true;
      sensorState.store(0, std::memory_order_release);
      sensorHealth.store(0xFF000000UL, std::memory_order_release);
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
        sensor.VL53L4CX_StopMeasurement();
        xSemaphoreGive(i2cMutex);
      } else {
        sensorSleeping = false;
      }
    }
    drawSensorInfoUI(); // full redraw to update toggle label
    return;
  }
  if (btnSensorHiRef.contains(p.x, p.y)) {
    if (!sensorSleeping) {
      // Restart with the new timing budget + ROI. Flip the mode only when the
      // stop/reconfig/restart actually ran.
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
        sensor.VL53L4CX_StopMeasurement();
        highReflMode = !highReflMode;
        applyHighReflConfig();   // 8 ms + 8×8 ROI  OR  33 ms + full ROI
        sensor.VL53L4CX_StartMeasurement();
        xSemaphoreGive(i2cMutex);
        sensorEmaReset.store(true, std::memory_order_release);
      }
    } else {
      highReflMode = !highReflMode;   // asleep: config is applied on wake
    }
    drawSensorInfoUI();
    return;
  }
  if (btnSensorBack.contains(p.x, p.y)) {
    currentMode = MAIN; drawMainScreen();
  }
}


// ─── patched3: WiFi indicator + WIFI_INFO screen ─────────────────────────────

// drawWifiIndicator()  —  header zone x=93..151, y=0..42.
// Draws 4 signal bars + a status label.  Called from drawMainScreen() and from
// the wifi tab whenever the connection state changes (refreshWifiIndicator).
void drawWifiIndicator() {
  const int ZX = 0,   ZW = 32,  ZH = 42;
  const int BAR_BASE = 38;                   // bottom y of the tallest bar
  const int BAR_W = 5, BAR_GAP = 3;
  const int ICON_W = 4 * BAR_W + 3 * BAR_GAP;  // 29 px
  const int ICON_X = ZX + (ZW - ICON_W) / 2;   // ≈ x=108
  const int BAR_H[4] = { 6, 11, 17, 23 };       // heights of the 4 bars (weakest→strongest)

  tft.fillRect(ZX, 0, ZW, ZH, THEME_BG);

  bool connected = wifiIsConnected();
  bool portal    = wifiIsPortal();
  int  rssi      = connected ? wifiGetRSSI() : 0;

  // How many bars are "lit" and in what color
  int     litBars = 0;
  uint16_t litColor = COLOR_PUREGREEN;
  const uint16_t dimColor = 0x2945;   // dark charcoal

  if (portal) {
    litBars  = 4;
    litColor = COLOR_ORANGE;
  } else if (connected) {
    if      (rssi > -50) { litBars = 4; litColor = COLOR_PUREGREEN; }
    else if (rssi > -65) { litBars = 3; litColor = COLOR_PUREGREEN; }
    else if (rssi > -75) { litBars = 2; litColor = COLOR_YELLOW;    }
    else                 { litBars = 1; litColor = COLOR_RED;        }
  }

  for (int i = 0; i < 4; i++) {
    int bx = ICON_X + i * (BAR_W + BAR_GAP);
    int bh = BAR_H[i];
    int by = BAR_BASE - bh;
    tft.fillRect(bx, by, BAR_W, bh, (i < litBars) ? litColor : dimColor);
  }

  // Small status label centered above the bars
  tft.setFont(); tft.setTextSize(1);
  uint16_t lblColor = portal    ? COLOR_ORANGE
                    : connected ? themedText(COLOR_GREENYELLOW)
                                : themedText(COLOR_DARKGREY);
  tft.setTextColor(lblColor);
  const char* lbl = portal ? "SETUP" : "WiFi";
  int lw = strlen(lbl) * 6;   // built-in 6×8 font: 6 px per char
  tft.setCursor(ZX + (ZW - lw) / 2, 4);
  tft.print(lbl);
}

// refreshWifiInfoValues()  —  redraws the dynamic rows on WIFI_INFO.
// Polled from loop() every 1 s so RSSI and connection state stay live without
// the user having to bounce off the screen and back to refresh.
void refreshWifiInfoValues() {
  String ssid = wifiGetSSID();
  if (ssid.isEmpty()) ssid = wifiIsPortal() ? "AutoFOV-Setup (AP)" : "---";
  sensorRow("SSID:", ssid.c_str(), themedText(COLOR_LIGHTGREY), 42);

  String ip = wifiGetIP();
  if (ip.isEmpty()) ip = wifiIsPortal() ? "192.168.4.1" : "---";
  sensorRow("IP:", ip.c_str(), COLOR_GREENYELLOW, 74);

  if (wifiIsConnected()) {
    char rssiBuf[16];
    int rssi = wifiGetRSSI();
    snprintf(rssiBuf, sizeof(rssiBuf), "%d dBm", rssi);
    uint16_t rssiCol = (rssi > -50) ? COLOR_PUREGREEN
                     : (rssi > -65) ? COLOR_YELLOW : COLOR_RED;
    sensorRow("RSSI:", rssiBuf, rssiCol, 106);
    sensorRow("Mode:", "STA (connected)", themedText(COLOR_LIGHTGREY), 138);
  } else if (wifiIsPortal()) {
    // Password row drawn by hand (not sensorRow) so the setup code can use
    // FreeSans12 — one size up from the FreeSans9 every other row uses. It's
    // the one value the user must read off and type into their phone, so it
    // gets the visual weight. Label stays FreeSans9 for row consistency.
    const char* code = wifiGetPortalCode();
    distSprite.fillScreen(THEME_BG);
    distSprite.setTextSize(1);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
    distSprite.setCursor(10, 24); distSprite.print("Password:");
    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(COLOR_ORANGE);
    int16_t cx1, cy1; uint16_t cw, ch;
    distSprite.getTextBounds(code, 0, 0, &cx1, &cy1, &cw, &ch);
    distSprite.setCursor(230 - cw - cx1, 24); distSprite.print(code);
    tft.drawRGBBitmap(0, 106, distSprite.getBuffer(), 240, 30);
    sensorRow("Mode:", "AP setup portal", COLOR_ORANGE, 138);
  } else {
    sensorRow("RSSI:", "---",              themedText(COLOR_DARKGREY), 106);
    sensorRow("Mode:", "STA (offline)",    COLOR_RED, 138);
  }

  // Clear the hint area (y=168..207) — content depends on connection state
  // and can change between refreshes (e.g. portal→connected after restart).
  tft.fillRect(0, 168, 240, 40, THEME_BG);
  tft.setFont(); tft.setTextSize(1);
  if (wifiIsConnected()) {
    tft.setTextColor(themedText(COLOR_DARKGREY));
    String hint = "http://" + ip;
    tft.setCursor(10, 175); tft.print(hint);
    // Login password (also required to flash firmware). Show the auto-generated
    // default so the user can read it off the device; once they've set their own
    // custom password, don't display the secret — point them at the BOOT reset.
    tft.setTextColor(COLOR_ORANGE);
    tft.setCursor(10, 189);
    if (wifiHasCustomPassword()) {
      tft.print("Login pw: custom");
      tft.setTextColor(themedText(COLOR_DARKGREY));
      tft.setCursor(10, 199); tft.print("(hold BOOT at power-on to reset)");
    } else {
      tft.print("Login pw: "); tft.print(wifiGetOtaPassword());
    }
  } else if (wifiIsPortal()) {
    tft.setTextColor(COLOR_ORANGE);
    tft.setCursor(10, 175); tft.print("1. Join 'AutoFOV-Setup' with password");
    tft.setCursor(10, 187); tft.print("2. Open browser -> 192.168.4.1");
  }
}

// drawWifiInfoUI()  —  full-screen WiFi status + forget-credentials screen.
// Paints the static parts (title banner, buttons) and then delegates to
// refreshWifiInfoValues() for the dynamic content.
void drawWifiInfoUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("WiFi", 5, 5, COLOR_TEAL);

  refreshWifiInfoValues();

  btnWifiInfoForget.draw(tft);
  btnWifiInfoBack.draw(tft);
  btnWifiInfoTips.draw(tft);
}

void handleWifiInfoTouch(TS_Point p) {
  if (btnWifiInfoTips.contains(p.x, p.y)) {
    openInfoScreen(WIFI_TIPS);
    return;
  }
  if (btnWifiInfoForget.contains(p.x, p.y)) {
    // Flash label and schedule the actual clear+restart for 600 ms later.
    // The deferred timer is polled by loop(); this returns immediately so
    // touch / WS handling stays responsive during the flash window.
    btnWifiInfoForget.draw(tft, "CLEARING...", COLOR_MAROON, TFT_WHITE);
    wifiForgetAtMs = millis() + 600;
    return;
  }
  if (btnWifiInfoBack.contains(p.x, p.y)) {
    currentMode = MAIN; drawMainScreen();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// V12.3: drag-scrolled info overlays — SECURITY TIPS (from WIFI_INFO) and
// RECOVERY FLASH help (from ABOUT). These bring the on-device screens to parity
// with the web dashboard's (i) overlays. Content is word-wrapped at runtime to
// the panel width, rendered once into a tall off-screen PSRAM canvas, then
// scrolled by blitting a finger-following window of it (smooth, no per-frame
// re-render). A thin right-edge scrollbar shows position + extent.
// (struct InfoBlock + INFO_* kinds are declared up by the DisplayMode enum.)
// ─────────────────────────────────────────────────────────────────────────────
static const InfoBlock WIFI_TIPS_CONTENT[] = {
  {"! Power from a wall charger", INFO_WARN},
  {"Power AutoFOV from a USB wall charger, not a computer. A USB data link to a PC can expose the device console and allow reflashing.", INFO_BODY},
  {"Use a strong login password", INFO_HEAD},
  {"Set a unique device password in WiFi Setup. It guards the dashboard and firmware flashing. Blank falls back to the random code shown on the device screen.", INFO_BODY},
  {"Keep AutoFOV on your LAN", INFO_HEAD},
  {"Traffic is plain HTTP (no TLS, by design). Never port-forward AutoFOV to the internet or join it to an untrusted public network.", INFO_BODY},
  {"Prefer the IP over autofov.local", INFO_HEAD},
  {"On a shared network the numeric IP skips the mDNS name lookup, which other devices can spoof. Otherwise the two are equally secure.", INFO_BODY},
  {"Sign out on shared computers", INFO_HEAD},
  {"Your login is remembered per-browser. Tap SIGN OUT when you're on a machine that isn't yours.", INFO_BODY},
  {"Locked out? Hold BOOT", INFO_HEAD},
  {"Hold the BOOT button while powering on to relaunch WiFi Setup and set new credentials - no calibration is lost.", INFO_BODY},
  {"Can't update?", INFO_HEAD},
  {"If an update fails or leaves AutoFOV misbehaving, you can reflash over USB from a browser - no terminal needed - and it keeps your calibration and WiFi settings.", INFO_BODY},
};

static const InfoBlock RECOVERY_HELP_CONTENT[] = {
  {"If an update or settings change leaves AutoFOV misbehaving, you can reflash it over USB from your browser - no terminal needed - keeping your calibration and WiFi settings.", INFO_BODY},
  {"! Bookmark the link first", INFO_WARN},
  {"The flasher runs on your computer, so it can even recover a device that won't boot - but you can't open the page if the device is dead. Save the link while you still can.", INFO_BODY},
  {"You need", INFO_HEAD},
  {"- A computer running Chrome or Edge (Safari and Firefox can't flash over USB).", INFO_BODY},
  {"- A data-capable USB-C cable - charge-only cables won't enumerate the device.", INFO_BODY},
  {"Steps", INFO_HEAD},
  {"1. Open toldxls.github.io/AutoFOV-V12 in Chrome or Edge.", INFO_BODY},
  {"2. Plug AutoFOV into the computer.", INFO_BODY},
  {"3. Click the green Recovery Flash button.", INFO_BODY},
  {"4. Pick the device's serial port, then Connect.", INFO_BODY},
  {"5. Click Install and wait - don't unplug.", INFO_BODY},
  {"6. The device reboots into working firmware on its own.", INFO_BODY},
  {"No port shown?", INFO_HEAD},
  {"The ESP32-S3 usually enters flash mode by itself. If no port appears: hold BOOT, briefly tap RESET, release BOOT, then click Recovery Flash again.", INFO_BODY},
};

static const InfoBlock CALIB_HELP_CONTENT[] = {
  {"What calibration does", INFO_HEAD},
  {"AutoFOV reads the distance to your subject and converts it to a field-of-view width. Calibration fits that distance-to-FOV curve to YOUR bellows and camera by recording a few known points.", INFO_BODY},
  {"You'll need", INFO_HEAD},
  {"A reference of known width in the frame - a stage micrometer or ruler - so you can count how many pixels it spans in a photo at each step.", INFO_BODY},
  {"The three settings", INFO_HEAD},
  {"Photo Width - your camera's image width in pixels (e.g. 6000 for a 24MP sensor).", INFO_BODY},
  {"Demarc Dist - the real width, in mm, of the reference feature you count pixels across.", INFO_BODY},
  {"Cal Points - a TARGET number of steps. You no longer have to hit it exactly (see below).", INFO_BODY},
  {"How to calibrate", INFO_HEAD},
  {"1. Set Photo Width and Demarc Dist, then tap START CAL.", INFO_BODY},
  {"2. At each bellows distance: take a photo, count the pixels across your reference, type that number, and CAPTURE & SAVE.", INFO_BODY},
  {"3. Move the bellows and repeat across its full range. About 10 mm steps work well.", INFO_BODY},
  {"Stop whenever you like", INFO_HEAD},
  {"You don't need to pre-figure how many steps fit your bellows. Capture points at whatever spacing you manage, then tap FINISH to compute the fit from exactly the points you took.", INFO_BODY},
  {"Tips", INFO_HEAD},
  {"Use at least 3 points; 5 or more spread across your whole focus range gives a tighter, more trustworthy fit. REVIEW lets you check or RETAKE any point before you finish.", INFO_BODY},
};

// Flattened wrapped lines for the active overlay (built by buildInfoLines).
#define INFO_MAX_LINES 80
static String      infoLines[INFO_MAX_LINES];
static uint8_t     infoLineKind[INFO_MAX_LINES];
static int         infoLineCount  = 0;
static const char* infoTitle      = "";
static uint16_t    infoTitleCol   = COLOR_TEAL;
static DisplayMode infoReturnMode = WIFI_INFO;

// Off-screen render of the whole text column. Scrolling just blits a different
// vertical slice — contiguous memory, since the canvas is 240 wide — so the drag
// follows the finger with one fast drawRGBBitmap per frame (no re-rendering).
static PSRAMCanvas16* infoCanvas  = nullptr;
static int  infoCanvasH         = 0;
static int  infoScrollY         = 0;    // current pixel scroll offset
static int  infoMaxScrollY      = 0;
static bool infoDragActive      = false;
static int  infoDragStartY      = 0;    // touch-Y where the drag began
static int  infoDragStartScroll = 0;    // infoScrollY at drag start

// Geometry. Text column is x=10..(10+INFO_TEXT_W); the scrollbar sits at x=234,
// and INFO_TEXT_W leaves a margin before both it and the 240 px canvas edge so a
// wrapped line never reaches the edge (which would trigger GFX auto-wrap).
static const int INFO_LINE_H = 18;
static const int INFO_TEXT_W = 208;     // word-wrap width, px (x=10..218)
static const int INFO_VP_TOP = 42;      // viewport top on screen
static const int INFO_VP_H   = 272;     // viewport height (42..314)

static int infoTextW(const String& s) {
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  return (int)w;
}
static void infoPushLine(const String& s, uint8_t kind) {
  if (infoLineCount < INFO_MAX_LINES) {
    infoLines[infoLineCount] = s; infoLineKind[infoLineCount] = kind; infoLineCount++;
  }
}

// Word-wrap the content blocks into infoLines[] at the render font (9pt). Uses
// tft metrics for FreeSans9 — the same font buildInfoCanvas() renders with — so
// the measured wrap matches the drawn glyphs. Words wider than the column (e.g.
// the recovery URL) are broken at the character level so nothing overflows.
static void buildInfoLines(const InfoBlock* blocks, int nBlocks) {
  infoLineCount = 0;
  setSmoothFont(1);
  for (int b = 0; b < nBlocks && infoLineCount < INFO_MAX_LINES; b++) {
    // Blank separator before each section heading (not the first block).
    if (b > 0 && blocks[b].kind != INFO_BODY) infoPushLine("", INFO_BODY);

    String text = blocks[b].text;
    String line = "";
    int start = 0;
    while (start <= (int)text.length() && infoLineCount < INFO_MAX_LINES) {
      int sp = text.indexOf(' ', start);
      String w = (sp < 0) ? text.substring(start) : text.substring(start, sp);

      String trial = line.length() ? line + " " + w : w;
      if (infoTextW(trial) <= INFO_TEXT_W) {
        line = trial;
      } else {
        if (line.length()) { infoPushLine(line, blocks[b].kind); line = ""; }
        if (infoTextW(w) <= INFO_TEXT_W) {
          line = w;
        } else {
          // Single word too wide for the column — break it character by character.
          String chunk = "";
          for (int ci = 0; ci < (int)w.length() && infoLineCount < INFO_MAX_LINES; ci++) {
            String t2 = chunk + w[ci];
            if (chunk.length() && infoTextW(t2) > INFO_TEXT_W) {
              infoPushLine(chunk, blocks[b].kind);
              chunk = String(w[ci]);
            } else {
              chunk = t2;
            }
          }
          line = chunk;
        }
      }
      if (sp < 0) break;
      start = sp + 1;
    }
    if (line.length()) infoPushLine(line, blocks[b].kind);
  }
}

static uint16_t infoLineColor(uint8_t kind) {
  return (kind == INFO_HEAD) ? themedText(COLOR_TEAL)
       : (kind == INFO_WARN) ? COLOR_ORANGE
       :                       themedText(COLOR_LIGHTGREY);
}

static void freeInfoCanvas() {
  if (infoCanvas) { delete infoCanvas; infoCanvas = nullptr; }
}

// Render every wrapped line into the tall off-screen canvas, once per open.
static void buildInfoCanvas() {
  freeInfoCanvas();
  infoCanvasH = infoLineCount * INFO_LINE_H + 14;
  if (infoCanvasH < INFO_VP_H) infoCanvasH = INFO_VP_H;
  infoMaxScrollY = infoCanvasH - INFO_VP_H;
  if (infoMaxScrollY < 0) infoMaxScrollY = 0;
  infoCanvas = new (std::nothrow) PSRAMCanvas16(240, infoCanvasH);
  if (!infoCanvas) return;                 // render falls back to software text
  infoCanvas->begin();
  if (!infoCanvas->allocated()) return;    // ps_malloc failed → text fallback
  infoCanvas->fillScreen(THEME_BG);
  infoCanvas->setFont(&FreeSans9pt7b);
  infoCanvas->setTextWrap(false);         // never auto-wrap onto the next line; lines are pre-fit
  int y = 14;                             // baseline of the first line
  for (int i = 0; i < infoLineCount; i++) {
    if (infoLines[i].length()) {
      infoCanvas->setTextColor(infoLineColor(infoLineKind[i]));
      infoCanvas->setCursor(10, y);
      infoCanvas->print(infoLines[i]);
    }
    y += INFO_LINE_H;
  }
}

// Disable hardware vertical scroll → restore normal full-screen addressing.
// MUST run before drawing any non-overlay screen, or that screen renders into a
// scrolled frame. Called from redrawCurrentScreen()/wakeScreen() and on open.
void infoScrollReset() {
  tft.setScrollMargins(0, 0);                 // TFA=0, VSA=320, BFA=0
  tft.scrollTo(0);
}

// Draw `count` virtual content rows starting at vStart into their GDDRAM ring
// slots (gddram_y = INFO_VP_TOP + (v % INFO_VP_H)) from the off-screen canvas.
// The ring wrap splits into at most two contiguous spans. Only the changed rows
// are pushed, so a scroll step touches just a few rows — no full-viewport sweep.
static void infoDrawRows(int vStart, int count) {
  if (!infoCanvas || !infoCanvas->allocated() || count <= 0) return;
  int v = vStart, remaining = count;
  while (remaining > 0) {
    int ring = v % INFO_VP_H;                 // v >= 0 here → plain modulo
    int gY   = INFO_VP_TOP + ring;
    int span = INFO_VP_H - ring;              // rows until the ring wraps
    if (span > remaining) span = remaining;
    int avail = infoCanvasH - v;              // real content rows left
    if (avail <= 0) {
      tft.fillRect(0, gY, 240, span, THEME_BG);
    } else {
      int drawn = (span < avail) ? span : avail;
      tft.drawRGBBitmap(0, gY, infoCanvas->getBuffer() + (int32_t)v * 240, 240, drawn);
      if (drawn < span) tft.fillRect(0, gY + drawn, 240, span - drawn, THEME_BG);
    }
    v += span; remaining -= span;
  }
}

// Software fallback when the PSRAM canvas failed to allocate: line-granular
// direct render (no hardware scroll → the old flicker, but it still works).
static void infoSoftRender() {
  tft.fillRect(0, INFO_VP_TOP, 240, INFO_VP_H, THEME_BG);
  setSmoothFont(1);
  tft.setTextWrap(false);
  int first = infoScrollY / INFO_LINE_H;
  int yb = INFO_VP_TOP + 14 - (infoScrollY % INFO_LINE_H);
  for (int i = first; i < infoLineCount && yb < INFO_VP_TOP + INFO_VP_H + INFO_LINE_H; i++) {
    if (infoLines[i].length()) {
      tft.setTextColor(infoLineColor(infoLineKind[i]));
      tft.setCursor(10, yb);
      tft.print(infoLines[i]);
    }
    yb += INFO_LINE_H;
  }
  tft.setTextWrap(true);
}

// Move to a new scroll position. With the canvas: draw only the newly-revealed
// rows into the ring, then nudge the hardware scroll pointer — the bulk of the
// content is never redrawn, so there's no blit sweep (the blink). Falls back to
// a full software re-render when the canvas is unavailable.
static void infoScrollToY(int newS) {
  if (newS < 0) newS = 0;
  if (newS > infoMaxScrollY) newS = infoMaxScrollY;
  if (newS == infoScrollY) return;
  if (infoCanvas && infoCanvas->allocated()) {
    int d = newS - infoScrollY;
    if (d >= INFO_VP_H || -d >= INFO_VP_H) infoDrawRows(newS, INFO_VP_H);  // big jump
    else if (d > 0) infoDrawRows(infoScrollY + INFO_VP_H, d);              // reveal bottom
    else            infoDrawRows(newS, -d);                                // reveal top
    infoScrollY = newS;
    tft.scrollTo(INFO_VP_TOP + (infoScrollY % INFO_VP_H));
  } else {
    infoScrollY = newS;
    infoSoftRender();
  }
}

void openInfoScreen(DisplayMode mode) {
  infoReturnMode = currentMode;     // return to whatever opened us
  if (mode == RECOVERY_HELP) {
    infoTitle = "RECOVERY FLASH"; infoTitleCol = COLOR_TEAL;
    buildInfoLines(RECOVERY_HELP_CONTENT,
                   sizeof(RECOVERY_HELP_CONTENT) / sizeof(RECOVERY_HELP_CONTENT[0]));
  } else if (mode == CALIB_HELP) {
    infoTitle = "CALIBRATION"; infoTitleCol = COLOR_TEAL;
    buildInfoLines(CALIB_HELP_CONTENT,
                   sizeof(CALIB_HELP_CONTENT) / sizeof(CALIB_HELP_CONTENT[0]));
  } else {
    infoTitle = "SECURITY TIPS"; infoTitleCol = COLOR_TEAL;
    buildInfoLines(WIFI_TIPS_CONTENT,
                   sizeof(WIFI_TIPS_CONTENT) / sizeof(WIFI_TIPS_CONTENT[0]));
  }
  buildInfoCanvas();
  infoScrollY    = 0;
  infoDragActive = false;
  currentMode    = mode;
  drawInfoTextScreen();
}

void drawInfoTextScreen() {
  infoScrollReset();                          // normal addressing for the fixed parts
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText(infoTitle, 5, 5, infoTitleCol);
  btnInfoTextClose.draw(tft);
  tft.drawFastHLine(5, 38, 230, 0x39E7);
  if (infoScrollY > infoMaxScrollY) infoScrollY = infoMaxScrollY;
  if (infoScrollY < 0) infoScrollY = 0;
  if (infoCanvas && infoCanvas->allocated()) {
    infoDrawRows(infoScrollY, INFO_VP_H);     // paint the visible window into the ring
    tft.setScrollMargins(INFO_VP_TOP, 320 - INFO_VP_TOP - INFO_VP_H);   // TFA=42, BFA=6
    tft.scrollTo(INFO_VP_TOP + (infoScrollY % INFO_VP_H));
  } else {
    infoSoftRender();
  }
}

void handleInfoTextTouch(TS_Point p) {
  // Title bar (above the viewport) — the X close. Never starts a drag, and is
  // ignored mid-drag so a vertical swipe can't accidentally close.
  if (p.y < INFO_VP_TOP) {
    if (!infoDragActive && btnInfoTextClose.contains(p.x, p.y)) {
      freeInfoCanvas();
      currentMode = infoReturnMode;
      redrawCurrentScreen();
    }
    return;
  }
  // Drag-scroll the body. The first sample of a press sets the baseline; later
  // samples track the finger 1:1 so the text stays under it and your place holds.
  if (!infoDragActive) {
    infoDragActive      = true;
    infoDragStartY      = p.y;
    infoDragStartScroll = infoScrollY;
    return;
  }
  int ns = infoDragStartScroll - (p.y - infoDragStartY);
  if (ns < 0) ns = 0;
  if (ns > infoMaxScrollY) ns = infoMaxScrollY;
  // Deadzone: the FT6206 jitters a couple px between reads. Only act once the
  // move exceeds a few px (but always honor the clamped ends so the last bit of
  // travel isn't lost). infoScrollToY() no-ops if the position is unchanged.
  if (abs(ns - infoScrollY) >= 3 || ns == 0 || ns == infoMaxScrollY) {
    infoScrollToY(ns);
  }
}


// redrawCurrentScreen() — full repaint of the currently-active mode.
// Used by the deferred WS theme/tint repaint (see displayNeedsRedraw poll
// in loop()).  Mirrors wakeScreen()'s switch — kept in sync with it.
//
// CAL_SAMPLING is excluded because it runs an active 2-second average that
// owns the screen; mid-sample repaint would erase the sampling progress
// rendered by the loop's CAL_SAMPLING block.
void redrawCurrentScreen() {
  // V12.3: leaving an info overlay must restore normal addressing first, or the
  // target screen renders into a scrolled frame. Overlay modes re-enable scroll
  // themselves in drawInfoTextScreen(), so an unconditional reset here is safe.
  infoScrollReset();
  // Release the ~334 KB overlay canvas whenever we're not (re)drawing an overlay
  // — e.g. when the web navigates away, which doesn't hit the X-close path.
  if (currentMode != WIFI_TIPS && currentMode != RECOVERY_HELP && currentMode != CALIB_HELP) freeInfoCanvas();
  switch (currentMode) {
    case MAIN:               drawMainScreen(); break;
    case APP_SETTINGS:       drawAppSettingsUI(); break;
    case CAL_SETTINGS:       drawCalSettingsUI(); break;
    case CAL_RUN:            drawPointEntryUI(); break;
    case STACK_CALC:         drawStackCalcUI(); break;
    case STACK_TIME:         drawStackTimeUI(); break;
    case CAL_CONFIRM:        drawConfirmUI(); break;
    case CAL_SUCCESS:        drawSuccessScreen(); break;
    case FOV_INFO:           drawFovInfoUI(); break;
    case CAL_REVIEW:         drawCalReviewUI(); break;
    case BRIGHTNESS_SETTINGS:drawBrightnessSettingsUI(); break;
    case SENSOR_INFO:        drawSensorInfoUI(); break;
    case MEM_INFO:           drawMemInfoUI(); break;
    case CAL_GRAPH:          drawCalGraphUI(); break;
    case ABOUT:              drawAboutUI(); break;
    case SCREEN_TIMEOUT:     drawScreenTimeoutUI(); break;
    case WIFI_INFO:          drawWifiInfoUI(); break;
    case IR_CONTROL:         drawIrControlUI(); break;
    case VIB_HOME:           drawVibHomeUI(); break;
    case VIB_SPECTRUM:       drawVibSpectrumUI(); break;
    case VIB_SETTLE:         drawVibSettleUI(); break;
    case VIB_SIGNATURES:     drawVibSignaturesUI(); break;
    case WIFI_TIPS:          drawInfoTextScreen(); break;
    case RECOVERY_HELP:      drawInfoTextScreen(); break;
    case CALIB_HELP:         drawInfoTextScreen(); break;
    case CAL_SAMPLING:       /* skip — sampling actively owns the screen */ break;
    default:                 drawMainScreen(); break;
  }
}

// ─────────────────────────────────────────────────────────────────────────────

void recoverI2CBus() {
  pinMode(SCL, OUTPUT); pinMode(SDA, INPUT_PULLUP);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL, HIGH); delayMicroseconds(5);
    if (digitalRead(SDA) == HIGH) break;
    digitalWrite(SCL, LOW); delayMicroseconds(5);
  }
  digitalWrite(SCL, LOW); delayMicroseconds(5);
  pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW); delayMicroseconds(5);
  digitalWrite(SCL, HIGH); delayMicroseconds(5);
  digitalWrite(SDA, HIGH); delayMicroseconds(5);
  pinMode(SCL, INPUT); pinMode(SDA, INPUT);
}

volatile bool touchDetected = false;
void IRAM_ATTR touchISR() { touchDetected = true; }

// Apply the timing budget and SPAD ROI that match the current highReflMode.
// Must be called with i2cMutex held and measurement already stopped.
//
// Normal mode  : 33 ms budget, full 16×16 ROI  → baseline sensitivity
// Hi-Refl mode : 8 ms budget, centre 8×8 ROI   → ~16× signal reduction
//   8 ms  vs 33 ms = 4.1× less photon accumulation
//   8×8   vs 16×16 = 4×   fewer active SPADs
//   Combined ≈ 16× — brings a 300 Mcps gold-silicon target to ~19 Mcps
void applyHighReflConfig() {
  VL53L4CX_UserRoi_t roi;
  if (highReflMode) {
    roi.TopLeftX  = 4;  roi.TopLeftY  = 4;
    roi.BotRightX = 11; roi.BotRightY = 11;   // centre 8×8 of 16×16 array
    sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(8000);
  } else {
    roi.TopLeftX  = 0;  roi.TopLeftY  = 0;
    roi.BotRightX = 15; roi.BotRightY = 15;   // full 16×16 array
    sensor.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(33000);
  }
  sensor.VL53L4CX_SetUserROI(&roi);
}

void sensorTask(void *pvParameters) {
  float emaDistance = -1.0;
  float emaSignal = -1.0;
  const float EMA_ALPHA = 0.3; 
  // 0.05 (was 0.2): at close range the VL53L4CX splits the return into 1-vs-2
  // "objects" frame to frame, so the summed Mcps flickers bimodally (observed
  // 17↔40 Mcps at 27 mm). At the 33 ms timing budget α=0.05 gives τ≈0.7 s —
  // the readout settles to the average return instead of strobing between the
  // two modes. High-refl mode updates ~4× faster → τ≈0.2 s, still calm.
  const float SIGNAL_EMA_ALPHA = 0.05;

  for (;;) {
    // V12.5 (F3): liveness heartbeat, stamped at the TOP of every iteration
    // (including the sleep path's continue). loop() (Core 1) records the worst
    // staleness in sensorMaxStallMs for /diag — the auto-reboot was REMOVED
    // (it false-fired on transient stalls; see the F3 note in loop()). Stored
    // before any i2cMutex take, so a task blocked on the mutex/Wire never
    // refreshes it.
    sensorHeartbeatMs.store(millis(), std::memory_order_release);

    // V17b: reset EMA state after a sleep/wake cycle
    if (sensorEmaReset.load(std::memory_order_acquire)) {
      emaDistance = -1.0;
      emaSignal   = -1.0;
      sensorEmaReset.store(false, std::memory_order_release);
    }
    // Asleep: don't touch the sensor at all. Polling here would see a
    // data-ready latched before the stop and restart ranging via
    // ClearInterruptAndStartMeasurement(). Re-clear the published state each
    // pass so a frame that raced past the sleep toggle can't stay frozen
    // on-screen as a stale "valid" distance for the whole sleep.
    if (sensorSleeping.load(std::memory_order_acquire)) {
      sensorState.store(0, std::memory_order_release);
      sensorHealth.store(0xFF000000UL, std::memory_order_release);
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    uint8_t dataReady = 0;
    if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
      // Re-check under the mutex: the sleep toggle can land between the check
      // above and this take. The sleep paths raise the flag BEFORE taking the
      // mutex to stop, so seeing it clear here means any concurrent stop will
      // run after we release — and seeing it set means we must not service a
      // latched data-ready (ClearInterrupt… would restart ranging).
      if (sensorSleeping.load(std::memory_order_acquire)) {
        xSemaphoreGive(i2cMutex);
        continue;
      }
      sensor.VL53L4CX_GetMeasurementDataReady(&dataReady);
      if (dataReady) {
        VL53L4CX_MultiRangingData_t multiRangingData;
        sensor.VL53L4CX_GetMultiRangingData(&multiRangingData);
        sensor.VL53L4CX_ClearInterruptAndStartMeasurement();
        xSemaphoreGive(i2cMutex); 

        float maxCps = 0.0;
        uint8_t bestStatus = 255;
        
        for (int i = 0; i < multiRangingData.NumberOfObjectsFound; i++) {
          uint8_t status = multiRangingData.RangeData[i].RangeStatus;
          if (status < bestStatus) bestStatus = status;
          float currentCps = multiRangingData.RangeData[i].SignalRateRtnMegaCps / 65536.0f;
          if (currentCps > maxCps) maxCps = currentCps;
        }

        float weightedDistanceSum = 0.0;
        float validSignalSum = 0.0; 
        float totalSignal = 0.0;    
        float signalThreshold = maxCps * 0.20; 

        for (int i = 0; i < multiRangingData.NumberOfObjectsFound; i++) {
          float cps = multiRangingData.RangeData[i].SignalRateRtnMegaCps / 65536.0f;
          uint8_t status = multiRangingData.RangeData[i].RangeStatus;
          
          if (cps >= signalThreshold) {
            totalSignal += cps;
            // Accept status 0 (RANGE_VALID), 3 (MIN_RANGE_CLIPPED — valid for
            // near targets), and 6 (RANGE_VALID_NO_WRAP_CHECK_FAIL — common
            // with high-reflectivity targets; wrap-around is impossible at
            // microscope distances so the distance is still correct).
            if (status == 0 || status == 3 || status == 6) {
              int dist = multiRangingData.RangeData[i].RangeMilliMeter;
              // RangeMilliMeter is int16_t and status-3 (MIN_RANGE_CLIPPED)
              // objects can come back slightly NEGATIVE at near-zero range
              // (offset correction). A negative average would wrap to ~2^31 mm
              // in the uint32 payload below — clamp at the source.
              if (dist < 0) dist = 0;
              weightedDistanceSum += (dist * cps);
              validSignalSum += cps;
            }
          }
        }
        
        if (totalSignal > 0.0) {
          if (emaSignal < 0.0) emaSignal = totalSignal;
          else emaSignal = (SIGNAL_EMA_ALPHA * totalSignal) + ((1.0 - SIGNAL_EMA_ALPHA) * emaSignal);
        } else {
          emaSignal = 0.0;
        }

        uint32_t mcpsPayload = (uint32_t)(emaSignal * 1000.0) & 0xFFFFFF;
        uint32_t healthPayload = ((uint32_t)bestStatus << 24) | mcpsPayload;
        sensorHealth.store(healthPayload, std::memory_order_release);

        // V17b: ambient rate and range sigma from best object
        float bestAmbient = 0.0f;
        for (int i = 0; i < multiRangingData.NumberOfObjectsFound; i++) {
          if (multiRangingData.RangeData[i].RangeStatus == 0) {
            bestAmbient = multiRangingData.RangeData[i].AmbientRateRtnMegaCps / 65536.0f;
            // RangeSigmaMilliMeter and ReflectanceEstPercent not available in this driver version
            break;
          }
        }
        sensorAmbient.store((uint32_t)(bestAmbient * 1000.0f) & 0xFFFFFF, std::memory_order_release);
        // sensorSigma fields unavailable in this driver version — store 0

        if (validSignalSum > 0.0) {
          int currentRange = round(weightedDistanceSum / validSignalSum);
          if (emaDistance < 0) emaDistance = currentRange;
          else emaDistance = (EMA_ALPHA * currentRange) + ((1.0 - EMA_ALPHA) * emaDistance);
          
          uint32_t distPayload = (uint32_t)round(emaDistance) & 0x7FFFFFFF;
          uint32_t statePayload = (1UL << 31) | distPayload;
          sensorState.store(statePayload, std::memory_order_release);
          // Sub-mm companion for the calibration sampler: tenths of a mm, valid
          // bit set. Capped at 30 bits (~10 cm worth of tenths headroom over the
          // sensor's range) so the valid bit never collides with the payload.
          uint32_t tenths = (uint32_t)roundf(emaDistance * 10.0f) & 0x7FFFFFFF;
          sensorDistTenths.store((1UL << 31) | tenths, std::memory_order_release);
        } else {
          sensorState.store(0, std::memory_order_release);
          sensorDistTenths.store(0, std::memory_order_release);
        }
      } else {
        xSemaphoreGive(i2cMutex); 
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

// ─── V12: LSM6DSOX vibration monitor ────────────────────────────────────────
// Phase 0 — sensor bring-up. The Adafruit driver handles reset + WHO_AM_I;
// FIFO setup and draining are done with direct register I/O (below) so they
// work regardless of the library's FIFO support. vibTask is pinned to Core 0
// at priority 1 — below sensorTask (2) so VL53 ranging is never delayed, and
// below the AsyncTCP task so telemetry is never starved.
//
// LSM6DSOX register map (FIFO subset only):
#define LSM6DS_REG_FIFO_CTRL3       0x09  // BDR_GY[7:4] | BDR_XL[3:0]
#define LSM6DS_REG_FIFO_CTRL4       0x0A  // ... | FIFO_MODE[2:0]
#define LSM6DS_REG_FIFO_STATUS1     0x3A  // DIFF_FIFO[7:0]
#define LSM6DS_REG_FIFO_DATA_OUT_TAG 0x78 // tag byte, then X/Y/Z (6 bytes)
#define LSM6DS_FIFO_TAG_ACCEL       0x02  // TAG_SENSOR field value for accel NC

// All three helpers below assume the caller already holds i2cMutex.
static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  bool ok = (Wire.endTransmission() == 0);
  if (!ok) i2cErrCount.fetch_add(1, std::memory_order_relaxed);   // #6 bus-health
  return ok;
}
static uint8_t imuReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {          // repeated start failed
    i2cErrCount.fetch_add(1, std::memory_order_relaxed);
    return 0;
  }
  uint8_t got = Wire.requestFrom(imuAddr, len);
  for (uint8_t i = 0; i < got && i < len; i++) buf[i] = Wire.read();
  if (got != len) i2cErrCount.fetch_add(1, std::memory_order_relaxed);   // partial read
  return got;
}

// imuSetup() — called from setup() with i2cMutex held, after the VL53L4CX is
// started. Probes both I²C addresses, configures accel-only ODR 416 Hz / ±2 g,
// and arms the hardware FIFO in continuous mode batching accel at 416 Hz.
static void imuSetup() {
  if      (lsm.begin_I2C(0x6A, &Wire)) imuAddr = 0x6A;
  else if (lsm.begin_I2C(0x6B, &Wire)) imuAddr = 0x6B;
  else { Serial.println("LSM6DSOX not found (tried 0x6A / 0x6B)"); imuPresent = false; return; }
  imuPresent = true;

  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);   // ±2 g — see VIB_MG_PER_LSB
  lsm.setAccelDataRate(LSM6DS_RATE_416_HZ);
  lsm.setGyroDataRate(LSM6DS_RATE_SHUTDOWN);   // accel-only — saves FIFO + power

  // The accel digital filter takes ~20 ms to settle after an ODR change;
  // samples produced during that window are filter-step-response garbage.
  // Wait it out before any sample reaches the FIFO so dcG can't be primed
  // to a transient value (and a one-shot 50+ mg RMS surface to the UI).
  delay(30);

  // FIFO_CTRL3: BDR_XL = 0110b (416 Hz), BDR_GY = 0 (gyro not batched).
  imuWriteReg(LSM6DS_REG_FIFO_CTRL3, 0x06);
  // FIFO_CTRL4: Bypass (000b) first — flushes any samples that landed in
  // the FIFO during begin_I2C / ODR setup — then Continuous (110b).
  imuWriteReg(LSM6DS_REG_FIFO_CTRL4, 0x00);
  delayMicroseconds(200);
  imuWriteReg(LSM6DS_REG_FIFO_CTRL4, 0x06);

  Serial.printf("LSM6DSOX online at 0x%02X — FIFO continuous, 416 Hz / +-2 g\n", imuAddr);
}

// vibBegin() — allocate the PSRAM buffers and build the Hann window. Mirrors
// PSRAMCanvas16::begin(): ps_malloc() with a DRAM fallback. Call from setup()
// once PSRAM is up and before vibTask is created.
void vibBegin() {
  size_t rawBytes = (size_t)VIB_RAW_LEN * sizeof(int16_t);
  size_t wfBytes  = (size_t)VIB_WF_W * VIB_WF_H * sizeof(uint16_t);
  vibRaw = (int16_t*)ps_malloc(rawBytes);
  if (!vibRaw) vibRaw = (int16_t*)malloc(rawBytes);
  vibWF = (uint16_t*)ps_malloc(wfBytes);
  if (!vibWF) vibWF = (uint16_t*)malloc(wfBytes);
  if (vibRaw) memset(vibRaw, 0, rawBytes);
  if (vibWF)  memset(vibWF, 0, wfBytes);

  // Hann window: w[n] = 0.5 * (1 - cos(2*pi*n / (N-1)))
  for (int n = 0; n < VIB_FFT_SIZE; n++)
    vibHann[n] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * n / (VIB_FFT_SIZE - 1)));

  vibReady = (vibRaw != nullptr && vibWF != nullptr);
  if (!vibReady) allocFailCount.fetch_add(1, std::memory_order_relaxed);   // #5 vib buffers
  Serial.printf("[vib] vibBegin: raw=%s waterfall=%s\n",
                vibRaw ? "ok" : "FAIL", vibWF ? "ok" : "FAIL");
}

// ── Phase 2 settle-time DSP helpers (all run inside vibTask, Core 0) ──

// Goertzel magnitude of one VIB_BLOCK-sample block at the locked resonance.
static float vibGoertzelBlock(uint32_t startPos, float coeff) {
  float s1 = 0.0f, s2 = 0.0f;
  for (int i = 0; i < VIB_BLOCK; i++) {
    float x  = (float)vibRaw[(startPos + i) % VIB_RAW_LEN];
    float s0 = x + coeff * s1 - s2;
    s2 = s1; s1 = s0;
  }
  float power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
  return (power > 0.0f) ? sqrtf(power) / VIB_BLOCK : 0.0f;
}

// Analyse a window of the raw ring: build the decay envelope, fit τ, derive a
// settle time, publish the results and append to the per-stack log. `startPos`
// is the ring index of the oldest sample; the window is VIB_SETTLE_SAMPLES long.
static void vibAnalyzeSettle(uint32_t startPos) {
  const float coeff = 2.0f * cosf(2.0f * (float)M_PI * vibResonanceHz / VIB_SAMPLE_RATE);
  int   peakIdx = 0;
  float peakVal = 0.0f;
  for (int b = 0; b < VIB_SETTLE_BLOCKS; b++) {
    float e = vibGoertzelBlock((startPos + (uint32_t)b * VIB_BLOCK) % VIB_RAW_LEN, coeff);
    vibEnvPlot[b] = e;
    if (e > peakVal) { peakVal = e; peakIdx = b; }
  }
  vibEnvPeak = peakVal;

  const float blockMs = VIB_BLOCK * 1000.0f / VIB_SAMPLE_RATE;   // ≈ 76.9 ms
  const float floorV  = peakVal * 0.05f + 1.0f;
  float thresh = peakVal * 0.12f;
  if (thresh < floorV) thresh = floorV;

  // Settle block = first block past the peak that falls below the threshold.
  int settleBlock = -1;
  for (int b = peakIdx + 1; b < VIB_SETTLE_BLOCKS; b++) {
    if (vibEnvPlot[b] < thresh) { settleBlock = b; break; }
  }
  // Exponential-decay fit: log-linear regression of ln(env) from the peak on.
  int n = 0; float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (int b = peakIdx; b < VIB_SETTLE_BLOCKS; b++) {
    if (vibEnvPlot[b] > floorV) {
      float x = (float)(b - peakIdx), y = logf(vibEnvPlot[b]);
      sx += x; sy += y; sxx += x * x; sxy += x * y; n++;
    }
  }
  uint32_t tauMs = 0;
  if (n >= 3) {
    float denom = n * sxx - sx * sx;
    if (fabsf(denom) > 1e-6f) {
      float slope = (n * sxy - sx * sy) / denom;     // d ln(env) / d block
      if (slope < -1e-4f) tauMs = (uint32_t)(-1.0f / slope * blockMs);
    }
  }
  // Never fell below threshold — including a peak sitting in the last block
  // (rig still moving hard at the shutter, i.e. WAIT far too short). The old
  // code defaulted settleBlock to the last block, which made that exact case
  // read 0 ms and drag the suggested-wait p95 DOWN when it should read max.
  // Report the full window as a conservative floor instead.
  uint32_t settleMs = (settleBlock >= 0)
      ? (uint32_t)((settleBlock - peakIdx) * blockMs)
      : (uint32_t)(VIB_SETTLE_BLOCKS * blockMs);
  vibSettleMs.store(settleMs);
  vibSettleTauMs.store(tauMs);
  vibSettleSeq.fetch_add(1);
  if (vibSettleLogCount < VIB_SETTLE_LOG_MAX)
    vibSettleLog[vibSettleLogCount++] = (settleMs > 65535) ? 65535 : (uint16_t)settleMs;
}

// 95th-percentile of the per-stack settle log → suggested wait (+15% margin).
static void vibComputeAggregate() {
  if (vibSettleLogCount <= 0) return;
  static uint16_t tmp[VIB_SETTLE_LOG_MAX];
  int cnt = vibSettleLogCount;
  memcpy(tmp, vibSettleLog, (size_t)cnt * sizeof(uint16_t));
  for (int i = 1; i < cnt; i++) {                    // insertion sort (cnt ≤ 400)
    uint16_t v = tmp[i]; int j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  uint32_t p95 = tmp[(int)(0.95f * (cnt - 1))];
  vibSuggestedWait.store((uint32_t)(p95 * 1.15f));
}

// vibTask — Core 0, priority 1. Drains the accel FIFO into the raw ring; every
// VIB_HOP fresh samples it runs a Hann-windowed 512-pt FFT and publishes the
// magnitude spectrum (ping-pong), the dominant frequency and a band RMS. It
// also runs the settle-time analysis on each post-pulse window.
void vibTask(void *pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(30);
  float dcG[3]      = { 0, 0, 0 };  // EMA of the gravity VECTOR, LSB (primed on 1st sample)
  bool  dcPrimed    = false;
  float vibE1[3] = { 0, 0, 0 };     // orthonormal basis for the horizontal plane,
  float vibE2[3] = { 0, 0, 0 };     // perpendicular to gravity; only refreshed
                                    // when the gravity vector has actually moved
                                    // (rig physically retilted). Continuous 30 ms
                                    // refresh injected micro-rotations into hpx/hpy
                                    // across the FFT window and dithered the gz
                                    // > 0.9 reference-axis switch — both surfaced
                                    // as spurious peaks on the horizontal trace.
  float dcGRef[3] = { 0, 0, 0 };    // dcG snapshot the current basis was derived from
  bool  basisInited = false;
  bool  basisGzNorth = false;       // reference-axis pole the current basis used —
                                    // hysteresis breaks the |gz|=0.9 flip-flop
  // Cosine of the "rig has tilted enough to redo the basis" angle. Below ~2°
  // of motion the projection error is small (sin 2° ≈ 0.035) and refreshing
  // every pass costs more (in basis-discontinuity sidebands) than it earns.
  const float VIB_BASIS_REFRESH_COS = 0.9994f;   // ≈ 2°
  uint32_t newSamples = 0;         // samples drained since the last FFT
  uint32_t specSeq    = 0;
  uint32_t lastPrint  = millis();
  // Narrowband-tonal tracker — see VIB_TONAL_* constants. Holds the previous
  // FFT frame's dominant bin and a run counter of consecutive frames where
  // the dom bin stayed within +-VIB_TONAL_BIN_TOL AND its peak amplitude
  // crossed VIB_TONAL_PEAK_MG. Resets to 0 the moment either condition fails.
  uint32_t tonalLastBin = 0;
  uint8_t  tonalRun     = 0;
  uint32_t lastPulseSeen = vibPulseSeq.load();
  uint32_t lastPulseMs   = 0;      // millis() when the last shutter pulse was seen
  uint32_t lastStartSeen = vibStackStartSeq.load();
  uint32_t lastDoneSeen  = vibStackDoneSeq.load();
  uint32_t lastAbortSeen = vibStackAbortSeq.load();
  // Per-stack V/H blur accumulators (stage-µm, shutter-weighted). Reset on
  // stack start, published to vibStackBlur{V,H}{Avg,Min,Max} after every FFT
  // hop so wifiNotifyStackComplete() can read current values without a race.
  // Mirrors the web analyzer's broadband-blur formula bin-for-bin so the PNG
  // card's V/H pixel-blur lines up with what the analyzer panel shows live.
  bool     inStack       = false;
  float    stackBlurMinV = 0.0f, stackBlurMaxV = 0.0f, stackBlurSumV = 0.0f;
  float    stackBlurMinH = 0.0f, stackBlurMaxH = 0.0f, stackBlurSumH = 0.0f;
  uint32_t stackBlurN    = 0;
  // Per-hop EMA coefficient for vibBaseV/H. Matches the web analyzer's
  // τ = 8 s: α = 1 - exp(-T_hop / τ), T_hop = VIB_HOP / VIB_SAMPLE_RATE.
  const float VIB_BASE_ALPHA =
      1.0f - expf(-((float)VIB_HOP / VIB_SAMPLE_RATE) / 8.0f);
  bool imuPoweredDown = false;     // tracks the auto-off power state (this task only)

  for (;;) {
    vTaskDelay(period);
    if (!imuPresent || !vibReady) continue;

    // ── Auto power-off (1 h idle) ──
    // loop() sets vibAutoOff; we own every LSM6DSOX register write so the
    // transition happens here, on this core. Power the accel down (ODR off,
    // FIFO bypassed) while off; re-arm exactly like imuSetup() on wake, with
    // the 30 ms filter-settle delay taken with the I²C mutex released.
    bool wantOff = vibAutoOff.load(std::memory_order_acquire);
    if (wantOff && !imuPoweredDown) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50))) {
        imuWriteReg(LSM6DS_REG_FIFO_CTRL4, 0x00);     // FIFO → bypass (stop batching)
        lsm.setAccelDataRate(LSM6DS_RATE_SHUTDOWN);   // accelerometer power-down
        xSemaphoreGive(i2cMutex);
      }
      imuPoweredDown = true;
      vibBandRms.store(0, std::memory_order_relaxed);     // publish "off" to UI/telemetry
      vibHorizRms.store(0, std::memory_order_relaxed);
      vibDominant.store(0, std::memory_order_relaxed);
      vibState.store(0, std::memory_order_relaxed);
    } else if (!wantOff && imuPoweredDown) {
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50))) {
        lsm.setAccelDataRate(LSM6DS_RATE_416_HZ);
        xSemaphoreGive(i2cMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(30));                  // filter settle, mutex released
      if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50))) {
        imuWriteReg(LSM6DS_REG_FIFO_CTRL4, 0x00);     // bypass → flush stale samples
        delayMicroseconds(200);
        imuWriteReg(LSM6DS_REG_FIFO_CTRL4, 0x06);     // back to continuous
        xSemaphoreGive(i2cMutex);
      }
      imuPoweredDown = false;
      newSamples = 0; dcPrimed = false; basisInited = false;   // re-prime cleanly
    }
    if (imuPoweredDown) continue;

    // ── Refresh the horizontal-plane basis from the current gravity vector ──
    // Combined horizontal power is basis-invariant, so any in-plane (e1,e2)
    // pair works — they need only be orthonormal and perpendicular to gravity.
    // We refresh only when the rig has actually tilted (cos of dcG vs the
    // basis-snapshot dcG drops below VIB_BASIS_REFRESH_COS); a stationary rig
    // gets one stable basis for the whole session, so the 512-sample FFT
    // window contains hx/hy samples all written under the same basis.
    {
      float gm = sqrtf(dcG[0]*dcG[0] + dcG[1]*dcG[1] + dcG[2]*dcG[2]);
      bool needRefresh = !basisInited;
      if (!needRefresh && dcPrimed && gm > 1.0f) {
        float rm = sqrtf(dcGRef[0]*dcGRef[0] + dcGRef[1]*dcGRef[1] + dcGRef[2]*dcGRef[2]);
        if (rm > 1.0f) {
          float cosT = (dcG[0]*dcGRef[0] + dcG[1]*dcGRef[1] + dcG[2]*dcGRef[2]) / (gm * rm);
          if (cosT < VIB_BASIS_REFRESH_COS) needRefresh = true;
        } else {
          needRefresh = true;
        }
      }
      if (needRefresh && dcPrimed && gm > 1.0f) {
        float gx = dcG[0]/gm, gy = dcG[1]/gm, gz = dcG[2]/gm;
        // Reference-axis hysteresis: once we've picked "X-pole" (because |gz|
        // > 0.9), stay on it until |gz| drops below 0.85 — otherwise micro-
        // dither in dcG around |gz|=0.9 flips the basis ±90° between FFTs
        // and prints a comb of harmonics into the horizontal spectrum.
        bool useGzNorth = basisGzNorth ? (fabsf(gz) > 0.85f) : (fabsf(gz) > 0.9f);
        basisGzNorth = useGzNorth;
        float rx = 0, ry = 0, rz = 1;
        if (useGzNorth) { rx = 1; rz = 0; }
        float e1x = gy*rz - gz*ry, e1y = gz*rx - gx*rz, e1z = gx*ry - gy*rx;
        float e1n = sqrtf(e1x*e1x + e1y*e1y + e1z*e1z);
        if (e1n < 1e-6f) e1n = 1.0f;
        vibE1[0] = e1x/e1n;  vibE1[1] = e1y/e1n;  vibE1[2] = e1z/e1n;
        vibE2[0] = gy*vibE1[2] - gz*vibE1[1];          // e2 = g x e1 (unit)
        vibE2[1] = gz*vibE1[0] - gx*vibE1[2];
        vibE2[2] = gx*vibE1[1] - gy*vibE1[0];
        dcGRef[0] = dcG[0]; dcGRef[1] = dcG[1]; dcGRef[2] = dcG[2];
        basisInited = true;
      }
    }

    // ── Drain the FIFO into the raw ring (vertical AC, LSB) ──
    // Drained in bounded chunks: i2cMutex is released and re-taken every
    // FIFO_CHUNK words so a large backlog can't monopolise the I²C bus and
    // stall VL53 ranging — sensorTask (higher priority) preempts and grabs the
    // mutex between chunks. Normal load (~13 words) fits in a single chunk.
    {
      const int FIFO_CHUNK = 16;     // words per mutex hold (~3 ms at 400 kHz)
      int  remaining = -1;           // -1 = FIFO_STATUS not yet read this pass
      bool stop = false;
      while (!stop && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(15)) == pdTRUE) {
        if (remaining < 0) {
          uint8_t st[2];
          remaining = (imuReadRegs(LSM6DS_REG_FIFO_STATUS1, st, 2) == 2)
                    ? (((uint16_t)(st[1] & 0x03) << 8) | st[0])   // DIFF_FIFO
                    : 0;
        }
        int chunk = (remaining < FIFO_CHUNK) ? remaining : FIFO_CHUNK;
        for (int k = 0; k < chunk; k++) {
          uint8_t w[7];
          if (imuReadRegs(LSM6DS_REG_FIFO_DATA_OUT_TAG, w, 7) != 7) { stop = true; break; }
          if ((w[0] >> 3) != LSM6DS_FIFO_TAG_ACCEL) continue;
          float rx = (int16_t)((w[2] << 8) | w[1]);
          float ry = (int16_t)((w[4] << 8) | w[3]);
          float rz = (int16_t)((w[6] << 8) | w[5]);

          // Slow per-axis gravity tracker — same 0.002 EMA as the old scalar
          // dcMag, but a 3-vector so we know which way "down" points.
          if (!dcPrimed) { dcG[0] = rx; dcG[1] = ry; dcG[2] = rz; dcPrimed = true; }
          dcG[0] += (rx - dcG[0]) * 0.002f;
          dcG[1] += (ry - dcG[1]) * 0.002f;
          dcG[2] += (rz - dcG[2]) * 0.002f;

          // AC = sample minus gravity. Project out the component ALONG gravity
          // (vertical). Gravity is projected out *first*, so neither channel
          // carries the sqrt(g²+δ²) squaring artifact — horizontal is linear
          // and at its true pitch. The horizontal RMS is taken as a boxcar in
          // the FFT block below, not an EMA here, so V and H match.
          float ax = rx - dcG[0], ay = ry - dcG[1], az = rz - dcG[2];
          float gmag = sqrtf(dcG[0]*dcG[0] + dcG[1]*dcG[1] + dcG[2]*dcG[2]);
          if (gmag < 1.0f) gmag = 1.0f;
          float vert   = (ax*dcG[0] + ay*dcG[1] + az*dcG[2]) / gmag;   // LSB, signed

          // Horizontal vector = AC minus its gravity-aligned part; project it
          // onto the (e1,e2) plane basis and ring-buffer both components for
          // the combined-horizontal FFT.
          float ig  = 1.0f / gmag;
          float hvx = ax - vert*dcG[0]*ig;
          float hvy = ay - vert*dcG[1]*ig;
          float hvz = az - vert*dcG[2]*ig;
          int32_t hpx = (int32_t)lroundf(hvx*vibE1[0] + hvy*vibE1[1] + hvz*vibE1[2]);
          int32_t hpy = (int32_t)lroundf(hvx*vibE2[0] + hvy*vibE2[1] + hvz*vibE2[2]);
          if (hpx >  32767) hpx =  32767; else if (hpx < -32768) hpx = -32768;
          if (hpy >  32767) hpy =  32767; else if (hpy < -32768) hpy = -32768;
          vibHx[vibHxHead] = (int16_t)hpx;
          vibHy[vibHxHead] = (int16_t)hpy;
          vibHxHead = (vibHxHead + 1) % VIB_FFT_SIZE;

          // The raw ring now carries the VERTICAL AC — the FFT, band-RMS,
          // dominant-freq and settle code downstream is unchanged and now
          // sees a cleaner, linear, correctly-pitched signal.
          int32_t ac = (int32_t)lroundf(vert);
          if (ac >  32767) ac =  32767;
          if (ac < -32768) ac = -32768;
          vibRaw[vibRawHead] = (int16_t)ac;
          vibRawHead = (vibRawHead + 1) % VIB_RAW_LEN;
          newSamples++;
        }
        xSemaphoreGive(i2cMutex);
        remaining -= chunk;
        if (remaining <= 0) stop = true;
      }
    }

    // ── Run an FFT once enough fresh samples have arrived ──
    if (newSamples >= VIB_HOP) {
      newSamples = 0;
      // Copy the most recent VIB_FFT_SIZE samples, oldest-first.
      uint32_t start = (vibRawHead + VIB_RAW_LEN - VIB_FFT_SIZE) % VIB_RAW_LEN;
      float mean = 0.0f;
      for (int i = 0; i < VIB_FFT_SIZE; i++)
        mean += vibRaw[(start + i) % VIB_RAW_LEN];
      mean /= VIB_FFT_SIZE;
      double sumSq = 0.0;
      for (int i = 0; i < VIB_FFT_SIZE; i++) {
        float s = (float)vibRaw[(start + i) % VIB_RAW_LEN] - mean;  // DC-removed, LSB
        sumSq += (double)s * s;
        vibFftReal[i] = s * vibHann[i];
        vibFftImag[i] = 0.0f;
      }
      vibFFT.compute(FFTDirection::Forward);
      // #2: the input is real, so the FFT is Hermitian-symmetric — bins
      // 256..511 are conjugate mirrors of 1..255 and carry no new information.
      // complexToMagnitude() would sqrt all 512 and we'd read only the lower
      // half. Compute magnitude inline for the 256 usable bins instead (matches
      // the horizontal channel's manual per-bin magnitude below).

      // Time-domain window RMS → band RMS in mg (VIB_MG_PER_LSB at ±2 g).
      float rmsMg = sqrtf((float)(sumSq / VIB_FFT_SIZE)) * VIB_MG_PER_LSB;

      // Publish the spectrum into the idle ping-pong buffer (deci-mg).
      // Hann coherent gain 0.5, real-FFT 2/N → amplitude ~ bin * 4/N.
      specSeq++;
      uint16_t *dst = vibSpec[specSeq & 1];
      const float binScale = VIB_MG_PER_LSB * 4.0f / VIB_FFT_SIZE * 10.0f;  // LSB-mag → deci-mg
      uint32_t domBin = 0; float domMag = 0.0f;
      for (int b = 0; b < VIB_FFT_BINS; b++) {
        float mag = sqrtf(vibFftReal[b]*vibFftReal[b] + vibFftImag[b]*vibFftImag[b]);
        float m = mag * binScale;
        if (m > 65535.0f) m = 65535.0f;
        dst[b] = (uint16_t)m;
        if (b >= 4 && mag > domMag) { domMag = mag; domBin = b; }
      }

      // ── Combined-horizontal spectrum — one complex-packed FFT: hx into the
      //    real input, hy into the imaginary input. The horizontal power at
      //    bin k is (|X[k]|^2 + |X[N-k]|^2)/2 (a Hermitian-symmetry identity),
      //    which is basis-invariant — both axes, one FFT, no rectification.
      double sumSqH = 0.0;       // mean-removed horizontal AC energy over the window
      {
        uint32_t hs0 = vibHxHead;            // oldest sample (ring == one window)
        // Mirror the vertical pipeline: subtract the segment mean of each
        // channel before Hann. The slow gravity tracker (α = 0.002) can leave
        // a residual offset in the projected horizontal stream during strong
        // shake — without this subtraction the offset gets windowed and
        // leaks energy across the spectrum, which is what produces the
        // "forest of spikes" on the H trace at MODERATE/STRONG levels.
        double sumX = 0.0, sumY = 0.0;
        for (int i = 0; i < VIB_FFT_SIZE; i++) {
          uint32_t idx = (hs0 + i) % VIB_FFT_SIZE;
          sumX += vibHx[idx];
          sumY += vibHy[idx];
        }
        const float meanX = (float)(sumX / VIB_FFT_SIZE);
        const float meanY = (float)(sumY / VIB_FFT_SIZE);
        for (int i = 0; i < VIB_FFT_SIZE; i++) {
          uint32_t idx = (hs0 + i) % VIB_FFT_SIZE;
          float xv = (float)vibHx[idx] - meanX;
          float yv = (float)vibHy[idx] - meanY;
          sumSqH += (double)xv*xv + (double)yv*yv;   // un-windowed — feeds the RMS
          vibFftReal[i] = xv * vibHann[i];
          vibFftImag[i] = yv * vibHann[i];
        }
        vibFFT.compute(FFTDirection::Forward);
        uint16_t *dstH = vibSpecH[specSeq & 1];
        for (int b = 0; b < VIB_FFT_BINS; b++) {
          int bn = (b == 0) ? 0 : (VIB_FFT_SIZE - b);
          float p = 0.5f * ((vibFftReal[b]*vibFftReal[b]   + vibFftImag[b]*vibFftImag[b]) +
                            (vibFftReal[bn]*vibFftReal[bn] + vibFftImag[bn]*vibFftImag[bn]));
          float m = sqrtf(p) * binScale;
          if (m > 65535.0f) m = 65535.0f;
          dstH[b] = (uint16_t)m;
        }
      }

      // ── Noise-floor EMA + shutter-weighted V/H blur ───────────────────────
      // The web analyzer subtracts a per-bin EMA noise floor before integrating
      // bins to displacement; this mirror keeps the PNG card's V/H blur lines
      // numerically equal to what the analyzer panel shows live.
      //
      // The floor only advances when !inStack so it can't be inflated by the
      // stack's own motion. When inStack each bin is converted to
      //   d_amp = max(0, dst[b] - vibBaseV/H[b]) × (deci-mg→m/s²) / ω²
      //   bl     = 2 · d_amp · |sin(πfT)|        (T = 1/vibShutterDenom)
      // and summed in quadrature; blur_µm = √(Σbl² / 2) × 1e6 — the same
      // amplitude-to-sinusoid-RMS conversion vibDispStats() uses in JS.
      {
        uint16_t *dstH = vibSpecH[specSeq & 1];
        if (!inStack) {
          if (!vibBasePrimed) {
            for (int b = 0; b < VIB_FFT_BINS; b++) {
              vibBaseV[b] = (float)dst[b];
              vibBaseH[b] = (float)dstH[b];
            }
            vibBasePrimed = true;
          } else {
            for (int b = 0; b < VIB_FFT_BINS; b++) {
              vibBaseV[b] += ((float)dst[b]  - vibBaseV[b]) * VIB_BASE_ALPHA;
              vibBaseH[b] += ((float)dstH[b] - vibBaseH[b]) * VIB_BASE_ALPHA;
            }
          }
        } else if ((uint32_t)(millis() - lastPulseMs) <= VIB_BLUR_GATE_MS) {
          // In-stack, and a shutter pulse fired within the gate window — this
          // hop covers an exposure / its ring-down, so it counts toward the
          // per-stack stats. Hops in long inter-shot gaps and the 5 s
          // stack-done silence tail are excluded (see VIB_BLUR_GATE_MS).
          const float T   = 1.0f / (float)vibShutterDenom;
          const float k_a = 9.81e-3f / 10.0f;   // deci-mg → m/s²
          double blurSqV  = 0.0;
          double blurSqH  = 0.0;
          for (int b = VIB_DISP_MIN_BIN; b < VIB_FFT_BINS; b++) {
            const float fb      = b * VIB_BIN_HZ;
            const float omega   = 2.0f * (float)M_PI * fb;
            const float invW2   = 1.0f / (omega * omega);
            const float sinW    = fabsf(sinf((float)M_PI * fb * T));
            float aV = (float)dst[b]  - vibBaseV[b]; if (aV < 0) aV = 0;
            float aH = (float)dstH[b] - vibBaseH[b]; if (aH < 0) aH = 0;
            const float dV = (aV * k_a) * invW2;     // m amplitude
            const float dH = (aH * k_a) * invW2;
            const float blV = 2.0f * dV * sinW;
            const float blH = 2.0f * dH * sinW;
            blurSqV += (double)blV * blV;
            blurSqH += (double)blH * blH;
          }
          // sinusoid-amplitude → RMS: √(Σbl² / 2). Result × 1e6 → µm.
          const float blurVum = sqrtf((float)(blurSqV * 0.5)) * 1e6f;
          const float blurHum = sqrtf((float)(blurSqH * 0.5)) * 1e6f;
          if (stackBlurN == 0) {
            stackBlurMinV = stackBlurMaxV = blurVum;
            stackBlurMinH = stackBlurMaxH = blurHum;
          } else {
            if (blurVum < stackBlurMinV) stackBlurMinV = blurVum;
            if (blurVum > stackBlurMaxV) stackBlurMaxV = blurVum;
            if (blurHum < stackBlurMinH) stackBlurMinH = blurHum;
            if (blurHum > stackBlurMaxH) stackBlurMaxH = blurHum;
          }
          stackBlurSumV += blurVum;
          stackBlurSumH += blurHum;
          stackBlurN++;
          const float avgV = stackBlurSumV / stackBlurN;
          const float avgH = stackBlurSumH / stackBlurN;
          vibStackBlurVAvg.store((uint32_t)lroundf(avgV          * 100.0f));
          vibStackBlurVMin.store((uint32_t)lroundf(stackBlurMinV * 100.0f));
          vibStackBlurVMax.store((uint32_t)lroundf(stackBlurMaxV * 100.0f));
          vibStackBlurHAvg.store((uint32_t)lroundf(avgH          * 100.0f));
          vibStackBlurHMin.store((uint32_t)lroundf(stackBlurMinH * 100.0f));
          vibStackBlurHMax.store((uint32_t)lroundf(stackBlurMaxH * 100.0f));
        }
      }

      vibSpecSeq.store(specSeq);

      // Cross-core scalars.
      vibDominant.store((uint32_t)lroundf(domBin * VIB_BIN_HZ * 10.0f));
      vibBandRms.store((uint32_t)lroundf(rmsMg * 100.0f));
      // Horizontal-plane RMS — a boxcar over the same VIB_FFT_SIZE window as
      // the vertical rmsMg, so the two are directly comparable for vibState.
      float horizMg = sqrtf((float)(sumSqH / VIB_FFT_SIZE)) * VIB_MG_PER_LSB;
      vibHorizRms.store((uint32_t)lroundf(horizMg * 100.0f));

      // "Is the bench quiet" reflects whichever axis is worse — a horizontal
      // mess vibrates a 1-2 um rig just as much as a vertical one.
      float worstMg = (horizMg > rmsMg) ? horizMg : rmsMg;
      uint32_t st = (worstMg >= VIB_RMS_STRONG)   ? 2 :
                    (worstMg >= VIB_RMS_MODERATE) ? 1 : 0;

      // Narrowband-tonal promotion: dst[domBin] is the FFT peak amplitude at
      // the dominant bin in deci-mg (binScale already folds Hann's 0.5
      // coherent gain into the 4/N real-FFT factor). A constant ~26 Hz dryer
      // shows ~25-75 deci-mg here while the time-domain RMS sits well below
      // VIB_RMS_MODERATE — without this branch the bench would read CALM.
      float peakMg = dst[domBin] / 10.0f;
      int32_t binDelta = (int32_t)domBin - (int32_t)tonalLastBin;
      if (binDelta < 0) binDelta = -binDelta;
      if (peakMg >= VIB_TONAL_PEAK_MG && binDelta <= VIB_TONAL_BIN_TOL) {
        if (tonalRun < 0xFF) tonalRun++;
      } else {
        tonalRun = 0;
      }
      tonalLastBin = domBin;
      if (st == 0 && tonalRun >= VIB_TONAL_FRAMES) st = 1;
      vibState.store(st);
    }

    // ── Settle-time analysis: on each shutter pulse, analyse the window of
    //    ring buffer ending at the pulse — it already holds the motor move +
    //    the full hold/wait settle, so analysis is immediate (no waiting).
    // Acquire pairs with the producer's release in loop()/RUN TEST so the
    // vibPulseRingPos write that precedes the seq bump is visible here.
    uint32_t ps = vibPulseSeq.load(std::memory_order_acquire);
    if (ps != lastPulseSeen) {
      lastPulseSeen = ps;
      lastPulseMs   = millis();
      uint32_t startPos = (vibPulseRingPos + VIB_RAW_LEN - VIB_SETTLE_SAMPLES) % VIB_RAW_LEN;
      vibAnalyzeSettle(startPos);
    }
    uint32_t ss = vibStackStartSeq.load();
    if (ss != lastStartSeen) {
      lastStartSeen = ss;
      vibSettleLogCount = 0;
      inStack       = true;
      stackBlurMinV = stackBlurMaxV = stackBlurSumV = 0.0f;
      stackBlurMinH = stackBlurMaxH = stackBlurSumH = 0.0f;
      stackBlurN    = 0;
      // Zero the published atomics so a wifiNotifyStackComplete() that
      // somehow fires before the first FFT hop can't surface the previous
      // stack's values. The `vva > 0` gate on the wifi side then naturally
      // omits the vva/vvn/... keys until we have real data this stack.
      vibStackBlurVAvg.store(0); vibStackBlurVMin.store(0); vibStackBlurVMax.store(0);
      vibStackBlurHAvg.store(0); vibStackBlurHMin.store(0); vibStackBlurHMax.store(0);
    }
    uint32_t ds = vibStackDoneSeq.load();
    if (ds != lastDoneSeen) {
      lastDoneSeen = ds;
      inStack      = false;
      vibComputeAggregate();
      // Blur atomics are already current from the last hop publish;
      // no extra store needed here.
    }
    uint32_t as = vibStackAbortSeq.load();
    if (as != lastAbortSeen) {
      lastAbortSeen = as;
      // Short/aborted sequence: resume the noise-floor EMA but skip the
      // aggregate — a single shot's settle log isn't a stack's.
      inStack = false;
    }

    if (millis() - lastPrint >= 2000) {
      lastPrint = millis();
      Serial.printf("[vib] dom %.1f Hz  rms %.2f mg  state %u\n",
                    vibDominant.load() / 10.0f, vibBandRms.load() / 100.0f,
                    (unsigned)vibState.load());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000);

  // ─── V12.5: field-resilience early block (boot-loop rollback + diagnostics) ──
  // Runs FIRST, before wifiSetup()/peripherals, so a rollback restart wastes no
  // init. NVS is available this early (the core mounts it before setup()).
  {
    esp_reset_reason_t rr = esp_reset_reason();
    if (g_rtc.sentinel == RTC_RESIL_SENTINEL) {
      // current layout — nothing to init
    } else if (g_rtc.sentinel == RTC_RESIL_SENTINEL_V1) {
      // M1: warm reboot from the pre-.54 (24-byte) layout — the old fields
      // through pendingReasonCode are valid; only the appended stack/frag tail
      // is garbage. MIGRATE (don't wipe) so bootCount / rollbackAttempted /
      // pendingReasonCode carry across the version boundary — preserving the
      // anti-ping-pong guard and the AUTO-ROLLBACK ring marker on a rollback.
      g_rtc.lastMinStack   = 0;
      g_rtc.lastMaxAllocKB = 0;
      g_rtc.sentinel       = RTC_RESIL_SENTINEL;
    } else {
      // unknown sentinel = cold power-on (RTC RAM is garbage) → clean init
      memset(&g_rtc, 0, sizeof(g_rtc));
      g_rtc.sentinel = RTC_RESIL_SENTINEL;
    }
    // Count ONLY unexpected failures toward the boot-loop counter (see
    // diagIsCrash). A deliberate restart (OTA / forget-wifi / F3 self-heal, all
    // ESP_RST_SW) or a button/power boot resets it — so intentional reboots and
    // F3's self-healing can never accumulate into a spurious rollback.
    if (diagIsCrash(rr)) g_rtc.bootCount++;
    else                 g_rtc.bootCount = 0;

    // F1 — boot-loop auto-rollback. Three consecutive boots that never reached
    // 30 s of healthy uptime → flip to the previous firmware in the inactive
    // OTA slot (still intact thanks to dual-OTA). Guarded so we never brick:
    // only if the inactive slot holds a valid image (first byte 0xE9), and only
    // once per streak (rollbackAttempted) so two bad slots don't ping-pong.
    if (g_rtc.bootCount >= 3 && !g_rtc.rollbackAttempted) {
      const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
      uint8_t imgMagic = 0;
      if (next && esp_partition_read(next, 0, &imgMagic, 1) == ESP_OK && imgMagic == 0xE9) {
        Serial.printf("[BOOT] boot-loop detected (%u) — rolling back to %s\n",
                      (unsigned)g_rtc.bootCount, next->label);
        Serial.flush();
        g_rtc.rollbackAttempted = 1;
        g_rtc.pendingReasonCode = DIAG_REASON_ROLLBACK;   // labels the NEXT boot's ring entry
        if (esp_ota_set_boot_partition(next) == ESP_OK) {
          delay(200);
          ESP.restart();
        }
        // set_boot_partition failed — fall through and boot this image anyway.
        Serial.println("[BOOT] rollback set_boot_partition failed — booting current");
      } else {
        Serial.println("[BOOT] boot-loop detected but no valid rollback slot — booting current");
      }
      g_rtc.bootCount = 0;   // don't re-trigger every subsequent boot
    }

    // F2 — record why the LAST run ended. Prefer a synthetic reason our own code
    // stashed (rollback / sensor-WDT); else the hardware reset reason. Uptime +
    // min-heap come from the periodic snapshot loop() wrote during that run.
    uint8_t reason = g_rtc.pendingReasonCode ? (uint8_t)g_rtc.pendingReasonCode : (uint8_t)rr;
    const esp_partition_t* run = esp_ota_get_running_partition();
    DiagEntry e = {0};
    e.reason    = reason;
    e.bootSlot  = (run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1) ? 1 : 0;
    e.uptimeSec = g_rtc.lastUptimeSec;
    e.minHeap   = g_rtc.lastMinHeap;
    e.minStack  = (g_rtc.lastMinStack   > 0xFFFF) ? 0xFFFF : (uint16_t)g_rtc.lastMinStack;
    e.maxAllocKB= (g_rtc.lastMaxAllocKB > 0xFFFF) ? 0xFFFF : (uint16_t)g_rtc.lastMaxAllocKB;
    strncpy(e.version, BUILD_VERSION, sizeof(e.version) - 1);
    Serial.printf("[BOOT] reset reason: %s (last run up %us, minHeap %uKB)\n",
                  diagReasonStr(reason), (unsigned)e.uptimeSec, (unsigned)e.minHeap);
    Serial.flush();
    // Compact uptime for the TFT line: 45s / 12m / 8h. Reason + uptime only —
    // no min-heap suffix here so the longest label ("SENSOR-STALL WDT") still
    // fits the 240 px row without overlapping "Last:". Full detail (heap, slot,
    // version) is on the dashboard DIAGNOSTICS screen.
    char up[8];
    if      (e.uptimeSec < 60)    snprintf(up, sizeof(up), "%us", (unsigned)e.uptimeSec);
    else if (e.uptimeSec < 3600)  snprintf(up, sizeof(up), "%um", (unsigned)(e.uptimeSec / 60));
    else                          snprintf(up, sizeof(up), "%uh", (unsigned)(e.uptimeSec / 3600));
    snprintf(g_bootReasonLine, sizeof(g_bootReasonLine), "%s %s",
             diagReasonStr(reason), up);
    preferences.begin("diag", false);
    {   // append e to the fixed-length ring (inlined — see note by DiagEntry)
      uint8_t head  = preferences.getUChar("head", 0);
      uint8_t count = preferences.getUChar("count", 0);
      // Dedup a reboot LOOP: if the previous run also failed the same way without
      // reaching healthy uptime (< 30 s), don't wear flash writing an identical
      // entry every ~15 s. Records the first loop entry, then coalesces repeats.
      bool dup = false;
      if (count > 0 && e.uptimeSec < 30) {
        DiagEntry last;
        char lk[8]; snprintf(lk, sizeof(lk), "e%u", (head + DIAG_RING_LEN - 1) % DIAG_RING_LEN);
        if (preferences.getBytes(lk, &last, sizeof(last)) == sizeof(last) &&
            last.reason == e.reason) dup = true;
      }
      if (!dup) {
        char key[8]; snprintf(key, sizeof(key), "e%u", head % DIAG_RING_LEN);
        preferences.putBytes(key, &e, sizeof(DiagEntry));
        preferences.putUChar("head", (head + 1) % DIAG_RING_LEN);
        preferences.putUChar("count", min<uint8_t>(DIAG_RING_LEN, count + 1));
      }
    }
    preferences.end();

    // New session — clear the consumed synthetic reason and the stale snapshot.
    g_rtc.pendingReasonCode = 0;
    g_rtc.lastUptimeSec = 0;
    g_rtc.lastMinHeap   = 0;
    g_rtc.lastMinStack  = 0;
    g_rtc.lastMaxAllocKB = 0;
  }

  // Allocate sprite buffers now that PSRAM is fully initialised.
  // (Constructor is a no-op — see PSRAMCanvas16::begin() comment.)
  fovSprite.begin(); distSprite.begin(); menuSprite.begin();
  barSprite.begin(); valSprite.begin();  objSprite.begin();
  vibPlotSprite.begin();   // V12: vibration spectrum plot canvas

  if (psramFound()) {
    Serial.printf("[PSRAM] Found: %u KB total, %u KB free\n",
                  ESP.getPsramSize() / 1024, ESP.getFreePsram() / 1024);
  } else {
    Serial.println("[PSRAM] Not found — sprites will use DRAM");
  }
  Serial.flush();

  pinMode(LITE_PIN, OUTPUT);
  // V12.3: drive the backlight PWM at 20 kHz. analogWrite() defaults to ~1 kHz
  // on the ESP32 core, which is low enough to read as a faint flicker / "wavy"
  // shimmer on the bright button fills (worse at reduced brightness). 20 kHz is
  // well above flicker perception and still comfortably within the LEDC range
  // (20 kHz x 256 steps = 5.12 MHz). Set before the first write so the channel
  // comes up at the high frequency.
  analogWriteFrequency(LITE_PIN, 20000);
  analogWrite(LITE_PIN, Config::DEFAULT_BRIGHTNESS);
  Serial.println("[BOOT] tft.begin()..."); Serial.flush();
  tft.begin();
  tft.setSPISpeed(70000000);
  tft.setRotation(0);
  tft.fillScreen(THEME_BG);
  Serial.println("[BOOT] TFT init OK"); Serial.flush();

  Serial.printf("[BOOT] WiFi init — free heap before: %u\n", ESP.getFreeHeap());
  Serial.flush();
  wifiSetup();
  // mDNS is started by staConnectTask() after WL_CONNECTED — calling
  // MDNS.begin() here returned true but never registered anything because the
  // STA interface had no IP yet.
  Serial.printf("[BOOT] WiFi init done — free heap after: %u\n", ESP.getFreeHeap());
  Serial.flush();

  preferences.begin("calib", false);
  size_t sch = preferences.getBytes("settings", &settings, sizeof(CalibData));
  bool calibWasReset = false;   // V12.5 (F4): true if we fell back to factory
  // Require a full-length read so a struct extension that forgets to bump
  // CALIB_MAGIC can't slip through with the trailing fields uninitialised.
  if (sch == sizeof(CalibData) && settings.magic == CALIB_MAGIC) {
    CTRLX = settings.ctrlX; CTRLY = settings.ctrlY;
    sensorWidthPixels = settings.sensorWidth; demarcationDist = settings.demarcation;
    CALIB_ERROR = settings.calibError; 
    // V17: guard against uninitialised calibR2 in NVS (e.g. upgraded from V16
    // where the field didn't exist, or magic matched but struct was partially written).
    // Valid measured R² is always in [0, 1]. Values outside that range or near 1.0000
    // (the old hard-coded fallback) are replaced with the known factory default.
    // Use a tolerance rather than exact equality to catch values like 1.0000001f
    // from older builds with slightly different math.
    if (settings.calibR2 > 0.0f && settings.calibR2 <= 1.0f && fabsf(settings.calibR2 - 1.0f) > 1e-4f) {
      CALIB_R2 = settings.calibR2;
    } else {
      CALIB_R2 = 0.9991f;  // safe default
      settings.calibR2 = CALIB_R2;
    }

    isCustomCalib = (settings.isCustom != 0);

    // If a previously-saved "custom" cal is actually the default line (e.g. the
    // user loaded the default cal back before this check existed), demote it to
    // default so the badge reads DEFAULT. Mirrors finalizeCalibration(); runs
    // before the points-restore below so it falls through to the factory points.
    // Not re-persisted here (idempotent each boot) — the next finalize saves it.
    if (isCustomCalib &&
        fabsf(CTRLX - Config::DEFAULT_CTRL_X) < 0.02f &&
        fabsf(CTRLY - Config::DEFAULT_CTRL_Y) < 3.0f) {
      CTRLX = Config::DEFAULT_CTRL_X; CTRLY = Config::DEFAULT_CTRL_Y;
      CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; CALIB_R2 = 0.9991f;
      isCustomCalib = false;
    }

    // V11 fix: restore the persisted calibration scatter points so CAL_GRAPH
    // and the web graph can plot them after a power-cycle.
    if (isCustomCalib && settings.calPointsCaptured > 0 &&
        settings.calPointsCaptured <= 20) {
      nPoints        = constrain((int)settings.calNPoints,
                                 Config::MIN_CALIB_POINTS, Config::MAX_CALIB_POINTS);
      pointsCaptured = constrain((int)settings.calPointsCaptured, 0, 20);
      for (int i = 0; i < pointsCaptured; i++) {
        distPoints[i] = settings.calDistPoints[i];
        fovPoints[i]  = settings.calFovPoints[i];
      }
    }

    if (settings.brightness < 1 || settings.brightness > 255) settings.brightness = Config::DEFAULT_BRIGHTNESS;
    currentBrightness = settings.brightness;
    
    if (settings.stackStepSize >= 0.156f && settings.stackStepSize <= 3.593f) {
      int bestIdx = 0;
      float bestDiff = fabsf(settings.stackStepSize - STEP_TABLE[0]);
      for (int i = 1; i < STEP_TABLE_LEN; i++) {
        float d = fabsf(settings.stackStepSize - STEP_TABLE[i]);
        if (d < bestDiff) { bestDiff = d; bestIdx = i; }
      }
      stackStepIndex = bestIdx;
      stackStepSize = STEP_TABLE[bestIdx];
    }
    if (settings.stackTotalDepth >= 1.0f && settings.stackTotalDepth <= 50000.0f) {
      stackTotalImgs = (int)roundf(settings.stackTotalDepth);
    }
    if (settings.stackTimePerStep >= 0.1f && settings.stackTimePerStep <= 60.0f) {
      stackTimePerStep = settings.stackTimePerStep;
    }
    // V17b: LED settings
    ledEnabled = (settings.ledEnabled != 0);
    // ledDuty is uint8_t so >=0 is always true; clamp at the upper bound only.
    currentLedDuty = (settings.ledDuty <= 127) ? settings.ledDuty : 127;
  } else {
    settings.magic = CALIB_MAGIC;
    settings.ctrlX = Config::DEFAULT_CTRL_X; settings.ctrlY = Config::DEFAULT_CTRL_Y;
    settings.sensorWidth = Config::DEFAULT_SENSOR_WIDTH_PX; settings.demarcation = Config::DEFAULT_DEMARCATION_MM;
    settings.calibError = Config::DEFAULT_CALIB_ERROR; settings.brightness = Config::DEFAULT_BRIGHTNESS;
    settings.stackStepSize = stackStepSize;
    settings.stackTotalDepth = (float)stackTotalImgs;
    settings.stackTimePerStep = stackTimePerStep;
    settings.isCustom = 0;
    settings.ledEnabled = 1;
    settings.ledDuty = TRIGGER_LED_DUTY;
    settings._pad = 0;
    settings.calibR2 = 0.9991f;
    settings.calNPoints = 0;
    settings.calPointsCaptured = 0;
    currentBrightness = Config::DEFAULT_BRIGHTNESS;
    isCustomCalib = false;
    preferences.putBytes("settings", &settings, sizeof(CalibData));
    calibWasReset = true;   // V12.5 (F4): try to restore from calibbak below
  }
  preferences.end();

  // V11: Restore screen timeouts + color theme from their own NVS namespace.
  // Done after the calibration block so loadDisplayPrefs() can't accidentally
  // disturb the calib namespace handle.
  loadDisplayPrefs();
  loadVibPrefs();           // V12: Goertzel lock freq + last suggested wait
  refreshCachedThemeBg();   // prime THEME_BG cache before any draw runs

  // V12.5 (F4): the calib struct was just reset (CALIB_MAGIC bump or corrupt
  // NVS). If a decoupled "calibbak" backup exists and validates, re-apply it so
  // a friend's calibration survives the firmware update. The factory defaults
  // above already populated the non-calib fields (stack/LED/brightness);
  // finalizeCalibration() re-fits from the restored points and overwrites the
  // calib fields in "calib". Validation mirrors POST /calib (never trusts a
  // stored slope). On any failure we simply stay factory.
  if (calibWasReset) {
    CalibBackup b = {0};
    Preferences bp;
    bp.begin("calibbak", true);   // read-only
    size_t bl = bp.getBytes("bak", &b, sizeof(b));
    bp.end();
    bool ok = (bl == sizeof(b) && b.magic == CALIBBAK_MAGIC &&
               b.n >= 2 && b.n <= 20 &&
               b.width >= 100.0f && b.width <= 30000.0f &&
               b.demarc >= 0.01f && b.demarc <= 5.0f);
    for (int i = 0; ok && i < b.n; i++)
      if (!isfinite(b.dist[i]) || !isfinite(b.fov[i]) || b.fov[i] <= 1e-4f) ok = false;
    if (ok) {
      sensorWidthPixels = b.width;  demarcationDist = b.demarc;
      nPoints = b.n; pointsCaptured = b.n;
      for (int i = 0; i < b.n; i++) { distPoints[i] = b.dist[i]; fovPoints[i] = b.fov[i]; }
      finalizeCalibration();   // re-fit + re-save "calib" under the NEW magic
      currentMode = MAIN;      // finalizeCalibration()→drawSuccessScreen() set CAL_SUCCESS
      Serial.printf("[calib] restored %d pts from NVS calibbak after reset\n", (int)b.n);
    } else if (bl > 0) {
      Serial.println("[calib] calibbak present but invalid — staying factory");
    }
  }

  // V11 fix: pre-load factory calibration points into distPoints/fovPoints
  // when no custom calibration exists. Previously these arrays were only
  // populated when the user entered FOV_INFO and tapped GRAPH; navigating
  // directly to CAL_GRAPH on a factory boot plotted uninitialised memory.
  if (!isCustomCalib) {
    // Factory cal IS the firmware's built-in default. Force the current default
    // coefficients so a stale persisted factory save (e.g. from a dev build that
    // shared this CALIB_MAGIC but used different defaults) can't leave CTRLX/CTRLY
    // out of sync with FACTORY_DIST/FACTORY_FOV — the cause of a fit line that
    // didn't match the plotted points.
    CTRLX = Config::DEFAULT_CTRL_X; CTRLY = Config::DEFAULT_CTRL_Y;
    CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; CALIB_R2 = 0.9991f;
    nPoints = FACTORY_N;
    for (int i = 0; i < FACTORY_N; i++) {
      distPoints[i] = FACTORY_DIST[i];
      fovPoints[i]  = FACTORY_FOV[i];
    }
    // patched3: also seed pointsCaptured so the WS calGraphPoints array
    // includes the 13 factory rows.  Without this, buildFullStateJson's
    // loop bound `constrain(pointsCaptured, 0, 20)` was zero, and the HTML
    // graph rendered "no data" on a factory boot.
    pointsCaptured = FACTORY_N;
  }

  computeCalStats();   // prime prediction-interval stats from the loaded cal points

  analogWrite(LITE_PIN, currentBrightness);

  recoverI2CBus();
  Wire.begin();
  Wire.setClock(400000); 
  i2cMutex = xSemaphoreCreateMutex();

  if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
    if (!touch.begin(40)) { Serial.println("Couldn't start FT6206"); }
    sensor.begin();
    sensor.VL53L4CX_StopMeasurement();
    delay(100);
    sensor.InitSensor(0x52);
    delay(100);
    applyHighReflConfig();   // sets timing budget + ROI per current highReflMode
    sensor.VL53L4CX_StartMeasurement();
    imuSetup();              // V12: bring up the LSM6DSOX on the same I²C bus
    xSemaphoreGive(i2cMutex);
  }

#if defined(ARDUINO_ARCH_ESP32)
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, &sensorTaskHandle, 0);
#else
  xTaskCreate(sensorTask, "SensorTask", 1024, NULL, 2, &sensorTaskHandle);
#endif

  // V12: vibration-monitor — allocate PSRAM buffers, then start the DSP task on
  // Core 0 at priority 1 (below sensorTask's 2 so VL53 ranging is never delayed).
  vibBegin();
  if (imuPresent && vibReady)
    xTaskCreatePinnedToCore(vibTask, "VibTask", 8192, NULL, 1, &vibTaskHandle, 0);

  pinMode(CAMERA_TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  pinMode(TRIGGER_LED_PIN, OUTPUT);         // V17b: trigger LED (active-low: HIGH=off)
  digitalWrite(TRIGGER_LED_PIN, HIGH);       // V17b: LED off at boot
  irLed.begin();                             // patched3: IR LED transmitter (GPIO 12)
  attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), touchISR, FALLING);
  
  lastActivityTime = millis();
  // patched3: when we boot straight into the WiFi setup portal, open WIFI_INFO
  // instead of MAIN. The 8-char AP password is generated per-boot and shown
  // only on that screen — landing there means the user can read it and join
  // the AP without having to discover the WiFi-zone tap on the header.
  if (wifiIsPortal()) {
    currentMode = WIFI_INFO;
    drawWifiInfoUI();
  } else {
    drawMainScreen();
  }

  // V12.5 (F3): subscribe THIS task (loopTask, Core 1) to the hardware task
  // watchdog, fed once per loop() iteration. A UI/touch/wifiLoop freeze past the
  // timeout stops the feed → panic reboot (shows as TASK-WDT in the diag ring) →
  // recoverI2CBus() heals at boot. IDF 5.1.x pre-initialises the TWDT, so
  // init() returns INVALID_STATE and we reconfigure the timeout instead of a
  // fresh init. Done LAST, after the 3 s boot delay and all blocking init.
  {
    // The core auto-inits the TWDT (sdkconfig: 5 s, panic, CHECK_IDLE_TASK_CPU0=y
    // — the Core-0 idle task is watched, Core-1's isn't since loopTask hogs it).
    // We reconfigure to a 15 s timeout (generous vs. our longest legit Core-1
    // op ~1 s) and PRESERVE idle_core_mask = bit0 so the existing Core-0
    // idle-hang watch stays — setting it to 0 would silently drop that coverage.
    // Then subscribe loopTask so a Core-1/UI freeze also trips it.
    esp_task_wdt_config_t wdtCfg = { .timeout_ms = 15000, .idle_core_mask = (1 << 0), .trigger_panic = true };
    esp_err_t we = esp_task_wdt_init(&wdtCfg);
    if (we == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdtCfg);  // already inited by the core
    esp_task_wdt_add(NULL);   // subscribe loopTask; INVALID_ARG if already added — harmless
    Serial.println("[BOOT] loopTask watchdog armed (15 s, Core-0 idle preserved)");
  }

  Serial.println("[BOOT] setup() complete — entering loop()"); Serial.flush();
}

void registerActivity() {
  if (isScreenDim) {
    isScreenDim = false;
    analogWrite(LITE_PIN, currentBrightness);
  }
  lastActivityTime = millis();
}

void wakeScreen() {
  if (isScreenSleep) {
    isScreenSleep = false;
    isScreenDim = false;
    analogWrite(LITE_PIN, currentBrightness);
    infoScrollReset();   // V12.3: clear any HW scroll before repaint (overlay re-enables it)
    if (preSleepMode != WIFI_TIPS && preSleepMode != RECOVERY_HELP && preSleepMode != CALIB_HELP) freeInfoCanvas();
    switch (preSleepMode) {
      case MAIN:         drawMainScreen(); break;
      case APP_SETTINGS: drawAppSettingsUI(); break;
      case CAL_SETTINGS: drawCalSettingsUI(); break;
      case CAL_RUN:      drawPointEntryUI(); break;
      case STACK_CALC:   drawStackCalcUI(); break;
      case STACK_TIME:   drawStackTimeUI(); break;
      case CAL_CONFIRM:  drawConfirmUI(); break;
      case CAL_SUCCESS:  drawSuccessScreen(); break;
      case FOV_INFO:            drawFovInfoUI(); break;
      case CAL_REVIEW:          drawCalReviewUI(); break;
      case BRIGHTNESS_SETTINGS: drawBrightnessSettingsUI(); break;
      case SENSOR_INFO:         drawSensorInfoUI(); break;
      case MEM_INFO:            drawMemInfoUI(); break;
      case CAL_GRAPH:          drawCalGraphUI(); break;
      case ABOUT:              drawAboutUI(); break;
      case SCREEN_TIMEOUT:     drawScreenTimeoutUI(); break;
      case WIFI_INFO:          drawWifiInfoUI(); break;
      case IR_CONTROL:         drawIrControlUI(); break;
      case VIB_HOME:           drawVibHomeUI(); break;
      case VIB_SPECTRUM:       drawVibSpectrumUI(); break;
      case VIB_SETTLE:         drawVibSettleUI(); break;
      case VIB_SIGNATURES:     drawVibSignaturesUI(); break;
      case WIFI_TIPS:          drawInfoTextScreen(); break;
      case RECOVERY_HELP:      drawInfoTextScreen(); break;
      case CALIB_HELP:         drawInfoTextScreen(); break;
      default:           drawMainScreen(); break;
    }
    currentMode = preSleepMode;
  } else if (isScreenDim) {
    isScreenDim = false;
    analogWrite(LITE_PIN, currentBrightness);
  }
  lastActivityTime = millis();
}

void saveStackSettings() {
  settings.stackStepSize = stackStepSize;
  settings.stackTotalDepth = (float)stackTotalImgs;
  settings.stackTimePerStep = stackTimePerStep;
  preferences.begin("calib", false);
  preferences.putBytes("settings", &settings, sizeof(CalibData));
  preferences.end();
}

void saveAllSettings() {
  settings.brightness = currentBrightness;
  settings.stackStepSize = stackStepSize;
  settings.stackTotalDepth = (float)stackTotalImgs;
  settings.stackTimePerStep = stackTimePerStep;
  settings.ledEnabled = ledEnabled ? 1 : 0;
  settings.ledDuty = (uint8_t)constrain(currentLedDuty, 0, 127);  // 127 = max safe duty
  preferences.begin("calib", false);
  preferences.putBytes("settings", &settings, sizeof(CalibData));
  preferences.end();
}

void loop() {
  // V12.5 (F3): feed the loopTask hardware watchdog at the TOP of every
  // iteration — always reached, so an early `return` in the body (e.g. the
  // wake-from-sleep touch path) can't skip it and cause a spurious reset. A
  // real Core-1 freeze stalls this feed → panic reboot → recoverI2CBus() heals.
  // Armed in setup(); harmless before that (esp_task_wdt_reset is a no-op if the
  // task isn't subscribed yet, though loop() only runs after setup() returns).
  esp_task_wdt_reset();

  // V11 deferred display-prefs save — if the tint slider was dragged and
  // 500ms has passed since the last drag sample, flush to NVS now. Avoids
  // writing flash on every drag step (which choked the UI and chewed through
  // the flash erase budget) while still persisting within half a second of
  // the user releasing the slider.
  if (displayPrefsDirty && (millis() - lastTintDragMs) > 500) {
    saveDisplayPrefs();
  }

  // V12: deferred "vib" namespace save — RE-LOCK and stack-complete mark
  // vibPrefsDirty; flush ~500 ms later (same pattern as the display prefs).
  if (vibPrefsDirty && (millis() - lastVibEditMs) > 500) {
    saveVibPrefs();
  }

  // V12: advance any armed signature capture (screen-independent).
  vibCapturePoll();

  // patched3: deferred "calib" namespace save — WS settings edits (brightness,
  // LED duty, secStep) mark calibPrefsDirty instead of writing flash per
  // message; flush once ~500 ms after the last edit. See calibPrefsDirty decl.
  if (calibPrefsDirty && (millis() - lastCalibEditMs) > 500) {
    saveAllSettings();
    calibPrefsDirty = false;
  }

  // patched3: deferred WS-driven repaint.  WebSocket theme/tint commands set
  // displayNeedsRedraw and stamp lastTintDragMs.  When a rapid HTML tint-drag
  // stream subsides for 500 ms, we redraw the current screen once (rather
  // than repainting on every 20-50 ms WS message, which would full-screen
  // flicker at 20+ Hz).  Uses the same timer as the NVS save above for
  // single-source-of-truth on "drag has stopped".
  if (displayNeedsRedraw && (millis() - lastTintDragMs) > 500) {
    redrawCurrentScreen();
    displayNeedsRedraw = false;
  }

  // ─── V12.5: field-resilience periodic checks (Core 1) ───────────────────────
  {
    uint32_t nowMs = millis();

    // F1 — once we've survived 30 s, this boot is "healthy": clear the boot-loop
    // counter so an intentional reboot (OTA, forget-wifi) never accumulates, and
    // mark the running image valid. The mark is a cheap no-op unless bootloader
    // rollback is ever enabled in menuconfig, in which case it's required.
    static bool healthyMarked = false;
    if (!healthyMarked && nowMs > 30000) {
      g_rtc.bootCount = 0;
      g_rtc.rollbackAttempted = 0;
      healthyMarked = true;
      esp_ota_mark_app_valid_cancel_rollback();
      Serial.println("[BOOT] 30 s healthy — boot-loop counter cleared");
    }

    // F2 — snapshot uptime + min-heap to RTC every ~5 s, so the NEXT boot can
    // report "was up 8 h / 40 KB then BROWNOUT" even after an uncontrolled reset.
    // V12.5.x: also sample the health metrics here (once, cheaply, on Core 1) —
    // #1 per-task free-stack high-water (predicts a stack-overflow PANIC and
    // names the culprit) and #3 the largest contiguous free block (fragmentation,
    // predicts an OOM despite "free heap OK"). The task-stack minimum and the
    // worst fragmentation are folded into the RTC snapshot so the NEXT boot's
    // ring entry records the pre-crash state.
    static uint32_t lastSnapMs = 0;
    if (nowMs - lastSnapMs > 5000) {
      lastSnapMs = nowMs;
      g_rtc.lastUptimeSec = nowMs / 1000;
      g_rtc.lastMinHeap   = ESP.getMinFreeHeap() / 1024;

      // uxTaskGetStackHighWaterMark returns the min free stack in WORDS (×4 = B).
      // Only sample tasks whose HANDLE exists — a NULL handle would make
      // FreeRTOS report the CALLING (loop) task, mislabeling it. Guarding on the
      // handle (not the value) also lets a genuine 0-word watermark — the exact
      // "overflow imminent" moment the metric is for — count toward the minimum.
      uint32_t minStk = 0xFFFFFFFFUL;
      uint32_t sLoop = uxTaskGetStackHighWaterMark(NULL) * 4;   // loopTask (this task) — always valid
      stackFreeLoop.store(sLoop, std::memory_order_relaxed);
      if (sLoop < minStk) minStk = sLoop;
      if (sensorTaskHandle) {
        uint32_t s = uxTaskGetStackHighWaterMark(sensorTaskHandle) * 4;
        stackFreeSensor.store(s, std::memory_order_relaxed);
        if (s < minStk) minStk = s;
      }
      if (vibTaskHandle) {
        uint32_t s = uxTaskGetStackHighWaterMark(vibTaskHandle) * 4;
        stackFreeVib.store(s, std::memory_order_relaxed);
        if (s < minStk) minStk = s;
      }
      if (minStk != 0xFFFFFFFFUL) g_rtc.lastMinStack = minStk;

      uint32_t maxBlockKB = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024;
      if (maxBlockKB < minMaxAllocKB.load(std::memory_order_relaxed))
        minMaxAllocKB.store(maxBlockKB, std::memory_order_relaxed);
      g_rtc.lastMaxAllocKB = minMaxAllocKB.load(std::memory_order_relaxed);
    }

    // F3 — sensor-heartbeat monitor. The auto-reboot is DISABLED: on real
    // hardware the heartbeat went stale >6 s intermittently during normal
    // operation (an I²C stall that recovers on its own), so rebooting on it
    // dropped WiFi every ~20-30 min for no benefit. We now only RECORD the worst
    // stall (surfaced in /diag) so the intermittent stall can be diagnosed
    // without disrupting the device. A genuine full Core-1 freeze is still caught
    // by the loopTask hardware watchdog. Root cause under investigation — do not
    // re-enable the reboot without understanding why sensorTask stalls.
    uint32_t hb = sensorHeartbeatMs.load(std::memory_order_acquire);
    if (hb && nowMs > 12000) {
      // SIGNED diff, freshly sampled: sensorTask (Core 0) and loop() (Core 1)
      // read millis() independently, so hb can be a millisecond or two NEWER than
      // us — an unsigned (millis()-hb) then underflows to ~2^32 and reads as an
      // absurd multi-billion-ms "stall". Treat a momentarily-ahead heartbeat as
      // no stall. (This same unsigned underflow was what tripped the removed F3
      // auto-reboot on a phantom stall.)
      int32_t stall = (int32_t)(millis() - hb);
      if (stall > 2000 && (uint32_t)stall > sensorMaxStallMs.load(std::memory_order_relaxed)) {
        sensorMaxStallMs.store((uint32_t)stall, std::memory_order_relaxed);
        Serial.printf("[SENSOR] heartbeat stall %d ms (max so far; not rebooting)\n", stall);
      }
    }
  }

  uint32_t currentState = sensorState.load(std::memory_order_acquire);
  uint32_t hState = sensorHealth.load(std::memory_order_acquire);

  bool rangeValid = (currentState >> 31) & 0x1;
  int currentDist = currentState & 0x7FFFFFFF;
  uint8_t lastStatus = (hState >> 24) & 0xFF;
  float lastMCPS = (hState & 0xFFFFFF) / 1000.0f;

  // patched3: drain WiFi command queue + push telemetry (non-blocking)
  wifiLoop();


  if (currentMode != CAL_SAMPLING) {
    unsigned long idle = millis() - lastActivityTime;
    if (!isScreenSleep && idle > sleepTimeoutMs) {
      isScreenSleep = true;
      isScreenDim = false;
      preSleepMode = currentMode;
      analogWrite(LITE_PIN, 0);
    } else if (!isScreenSleep && !isScreenDim && idle > dimTimeoutMs) {
      isScreenDim = true;
      analogWrite(LITE_PIN, DIM_BRIGHTNESS);
    }

    // ── Sensor auto-power-off after prolonged inactivity (1 h) ──
    // Same idle clock as the screen timeouts. Powers down the TOF (reusing the
    // manual-sleep stop path) and the accelerometer (vibTask honours vibAutoOff)
    // once idle, and restores them the moment any activity resets lastActivityTime
    // (touch, camera-trigger pulse, or a web menu/settings change).
    if (!sensorsAutoOff && idle > SENSOR_IDLE_OFF_MS) {
      bool tofSleepOk = true;
      if (!sensorSleeping) {                 // leave a user-slept TOF as-is
        // Flag first so sensorTask stops polling; revert if the stop timed out
        // (same pattern as handleSensorInfoTouch — see sensorSleeping decl).
        sensorSleeping = true;
        sensorState.store(0, std::memory_order_release);
        sensorHealth.store(0xFF000000UL, std::memory_order_release);
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
          sensor.VL53L4CX_StopMeasurement();
          xSemaphoreGive(i2cMutex);
          tofAutoSlept = true;
        } else {
          sensorSleeping = false;
          tofSleepOk = false;   // keep sensorsAutoOff clear so this retries —
                                // otherwise the TOF ranges all idle-night while
                                // the log claims it was powered down
        }
      }
      vibAutoOff.store(true, std::memory_order_release);   // vibTask powers the IMU down
      if (tofSleepOk) {
        sensorsAutoOff = true;
        Serial.println("[idle] inactivity timeout — TOF + accelerometer powered down");
      }
    } else if (sensorsAutoOff && idle < SENSOR_IDLE_OFF_MS) {
      bool tofOk = true;
      if (tofAutoSlept) {                    // only wake the TOF if WE slept it
        if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
          applyHighReflConfig();
          sensor.VL53L4CX_StartMeasurement();
          xSemaphoreGive(i2cMutex);
          sensorEmaReset.store(true, std::memory_order_release);
          sensorSleeping = false;
          tofAutoSlept = false;
        } else {
          tofOk = false;   // keep sensorsAutoOff set so this pass retries
        }
      }
      vibAutoOff.store(false, std::memory_order_release);  // vibTask re-powers the IMU
      if (tofOk) {
        sensorsAutoOff = false;
        Serial.println("[idle] activity resumed — TOF + accelerometer back on");
      }
    }
  }

  if (currentMode == CAL_SAMPLING) {
    if ((unsigned long)(millis() - samplingStartTime) < 2000) {
      // Average the sub-mm (tenths-of-a-mm) distance, not the whole-mm currentDist,
      // so a captured point keeps fractional resolution instead of snapping to 1 mm.
      uint32_t t = sensorDistTenths.load(std::memory_order_acquire);
      if (t >> 31) { samplingSum += (long)(t & 0x7FFFFFFF); samplingCount++; }
    } else {
      if (samplingCount > 0) {
        tft.fillRect(0, 305, 240, 15, THEME_BG);
        distPoints[currentCalIndex] = (float)samplingSum / samplingCount / 10.0f;  // tenths → mm
        fovPoints[currentCalIndex] = (demarcationDist / (float)tempPixels) * sensorWidthPixels;

        // V17: branch on retake mode vs normal sequential capture
        if (isRetakeMode) {
          // Slot was overwritten in place; pointsCaptured unchanged.
          // Return to review screen so user can verify the new value.
          isRetakeMode = false;
          tempPixels = Config::DEFAULT_TEMP_PIXELS;
          currentMode = CAL_REVIEW;
          drawCalReviewUI();
        } else {
          currentCalIndex++;
          if (currentCalIndex > pointsCaptured) pointsCaptured = currentCalIndex;

          if (currentCalIndex < nPoints) {
            tempPixels = Config::DEFAULT_TEMP_PIXELS;
            currentMode = CAL_RUN;
            drawPointEntryUI();
          } else {
            currentMode = CAL_RUN;
            lastCalibName[0] = '\0';   // device cal → generic success (no profile name)
            finalizeCalibration();
          }
        }
      } else {
        tft.fillRect(0, 305, 240, 15, THEME_BG);
        setSmoothFont(1); tft.setCursor(5, 318);
        tft.setTextColor(COLOR_RED); tft.print("SAMPLE FAILED");
        delay(1000);
        currentMode = CAL_RUN;
        drawPointEntryUI();
      }
    }
  }

  static unsigned long lowStartTime = 0;
  static bool isCurrentlyLow = false;
  // patched3: lastCircleState was renamed and promoted to file-scope as
  // `shutterActive` so the wifi tab can include it in fast telemetry.

  bool rawState = (digitalRead(CAMERA_TRIGGER_PIN) == LOW);

  if (rawState) {
    if (!isCurrentlyLow) {
      lowStartTime = millis();
      isCurrentlyLow = true;
    }
  } else {
    isCurrentlyLow = false;
  }

  bool validTrigger = (isCurrentlyLow && (millis() - lowStartTime > 15));

  // V17: LED mirrors validTrigger on any screen; circle only on MAIN.
  if (validTrigger && !shutterActive) {
    fireTriggerLed(true);
    if (currentMode == MAIN) {
      tft.fillCircle(114, 299, 6, COLOR_DARKTEAL);
      tft.fillCircle(112, 297, 6, COLOR_TEAL);
      tft.fillCircle(110, 295, 2, TFT_WHITE);
    }
    shutterActive = true;
    // V12.3: count each shot after the first. The first pulse seeds the count
    // in the sequence-init block below (isSequenceActive is still false here on
    // that first edge), so every subsequent rising edge increments cleanly.
    if (isSequenceActive) stackPulseCount++;
    // V12: note the shutter-pulse edge for passive settle-time analysis.
    // Passive only — nothing here delays or gates the trigger path.
    vibPulseRingPos = vibRawHead;
    // Release on the seq bump pairs with acquire in vibTask so the
    // ringPos write above is visible there before settle analysis runs.
    vibPulseSeq.fetch_add(1, std::memory_order_release);
  } else if (!validTrigger && shutterActive) {
    fireTriggerLed(false);
    if (currentMode == MAIN) {
      tft.fillCircle(113, 298, 8, THEME_BG);
    }
    shutterActive = false;
  }

  if (validTrigger) {
    unsigned long pulseTime = millis();
    if (!isSequenceActive) {
      isSequenceActive = true;
      firstPulseTime = pulseTime;
      stackPulseCount = 1;   // V12.3: this first shot; later edges add to it
      vibStackStartSeq.fetch_add(1, std::memory_order_relaxed);
    }
    lastPulseTime = pulseTime;
    registerActivity();
  }

  if (isSequenceActive && ((unsigned long)(millis() - lastPulseTime) >= SILENCE_DURATION)) {
    mjkzzStackActive = false;   // V12.3: pulses stopped → stack ended; sync web buttons
    unsigned long totalActiveTime = (unsigned long)(lastPulseTime - firstPulseTime);
    if (totalActiveTime >= MIN_ACTIVE_DURATION) {
      wifiNotifyStackComplete();
      // V12: ask vibTask to compute the aggregate suggested wait, then persist.
      vibStackDoneSeq.fetch_add(1, std::memory_order_relaxed);
      vibPrefsDirty = true; lastVibEditMs = millis();

      // V12.3: turn this run into a measured stack time. Per-step is the mean
      // interval between shots (gaps = pulses - 1), which is robust even when
      // the rail fired a different shot count than the configured stackTotalImgs.
      measuredPulses   = stackPulseCount;
      measuredTotalSec = (uint32_t)((totalActiveTime + 500) / 1000);
      if (stackPulseCount >= 2) {
        measuredPerStep = (float)totalActiveTime / 1000.0f / (float)(stackPulseCount - 1);
        // Auto-apply when the projected total at the configured image count
        // would shift by >= 30 s, so the manual Sec/Step keeps up on its own.
        float projectedDelta = fabsf(measuredPerStep - stackTimePerStep) * (float)stackTotalImgs;
        if (projectedDelta >= MEASURED_DRIFT_SEC) {
          stackTimePerStep = constrain(measuredPerStep, 0.1f, 60.0f);
          settings.stackTimePerStep = stackTimePerStep;
          calibPrefsDirty = true; lastCalibEditMs = millis();
        }
        if (currentMode == STACK_TIME) drawStackTimeUI();
      }
    } else {
      // Too short to be a stack (single shot / aborted run) — tell vibTask to
      // leave in-stack mode without folding this into the suggested wait.
      vibStackAbortSeq.fetch_add(1, std::memory_order_relaxed);
    }
    isSequenceActive = false;
  }

  static bool activelyTouching = false;
  bool isTouched = false;
  TS_Point p;
  
  if (touchDetected || activelyTouching) {
    if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(10))) {
      if (touch.touched()) {
        p = touch.getPoint();
        isTouched = true;
        activelyTouching = true;
      } else {
        // V11: tint slider lift — finger went from down to up while a tint
        // drag was in progress. Now do ONE clean drawScreenTimeoutUI() with
        // the final tint, which fillScreen's the page once and renders
        // everything consistently in the new shade. Doing it here (instead
        // of mid-drag) is what stops the flicker/banding.
        if (tintDragActive && currentMode == SCREEN_TIMEOUT) {
          refreshCachedThemeBg();   // tint changed during drag; bake new bg
          drawScreenTimeoutUI();
          if (displayPrefsDirty) saveDisplayPrefs();
          tintDragActive = false;
        }
        // V12.3: finger lifted — end any info-overlay drag so the next press
        // starts a fresh 1:1 baseline instead of jumping.
        if (currentMode == WIFI_TIPS || currentMode == RECOVERY_HELP || currentMode == CALIB_HELP) infoDragActive = false;
        activelyTouching = false;
        adjFingerLifted = true;   // tell adj logic the finger genuinely lifted
      }
      touchDetected = false;
      xSemaphoreGive(i2cMutex);
    }
  }

  if (isTouched && currentMode != CAL_SAMPLING) {
    p.x = 240 - p.x; p.y = 320 - p.y; 
    
    if (isScreenSleep) {
      wakeScreen();
      return;
    }
    
    registerActivity();
    
    // Brightness sliders now live in BRIGHTNESS_SETTINGS.
    // patched3: LED slider zone is y=166..196 (bar at y=172..194) after the
    // LED group moved up 12 px to make room for the IR CONTROL button.
    // SCREEN_TIMEOUT also has a tint slider on the THEME row (y=207..232)
    // — tagged here so drags get the fast 20ms debounce instead of 150ms.
    bool isSlider = (currentMode == BRIGHTNESS_SETTINGS && ((p.y >= 80 && p.y <= 125) || (p.y >= 166 && p.y <= 196)))
                  || (currentMode == APP_SETTINGS && p.y >= 76 && p.y <= 118)
                  || (currentMode == SCREEN_TIMEOUT && p.x >= 65 && p.x <= 230 && p.y >= 183 && p.y <= 215);
    // V12.3: info-overlay body drag wants the same fast sampling as sliders so
    // the text tracks the finger smoothly. It is NOT isSlider (which routes to
    // the inline brightness/tint handlers) — it falls through to the switch.
    bool isInfoDrag = (currentMode == WIFI_TIPS || currentMode == RECOVERY_HELP || currentMode == CALIB_HELP) && p.y >= INFO_VP_TOP;
    int debounce = (isSlider || isInfoDrag) ? 20 : 150;

    // ── Menu-transition guard ───────────────────────────────────────────────
    // Ignore all button taps for 350 ms after any screen change.  This stops
    // a "too-long" press on a back/close button from immediately triggering
    // whatever button happens to share the same coordinates on the new screen.
    // We keep advancing lastTouchTime so the held touch cannot queue up and
    // fire the instant the 350 ms window expires.
    if ((unsigned long)(millis() - lastModeChangeMs) < 350UL) {
      lastTouchTime = millis();   // eat the event; don't dispatch
    } else if ((unsigned long)(millis() - lastTouchTime) > debounce) {
      lastTouchTime = millis();
      DisplayMode modeBefore = currentMode;   // for transition-guard stamp below

      if (isSlider) {
        if (currentMode == APP_SETTINGS) {
          // Live brightness drag in settings bar (slider y≈84..104, bar x=15..225,
          // touch zone y=76..118 with a buffer above STACK CALC at y=127)
          int valX = constrain(p.x, 15, 225);
          currentBrightness = max(1, (int)map(valX, 15, 225, 1, 255));
          analogWrite(LITE_PIN, currentBrightness);
          drawOldBrightnessBar();
          wifiPushSettings();
        } else if (currentMode == SCREEN_TIMEOUT) {
          handleScreenTimeoutTouch(p);
        } else {
          handleBrightnessSettingsTouch(p, 0);
        }
      } else {
        int adj = 0; 
        if (currentMode == CAL_SETTINGS || currentMode == CAL_RUN || 
            currentMode == STACK_CALC || currentMode == STACK_TIME) {
          // BRIGHTNESS_SETTINGS has no adj buttons — its touches go through
          // the slider path above.
          int btnY;
          if (currentMode == CAL_SETTINGS) btnY = 165;
          else if (currentMode == STACK_CALC) btnY = 207;   // V17b: was 197
          else if (currentMode == STACK_TIME) btnY = 155;
          else btnY = 135;
          
          if (p.y > btnY && p.y < (btnY + 40)) { 
            static unsigned long holdStartTime = 0;
            static int lastRawAdj = 0;
            static unsigned long lastAdjTime = 0;
            static DisplayMode lastAdjMode = MAIN;

            int rawAdj = 0;
            if (p.x < 65) rawAdj = -100; else if (p.x < 120) rawAdj = -1; else if (p.x < 175) rawAdj = 1; else rawAdj = 100;

            unsigned long now = millis();
            // Reset the hold timer if:
            //   (a) the finger was genuinely lifted since the last adj — this is
            //       the primary guard against rapid single-taps triggering accel,
            //   (b) the direction or screen changed, or
            //   (c) there's been a long pause (safety net).
            if (adjFingerLifted || rawAdj != lastRawAdj || lastAdjMode != currentMode
                || now - lastAdjTime > 400) {
              holdStartTime = now;
              adjFingerLifted = false;
            }
            lastAdjTime = now; lastRawAdj = rawAdj; lastAdjMode = currentMode; adj = rawAdj;
            if ((rawAdj == 1 || rawAdj == -1) && (now - holdStartTime > 1750)) adj *= 5;
          }
        }

        switch (currentMode) {
          case MAIN:                handleMainTouch(p); break;
          case APP_SETTINGS:        handleAppSettingsTouch(p); break;
          case CAL_SETTINGS:        handleCalSettingsTouch(p, adj); break;
          case CAL_RUN:             handleCalRunTouch(p, adj); break;
          case CAL_CONFIRM:         handleCalConfirmTouch(p); break;
          case CAL_SUCCESS:         handleCalSuccessTouch(p); break;
          case STACK_CALC:          handleStackCalcTouch(p, adj); break;
          case STACK_TIME:          handleStackTimeTouch(p, adj); break;
          case FOV_INFO:            handleFovInfoTouch(p); break;
          case CAL_REVIEW:          handleCalReviewTouch(p); break;
          case BRIGHTNESS_SETTINGS: handleBrightnessSettingsTouch(p, adj); break;
          case SENSOR_INFO:         handleSensorInfoTouch(p); break;
          case MEM_INFO:           handleMemInfoTouch(p); break;
          case CAL_GRAPH:          handleCalGraphTouch(p); break;
          case ABOUT:              handleAboutTouch(p); break;
          case SCREEN_TIMEOUT:     handleScreenTimeoutTouch(p); break;
          case WIFI_INFO:          handleWifiInfoTouch(p); break;
          case IR_CONTROL:         handleIrControlTouch(p); break;
          case VIB_HOME:           handleVibHomeTouch(p); break;
          case VIB_SPECTRUM:       handleVibSpectrumTouch(p); break;
          case VIB_SETTLE:         handleVibSettleTouch(p); break;
          case VIB_SIGNATURES:     handleVibSignaturesTouch(p); break;
          case WIFI_TIPS:          handleInfoTextTouch(p); break;
          case RECOVERY_HELP:      handleInfoTextTouch(p); break;
          case CALIB_HELP:         handleInfoTextTouch(p); break;
          default: break;
        }
      }
      // If this touch caused a screen change, arm the transition guard so the
      // very next touch event(s) are eaten until the 350 ms window expires.
      if (currentMode != modeBefore) lastModeChangeMs = millis();
    }
  }
  
  if (currentMode == SENSOR_INFO && ((unsigned long)(millis() - lastSensorInfoUpdate) > 500)) {
    refreshSensorInfoValues();
    lastSensorInfoUpdate = millis();
  }

  // patched3: live refresh for WIFI_INFO (RSSI / SSID / IP / mode rows).
  if (currentMode == WIFI_INFO && ((unsigned long)(millis() - lastWifiInfoUpdate) > 1000)) {
    refreshWifiInfoValues();
    lastWifiInfoUpdate = millis();
  }

  // V12: live refresh for VIB_SPECTRUM (~7 Hz). The readout updates every call;
  // the spectrum plot + waterfall are only redrawn when vibTask publishes a new
  // FFT frame (see the isNewFrame gate in refreshVibSpectrumValues).
  if (currentMode == VIB_SPECTRUM && ((unsigned long)(millis() - lastVibSpecUpdate) > 150)) {
    refreshVibSpectrumValues(false);
    lastVibSpecUpdate = millis();
  }
  // V12: VIB_SETTLE refreshes slowly (analysis events are ~seconds apart);
  // VIB_SIGNATURES refreshes faster to stream the live comparison + capture.
  if (currentMode == VIB_SETTLE && ((unsigned long)(millis() - lastVibSpecUpdate) > 250)) {
    refreshVibSettleValues(false);
    lastVibSpecUpdate = millis();
  }
  if (currentMode == VIB_SIGNATURES && ((unsigned long)(millis() - lastVibSpecUpdate) > 180)) {
    refreshVibSignaturesValues(false);
    lastVibSpecUpdate = millis();
  }

  // patched3: poll header WiFi indicator on MAIN every 5 s to track RSSI drift.
  // Cheap (one fillRect zone + 4 small bar rects + a label) — fine at this rate.
  if (currentMode == MAIN && ((unsigned long)(millis() - lastWifiHeaderCheckMs) > 5000)) {
    drawWifiIndicator();
    lastWifiHeaderCheckMs = millis();
  }

  // patched3: deferred forget-WiFi action (set by handleWifiInfoTouch).
  // Lets the "CLEARING..." flash render and stays responsive while waiting.
  if (wifiForgetAtMs != 0 && (int32_t)(millis() - wifiForgetAtMs) >= 0) {
    wifiForgetAndRestart();   // does not return
  }

  // Compact live "D:XXXmm" + signal-health readout at the top-right header.
  // Only the calibration-flow screens want it — every other screen either
  // owns its own readout (MAIN's big FOV/dist block) or has no room. New
  // screens default to "no header readout"; opt in by adding the mode here.
  if (currentMode == CAL_SETTINGS ||
      currentMode == CAL_RUN      ||
      currentMode == CAL_SAMPLING) {
    if ((unsigned long)(millis() - lastCalibDistUpdate) > 600) {
      char distBuf[16];
      if (rangeValid) snprintf(distBuf, sizeof(distBuf), "D:%dmm", currentDist);
      else snprintf(distBuf, sizeof(distBuf), "D:---");
      
      menuSprite.fillScreen(THEME_BG); 
      menuSprite.setTextSize(1);
      menuSprite.setFont(&FreeSans9pt7b);
      menuSprite.setTextColor(rangeValid ? COLOR_LIGHTGREY : COLOR_RED);
      
      int16_t x1, y1; uint16_t w, h;
      menuSprite.getTextBounds(distBuf, 0, 0, &x1, &y1, &w, &h);
      menuSprite.setCursor(75 - w - x1, 15); 
      menuSprite.print(distBuf); 
      
      drawSignalHealthBar(lastStatus, lastMCPS, 33, 20, true); 
      
      tft.drawRGBBitmap(160, 0, menuSprite.getBuffer(), 80, 40);
      lastCalibDistUpdate = millis();
    }
  }
  
  if ((unsigned long)(millis() - lastDisplayUpdate) > 30) {
    updateSensorAverages();   // feeds sensorAvgDist/sensorErrInt on every screen
    if (currentMode == MAIN) {
      updateDisplay();
      drawSignalHealthBar(lastStatus, lastMCPS, 44, 12, false);
    }
    lastDisplayUpdate = millis();
  }

  vTaskDelay(pdMS_TO_TICKS(1));
}

// --- Touch Handlers ---

// handleBrightnessSlider removed — handled in handleBrightnessSettingsTouch

void handleMainTouch(TS_Point p) {
  // Top-strip tappable zones (y=0..42):
  // WiFi badge:   x=0..32   → WIFI_INFO screen
  // Signal bar:   x=33..90  → SENSOR_INFO screen
  // VIBE CHECK:   x=90..150 → VIB_HOME (vibration hub)
  if (p.y < 42) {
    if (p.x < 32)  { currentMode = WIFI_INFO;   drawWifiInfoUI();   return; }
    if (p.x < 90)  { currentMode = SENSOR_INFO; drawSensorInfoUI(); return; }
    if (p.x < 150) { currentMode = VIB_HOME;    drawVibHomeUI();    return; }
  }

  if (p.y > Y_POS && p.y < (Y_POS + BOX_SIZE)) {
    int newObj = currentobj;
    if (p.x < BOX_SIZE + 5) newObj = 1; 
    else if (p.x < X_POS_20X - 5) newObj = 2; 
    else newObj = 3;
    
    if (newObj != currentobj) {
      currentobj = newObj;
      updateObjectiveButtons();
    }
  }
  // V17. Exact-rect test (not Button::contains, whose 5 px pad bled the hitbox
  // into the dead space before the gear icon).
  if (p.x >= btnFovInfo.x && p.x < btnFovInfo.x + btnFovInfo.w &&
      p.y >= btnFovInfo.y && p.y < btnFovInfo.y + btnFovInfo.h) {
    currentMode = FOV_INFO; drawFovInfoUI(); return;
  }
  // V11: tap on the calib/version block (bottom-left) opens the ABOUT screen.
  // Box drawn in drawMainScreen at x=2..96, y=283..317.
  if (p.x >= 0 && p.x <= 100 && p.y >= 280 && p.y <= 320) {
    currentMode = ABOUT; drawAboutUI(); return;
  }
  if (btnSettingsGear.contains(p.x, p.y)) { 
    currentMode = APP_SETTINGS; drawAppSettingsUI(); 
  }
  if (btnMainCalibrate.contains(p.x, p.y)) { 
    currentMode = CAL_SETTINGS; calibSelection = 0; drawCalSettingsUI(); 
  }
}

void handleAppSettingsTouch(TS_Point p) {
  // Brightness bar (y=45..95) is a live drag slider — handled via isSlider in loop
  if (btnStackCalc.contains(p.x, p.y)) {
    currentMode = STACK_CALC; stackCalcSelection = 0; drawStackCalcUI(); return;
  }
  if (btnBrightness.contains(p.x, p.y)) {
    currentMode = BRIGHTNESS_SETTINGS; drawBrightnessSettingsUI(); return;
  }
  if (btnSettingsMem.contains(p.x, p.y)) {
    currentMode = MEM_INFO; drawMemInfoUI(); return;
  }
  if (btnVibMonitor.contains(p.x, p.y)) {
    currentMode = VIB_HOME; drawVibHomeUI(); return;
  }
  if (btnSettingsClose.contains(p.x, p.y)) {
    settings.brightness = currentBrightness;
    preferences.begin("calib", false);
    preferences.putBytes("settings", &settings, sizeof(CalibData));
    preferences.end();
    currentMode = MAIN; drawMainScreen(); return;
  }
}

void handleCalSettingsTouch(TS_Point p, int adj) {
  if (btnCalInfo.contains(p.x, p.y)) {
    openInfoScreen(CALIB_HELP);
    return;
  }
  bool selChanged = false;
  if (p.y > 45 && p.y < 80) { calibSelection = 0; selChanged = true; }
  else if (p.y > 80 && p.y < 120) { calibSelection = 1; selChanged = true; }
  else if (p.y > 120 && p.y < 160) { calibSelection = 2; selChanged = true; }
  
  if (selChanged) refreshCalSettingsValues(false);
  
  if (adj != 0) {
    if (calibSelection == 0) {
      sensorWidthPixels = constrain(sensorWidthPixels + adj, 100.0f, 30000.0f);
    }
    else if (calibSelection == 1) {
      float step = (abs(adj) >= 2) ? 0.1f : 0.01f;
      float direction = (adj > 0) ? 1.0f : -1.0f;
      demarcationDist = constrain(demarcationDist + direction * step, 0.01f, 5.0f);
    }
    else {
      int delta = (abs(adj) >= 2) ? 5 : 1;
      if (adj < 0) delta = -delta;
      nPoints = constrain(nPoints + delta, Config::MIN_CALIB_POINTS, Config::MAX_CALIB_POINTS);
    }
    refreshCalSettingsValues(false);
  }
  
  if (btnStartCal.contains(p.x, p.y)) {
    currentMode = CAL_RUN;
    currentCalIndex = 0;
    pointsCaptured = 0;          // V17: fresh calibration starts with no captured points
    isRetakeMode = false;        // V17
    calReviewReadOnly = false;   // editable review for this session (clears the viewer flag)
    tempPixels = Config::DEFAULT_TEMP_PIXELS;
    drawPointEntryUI(); 
  }
  else if (btnResetAll.contains(p.x, p.y)) {
    btnResetAll.draw(tft, "RESETTING", COLOR_MAROON, TFT_WHITE);
    resetToFactory();
    delay(300);
    btnResetAll.draw(tft);
  }
  else if (btnCancelCal.contains(p.x, p.y)) { 
    currentMode = MAIN; drawMainScreen(); 
  }
}

void handleCalRunTouch(TS_Point p, int adj) {
  if (adj != 0) { 
    tempPixels += adj; 
    if (tempPixels < 1) tempPixels = 1; 
    refreshPixelValue(tempPixels, false); 
  }
  
  if (btnCapSave.contains(p.x, p.y)) { 
    if ((sensorState.load(std::memory_order_acquire) >> 31) == 0) return; 
    
    currentMode = CAL_SAMPLING;
    samplingStartTime = millis(); samplingSum = 0; samplingCount = 0;
    
    tft.fillRect(0, 305, 240, 15, THEME_BG);
    setSmoothFont(1); tft.setCursor(5, 318); 
    tft.setTextColor(themedText(COLOR_GREENYELLOW)); tft.print("SAMPLING...");
  }
  else if (btnGoBackPt.contains(p.x, p.y)) {
    // V17: in retake mode, GO BACK returns to review without changes
    if (isRetakeMode) {
      isRetakeMode = false;
      currentMode = CAL_REVIEW;
      drawCalReviewUI();
      return;
    }
    if (currentCalIndex == 0) { currentMode = CAL_SETTINGS; drawCalSettingsUI(); }
    else { currentCalIndex--; drawPointEntryUI(); }
  }
  // V17: REVIEW button — only active when at least one point has been captured
  else if (btnReviewPts.contains(p.x, p.y)) {
    if (pointsCaptured > 0 && !isRetakeMode) {
      reviewSelected = -1;
      reviewScrollOffset = 0;
      currentMode = CAL_REVIEW;
      drawCalReviewUI();
    }
  }
  // FINISH — finalize early once enough points exist for a valid fit.
  else if (btnFinalize.contains(p.x, p.y)) {
    if (pointsCaptured >= Config::MIN_CALIB_POINTS && !isRetakeMode) {
      finalizeEarly();
    }
  }
  else if (btnCancelPt.contains(p.x, p.y)) {
    if (isRetakeMode) {
      // Cancel during a retake just goes back to review, doesn't nuke the cal
      isRetakeMode = false;
      currentMode = CAL_REVIEW;
      drawCalReviewUI();
    } else if (pointsCaptured > 0) {
      currentMode = CAL_CONFIRM;
      drawConfirmUI();
    } else {
      currentMode = MAIN;
      drawMainScreen();
    }
  }
}

void handleCalConfirmTouch(TS_Point p) {
  if (btnYesDel.contains(p.x, p.y)) {
    pointsCaptured = 0;          // V17: clear captured-point counter on cancel-confirm
    isRetakeMode = false;
    currentMode = MAIN; drawMainScreen();
  }
  if (btnNoKeep.contains(p.x, p.y)) { currentMode = CAL_RUN; drawPointEntryUI(); }
}

void handleCalSuccessTouch(TS_Point p) {
  if (btnFinishCal.contains(p.x, p.y)) { currentMode = MAIN; drawMainScreen(); }
}

void handleStackCalcTouch(TS_Point p, int adj) {
  bool selChanged = false;
  if (p.y > 45 && p.y < 76) { stackCalcSelection = 0; selChanged = true; }
  else if (p.y > 78 && p.y < 110) { stackCalcSelection = 1; selChanged = true; }
  
  if (selChanged) refreshStackCalcValues(false);
  
  if (adj != 0) {
    if (stackCalcSelection == 0) {
      int delta = (abs(adj) >= 2) ? 2 : 1;
      if (adj < 0) delta = -delta;
      stackStepIndex = constrain(stackStepIndex + delta, 0, STEP_TABLE_LEN - 1);
      stackStepSize = STEP_TABLE[stackStepIndex];
    } else {
      int delta = (abs(adj) >= 2) ? 10 : 1;
      if (adj < 0) delta = -delta;
      stackTotalImgs = constrain(stackTotalImgs + delta, 2, 50000);
    }
    refreshStackCalcValues(false);
  }
  
  if (btnTimeCalc.contains(p.x, p.y)) {
    currentMode = STACK_TIME;
    drawStackTimeUI();
  }
  else if (btnCalcBack.contains(p.x, p.y)) {
    saveStackSettings();
    currentMode = APP_SETTINGS;
    drawAppSettingsUI();
  }
}

void handleStackTimeTouch(TS_Point p, int adj) {
  if (adj != 0) {
    float step = (abs(adj) >= 2) ? 1.0f : 0.1f;
    float direction = (adj > 0) ? 1.0f : -1.0f;
    stackTimePerStep = constrain(stackTimePerStep + direction * step, 0.1f, 60.0f);
    refreshStackTimeValues(false);
  }
  
  if (btnTimeBack.contains(p.x, p.y)) {
    saveStackSettings();
    currentMode = STACK_CALC;
    drawStackCalcUI();
  }
}

// V17: FOV INFO touch handler — single button, just go back
void handleFovInfoTouch(TS_Point p) {
  // X close button — top-right
  if (btnInfoClose.contains(p.x, p.y)) {
    currentMode = MAIN; drawMainScreen(); return;
  }
  // GRAPH button at bottom (btnInfoBack repurposed) OR the old top-right GRAPH button
  if (btnInfoBack.contains(p.x, p.y) || btnInfoGraph.contains(p.x, p.y)) {
    if (!isCustomCalib) {
      nPoints = FACTORY_N;
      pointsCaptured = FACTORY_N;   // V11 fix: CAL_GRAPH bounds the plot by this
      for (int i = 0; i < FACTORY_N; i++) {
        distPoints[i] = FACTORY_DIST[i];
        fovPoints[i]  = FACTORY_FOV[i];
      }
    }
    currentMode = CAL_GRAPH; drawCalGraphUI(); return;
  }
  // POINTS button — open the read-only point list. Seed the factory points so
  // the list is populated on the factory default. Unlike GRAPH, we leave nPoints
  // (the Cal Points target) alone — viewing the list shouldn't change a setting.
  if (btnInfoPoints.contains(p.x, p.y)) {
    if (!isCustomCalib) {
      pointsCaptured = FACTORY_N;
      for (int i = 0; i < FACTORY_N; i++) {
        distPoints[i] = FACTORY_DIST[i];
        fovPoints[i]  = FACTORY_FOV[i];
      }
    }
    calReviewReadOnly = true;
    reviewSelected = -1;
    reviewScrollOffset = 0;
    currentMode = CAL_REVIEW; drawCalReviewUI(); return;
  }
}

// V17: CAL_REVIEW touch handler
void handleCalReviewTouch(TS_Point p) {
  // Read-only viewer (opened from CALIB INFO): scroll + BACK only. No row
  // selection, retake, or finalize — there is no active calibration session.
  if (calReviewReadOnly) {
    if (btnReviewUp.contains(p.x, p.y)) {
      if (reviewScrollOffset > 0) { reviewScrollOffset--; drawCalReviewUI(); }
      return;
    }
    if (btnReviewDown.contains(p.x, p.y)) {
      int maxOffset = pointsCaptured - REVIEW_VISIBLE_ROWS;
      if (maxOffset < 0) maxOffset = 0;
      if (reviewScrollOffset < maxOffset) { reviewScrollOffset++; drawCalReviewUI(); }
      return;
    }
    if (btnPointsBack.contains(p.x, p.y)) {
      calReviewReadOnly = false;
      currentMode = FOV_INFO; drawFovInfoUI();
    }
    return;
  }

  // Scroll buttons
  if (btnReviewUp.contains(p.x, p.y)) {
    if (reviewScrollOffset > 0) {
      reviewScrollOffset--;
      drawCalReviewUI();
    }
    return;
  }
  if (btnReviewDown.contains(p.x, p.y)) {
    int maxOffset = pointsCaptured - REVIEW_VISIBLE_ROWS;
    if (maxOffset < 0) maxOffset = 0;
    if (reviewScrollOffset < maxOffset) {
      reviewScrollOffset++;
      drawCalReviewUI();
    }
    return;
  }

  // List row taps — area x=5..190, y=REVIEW_LIST_TOP..(REVIEW_LIST_TOP+rows*REVIEW_ROW_HEIGHT)
  if (p.x >= 5 && p.x <= 190 &&
      p.y >= REVIEW_LIST_TOP &&
      p.y <  REVIEW_LIST_TOP + REVIEW_VISIBLE_ROWS * REVIEW_ROW_HEIGHT) {
    int row = (p.y - REVIEW_LIST_TOP) / REVIEW_ROW_HEIGHT;
    int idx = reviewScrollOffset + row;
    if (idx >= 0 && idx < pointsCaptured) {
      reviewSelected = (reviewSelected == idx) ? -1 : idx;  // tap-again deselects
      drawCalReviewUI();
    }
    return;
  }

  // Action buttons
  if (btnRetake.contains(p.x, p.y)) {
    if (reviewSelected >= 0 && reviewSelected < pointsCaptured) {
      currentCalIndex = reviewSelected;
      isRetakeMode = true;
      tempPixels = Config::DEFAULT_TEMP_PIXELS;
      currentMode = CAL_RUN;
      drawPointEntryUI();
    }
    return;
  }
  // FINISH — compute the fit now from the captured points.
  if (btnFinalize.contains(p.x, p.y)) {
    if (pointsCaptured >= Config::MIN_CALIB_POINTS) {
      reviewSelected = -1;
      finalizeEarly();
    }
    return;
  }
  if (btnReviewBack.contains(p.x, p.y)) {
    // Resume cal at the next un-captured slot. (pointsCaptured can never be
    // >= nPoints here because finalizeCalibration() runs immediately on the
    // last sequential capture, leaving no path back to CAL_REVIEW.)
    if (pointsCaptured >= nPoints) {
      currentMode = MAIN;
      drawMainScreen();
    } else {
      currentCalIndex = pointsCaptured;
      tempPixels = Config::DEFAULT_TEMP_PIXELS;
      currentMode = CAL_RUN;
      drawPointEntryUI();
    }
    return;
  }
}

// --- Drawing & Logic ---


void drawSignalHealthBar(uint8_t status, float mcps, int x, int y, bool toSprite) {
  // Continuous filled bar — same colour logic as before but smooth (no gaps)
  const int barW = 42, barH = 10;
  uint16_t color = COLOR_RED;
  float fillFrac = 0.0f;

  if (status == 0) {
    // Map mcps 0..50 to 0..1. Colour thresholds match segment logic.
    fillFrac = constrain(mcps / 50.0f, 0.02f, 1.0f);
    if      (fillFrac >= 0.90f) color = COLOR_DARKGREEN;
    else if (fillFrac >= 0.70f) color = COLOR_PUREGREEN;
    else if (fillFrac >= 0.50f) color = COLOR_YELLOW;
    else if (fillFrac >= 0.30f) color = COLOR_ORANGE;
    else                        color = COLOR_RED;
  } else if (status == 11) {
    fillFrac = 0.2f; color = COLOR_ORANGE;
  } else {
    fillFrac = 0.05f; color = COLOR_RED;
  }

  int filled = max(1, (int)(fillFrac * (barW - 2)));

  if (toSprite) {
    menuSprite.fillRect(x, y, barW, barH, THEME_BG);
    menuSprite.drawRect(x, y, barW, barH, COLOR_DARKGREY);
    menuSprite.fillRect(x + 1, y + 1, filled, barH - 2, color);
  } else {
    // #1: the MAIN 30 Hz refresh hits us every frame; only rebuild + blit when
    // the rendered bar actually changes. lastBarFilled is reset to -1 on MAIN
    // entry (drawMainScreen) so the first frame always paints.
    if (filled != lastBarFilled || color != lastBarColor) {
      lastBarFilled = filled; lastBarColor = color;
      barSprite.fillScreen(THEME_BG);
      barSprite.drawRect(0, 0, barW, barH, COLOR_DARKGREY);
      barSprite.fillRect(1, 1, filled, barH - 2, color);
      tft.drawRGBBitmap(x, y, barSprite.getBuffer(), 44, 12);
    }
  }
}

// V12.3 pixel-space calibration: pixels = CTRLX*dist + CTRLY (linear fit), and
// the demarcation's field of view is FOV = demarcation * sensorWidth / pixels.
// Centralises the derivation so every readout uses the same math. Returns the
// base (20x) FOV in mm; callers apply the objective multiplier.
float fovAt(float distMm) {
  float px = CTRLX * distMm + CTRLY;
  return (px > 1.0f) ? (demarcationDist * sensorWidthPixels / px) : 0.0f;
}

// V12.3: recompute the cached fit statistics used by the live prediction
// interval. Call after any change to the calibration points/coefficients.
void computeCalStats() {
  int nn = (pointsCaptured < 0) ? 0 : pointsCaptured;
  gCalN = nn;
  if (nn < 1) { gCalSpx = 0; gCalXbar = 0; gCalSxx = 0; return; }
  float kPx = demarcationDist * sensorWidthPixels;
  float sx = 0;
  for (int i = 0; i < nn; i++) sx += distPoints[i];
  gCalXbar = sx / nn;
  float sxx = 0, sse = 0;
  for (int i = 0; i < nn; i++) {
    float dx = distPoints[i] - gCalXbar;
    sxx += dx * dx;
    float pxMeas = (fovPoints[i] > 1e-4f) ? (kPx / fovPoints[i]) : 0.0f;
    float pxPred = CTRLX * distPoints[i] + CTRLY;
    sse += (pxMeas - pxPred) * (pxMeas - pxPred);
  }
  gCalSxx = sxx;
  gCalSpx = (nn > 2) ? sqrtf(sse / (nn - 2)) : 0.0f;   // pixel residual SD
}

// V12.5 (F4): mirror the current custom calibration into the "calibbak" NVS
// namespace (its own Preferences handle — no collision with the "calib" one).
// Stores exactly the points the fit used (fitN = min(pointsCaptured, nPoints)),
// matching how finalizeCalibration derives its line.
static void saveCalibBackup() {
  int n = (pointsCaptured < nPoints) ? pointsCaptured : nPoints;
  n = constrain(n, 0, 20);
  CalibBackup b = {0};
  b.magic  = CALIBBAK_MAGIC;
  b.width  = sensorWidthPixels;
  b.demarc = demarcationDist;
  b.n      = n;
  for (int i = 0; i < n; i++) { b.dist[i] = distPoints[i]; b.fov[i] = fovPoints[i]; }
  Preferences p;
  p.begin("calibbak", false);
  p.putBytes("bak", &b, sizeof(b));
  p.end();
  Serial.printf("[calib] backup saved (%d pts) to NVS calibbak\n", n);
}
// Drop the backup — the current cal is the built-in default (nothing custom to
// keep) or the user explicitly reset to factory.
static void clearCalibBackup() {
  Preferences p;
  p.begin("calibbak", false);
  p.clear();
  p.end();
}

void finalizeCalibration() {
  tft.fillRect(0, 305, 240, 15, THEME_BG);
  setSmoothFont(1); tft.setTextColor(0x07FF);
  tft.setCursor(5, 318); 
  tft.print("CALIBRATING..."); 

  // Defensive: only iterate over slots that actually hold captured data.
  // Today all call sites guarantee pointsCaptured >= nPoints before invoking
  // us, but a future code path could miss that check and we'd otherwise read
  // uninitialised distPoints[] / fovPoints[] entries.
  int fitN = (pointsCaptured < nPoints) ? pointsCaptured : nPoints;
  // V12.3: fit in PIXEL space. Pixels are linear in distance; FOV (∝ 1/pixels)
  // is not, so the old FOV-vs-distance line systematically under/over-shot over
  // a wide range. Recover each point's measured pixel count from its stored FOV
  // (px = kPx/FOV — the exact inverse of how calCapture made FOV), fit
  // pixels = CTRLX*dist + CTRLY, and derive FOV from that at runtime (fovAt()).
  float kPx = demarcationDist * sensorWidthPixels;       // FOV = kPx / pixels
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < fitN; i++) {
    float px = (fovPoints[i] > 1e-4f) ? (kPx / fovPoints[i]) : 0.0f;
    sumX += distPoints[i]; sumY += px;
    sumXY += (distPoints[i] * px); sumX2 += (distPoints[i] * distPoints[i]);
  }

  float denominator = (fitN * sumX2 - sumX * sumX);
  if (fabs(denominator) < 0.0001) {
    // Degenerate fit → factory defaults (R² clamped <1 so loadCalibration()'s
    // boot heuristic doesn't treat it as uninitialised).
    CTRLX = Config::DEFAULT_CTRL_X; CTRLY = Config::DEFAULT_CTRL_Y; CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; CALIB_R2 = 0.9999f;
  } else {
    CTRLX = (fitN * sumXY - sumX * sumY) / denominator;   // px per mm
    CTRLY = (sumY - CTRLX * sumX) / fitN;                  // px at distance 0
    // R² in pixel space (the quantity actually fit); RMSE in FOV mm so it stays
    // meaningful as the calibration's FOV uncertainty.
    float ssePx = 0, sstPx = 0, sseFov = 0, meanPx = sumY / fitN;
    for (int i = 0; i < fitN; i++) {
      float px      = (fovPoints[i] > 1e-4f) ? (kPx / fovPoints[i]) : 0.0f;
      float predPx  = CTRLX * distPoints[i] + CTRLY;
      ssePx += (px - predPx) * (px - predPx);
      sstPx += (px - meanPx) * (px - meanPx);
      float predFov = (predPx > 1.0f) ? (kPx / predPx) : 0.0f;
      sseFov += (fovPoints[i] - predFov) * (fovPoints[i] - predFov);
    }
    // n-2 dof for a 2-parameter fit; degenerate (fitN < 3) → use 1.
    CALIB_ERROR = sqrt(sseFov / ((fitN > 2) ? (fitN - 2) : 1));   // FOV mm RMSE
    CALIB_R2    = (sstPx > 0.0001) ? (1.0f - (ssePx / sstPx)) : 0.9999f;
  }

  // If the freshly-fit calibration is indistinguishable from the built-in
  // default pixel line (e.g. the user LOADED the default cal back), treat it as
  // default rather than "custom/loaded": the badge reads DEFAULT and the
  // canonical default coefficients are stored so a round-trip can't drift.
  // Pixel-space tolerances (slope ~9 px/mm, intercept ~1669 px).
  bool matchesDefault =
      fabsf(CTRLX - Config::DEFAULT_CTRL_X) < 0.02f &&
      fabsf(CTRLY - Config::DEFAULT_CTRL_Y) < 3.0f;
  if (matchesDefault) {
    CTRLX = Config::DEFAULT_CTRL_X; CTRLY = Config::DEFAULT_CTRL_Y;
    CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; CALIB_R2 = 0.9991f;
  }

  settings.magic = CALIB_MAGIC; settings.ctrlX = CTRLX; settings.ctrlY = CTRLY;
  settings.sensorWidth = sensorWidthPixels; settings.demarcation = demarcationDist;
  settings.calibError = CALIB_ERROR; settings.brightness = currentBrightness;
  settings.isCustom = matchesDefault ? 0 : 1;
  settings.calibR2 = CALIB_R2;        // V17: persist R²

  // V11 fix: persist the scatter points so CAL_GRAPH survives a reboot.
  settings.calNPoints        = nPoints;
  settings.calPointsCaptured = pointsCaptured;
  for (int i = 0; i < 20; i++) {
    settings.calDistPoints[i] = (i < pointsCaptured) ? distPoints[i] : 0.0f;
    settings.calFovPoints[i]  = (i < pointsCaptured) ? fovPoints[i]  : 0.0f;
  }
  
  preferences.begin("calib", false);
  preferences.putBytes("settings", &settings, sizeof(CalibData));
  preferences.end();

  // V12.5 (F4): keep the decoupled backup in sync — save a genuine custom cal,
  // drop it when this fit is the built-in default (so a later magic bump doesn't
  // resurrect a stale custom cal the user has since abandoned).
  if (!matchesDefault) saveCalibBackup();
  else                 clearCalibBackup();

  isCustomCalib = !matchesDefault;
  computeCalStats();            // refresh prediction-interval stats from the new fit
  drawSuccessScreen();
}

// FINISH button (CAL_RUN + CAL_REVIEW): finalize with the points captured so
// far instead of waiting to reach the Cal Points target. The least-squares fit
// in finalizeCalibration() is generic, so any count >= MIN_CALIB_POINTS is a
// valid calibration — the user can stop wherever their bellows range ends.
// Pinning nPoints to the captured count keeps the stored metadata and the
// CAL_GRAPH plot consistent with the points actually used.
void finalizeEarly() {
  if (pointsCaptured < Config::MIN_CALIB_POINTS) return;
  nPoints = pointsCaptured;
  isRetakeMode = false;
  lastCalibName[0] = '\0';      // device cal → generic success (no profile name)
  currentMode = CAL_RUN;        // finalizeCalibration() repaints into CAL_SUCCESS
  finalizeCalibration();
}

void drawMainScreen() {
  tft.fillScreen(THEME_BG);
  updateObjectiveButtons();
  
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  centerStaticText("AVG FOV:", avgFovLabelY, 2);
  centerStaticText("DISTANCE:", distanceLabelY, 1); 

  tft.setFont(); 
  tft.setTextSize(1);
  tft.setTextColor(0xFFE0); 
  tft.setCursor(44, 26); 
  tft.print("TOF int");
  
  // Calib + version block — now a tappable region that opens the ABOUT screen.
  // Subtle dark-grey rectangle hints it's interactive without dominating the
  // main view. Touch zone is registered in handleMainTouch.
  // Box: x=2..96 (~94 wide), y=280..315 (35 tall) — aligned to btnMainCalibrate.
  tft.drawRoundRect(2, 280, 94, 35, UI_RADIUS, 0x2945);
  tft.setFont();
  tft.setTextSize(1);
  tft.setTextColor(isCustomCalib? COLOR_GREENYELLOW : COLOR_RED);
  tft.setCursor(5, 287);
  tft.print(isCustomCalib? "CALIB: LOADED" : "CALIB: DEFAULT");

  tft.setTextColor(themedText(COLOR_DARKGREY));
  tft.setCursor(5, 302);
  tft.print(FIRMWARE_VERSION);           // V11: user-facing build label

  drawGearIcon(217, 22, themedText(COLOR_PUREGREEN), THEME_BG);

  // patched3: WiFi indicator zone — far-left, x=0..32, y=0..42.
  drawWifiIndicator();

  // VIBE CHECK shortcut — center-top gap (x≈90..150), mirrors the web UI's
  // spring-green "VIBE / decaying-wave / CHECK" button. Taps open the
  // vibration hub (VIB_HOME). Built-in 6x8 font, centered on the display
  // midline (x=120), clear of "TOF int" (ends x≈86) and FOV Info (x=152).
  {
    const uint16_t vibCol  = (currentThemeIndex == THEME_DAYLIGHT_IDX)
                               ? 0x03E0 : 0x3ECF;   // #3ddb7a, daylight-darkened
    const uint16_t vibBase = 0x19C6;                // #1f3a33 faint baseline
    tft.setFont(); tft.setTextSize(1);
    tft.setTextColor(vibCol);
    tft.setCursor(108, 3);  tft.print("VIBE");       // 24 px wide → centered on x=120
    tft.setCursor(105, 31); tft.print("CHECK");      // 30 px wide → centered on x=120
    // Decaying ring-down wave between the two labels (baseline y=20).
    tft.drawFastHLine(105, 20, 31, vibBase);
    int prevx = 105, prevy = 20;
    for (int i = 1; i <= 30; i++) {
      int x = 105 + i;
      float amp = 7.0f * (1.0f - (i - 1) / 30.0f);   // 7 px → ~0, left-to-right decay
      int y = 20 - (int)roundf(amp * sinf(i * 0.85f));
      tft.drawLine(prevx, prevy, x, y, vibCol);
      prevx = x; prevy = y;
    }
  }

// V17b: FOV Info button — tightened text spacing.
  // V11: now bracketed by twin bullet dots on each side for visual symmetry.
  // The whole text block also shifted 4px left so the right-side dot doesn't
  // crowd the gear icon at x≈190+. Gap between each dot and the nearest text
  // edge is preserved at ~3px (matching the original left-dot spacing).
  // Button: x=152 y=2 w=44 h=40 (centre x=174, centre y=22).
  // "FOV"  18px wide → startX = 161 (was 165)
  // "Info" 24px wide → startX = 158 (was 162)
  // Bullets centred vertically at y=21, horizontally at x=155 and x=185.
  tft.fillRect(btnFovInfo.x, btnFovInfo.y, btnFovInfo.w, btnFovInfo.h, THEME_BG);
  tft.setFont(); tft.setTextSize(1); tft.setTextColor(themedText(0xC81F));
  tft.setCursor(161, 12); tft.print("FOV");
  // Twin bullets — one left, one right of the FOV/Info stack.
  tft.fillCircle(155, 22, 1, 0xC81F);
  tft.fillCircle(185, 22, 1, 0xC81F);
  tft.setCursor(158, 25); tft.print("Info");

  // Main-screen CALIBRATE button — draw manually with the local FreeSans7pt7b
  // font instead of going through Button::draw / setSmoothFont. Keeps the rest
  // of the UI's font dispatch unchanged; just this one button uses the 7pt face.
  {
    const Button& b = btnMainCalibrate;
    tft.fillRoundRect(b.x, b.y, b.w, b.h, UI_RADIUS, b.boxCol);
    tft.setFont(&FreeSans7pt7b);
    tft.setTextSize(1);
    tft.setTextColor(b.textCol);
    int16_t x1, y1; uint16_t bw, bh;
    tft.getTextBounds(b.label, 0, 0, &x1, &y1, &bw, &bh);
    int cx = b.x + (b.w - bw) / 2 - x1;
    int cy = b.y + (b.h - bh) / 2 - y1;
    tft.setCursor(cx, cy); tft.print(b.label);
    // Restore the default 9pt font so subsequent draws on this screen
    // (which expect setSmoothFont(1) state) aren't surprised.
    tft.setFont(&FreeSans9pt7b);
  }

  bufferFilled = false;
  readIndex = 0;
  totalDist = 0;
  
  lastAvgFov = -1; lastDistance = 0xFFFF; lastDisplayUpdate = 0; lastBarFilled = -1; 
}

void drawAppSettingsUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("SETTINGS", 5, 5, COLOR_DARKBLUE);
  drawOldBrightnessBar();

  btnStackCalc.draw(tft);
  // Calculator icon on STACK CALC (button at x=20 y=127 w=200 h=35).
  // patched3: redrawn — rounded body, screen, 2x3 key grid, tall accent key.
  { int ix = 25, iy = 132, iw = 21, ih = 24;
    tft.drawRoundRect(ix, iy, iw, ih, 2, TFT_WHITE);
    tft.fillRect(ix + 3, iy + 3, iw - 6, 6, COLOR_GREENYELLOW);      // screen
    for (int row = 0; row < 3; row++)
      for (int col = 0; col < 2; col++)
        tft.fillRect(ix + 3 + col * 6, iy + 12 + row * 5, 4, 3, TFT_WHITE);
    tft.fillRect(ix + iw - 6, iy + 12, 4, 13, COLOR_GREENYELLOW);    // tall key
  }

btnBrightness.draw(tft);
  // patched3: sun icon on BRIGHTNESS — was a lightbulb whose X-filament read
  // like a cancel mark.  A disc + 8 rays is the universal brightness glyph.
  // cx=35 matches the calculator-icon centre; cy follows the button (y=172).
  { int cx = 35, cy = 189;
    const float DEG = M_PI / 180.0f;
    tft.fillCircle(cx, cy, 5, 0xFEC0);                 // warm-yellow disc
    tft.drawCircle(cx, cy, 5, COLOR_GREENYELLOW);
    for (int i = 0; i < 8; i++) {
      float a = i * 45.0f * DEG;
      int x0 = cx + (int)roundf( 7.5f * cosf(a));
      int y0 = cy + (int)roundf( 7.5f * sinf(a));
      int x1 = cx + (int)roundf(11.0f * cosf(a));
      int y1 = cy + (int)roundf(11.0f * sinf(a));
      tft.drawLine(x0, y0, x1, y1, COLOR_GREENYELLOW);
    }
  }

btnSettingsMem.draw(tft);
  // V17b: DIMM/RAM-stick icon — green PCB with two memory chips, gold contact
  // pins along the bottom edge (all contained within the PCB width), and a
  // small alignment notch. my follows the MEMORY button (y=217) so it stays
  // vertically centered in the 38px-tall button.
  { int mx = 23, my = 229, mw = 24, mh = 14;
    const uint16_t PCB_GREEN = COLOR_DARKGREEN;
    const uint16_t GOLD      = 0xFEA0;   // RGB565 ≈ #FFD600 — contact-pin gold
    const uint16_t CHIP_BLK  = TFT_BLACK;

    // Green PCB body
    tft.fillRect(mx, my, mw, mh, PCB_GREEN);
    tft.drawRect(mx, my, mw, mh, COLOR_GREENYELLOW); // bright outline

    // Two memory chips on top — symmetric, leave a 2px gutter between
    int chipY = my + 2, chipH = 5;
    tft.fillRect(mx + 2,           chipY, 8, chipH, CHIP_BLK);
    tft.fillRect(mx + mw - 2 - 8,  chipY, 8, chipH, CHIP_BLK);

    // Gold contact strip along the bottom (all pins inside the PCB).
    // Strip occupies the inner 2px-tall row just above the bottom edge.
    int stripY = my + mh - 3;
    tft.fillRect(mx + 1, stripY, mw - 2, 2, GOLD);

    // Black gaps between contacts: every 2px starting 2px in, leaving the
    // gold strip showing through as individual pins. 1px-wide gaps.
    for (int i = 1; i < mw - 2; i += 2) {
      tft.drawFastVLine(mx + 1 + i, stripY, 2, PCB_GREEN);
    }

    // Alignment notch — small black square biting into the contact strip
    // slightly off-center, the way real DIMMs are keyed.
    int notchX = mx + (mw / 2) + 2, notchW = 3;
    tft.fillRect(notchX, stripY, notchW, 2, CHIP_BLK);
  }

  btnVibMonitor.draw(tft);
  // V12: waveform icon on the VIBRATION tile (button at x=20 y=265 w=200 h=34).
  { int wx = 27, wy = 282, prevx = wx, prevy = wy;
    for (int i = 1; i <= 18; i++) {
      int x = wx + i;
      int y = wy - (int)roundf(6.0f * sinf(i * 0.85f));
      tft.drawLine(prevx, prevy, x, y, COLOR_GREENYELLOW);
      prevx = x; prevy = y;
    }
  }

  btnSettingsClose.draw(tft);
}

// drawOldBrightnessBar: sprite-blitted brightness slider in settings.
void drawOldBrightnessBar() {
  int brightnessPct = map(currentBrightness, 1, 255, 1, 100);

  // Label + value — top stripe, y=45..59 (15px)
  distSprite.fillScreen(THEME_BG);
  distSprite.setFont(&FreeSans9pt7b);
  distSprite.setTextSize(1);
  distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
  distSprite.setCursor(15, 13); distSprite.print("BRIGHTNESS");
  char valBuf[8]; snprintf(valBuf, sizeof(valBuf), "%d%%", brightnessPct);
  distSprite.setTextColor(themedText(COLOR_GREENYELLOW));
  int16_t x1, y1; uint16_t w, h;
  distSprite.getTextBounds(valBuf, 0, 0, &x1, &y1, &w, &h);
  distSprite.setCursor(228 - w - x1, 13); distSprite.print(valBuf);
  tft.drawRGBBitmap(0, 52, distSprite.getBuffer(), 240, 20);

  // Slider — blitted at y=80 so the bar sits at screen y≈84..104.
  // Adds breathing room above (from "BRIGHTNESS"/% label) and below
  // (from the STACK CALC button at y=127).
  distSprite.fillScreen(THEME_BG);
  int barW = 210, barH = 20, barX = 15, barY = 4;
  int filled = map(currentBrightness, 1, 255, 0, barW);
  distSprite.fillRoundRect(barX, barY, barW, barH, 2, COLOR_DARKGREY);
  if (filled > 4) distSprite.fillRoundRect(barX, barY, filled, barH, 2, COLOR_GREENYELLOW);
  distSprite.drawRoundRect(barX, barY, barW, barH, 2, COLOR_LIGHTGREY);
  distSprite.fillRect(barX + filled - 2, barY - 2, 4, barH + 4, TFT_WHITE);
  tft.drawRGBBitmap(0, 80, distSprite.getBuffer(), 240, 28);
}

// refreshBrightnessValue removed — brightness now managed in BRIGHTNESS_SETTINGS screen

void drawCalSettingsUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("CALIBRATOR", 5, 5, COLOR_DARKBLUE);
  btnCalInfo.draw(tft);
  refreshCalSettingsValues(true);
  drawAdjButtons(165); 
  
  btnStartCal.draw(tft);
  btnResetAll.draw(tft);
  btnCancelCal.draw(tft);
}

void drawPointEntryUI() {
  tft.fillScreen(THEME_BG);

  // V17: title shows retake mode when applicable
  char ptBuf[24];
  if (isRetakeMode) {
    snprintf(ptBuf, sizeof(ptBuf), "RETAKE PT %d", currentCalIndex + 1);
  } else {
    snprintf(ptBuf, sizeof(ptBuf), "POINT %d/%d", currentCalIndex + 1, nPoints);
  }
  
  tft.setTextColor(isRetakeMode ? COLOR_PURPLE : COLOR_DARKBLUE);
  setSmoothFont(1);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(ptBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(10 - x1, 20 - y1); 
  tft.print(ptBuf);

  tft.fillRect(20, 40, 200, 45, THEME_BG); refreshPixelValue(tempPixels, true);
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  centerStaticText("take photo, input pixel #", 95, 1); centerStaticText("and hit capture & save", 110, 1);
  drawAdjButtons(135); 
  
  drawCaptureSaveButton(btnCapSave, COLOR_GREENYELLOW, TFT_BLACK);
  btnGoBackPt.draw(tft);

  // V17: REVIEW button enabled-look only when there are captured points,
  // and never during a retake (we're already inside one).
  bool reviewEnabled = (pointsCaptured > 0) && !isRetakeMode;
  btnReviewPts.draw(tft, nullptr,
                    reviewEnabled ? COLOR_BLUEGREEN : COLOR_DARKGREY,
                    reviewEnabled ? TFT_WHITE : COLOR_LIGHTGREY);
  // FINISH — finalize early; lit once enough points exist for a valid fit.
  bool finishEnabled = (pointsCaptured >= Config::MIN_CALIB_POINTS) && !isRetakeMode;
  btnFinalize.draw(tft, nullptr,
                   finishEnabled ? COLOR_DARKGREEN : COLOR_DARKGREY,
                   finishEnabled ? TFT_WHITE : COLOR_LIGHTGREY);
  btnCancelPt.draw(tft);
}

void drawConfirmUI() {
  tft.fillScreen(THEME_BG);
  tft.setTextColor(COLOR_MAROON); centerStaticText("WARNING", 40, 2);
  tft.setTextColor(themedText(TFT_WHITE));
  centerStaticText("This will delete all", 90, 1); centerStaticText("calibration progress.", 110, 1);
  
  btnYesDel.draw(tft);
  btnNoKeep.draw(tft);
}

void drawStackCalcUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("STACK CALC", 5, 5, COLOR_DARKBLUE);
  
  // Smaller font for the OBJ label — FreeSans9 butted right up against the
  // "STACK CALC" title box on the 240px TFT; FreeSans7 gives it clearance.
  tft.setFont(&FreeSans7pt7b);
  tft.setTextColor(themedText(COLOR_GREENYELLOW));
  const char* objLabel = (currentobj == 1) ? "5x" : (currentobj == 2) ? "10x" : "20x";
  char objBuf[16];
  snprintf(objBuf, sizeof(objBuf), "OBJ: %s", objLabel);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(objBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(198 - w - x1, 24);   // right-aligned clear of the X close button
  tft.print(objBuf);
  
  refreshStackCalcValues(true);
  drawAdjButtons(207);
  
  btnTimeCalc.draw(tft);
  btnCalcBack.draw(tft);
}

void refreshStackCalcValues(bool force) {
  static int lastSel = -1;
  static float lastStep = -1;
  static int lastImgs = -1;
  static int lastObj = -1;

  if (force) { lastSel = -1; lastStep = -1; lastImgs = -1; lastObj = -1; }
  distSprite.setTextSize(1);
  float NA = (currentobj == 1) ? NA_5X : (currentobj == 2) ? NA_10X : NA_20X;
  float dof_um = WAVELENGTH_UM / (NA * NA);

  if (lastSel != stackCalcSelection || fabsf(lastStep - stackStepSize) > 0.0001f) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(stackCalcSelection == 0 ? COLOR_GREENYELLOW : TFT_WHITE);
    distSprite.setCursor(10, 24);
    distSprite.print("Step Size:");
    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 24);
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%.3f um", stackStepSize);
    distSprite.print(vbuf);
    tft.drawRGBBitmap(0, 45, distSprite.getBuffer(), 240, 30);
  }

  if (lastSel != stackCalcSelection || lastImgs != stackTotalImgs) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(stackCalcSelection == 1 ? COLOR_PURPLE : TFT_WHITE);
    distSprite.setCursor(10, 24);
    distSprite.print("Images:");
    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 24);
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%d", stackTotalImgs);
    distSprite.print(vbuf);
    tft.drawRGBBitmap(0, 78, distSprite.getBuffer(), 240, 30);
  }

  if (lastObj != currentobj || fabsf(lastStep - stackStepSize) > 0.0001f ||
      lastImgs != stackTotalImgs || force) {

    float stackDepth_um = (stackTotalImgs - 1) * stackStepSize;

    float overlap = ((dof_um - stackStepSize) / dof_um) * 100.0f;
    uint16_t overlapColor;
    if      (overlap < 0.0f)  overlapColor = COLOR_RED;
    else if (overlap < 30.0f) overlapColor = COLOR_ORANGE;
    else                      overlapColor = COLOR_PUREGREEN;

    char buf[32];

    // V17b: each info row gets a 28px-tall slot so descenders aren't clipped,
    // and rows are spaced 32px apart so Overlap can't overlap Stack Depth.
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(10, 22);
    snprintf(buf, sizeof(buf), "DOF: %.3f um", dof_um);
    distSprite.print(buf);
    tft.drawRGBBitmap(0, 110, distSprite.getBuffer(), 240, 28);

    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(themedText(COLOR_LIGHTGREY));
    distSprite.setCursor(10, 22);
    snprintf(buf, sizeof(buf), "Stack Depth: %.1f um", stackDepth_um);
    distSprite.print(buf);
    tft.drawRGBBitmap(0, 142, distSprite.getBuffer(), 240, 28);

    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(overlapColor);
    distSprite.setCursor(10, 22);
    snprintf(buf, sizeof(buf), "Overlap: %.0f%%", overlap);
    distSprite.print(buf);
    tft.drawRGBBitmap(0, 174, distSprite.getBuffer(), 240, 28);

    lastObj = currentobj;
  }

  lastStep = stackStepSize;
  lastImgs = stackTotalImgs;
  lastSel  = stackCalcSelection;
}

void drawStackTimeUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("STACK TIME", 5, 5, COLOR_DARKBLUE);

  setSmoothFont(1);
  char buf[32];
  int16_t x1, y1; uint16_t w, h;

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  snprintf(buf, sizeof(buf), "Step: %.3f um", stackStepSize);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2 - x1, 58);
  tft.print(buf);

  float calcDepth = (stackTotalImgs - 1) * stackStepSize;
  snprintf(buf, sizeof(buf), "Depth: %.1f um", calcDepth);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2 - x1, 76);
  tft.print(buf);

  snprintf(buf, sizeof(buf), "IMGS: %d", stackTotalImgs);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2 - x1, 94);
  tft.print(buf);

  refreshStackTimeValues(true);

  btnTimeBack.draw(tft);
}

void refreshStackTimeValues(bool force) {
  distSprite.setTextSize(1);
  static float lastTime = -1;
  static int lastImgs = -1;

  if (force) { lastTime = -1; lastImgs = -1; }

  bool timeChanged = (fabsf(lastTime - stackTimePerStep) > 0.01f);
  if (force || timeChanged) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(themedText(COLOR_GREENYELLOW));
    distSprite.setCursor(10, 24);
    distSprite.print("Sec/Step:");
    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 24);
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%.1f s", stackTimePerStep);
    distSprite.print(vbuf);
    tft.drawRGBBitmap(0, 116, distSprite.getBuffer(), 240, 35);
    lastTime = stackTimePerStep;
  }

  if (force) {
    drawAdjButtons(155);
  }

  if (force || timeChanged || lastImgs != stackTotalImgs) {
    long totalSec = (long)roundf((float)stackTotalImgs * stackTimePerStep);
    int hours = totalSec / 3600;
    int mins  = (totalSec % 3600) / 60;
    int secs  = totalSec % 60;

    char timeBuf[32];
    if (hours > 0)     snprintf(timeBuf, sizeof(timeBuf), "%dh %dm %ds", hours, mins, secs);
    else if (mins > 0) snprintf(timeBuf, sizeof(timeBuf), "%dm %ds", mins, secs);
    else               snprintf(timeBuf, sizeof(timeBuf), "%ds", secs);

    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(COLOR_PURPLE);
    int16_t lx, ly; uint16_t lw, lh;
    distSprite.getTextBounds("Calculated Stack Time", 0, 0, &lx, &ly, &lw, &lh);
    distSprite.setCursor((240 - lw) / 2 - lx, 22);
    distSprite.print("Calculated Stack Time");
    tft.drawRGBBitmap(0, 202, distSprite.getBuffer(), 240, 28);

    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(COLOR_ORANGE);
    int16_t tx, ty; uint16_t tw, th;
    distSprite.getTextBounds(timeBuf, 0, 0, &tx, &ty, &tw, &th);
    distSprite.setCursor((240 - tw) / 2 - tx, 24);
    distSprite.print(timeBuf);
    tft.drawRGBBitmap(0, 234, distSprite.getBuffer(), 240, 35);

    lastImgs = stackTotalImgs;
  }

  // V12.3: measured sec/step from the last completed stack. Auto-applies above
  // on >=30 s drift, so this line just makes the source of any change visible.
  static float lastMeasuredShown = -2.0f;
  if (force || fabsf(lastMeasuredShown - measuredPerStep) > 0.01f) {
    char mbuf[40];
    if (measuredPerStep > 0.0f)
      snprintf(mbuf, sizeof(mbuf), "Measured: %.1f s/step (%u sh)",
               measuredPerStep, (unsigned)measuredPulses);
    else
      snprintf(mbuf, sizeof(mbuf), "Measured: run a stack to set");

    distSprite.fillScreen(THEME_BG);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(measuredPerStep > 0.0f ? themedText(COLOR_TEAL)
                                                   : themedText(COLOR_LIGHTGREY));
    int16_t mx, my; uint16_t mw, mh;
    distSprite.getTextBounds(mbuf, 0, 0, &mx, &my, &mw, &mh);
    distSprite.setCursor((240 - mw) / 2 - mx, 20);
    distSprite.print(mbuf);
    tft.drawRGBBitmap(0, 276, distSprite.getBuffer(), 240, 28);
    lastMeasuredShown = measuredPerStep;
  }
}

// V17: FOV calibration info screen
void drawFovInfoUI() {
  tft.fillScreen(THEME_BG);
  drawLeftBoxedText("CALIB INFO", 5, 5, COLOR_DARKBLUE);

  setSmoothFont(1);

  char buf[32];

  // Status line — top, color coded
  tft.setTextColor(isCustomCalib ? COLOR_GREENYELLOW : COLOR_RED);
  const char* statusTxt = isCustomCalib ? "Status: CUSTOM CAL" : "Status: FACTORY DEFAULT";
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(statusTxt, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2 - x1, 60);
  tft.print(statusTxt);

  // Linear fit values
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 95);
  tft.print("Slope:");
  tft.setTextColor(themedText(TFT_WHITE));
  snprintf(buf, sizeof(buf), "%.5f", CTRLX);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 95);
  tft.print(buf);

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 120);
  tft.print("Intercept:");
  tft.setTextColor(themedText(TFT_WHITE));
  snprintf(buf, sizeof(buf), "%.4f", CTRLY);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 120);
  tft.print(buf);

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 145);
  tft.print("Base Err:");
  // R² color: green if good, yellow if mediocre, red if bad
  // CALIB_ERROR is the std error of the fit in mm — smaller is better.
  uint16_t errCol = TFT_WHITE;
  if (isCustomCalib) {
    if      (CALIB_ERROR < 0.02f) errCol = COLOR_PUREGREEN;
    else if (CALIB_ERROR < 0.05f) errCol = COLOR_YELLOW;
    else                          errCol = COLOR_ORANGE;
  }
  tft.setTextColor(errCol);
  snprintf(buf, sizeof(buf), "%.4f mm", CALIB_ERROR);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 145);
  tft.print(buf);

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 170);
  tft.print("R-Squared:");
  uint16_t r2Col = TFT_WHITE;
  if (isCustomCalib) {
    if      (CALIB_R2 > 0.995f) r2Col = COLOR_PUREGREEN;
    else if (CALIB_R2 > 0.98f)  r2Col = COLOR_YELLOW;
    else                        r2Col = COLOR_ORANGE;
  }
  tft.setTextColor(r2Col);
  snprintf(buf, sizeof(buf), "%.4f", CALIB_R2);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 170);
  tft.print(buf);

  // Inputs used
  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 200);
  tft.print("Photo Width:");
  tft.setTextColor(themedText(TFT_WHITE));
  snprintf(buf, sizeof(buf), "%d px", (int)sensorWidthPixels);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 200);
  tft.print(buf);

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 225);
  tft.print("Demarc:");
  tft.setTextColor(themedText(TFT_WHITE));
  snprintf(buf, sizeof(buf), "%.2f mm", demarcationDist);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 225);
  tft.print(buf);

  tft.setTextColor(themedText(COLOR_LIGHTGREY));
  tft.setCursor(20, 250);
  tft.print("Cal Points:");
  tft.setTextColor(themedText(TFT_WHITE));
  snprintf(buf, sizeof(buf), "%d", nPoints);
  tft.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(225 - w - x1, 250);
  tft.print(buf);

  btnInfoClose.draw(tft);   // X close top-right
  btnInfoBack.draw(tft);    // GRAPH button (bottom-left)
  btnInfoPoints.draw(tft);  // POINTS button (bottom-right) → read-only point list
}

// V17: calibration point review/edit screen
void drawCalReviewUI() {
  tft.fillScreen(THEME_BG);

  // Title with point count (read-only viewer vs editable review)
  char titleBuf[24];
  if (calReviewReadOnly) snprintf(titleBuf, sizeof(titleBuf), "POINTS (%d)", pointsCaptured);
  else                   snprintf(titleBuf, sizeof(titleBuf), "REVIEW (%d pts)", pointsCaptured);
  drawLeftBoxedText(titleBuf, 5, 5, COLOR_DARKBLUE);

  setSmoothFont(1);

  if (pointsCaptured == 0) {
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    centerStaticText("No points captured yet", 130, 1);
  } else {
    // Render visible rows
    int rowsToShow = pointsCaptured - reviewScrollOffset;
    if (rowsToShow > REVIEW_VISIBLE_ROWS) rowsToShow = REVIEW_VISIBLE_ROWS;

    for (int r = 0; r < rowsToShow; r++) {
      int idx = reviewScrollOffset + r;
      int rowY = REVIEW_LIST_TOP + r * REVIEW_ROW_HEIGHT;
      bool selected = (!calReviewReadOnly && idx == reviewSelected);

      uint16_t bgCol = selected ? COLOR_DARKBLUE : THEME_BG;
      uint16_t fgCol = selected ? COLOR_GREENYELLOW : themedText(TFT_WHITE);

      // Row background — leave gap on right for scroll buttons (x=195+)
      tft.fillRect(5, rowY, 185, REVIEW_ROW_HEIGHT - 2, bgCol);

      // Row text: "Pt N: D=DD.Dmm  F=X.XX" — 0.1 mm now that capture is sub-mm.
      char rowBuf[40];
      snprintf(rowBuf, sizeof(rowBuf), "Pt%d: D=%.1fmm F=%.2f",
               idx + 1,
               distPoints[idx],
               fovPoints[idx]);
      tft.setTextColor(fgCol);
      tft.setCursor(10, rowY + 18);
      tft.print(rowBuf);
    }

    // Scroll buttons — only enabled when scroll is possible
    bool canUp = (reviewScrollOffset > 0);
    bool canDown = (reviewScrollOffset + REVIEW_VISIBLE_ROWS) < pointsCaptured;
    btnReviewUp.draw(tft, nullptr,
                     canUp ? COLOR_BLUEGREEN : COLOR_DARKGREY,
                     canUp ? TFT_WHITE : COLOR_LIGHTGREY);
    btnReviewDown.draw(tft, nullptr,
                       canDown ? COLOR_BLUEGREEN : COLOR_DARKGREY,
                       canDown ? TFT_WHITE : COLOR_LIGHTGREY);

    // Hint text below list, above buttons
    tft.setTextColor(themedText(COLOR_LIGHTGREY));
    if (calReviewReadOnly) {
      centerStaticText(isCustomCalib ? "Current calibration points"
                                     : "Factory default points", 245, 1);
    } else if (reviewSelected >= 0) {
      char hintBuf[40];  // room for "Selected pt NN — tap RETAKE" (em-dash is 3 bytes UTF-8)
      snprintf(hintBuf, sizeof(hintBuf), "Selected pt %d — tap RETAKE", reviewSelected + 1);
      centerStaticText(hintBuf, 245, 1);
    } else {
      centerStaticText("Tap a row to select", 245, 1);
    }
  }

  // Action buttons. Read-only viewer (from CALIB INFO) shows just BACK; the
  // editable review shows RETAKE / FINISH / BACK.
  if (calReviewReadOnly) {
    btnPointsBack.draw(tft);
  } else {
    bool retakeEnabled = (reviewSelected >= 0);
    btnRetake.draw(tft, nullptr,
                   retakeEnabled ? 0x001F : COLOR_DARKGREY,
                   retakeEnabled ? TFT_WHITE : COLOR_LIGHTGREY);
    // FINISH — finalize the calibration from the captured points (>= 3).
    bool finishEnabled = (pointsCaptured >= Config::MIN_CALIB_POINTS);
    btnFinalize.draw(tft, nullptr,
                     finishEnabled ? COLOR_DARKGREEN : COLOR_DARKGREY,
                     finishEnabled ? TFT_WHITE : COLOR_LIGHTGREY);
    btnReviewBack.draw(tft);
  }
}

void refreshCalSettingsValues(bool force) {
  static int lastSelection = -1;
  static float lastWidth = -1;
  static float lastDemarc = -1;
  static int lastPoints = -1;

  if (force) {
    lastSelection = -1; lastWidth = -1; lastDemarc = -1; lastPoints = -1;
  }

  if (lastSelection != calibSelection || lastWidth != sensorWidthPixels) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setTextSize(1);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(calibSelection == 0 ? COLOR_GREENYELLOW : TFT_WHITE);
    distSprite.setCursor(10, 24); 
    distSprite.print("Photo Width:");

    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 26);   // +2 baseline → vertically centre value in the 35px row
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%d px", (int)sensorWidthPixels);
    distSprite.print(vbuf);

    tft.drawRGBBitmap(0, 50, distSprite.getBuffer(), 240, 35);
    lastWidth = sensorWidthPixels;
  }

  if (lastSelection != calibSelection || fabsf(lastDemarc - demarcationDist) > 0.001f) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setTextSize(1);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(calibSelection == 1 ? COLOR_PURPLE : TFT_WHITE);
    distSprite.setCursor(10, 24);
    distSprite.print("Demarc Dist:");

    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 26);   // +2 baseline → vertically centre value in the 35px row
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%.2f mm", demarcationDist);
    distSprite.print(vbuf);

    tft.drawRGBBitmap(0, 85, distSprite.getBuffer(), 240, 35);
    lastDemarc = demarcationDist;
  }

  if (lastSelection != calibSelection || lastPoints != nPoints) {
    distSprite.fillScreen(THEME_BG);
    distSprite.setTextSize(1);
    distSprite.setFont(&FreeSans9pt7b);
    distSprite.setTextColor(calibSelection == 2 ? 0x001F : TFT_WHITE);
    distSprite.setCursor(10, 24);
    distSprite.print("Cal Points:");

    distSprite.setFont(&FreeSans12pt7b);
    distSprite.setTextColor(themedText(TFT_WHITE));
    distSprite.setCursor(115, 26);   // +2 baseline → vertically centre value in the 35px row
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%d", nPoints);
    distSprite.print(vbuf);

    tft.drawRGBBitmap(0, 120, distSprite.getBuffer(), 240, 35);
    lastPoints = nPoints;
  }
  
  lastSelection = calibSelection;
}

void setSmoothFont(uint8_t size) {
  tft.setTextSize(1); fovSprite.setTextSize(1); distSprite.setTextSize(1);
  const GFXfont* font;
  if (size == 1) font = &FreeSans9pt7b;
  else if (size == 2) font = &FreeSans12pt7b;
  else if (size == 3) font = &FreeSans18pt7b;
  else font = &FreeSans24pt7b;
  tft.setFont(font); fovSprite.setFont(font); distSprite.setFont(font);
}

void drawLeftBoxedText(const char* txt, int x, int y, uint16_t bgCol) {
  setSmoothFont(1);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  int boxH = 28; 
  // V11: theme override — when a non-CLASSIC theme is active, every screen's
  // title stripe paints in the theme accent so the theming reads at a glance.
  // CLASSIC (index 0) keeps each caller's per-screen bgCol so the original
  // look is preserved exactly.
  uint16_t fillCol = (currentThemeIndex == 0) ? bgCol : THEME_TITLE_BG;
  int boxW = w + 16;
  tft.fillRoundRect(x, y, boxW, boxH, UI_RADIUS, fillCol);
  // patched3 UI polish: bevel — brighter top edge, darker bottom edge.
  tft.drawFastHLine(x + UI_RADIUS, y + 1,        boxW - 2 * UI_RADIUS, scaleColor565(fillCol, 9, 5));
  tft.drawFastHLine(x + UI_RADIUS, y + boxH - 2, boxW - 2 * UI_RADIUS, scaleColor565(fillCol, 1, 2));
  // patched3 UI polish: engraved two-tone title — a dark shadow glyph offset
  // by 1px under the bright cyan title gives the text a chiselled look.
  tft.setCursor(x + 9, y + 21);
  tft.setTextColor(scaleColor565(fillCol, 1, 3)); tft.print(txt);
  tft.setCursor(x + 8, y + 20);
  tft.setTextColor(0x07FF); tft.print(txt);
}

void drawGearIcon(int cx, int cy, uint16_t color, uint16_t bgCol) {
  // 8-tooth gear: trapezoidal teeth following radial direction.
  // Body circle (r=7) fills root; teeth extend to r=11 as trapezoids.
  // Shoulders ±14° at r=7; tips ±7° at r=11 → clean taper toward tip.
  const float DEG = M_PI / 180.0f;
  tft.fillCircle(cx, cy, 7, color);
  for (int i = 0; i < 8; i++) {
    float a = i * 45.0f * DEG;
    int x0 = cx + (int)roundf( 7.0f * cosf(a - 14.0f*DEG));
    int y0 = cy + (int)roundf( 7.0f * sinf(a - 14.0f*DEG));
    int x1 = cx + (int)roundf(11.0f * cosf(a -  7.0f*DEG));
    int y1 = cy + (int)roundf(11.0f * sinf(a -  7.0f*DEG));
    int x2 = cx + (int)roundf(11.0f * cosf(a +  7.0f*DEG));
    int y2 = cy + (int)roundf(11.0f * sinf(a +  7.0f*DEG));
    int x3 = cx + (int)roundf( 7.0f * cosf(a + 14.0f*DEG));
    int y3 = cy + (int)roundf( 7.0f * sinf(a + 14.0f*DEG));
    tft.fillTriangle(x0, y0, x1, y1, x2, y2, color);
    tft.fillTriangle(x0, y0, x2, y2, x3, y3, color);
  }
  tft.fillCircle(cx, cy, 3, bgCol);
}


void drawCaptureSaveButton(Button& btn, uint16_t boxCol, uint16_t textCol) {
  tft.fillRect(btn.x, btn.y, btn.w, btn.h, boxCol); 
  setSmoothFont(1); tft.setTextColor(textCol); 
  int16_t x1, y1; uint16_t w1, h1;
  tft.getTextBounds("CAPTURE", 0, 0, &x1, &y1, &w1, &h1);
  int fh = h1 + 6; int totalH = fh * 3; int startY = btn.y + (btn.h - totalH) / 2 - y1;
  
  tft.setCursor(btn.x + (btn.w - w1) / 2 - x1, startY); tft.print("CAPTURE");
  tft.getTextBounds("&", 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(btn.x + (btn.w - w1) / 2 - x1, startY + fh); tft.print("&");
  tft.getTextBounds("SAVE", 0, 0, &x1, &y1, &w1, &h1);
  tft.setCursor(btn.x + (btn.w - w1) / 2 - x1, startY + fh * 2); tft.print("SAVE");
}

void centerStaticText(const char* txt, int y, uint8_t size) {
  setSmoothFont(size);
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(txt, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor((240 - w) / 2 - x1, y - y1); 
  tft.print(txt);
}

void refreshPixelValue(int p, bool force) {
  static int lastP = -1;
  if (!force && lastP == p) return;
  lastP = p;

  setSmoothFont(3);
  tft.setTextColor(themedText(TFT_WHITE));

  if (force) {
    tft.fillRect(20, 40, 200, 45, THEME_BG);
    tft.setCursor(140, 75);
    tft.print("px");
  }

  tft.fillRect(20, 40, 115, 45, THEME_BG);
  
  char pxBuf[16]; 
  snprintf(pxBuf, sizeof(pxBuf), "%d", p);
  
  int16_t x1, y1; uint16_t w, h;
  tft.getTextBounds(pxBuf, 0, 0, &x1, &y1, &w, &h);
  tft.setCursor(130 - w - x1, 75); 
  tft.print(pxBuf);
}

void resetToFactory() {
  CTRLX = Config::DEFAULT_CTRL_X; CTRLY = Config::DEFAULT_CTRL_Y;
  sensorWidthPixels = Config::DEFAULT_SENSOR_WIDTH_PX; demarcationDist = Config::DEFAULT_DEMARCATION_MM;
  nPoints = 10;  // sensible default; user can adjust between MIN_CALIB_POINTS and MAX_CALIB_POINTS
  CALIB_ERROR = Config::DEFAULT_CALIB_ERROR; isCustomCalib = false;
  CALIB_R2 = 0.9991f;           // factory-default R² (matches boot-load fallback)
  pointsCaptured = 0;          // V17: clear captured points
  isRetakeMode = false;

  // Persist the factory-default calibration WITH a valid magic. The old
  // approach (magic = 0, discard at next boot) had a trap: every later
  // settings save rewrites the whole struct without restoring the magic, so
  // brightness/LED/stack tweaks made after a RESET ALL were silently thrown
  // away at the next power cycle. Non-calibration prefs keep their current
  // values — this reset is about the calibration, not the whole device.
  settings.magic = CALIB_MAGIC;
  settings.ctrlX = CTRLX; settings.ctrlY = CTRLY;
  settings.sensorWidth = sensorWidthPixels; settings.demarcation = demarcationDist;
  settings.calibError = CALIB_ERROR; settings.calibR2 = CALIB_R2;
  settings.isCustom = 0;
  settings.calNPoints = 0;
  settings.calPointsCaptured = 0;
  settings.brightness = currentBrightness;
  settings.stackStepSize = stackStepSize;
  settings.stackTotalDepth = (float)stackTotalImgs;
  settings.stackTimePerStep = stackTimePerStep;
  settings.ledEnabled = ledEnabled ? 1 : 0;
  settings.ledDuty = (uint8_t)constrain(currentLedDuty, 0, 127);
  settings._pad = 0;
  preferences.begin("calib", false);
  preferences.putBytes("settings", &settings, sizeof(CalibData));
  preferences.end();
  clearCalibBackup();   // V12.5 (F4): deliberate reset — drop the custom backup

  refreshCalSettingsValues(true);
}

// Advances the 5-sample rolling distance average and the 5 s FOV-error window,
// then publishes sensorAvgDist / sensorErrInt for the web telemetry. Runs from
// loop() on the 30 ms display cadence on EVERY screen — previously this lived
// inside updateDisplay() (MAIN only), so the dashboard's FOV and error froze
// whenever the TFT was on any other screen, and stack-complete cards could
// record a stale FOV. updateDisplay() now just renders these results.
void updateSensorAverages() {
  uint32_t currentState = sensorState.load(std::memory_order_acquire);
  if (!((currentState >> 31) & 0x1)) return;
  int currentRange = currentState & 0x7FFFFFFF;

  if (!bufferFilled) {
    for(int i = 0; i < numReadings; i++) readings[i] = currentRange;
    totalDist = currentRange * numReadings;
    bufferFilled = true;
  }

  totalDist -= readings[readIndex];
  readings[readIndex] = currentRange;
  totalDist += readings[readIndex];
  readIndex = (readIndex + 1) % numReadings;
  averageDist = (float)totalDist / numReadings;
  sensorAvgDist.store((uint32_t)roundf(averageDist * 10.0f), std::memory_order_release);

  float mult = (currentobj == 1)? mul_5x : (currentobj == 2)? mul_10x : mul_20x;
  float fovBase = fovAt(averageDist);   // 20x base FOV — objective-independent

  // ── Error = 2σ of the AVG FOV over the last ~5 seconds ──
  // A direct empirical measure of how much the reading actually moves (sensor
  // jitter + slow drift) — not a model estimate, not averaged down by √N.
  // We track the OBJECTIVE-INDEPENDENT base FOV so switching objectives doesn't
  // inject a fake jump into the window, then scale the spread by the current
  // multiplier so the ± lands in the displayed FOV's units. Sampled ~20 Hz; the
  // window is a true 5 s; builds up once ≥3 samples are in range.
  uint32_t nowMs = millis();
  if (nowMs - fovErrLastSample >= 50) {
    fovErrLastSample = nowMs;
    fovErrWin[fovErrHead] = fovBase; fovErrWinT[fovErrHead] = nowMs;
    fovErrHead = (fovErrHead + 1) % FOV_ERR_WIN;
    if (fovErrCount < FOV_ERR_WIN) fovErrCount++;
  }
  float fovErrorBound = 0.0f;
  {
    float sum = 0; int cnt = 0;
    for (int i = 0; i < fovErrCount; i++)
      if (nowMs - fovErrWinT[i] <= 5000) { sum += fovErrWin[i]; cnt++; }
    if (cnt >= 3) {
      float mean = sum / cnt, var = 0;
      for (int i = 0; i < fovErrCount; i++)
        if (nowMs - fovErrWinT[i] <= 5000) { float dv = fovErrWin[i] - mean; var += dv * dv; }
      fovErrorBound = 2.0f * mult * sqrtf(var / (cnt - 1));   // 2σ, scaled to displayed units
    }
  }

  int errInt = round(fovErrorBound * 100.0);
  if (errInt < 1) errInt = 1;
  sensorErrInt.store((uint32_t)errInt, std::memory_order_release);
}

void updateDisplay() {
  uint32_t currentState = sensorState.load(std::memory_order_acquire);
  bool rangeValid = (currentState >> 31) & 0x1;
  int currentRange = currentState & 0x7FFFFFFF;

  if (rangeValid) {
    float mult = (currentobj == 1)? mul_5x : (currentobj == 2)? mul_10x : mul_20x;
    float fov = mult * fovAt(averageDist);
    int errInt = (int)sensorErrInt.load(std::memory_order_acquire);

    static int lastErrInt = -1;
    static unsigned long lastTextUpdate = 0;

    if (millis() - lastTextUpdate > 600 || lastDistance == 0xFFFF) {
      if (fabs(fov - lastAvgFov) > 0.01 || errInt != lastErrInt) {
        fovSprite.fillScreen(THEME_BG); 
        
        char baseFov[16];
        char errStr[16];
        snprintf(baseFov, sizeof(baseFov), "%.2f", fov);
        snprintf(errStr, sizeof(errStr), "(%d)", errInt);

        int16_t bx, by; uint16_t bw, bh;
        int baseW = 0, errW = 0, mmW = 0;

        setSmoothFont(4); fovSprite.getTextBounds(baseFov, 0, 0, &bx, &by, &bw, &bh); baseW = bw;
        setSmoothFont(3); fovSprite.getTextBounds(errStr,  0, 0, &bx, &by, &bw, &bh); errW  = bw;
                          fovSprite.getTextBounds("mm",    0, 0, &bx, &by, &bw, &bh); mmW   = bw;

        int totalW = baseW + 8 + errW + 8 + mmW;
        int startX = (240 - totalW) / 2;

        // ── Vertical positioning: derived from exact FreeSans glyph data ───
        // FreeSans24 digit at cursor 42: pixel span y = [10, 43]  (yOff=-32, h=33)
        // FreeSans18 "(" at cursor 35:  pixel span y = [10, 43]  (yOff=-25, h=33)
        //   → both spans are identical; top-align parens = bottom-align parens.
        // FreeSans18 digit at cursor 38: pixel span y = [14, 39]  (yOff=-24, h=25)
        //   → centred inside [10,43] with 4 px margin top and bottom.
        const int FOV_BASE = 42;
        const int PAREN_Y  = 35;
        const int NUM_Y    = 38;

        fovSprite.setTextColor(themedText(COLOR_GREENYELLOW));

        int errStartX = startX + baseW + 8;
        setSmoothFont(4); fovSprite.setCursor(startX, FOV_BASE);                   fovSprite.print(baseFov);
        setSmoothFont(3); fovSprite.setCursor(errStartX, PAREN_Y);                 fovSprite.print("(");
                          fovSprite.setCursor(fovSprite.getCursorX(), NUM_Y);      fovSprite.print(errInt);
                          fovSprite.setCursor(fovSprite.getCursorX(), PAREN_Y);    fovSprite.print(")");
                          fovSprite.setCursor(errStartX + errW + 8, FOV_BASE);     fovSprite.print("mm");
        
        tft.drawRGBBitmap(0, avgFovValueY, fovSprite.getBuffer(), 240, 48);
        lastAvgFov = fov; lastErrInt = errInt;
      }
      
      if (currentRange != lastDistance) {
        distSprite.fillScreen(THEME_BG); 
        
        char distNum[16];
        snprintf(distNum, sizeof(distNum), "%d", currentRange);
        
        int16_t bx, by; uint16_t bw, bh;
        int numW_d = 0, mmW_d = 0;
        
        setSmoothFont(3); distSprite.getTextBounds(distNum, 0, 0, &bx, &by, &bw, &bh); numW_d = bw;
        setSmoothFont(2); distSprite.getTextBounds("mm", 0, 0, &bx, &by, &bw, &bh); mmW_d = bw;

        int startX_d = (240 - (numW_d + 8 + mmW_d)) / 2;
        
        distSprite.setTextColor(0x07FF); 
        
        setSmoothFont(3); distSprite.setCursor(startX_d, 32); distSprite.print(distNum);
        setSmoothFont(2); distSprite.setCursor(startX_d + numW_d + 8, 32); distSprite.print("mm");
        
        tft.drawRGBBitmap(0, distanceValueY, distSprite.getBuffer(), 240, 45);
        lastDistance = currentRange;
      }
      
      lastTextUpdate = millis();
    }
  } else {
    if (lastDistance != 0xFFFF) {
      lastDistance = 0xFFFF;
      lastAvgFov = -1;
      
      int16_t bx, by; uint16_t bw, bh;
      
      fovSprite.fillScreen(THEME_BG); fovSprite.setTextColor(themedText(COLOR_DARKGREY)); 
      setSmoothFont(4); fovSprite.getTextBounds("---", 0, 0, &bx, &by, &bw, &bh);
      fovSprite.setCursor((240 - bw) / 2 - bx, 42); fovSprite.print("---"); 
      tft.drawRGBBitmap(0, avgFovValueY, fovSprite.getBuffer(), 240, 48);
      
      distSprite.fillScreen(THEME_BG); distSprite.setTextColor(themedText(COLOR_DARKGREY)); 
      setSmoothFont(3); distSprite.getTextBounds("---", 0, 0, &bx, &by, &bw, &bh);
      distSprite.setCursor((240 - bw) / 2 - bx, 42); distSprite.print("---"); 
      tft.drawRGBBitmap(0, distanceValueY, distSprite.getBuffer(), 240, 45);
    }
  }
}

void updateObjectiveButtons() {
  // Update stored highlight colours (used below and by any external readers).
  btnObj5x.boxCol  = (currentobj == 1) ? 0xF800 : MUTED_5X;
  btnObj10x.boxCol = (currentobj == 2) ? 0x7BEF : MUTED_10X;
  btnObj20x.boxCol = (currentobj == 3) ? 0x001F : MUTED_20X;

  // ── Composite into objSprite, blit once ────────────────────────────────
  // All three fill→text→ring steps happen in RAM; a single drawRGBBitmap
  // replaces the previous multi-step direct-TFT sequence that caused ~60 Hz
  // flicker visible on the main screen.
  //
  // Sprite origin maps to TFT (0, Y_POS-1):
  //   sprite y = 0  ↔  TFT y = Y_POS-1 = 199   (ring top margin)
  //   sprite y = 1  ↔  TFT y = Y_POS    = 200   (button fill top)
  const int SY = Y_POS - 1;  // TFT row of sprite top
  const int BY = 1;           // button-fill y inside sprite

  objSprite.fillScreen(THEME_BG);

  // Labels use the same font as Button::draw() with textSize=2.
  objSprite.setFont(&FreeSans12pt7b);
  objSprite.setTextSize(1);
  objSprite.setTextColor(TFT_WHITE);

  Button* btns[3] = { &btnObj5x, &btnObj10x, &btnObj20x };
  for (int i = 0; i < 3; i++) {
    Button* b = btns[i];
    // Fill
    objSprite.fillRoundRect(b->x, BY, b->w, b->h, UI_RADIUS, b->boxCol);
    // Centred label (same centering math as Button::draw)
    int16_t x1, y1; uint16_t bw, bh;
    objSprite.getTextBounds(b->label, 0, 0, &x1, &y1, &bw, &bh);
    objSprite.setCursor(b->x + (b->w - bw) / 2 - x1,
                        BY  + (b->h - bh) / 2 - y1);
    objSprite.print(b->label);
  }

  // Selection ring — sprite is already THEME_BG so no erase rings needed;
  // just draw white on the active button.
  int rx = (currentobj == 1) ? btnObj5x.x : (currentobj == 2) ? btnObj10x.x : btnObj20x.x;
  objSprite.drawRoundRect(rx - 1, 0, BOX_SIZE + 2, BOX_SIZE + 2, UI_RADIUS + 1, TFT_WHITE);

  tft.drawRGBBitmap(0, SY, objSprite.getBuffer(), 240, 67);
}

void drawAdjButtons(int y) {
  tft.fillRect(10,  y, 50, 40, COLOR_RED);       // --
  tft.fillRect(65,  y, 50, 40, COLOR_DARKGREY);  // -
  tft.fillRect(120, y, 50, 40, COLOR_DARKGREY);  // +
  tft.fillRect(175, y, 50, 40, COLOR_DARKGREEN); // ++

  // Black triangles, apex pointing the direction of change: double chevrons
  // for the accelerated --/++ buttons, single for the fine -/+ buttons. Each
  // button is 50 wide; its centre is bx+25.
  const int cy = y + 20;
  // -- : two left-pointing triangles (centre x = 35)
  tft.fillTriangle(35 - 11, cy, 35 - 2,  cy - 8, 35 - 2,  cy + 8, TFT_BLACK);
  tft.fillTriangle(35 + 1,  cy, 35 + 10, cy - 8, 35 + 10, cy + 8, TFT_BLACK);
  // -  : one left-pointing triangle (centre x = 90)
  tft.fillTriangle(90 - 7,  cy, 90 + 6,  cy - 9, 90 + 6,  cy + 9, TFT_BLACK);
  // +  : one right-pointing triangle (centre x = 145)
  tft.fillTriangle(145 + 7, cy, 145 - 6, cy - 9, 145 - 6, cy + 9, TFT_BLACK);
  // ++ : two right-pointing triangles (centre x = 200)
  tft.fillTriangle(200 - 1,  cy, 200 - 10, cy - 8, 200 - 10, cy + 8, TFT_BLACK);
  tft.fillTriangle(200 + 10, cy, 200 + 1,  cy - 8, 200 + 1,  cy + 8, TFT_BLACK);
}

void drawSuccessScreen() {
  currentMode = CAL_SUCCESS; tft.fillScreen(THEME_BG);
  tft.setTextColor(themedText(COLOR_GREENYELLOW)); centerStaticText("CALIB SUCCESS!", 40, 2);
  setSmoothFont(1);
  // V12.3: show the loaded profile name for a named import (blank = device cal).
  if (lastCalibName[0]) {
    char nameBuf[40];
    snprintf(nameBuf, sizeof(nameBuf), "Loaded: %s", lastCalibName);
    tft.setTextColor(themedText(COLOR_TEAL)); centerStaticText(nameBuf, 66, 1);
  }
  tft.setTextColor(themedText(TFT_WHITE));

  char floatBuf[32];
  
  snprintf(floatBuf, sizeof(floatBuf), "Slope: %.5f", CTRLX);
  tft.setCursor(30, 95); tft.print(floatBuf);
  
  snprintf(floatBuf, sizeof(floatBuf), "Intercept: %.3f", CTRLY);
  tft.setCursor(30, 125); tft.print(floatBuf);
  
  snprintf(floatBuf, sizeof(floatBuf), "Base Err: %.4f", CALIB_ERROR);
  tft.setCursor(30, 155); tft.print(floatBuf);
  
  snprintf(floatBuf, sizeof(floatBuf), "R-Sq: %.4f", CALIB_R2);
  tft.setCursor(30, 185); tft.print(floatBuf);
  
  btnFinishCal.draw(tft);
}
