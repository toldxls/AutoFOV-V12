# AutoFOV V11 (WiFi & BLE Edition)

AutoFOV is a specialized photomicroscopy field-of-view (FOV) calculator and automated focus stacking assistant with wireless camera trigger. Powered by an ESP32-S3, it uses a Time-of-Flight (ToF) sensor to measure bellows separation distance and calculates a real-time FOV based on a calibrated linear regression model for any bellows/magnification. 

This version (V11) introduces concurrent Bluetooth Low Energy (BLE) HID connectivity and notifications, and a high-speed WiFi WebSocket server for live remote telemetry and control.

## Features

* **Real-Time FOV Calculation:** Uses a VL53L4CX ToF sensor to measure distance and computes FOV for 5x, 10x, and 20x objectives.
* **Custom Calibration:** Calibrate the device to your specific optical setup using a scaled micrometer. Calculates standard error (RMSE) and R² of the calibration fit.
* **Stack Calculator:** Automatically calculates image overlap % and total depth from step size, and required image # based on objective numerical aperture (NA) and depth of field (DOF).
* **BLE Camera Triggering:** Acts as a BLE HID Keyboard. Sends an F12 keystroke to your connected device after the stack finishes.
* **Web Dashboard & Telemetry:** Connects to WiFi to provide a high-speed (~30Hz) data stream (distance, AVG FOV, TOF signal rate) and remote control of buttons and screen settings.
* **Captive Portal Setup:** First-time WiFi setup is handled via a built-in access point (AutoFOV-Setup).
* **PSRAM-Backed UI:** Fluid, flicker-free UI utilizing off-screen 16-bit sprites buffered in PSRAM.
* **Customization:** Adjustable screen brightness, sleep timeouts, and multiple color themes (Classic, Midnight, Forest, Daylight) with adjustable tint.

## Hardware Requirements

* **Microcontroller:** ESP32-S3 (Requires 2MB+ PSRAM and 4MB+ Flash)
* **Display:** 2.8" ILI9341 TFT SPI Display
* **Touch Controller:** FT6206 Capacitive Touch (I2C)
* **Sensor:** VL53L4CX Time-of-Flight Sensor (I2C)
* **IMU:** LSM6DSOX 6-axis accelerometer/gyro (I2C) — bench-vibration monitor
* **Trigger Interface:** Optocoupler to sense your stepper controller's shutter release

### Pin Configuration

| Component | Pin (ESP32-S3) | Notes |
| :--- | :--- | :--- |
| **TFT CS** | 9 | SPI Chip Select |
| **TFT DC** | 10 | SPI Data/Command |
| **TFT RST** | 6 | SPI Reset |
| **TFT Backlight** | A2 (LITE_PIN) | PWM Brightness Control |
| **Touch Int** | 11 (TOUCH_INT_PIN) | Active Low Interrupt |
| **Camera Trigger** | A4 | Input Pullup |
| **Trigger LED** | A3 | Active-Low PWM output (3.3V → LED → A3) |
| **IR LED** | 12 | NEC IR transmit to the MJKZZ controller (GPIO12 → 100 Ω → IR LED → GND) |
| **I2C SDA / SCL** | Default | Shared by FT6206, VL53L4CX and LSM6DSOX (0x6A/0x6B) |

## Software Dependencies

Install the following libraries via the Arduino Library Manager or manually:
* `Adafruit_GFX`
* `Adafruit_ILI9341`
* `Adafruit_FT6206`
* `vl53l4cx_class` (STM32duino)
* `NimBLE-Arduino`
* `ESPAsyncWebServer` (by me-no-dev)
* `AsyncTCP` (by me-no-dev)
* `ArduinoJson` (v6.x)
* `IRremoteESP8266` (IR remote transmit)
* `Adafruit LSM6DS` (+ `Adafruit Unified Sensor`) — LSM6DSOX IMU driver
* `arduinoFFT` — spectrum analysis for the vibration monitor

## Installation & Flashing

### 1. Arduino IDE Setup
Because this firmware relies heavily on PSRAM for UI buffering and features concurrent BLE/wifi radio usage, strict settings are required. Configure your board settings as follows:
* **Board:** ESP32S3 Dev Module
* **PSRAM:** QSPI PSRAM 
* **Partition Scheme:** Huge APP 3MB no OTA
* **Events Run On:** Core 1
* **Arduino Runs On:** Core 1

### 2. Upload the Web Interface (LittleFS)
The web interface HTML is required for the WiFi dashboard to load.
1. Requires a `data` folder in the same directory as the `.ino` files.
2. Place the `index.html` file inside the `data` folder.
3. Use the Arduino ESP32 LittleFS Data Upload Tool to flash the contents of the `data` folder to the ESP32.

### 3. Flash the Firmware
Compile and upload the combined sketch (`AutoFOV_V11_patched3.ino` and `AutoFOV_V11_patched3_wifi.ino`).

## Usage Instructions

### WiFi & Web Interface
1. **Initial Boot:** If no WiFi credentials are saved, the device will boot into Captive Portal mode.
2. **Connect to AP:** Connect your phone or PC to the WiFi network `AutoFOV-Setup`.
3. **Configure:** A setup page will pop up automatically. Enter your credentials. The device will save these, restart, and connect to your local network (use a very high IP like 192.168.1.250).
4. **Access Dashboard:** The device's assigned IP address will appear on the TFT screen (under the WIFI INFO menu, accessed by tapping the WiFi icon). Enter this IP into any browser to access the remote controller. 

### BLE Trigger Setup
1. Turn on the device and wait for WiFi to connect. BLE initialization is deferred until WiFi stabilizes to preserve heap memory.
2. Go to the Bluetooth settings on your PC or camera control device.
3. Look for **ESP_Cam** and pair with it.
4. When the hardware trigger (Pin A4) stops being pulled LOW, the ESP32 will send an F12 keystroke over Bluetooth (see thresholds below).

### Stack-Complete Thresholds

Three constants govern the whole trigger sequence (`AutoFOV_V11_patched3.ino:598-599` and the trigger watcher):

| Limit | Value | Role |
| :--- | :--- | :--- |
| Pulse debounce | 15 ms | A4 must read LOW continuously for >15 ms to count as one real trigger pulse (rejects noise/glitches). |
| `MIN_ACTIVE_DURATION` | 20 s (20000) | The stack must have been active at least this long — measured first pulse → last pulse — or nothing fires. |
| `SILENCE_DURATION` | 5 s (5000) | After the last detected A4 pulse, this much silence must pass before the stack is declared "complete." |

When the stack is declared complete, the device sends the BLE F12 keystroke and a browser notification (3-beep audible cue) to any connected web dashboard. A run shorter than `MIN_ACTIVE_DURATION` is treated as a false start and is silently ignored. If your camera's interval between shots exceeds `SILENCE_DURATION`, raise that constant or the stack will falsely complete mid-run.

### Calibration
A set of "factory" calibration values is stored in memory but if you change optics or sensors, run the built-in calibration:
1. Tap **CALIBRATE** on the main screen.
2. Set your photo width (in pixels) and demarcation distance used to measure # of pixels between marks (0.4 mm default).
3. Follow the on-screen prompts to capture multiple points (3 to 20) at varying distances.
4. The device will calculate a linear regression, saving the slope, intercept, R², and other metrics to non-volatile memory.

