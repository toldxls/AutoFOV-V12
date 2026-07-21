# Changelog

High-level history of AutoFOV V12, grouped by minor-version milestone. Patch
numbers increment on every commit (`12.MINOR.<commit-count>`), so individual
patches aren't listed — each section below folds a milestone's worth of them
into the changes that matter. Newest first.

## 12.5 — Vibration cull report & camera tools (Jul 19–21, 2026)

- **Per-stack vibration "cull report"** (`/vibhist`): per-frame blur/time table,
  clickable timeline marker with hover cursor, SNTP-stamped, 8-deep saved-report
  archive, SAVE PNG card, and full-CSV COPY TABLE export.
- **µm/mg timeline toggle** (µm default, converted at the stack's stamped
  resonance f0) over a true per-hop broadband displacement envelope.
- **VALIDATE** — score real photos' sharpness (directional Sobel, native-res
  center crop) and rank-correlate it against the predicted blur.
- **Optical-spot threshold** (0.61λ/NA) as the blur-flag anchor and auto-default,
  with a persistent user override and a stale-Photo-Width mismatch warning.
- **Low-frequency motion watchdog** — tracks sub-3 Hz displacement per stack and
  banners when the unmeasurable band is hot.
- **Camera / Photo-Width picker** — maker → sensor-format → model drill-down plus
  an MP calculator; the calibrator captions the picked model and its fields now
  read as tap-to-edit.
- Unified help overlays (close on any click); analyzer axes-geometry tooltip.

## 12.4 — MTF tool, OTA & field resilience (Jul 1–19, 2026)

- **MTF / RESOLUTION tool** — slanted-edge MTF from a USAF-target shot, with an
  objective-ideal curve, µm axis, equivalent-NA fit, TIFF/RAW support, and
  multi-shot focus ranking.
- **Web calibrator FOCUS/TILT analysis** + per-photo span picker that shows the
  scaling arithmetic behind each result.
- **OTA overhaul** — buffer the whole image in PSRAM and flash off the receive
  task (cures the ~30 s upload stall), with a stale-disconnect guard and
  tolerant status polling.
- **Field resilience** for unattended devices — boot-loop rollback, reset-reason
  diagnostics ring, task watchdog + I²C self-heal, calibration backup, and a
  wrong-password → captive-portal fallback.
- **Diagnostics screen** — task stack watermarks, heap fragmentation,
  alloc/I²C/WiFi fault counters, and last-panic backtrace capture.
- **TOF cold-start re-lock** via a config-cycle, with a COLD/HOT tag and RE-LOCK
  button.
- One green "?" help badge in every menu.

## 12.3 — Photo-assisted calibration (Jun 29 – Jul 1, 2026)

- **Photo-assisted calibration** — auto pixel-count from a micrometer JPEG, and a
  new 14-point factory calibration derived from it.
- **LENS CENTERING** tool — left/right decentering check from a calibration set.
- "How to calibrate" guidance and a native-resolution measure preview.

## 12.2 — Calibration model & stack timing (Jun 12–29, 2026)

- **Pixel-space calibration model** (FOV = k/pixels) with prediction-interval FOV
  error, R² to four decimals, and an AVG-FOV error read as 2σ over the last 5 s.
- **Calibration point list + review**, draggable graph legend, .txt import/export,
  and an owner-curated gh-pages calibration library wired to on-device CAL I/O.
- Measured stack time auto-tunes Sec/Step.
- Vib home-spectrum graph — x-axis ticks, dominant-frequency marker, touch-drag
  cursor.
- Notify polish, recovery help overlays, live GitHub-update glow, 3D header
  icons, and HTML/JS/CSS minification in the build.

## 12.1 — Notifications, auth & signatures (May 24 – Jun 12, 2026)

- **ntfy.sh push notifications** on stack-complete (replacing AutoRemote), plus a
  stack-done toast with a Save-PNG summary card.
- **Login + challenge-response auth** — the password never crosses the wire;
  IP-bound tokens, per-IP lockout, and a nonce ring back it.
- **Vibration signatures** — 10 s peak-hold capture with ambient-floor
  subtraction.
- Full-viewport vibration dashboard with wait-analysis overlays; per-stack V/H
  pixel-blur.
- Sensors auto power-off after 30 min idle; any dashboard tap wakes them.

## 12.0 — Initial V12 release (May 20–24, 2026)

- ESP32-S3 photomicroscopy FOV calculator and focus-stacking assistant:
  VL53L4CX time-of-flight ranging, LSM6DSOX vibration monitor, TFT UI, and a
  WiFi web dashboard.
- **Web vibration analyzer** — live waterfall, filmstrip, RMS trend, and a blur
  estimate.
- **OTA firmware update** — `/ota` endpoint over a dual-partition table with
  SHA-256 verify and rollback, GitHub auto-update, and a browser-based USB
  recovery flasher.
- HTML embedded in the firmware as gzip PROGMEM; version auto-derived from the
  git commit count.
- Security hardening throughout — SSRF/CSRF/XSS fixes, path-traversal guards, and
  an auto-generated OTA password.
