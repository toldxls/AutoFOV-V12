# AutoFOV backlog

Open items that have survived at least one review. Update this file in the same
commit that fixes (or deliberately closes) an item. Not a firmware path — edits
here never bump the version.

## Investigating

- **Periodic 6–8 s telemetry blackouts, every ~10–16 min** (first logged
  9/19/26, v12.7.20). Device stays associated (`wifi drops 0`), reconnect
  succeeds at once, the device's old socket then stall-closes because lwIP's
  retransmit timer has backed off. Which end goes deaf is unknown. **Next step:**
  run the dashboard on a phone alongside the laptop for an hour and compare the
  two SOCKET logs — shared timestamps implicate the device/router, laptop-only
  implicates the Mac (WiFi scans, AirDrop/AWDL). Also note DIAGNOSTICS →
  max sensor stall at the same moments.
- **sensorTask heartbeat stalls > 6 s** (I²C stall that self-recovers). The
  self-heal reboot is disabled for this reason (`loop()` F3 block); only the
  worst stall is recorded in `/diag`. Root cause unknown.

## Needs a bench session

- TOF data-ready is polled at 100 Hz for a 5 Hz sensor.
- IMU FIFO is read one 7-byte word per I²C transaction; a burst read needs a
  check of the FIFO address-wrap behaviour first.
- TFT layout eyes-on check from the 8/22/26 device-side batch (MEM_INFO rows,
  Z/T tags on MAIN, STACK DONE banner, REBOOT SURE?).
- Never reviewed: `sensorTask` / `vibTask` internals, the TFT screen triads,
  the camera-trigger watcher, JSON builders vs the dashboard parser.

## Open, low priority

- Captive-portal `/save` has no CSRF defence (a hostile tab in a browser that
  is on the setup AP can submit it). An `Origin` check is NOT safe as-is: `/`
  serves the form under whatever Host the OS probe used, so a legitimate
  Origin can be anything. Needs a per-boot form token, or `/` redirecting to
  `192.168.4.1` first — both want testing on iOS/Android/Windows portals.
- Published `firmware.bin` contains three `/Users/…/Arduino15/…` path strings
  (`-fmacro-prefix-map` via a build property; forces a full core rebuild).
- `web/recovery.html` loads esp-web-tools 10.4.0 from unpkg — pinned, not
  vendored. Vendoring `dist/web` into gh-pages needs a real USB flash to verify.
- Six `calibrations/tofdbg-2026-08-1[23]-*.json` dumps are tracked, so they
  publish to gh-pages. `git rm --cached` if unintended.
- `StaticJsonDocument<N>` / `DynamicJsonDocument(N)` sizes and the comments
  budgeting them are obsolete: the installed ArduinoJson is 7.x, which ignores N
  (elastic heap pool). Migrate to `JsonDocument` and drop the comments.
- Web `calPoints` / `calWidth` / `demarcDist` are accepted while a calibration
  is running on the TFT (only `calStart`/`calCapture`/imports are fenced).

## Accepted risks (deliberate — do not re-open without new information)

- No TLS, no flash encryption / secure boot (see the security notes in
  `AutoFOV_V12_wifi.ino`); power the device from a wall charger.
- A sniffed login exchange allows an offline guess of a SHORT custom password
  (HMAC-SHA256 challenge-response over plaintext HTTP). The 12-character default
  is out of reach; choose a long custom one.
- Login lockout can be dodged by rotating source IPs (6-slot table); the nonce
  ring can be flushed by spamming `/login-challenge`; the setup-AP code is 40
  bits; `otaRollback` needs only the session token; the web-server library
  parses request bodies before our handlers can authenticate them.
- The portal trims the WiFi passphrase: phone keyboards append a trailing space
  far more often than a passphrase legitimately starts or ends with one.
