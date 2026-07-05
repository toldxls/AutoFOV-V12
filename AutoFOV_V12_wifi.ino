// =============================================================================
// AutoFOV — photomicroscopy FOV calculator & focus-stacking assistant
// Copyright © 2026 Travis Olds
//
// Licensed under the PolyForm Noncommercial License 1.0.0 — noncommercial use
// only; commercial use requires a separate license from the author.
// Full terms: see LICENSE.md or https://polyformproject.org/licenses/noncommercial/1.0.0
// =============================================================================
// AutoFOV_V12_wifi.ino  —  WiFi / WebSocket server tab
//
// Arduino multi-file tab: compiled together with AutoFOV_V12.ino.
// All variables from the main file are accessible directly (same compilation
// unit — Arduino concatenates all .ino files before compiling).
//
// ── INTEGRATION ───────────────────────────────────────────────────────────────
//   Already wired up in patched3.ino:
//     setup() end   → wifiSetup();
//     loop() top    → wifiLoop();
//
// ── REQUIRED LIBRARIES (install via Arduino Library Manager) ─────────────────
//   • ESPAsyncWebServer  (by me-no-dev)
//   • AsyncTCP           (by me-no-dev)
//   • ArduinoJson        (by Benoit Blanchon, v6.x)
//   DNSServer, LittleFS, and WiFi are part of the ESP32 Arduino core.
//
// ── HTML FILE ────────────────────────────────────────────────────────────────
//   Create a  data/  folder alongside this .ino and place index.html in it.
//   Upload via the Arduino LittleFS filesystem uploader:
//   https://github.com/lorol/arduino-esp32fs-plugin
//
// ── CAPTIVE PORTAL ───────────────────────────────────────────────────────────
//   First boot (no saved credentials) → AP mode "AutoFOV-Setup".
//   Connect any device to that SSID, browser opens setup form automatically.
//   Enter SSID + password; optionally set a static IP (leave blank for DHCP).
//   After Save the ESP32 restarts, connects to your network, and shows its IP
//   on the Serial console + TFT WiFi indicator.
//   Hold BOOT (GPIO0) during power-on to force the portal at any time.
//   Tap the WiFi zone in the TFT header (x=93..151, y=0..42) to open the
//   WIFI_INFO screen, which includes a FORGET WiFi button for manual reset.
// =============================================================================

// patched3: these headers are also included in patched3.ino — kept here for
// readability and standalone-tab usability.  Header guards make the duplicate
// #include harmless.  Do NOT move the *primary* copy back to this tab: the
// Arduino preprocessor injects forward declarations for all .ino functions
// before the wifi tab's #includes are processed, which would break compilation
// of the AsyncWebSocket* / AsyncWebSocketClient* signatures below.
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <HTTPClient.h>
#include <esp_ota_ops.h>      // esp_ota_get_running_partition() — running OTA slot
#include <mbedtls/sha256.h>   // end-to-end OTA integrity check
#include <mbedtls/md.h>       // HMAC-SHA256 for challenge-response login / OTA auth
#include "data/web_ui.h"

// ─────────────────────────────────────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────────────────────────────────────
static const char*     AP_SSID             = "AutoFOV-Setup";
static const IPAddress AP_IP               (192, 168, 4, 1);
static const IPAddress AP_GW               (192, 168, 4, 1);
static const IPAddress AP_MASK             (255, 255, 255, 0);
static const uint8_t   DNS_PORT            = 53;
static const uint32_t  CONNECT_TIMEOUT_MS  = 15000UL;    // 15 s to acquire IP
static const uint32_t  RECONNECT_INTERVAL_MS = 30000UL;  // retry every 30 s
static const uint32_t  FAST_TELEM_MS       = 33UL;       // ~30 Hz live sensor push — matches VL53L4CX timing budget 1:1
static const uint32_t  SLOW_TELEM_MS       = 5000UL;     // 5 s memory + BT push
static const int       CMD_QUEUE_DEPTH     = 16;
// SoC die temperature ×10, cached. temperatureRead() lazily installs the
// temp-sensor peripheral with a non-thread-safe init and a shared handle —
// calling it from both the AsyncTCP task (WS-connect / GET /state on Core 0)
// and the slow telemetry (Core 1) races. Only wifiLoop() (Core 1) reads the
// hardware; every JSON builder reads this cache.
static std::atomic<uint32_t> gDieTempC10{0};
static char            otaPassword[13]     = {0};            // auto-generated on first boot, NVS "wifi"/"otapw" — the DEFAULT login password shown on the TFT
// Optional user-set device password (NVS "wifi"/"devpw"). When non-empty it
// overrides otaPassword as the login/OTA credential; empty means "use the
// default". The effective password is never displayed or sent over the wire.
static char            devPassword[64]     = {0};
// Per-boot session token. Issued only by POST /login after the device password
// is verified; required as X-AutoFOV-Token on /cmd, /state, /forget-wifi,
// /vibsig and as ?tok= on the WebSocket upgrade. Because /login gates issuance,
// a client without the password never obtains a token — this is the real auth
// boundary, not just an anti-CSRF measure. See startFullServer().
static char            csrfToken[17]       = {0};
// IPs that have actually completed a password login this boot. The session token
// is honoured ONLY from one of these (see apiAuthed / the WS filter): on the
// plaintext-HTTP LAN a sniffed token replayed from any other IP is rejected, so
// token theft now needs active on-path spoofing rather than a passive capture.
// Up to 4 so a phone and a laptop can both be signed in. Cleared on reboot,
// which also rotates csrfToken — the two share a lifetime.
static const int       AUTHED_IP_MAX       = 4;
static uint32_t        authedIPs[AUTHED_IP_MAX] = {};
static uint8_t         authedIPHead        = 0;
// Brute-force guard for /login, tracked PER CLIENT IP so a hostile LAN host can
// only lock out itself — not the owner. Bounded table; when full, evict the
// entry whose lockout expires soonest (idle/zero-lockout entries go first, so an
// active lockout is never dropped). The old GLOBAL lockout let any unauthenticated
// LAN host wrong-guess the owner into a rolling 4-minute shut-out.
struct LoginFailEntry { uint32_t ip; uint8_t fails; uint32_t lockoutUntilMs; };
static const int       LOGIN_FAIL_MAX      = 6;
static LoginFailEntry  loginFails[LOGIN_FAIL_MAX] = {};
// Challenge-response nonces: a small RING of single-use, 30 s nonces (was one
// global slot). The ring lets concurrent/competing challenges coexist — the
// login overlay, the OTA re-auth, and a JS auto-retry no longer clobber each
// other's nonce — and widens the window an attacker must spam to evict a
// legitimate one. The client returns HMAC-SHA256(password, nonce) so the
// password never crosses the wire.
static const int       NONCE_RING          = 4;
static char            loginNonce[NONCE_RING][33] = {};   // 32 hex chars + NUL each
static uint32_t        loginNonceMs[NONCE_RING]   = {};
static uint8_t         loginNonceHead      = 0;

// ─────────────────────────────────────────────────────────────────────────────
// WIFI STATE
// ─────────────────────────────────────────────────────────────────────────────
enum WifiServerMode { WMODE_NONE, WMODE_PORTAL, WMODE_STA };
static WifiServerMode  wifiServerMode      = WMODE_NONE;
static char            portalCode[9]       = {0};  // random 8-char WPA2 password shown on TFT during setup
static bool            wifiConnected       = false;
static uint32_t        lastReconnectMs     = 0;
// V12.5 (F5): wrong-WiFi-password → captive-portal fallback. The STA disconnect
// reason (wifi_err_reason_t) is captured by an onEvent handler; staConnectTask
// inspects it after a failed connect and, on an AUTH-class failure, asks Core 1
// (via g_portalFallbackReq) to reopen the setup portal so a friend can retype
// the password — instead of the silent 30 s retry loop with no way back.
static volatile uint8_t     g_lastStaDisconnReason = 0;
static std::atomic<bool>    g_portalFallbackReq{false};
// True while staConnectTask (Core 0) owns the WiFi association during its
// connect attempts. wifiLoop's reconnect logic (Core 1) checks it so the two
// cores never command the WiFi driver at once — the 2-attempt connect can run
// up to 30 s, past wifiLoop's 30 s reconnect timer, which would otherwise inject
// a WiFi.reconnect() mid-handshake and scramble the disconnect-reason capture.
static std::atomic<bool>    g_staConnecting{false};
// Set to millis() when F5 opens the FALLBACK portal (0 = normal no-creds boot
// portal). If nobody configures within the timeout, reboot to re-try the saved
// creds — so a transient handshake failure can't permanently strand the device.
static uint32_t             fallbackPortalMs = 0;
static const uint32_t       FALLBACK_PORTAL_TIMEOUT_MS = 180000UL;   // 3 min
static uint32_t        lastFastTelemMs     = 0;
static uint32_t        lastSlowTelemMs     = 0;
static uint32_t        lastVibPushMs       = 0;       // V12: vibration spectrum stream timer
static bool            vibWebPanelOpen     = false;   // V12: spectrum stream gated on this
static volatile bool   otaInProgress       = false;   // suppresses telemetry during OTA flash
// OTA stall watchdog + integrity check.  If the browser tab dies between
// chunks the upload handler never sees `final`, so wifiLoop() trips
// Update.abort() once otaLastChunkMs goes stale.  The SHA-256 context is fed
// each chunk and compared against the client's X-OTA-SHA256 header at final.
static uint32_t                otaLastChunkMs   = 0;
static mbedtls_sha256_context  otaShaCtx;
static bool                    otaShaActive     = false;   // ctx initialised?
static char                    otaExpectedSha[65] = {0};   // hex, 64 chars + NUL
// V12: PSRAM whole-image OTA buffer.  Writing each chunk to flash inline on the
// AsyncTCP receive task stalls RX (flash erase/write pins the task → dropped
// packets → the intermittent TCP-RTO upload stall).  Instead we memcpy the whole
// .bin into PSRAM during receive — pure RAM, ZERO flash — then a Core-1 flush
// task writes it to flash AFTER the socket closes, so flash never competes with
// RX and the stall is structurally impossible.  If the image won't fit in PSRAM
// (or the alloc fails) otaBufMode stays false and the proven inline Update.write
// path runs instead, so OTA never breaks.  Every Update.* and the SHA verify
// happen ONLY in the flush task — single owner, no cross-core Updater race.
static uint8_t*     otaBuf       = nullptr;   // heap_caps_malloc(MALLOC_CAP_SPIRAM)
static size_t       otaBufCap    = 0;         // allocated capacity (bytes)
static size_t       otaBufLen    = 0;         // .bin bytes received so far
static bool         otaBufMode   = false;     // true = buffering this OTA to PSRAM
static TaskHandle_t otaFlushTask = nullptr;   // Core-1 buffer -> flash writer
// Upload generation — bumped by every /ota pre-flight (AsyncTCP task only, so
// no cross-core concern).  The onDisconnect cleanup lambda captures its own
// upload's generation and no-ops on mismatch: an abandoned upload with no FIN
// (half-open TCP — e.g. a laptop that slept mid-upload) can outlive the 30 s
// stall watchdog and disconnect only after a RETRY has taken over the buffer
// globals — without the check it would free the retry's buffer mid-receive.
static uint32_t     otaUploadGen = 0;
// Async flash result for the buffered path.  The buffered /ota returns 200 as
// soon as the body is received (before the flash runs), so the flush task's
// outcome can't ride that response — the dashboard polls GET /ota-status instead
// to learn success (device reboots) vs a specific failure.  0=idle 1=flushing
// 2=committed(rebooting) -1=failed.
static volatile int8_t otaFlashState = 0;
static char            otaFlashMsg[48] = {0};   // human-readable reason on failure
// Leave this much internal-heap-independent PSRAM free after the buffer so the
// live sprites / vib buffers / any transient alloc during OTA don't get squeezed.
static const size_t OTA_PSRAM_MARGIN = 160 * 1024;
static void otaBufFree() {
    if (otaBuf) { heap_caps_free(otaBuf); otaBuf = nullptr; }
    otaBufCap = 0; otaBufLen = 0; otaBufMode = false;
}
// Restart can't happen inline in the OTA completion handler — that runs on the
// AsyncTCP task, and ESP.restart() preempts the response transmit so the
// browser sees a connection drop instead of the 200 OK.  We stash a timestamp
// and let wifiLoop() (Core 1) restart after the response has had time to flush.
static uint32_t                otaRestartPendingMs = 0;
// Same problem outside OTA: /save (captive portal) and /forget-wifi reboot the
// device, and inline ESP.restart() on Core 0 cuts the response short. Stash a
// deadline and let wifiLoop() (Core 1) do the actual restart once the queued
// HTML/JSON has had time to leave the wire.
static uint32_t                restartPendingMs = 0;
// Version string of the firmware in the *inactive* OTA slot — the rollback
// target.  Read once at boot from NVS (see wifiSetup).  esp_app_desc_t can't
// supply this: arduino-esp32 ships a precompiled esp_app_desc, so its
// date/time is frozen at the core's build, identical for every slot.  Each
// firmware instead self-records its BUILD_VERSION against its running slot.
static String                  otaRollbackInfo = "";

// ─────────────────────────────────────────────────────────────────────────────
// SERVER OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
static AsyncWebServer  httpServer(80);
static AsyncWebSocket  wsServer("/ws");
static DNSServer       dnsServer;

// Separate Preferences object for WiFi credentials — namespace "wifi".
// This is distinct from the main sketch's "calib" and "display" namespaces.
static Preferences     wifiPrefs;

// AutoRemote / stack-done HTTP notify URL (empty = disabled).
// ntfy.sh topic for stack-complete push notification (plain HTTP, no TLS).
// Stored in NVS "wifi" namespace, key "ntfy".
static String ntfyTopic = "";

// ─────────────────────────────────────────────────────────────────────────────
// COMMAND QUEUE
// WS onMessage callback fires on the lwIP/async task (NOT Core 1).
// We must not touch I²C, TFT, or FreeRTOS semaphores from there.
// Commands are pushed into this queue and drained in wifiLoop() on Core 1.
// ─────────────────────────────────────────────────────────────────────────────
struct WifiCmd {
    char key[32];
    char val[64];
};
static QueueHandle_t wifiCmdQueue = nullptr;

// V12.3: calibration-import staging. A full calibration (up to 20 point-pairs)
// is far too big for WifiCmd's val[64], so POST /calib parses + range-checks the
// JSON on the AsyncTCP task (Core 0) into this buffer, then enqueues a one-word
// "calApply"; wifiLoop() (Core 1) loads it and runs finalizeCalibration(), the
// only place TFT + NVS writes are safe. pendingCalibReady is the cross-core
// handoff fence — release on the writer, acquire on the reader.
struct PendingCalib {
    float width;
    float demarc;
    int   n;
    float dist[20];
    float fov[20];
    char  name[32];     // V12.3: optional profile name, shown on the CAL_SUCCESS screen
};
static PendingCalib pendingCalib;
static std::atomic<bool> pendingCalibReady{false};

// Payload for ntfyTask. The topic is COPIED here at send time (Core 1) — the
// task used to read the global ntfyTopic String directly, which the ntfyTopic
// command can reassign (freeing the old buffer) while the task is mid-request.
struct NtfyMsg {
    String topic;
    String body;
};

// ─────────────────────────────────────────────────────────────────────────────
// CAPTIVE PORTAL HTML  (served in AP mode, embedded in flash via PROGMEM)
// ─────────────────────────────────────────────────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>AutoFOV WiFi Setup</title>
<style>
  *{box-sizing:border-box}
  body{background:#111;color:#eee;font-family:Arial,sans-serif;
       display:flex;flex-direction:column;align-items:center;padding:30px 20px;margin:0}
  h2{color:#ADFF2F;margin-bottom:6px;font-size:22px}
  p{color:#666;font-size:12px;margin:0 0 24px}
  label{display:block;margin-bottom:4px;font-size:13px;color:#888}
  input{display:block;width:100%;max-width:320px;padding:10px;margin-bottom:16px;
        background:#222;border:1px solid #444;color:#fff;border-radius:4px;font-size:15px}
  input:focus{outline:none;border-color:#008894}
  .sep{width:100%;max-width:320px;border:none;border-top:1px solid #2a2a2a;margin:8px 0 20px}
  .hint{font-size:11px;color:#555;margin:-12px 0 14px}
  .tip{max-width:320px;margin:20px 0 0;font-size:11px;line-height:1.5;
       color:#d9a441;text-align:center}
  button{background:#008894;color:#fff;border:none;padding:13px;font-size:15px;
         font-weight:bold;border-radius:4px;cursor:pointer;width:100%;max-width:320px}
  button:active{filter:brightness(1.3)}
</style></head>
<body>
<h2>AutoFOV WiFi Setup</h2>
<p>Connect to your home network</p>
<form method=POST action=/save>
  <label>Network SSID</label>
  <input type=text name=ssid placeholder="Your WiFi name" required autocomplete=off spellcheck=false>
  <label>Password</label>
  <input type=password name=pass placeholder="WiFi password" autocomplete=off>
  <hr class=sep>
  <label>Static IP <small style=color:#555>(optional — blank = DHCP)</small></label>
  <input type=text name=ip placeholder="192.168.1.200"
         pattern="^$|^(\d{1,3}\.){3}\d{1,3}$">
  <p class=hint>Leave blank to let your router assign an IP automatically.</p>
  <label>Gateway <small style=color:#555>(optional — auto-derived if blank)</small></label>
  <input type=text name=gw placeholder="192.168.1.1"
         pattern="^$|^(\d{1,3}\.){3}\d{1,3}$">
  <hr class=sep>
  <label>Device password <small style=color:#555>(optional)</small></label>
  <input type=password name=devpw placeholder="Set a login password"
         autocomplete=new-password minlength=6 maxlength=63>
  <p class=hint>Required to open the dashboard and flash firmware. Leave blank to
     use the random code shown on the device screen. Minimum 6 characters.</p>
  <button type=submit>SAVE &amp; CONNECT</button>
</form>
<p class=tip>&#9888; For best security, power AutoFOV from a USB wall charger &mdash;
   not a computer. A USB data link to a PC can expose the device console and
   allow reflashing.</p>
</body></html>)html";

static const char PORTAL_SAVED_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>AutoFOV</title>
<style>body{background:#111;color:#eee;font-family:Arial,sans-serif;
            text-align:center;padding:40px 20px;margin:0}
h2{color:#ADFF2F}p{color:#888;font-size:13px;line-height:1.7}</style>
</head><body>
<h2>&#10003; Credentials saved!</h2>
<p>AutoFOV is restarting and connecting to your network.<br>
The assigned IP address will appear on the device screen and Serial console.<br>
<br>
Point your browser to that address to open the control panel.</p>
</body></html>)html";

// ─────────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────

// Idempotent mDNS registration. Safe to call on every transition into
// WL_CONNECTED — the static flag suppresses re-registration once it's up.
// We have to make sure this fires whenever WiFi *first* comes up in a given
// session, even when that's via wifiLoop's reconnect retry after an initial
// staConnectTask attempt failed. ESPmDNS re-announces on STA_GOT_IP events
// internally, but only once the first MDNS.begin() has actually run.
static bool mdnsStarted = false;
static void ensureMdns() {
    if (mdnsStarted) return;
    if (MDNS.begin("autofov")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("[mDNS] autofov.local registered");
        mdnsStarted = true;
    } else {
        Serial.println("[mDNS] registration failed — will retry on next connect");
    }
}

// Uniform random index in [0, n) using rejection sampling — drops esp_random()
// draws that fall into the partial bucket so the result is bias-free for any n.
// (Alphabets of 32 chars have no modulo bias on a uint32_t to begin with; the
// 56-char CSRF alphabet does. One helper keeps every caller uniform regardless
// of how a future alphabet is sized.)
static uint32_t uniformRandom(uint32_t n) {
    if (n == 0) return 0;
    uint32_t limit = (uint32_t)0xFFFFFFFFUL - ((uint32_t)0xFFFFFFFFUL % n);
    for (;;) {
        uint32_t r = esp_random();
        if (r < limit) return r % n;
    }
}

static void generatePortalCode() {
    // 8 chars: WPA2-PSK requires an 8-character minimum — softAP() rejects
    // anything shorter and the AP never starts.
    // Omit visually ambiguous characters (0/O, 1/I/L).
    static const char CHARS[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
    const uint32_t N = sizeof(CHARS) - 1;
    for (int i = 0; i < 8; i++)
        portalCode[i] = CHARS[uniformRandom(N)];
    portalCode[8] = '\0';
}

static void generateOtaPassword() {
    static const char CHARS[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
    const uint32_t N = sizeof(CHARS) - 1;
    for (int i = 0; i < 12; i++)
        otaPassword[i] = CHARS[uniformRandom(N)];
    otaPassword[12] = '\0';
}

// First-boot generation of the default device password is DEFERRED until after
// WiFi.mode() has enabled the RF subsystem, so esp_random() is the true hardware
// RNG — exactly the property generatePortalCode()/generateCsrfToken() rely on.
// Generating it inside wifiSetup() (before any WiFi.mode() call) would seed the
// single most important default credential — the login/OTA password most users
// keep — from the weak pre-RF PRNG. wifiSetup() only raises this flag; both
// start paths call ensureOtaPassword() once RF is up.
static bool otaPwNeedsGen = false;
static void ensureOtaPassword() {
    if (!otaPwNeedsGen) return;
    generateOtaPassword();                       // esp_random() now HW-backed
    wifiPrefs.begin("wifi", false);
    wifiPrefs.putString("otapw", String(otaPassword));
    wifiPrefs.end();
    otaPwNeedsGen = false;
    // Do NOT print the password itself — a friend may power the device from a
    // computer, exposing this UART to any program on that host. It's shown on the
    // TFT WiFi Info screen (physical access) instead.
    Serial.println("[OTA] default device password generated (see TFT WiFi Info screen)");
}

static void generateCsrfToken() {
    static const char CHARS[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789abcdefghijkmnpqrstuvwxyz";
    const uint32_t N = sizeof(CHARS) - 1;
    for (int i = 0; i < 16; i++)
        csrfToken[i] = CHARS[uniformRandom(N)];
    csrfToken[16] = '\0';
}

// The login/OTA credential: the user's custom password when set, else the
// auto-generated default shown on the TFT.
static const char* effectivePassword() {
    return devPassword[0] ? devPassword : otaPassword;
}

// Issue a fresh single-use challenge nonce (32 hex chars) into the next ring
// slot. Returns a pointer to THAT slot so the caller echoes the exact nonce it
// minted (not whatever lands at head afterwards). Stamped for the 30 s validity
// window checked in checkChallenge().
static const char* generateLoginNonce() {
    static const char kHex[] = "0123456789abcdef";   // (not HEX — that's an Arduino macro)
    char* slot = loginNonce[loginNonceHead];
    for (int i = 0; i < 32; i++) slot[i] = kHex[uniformRandom(16)];
    slot[32] = '\0';
    loginNonceMs[loginNonceHead] = millis();
    loginNonceHead = (loginNonceHead + 1) % NONCE_RING;
    return slot;
}

// HMAC-SHA256(effective device password, msg) → lowercase hex (64 chars + NUL).
// The browser recomputes the same value, so the password itself never crosses
// the wire — only this one-way digest of (password, nonce) does.
static void computeAuthHmacHex(const String& msg, char out[65]) {
    uint8_t mac[32];
    const char* key = effectivePassword();
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    (const unsigned char*)key, strlen(key),
                    (const unsigned char*)msg.c_str(), msg.length(), mac);
    static const char kHex[] = "0123456789abcdef";   // (not HEX — Arduino macro)
    for (int i = 0; i < 32; i++) { out[i*2] = kHex[mac[i] >> 4]; out[i*2+1] = kHex[mac[i] & 0xF]; }
    out[64] = '\0';
}

// Constant-time-ish compare. `b` is the secret, `a` the attacker-supplied
// value (both call sites pass them in that order). The loop count tracks only
// the secret's length — which is fixed per device — never the caller's input,
// so response timing can't be used to probe the attempt length, and there's no
// per-character early-out. The la^lb term fails any length mismatch outright.
static bool secureEquals(const char* a, const char* b) {
    size_t la = strlen(a), lb = strlen(b);
    uint8_t diff = (uint8_t)(la ^ lb);
    size_t n = lb ? lb : 1;
    for (size_t i = 0; i < n; i++)
        diff |= (uint8_t)(a[la ? (i % la) : 0] ^ b[i % (lb ? lb : 1)]);
    return diff == 0;
}

// Validate a challenge response (nonce + client HMAC). A valid, current nonce is
// consumed (single-use) whether or not the HMAC matched. Returns:
//   CR_NO_CHALLENGE — missing / expired / mismatched nonce. NOT a password guess,
//                     so callers must NOT count it toward the brute-force lockout
//                     (also closes drive-by lockout: a cross-origin page can't
//                     read a nonce, so it can never produce a counted attempt).
//   CR_BAD          — valid nonce, wrong HMAC: a genuine wrong-password guess.
//   CR_OK           — valid nonce, correct HMAC.
// (Plain ints + #define rather than an enum: a function returning a sketch-local
// enum trips arduino-cli's auto-prototype generation, which lands the prototype
// above the type definition.)
#define CR_OK            0
#define CR_NO_CHALLENGE  1
#define CR_BAD           2
static int checkChallenge(const String& nonce, const String& respHex) {
    uint32_t now = millis();
    int found = -1;
    for (int i = 0; i < NONCE_RING; i++) {
        if (!loginNonce[i][0]) continue;
        if ((uint32_t)(now - loginNonceMs[i]) > 30000UL) { loginNonce[i][0] = '\0'; continue; }
        if (nonce == loginNonce[i]) { found = i; break; }
    }
    if (found < 0) return CR_NO_CHALLENGE;
    loginNonce[found][0] = '\0';                // consume — single use
    char expected[65];
    computeAuthHmacHex(nonce, expected);
    return secureEquals(respHex.c_str(), expected) ? CR_OK : CR_BAD;
}

// Signature names from the web must match /vibsig/*.bin's filename whitelist so
// nothing from the network can write outside /vibsig/. `allowDotBin` is true
// when the caller passes a full "name.bin" (vibDelSig); false for the bare
// basename (vibCapture, the rename target). The basename length cap is
// VIB_SIG_NAME_MAX — see that constant for why the limit is what it is.
static bool vibNameSafe(const char* s, bool allowDotBin) {
    if (!s || !*s) return false;
    size_t len = strlen(s);
    size_t scanLen = len;
    if (allowDotBin) {
        if (len <= 4 || strcmp(s + len - 4, ".bin") != 0) return false;
        scanLen = len - 4;
    }
    if (scanLen == 0 || scanLen > VIB_SIG_NAME_MAX) return false;
    for (size_t i = 0; i < scanLen; i++) {
        char c = s[i];
        bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) return false;
    }
    return true;
}

// One-shot: at boot, drop any /vibsig/ entries that fail the current name
// whitelist (vibNameSafe with .bin). Prior firmware accepted longer or oddly-
// charactered names from the network; this pass purges those so the dashboard
// never displays a name the device can't round-trip through its buffers.
static void vibSigCleanupLegacyNames() {
    File dir = LittleFS.open("/vibsig");
    if (!dir || !dir.isDirectory()) { if (dir) dir.close(); return; }
    String bad[8]; int badN = 0;
    for (;;) {
        File e = dir.openNextFile();
        if (!e) break;
        if (!e.isDirectory()) {
            const char* fn = e.name();
            const char* slash = strrchr(fn, '/');
            const char* base = slash ? slash + 1 : fn;
            if (!vibNameSafe(base, true) && badN < (int)(sizeof(bad)/sizeof(bad[0]))) {
                bad[badN++] = String("/vibsig/") + base;
            }
        }
        e.close();
    }
    dir.close();
    for (int i = 0; i < badN; i++) {
        Serial.printf("[vib] cleanup: removing %s\n", bad[i].c_str());
        LittleFS.remove(bad[i]);
    }
}

static void startPortalMode();
static void startStaMode(const String& ssid, const String& pass,
                          const String& staticIP, const String& gateway);
static void startFullServer();
static void handleWifiCommand(const char* key, const char* val);
static void buildFullStateJson(String& out, bool includeCalGraph = true);
static void buildFastTelemJson(String& out);
static void pushVibSpectrumBinary();             // V12 — binary WS frame
static void buildSlowTelemJson(String& out);
static void buildSettingsJson(String& out);

// ─────────────────────────────────────────────────────────────────────────────
// SCREEN-NAVIGATION SYNC  —  mirrors the device menu ↔ the web dashboard menu.
//   The web sends {"nav":"screen-xxx"} when the user navigates; the device
//   broadcasts {"screen":"screen-xxx"} from wifiLoop() when currentMode changes.
//   The "simple" menus and the read-only FOV-info / cal-graph screens sync;
//   the interactive calibration capture flow (CAL_SETTINGS/RUN/SAMPLING/REVIEW/
//   SUCCESS/CONFIRM) is deliberately excluded, and a remote nav is ignored
//   while the device is in that flow so the web can't interrupt a calibration.
//   Defined here, ahead of wifiLoop(), which both reads and writes it.
// ─────────────────────────────────────────────────────────────────────────────
struct ScreenMapEntry { DisplayMode mode; const char* id; };
static const ScreenMapEntry kScreenMap[] = {
    { MAIN,                "screen-main"       },
    { APP_SETTINGS,        "screen-settings"   },
    { STACK_CALC,          "screen-stackcalc"  },
    { STACK_TIME,          "screen-stacktime"  },
    { BRIGHTNESS_SETTINGS, "screen-brightness" },
    { SCREEN_TIMEOUT,      "screen-timeout"    },
    { SENSOR_INFO,         "screen-sensorinfo" },
    { MEM_INFO,            "screen-memory"     },
    { FOV_INFO,            "screen-fovinfo"    },
    { CAL_GRAPH,           "screen-calgraph"   },
    { ABOUT,               "screen-about"      },
    { WIFI_INFO,           "screen-wifiinfo"   },
    { IR_CONTROL,          "screen-ir"         },
    // V12.3: info overlays. ids match the web overlay element IDs (not .screen
    // elements) — the web opens/closes them as overlays; the nav handler below
    // routes these through openInfoScreen() since they need the text canvas.
    { WIFI_TIPS,           "wifi-tips"         },
    { RECOVERY_HELP,       "recovery-help"     },
};
static const size_t kScreenMapLen = sizeof(kScreenMap) / sizeof(kScreenMap[0]);

// currentMode value last reflected to / received from the web — the echo guard.
// Written by both the wifiLoop() broadcast poll and the "nav" command handler.
static DisplayMode lastSyncedMode = MAIN;

// Web screen-id for a syncable mode, or nullptr when the mode is not synced.
// Takes int (not DisplayMode) so an auto-generated prototype can't reference
// the DisplayMode enum ahead of its definition in the main tab.
static const char* screenIdForMode(int m) {
    for (size_t i = 0; i < kScreenMapLen; i++)
        if ((int)kScreenMap[i].mode == m) return kScreenMap[i].id;
    return nullptr;
}

// True while the device is in the interactive calibration flow — remote nav is
// ignored in these modes so the web can't yank the user out of a calibration.
static bool inCalibFlow() {
    return currentMode == CAL_SETTINGS || currentMode == CAL_RUN     ||
           currentMode == CAL_SAMPLING || currentMode == CAL_REVIEW  ||
           currentMode == CAL_SUCCESS  || currentMode == CAL_CONFIRM;
}

// Narrower predicate for remote-write contention: the TFT is actively in the
// capture half of the cal flow, so a remote calStart/calCapture would race on
// pointsCaptured/distPoints[]/fovPoints[]. CAL_SETTINGS/CAL_REVIEW/CAL_SUCCESS
// don't mutate those arrays, so remote captures are still safe there.
static bool isLocalCalActive() {
    return currentMode == CAL_RUN || currentMode == CAL_SAMPLING;
}

// ─────────────────────────────────────────────────────────────────────────────
// WEBSOCKET EVENT HANDLER  (runs on async task — enqueue only, no I²C/TFT)
// ─────────────────────────────────────────────────────────────────────────────
static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {

    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] client #%u connected from %s — sending full state\n",
                      client->id(), client->remoteIP().toString().c_str());
        Serial.flush();
        // Push the full device state to the newly connected client.
        // NOTE: buildFullStateJson() runs here on the AsyncTCP task (Core 0) and
        // reads mutable calibration globals (distPoints/fovPoints/pointsCaptured/
        // CTRLX...) that Core 1 may update mid-calibration. 32-bit reads are
        // atomic on Xtensa so this won't crash, but a rare connect-time snapshot
        // may be slightly inconsistent. Acceptable for a one-shot push.
        String out;
        buildFullStateJson(out, /*includeCalGraph=*/true);
        client->text(out);
        Serial.printf("[WS] full-state push: %u bytes\n", out.length());
        Serial.flush();

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        // Accept only single-frame, complete text messages
        if (!info->final || info->index != 0 || info->len != len
            || info->opcode != WS_TEXT) {
            Serial.printf("[WS] data dropped (final=%d index=%u len=%u opcode=%d)\n",
                          info->final, (unsigned)info->index, (unsigned)info->len, info->opcode);
            Serial.flush();
            return;
        }

        if (len > 512) {
            Serial.printf("[WS] Payload too large (%u bytes), dropping\n", (unsigned)len);
            Serial.flush();
            return;
        }
        char buf[513];
        memcpy(buf, data, len);
        buf[len] = '\0';
        Serial.printf("[WS] data rx: %s\n", buf);
        Serial.flush();

        StaticJsonDocument<512> doc;   // sized to the 512-byte payload cap above
        if (deserializeJson(doc, buf) != DeserializationError::Ok) return;

        for (JsonPair kv : doc.as<JsonObject>()) {
            WifiCmd cmd;
            strncpy(cmd.key, kv.key().c_str(), sizeof(cmd.key) - 1);
            cmd.key[sizeof(cmd.key) - 1] = '\0';

            // Serialise value to string for uniform dispatch
            String vStr;
            if      (kv.value().is<const char*>()) vStr = kv.value().as<const char*>();
            else if (kv.value().is<double>())       vStr = String(kv.value().as<double>(), 6);
            else if (kv.value().is<long>())         vStr = String(kv.value().as<long>());
            else if (kv.value().is<bool>())         vStr = kv.value().as<bool>() ? "1" : "0";
            else serializeJson(kv.value(), vStr);

            strncpy(cmd.val, vStr.c_str(), sizeof(cmd.val) - 1);
            cmd.val[sizeof(cmd.val) - 1] = '\0';
            xQueueSend(wifiCmdQueue, &cmd, (TickType_t)0);
        }

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] client #%u disconnected\n", client->id());
        Serial.flush();
        wsServer.cleanupClients();
    } else if (type == WS_EVT_ERROR) {
        Serial.printf("[WS] error on client #%u\n", client->id());
        Serial.flush();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiSetup()  —  call at the END of setup() in the main .ino
// ─────────────────────────────────────────────────────────────────────────────
void wifiSetup() {
    wifiCmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(WifiCmd));

    // Mount LittleFS — used for vibration signatures (/vibsig/*.bin)
    if (!LittleFS.begin(true)) {
        Serial.println("[WiFi] LittleFS mount failed — vibration signatures unavailable");
    } else {
        vibSigCleanupLegacyNames();   // drop any pre-sanitisation /vibsig/ files
    }

    // GPIO0 (BOOT button) held LOW at power-on → force captive portal
    pinMode(0, INPUT_PULLUP);
    delay(50);
    bool forcePortal = (digitalRead(0) == LOW);

    // Load saved credentials and notify URL
    wifiPrefs.begin("wifi", true);   // read-only
    String ssid     = wifiPrefs.getString("ssid",  "");
    String pass     = wifiPrefs.getString("pass",  "");
    String staticIP = wifiPrefs.getString("ip",    "");
    String gateway  = wifiPrefs.getString("gw",    "");
    ntfyTopic       = wifiPrefs.getString("ntfy",  "");
    String savedOta = wifiPrefs.getString("otapw", "");
    String savedDev = wifiPrefs.getString("devpw", "");
    wifiPrefs.end();

    // Optional user-set login password (overrides the auto default). Bounded by
    // sizeof(devPassword)-1; left empty when the user never set one.
    savedDev.toCharArray(devPassword, sizeof(devPassword));

    // Per-device OTA password: load the saved one, or defer first-boot
    // generation until RF is up (see ensureOtaPassword()). Generating here —
    // ahead of every WiFi.mode() call — would draw the default credential from
    // the pre-RF PRNG instead of the hardware RNG.
    if (savedOta.length() == 12) {
        savedOta.toCharArray(otaPassword, sizeof(otaPassword));
    } else {
        otaPwNeedsGen = true;
    }

    // Stamp this firmware's version against the OTA slot it booted from, and
    // cache the *other* slot's stamp as the rollback target.  Refreshed every
    // boot so the running slot's record is always accurate; the inactive
    // slot's record only changes when an OTA flashes it (which reboots us),
    // so caching it here is safe for the whole session.  Keys live in the
    // existing "wifi" NVS namespace.
    {
        const esp_partition_t* run = esp_ota_get_running_partition();
        bool run1 = (run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1);
        const char* myKey    = run1 ? "fw_ota1" : "fw_ota0";
        const char* otherKey = run1 ? "fw_ota0" : "fw_ota1";
        String mine = String("v") + BUILD_VERSION + "  " + __DATE__;
        wifiPrefs.begin("wifi", false);
        if (wifiPrefs.getString(myKey, "") != mine)
            wifiPrefs.putString(myKey, mine);
        otaRollbackInfo = wifiPrefs.getString(otherKey, "");
        wifiPrefs.end();
        Serial.printf("[OTA] running %s as %s; rollback target = %s\n",
                      run1 ? "ota_1" : "ota_0", mine.c_str(),
                      otaRollbackInfo.length() ? otaRollbackInfo.c_str() : "(unknown)");
    }

    if (forcePortal || ssid.isEmpty()) {
        if (forcePortal) Serial.println("[WiFi] BOOT held — forcing captive portal");
        startPortalMode();
    } else {
        Serial.printf("[WiFi] Connecting to \"%s\"…\n", ssid.c_str());
        startStaMode(ssid, pass, staticIP, gateway);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiLoop()  —  call in loop() after the atomic-read block in the main .ino
// ─────────────────────────────────────────────────────────────────────────────
void wifiLoop() {

    // ── Generic deferred restart (set by /save, /forget-wifi, etc.) ──────────
    // Checked before the portal early-return so it fires in either mode.
    // 800 ms matches the OTA restart window — enough for AsyncTCP to flush
    // the queued response and FIN to the client at LAN latency.
    if (restartPendingMs && (millis() - restartPendingMs) > 800UL) {
        Serial.println("[WiFi] deferred restart firing");
        Serial.flush();
        ESP.restart();
    }

    // ── V12.5 (F5): wrong-password portal fallback (Core-0 → Core-1 handoff) ──
    // staConnectTask (Core 0) detected an AUTH-class STA failure and set the
    // flag instead of starting the STA server. startPortalMode() writes NVS and
    // registers/starts HTTP routes — Core-1-only work — so we run it here, on
    // Core 1, exactly as boot does, then flip the TFT to the WiFi-Info screen so
    // the friend sees the setup AP + password. Runs once (exchange clears it).
    if (g_portalFallbackReq.exchange(false, std::memory_order_acq_rel)) {
        Serial.println("[WiFi] portal fallback — starting setup AP");
        startPortalMode();
        fallbackPortalMs = millis();   // arm the auto-return-to-STA timer
        currentMode = WIFI_INFO;
        drawWifiInfoUI();
        return;
    }

    // Captive portal: keep the DNS server ticking
    if (wifiServerMode == WMODE_PORTAL) {
        dnsServer.processNextRequest();

        // A FALLBACK portal (auth-class STA failure) is only correct if the
        // password was actually wrong. reason 15/204 can also be a TRANSIENT
        // handshake failure on a weak AP, which would otherwise strand the
        // device here until a manual power-cycle. So: if nobody has connected to
        // the setup AP within the timeout, reboot and re-try the saved creds — a
        // transient failure then reconnects; a genuinely wrong password fails
        // again and returns here. Skipped while a client IS connected (the friend
        // is mid-reconfigure) and for the normal no-creds boot portal
        // (fallbackPortalMs == 0).
        if (fallbackPortalMs &&
            (uint32_t)(millis() - fallbackPortalMs) > FALLBACK_PORTAL_TIMEOUT_MS &&
            WiFi.softAPgetStationNum() == 0) {
            Serial.println("[WiFi] fallback portal idle — rebooting to retry saved WiFi");
            Serial.flush();
            ESP.restart();
        }
        return;   // no telemetry in portal mode
    }

    if (wifiServerMode != WMODE_STA) return;

    // ── Reconnect logic ──────────────────────────────────────────────────────
    // Skip while staConnectTask (Core 0) still owns the association during its
    // initial connect attempts — otherwise both cores drive the WiFi driver.
    if (!wifiConnected && !g_staConnecting.load(std::memory_order_acquire)) {
        if (millis() - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
            lastReconnectMs = millis();
            if (WiFi.status() == WL_CONNECTED) {
                wifiConnected = true;
                Serial.printf("[WiFi] Reconnected: %s\n",
                              WiFi.localIP().toString().c_str());
                if (currentMode == MAIN) drawWifiIndicator();   // patched3: update header
                // Catch the case where the initial staConnectTask attempt
                // failed and this is the first time we've actually had an IP
                // in this session — without it, mDNS would stay unregistered
                // for the rest of the boot.
                ensureMdns();
                String out; buildFullStateJson(out, false);    // reconnect: skip calGraphPoints
                wsServer.textAll(out);
            } else {
                Serial.println("[WiFi] Attempting reconnect…");
                WiFi.reconnect();
            }
        }
        return;
    }

    // Check for drop since last loop
    if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        lastReconnectMs = millis();
        Serial.println("[WiFi] Connection lost — will retry");
        if (currentMode == MAIN) drawWifiIndicator();   // patched3: show disconnected bars
        return;
    }

    // ── Drain command queue ──────────────────────────────────────────────────
    WifiCmd cmd;
    while (xQueueReceive(wifiCmdQueue, &cmd, 0) == pdTRUE) {
        Serial.printf("[CMD] dispatch %s = %s\n", cmd.key, cmd.val);
        Serial.flush();
        handleWifiCommand(cmd.key, cmd.val);
    }

    // ── OTA deferred restart ─────────────────────────────────────────────────
    // The completion handler queued a 200 response and asked us to reboot.
    // 800 ms is enough for AsyncTCP to flush the response + FIN to the client
    // (typical wire time well under 50 ms); pushing it further only delays the
    // UX without buying reliability.
    if (otaRestartPendingMs && (millis() - otaRestartPendingMs) > 800UL) {
        Serial.println("[OTA] response flushed, restarting");
        Serial.flush();
        ESP.restart();
    }

    // ── OTA stall watchdog ───────────────────────────────────────────────────
    // If the browser closes the upload mid-flight, the upload handler never
    // sees `final` and otaInProgress stays true — silently muting telemetry.
    // After 30 s of silence we resume telemetry, and nothing else.
    //
    // CRITICAL: this runs on Core 1 (wifiLoop) while the OTA upload handler
    // runs on Core 0 (AsyncTCP).  Update.h and the mbedtls context are NOT
    // thread-safe — a cross-core Update.abort() here races _writeBuffer()'s
    // use of the shared _buffer and corrupts the Updater state machine,
    // leaving _error stuck at ABORT.  That was the root cause of the
    // persistent "Rejected: Aborted" failures.  So this watchdog touches
    // ONLY the plain bool/uint32 flags; the leftover Update session, SHA
    // context, AND the PSRAM whole-image buffer are torn down safely on Core 0 by
    // the next OTA request's pre-flight cleanup (freeing otaBuf here would race a
    // producer memcpy on the AsyncTCP task, same class of cross-core bug).
    // The !otaFlushTask gate is essential: during a buffered flash otaLastChunkMs
    // is frozen at the last received chunk while the Core-1 flush task writes for
    // seconds — without the gate a slow/large flush would trip this, resume 30 Hz
    // telemetry mid-flash and reintroduce the exact RX/flash contention the
    // buffered path exists to avoid.  A live flush task means "not abandoned".
    if (otaInProgress && !otaFlushTask && otaLastChunkMs &&
        (uint32_t)(millis() - otaLastChunkMs) > 30000UL) {
        Serial.println("[OTA] stall watchdog — upload abandoned, resuming telemetry");
        otaInProgress  = false;
        otaLastChunkMs = 0;
    }

    // ── Heap watermark log ──────────────────────────────────────────────────
    // After observing a ~4 KB lifetime minimum the spectrum-push was the
    // suspect; this prints whenever the lifetime minimum drops by another
    // >=1 KB so a future regression announces itself without a profiler.
    // ESP.getMinFreeHeap() is monotonically non-increasing per boot.
    {
        static uint32_t lastReportedMin = 0xFFFFFFFFUL;
        uint32_t minNow = ESP.getMinFreeHeap();
        if (lastReportedMin == 0xFFFFFFFFUL) {
            lastReportedMin = minNow;
        } else if (lastReportedMin >= minNow + 1024) {
            Serial.printf("[HEAP] new min %u (was %u, free now %u, psram free %u)\n",
                          (unsigned)minNow, (unsigned)lastReportedMin,
                          (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getFreePsram());
            Serial.flush();
            lastReportedMin = minNow;
        }
    }

    // ── Screen-navigation sync (device → web) ────────────────────────────────
    // currentMode changes on every on-device menu touch. When it lands on a
    // syncable mode, broadcast the matching web screen-id so connected
    // dashboards follow. The "nav" command handler also updates lastSyncedMode,
    // so a web-initiated change is absorbed here without echoing back. Runs
    // before the no-clients return below so lastSyncedMode tracks continuously
    // (textAll is a no-op with zero clients).
    if (currentMode != lastSyncedMode) {
        const char* navId = screenIdForMode(currentMode);
        if (navId) {
            String navMsg = "{\"screen\":\"";
            navMsg += navId;
            navMsg += "\"}";
            wsServer.textAll(navMsg);
        }
        lastSyncedMode = currentMode;
    }

    // Refresh the cached die temperature on the slow cadence — BEFORE the
    // early-returns below, so GET /state stays fresh with zero WS clients.
    // This is the only temperatureRead() call site; see gDieTempC10.
    static uint32_t lastDieTempMs = 0;
    if (lastDieTempMs == 0 || (uint32_t)(millis() - lastDieTempMs) >= SLOW_TELEM_MS) {
        lastDieTempMs = millis();
        // int32 bit pattern: a sub-zero die temp (cold-garage power-up) must
        // not wrap to ~4.29e9 in the unsigned atomic
        gDieTempC10.store((uint32_t)(int32_t)lroundf(temperatureRead() * 10.0f), std::memory_order_relaxed);
    }

    // ── Telemetry push ───────────────────────────────────────────────────────
    if (otaInProgress) return;           // OTA flash in progress — don't clog the WS queue
    if (wsServer.count() == 0) return;   // no connected clients — skip serialisation

    // Drop the frame entirely when any client's TX queue is full. Pushing into
    // a full AsyncWebSocket queue drops the message or closes the socket; live
    // telemetry wants the freshest frame, not a backlog. The timestamp is
    // advanced regardless so a slow client just thins the stream.
    uint32_t now = millis();
    if (now - lastFastTelemMs >= FAST_TELEM_MS) {
        lastFastTelemMs = now;
        if (wsServer.availableForWriteAll()) {
            String out; buildFastTelemJson(out);
            wsServer.textAll(out);
        }
    }
    if (now - lastSlowTelemMs >= SLOW_TELEM_MS) {
        lastSlowTelemMs = now;
        if (wsServer.availableForWriteAll()) {
            String out; buildSlowTelemJson(out);
            wsServer.textAll(out);
        }
    }

    // V12: vibration spectrum stream — ~5 Hz, streamed continuously to every
    // connected client (the wsServer.count()==0 early-return above already
    // skips us when nobody's listening). Used to be gated on vibWebPanelOpen
    // but that broke the 90 s RMS trend pane: navigating off the analyzer
    // mid-stack stopped the µm samples, leaving a visible gap when the user
    // came back. Bandwidth is ~5 KB/s/client; backpressure is handled below.
    // Pushed as a binary WebSocket frame (raw little-endian uint16) — the old
    // JSON form cost ~3.4 KB of String heap per push and another ~3.4 KB in
    // the per-client AWS TX queue. Binary is ~1 KB.
    if (now - lastVibPushMs >= 200) {
        lastVibPushMs = now;
        if (wsServer.availableForWriteAll()) {
            pushVibSpectrumBinary();
        }
    }

    // ── Device-side state-change detection ──────────────────────────────────
    // Touch handlers on the device modify settings directly without notifying
    // wifi.  Snapshot the values that can change on-device; when any differs
    // from the last push, send a fresh full-state frame so the web UI tracks.
    static int      lastObj          = -1;
    static int      lastBrightness   = -1;
    static bool     lastLedEnabled   = false;
    static int      lastLedDuty      = -1;
    static bool     lastSensorSleep  = false;
    static bool     lastHighRefl     = false;
    static int      lastStepIndex    = -1;
    static int      lastImgs         = -1;
    static float    lastSecStep      = -1.0f;
    static float    lastMeasStep     = -1.0f;   // V12.3: measured sec/step
    static int      lastStackActive  = -1;      // V12.3: MJKZZ stack running flag
    static int      lastTheme        = -1;
    static int      lastTint         = -1;
    static uint32_t lastDimMs        = 0xFFFFFFFFUL;
    static uint32_t lastSleepMs      = 0xFFFFFFFFUL;
    static bool     stateSnapInit    = false;

    if (currentobj      != lastObj          ||
        currentBrightness != lastBrightness ||
        ledEnabled      != lastLedEnabled   ||
        currentLedDuty  != lastLedDuty      ||
        sensorSleeping  != lastSensorSleep  ||
        highReflMode    != lastHighRefl     ||
        stackStepIndex  != lastStepIndex    ||
        stackTotalImgs  != lastImgs         ||
        fabsf(stackTimePerStep - lastSecStep) > 0.01f ||
        fabsf(measuredPerStep - lastMeasStep) > 0.01f ||
        (mjkzzStackActive ? 1 : 0) != lastStackActive ||
        currentThemeIndex != lastTheme      ||
        themeIntensity  != lastTint         ||
        (uint32_t)dimTimeoutMs   != lastDimMs   ||
        (uint32_t)sleepTimeoutMs != lastSleepMs) {

        lastObj         = currentobj;
        lastBrightness  = currentBrightness;
        lastLedEnabled  = ledEnabled;
        lastLedDuty     = currentLedDuty;
        lastSensorSleep = sensorSleeping;
        lastHighRefl    = highReflMode;
        lastStepIndex   = stackStepIndex;
        lastImgs        = stackTotalImgs;
        lastSecStep     = stackTimePerStep;
        lastMeasStep    = measuredPerStep;
        lastStackActive = mjkzzStackActive ? 1 : 0;
        lastTheme       = currentThemeIndex;
        lastTint        = themeIntensity;
        lastDimMs       = (uint32_t)dimTimeoutMs;
        lastSleepMs     = (uint32_t)sleepTimeoutMs;

        // Skip the very first iteration — we'd be reacting to the snapshot
        // initialization itself, not a real user change.
        if (stateSnapInit) {
            String out; buildSettingsJson(out);
            wsServer.textAll(out);
        }
        stateSnapInit = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AP / CAPTIVE PORTAL MODE
// ─────────────────────────────────────────────────────────────────────────────
static void startPortalMode() {
    wifiServerMode = WMODE_PORTAL;

    Serial.printf("[WiFi] === startPortalMode entry  heap=%u ===\n", ESP.getFreeHeap());
    Serial.flush();

    // Reset any stale WiFi driver state from a previous boot/init attempt.
    WiFi.persistent(false);   // don't write to NVS — we manage our own creds
    WiFi.disconnect(true, true);
    delay(100);

    bool mOk = WiFi.mode(WIFI_AP);
    Serial.printf("[WiFi] mode(AP) = %d  heap=%u\n", mOk, ESP.getFreeHeap());
    Serial.flush();
    delay(100);

    // Generate the setup code AFTER WiFi.mode() — that enables the RF
    // subsystem, which is what makes esp_random() a true hardware RNG.
    // Called before this point it would return only pseudo-random values.
    generatePortalCode();
    Serial.printf("[WiFi] Portal code: %s\n", portalCode);

    // RF is up now — safe to mint the default device password with the HW RNG.
    ensureOtaPassword();

    bool cOk = WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    Serial.printf("[WiFi] softAPConfig = %d\n", cOk); Serial.flush();

    // softAP(ssid, pass, channel, hidden, max_connection)
    //   channel 1  — least-used in most environments
    //   hidden 0   — broadcast SSID
    //   max_conn 4 — default
    bool aOk = WiFi.softAP(AP_SSID, portalCode, /*channel*/ 1, /*hidden*/ 0, /*max_conn*/ 4);
    Serial.printf("[WiFi] softAP = %d\n", aOk); Serial.flush();

    // Bump TX power to maximum allowed (19.5 dBm).
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    delay(300);  // let the AP fully start before reporting

    Serial.printf("[WiFi] AP IP      = %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WiFi] AP MAC     = %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("[WiFi] AP hostname= %s\n", WiFi.softAPgetHostname());
    Serial.printf("[WiFi] AP channel = %d\n", WiFi.channel());
    Serial.printf("[WiFi] AP clients = %d\n", WiFi.softAPgetStationNum());
    Serial.println("[WiFi] === startPortalMode AP up — should be visible NOW ===");
    Serial.flush();

    // Redirect ALL DNS queries to our IP so the OS captive-portal detector fires
    dnsServer.start(DNS_PORT, "*", AP_IP);

    // 302-redirect every unknown URL to http://192.168.4.1/ so the OS
    // captive-portal detector trips reliably:
    //   • iOS  hits /hotspot-detect.html, sees a non-"Success" 302 → portal pops
    //   • Android hits /generate_204, gets a 302 (not 204) → portal pops
    //   • Windows /connecttest.txt, same trip
    // Returning portal HTML directly on the probe URL was leaving some clients
    // in "captive portal detected — open browser manually" limbo.
    httpServer.onNotFound([](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* resp = req->beginResponse(302, "text/plain", "");
        resp->addHeader("Location", "http://192.168.4.1/");
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", PORTAL_HTML);
    });

    // POST /save — persist credentials and restart
    httpServer.on("/save", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            String ssid = req->hasParam("ssid", true)
                          ? req->getParam("ssid", true)->value() : "";
            String pass = req->hasParam("pass", true)
                          ? req->getParam("pass", true)->value() : "";
            String ip   = req->hasParam("ip",   true)
                          ? req->getParam("ip",   true)->value() : "";
            String gw   = req->hasParam("gw",   true)
                          ? req->getParam("gw",   true)->value() : "";
            // Device login password: kept verbatim (no trim) so leading/trailing
            // characters the user intended are preserved. Empty = use the default.
            String devpw = req->hasParam("devpw", true)
                          ? req->getParam("devpw", true)->value() : "";

            ssid.trim(); pass.trim(); ip.trim(); gw.trim();
            if (ssid.isEmpty()) { req->send(400, "text/plain", "SSID required"); return; }

            // Reject oversized inputs before they hit Preferences / the heap.
            // SSID ≤ 32 bytes (802.11), WPA passphrase 8-63 chars (we allow 0
            // for open networks), IP/GW max 15 chars (xxx.xxx.xxx.xxx).
            if (ssid.length() > 32 || pass.length() > 63 ||
                ip.length() > 15   || gw.length()   > 15) {
                req->send(400, "text/plain", "Input too long");
                return;
            }
            // Device password: blank (use default) or 6–63 chars.
            if (devpw.length() && (devpw.length() < 6 || devpw.length() > 63)) {
                req->send(400, "text/plain", "Device password must be 6–63 characters");
                return;
            }

            // Auto-derive gateway (x.x.x.1) when static IP is given but gateway omitted
            if (!ip.isEmpty() && gw.isEmpty()) {
                int dot = ip.lastIndexOf('.');
                if (dot != -1) gw = ip.substring(0, dot + 1) + "1";
            }

            wifiPrefs.begin("wifi", false);
            wifiPrefs.putString("ssid", ssid);
            wifiPrefs.putString("pass", pass);
            wifiPrefs.putString("ip",   ip);
            wifiPrefs.putString("gw",   gw);
            // Store or clear the custom login password (blank → fall back to the
            // auto-generated default shown on the TFT).
            if (devpw.length()) wifiPrefs.putString("devpw", devpw);
            else                wifiPrefs.remove("devpw");
            // Drop any cached BSSID/channel from older firmware revisions.
            wifiPrefs.remove("bssid");
            wifiPrefs.remove("chan");
            wifiPrefs.end();

            req->send_P(200, "text/html", PORTAL_SAVED_HTML);
            restartPendingMs = millis();   // wifiLoop() reboots once flushed
        }
    );

    httpServer.begin();
    Serial.printf("[WiFi] Portal AP: \"%s\"  IP: %s\n",
                  AP_SSID, AP_IP.toString().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// STA (station) MODE  —  non-blocking
//
// patched3: setup() previously blocked up to 15 s here, freezing touch.
// We now run WiFi.begin() + the connection wait inside a one-shot FreeRTOS
// task pinned to Core 0 (alongside AsyncTCP).  The task:
//   1. Configures static IP if provided
//   2. Calls WiFi.begin() and polls WL_CONNECTED with vTaskDelay (yields CPU)
//   3. Starts the HTTP/WS server when the wait completes (success OR timeout)
//   4. Self-deletes
//
// The task does NOT touch the TFT.  The header WiFi indicator updates via
// the loop()-side RSSI poll, which detects wifiConnected→true within 5 s.
// ─────────────────────────────────────────────────────────────────────────────
struct StaConnectArgs {
    String ssid;
    String pass;
    String staticIP;
    String gateway;
};

static void staConnectTask(void* arg) {
    StaConnectArgs* args = (StaConnectArgs*)arg;
    g_staConnecting.store(true, std::memory_order_release);   // F5: own the WiFi driver
    // Apply static IP before begin() if the user configured one
    if (!args->staticIP.isEmpty()) {
        IPAddress ip, gw, subnet(255, 255, 255, 0), dns(8, 8, 8, 8);
        if (ip.fromString(args->staticIP)) {
            if (!gw.fromString(args->gateway)) {
                int dot = args->staticIP.lastIndexOf('.');
                String gwStr = (dot != -1)
                    ? args->staticIP.substring(0, dot + 1) + "1"
                    : "192.168.1.1";
                gw.fromString(gwStr);
            }
            WiFi.config(ip, gw, subnet, dns);
            Serial.printf("[WiFi] Static IP: %s  GW: %s\n",
                          ip.toString().c_str(), gw.toString().c_str());
        } else {
            Serial.println("[WiFi] Static IP parse failed — falling back to DHCP");
        }
    }

    // V12.5 (F5): up to 2 connect attempts before giving up. A single bad 4-way
    // handshake (reason 15/204) is often transient, so we retry once to avoid
    // dropping a friend off their real network over a fluke; a genuinely wrong
    // password just fails both and routes to the portal below.
    bool connected = false;
    for (int attempt = 0; attempt < 2 && !connected; attempt++) {
        g_lastStaDisconnReason = 0;   // reflect only THIS attempt's failure
        if (attempt > 0) Serial.println("[WiFi] STA connect retry…");
        uint32_t tStart = millis();
        WiFi.begin(args->ssid.c_str(), args->pass.c_str());
        while (WiFi.status() != WL_CONNECTED && millis() - tStart < CONNECT_TIMEOUT_MS) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        connected = (WiFi.status() == WL_CONNECTED);
        // Brief grace for the async STA_DISCONNECTED event to land before we read
        // the reason below — the deauth normally arrives well within the attempt,
        // but this closes the window where a late event is missed.
        if (!connected) vTaskDelay(pdMS_TO_TICKS(300));
    }
    // NB: g_staConnecting is released on the paths that reach startFullServer()
    // below (success / non-auth failure). The auth-fail path deliberately keeps
    // it held through the portal handoff so wifiLoop can't inject a reconnect in
    // the gap between clearing it and setting g_portalFallbackReq.

    if (connected) {
        wifiConnected = true;
        Serial.printf("[WiFi] Connected!  IP: %s  RSSI: %d dBm  ch %d\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI(),
                      WiFi.channel());
        // mDNS — register only after the STA interface has an IP.
        // Heads-up: Android Chrome doesn't resolve .local from the URL bar;
        // desktop browsers and iOS do.
        ensureMdns();
    } else {
        // AUTH-class disconnect = wrong password / key exchange failure. Reopen
        // the setup portal so the friend can fix it. Deliberately EXCLUDES
        // NO_AP_FOUND (201) — a transiently-off/out-of-range AP should keep
        // retrying STA, not force the user through re-setup. On fallback we do
        // NOT start the STA server; wifiLoop (Core 1) runs startPortalMode().
        uint8_t r = g_lastStaDisconnReason;
        bool authFail = (r == WIFI_REASON_AUTH_FAIL          /* 202 */ ||
                         r == WIFI_REASON_AUTH_EXPIRE        /* 2   */ ||
                         r == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT /* 15 */ ||
                         r == WIFI_REASON_HANDSHAKE_TIMEOUT  /* 204 */);
        if (authFail) {
            Serial.printf("[WiFi] auth failure (reason %u) — reopening setup portal\n", r);
            g_portalFallbackReq.store(true, std::memory_order_release);
            delete args;
            vTaskDelete(NULL);   // NB: does not return
        }
        wifiConnected = false;
        lastReconnectMs = millis();
        Serial.printf("[WiFi] Connect failed (status %d, reason %u) — will retry every %u s\n",
                      WiFi.status(), r, RECONNECT_INTERVAL_MS / 1000);
    }

    g_staConnecting.store(false, std::memory_order_release);   // reached only by non-fallback paths

    // startFullServer registers handlers + calls httpServer.begin().
    // Doesn't touch TFT — safe from Core 0.
    startFullServer();

    delete args;
    vTaskDelete(NULL);
}

static void startStaMode(const String& ssid, const String& pass,
                          const String& staticIP, const String& gateway) {
    wifiServerMode = WMODE_STA;

    WiFi.mode(WIFI_STA);
    // V12.5 (F5): capture the STA disconnect reason so staConnectTask can tell a
    // wrong password (auth failure) from a transiently-absent AP. Non-capturing
    // lambda writing a global — runs on the WiFi event task; a single byte store.
    WiFi.onEvent([](WiFiEvent_t, WiFiEventInfo_t info) {
        g_lastStaDisconnReason = info.wifi_sta_disconnected.reason;
    }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    // Disable WiFi modem power-save. The Arduino default is WIFI_PS_MIN_MODEM,
    // which sleeps the radio between DTIM beacons; during a sustained OTA upload
    // those sleep windows drop incoming TCP segments, TCP backs off its
    // retransmit timer, and the transfer shows the intermittent multi-second
    // "burst -> stall -> recover" the serial logs captured (the flash still
    // completed, just slowly, tripping the 30 s stall watchdog first). Ruled out
    // heap/CPU/WS/RSSI first; this is the remaining default that matches the
    // symptom. The device is USB/mains powered, so the extra idle current is
    // irrelevant — and it also lowers dashboard latency in general.
    WiFi.setSleep(false);
    // Boot-banner marker so a serial capture proves WHICH build is running: the
    // version string doesn't change between these experiments, but this line
    // (and WIFI_PS_NONE below) only appears on builds that carry the fix.
    Serial.printf("[WiFi] modem power-save disabled — getSleep()=%d (0=NONE)\n",
                  (int)WiFi.getSleep());
    WiFi.setAutoReconnect(true);

    // RF is up now — safe to mint the default device password with the HW RNG.
    // Runs before staConnectTask/startFullServer, so effectivePassword() is
    // correct by the time /login is reachable.
    ensureOtaPassword();

    StaConnectArgs* args = new StaConnectArgs();
    args->ssid     = ssid;
    args->pass     = pass;
    args->staticIP = staticIP;
    args->gateway  = gateway;

    // Pin to Core 0 alongside AsyncTCP/sensor-event tasks.  Priority 1 = low.
    // 8 KB: startFullServer() calls into AsyncTCP/lwIP which needs >4 KB.
    xTaskCreatePinnedToCore(staConnectTask, "WiFiConnect",
                            8192, args, 1, NULL, 0);
}

// ── Per-IP auth bookkeeping ──────────────────────────────────────────────────
// Enrol an IP as password-authenticated (called on a successful /login).
static void authIPAdd(uint32_t ip) {
    if (ip == 0) return;
    for (int i = 0; i < AUTHED_IP_MAX; i++) if (authedIPs[i] == ip) return;   // dedupe
    authedIPs[authedIPHead] = ip;
    authedIPHead = (authedIPHead + 1) % AUTHED_IP_MAX;
}
static bool authIPKnown(uint32_t ip) {
    if (ip == 0) return false;
    for (int i = 0; i < AUTHED_IP_MAX; i++) if (authedIPs[i] == ip) return true;
    return false;
}
// Find (or allocate) this IP's brute-force entry, returned as an INDEX into
// loginFails[] (not a pointer — a function returning the sketch-local struct
// type would trip arduino-cli's auto-prototype, landing the prototype above the
// struct definition; same reason checkChallenge() returns a plain int). Full
// table → evict the entry whose lockout expires soonest, so an active lockout is
// never dropped for an idle one (and an attacker can't reset their own lockout by
// churning the table).
static int loginFailSlot(uint32_t ip) {
    for (int i = 0; i < LOGIN_FAIL_MAX; i++)
        if (loginFails[i].ip == ip) return i;
    for (int i = 0; i < LOGIN_FAIL_MAX; i++)
        if (loginFails[i].ip == 0) { loginFails[i] = { ip, 0, 0 }; return i; }
    int v = 0;
    for (int i = 1; i < LOGIN_FAIL_MAX; i++)
        if ((int32_t)(loginFails[i].lockoutUntilMs - loginFails[v].lockoutUntilMs) < 0) v = i;
    loginFails[v] = { ip, 0, 0 };
    return v;
}

// ── Request gates (STA mode) ─────────────────────────────────────────────────
// DNS-rebinding defense: only honour requests whose Host header is one we
// actually serve under (our STA IP, the mDNS name, or the AP IP). A rebinding
// page reaches us by IP but carries the attacker's own hostname in Host, so it
// fails here. Port suffix is stripped before comparison. The captive portal
// deliberately does NOT use this — it must answer for any Host so the OS
// portal-detector fires.
static bool hostAllowed(AsyncWebServerRequest* req) {
    String h = req->host();
    int colon = h.indexOf(':');
    if (colon >= 0) h = h.substring(0, colon);
    h.toLowerCase();
    return h == WiFi.localIP().toString()
        || h == "autofov.local"
        || h == "autofov"
        || h == AP_IP.toString();
}
// Per-boot session token, issued only by POST /login.
static bool tokenOk(AsyncWebServerRequest* req) {
    return req->hasHeader("X-AutoFOV-Token") &&
           req->header("X-AutoFOV-Token") == String(csrfToken);
}
// Standard gate for reading/state-changing endpoints: right Host + valid token +
// the token presented from an IP that actually logged in (token-replay defense).
static bool apiAuthed(AsyncWebServerRequest* req) {
    return hostAllowed(req) && tokenOk(req) &&
           authIPKnown((uint32_t)req->client()->remoteIP());
}

// Shared auth for BOTH OTA endpoints — the /ota-verify pre-flight and /ota's
// first upload chunk route through this one path so the brute-force throttle
// can't be sidestepped by hammering /ota directly. Returns the HTTP status the
// caller sends back:
//   200 — right Host + session token + correct password challenge-response
//   403 — wrong Host (DNS-rebinding defense)
//   409 — session token invalid, or the challenge nonce was missing/expired.
//         RETRYABLE and explicitly NOT a wrong password — the client must not
//         report it as one (re-login / fetch a fresh nonce instead).
//   401 — valid nonce but wrong password (CR_BAD): a genuine guess
//   429 — too many wrong-password guesses from this IP; locked out
// CR_BAD feeds the SAME escalating per-IP lockout as /login (5 fails → 15 s,
// doubling to a 4 min cap), so a token passively sniffed off the plaintext-HTTP
// LAN can't be replayed to guess the device password at wire speed. A correct
// challenge resets the counter, exactly like /login's CR_OK path.
static int otaAuthCheck(AsyncWebServerRequest* req) {
    if (!hostAllowed(req)) return 403;
    if (!tokenOk(req))     return 409;
    uint32_t now = millis();
    uint32_t ip  = (uint32_t)req->client()->remoteIP();
    LoginFailEntry& fe = loginFails[loginFailSlot(ip)];
    if (fe.lockoutUntilMs && (int32_t)(fe.lockoutUntilMs - now) > 0) return 429;
    if (!req->hasHeader("X-Login-Nonce") || !req->hasHeader("X-OTA-Response"))
        return 409;
    switch (checkChallenge(req->header("X-Login-Nonce"), req->header("X-OTA-Response"))) {
    case CR_OK:
        fe.fails = 0; fe.lockoutUntilMs = 0;
        return 200;
    case CR_BAD:
        if (fe.fails < 255) fe.fails++;
        if (fe.fails >= 5) { uint8_t shift = fe.fails - 5; if (shift > 4) shift = 4;
                             fe.lockoutUntilMs = now + (15000UL << shift); }
        return 401;
    default:  // CR_NO_CHALLENGE — stale/missing/evicted nonce
        return 409;
    }
}

// One-line body for an otaAuthCheck() status — shared by /ota-verify and /ota.
static const char* otaAuthMsg(int st) {
    return st == 200 ? "OK"
         : st == 401 ? "Unauthorized"
         : st == 429 ? "Too many attempts — wait a bit"
         : st == 403 ? "Forbidden"
         :             "Session or challenge expired — re-login";
}

// OTA flush task (Core 1) — writes the PSRAM-buffered .bin to flash AFTER the
// upload socket has closed, so no flash write ever competes with WiFi RX.  This
// task is the SOLE owner of Update.* and the mbedtls SHA for the buffered path,
// so there is no cross-core Updater race.  The ONLY irreversible step —
// Update.end(true), which flips the boot partition — runs only after the SHA
// matches, so any failure here leaves the device on the current firmware (dual-
// OTA rollback also stays intact).  Frees the PSRAM buffer and self-deletes.
static void otaFlushTaskFn(void*) {
    const size_t total = otaBufLen;
    otaFlashState = 1;                                     // flushing
    const char* fail = nullptr;                            // set = failure reason
    do {
        if (!total)                       { fail = "empty image"; break; }
        if (!Update.begin(total)) {                       // exact size known now
            Serial.printf("[OTA] flush: begin failed: %s\n", Update.errorString());
            fail = "flash begin failed"; break;
        }
        mbedtls_sha256_context sha; mbedtls_sha256_init(&sha);
        mbedtls_sha256_starts(&sha, 0);
        bool werr = false;
        for (size_t off = 0; off < total; ) {
            size_t n = total - off; if (n > 4096) n = 4096;
            mbedtls_sha256_update(&sha, otaBuf + off, n);
            if (Update.write(otaBuf + off, n) != n) {
                Serial.printf("[OTA] flush: write error at %u: %s\n",
                              (unsigned)off, Update.errorString());
                werr = true; break;
            }
            off += n;
            if ((off & 0x1FFFF) == 0) vTaskDelay(1);       // yield ~every 128 KB
        }
        uint8_t digest[32]; mbedtls_sha256_finish(&sha, digest); mbedtls_sha256_free(&sha);
        char hex[65]; for (int i = 0; i < 32; i++) snprintf(hex + i*2, 3, "%02x", digest[i]);
        hex[64] = '\0';
        if (werr) { Update.abort(); fail = "flash write error"; break; }
        if (otaExpectedSha[0] && strcmp(hex, otaExpectedSha) != 0) {
            Serial.printf("[OTA] flush: sha mismatch got %s expected %s — refusing to flash\n",
                          hex, otaExpectedSha);
            Update.abort(); fail = "SHA-256 mismatch — image rejected"; break;
        }
        if (!Update.end(true)) {                           // COMMIT — only after SHA ok
            Serial.printf("[OTA] flush: end error: %s\n", Update.errorString());
            fail = "flash commit failed"; break;
        }
        Serial.printf("[OTA] flush: success, %u bytes, sha ok\n", (unsigned)total);
    } while (0);

    otaBufFree();
    if (!fail) {
        otaFlashState       = 2;                           // committed — rebooting
        otaRestartPendingMs = millis();                    // wifiLoop (Core 1) reboots
    } else {
        // Stay on the current firmware.  Publish the reason for GET /ota-status,
        // then resume telemetry so the dashboard recovers.
        strncpy(otaFlashMsg, fail, sizeof(otaFlashMsg) - 1);
        otaFlashMsg[sizeof(otaFlashMsg) - 1] = '\0';
        otaFlashState  = -1;                               // failed
        otaInProgress  = false;
        otaLastChunkMs = 0;
    }
    otaFlushTask = nullptr;
    vTaskDelete(NULL);
}

// ─────────────────────────────────────────────────────────────────────────────
// FULL HTTP + WEBSOCKET SERVER  (STA mode)
// ─────────────────────────────────────────────────────────────────────────────
static void startFullServer() {
    // Generate the per-boot CSRF token before any handler that consults it
    // is registered. Called from staConnectTask on Core 0 once WiFi.mode(STA)
    // has already enabled the RF subsystem, so esp_random() is HW-backed here.
    generateCsrfToken();

    // ── Auth / CORS policy ───────────────────────────────────────────────────
    // The session token is no longer handed out for free: POST /login verifies
    // the device password (custom, or the auto default shown on the TFT) before
    // issuing it. A client without the password — browser or raw socket — never
    // gets a token, so it can't reach /state, /cmd, the WebSocket, or /ota. That
    // makes the password the real authentication boundary, not just anti-CSRF.
    // Layered with it:
    //   * No Access-Control-Allow-Origin header is ever added, so cross-origin
    //     browser JS still can't read /login's response even on the same LAN.
    //   * hostAllowed() rejects mismatched Host headers, defeating DNS rebinding.
    //   * Challenge-response: the password never crosses the wire — the client
    //     proves it by returning HMAC-SHA256(password, nonce) for a one-time nonce.
    // Endpoint gates:
    //   * /login-challenge: hostAllowed → issues a single-use 30 s nonce
    //   * /login:       hostAllowed + valid HMAC over the nonce → returns the token
    //   * /state /cmd /forget-wifi /vibsig: apiAuthed (host + token)
    //   * /ota:         apiAuthed + valid HMAC over a nonce (X-Login-Nonce/X-OTA-Response)
    //   * /ota-verify:  same gates as /ota, no body — pre-flight so a wrong
    //                   password fails before the .bin uploads
    //   * WS:           hostAllowed + ?tok= matching csrfToken (below)
    //   * /ping:        hostAllowed only — liveness, reveals nothing

    // WS upgrade gate — reject any handshake without the right Host and the
    // session token in the query string. setFilter() runs before WS_EVT_CONNECT
    // so unauthenticated clients never receive the full-state push.
    wsServer.setFilter([](AsyncWebServerRequest* req) {
        if (!hostAllowed(req) || !req->hasParam("tok")) return false;
        if (req->getParam("tok")->value() != String(csrfToken)) return false;
        // Same token-replay defense as apiAuthed: the upgrade must come from an
        // IP that completed /login this boot.
        return authIPKnown((uint32_t)req->client()->remoteIP());
    });
    wsServer.onEvent(onWsEvent);
    httpServer.addHandler(&wsServer);

    // GET /login-challenge — issue a fresh single-use nonce (host-gated, no auth).
    // The response body is hidden from cross-origin JS (no CORS), so a drive-by
    // page can't read a nonce and therefore can't even attempt /login.
    httpServer.on("/login-challenge", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!hostAllowed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        const char* nonce = generateLoginNonce();
        AsyncWebServerResponse* resp = req->beginResponse(200, "text/plain", nonce);
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });

    // POST /login — challenge-response. The client returns the issued nonce
    // (X-Login-Nonce) and HMAC-SHA256(password, nonce) (X-Login-Response); the
    // password never transmits. Escalating lockout guards online guessing; only
    // a valid-nonce wrong-HMAC (a real guess) counts toward it.
    httpServer.on("/login", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!hostAllowed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        uint32_t now = millis();
        uint32_t ip  = (uint32_t)req->client()->remoteIP();
        LoginFailEntry& fe = loginFails[loginFailSlot(ip)];   // per-IP brute-force state
        if (fe.lockoutUntilMs && (int32_t)(fe.lockoutUntilMs - now) > 0) {
            uint32_t secs = (fe.lockoutUntilMs - now + 999) / 1000;
            AsyncWebServerResponse* r = req->beginResponse(
                429, "text/plain", "Too many attempts; wait " + String(secs) + "s");
            r->addHeader("Retry-After", String(secs));
            req->send(r);
            return;
        }
        // Both custom headers required. A browser can't attach them cross-origin
        // without a (failing) preflight, so a drive-by page can't reach the
        // lockout path; missing headers are a 400 that never counts.
        if (!req->hasHeader("X-Login-Nonce") || !req->hasHeader("X-Login-Response")) {
            req->send(400, "text/plain", "Missing challenge response");
            return;
        }
        switch (checkChallenge(req->header("X-Login-Nonce"), req->header("X-Login-Response"))) {
        case CR_OK: {
            fe.fails = 0;
            fe.lockoutUntilMs = 0;
            authIPAdd(ip);                 // enrol this IP so its token is honoured
            AsyncWebServerResponse* resp = req->beginResponse(200, "text/plain", csrfToken);
            resp->addHeader("Cache-Control", "no-store");
            req->send(resp);
            return;
        }
        case CR_BAD: {
            if (fe.fails < 255) fe.fails++;
            if (fe.fails >= 5) {
                // 5 fails → 15 s, doubling to a 4 min cap; persists until success.
                uint8_t shift = fe.fails - 5;
                if (shift > 4) shift = 4;     // 15 s << 4 = 240 s; also bounds the shift
                fe.lockoutUntilMs = now + (15000UL << shift);
            }
            Serial.printf("[LOGIN] failed attempt #%u from %s\n",
                          fe.fails, req->client()->remoteIP().toString().c_str());
            req->send(401, "text/plain", "Unauthorized");
            return;
        }
        default:  // CR_NO_CHALLENGE — stale/missing/mismatched nonce; not a guess
            req->send(400, "text/plain", "Challenge expired — retry");
            return;
        }
    });

    // GET /ping — unauthenticated liveness probe (host-gated). The dashboard
    // polls it after an OTA reboot to know when the device is back. Returns
    // nothing sensitive; never the token.
    httpServer.on("/ping", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!hostAllowed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        AsyncWebServerResponse* resp = req->beginResponse(200, "text/plain", "ok");
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });

    // Serve HTML from firmware PROGMEM (gzip — regenerate web_ui.h via tools/embed_html.py)
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!hostAllowed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        AsyncWebServerResponse* resp = req->beginResponse_P(
            200, "text/html", WEB_UI_HTML_GZ, WEB_UI_HTML_GZ_LEN);
        resp->addHeader("Content-Encoding", "gzip");
        resp->addHeader("Cache-Control", "no-cache");
        req->send(resp);
    });

    // GET /state — full JSON snapshot (used by HTML on initial WiFi connect).
    // Gated by the same CSRF token as /cmd: the dashboard fetches /token first,
    // then includes X-AutoFOV-Token here. Closes a same-LAN snooping vector
    // where a raw HTTP client could read calibration, SSID/RSSI, and live
    // sensor values without authenticating.
    httpServer.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!apiAuthed(req)) {
            req->send(403, "text/plain", "Forbidden");
            return;
        }
        String out; buildFullStateJson(out);
        req->send(200, "application/json", out);
    });

    // GET /diag — V12.5 boot/crash diagnostics. Its OWN JSON doc, deliberately
    // NOT folded into buildFullStateJson (keeps that 4096 budget untouched).
    // Reading g_rtc here (Core 0/AsyncTCP) is a plain memory read; the ring is a
    // read-only NVS open. apiAuthed-gated like /state — reveals only reset
    // history + live heap, nothing sensitive.
    httpServer.on("/diag", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!apiAuthed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        DynamicJsonDocument doc(2048);
        doc["resetReasonNow"] = diagReasonStr((uint8_t)esp_reset_reason());
        doc["bootCount"]  = g_rtc.bootCount;
        doc["uptimeSec"]  = millis() / 1000;
        doc["freeHeap"]   = ESP.getFreeHeap()    / 1024;
        doc["minHeap"]    = ESP.getMinFreeHeap() / 1024;
        doc["maxSensorStallMs"] = sensorMaxStallMs.load(std::memory_order_relaxed);
        JsonArray ring = doc.createNestedArray("ring");
        Preferences dp;
        dp.begin("diag", true);   // read-only
        uint8_t head  = dp.getUChar("head", 0);
        uint8_t count = dp.getUChar("count", 0);
        // Walk newest → oldest so the dashboard shows the most recent reset first.
        for (int i = 0; i < count; i++) {
            int idx = (head - 1 - i + 2 * DIAG_RING_LEN) % DIAG_RING_LEN;
            char key[8]; snprintf(key, sizeof(key), "e%d", idx);
            DiagEntry e;
            if (dp.getBytes(key, &e, sizeof(e)) != sizeof(e)) continue;
            char ver[13]; memcpy(ver, e.version, 12); ver[12] = '\0';
            JsonObject o = ring.createNestedObject();
            o["reason"]    = diagReasonStr(e.reason);
            o["uptimeSec"] = e.uptimeSec;
            o["minHeap"]   = e.minHeap;
            o["slot"]      = e.bootSlot ? "ota_1" : "ota_0";
            o["ver"]       = ver;
        }
        dp.end();
        String out; serializeJson(doc, out);
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", out);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // POST /cmd — fallback for commands when WebSocket is not yet open.
    //
    // patched3 (CRITICAL): AsyncWebServer dispatches ALL HTTP body callbacks
    // on the AsyncTCP/lwIP task, which is pinned to Core 0.  Calling
    // handleWifiCommand() directly from here would execute analogWrite,
    // NVS writes, I²C-mutex acquisition, and TFT redraws (via
    // finalizeCalibration) on Core 0 while the main UI runs on Core 1 —
    // an immediate mutex/hardware collision.
    // We enqueue commands and let wifiLoop() (running on Core 1) dispatch
    // them, mirroring the WebSocket handler's behaviour.
    httpServer.on("/cmd", HTTP_POST,
        [](AsyncWebServerRequest* req) { /* headers only — body handled below */ },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            // Auth gate — host + session token (see startFullServer notes).
            if (!apiAuthed(req)) {
                req->send(403, "text/plain", "Forbidden");
                return;
            }
            if (total > 512) {
                if (index == 0) req->send(400, "text/plain", "Payload too large");
                return;
            }
            // The library delivers the body per TCP segment — parse only once
            // the final chunk lands. Treating each segment as a complete body
            // silently dropped any command split across segments while still
            // answering {"ok":1}. Single-segment bodies (the overwhelmingly
            // common case at our 512-byte cap) parse from the stack; only a
            // genuinely split body heap-accumulates via _tempObject (freed by
            // the request destructor) — no per-command calloc on the WiFi heap.
            char stackBuf[513];
            char* buf;
            if (index == 0 && len == total) {
                memcpy(stackBuf, data, len);
                buf = stackBuf;
            } else {
                if (!req->_tempObject) req->_tempObject = calloc(1, 513);
                if (!req->_tempObject) { req->send(500, "text/plain", "OOM"); return; }
                buf = (char*)req->_tempObject;
                if (index + len <= 512) memcpy(buf + index, data, len);
                if (index + len < total) return;   // more segments coming
            }
            buf[total] = '\0';
            StaticJsonDocument<512> doc;   // sized to the 512-byte payload cap above
            if (deserializeJson(doc, buf) == DeserializationError::Ok) {
                for (JsonPair kv : doc.as<JsonObject>()) {
                    WifiCmd cmd;
                    strncpy(cmd.key, kv.key().c_str(), sizeof(cmd.key) - 1);
                    cmd.key[sizeof(cmd.key) - 1] = '\0';

                    String vStr;
                    if      (kv.value().is<const char*>()) vStr = kv.value().as<const char*>();
                    else if (kv.value().is<double>())       vStr = String(kv.value().as<double>(), 6);
                    else if (kv.value().is<long>())         vStr = String(kv.value().as<long>());
                    else if (kv.value().is<bool>())         vStr = kv.value().as<bool>() ? "1" : "0";
                    else serializeJson(kv.value(), vStr);

                    strncpy(cmd.val, vStr.c_str(), sizeof(cmd.val) - 1);
                    cmd.val[sizeof(cmd.val) - 1] = '\0';
                    xQueueSend(wifiCmdQueue, &cmd, (TickType_t)0);
                }
                req->send(200, "application/json", "{\"ok\":1}");
            } else {
                // Bad JSON used to be silently dropped with a 200 {"ok":1}
                req->send(400, "application/json", "{\"ok\":0}");
            }
        }
    );

    // GET /calib — full-precision calibration export. The /state copy is rounded
    // for the dashboard; a backup/library file wants the real numbers. Reading
    // the calib globals here (Core 0) is a plain memory read — they only mutate
    // on Core 1 during an explicit cal/import, never mid-request.
    httpServer.on("/calib", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!apiAuthed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        DynamicJsonDocument doc(2048);
        doc["fmt"]        = "autofov-calib";
        doc["ver"]        = 1;
        doc["device"]     = "AutoFOV V12";
        doc["isCustom"]   = isCustomCalib ? 1 : 0;
        doc["calWidth"]   = sensorWidthPixels;
        doc["demarcDist"] = demarcationDist;
        doc["calPoints"]  = nPoints;
        JsonObject fit = doc.createNestedObject("fit");   // informational only
        fit["slope"]     = CTRLX;
        fit["intercept"] = CTRLY;
        fit["r2"]        = CALIB_R2;
        fit["err"]       = CALIB_ERROR;
        int n = constrain(pointsCaptured, 0, 20);
        JsonArray pts = doc.createNestedArray("points");
        for (int i = 0; i < n; i++) {
            JsonObject p = pts.createNestedObject();
            p["dist"] = distPoints[i];
            p["fov"]  = fovPoints[i];
        }
        String out; serializeJson(doc, out);
        req->send(200, "application/json", out);
    });

    // POST /calib — import a calibration. Auth-gated like /cmd. The body is
    // parsed + range-checked HERE on Core 0 (parse only — no TFT/I²C/NVS),
    // staged into pendingCalib, then a one-word "calApply" hops to Core 1 which
    // runs finalizeCalibration(). The points are re-fit on-device, so a file's
    // stored slope is never trusted. A local cal in progress on the TFT owns
    // the arrays, so we refuse while it runs.
    httpServer.on("/calib", HTTP_POST,
        [](AsyncWebServerRequest* req) { /* headers only — body handled below */ },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
            if (!apiAuthed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
            if (total > 4096)    { req->send(400, "text/plain", "Payload too large"); return; }
            // Accumulate chunks. Body callbacks for one request are serialised on
            // the AsyncTCP task, and a calibration import is a rare manual action,
            // so a single static buffer is sufficient (no concurrent imports).
            static String body;
            if (index == 0) body = "";
            body.reserve(total + 1);
            body.concat((const char*)data, len);
            if (index + len < total) return;            // wait for the final chunk

            DynamicJsonDocument doc(4096);
            DeserializationError err = deserializeJson(doc, body);
            body = String();                            // free early
            if (err) { req->send(400, "text/plain", "Bad JSON"); return; }
            if (strcmp(doc["fmt"] | "", "autofov-calib") != 0) {
                req->send(400, "text/plain", "Not an AutoFOV calibration"); return;
            }
            if (isLocalCalActive()) {
                req->send(409, "text/plain", "Calibration in progress"); return;
            }

            float width  = doc["calWidth"]   | 0.0f;
            float demarc = doc["demarcDist"] | 0.0f;
            JsonArray pts = doc["points"].as<JsonArray>();
            if (width < 100.0f || width > 30000.0f ||
                demarc < 0.01f || demarc > 5.0f ||
                pts.isNull() || pts.size() < 2 || pts.size() > 20) {
                req->send(400, "text/plain", "Out-of-range calibration"); return;
            }
            pendingCalib.width  = width;
            pendingCalib.demarc = demarc;
            pendingCalib.n      = 0;
            strncpy(pendingCalib.name, doc["name"] | "", sizeof(pendingCalib.name) - 1);
            pendingCalib.name[sizeof(pendingCalib.name) - 1] = '\0';
            for (JsonObject p : pts) {
                float d = p["dist"] | NAN;
                float f = p["fov"]  | NAN;
                if (!isfinite(d) || !isfinite(f)) {
                    req->send(400, "text/plain", "Non-finite point"); return;
                }
                pendingCalib.dist[pendingCalib.n] = d;
                pendingCalib.fov[pendingCalib.n]  = f;
                pendingCalib.n++;
            }
            pendingCalibReady.store(true, std::memory_order_release);

            WifiCmd cmd;
            strncpy(cmd.key, "calApply", sizeof(cmd.key) - 1);
            cmd.key[sizeof(cmd.key) - 1] = '\0';
            cmd.val[0] = '\0';
            xQueueSend(wifiCmdQueue, &cmd, (TickType_t)0);
            req->send(200, "application/json", "{\"ok\":1}");
        }
    );

    // POST /forget-wifi — clear credentials and return to portal on next restart.
    // Safe to run on Core 0: only NVS write (Preferences is internally serialised)
    // and ESP.restart() (core-agnostic). The delay() blocks the async task but
    // we're about to reboot, so request servicing is irrelevant.
    httpServer.on("/forget-wifi", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!apiAuthed(req)) {
            req->send(403, "text/plain", "Forbidden");
            return;
        }
        wifiPrefs.begin("wifi", false);
        wifiPrefs.clear();
        wifiPrefs.end();
        req->send(200, "text/plain", "WiFi credentials cleared. Restarting into setup portal…");
        restartPendingMs = millis();   // wifiLoop() reboots once flushed
    });

    // Serve individual signature .bin files — the web dashboard fetches these
    // to overlay a saved spectrum in the panel.  URL: /vibsig/<name>.bin
    // Regex routes require -DASYNCWEBSERVER_REGEX, which the Arduino IDE does
    // not pass through.  Use a wildcard prefix route and re-join a sanitized
    // basename onto "/vibsig/" so a request like /vibsig/..%2fwifi can never
    // wander outside the directory.
    httpServer.on("/vibsig/*", HTTP_GET, [](AsyncWebServerRequest* req) {
        // Same CSRF gate as /cmd — the signature blobs aren't sensitive on
        // their own, but the auth model should be consistent across endpoints
        // so a future feature added here doesn't inherit an open door.
        if (!apiAuthed(req)) {
            req->send(403, "text/plain", "Forbidden");
            return;
        }
        String url = req->url();
        int slash = url.lastIndexOf('/');
        String base = (slash >= 0) ? url.substring(slash + 1) : url;
        if (!base.length() || !base.endsWith(".bin")) {
            req->send(400, "text/plain", "Bad name"); return;
        }
        // Whitelist matches vibRename's sanitiser plus the trailing ".bin".
        for (size_t i = 0; i < base.length(); i++) {
            char c = base[i];
            bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                      (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
            if (!ok) { req->send(400, "text/plain", "Bad name"); return; }
        }
        // No "..", no embedded slash (already excluded by the whitelist, but
        // belt-and-braces in case the loop is loosened later).
        if (base.indexOf("..") >= 0 || base.indexOf('/') >= 0) {
            req->send(400, "text/plain", "Bad name"); return;
        }
        String path = "/vibsig/" + base;
        if (!LittleFS.exists(path)) { req->send(404, "text/plain", "Not found"); return; }
        req->send(LittleFS, path, "application/octet-stream");
    });

    // ── OTA pre-flight auth check ────────────────────────────────────────────
    // The /ota handler authenticates on the FIRST upload chunk, but a mid-upload
    // rejection can't stop the browser from streaming the whole body it has
    // already committed to — so a wrong password otherwise wastes a full
    // multi-second upload before "Unauthorized" surfaces.  This endpoint runs the
    // EXACT same three gates as /ota (right Host + session token + password
    // challenge-response) against a one-time nonce, touches no file and no
    // Updater state, and answers in milliseconds.  The dashboard calls it before
    // uploading and only streams the .bin on a 200.  It must accept precisely
    // what /ota accepts — no looser (a wrong password would pass then fail at
    // /ota), no stricter (a valid flash would be blocked).  Uses its own nonce;
    // the real /ota upload fetches a fresh challenge.  No lockout counter, mirror-
    // ing /ota — the session token already gates who can reach it.
    httpServer.on("/ota-verify", HTTP_POST, [](AsyncWebServerRequest* req) {
        int st = otaAuthCheck(req);
        req->send(st, "text/plain", otaAuthMsg(st));
    });

    // GET /ota-status — async result of the buffered flash.  The buffered /ota
    // acks 200 before the flash runs, so the dashboard's reboot overlay polls
    // this to distinguish "committed → device is rebooting" from a specific
    // failure ("SHA-256 mismatch", "flash write error", …) without waiting for
    // the generic "never went offline" timeout.  Host-gated only (like /ping) so
    // the poll keeps working across the reboot's session-token rotation; it
    // reveals only the flash outcome, nothing sensitive.
    httpServer.on("/ota-status", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!hostAllowed(req)) { req->send(403, "text/plain", "Forbidden"); return; }
        const char* state = otaFlashState == 2 ? "ok"
                          : otaFlashState == 1 ? "flushing"
                          : otaFlashState <  0 ? "failed"
                          :                      "idle";
        String body = "{\"state\":\"" + String(state) + "\",\"msg\":\"" +
                      (otaFlashState < 0 ? otaFlashMsg : "") + "\"}";
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", body);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── OTA firmware update ──────────────────────────────────────────────────
    // Accepts a multipart/form-data POST with the compiled .bin as the "firmware"
    // field.  Runs entirely on the AsyncTCP task (Core 0); Update.h is safe from
    // any core.  Telemetry is suppressed via otaInProgress so the WS queue stays
    // clear during the ~3 s flash window.  Device reboots automatically on success.
    httpServer.on("/ota", HTTP_POST,
        // Completion handler — called after the last upload chunk.
        [](AsyncWebServerRequest* req) {
            // If the upload handler already responded (auth fail / abort), bail.
            // _tempObject is a small heap flag the framework free()s for us, so
            // it must be malloc'd — never a literal pointer like (void*)1.
            if (req->_tempObject != nullptr) return;
            if (otaBufMode) {
                // Buffered path: the whole .bin is in PSRAM now.  Ack immediately,
                // then flush to flash on a Core-1 task once this socket closes, so
                // no flash write competes with RX.  The browser's reboot overlay
                // polls /ping; on success the flush task reboots, on failure it
                // resumes telemetry (the "device never went offline" path).
                // Spawn the Core-1 flush task FIRST, so its create result decides
                // the HTTP status — a create failure must report 500, never a
                // false 200.  (The flush's OWN async result — SHA / write /
                // commit — can't ride this response; on failure the device simply
                // doesn't reboot, which the dashboard's reboot-overlay /ping poll
                // reports as "device never went offline — may have been rejected".)
                bool started = !otaFlushTask &&
                    xTaskCreatePinnedToCore(otaFlushTaskFn, "otaFlush", 6144, NULL,
                                            4, &otaFlushTask, 1) == pdPASS;
                AsyncWebServerResponse* resp = req->beginResponse(
                    started ? 200 : 500, "text/plain",
                    started ? "OK" : "Could not start flash — try again");
                resp->addHeader("Connection", "close");
                req->send(resp);
                if (!started) {
                    Serial.println("[OTA] flush task create failed — staying on current fw");
                    otaBufFree();
                    otaInProgress = false; otaLastChunkMs = 0;
                }
                return;
            }
            bool ok = !Update.hasError();
            String msg = ok ? "OK" : String(Update.errorString());
            AsyncWebServerResponse* resp =
                req->beginResponse(ok ? 200 : 500, "text/plain", msg);
            resp->addHeader("Connection", "close");
            req->send(resp);
            if (ok) {
                // Hand off restart to wifiLoop — must NOT block here or the
                // response never leaves the AsyncTCP send queue before reboot.
                otaRestartPendingMs = millis();
            } else {
                if (otaShaActive) { mbedtls_sha256_free(&otaShaCtx); otaShaActive = false; }
                otaInProgress  = false;
                otaLastChunkMs = 0;
            }
        },
        // Upload chunk handler — streams the binary into the OTA partition.
        [](AsyncWebServerRequest* req, const String& filename,
           size_t index, uint8_t* data, size_t len, bool final) {
            // Skip every subsequent chunk once we've decided to abort. Without
            // this, an auth-failed first chunk still saw later Update.write()
            // calls (no-ops since Update.begin never ran) and produced a second
            // response in the completion handler.
            if (req->_tempObject != nullptr) return;

            auto markAborted = [req]() {
                if (req->_tempObject == nullptr) req->_tempObject = malloc(1);
            };

            if (!index) {
                // Right Host + per-boot session token + the device password
                // re-typed here (challenge-response, so the password never
                // crosses the wire even on this irreversible action). Same gates
                // as the /ota-verify pre-flight — routed through otaAuthCheck() so
                // a wrong-password guess here ALSO counts toward the per-IP
                // lockout; a stolen mid-session token alone still can't flash, and
                // can't be used to brute-force the password unthrottled.
                int authSt = otaAuthCheck(req);
                if (authSt != 200) {
                    markAborted();
                    req->send(authSt, "text/plain", otaAuthMsg(authSt));
                    return;
                }
                // A previous buffered flash still writing to flash?  Don't start a
                // second OTA on top of its live Updater session.
                if (otaFlushTask) {
                    markAborted();
                    req->send(409, "text/plain", "A flash is already in progress");
                    return;
                }
                // Pre-flight cleanup — runs on Core 0, the ONLY safe place to
                // touch Update.h / mbedtls.  A previous OTA abandoned mid-
                // upload (browser tab closed) leaves the Updater "running"
                // (_size > 0) and the SHA context allocated; the stall
                // watchdog deliberately does NOT clear those to avoid a
                // cross-core race.  Tear them down here so begin() starts
                // from a guaranteed-clean state.  Update.isRunning() is the
                // Updater's own "session live" flag.
                if (Update.isRunning()) {
                    Serial.println("[OTA] leftover Update session — aborting before begin");
                    Update.abort();
                }
                if (otaShaActive) { mbedtls_sha256_free(&otaShaCtx); otaShaActive = false; }
                otaBufFree();            // free any PSRAM buffer from an abandoned OTA
                otaUploadGen++;          // invalidate stale onDisconnect cleanups (see decl)
                otaFlashState = 0; otaFlashMsg[0] = '\0';   // clear the last flash result
                otaInProgress       = false;
                otaLastChunkMs      = 0;
                otaRestartPendingMs = 0;
                // SHA-256 verification is best-effort.  The web UI computes
                // the digest before uploading and sends it as X-OTA-SHA256;
                // when present, a mismatch at `final` triggers Update.abort()
                // so the bootloader never flips to a corrupt image.  We do
                // NOT require the header — browsers expose crypto.subtle only
                // in secure contexts, so over plain-HTTP LAN the dashboard
                // falls back to a pure-JS hash, and a future client without
                // SHA support shouldn't wedge OTA entirely.
                otaExpectedSha[0] = '\0';
                if (req->hasHeader("X-OTA-SHA256")) {
                    String sha = req->header("X-OTA-SHA256");
                    sha.toLowerCase();
                    if (sha.length() == 64) {
                        strncpy(otaExpectedSha, sha.c_str(), sizeof(otaExpectedSha) - 1);
                        otaExpectedSha[sizeof(otaExpectedSha) - 1] = '\0';
                    } else {
                        Serial.printf("[OTA] ignoring malformed X-OTA-SHA256 (%u chars)\n",
                                      (unsigned)sha.length());
                    }
                }
                otaInProgress  = true;
                otaLastChunkMs = millis();
                // Pick the transfer strategy for this OTA.  Preferred: buffer the
                // whole .bin into PSRAM (Content-Length gives the size up front) so
                // the receive task only memcpys and no flash write competes with
                // RX — the flush task writes it after the socket closes.  Fall back
                // to inline Update.write per chunk if the image won't fit in PSRAM
                // (keeping OTA_PSRAM_MARGIN free) or the alloc fails, so OTA never
                // breaks.  otaBufFree() above already cleared any stale buffer.
                otaBufMode = false;
                size_t cl    = req->contentLength();     // multipart body ~ .bin size
                size_t freeP = ESP.getFreePsram();
                if (cl > 0 && freeP > OTA_PSRAM_MARGIN && cl <= freeP - OTA_PSRAM_MARGIN) {
                    otaBuf = (uint8_t*)heap_caps_malloc(cl, MALLOC_CAP_SPIRAM);
                    if (otaBuf) { otaBufCap = cl; otaBufLen = 0; otaBufMode = true; }
                }
                if (otaBufMode) {
                    // SHA + Update.* run in the flush task; nothing to set up here.
                    // Free the buffer promptly if the client disconnects before
                    // completion starts the flush (abandoned upload).  This runs on
                    // the AsyncTCP task — same task as the producer, so no race —
                    // and the !otaFlushTask guard skips it once the flush owns the
                    // buffer (a completed upload has already spawned the task).
                    // The generation check makes a LATE disconnect a no-op — an
                    // abandoned half-open upload can disconnect only after a retry
                    // has taken over the buffer globals (see otaUploadGen decl).
                    const uint32_t gen = otaUploadGen;
                    req->onDisconnect([gen]() {
                        if (gen == otaUploadGen && otaBufMode && !otaFlushTask) {
                            Serial.println("[OTA] upload aborted before flush — freeing PSRAM buffer");
                            otaBufFree();
                            otaInProgress = false; otaLastChunkMs = 0;
                        }
                    });
                    Serial.printf("[OTA] start: %s — buffering %u B to PSRAM (%s)\n",
                                  filename.c_str(), (unsigned)cl,
                                  otaExpectedSha[0] ? "sha checked" : "no sha");
                } else {
                    if (otaShaActive) mbedtls_sha256_free(&otaShaCtx);
                    mbedtls_sha256_init(&otaShaCtx);
                    mbedtls_sha256_starts(&otaShaCtx, 0 /* 0 = SHA-256, not SHA-224 */);
                    otaShaActive = true;
                    Serial.printf("[OTA] start: %s — inline write (%s)\n",
                                  filename.c_str(), otaExpectedSha[0] ? "sha checked" : "no sha");
                    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                        Serial.printf("[OTA] begin failed: %s\n", Update.errorString());
                        markAborted();
                        req->send(500, "text/plain", Update.errorString());
                        mbedtls_sha256_free(&otaShaCtx); otaShaActive = false;
                        otaInProgress = false;
                        return;
                    }
                }
            }
            otaLastChunkMs = millis();
            if (otaBufMode) {
                // Buffered path: pure PSRAM memcpy, no flash, no SHA on this task.
                if (otaBufLen + len > otaBufCap) {       // never overrun the buffer
                    Serial.printf("[OTA] buffer overflow %u+%u > %u — aborting\n",
                                  (unsigned)otaBufLen, (unsigned)len, (unsigned)otaBufCap);
                    otaBufFree();
                    markAborted();
                    req->send(500, "text/plain", "Image larger than declared length");
                    otaInProgress = false; otaLastChunkMs = 0;
                    return;
                }
                memcpy(otaBuf + otaBufLen, data, len);
                otaBufLen += len;
                return;
            }
            if (otaShaActive) mbedtls_sha256_update(&otaShaCtx, data, len);
            if (Update.write(data, len) != len) {
                // Capture the specific Update error string BEFORE calling
                // Update.abort() — abort() overwrites _error to ABORT and
                // every subsequent errorString() call would return the
                // useless generic "Aborted".  The browser needs to see the
                // real cause (magic-byte, write fail, space, etc.).
                String why = Update.errorString();
                Serial.printf("[OTA] write error at %u/%u (chunk %u): %s\n",
                              (unsigned)index, (unsigned)(index + len),
                              (unsigned)len, why.c_str());
                Update.abort();
                markAborted();
                req->send(500, "text/plain", why);
                if (otaShaActive) { mbedtls_sha256_free(&otaShaCtx); otaShaActive = false; }
                otaInProgress = false;
                otaLastChunkMs = 0;
                return;
            }
            if (final) {
                // Compare SHA-256 before committing.  A mismatch means the file
                // arrived corrupt (or the user picked the wrong .bin); refuse to
                // flip the boot partition so the running firmware is preserved.
                uint8_t digest[32];
                if (otaShaActive) {
                    mbedtls_sha256_finish(&otaShaCtx, digest);
                    mbedtls_sha256_free(&otaShaCtx);
                    otaShaActive = false;
                }
                char hex[65];
                for (int i = 0; i < 32; i++) snprintf(hex + i*2, 3, "%02x", digest[i]);
                hex[64] = '\0';
                if (otaExpectedSha[0] && strcmp(hex, otaExpectedSha) != 0) {
                    Serial.printf("[OTA] sha mismatch: got %s, expected %s\n",
                                  hex, otaExpectedSha);
                    Update.abort();
                    markAborted();
                    req->send(400, "text/plain", "SHA-256 mismatch — refusing to flash");
                    otaInProgress  = false;
                    otaLastChunkMs = 0;
                    return;
                }
                if (!otaExpectedSha[0])
                    Serial.printf("[OTA] computed sha (unchecked): %s\n", hex);
                if (Update.end(true)) {
                    Serial.printf("[OTA] success: %u bytes, sha ok\n",
                                  (unsigned)(index + len));
                } else {
                    Serial.printf("[OTA] end error: %s\n", Update.errorString());
                }
                otaLastChunkMs = 0;
            }
        }
    );

    // CORS preflight — browsers send OPTIONS before any cross-origin POST with
    // a JSON body.  Catch every unknown route and reply 204 with the headers
    // already attached by DefaultHeaders.  Real 404s still get a body.
    httpServer.onNotFound([](AsyncWebServerRequest* req) {
        if (req->method() == HTTP_OPTIONS) {
            req->send(204);
        } else {
            req->send(404, "text/plain", "Not found");
        }
    });

    httpServer.begin();
    Serial.println("[WiFi] HTTP server listening on port 80 (login-gated, host-checked, queued /cmd)");
}

// A web-side delete/rename changed /vibsig contents — resync the TFT screen's
// cached list. Selection is dropped rather than remapped: the selected index
// may now point at a different file. Runs on Core 1 (wifiLoop), so touching
// the UI state and LittleFS here is safe; vibSigUiDirty makes the open
// VIB_SIGNATURES screen redraw its rows on the next refresh tick.
static void vibSigWebChanged() {
    vibSigSel = -1;
    vibSigLoadedValid = false;
    vibSigScan();
    vibSigUiDirty = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// COMMAND DISPATCHER
// Called from wifiLoop() on Core 1 — safe to call TFT, analogWrite, i2cMutex.
//
// Key reference (mirrors HTML sendCommand calls):
//   obj, stepIndex, imgs, brightness, ledEnabled, ledDuty,
//   sensorSleep, highRefl, dimMs, sleepMs, theme, tint,
//   calWidth, demarcDist, calPoints, calStart, calCapture, calUndoPoint,
//   calTruncate, resetAll, testAlert, ir, nav
// ─────────────────────────────────────────────────────────────────────────────
static void handleWifiCommand(const char* key, const char* val) {
    float fVal = atof(val);
    int   iVal = atoi(val);

    // Any genuine web interaction (menu nav, settings change, button) counts as
    // activity and resets the idle clock that drives screen + sensor timeouts.
    // The 1 Hz latency "ping" is excluded — the dashboard merely being open must
    // not keep the sensors awake. Runs on Core 1 (wifiLoop), same as the timer.
    if (strcmp(key, "ping") != 0) lastActivityTime = millis();

    // ── Objective ────────────────────────────────────────────────────────────
    if (strcmp(key, "obj") == 0) {
        if (iVal >= 1 && iVal <= 3) currentobj = iVal;
        // patched3: redraw the objective-buttons strip on the device so the
        // highlight follows the HTML change immediately.  Cheap (sprite blit).
        if (currentMode == MAIN) updateObjectiveButtons();

    // ── Stack step index ─────────────────────────────────────────────────────
    } else if (strcmp(key, "stepIndex") == 0) {
        stackStepIndex = constrain(iVal, 0, STEP_TABLE_LEN - 1);
        stackStepSize  = STEP_TABLE[stackStepIndex];
        if (currentMode == STACK_CALC) refreshStackCalcValues(true);

    // ── Stack image count ────────────────────────────────────────────────────
    } else if (strcmp(key, "imgs") == 0) {
        stackTotalImgs = constrain(iVal, 2, 50000);
        if (currentMode == STACK_CALC) refreshStackCalcValues(true);

    // ── Time per step (seconds, 0.1-60) ─────────────────────────────────────
    } else if (strcmp(key, "secStep") == 0) {
        stackTimePerStep = constrain(fVal, 0.1f, 60.0f);
        settings.stackTimePerStep = stackTimePerStep;
        calibPrefsDirty = true; lastCalibEditMs = millis();   // deferred NVS save
        if (currentMode == STACK_TIME) refreshStackTimeValues(true);

    // ── Screen brightness (LITE_PIN, analogWrite 1-255) ──────────────────────
    } else if (strcmp(key, "brightness") == 0) {
        currentBrightness = constrain(iVal, 1, 255);
        analogWrite(LITE_PIN, currentBrightness);
        calibPrefsDirty = true; lastCalibEditMs = millis();   // deferred NVS save
        if (currentMode == BRIGHTNESS_SETTINGS) refreshBrightnessSettingsValues(true);
        else if (currentMode == APP_SETTINGS)   drawOldBrightnessBar();

    // ── Trigger LED enabled flag ──────────────────────────────────────────────
    } else if (strcmp(key, "ledEnabled") == 0) {
        ledEnabled = (iVal != 0);
        analogWrite(TRIGGER_LED_PIN, ledEnabled ? (255 - currentLedDuty) : 255);
        calibPrefsDirty = true; lastCalibEditMs = millis();   // deferred NVS save
        if (currentMode == BRIGHTNESS_SETTINGS) drawBrightnessSettingsUI();  // toggle text+color changes

    // ── Trigger LED duty (0-127: max safe duty matches on-device slider) ──────
    } else if (strcmp(key, "ledDuty") == 0) {
        currentLedDuty = constrain(iVal, 0, 127);
        analogWrite(TRIGGER_LED_PIN, ledEnabled ? (255 - currentLedDuty) : 255);
        calibPrefsDirty = true; lastCalibEditMs = millis();   // deferred NVS save
        if (currentMode == BRIGHTNESS_SETTINGS) refreshBrightnessSettingsValues(true);

    // ── Sensor sleep / wake ──────────────────────────────────────────────────
    } else if (strcmp(key, "sensorSleep") == 0) {
        // Mirrors handleSensorInfoTouch: the flag flips only when the guarded
        // I2C work ran, and the sleep flag is raised BEFORE the stop so
        // sensorTask quits polling (a latched data-ready would otherwise
        // restart ranging — see the sensorSleeping declaration).
        bool wantSleep = (iVal != 0);
        if (wantSleep && !sensorSleeping) {
            sensorSleeping = true;
            sensorState.store(0, std::memory_order_release);
            sensorHealth.store(0xFF000000UL, std::memory_order_release);
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                sensor.VL53L4CX_StopMeasurement();
                xSemaphoreGive(i2cMutex);
            } else {
                sensorSleeping = false;   // stop never ran — don't claim asleep
            }
        } else if (!wantSleep && sensorSleeping) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                applyHighReflConfig();
                sensor.VL53L4CX_StartMeasurement();
                xSemaphoreGive(i2cMutex);
                sensorEmaReset.store(true, std::memory_order_release);
                sensorSleeping = false;
                tofAutoSlept = false;     // manually woken — idle-resume must
                                          // not re-configure a ranging sensor
            }                             // timeout: stay asleep
        }
        if (currentMode == SENSOR_INFO) drawSensorInfoUI();

    // ── High-reflectivity mode ────────────────────────────────────────────────
    // applyHighReflConfig() requires measurement to be stopped first — mirror
    // the device-side path (handleSensorInfoTouch) exactly: stop, reconfigure,
    // restart, then request an EMA reset so the post-change samples are clean.
    } else if (strcmp(key, "highRefl") == 0) {
        bool prevRefl = highReflMode;
        highReflMode = (iVal != 0);
        if (!sensorSleeping && highReflMode != prevRefl) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
                sensor.VL53L4CX_StopMeasurement();
                applyHighReflConfig();
                sensor.VL53L4CX_StartMeasurement();
                xSemaphoreGive(i2cMutex);
                sensorEmaReset.store(true, std::memory_order_release);
            } else {
                highReflMode = prevRefl;  // reconfig never ran — keep the old mode
            }
        }
        if (currentMode == SENSOR_INFO) drawSensorInfoUI();

    // ── Screen dim timeout ───────────────────────────────────────────────────
    } else if (strcmp(key, "dimMs") == 0) {
        dimTimeoutMs = constrain((unsigned long)fVal, DIM_MIN_MS, DIM_MAX_MS);
        displayPrefsDirty   = true;
        displayNeedsRedraw  = true;   // patched3: trigger deferred redraw for SCREEN_TIMEOUT
        lastTintDragMs = millis();

    // ── Auto-sleep timeout ────────────────────────────────────────────────────
    } else if (strcmp(key, "sleepMs") == 0) {
        sleepTimeoutMs = constrain((unsigned long)fVal, SLEEP_MIN_MS, SLEEP_MAX_MS);
        displayPrefsDirty   = true;
        displayNeedsRedraw  = true;   // patched3
        lastTintDragMs = millis();

    // ── Theme index ──────────────────────────────────────────────────────────
    // Guard: only force a repaint when the value actually changed.  A no-op
    // command (e.g. an echo of current state from the web) must NOT trip
    // displayNeedsRedraw — that would cause a full-screen flicker on every
    // unrelated device-side button press.
    } else if (strcmp(key, "theme") == 0) {
        if (iVal >= 0 && iVal < NUM_THEMES && iVal != currentThemeIndex) {
            currentThemeIndex = iVal;
            refreshCachedThemeBg();
            displayPrefsDirty   = true;
            displayNeedsRedraw  = true;   // patched3: trigger deferred TFT repaint
            lastTintDragMs      = millis();
        }

    // ── Tint / intensity ─────────────────────────────────────────────────────
    } else if (strcmp(key, "tint") == 0) {
        int newTint = constrain(iVal, 0, 100);
        if (newTint != themeIntensity) {
            themeIntensity      = newTint;
            refreshCachedThemeBg();
            displayPrefsDirty   = true;
            displayNeedsRedraw  = true;   // patched3: trigger deferred TFT repaint
            lastTintDragMs      = millis();
        }

    // ── Calibration: photo width ──────────────────────────────────────────────
    } else if (strcmp(key, "calWidth") == 0) {
        sensorWidthPixels = constrain(fVal, 100.0f, 30000.0f);
        settings.sensorWidth = sensorWidthPixels;
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: demarcation distance ────────────────────────────────────
    } else if (strcmp(key, "demarcDist") == 0) {
        demarcationDist = constrain(fVal, 0.01f, 5.0f);
        settings.demarcation = demarcationDist;
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: target point count ──────────────────────────────────────
    } else if (strcmp(key, "calPoints") == 0) {
        nPoints = constrain(iVal, 3, 20);
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: start a fresh session (clear prior pointsCaptured) ──────
    //    HTML sends this on entering cal mode so that captures land in slot 0.
    //    Without it, leftover points from a previous physical-screen capture
    //    would cause new HTML captures to mix into the old data and
    //    finalizeCalibration() would fire early.
    } else if (strcmp(key, "calStart") == 0) {
        if (isLocalCalActive()) {
            // Device user is mid-capture on the TFT — refuse silently rather
            // than zero pointsCaptured underneath them.
            StaticJsonDocument<96> nak;
            nak["calRefused"] = "localBusy";
            String out; serializeJson(nak, out);
            wsServer.textAll(out);
            return;
        }
        pointsCaptured = 0;

    // ── Calibration: capture a point ─────────────────────────────────────────
    //    iVal = pixel count measured in the photo at current distance
    //    Formula: fov_mm = (demarcationDist / pixels) * sensorWidthPixels
    } else if (strcmp(key, "calCapture") == 0) {
        if (iVal < 1) return;  // guard against zero/negative pixel counts
        if (isLocalCalActive()) {
            // Refuse — TFT is mid-capture; UI would race on the cal arrays.
            StaticJsonDocument<96> nak;
            nak["calRefused"] = "localBusy";
            String out; serializeJson(nak, out);
            wsServer.textAll(out);
            return;
        }
        uint32_t st   = sensorState.load(std::memory_order_acquire);
        bool     valid = (st >> 31) & 1;
        float    dist  = (float)(st & 0x7FFFFFFF);          // mm integer from EMA
        float    fov   = (demarcationDist / (float)iVal) * sensorWidthPixels;

        // pointsCaptured >= nPoints also rejects a stray calCapture sent
        // without calStart: a factory boot pre-seeds pointsCaptured to
        // FACTORY_N == nPoints, and one junk append used to immediately
        // finalize (and persist) a bogus calibration.
        if (!valid || pointsCaptured >= nPoints || pointsCaptured >= 20) return;

        int slot = pointsCaptured;
        distPoints[slot] = dist;
        fovPoints[slot]  = fov;
        pointsCaptured++;

        // Echo the result back — HTML review list needs the actual dist+fov values
        {
            StaticJsonDocument<128> ack;
            JsonObject r = ack.createNestedObject("calPointResult");
            r["idx"]  = slot;
            r["dist"] = roundf(dist * 10.0f) / 10.0f;
            r["fov"]  = roundf(fov * 10000.0f) / 10000.0f;
            String out; serializeJson(ack, out);
            wsServer.textAll(out);
        }

        // When all target points are captured, run the least-squares fit
        if (pointsCaptured >= nPoints) {
            lastCalibName[0] = '\0';   // web point-cal → generic success (no profile name)
            finalizeCalibration();   // computes CTRLX/CTRLY, saves NVS, sets isCustomCalib
            // finalizeCalibration() calls drawSuccessScreen() — the physical display
            // will show the CAL_SUCCESS screen briefly (same as touch-triggered flow)
            String out; buildFullStateJson(out);
            wsServer.textAll(out);
        }

    // ── Calibration: undo last captured point ────────────────────────────────
    } else if (strcmp(key, "calUndoPoint") == 0) {
        if (isLocalCalActive()) return;        // TFT mid-capture owns the arrays
        if (pointsCaptured > 0) pointsCaptured--;

    // ── Calibration: truncate to the first N points ──────────────────────────
    //    Sent by the web RETAKE flow, which discards the selected point and
    //    everything after it; the next calCapture then lands in slot N. Only
    //    ever shrinks — growing would expose stale array slots. Without this,
    //    a web retake left the device's pointsCaptured untouched, so the next
    //    capture APPENDED and the "discarded" points still entered the fit.
    } else if (strcmp(key, "calTruncate") == 0) {
        if (isLocalCalActive()) return;
        if (iVal >= 0 && iVal < pointsCaptured) pointsCaptured = iVal;

    // ── Calibration: apply an imported calibration (POST /calib) ──────────────
    //    The body was parsed + range-checked on Core 0 into pendingCalib; here
    //    on Core 1 we load the points and re-run the on-device least-squares fit
    //    (finalizeCalibration), so an imported file's stored slope is never
    //    trusted. nPoints must track the imported count or finalizeCalibration's
    //    fitN = min(pointsCaptured, nPoints) would silently drop points. It saves
    //    NVS + shows the CAL_SUCCESS screen, then we push fresh state (incl. the
    //    cal graph) to web clients — mirroring the live web-cal flow below.
    } else if (strcmp(key, "calApply") == 0) {
        if (isLocalCalActive()) return;        // TFT mid-capture owns the arrays
        if (!pendingCalibReady.load(std::memory_order_acquire)) return;
        int n = pendingCalib.n;
        if (n < 2 || n > 20) { pendingCalibReady.store(false, std::memory_order_relaxed); return; }
        sensorWidthPixels = pendingCalib.width;  settings.sensorWidth = sensorWidthPixels;
        demarcationDist   = pendingCalib.demarc; settings.demarcation = demarcationDist;
        for (int i = 0; i < n; i++) {
            distPoints[i] = pendingCalib.dist[i];
            fovPoints[i]  = pendingCalib.fov[i];
        }
        pointsCaptured = n;
        nPoints        = n;
        strncpy(lastCalibName, pendingCalib.name, sizeof(lastCalibName) - 1);
        lastCalibName[sizeof(lastCalibName) - 1] = '\0';
        pendingCalibReady.store(false, std::memory_order_relaxed);
        finalizeCalibration();                 // re-fit + NVS save + CAL_SUCCESS screen (shows the name)
        String out; buildFullStateJson(out);
        wsServer.textAll(out);

    // ── Reset to factory calibration ─────────────────────────────────────────
    //    NOTE: we manually restore defaults rather than calling finalizeCalibration()
    //    to avoid setting isCustom=1 and showing the TFT success screen.
    } else if (strcmp(key, "resetAll") == 0) {
        // Mirror resetToFactory(): the calibration INPUTS reset too, not just
        // the fit. (pointsCaptured stays at FACTORY_N so the web graph keeps
        // its 13 factory rows — the device-side reset leaves the plot empty
        // until the FOV-info GRAPH tap reseeds it.)
        sensorWidthPixels = Config::DEFAULT_SENSOR_WIDTH_PX;
        demarcationDist   = Config::DEFAULT_DEMARCATION_MM;
        nPoints        = FACTORY_N;
        pointsCaptured = FACTORY_N;
        for (int i = 0; i < FACTORY_N; i++) {
            distPoints[i] = FACTORY_DIST[i];
            fovPoints[i]  = FACTORY_FOV[i];
        }
        CTRLX        = Config::DEFAULT_CTRL_X;
        CTRLY        = Config::DEFAULT_CTRL_Y;
        CALIB_ERROR  = Config::DEFAULT_CALIB_ERROR;
        CALIB_R2     = 0.9991f;
        isCustomCalib = false;
        // Persist the reset
        settings.magic      = CALIB_MAGIC;
        settings.ctrlX      = CTRLX;
        settings.ctrlY      = CTRLY;
        settings.sensorWidth  = sensorWidthPixels;
        settings.demarcation  = demarcationDist;
        settings.calibError = CALIB_ERROR;
        settings.calibR2    = CALIB_R2;
        settings.isCustom   = 0;
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        clearCalibBackup();   // V12.5 (F4): deliberate reset — drop the custom backup
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);
        // Push updated calibration state to all HTML clients
        String out; buildFullStateJson(out);
        wsServer.textAll(out);

    // ── Test alert (beeps all connected WebSocket clients) ───────────────────
    } else if (strcmp(key, "testAlert") == 0) {
        wifiNotifyTestAlert();

    // ── OTA rollback ─────────────────────────────────────────────────────────
    //    Flip the boot partition to the *inactive* OTA slot (the previous
    //    firmware, still intact thanks to dual-OTA) and reboot.  Refuses if the
    //    inactive slot is blank — the first ever OTA flash on a fresh device
    //    leaves it that way, and rolling back to 0xFF flash would brick.
    } else if (strcmp(key, "otaRollback") == 0) {
        const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
        uint8_t magic = 0;
        if (!next || esp_partition_read(next, 0, &magic, 1) != ESP_OK || magic != 0xE9) {
            Serial.println("[OTA] rollback refused — inactive slot has no valid image");
            return;
        }
        Serial.printf("[OTA] rolling back to %s\n", next->label);
        esp_err_t err = esp_ota_set_boot_partition(next);
        if (err != ESP_OK) {
            Serial.printf("[OTA] rollback failed: %s\n", esp_err_to_name(err));
            return;
        }
        Serial.flush();
        delay(200);
        ESP.restart();

    // ── IR remote (MJKZZ stacking controller) ────────────────────────────────
    //    val = button name; fires the matching NEC code over the IR LED.
    //    Runs on Core 1 via wifiLoop(), so the blocking sendNEC() is safe here.
    } else if (strcmp(key, "ir") == 0) {
        if      (strcmp(val, "start")   == 0) { irLed.sendNEC(IR_MJKZZ_START); mjkzzStackActive = true; }
        else if (strcmp(val, "stop")    == 0) { irLed.sendNEC(IR_MJKZZ_STOP);  mjkzzStackActive = false; }
        else if (strcmp(val, "up")      == 0) irLed.sendNEC(IR_MJKZZ_UP);
        else if (strcmp(val, "down")    == 0) irLed.sendNEC(IR_MJKZZ_DOWN);
        else if (strcmp(val, "capture") == 0) irLed.sendNEC(IR_MJKZZ_CAPTURE);

    // ── Remote menu navigation (web → device) ────────────────────────────────
    //    val = web screen-id (e.g. "screen-settings"). Mirrors the web menu
    //    onto the TFT. Ignored mid-calibration and for non-syncable ids.
    //    A nav wakes the display like a physical touch; preSleepMode is set so
    //    a later wakeScreen() restores the synced screen too.
    } else if (strcmp(key, "nav") == 0) {
        // Don't interrupt an active calibration — but DO allow nav to dismiss the
        // CAL_SUCCESS screen (it's a finished-confirmation, not a capture), so a
        // web-triggered import doesn't leave the device stuck there. V12.3.
        if (inCalibFlow() && currentMode != CAL_SUCCESS) return;
        // V12.3: info overlays open via openInfoScreen() (builds the text canvas);
        // the generic mode-switch below would blit an empty canvas. The parent
        // mode is set first so the overlay's "return" target is correct.
        if (strcmp(val, "wifi-tips") == 0 || strcmp(val, "recovery-help") == 0) {
            bool wasOff = isScreenSleep || isScreenDim;
            if (wasOff) {
                isScreenSleep = false; isScreenDim = false;
                analogWrite(LITE_PIN, currentBrightness);
            }
            lastActivityTime = millis();
            bool tips    = (strcmp(val, "wifi-tips") == 0);
            DisplayMode overlay = tips ? WIFI_TIPS : RECOVERY_HELP;
            if (currentMode != overlay || wasOff) {
                currentMode  = tips ? WIFI_INFO : ABOUT;   // openInfoScreen() captures this as the return mode
                openInfoScreen(overlay);                   // builds lines + canvas, sets currentMode = overlay
                preSleepMode = overlay;
                lastModeChangeMs = millis();
            }
            lastSyncedMode = currentMode;                  // suppress the echo broadcast
            return;
        }
        for (size_t i = 0; i < kScreenMapLen; i++) {
            if (strcmp(val, kScreenMap[i].id) != 0) continue;
            DisplayMode m = kScreenMap[i].mode;
            // Wake the TFT like a physical touch would: clear dim/sleep,
            // restore the backlight, reset the idle timer.
            bool wasOff = isScreenSleep || isScreenDim;
            if (wasOff) {
                isScreenSleep = false;
                isScreenDim   = false;
                analogWrite(LITE_PIN, currentBrightness);
            }
            lastActivityTime = millis();
            if (m != currentMode || wasOff) {        // repaint on a real change or after wake
                currentMode      = m;
                preSleepMode     = m;
                lastModeChangeMs = millis();         // arm the touch-transition guard
                redrawCurrentScreen();
            }
            lastSyncedMode = currentMode;            // suppress the echo broadcast
            break;
        }

    // ── V12 vibration monitor ────────────────────────────────────────────────
    //    All run on Core 1 via wifiLoop(), so I²C / LittleFS / NVS work is safe.
    } else if (strcmp(key, "vibPanel") == 0) {
        // Gate the spectrum stream — the web panel opens/closes it.
        vibWebPanelOpen = (iVal != 0);
        if (vibWebPanelOpen) wifiPushVibSigList();

    } else if (strcmp(key, "vibReLock") == 0) {
        float d = vibDominant.load() / 10.0f;
        if (d < VIB_RES_MIN_HZ) d = VIB_RES_MIN_HZ;
        if (d > VIB_RES_MAX_HZ) d = VIB_RES_MAX_HZ;
        vibResonanceHz = d;
        vibPrefsDirty = true; lastVibEditMs = millis();

    } else if (strcmp(key, "vibSetWait") == 0) {
        // The controller's current WAIT setting, entered from the web panel.
        vibCurrentWaitMs = constrain(iVal, 0, 20000);
        vibPrefsDirty = true; lastVibEditMs = millis();

    } else if (strcmp(key, "vibShutter") == 0) {
        // 1/N s exposure used by the web analyzer's blur readout. Persisted so
        // the firmware's per-stack V/H blur stats (vibStackBlur*) apply the
        // same |sin(πfT)| weight the analyzer is showing live.
        int d = constrain(iVal, 1, 2000);
        if ((uint16_t)d != vibShutterDenom) {
            vibShutterDenom = (uint16_t)d;
            vibPrefsDirty = true; lastVibEditMs = millis();
        }

    } else if (strcmp(key, "vibCapture") == 0) {
        // val = source name. Arms a capture; vibCapturePoll() finishes it.
        // vibNameSafe enforces [A-Za-z0-9_-]{1,30} so the eventual
        // snprintf("/vibsig/%s.bin", val) in vibSigFinishCapture() can never
        // escape the directory.
        if (vibNameSafe(val, false)) {
            strncpy(vibSigCaptureName, val, sizeof(vibSigCaptureName) - 1);
            vibSigCaptureName[sizeof(vibSigCaptureName) - 1] = '\0';
            for (int b = 0; b < VIB_FFT_BINS; b++) {
                vibSigAccum[b]    = 0;
                vibSigBaseSnap[b] = vibBaseV[b];     // freeze ambient for subtraction
            }
            vibSigFrames    = 0;
            vibCapSeqSeen   = vibSpecSeq.load();
            vibSigCapturing = true;
        } else {
            Serial.printf("[vib] vibCapture rejected: bad name\n");
        }

    } else if (strcmp(key, "vibDelSig") == 0) {
        // Whitelist the full "name.bin" so `..` and `/` can't slip through.
        if (vibNameSafe(val, true)) {
            char path[48];
            snprintf(path, sizeof(path), "/vibsig/%s", val);
            LittleFS.remove(path);
            vibSigWebChanged();
            wifiPushVibSigList();
        } else {
            Serial.printf("[vib] vibDelSig rejected: bad name\n");
        }

    } else if (strcmp(key, "vibRename") == 0) {
        // val = "oldfile.bin|NewName" — rewrites the file under the new name.
        // Both halves are whitelisted before any LittleFS path is built.
        const char* bar = strchr(val, '|');
        if (bar && bar[1]) {
            char oldbase[32];
            int  ol = bar - val;
            if (ol > 31) ol = 31;
            strncpy(oldbase, val, ol);
            oldbase[ol] = '\0';
            const char* newName = bar + 1;
            if (!vibNameSafe(oldbase, true) || !vibNameSafe(newName, false)) {
                Serial.printf("[vib] vibRename rejected: bad name\n");
            } else {
                // Read into a LOCAL struct — routing through vibSigLoad()
                // clobbered vibSigLoaded, silently swapping whatever signature
                // the TFT comparison plot had selected for the renamed one.
                VibSignature sig;
                char oldp[48], newp[48], tmpp[52];
                snprintf(oldp, sizeof(oldp), "/vibsig/%s", oldbase);
                File rf = LittleFS.open(oldp, "r");
                bool loaded = rf && rf.read((uint8_t*)&sig, sizeof(sig)) == (int)sizeof(sig);
                if (rf) rf.close();
                if (loaded) {
                    strncpy(sig.name, newName, sizeof(sig.name) - 1);
                    sig.name[sizeof(sig.name) - 1] = '\0';
                    snprintf(newp, sizeof(newp), "/vibsig/%s.bin", newName);
                    snprintf(tmpp, sizeof(tmpp), "%s.tmp", newp);
                    // Atomic write: .tmp sibling, then rename. Only after the
                    // new file is durably in place do we remove the old name —
                    // any failure leaves the original signature recoverable.
                    bool ok = false;
                    File f = LittleFS.open(tmpp, "w");
                    if (f) {
                        size_t wrote = f.write((const uint8_t*)&sig, sizeof(VibSignature));
                        f.close();
                        ok = (wrote == sizeof(VibSignature));
                    }
                    if (ok) ok = LittleFS.rename(tmpp, newp);
                    if (!ok) {
                        LittleFS.remove(tmpp);
                        Serial.printf("[vib] vibRename write failed: %s\n", newp);
                    } else if (strcmp(oldp, newp) != 0) {
                        LittleFS.remove(oldp);
                    }
                    vibSigWebChanged();
                    wifiPushVibSigList();
                }
            }
        }

    // ── Activity beacon ──────────────────────────────────────────────────────
    //    The dashboard sends {"activity":1} on any pointerdown so a tap that
    //    doesn't land on a control still counts as use. The whole effect is the
    //    idle-clock reset at the top of this function (the lastActivityTime line
    //    above the obj branch) — it wakes an auto-slept TOF/IMU and holds off
    //    screen dim/sleep. Deliberately a no-op body; unlike "ping" it is NOT
    //    excluded from that reset.
    } else if (strcmp(key, "activity") == 0) {
        // intentionally empty — lastActivityTime was already refreshed above

    // ── Latency ping ─────────────────────────────────────────────────────────
    //    The web dashboard sends {"ping":1} and times the round-trip to this
    //    {"pong":1} echo.  Routed through the normal command queue (same as
    //    every other command) so the measured value reflects real command
    //    responsiveness, not just raw network RTT.
    } else if (strcmp(key, "ping") == 0) {
        wsServer.textAll("{\"pong\":1}");

    // ── AutoRemote notify URL ─────────────────────────────────────────────────
    } else if (strcmp(key, "ntfyTopic") == 0) {
        // Validate: ntfy.sh topics are alphanumeric, hyphen, underscore only.
        // Reject anything else to prevent SSRF via URL control characters (e.g. @host).
        String candidate = String(val);
        bool valid = candidate.length() <= 64;
        for (int i = 0; valid && i < (int)candidate.length(); i++) {
            char c = candidate[i];
            if (!isalnum((unsigned char)c) && c != '-' && c != '_') valid = false;
        }
        if (!valid) {
            Serial.println("[NTFY] invalid topic rejected");
            return;
        }
        ntfyTopic = candidate;
        wifiPrefs.begin("wifi", false);
        wifiPrefs.putString("ntfy", ntfyTopic);
        wifiPrefs.end();
        Serial.printf("[NTFY] topic set: %s\n", ntfyTopic.c_str());

    } else if (strcmp(key, "ntfyTest") == 0) {
        if (ntfyTopic.length() > 0) {
            NtfyMsg* msg = new NtfyMsg{ ntfyTopic, "AutoFOV test" };
            xTaskCreate(ntfyTask, "ntfy", 4096, msg, 1, nullptr);
            Serial.println("[NTFY] test fired");
        } else {
            Serial.println("[NTFY] test skipped — no topic set");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON BUILDERS
// ─────────────────────────────────────────────────────────────────────────────

// Full state — sent on GET /state and when a new WebSocket client connects.
// Pass includeCalGraph=false on reconnect / periodic sends; true only on new-client
// connect, after calibration changes, and on explicit /state requests.
static void buildFullStateJson(String& out, bool includeCalGraph) {
    // Read all atomics in one burst
    uint32_t st  = sensorState.load(std::memory_order_acquire);
    uint32_t hst = sensorHealth.load(std::memory_order_acquire);
    uint32_t amb = sensorAmbient.load(std::memory_order_acquire);
    bool    valid   = (st >> 31) & 1;
    int     dist_mm = st & 0x7FFFFFFF;
    uint8_t status  = (hst >> 24) & 0xFF;
    float   signal  = (hst & 0xFFFFFF) / 1000.0f;
    float   ambient = (amb & 0xFFFFFF) / 1000.0f;

    // FOV — use the 5-sample averaged distance so the connect snapshot matches
    // the value the ~30 Hz fast-telemetry stream sends (buildFastTelemJson).
    float avgDist = sensorAvgDist.load(std::memory_order_acquire) / 10.0f;
    float mult = (currentobj == 1) ? mul_5x : (currentobj == 2) ? mul_10x : mul_20x;
    float fov  = valid ? (mult * fovAt(avgDist)) : 0.0f;

    // 4096 (was 3584): the vibsigs array now stores COPIED name strings (the
    // fix for the dangling File::name() pointer below), so the pool must hold up
    // to VIB_SIG_MAX(12) × VIB_SIG_NAME_MAX(15) chars on top of the state frame.
    DynamicJsonDocument doc(4096);

    // ── Live sensor ──────────────────────────────────────────────────────────
    doc["dist"]          = valid ? roundf(dist_mm * 10.0f) / 10.0f : 0;
    doc["fov"]           = roundf(fov * 1000.0f) / 1000.0f;
    doc["signal"]        = roundf(signal * 10.0f) / 10.0f;
    doc["sensorSignal"]  = doc["signal"];       // HTML uses both keys
    doc["sensorAmbient"] = roundf(ambient * 100.0f) / 100.0f;
    doc["sensorStatus"]  = status;
    doc["trigger"]       = shutterActive ? 1 : 0;   // patched3

    // ── Settings ─────────────────────────────────────────────────────────────
    doc["obj"]           = currentobj;
    doc["stepIndex"]     = stackStepIndex;
    doc["imgs"]          = stackTotalImgs;
    // secStep was only in buildSettingsJson, so the web's Sec/Step sat at its
    // 3.3 default until some other device-side change pushed a settings frame.
    doc["secStep"]       = stackTimePerStep;
    // V12.3: measured stack time (0 sec/step until a stack has been measured).
    doc["measStep"]      = roundf(measuredPerStep * 10.0f) / 10.0f;
    doc["measSh"]        = (int)measuredPulses;
    doc["stackActive"]   = mjkzzStackActive ? 1 : 0;   // V12.3: MJKZZ stack running?
    doc["brightness"]    = currentBrightness;
    doc["ledEnabled"]    = ledEnabled ? 1 : 0;
    doc["ledDuty"]       = currentLedDuty;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode ? 1 : 0;
    doc["dimMs"]         = (uint32_t)dimTimeoutMs;
    doc["sleepMs"]       = (uint32_t)sleepTimeoutMs;
    doc["theme"]         = currentThemeIndex;
    doc["tint"]          = themeIntensity;

    // ── Calibration fit ───────────────────────────────────────────────────────
    doc["isCustomCalib"] = isCustomCalib ? 1 : 0;
    doc["fovSlope"]      = roundf(CTRLX * 1e6f) / 1e6f;   // 6 dp — slope is ~0.000xxx
    doc["fovIntercept"]  = roundf(CTRLY * 1000.0f) / 1000.0f;
    // 5 dp: a near-perfect pixel-space fit (R²≈0.9991, RMSE≈0.0060) must not get
    // rounded to 1.000 / 0.005 over the wire — that defeated the web's 4-dp display.
    doc["calibError"]    = roundf(CALIB_ERROR * 100000.0f) / 100000.0f;
    doc["calibR2"]       = roundf(CALIB_R2 * 100000.0f) / 100000.0f;
    doc["calWidth"]      = (int)sensorWidthPixels;
    doc["demarcDist"]    = roundf(demarcationDist * 100.0f) / 100.0f;
    doc["calPoints"]     = nPoints;

    // Cal graph scatter data — only sent when calibration changes or on new connect.
    if (includeCalGraph) {
        int n = constrain(pointsCaptured, 0, 20);
        JsonArray pts = doc.createNestedArray("calGraphPoints");
        for (int i = 0; i < n; i++) {
            JsonObject p = pts.createNestedObject();
            p["dist"] = roundf(distPoints[i] * 10.0f) / 10.0f;
            p["fov"]  = roundf(fovPoints[i] * 1000.0f) / 1000.0f;
        }
    }

    // ── Memory ───────────────────────────────────────────────────────────────
    doc["freeHeap"]   = ESP.getFreeHeap()    / 1024;
    doc["totalHeap"]  = ESP.getHeapSize()    / 1024;
    doc["lowestFree"] = ESP.getMinFreeHeap() / 1024;
    doc["spriteKB"]   = SPRITE_BYTES / 1024;
    doc["psramFree"]  = ESP.getFreePsram()   / 1024;
    doc["psramTotal"] = ESP.getPsramSize()   / 1024;
    doc["cpu"]        = getCpuFrequencyMhz();
    doc["sdk"]        = ESP.getSdkVersion();

    // ── About / build ─────────────────────────────────────────────────────────
    doc["buildVersion"] = BUILD_VERSION;
    doc["buildCommit"]  = BUILD_COMMIT;
    doc["buildDate"]   = __DATE__;
    doc["buildTime"]   = __TIME__;
    // Per-slot OTA size (tools/partitions.csv: app0/app1 = 0x1D0000 each) and
    // which slot the bootloader picked at startup.  The memory screen draws
    // both slots so the dual-OTA layout is visible at a glance.
    doc["partKB"]      = 0x1D0000 / 1024;
    doc["sketchKB"]    = ESP.getSketchSize() / 1024;
    {
        const esp_partition_t* run = esp_ota_get_running_partition();
        // "Free" = real headroom in the RUNNING app slot (capacity − sketch), NOT
        // ESP.getFreeSketchSpace(), which returns the identically-sized standby
        // OTA slot.  That 1.8 MB is reserved for the A/B rollback image, not app
        // growth, so counting it as free flash was misleading.
        uint32_t slot = run ? run->size : 0x1D0000;
        uint32_t used = ESP.getSketchSize();
        doc["freeFlashKB"] = (slot > used ? slot - used : 0) / 1024;
        doc["partRunning"] = (run && run->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_1)
                             ? "ota_1" : "ota_0";
    }
    // Rollback target — the inactive OTA slot.  Only advertise rollback to the
    // UI if its first byte matches the ESP image magic (0xE9); a freshly-flashed
    // factory image leaves the other slot blank and rolling back to it would
    // brick the device.
    {
        const esp_partition_t* next = esp_ota_get_next_update_partition(NULL);
        bool ok = false;
        if (next) {
            uint8_t magic = 0;
            if (esp_partition_read(next, 0, &magic, 1) == ESP_OK && magic == 0xE9) {
                ok = true;
                // otaRollbackInfo is the inactive slot's self-recorded version
                // (see wifiSetup).  Empty when that slot was last written by a
                // build predating this feature, or by a USB recovery flash —
                // honest "unknown" beats the bogus frozen esp_app_desc date.
                doc["rollbackBuilt"] = otaRollbackInfo.length()
                                       ? otaRollbackInfo : String("unknown");
            }
        }
        doc["rollbackAvailable"] = ok;
    }
    // SoC junction temperature.  arduino-esp32 v3.x returns Celsius on the S3
    // (older cores returned Fahrenheit — confirm the unit if porting).  Useful
    // for deciding whether the chip needs a heatsink in your enclosure.
    // Cached — this builder also runs on Core 0 (see gDieTempC10).
    doc["dieC"]        = (int32_t)gDieTempC10.load(std::memory_order_relaxed) / 10.0f;
    doc["chipInfo"]    = ESP.getChipModel();
    // BUILD_SLOC / BUILD_SKB are computed by tools/embed_html.py across the
    // hand-written .ino + .html + tools sources at build time, so the count
    // stays current without anyone having to remember to update it.
    doc["sourceLines"] = BUILD_SLOC;
    doc["sourceKB"]    = BUILD_SKB;

    // ── WiFi ─────────────────────────────────────────────────────────────────
    doc["wifiSSID"]   = WiFi.SSID();
    doc["wifiRSSI"]   = (int)WiFi.RSSI();
    doc["ntfyTopic"]  = ntfyTopic;
    // The login/OTA password is NEVER serialized here. The dashboard no longer
    // needs it (the user types it at login, and again to flash); the default is
    // shown on the device's WiFi Info screen. Whether a custom one is set is the
    // only thing exposed, so the UI can label its prompts.
    doc["hasCustomPw"] = wifiHasCustomPassword() ? 1 : 0;

    // ── Vibration signatures ─────────────────────────────────────────────────
    // Include the saved signature filename list so the web client shows them
    // immediately on connect without needing a separate vibPanel command.
    {
        JsonArray sigArr = doc.createNestedArray("vibsigs");
        File sigDir = LittleFS.open("/vibsig");
        if (sigDir && sigDir.isDirectory()) {
            for (File e = sigDir.openNextFile(); e; e = sigDir.openNextFile()) {
                if (!e.isDirectory()) {
                    const char* fn = e.name();
                    const char* slash = strrchr(fn, '/');
                    // Copy into a String: ArduinoJson stores const char* BY
                    // REFERENCE, and fn points into the File impl that e.close()
                    // frees below — serializeJson() would read freed memory.
                    sigArr.add(String(slash ? slash + 1 : fn));
                }
                e.close();
            }
        }
        if (sigDir) sigDir.close();
    }

    serializeJson(doc, out);
}

// Fast telemetry — ~30 Hz live sensor values + WiFi/BT signal levels
static void buildFastTelemJson(String& out) {
    uint32_t st  = sensorState.load(std::memory_order_acquire);
    uint32_t hst = sensorHealth.load(std::memory_order_acquire);
    uint32_t amb = sensorAmbient.load(std::memory_order_acquire);
    bool    valid   = (st >> 31) & 1;
    int     dist_mm = st & 0x7FFFFFFF;                              // raw EMA — matches TFT DISTANCE display
    float   avgDist = sensorAvgDist.load(std::memory_order_acquire) / 10.0f; // 5-sample avg — matches TFT AVG FOV
    uint8_t status  = (hst >> 24) & 0xFF;
    float   signal  = (hst & 0xFFFFFF) / 1000.0f;
    float   ambient = (amb & 0xFFFFFF) / 1000.0f;

    float mult = (currentobj == 1) ? mul_5x : (currentobj == 2) ? mul_10x : mul_20x;
    float fov  = valid ? (mult * fovAt(avgDist)) : 0.0f;

    // Short keys save ~30 bytes/frame at 30 Hz.  HTML applyFullState() unpacks
    // them back to long names at the top of the function.
    StaticJsonDocument<448> doc;
    doc["d"]  = valid ? roundf(dist_mm * 10.0f) / 10.0f : 0;   // dist
    doc["f"]  = roundf(fov * 1000.0f) / 1000.0f;                // fov
    doc["e"]  = (int)sensorErrInt.load(std::memory_order_acquire); // fovErr (2σ × 100, centimm)
    doc["sg"] = roundf(signal * 10.0f) / 10.0f;                 // signal
    doc["a"]  = roundf(ambient * 100.0f) / 100.0f;              // sensorAmbient
    doc["ss"] = status;                                          // sensorStatus
    doc["t"]  = shutterActive ? 1 : 0;                          // trigger
    doc["wr"] = (int)WiFi.RSSI();                                // wifiRSSI
    // V12: vibration monitor — dominant Hz, vertical + horizontal RMS mg,
    // suggested wait ms, state.
    doc["vd"]  = roundf(vibDominant.load() / 10.0f * 10.0f) / 10.0f;
    doc["vr"]  = roundf((float)vibBandRms.load() / 100.0f * 10.0f) / 10.0f;
    doc["vh"]  = roundf((float)vibHorizRms.load() / 100.0f * 10.0f) / 10.0f;
    doc["vw"]  = (int)vibSuggestedWait.load();
    doc["vst"] = (int)vibState.load();
    doc["vcw"] = vibCurrentWaitMs;                               // controller WAIT setting
    doc["vsh"] = (int)vibShutterDenom;                           // analyzer/PNG-card shutter 1/N s

    serializeJson(doc, out);
}

// V12: vibration spectrum frame — 256 magnitude bins (deci-mg) per channel,
// streamed as a binary WebSocket frame at ~5 Hz only while a web vibration
// panel is open. Wire layout (all little-endian, matches ESP32-S3 + every
// browser host arch we care about):
//
//   byte  0-1  : magic 'V','S'                       (0x56, 0x53)
//   byte  2    : version                             (= 1)
//   byte  3    : channel count                       (= 2: vertical, horizontal)
//   byte  4-5  : bins per channel (uint16)           (= VIB_FFT_BINS = 256)
//   byte  6-7  : resonance-lock Hz x10 (uint16)      (was JSON "vrh")
//   byte  8-11 : reserved (= 0)                      (header padding to 12)
//   byte 12 .. 12 + bins*2 - 1                       : vertical channel uint16[]
//   byte 12 + bins*2 .. 12 + bins*4 - 1              : horizontal channel uint16[]
//
// vd (dominant Hz) and vh (horizontal RMS) used to ride along here but are
// also sent in buildFastTelemJson at 30 Hz, so they were duplicated; the
// JS side already keys off the fast-telem copy.
//
// Heap savings: the old JSON form built a ~3.4 KB String, then textAll()
// strdup'd it into the per-client AWS TX queue — ~6.8 KB transient heap
// per push at 5 Hz. The binary frame is 12 + 256*2*2 = 1036 bytes, and the
// stack buffer below means only the AWS-side ~1 KB queue copy hits the heap.
static void pushVibSpectrumBinary() {
    static constexpr size_t HEADER = 12;
    static constexpr size_t FRAME  = HEADER + (size_t)VIB_FFT_BINS * 2 * 2;
    uint8_t buf[FRAME];

    // Snapshot the spectrum ping-pong — lock-free SPSC: read seq, copy, re-check.
    uint16_t spec[VIB_FFT_BINS], specH[VIB_FFT_BINS]; uint32_t seq = 0;
    for (int t = 0; t < 3; t++) {
        seq = vibSpecSeq.load();
        memcpy(spec,  vibSpec[seq & 1],  sizeof(spec));
        memcpy(specH, vibSpecH[seq & 1], sizeof(specH));
        if (vibSpecSeq.load() == seq) break;
    }

    buf[0] = 'V'; buf[1] = 'S';
    buf[2] = 1;          // version
    buf[3] = 2;          // channel count
    uint16_t bins = (uint16_t)VIB_FFT_BINS;
    memcpy(buf + 4, &bins, 2);
    uint16_t vrhFix = (uint16_t)lroundf(vibResonanceHz * 10.0f);
    memcpy(buf + 6, &vrhFix, 2);
    buf[8] = buf[9] = buf[10] = buf[11] = 0;          // reserved
    memcpy(buf + HEADER,                     spec,  VIB_FFT_BINS * 2);
    memcpy(buf + HEADER + VIB_FFT_BINS * 2,  specH, VIB_FFT_BINS * 2);

    wsServer.binaryAll(buf, FRAME);
}

// V12: push the saved-signature filename list to all web clients.
void wifiPushVibSigList() {
    if (wsServer.count() == 0) return;
    String out = "{\"vibsigs\":[";
    out.reserve(360);
    File dir = LittleFS.open("/vibsig");
    bool first = true;
    if (dir && dir.isDirectory()) {
        for (File e = dir.openNextFile(); e; e = dir.openNextFile()) {
            if (!e.isDirectory()) {
                const char* fn = e.name();
                const char* slash = strrchr(fn, '/');
                const char* base = slash ? slash + 1 : fn;
                if (!first) out += ',';
                // Defensive JSON escape — vibRename sanitises names to
                // [A-Za-z0-9_-] today, but escape " \ and skip control chars
                // so a future loosening can't break the WS payload.
                out += '"';
                for (const char* p = base; *p; ++p) {
                    unsigned char c = (unsigned char)*p;
                    if (c < 0x20) continue;
                    if (c == '"' || c == '\\') out += '\\';
                    out += (char)c;
                }
                out += '"';
                first = false;
            }
            e.close();
        }
    }
    if (dir) dir.close();
    out += "]}";
    wsServer.textAll(out);
}

// Settings-only push — used by the state-change detector in wifiLoop().
// Compact (≤ 400 bytes) so wsServer.textAll() doesn't stall Core 1 the way a
// full-state frame (2 KB, DynamicJsonDocument 3072) does.
void wifiPushSettings() {
    if (!wifiConnected || wsServer.count() == 0) return;
    String out; buildSettingsJson(out);
    wsServer.textAll(out);
}

static void buildSettingsJson(String& out) {
    StaticJsonDocument<512> doc;
    doc["obj"]           = currentobj;
    doc["stepIndex"]     = stackStepIndex;
    doc["imgs"]          = stackTotalImgs;
    doc["secStep"]       = stackTimePerStep;
    // V12.3: measured stack time — pushed on change (incl. when a new measure
    // does NOT auto-apply), so the dashboard's Measured line tracks the device.
    doc["measStep"]      = roundf(measuredPerStep * 10.0f) / 10.0f;
    doc["measSh"]        = (int)measuredPulses;
    doc["stackActive"]   = mjkzzStackActive ? 1 : 0;   // V12.3: MJKZZ stack running?
    doc["brightness"]    = currentBrightness;
    doc["ledEnabled"]    = ledEnabled ? 1 : 0;
    doc["ledDuty"]       = currentLedDuty;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode ? 1 : 0;
    doc["dimMs"]         = (uint32_t)dimTimeoutMs;
    doc["sleepMs"]       = (uint32_t)sleepTimeoutMs;
    doc["theme"]         = currentThemeIndex;
    doc["tint"]          = themeIntensity;
    serializeJson(doc, out);
}

// Slow telemetry — 5 s memory stats
static void buildSlowTelemJson(String& out) {
    StaticJsonDocument<384> doc;
    doc["freeHeap"]      = ESP.getFreeHeap()    / 1024;
    doc["lowestFree"]    = ESP.getMinFreeHeap() / 1024;
    doc["psramFree"]     = ESP.getFreePsram()   / 1024;
    doc["psramTotal"]    = ESP.getPsramSize()   / 1024;
    doc["wifiSSID"]      = WiFi.SSID();
    // otaPassword removed from slow telem — it now ships only in the one-shot
    // full-state push (buildFullStateJson, sent on authenticated WS connect).
    doc["obj"]           = currentobj;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode   ? 1 : 0;
    // Die temp was only in the one-shot full-state frame, so the memory
    // screen showed a value frozen at connect time. Riding the 5 s slow
    // frame keeps it live; the JS dieC handler already accepts any frame.
    doc["dieC"]          = (int32_t)gDieTempC10.load(std::memory_order_relaxed) / 10.0f;

    serializeJson(doc, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// ACCESSORS  —  read-only view of wifi state for drawWifiIndicator() in the
//   main tab.  Because the main tab is concatenated BEFORE this tab, its code
//   sees these functions only via the forward declarations it contains.
//   (Arduino builds all .ino files into one TU in alphabetical order, with the
//   sketch-name file first.)
// ─────────────────────────────────────────────────────────────────────────────
bool        wifiIsConnected()    { return wifiConnected; }
bool        wifiIsPortal()       { return wifiServerMode == WMODE_PORTAL; }
const char* wifiGetPortalCode()  { return portalCode; }
const char* wifiGetOtaPassword() { return otaPassword; }   // the auto default (TFT display only)
bool wifiHasCustomPassword()     { return devPassword[0] != 0; }
int    wifiGetRSSI()     { return wifiConnected ? (int)WiFi.RSSI() : 0; }
String wifiGetIP()       { if (wifiServerMode == WMODE_PORTAL) return AP_IP.toString();
                           return wifiConnected ? WiFi.localIP().toString() : ""; }
String wifiGetSSID()     { if (wifiServerMode == WMODE_PORTAL) return String(AP_SSID);
                           return wifiConnected ? WiFi.SSID() : ""; }

// Called from main.ino when a stack sequence finishes (MIN_ACTIVE_DURATION of
// shutter activity followed by SILENCE_DURATION quiet).  Pushes a one-shot WS
// event so connected dashboards fire a browser Notification + the JS-side
// 3-beep AudioContext cue, and optionally fires an ntfy.sh push.
static void ntfyTask(void* param) {
    NtfyMsg* msg = static_cast<NtfyMsg*>(param);
    WiFiClient client;
    HTTPClient http;
    http.begin(client, "http://ntfy.sh/" + msg->topic);
    http.setTimeout(4000);
    http.addHeader("X-Priority", "high");
    http.addHeader("X-Title", "AutoFOV");
    int code = http.POST(msg->body);
    Serial.printf("[NTFY] -> %d\n", code);
    http.end();
    delete msg;
    vTaskDelete(nullptr);
}

void wifiNotifyStackComplete() {
    float avgDist = sensorAvgDist.load(std::memory_order_acquire) / 10.0f;
    float mult    = (currentobj == 1) ? mul_5x : (currentobj == 2) ? mul_10x : mul_20x;
    float fov     = mult * fovAt(avgDist);
    unsigned long durMs = lastPulseTime - firstPulseTime;

    if (wsServer.count() > 0) {
        StaticJsonDocument<512> doc;
        doc["stackDone"] = 1;
        doc["fov"]    = roundf(fov * 1000.0f) / 1000.0f;
        doc["err"]    = (int)sensorErrInt.load(std::memory_order_acquire);
        doc["dist"]   = roundf(avgDist * 10.0f) / 10.0f;
        doc["obj"]    = currentobj;
        doc["stepMm"] = roundf(stackStepSize * 1000.0f) / 1000.0f;
        doc["imgs"]   = stackTotalImgs;
        doc["durS"]   = (int)(durMs / 1000UL);
        doc["vst"]    = (int)vibState.load();
        // Per-stack V/H broadband blur (stage µm, 2 dp) and the shutter T = 1/N
        // s it was computed for. Matches the web analyzer's broadband row;
        // the PNG card uses the FOV-derived magnification to convert to
        // on-image µm and camera pixels.
        uint32_t vva = vibStackBlurVAvg.load();
        if (vva > 0) {
            doc["vva"] = roundf(vva                       / 100.0f * 100.0f) / 100.0f;
            doc["vvn"] = roundf(vibStackBlurVMin.load()   / 100.0f * 100.0f) / 100.0f;
            doc["vvx"] = roundf(vibStackBlurVMax.load()   / 100.0f * 100.0f) / 100.0f;
            doc["vha"] = roundf(vibStackBlurHAvg.load()   / 100.0f * 100.0f) / 100.0f;
            doc["vhn"] = roundf(vibStackBlurHMin.load()   / 100.0f * 100.0f) / 100.0f;
            doc["vhx"] = roundf(vibStackBlurHMax.load()   / 100.0f * 100.0f) / 100.0f;
            doc["shut"] = (int)vibShutterDenom;
        }
        String out; serializeJson(doc, out);
        wsServer.textAll(out);
    }
    if (ntfyTopic.length() > 0) {
        int errCentimm = (int)sensorErrInt.load(std::memory_order_acquire);
        char buf[64];
        snprintf(buf, sizeof(buf), "Stack finished \xe2\x80\x94 FOV %.2f(%d) mm", fov, errCentimm);
        NtfyMsg* msg = new NtfyMsg{ ntfyTopic, String(buf) };
        xTaskCreate(ntfyTask, "ntfy", 4096, msg, 1, nullptr);
    }
}

// Pings connected dashboards to play the alert beep.
void wifiNotifyTestAlert() {
    if (wsServer.count() == 0) return;
    wsServer.textAll("{\"beep\":1}");
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiForgetAndRestart()  —  called from handleWifiInfoTouch() in the main tab.
//   Clears saved credentials and reboots into captive-portal mode.
// ─────────────────────────────────────────────────────────────────────────────
void wifiForgetAndRestart() {
    wifiPrefs.begin("wifi", false);
    wifiPrefs.clear();
    wifiPrefs.end();
    Serial.println("[WiFi] Credentials cleared — restarting into setup portal");
    delay(300);
    ESP.restart();
}
