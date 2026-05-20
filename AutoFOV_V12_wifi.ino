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
static const char*     OTA_PASSWORD        = "autofov-ota";  // change this

// ─────────────────────────────────────────────────────────────────────────────
// WIFI STATE
// ─────────────────────────────────────────────────────────────────────────────
enum WifiServerMode { WMODE_NONE, WMODE_PORTAL, WMODE_STA };
static WifiServerMode  wifiServerMode      = WMODE_NONE;
static char            portalCode[9]       = {0};  // random 8-char WPA2 password shown on TFT during setup
static bool            wifiConnected       = false;
static uint32_t        lastReconnectMs     = 0;
static uint32_t        lastFastTelemMs     = 0;
static uint32_t        lastSlowTelemMs     = 0;
static uint32_t        lastVibPushMs       = 0;       // V12: vibration spectrum stream timer
static bool            vibWebPanelOpen     = false;   // V12: spectrum stream gated on this
static volatile bool   otaInProgress       = false;   // suppresses telemetry during OTA flash

// ─────────────────────────────────────────────────────────────────────────────
// SERVER OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
static AsyncWebServer  httpServer(80);
static AsyncWebSocket  wsServer("/ws");
static DNSServer       dnsServer;

// Separate Preferences object for WiFi credentials — namespace "wifi".
// This is distinct from the main sketch's "calib" and "display" namespaces.
static Preferences     wifiPrefs;

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
  <button type=submit>SAVE &amp; CONNECT</button>
</form>
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
static void generatePortalCode() {
    // 8 chars: WPA2-PSK requires an 8-character minimum — softAP() rejects
    // anything shorter and the AP never starts.
    // Omit visually ambiguous characters (0/O, 1/I/L).
    static const char CHARS[] = "ABCDEFGHJKLMNPQRSTUVWXYZ23456789";
    for (int i = 0; i < 8; i++)
        portalCode[i] = CHARS[esp_random() % (sizeof(CHARS) - 1)];
    portalCode[8] = '\0';
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
        buildFullStateJson(out);
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

    // Mount LittleFS — HTML served from /index.html
    if (!LittleFS.begin(true)) {
        Serial.println("[WiFi] LittleFS mount failed — /index.html will not be served");
    }

    // GPIO0 (BOOT button) held LOW at power-on → force captive portal
    pinMode(0, INPUT_PULLUP);
    delay(50);
    bool forcePortal = (digitalRead(0) == LOW);

    // Load saved credentials
    wifiPrefs.begin("wifi", true);   // read-only
    String ssid     = wifiPrefs.getString("ssid", "");
    String pass     = wifiPrefs.getString("pass", "");
    String staticIP = wifiPrefs.getString("ip",   "");
    String gateway  = wifiPrefs.getString("gw",   "");
    wifiPrefs.end();

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

    // Captive portal: keep the DNS server ticking
    if (wifiServerMode == WMODE_PORTAL) {
        dnsServer.processNextRequest();

        return;   // no telemetry in portal mode
    }

    if (wifiServerMode != WMODE_STA) return;

    // ── Reconnect logic ──────────────────────────────────────────────────────
    if (!wifiConnected) {
        if (millis() - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
            lastReconnectMs = millis();
            if (WiFi.status() == WL_CONNECTED) {
                wifiConnected = true;
                Serial.printf("[WiFi] Reconnected: %s\n",
                              WiFi.localIP().toString().c_str());
                if (currentMode == MAIN) drawWifiIndicator();   // patched3: update header
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

    // V12: vibration spectrum stream — ~5 Hz, only while a web panel is open,
    // and dropped (not queued) when any client's TX buffer is full. Pushed as
    // a binary WebSocket frame (raw little-endian uint16) — the old JSON
    // serialisation cost ~3.4 KB of String heap per push and another ~3.4 KB
    // in the per-client AWS TX queue. Binary is ~1 KB, recurring at 5 Hz, so
    // this saves ~4-5 KB of transient heap every 200 ms.
    if (vibWebPanelOpen && (now - lastVibPushMs >= 200)) {
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

            ssid.trim(); pass.trim(); ip.trim(); gw.trim();
            if (ssid.isEmpty()) { req->send(400, "text/plain", "SSID required"); return; }

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
            wifiPrefs.end();

            req->send_P(200, "text/html", PORTAL_SAVED_HTML);
            delay(1500);
            ESP.restart();
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

    WiFi.begin(args->ssid.c_str(), args->pass.c_str());

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < CONNECT_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("[WiFi] Connected!  IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    } else {
        wifiConnected = false;
        lastReconnectMs = millis();
        Serial.printf("[WiFi] Connect failed (status %d) — will retry every %u s\n",
                      WiFi.status(), RECONNECT_INTERVAL_MS / 1000);
    }

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
    WiFi.setAutoReconnect(true);

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

// ─────────────────────────────────────────────────────────────────────────────
// FULL HTTP + WEBSOCKET SERVER  (STA mode)
// ─────────────────────────────────────────────────────────────────────────────
static void startFullServer() {
    // ── CORS / preflight ────────────────────────────────────────────────────
    // Without these headers, opening the HTML directly from a local file://
    // URL (or any cross-origin host) is blocked by the browser's CORS policy.
    // DefaultHeaders adds the headers to every response served by httpServer.
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin",  "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    wsServer.onEvent(onWsEvent);
    httpServer.addHandler(&wsServer);

    // Serve HTML from LittleFS /index.html
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (LittleFS.exists("/index.html")) {
            req->send(LittleFS, "/index.html", "text/html");
        } else {
            req->send(200, "text/plain",
                "No HTML found. Upload data/index.html via the LittleFS uploader.");
        }
    });

    // GET /state — full JSON snapshot (used by HTML on initial WiFi connect)
    httpServer.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
        String out; buildFullStateJson(out);
        req->send(200, "application/json", out);
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
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            if (len > 512) {
                req->send(400, "text/plain", "Payload too large");
                return;
            }
            char buf[513];
            memcpy(buf, data, len);
            buf[len] = '\0';
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
            }
            req->send(200, "application/json", "{\"ok\":1}");
        }
    );

    // POST /forget-wifi — clear credentials and return to portal on next restart.
    // Safe to run on Core 0: only NVS write (Preferences is internally serialised)
    // and ESP.restart() (core-agnostic). The delay() blocks the async task but
    // we're about to reboot, so request servicing is irrelevant.
    httpServer.on("/forget-wifi", HTTP_POST, [](AsyncWebServerRequest* req) {
        wifiPrefs.begin("wifi", false);
        wifiPrefs.clear();
        wifiPrefs.end();
        req->send(200, "text/plain", "WiFi credentials cleared. Restarting into setup portal…");
        delay(500);
        ESP.restart();
    });

    // Serve individual signature .bin files — the web dashboard fetches these
    // to overlay a saved spectrum in the panel.  URL: /vibsig/<name>.bin
    // Regex routes require -DASYNCWEBSERVER_REGEX, which the Arduino IDE does
    // not pass through.  Use a wildcard prefix route and re-join a sanitized
    // basename onto "/vibsig/" so a request like /vibsig/..%2fwifi can never
    // wander outside the directory.
    httpServer.on("/vibsig/*", HTTP_GET, [](AsyncWebServerRequest* req) {
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

    // ── OTA firmware update ──────────────────────────────────────────────────
    // Accepts a multipart/form-data POST with the compiled .bin as the "firmware"
    // field.  Runs entirely on the AsyncTCP task (Core 0); Update.h is safe from
    // any core.  Telemetry is suppressed via otaInProgress so the WS queue stays
    // clear during the ~3 s flash window.  Device reboots automatically on success.
    httpServer.on("/ota", HTTP_POST,
        // Completion handler — called after the last upload chunk.
        [](AsyncWebServerRequest* req) {
            bool ok = !Update.hasError();
            String msg = ok ? "OK" : String(Update.errorString());
            AsyncWebServerResponse* resp =
                req->beginResponse(ok ? 200 : 500, "text/plain", msg);
            resp->addHeader("Connection", "close");
            req->send(resp);
            if (ok) { delay(500); ESP.restart(); }
            else    { otaInProgress = false; }
        },
        // Upload chunk handler — streams the binary into the OTA partition.
        [](AsyncWebServerRequest* req, const String& filename,
           size_t index, uint8_t* data, size_t len, bool final) {
            if (!index) {
                // Reject if the X-OTA-Password header doesn't match
                if (!req->hasHeader("X-OTA-Password") ||
                    req->header("X-OTA-Password") != OTA_PASSWORD) {
                    req->send(401, "text/plain", "Unauthorized");
                    return;
                }
                otaInProgress = true;
                Serial.printf("[OTA] start: %s\n", filename.c_str());
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Serial.printf("[OTA] begin failed: %s\n", Update.errorString());
                }
            }
            if (Update.write(data, len) != len) {
                Serial.printf("[OTA] write error: %s\n", Update.errorString());
            }
            if (final) {
                if (Update.end(true)) {
                    Serial.printf("[OTA] success: %u bytes\n", index + len);
                } else {
                    Serial.printf("[OTA] end error: %s\n", Update.errorString());
                }
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
    Serial.println("[WiFi] HTTP server listening on port 80 (CORS: *, queued /cmd)");
}

// ─────────────────────────────────────────────────────────────────────────────
// COMMAND DISPATCHER
// Called from wifiLoop() on Core 1 — safe to call TFT, analogWrite, i2cMutex.
//
// Key reference (mirrors HTML sendCommand calls):
//   obj, stepIndex, imgs, brightness, ledEnabled, ledDuty,
//   sensorSleep, highRefl, dimMs, sleepMs, theme, tint,
//   calWidth, demarcDist, calPoints, calStart, calCapture, calUndoPoint,
//   resetAll, testAlert, ir, nav
// ─────────────────────────────────────────────────────────────────────────────
static void handleWifiCommand(const char* key, const char* val) {
    float fVal = atof(val);
    int   iVal = atoi(val);

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
        bool wantSleep = (iVal != 0);
        if (wantSleep && !sensorSleeping) {
            sensorSleeping = true;
            sensorState.store(0, std::memory_order_release);
            sensorHealth.store(0xFF000000UL, std::memory_order_release);
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                sensor.VL53L4CX_StopMeasurement();
                xSemaphoreGive(i2cMutex);
            }
        } else if (!wantSleep && sensorSleeping) {
            sensorEmaReset.store(true, std::memory_order_release);
            sensorSleeping = false;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                applyHighReflConfig();
                sensor.VL53L4CX_StartMeasurement();
                xSemaphoreGive(i2cMutex);
            }
        }
        if (currentMode == SENSOR_INFO) drawSensorInfoUI();

    // ── High-reflectivity mode ────────────────────────────────────────────────
    // applyHighReflConfig() requires measurement to be stopped first — mirror
    // the device-side path (handleSensorInfoTouch) exactly: stop, reconfigure,
    // restart, then request an EMA reset so the post-change samples are clean.
    } else if (strcmp(key, "highRefl") == 0) {
        highReflMode = (iVal != 0);
        if (!sensorSleeping) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(500))) {
                sensor.VL53L4CX_StopMeasurement();
                applyHighReflConfig();
                sensor.VL53L4CX_StartMeasurement();
                xSemaphoreGive(i2cMutex);
            }
            sensorEmaReset.store(true, std::memory_order_release);
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
        pointsCaptured = 0;

    // ── Calibration: capture a point ─────────────────────────────────────────
    //    iVal = pixel count measured in the photo at current distance
    //    Formula: fov_mm = (demarcationDist / pixels) * sensorWidthPixels
    } else if (strcmp(key, "calCapture") == 0) {
        if (iVal < 1) return;  // guard against zero/negative pixel counts
        uint32_t st   = sensorState.load(std::memory_order_acquire);
        bool     valid = (st >> 31) & 1;
        float    dist  = (float)(st & 0x7FFFFFFF);          // mm integer from EMA
        float    fov   = (demarcationDist / (float)iVal) * sensorWidthPixels;

        if (!valid || pointsCaptured >= 20) return;   // no valid range or buffer full

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
            finalizeCalibration();   // computes CTRLX/CTRLY, saves NVS, sets isCustomCalib
            // finalizeCalibration() calls drawSuccessScreen() — the physical display
            // will show the CAL_SUCCESS screen briefly (same as touch-triggered flow)
            String out; buildFullStateJson(out);
            wsServer.textAll(out);
        }

    // ── Calibration: undo last captured point ────────────────────────────────
    } else if (strcmp(key, "calUndoPoint") == 0) {
        if (pointsCaptured > 0) pointsCaptured--;

    // ── Reset to factory calibration ─────────────────────────────────────────
    //    NOTE: we manually restore defaults rather than calling finalizeCalibration()
    //    to avoid setting isCustom=1 and showing the TFT success screen.
    } else if (strcmp(key, "resetAll") == 0) {
        nPoints        = FACTORY_N;
        pointsCaptured = FACTORY_N;
        for (int i = 0; i < FACTORY_N; i++) {
            distPoints[i] = FACTORY_DIST[i];
            fovPoints[i]  = FACTORY_FOV[i];
        }
        CTRLX        = Config::DEFAULT_CTRL_X;
        CTRLY        = Config::DEFAULT_CTRL_Y;
        CALIB_ERROR  = Config::DEFAULT_CALIB_ERROR;
        CALIB_R2     = 0.994f;
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
        // Push updated calibration state to all HTML clients
        String out; buildFullStateJson(out);
        wsServer.textAll(out);

    // ── Test alert (beeps all connected WebSocket clients) ───────────────────
    } else if (strcmp(key, "testAlert") == 0) {
        wifiNotifyTestAlert();

    // ── IR remote (MJKZZ stacking controller) ────────────────────────────────
    //    val = button name; fires the matching NEC code over the IR LED.
    //    Runs on Core 1 via wifiLoop(), so the blocking sendNEC() is safe here.
    } else if (strcmp(key, "ir") == 0) {
        if      (strcmp(val, "start")   == 0) irLed.sendNEC(IR_MJKZZ_START);
        else if (strcmp(val, "stop")    == 0) irLed.sendNEC(IR_MJKZZ_STOP);
        else if (strcmp(val, "up")      == 0) irLed.sendNEC(IR_MJKZZ_UP);
        else if (strcmp(val, "down")    == 0) irLed.sendNEC(IR_MJKZZ_DOWN);
        else if (strcmp(val, "capture") == 0) irLed.sendNEC(IR_MJKZZ_CAPTURE);

    // ── Remote menu navigation (web → device) ────────────────────────────────
    //    val = web screen-id (e.g. "screen-settings"). Mirrors the web menu
    //    onto the TFT. Ignored mid-calibration and for non-syncable ids.
    //    A nav wakes the display like a physical touch; preSleepMode is set so
    //    a later wakeScreen() restores the synced screen too.
    } else if (strcmp(key, "nav") == 0) {
        if (inCalibFlow()) return;                   // don't interrupt a calibration
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

    } else if (strcmp(key, "vibCapture") == 0) {
        // val = source name. Arms a capture; vibCapturePoll() finishes it.
        if (val[0]) {
            strncpy(vibSigCaptureName, val, sizeof(vibSigCaptureName) - 1);
            vibSigCaptureName[sizeof(vibSigCaptureName) - 1] = '\0';
            for (int b = 0; b < VIB_FFT_BINS; b++) vibSigAccum[b] = 0;
            vibSigFrames    = 0;
            vibCapSeqSeen   = vibSpecSeq.load();
            vibSigCapturing = true;
        }

    } else if (strcmp(key, "vibDelSig") == 0) {
        if (val[0]) {
            char path[48];
            snprintf(path, sizeof(path), "/vibsig/%s", val);
            LittleFS.remove(path);
            wifiPushVibSigList();
        }

    } else if (strcmp(key, "vibRename") == 0) {
        // val = "oldfile.bin|NewName" — rewrites the file under the new name.
        const char* bar = strchr(val, '|');
        if (bar && bar[1]) {
            char oldbase[32];
            int  ol = bar - val;
            if (ol > 31) ol = 31;
            strncpy(oldbase, val, ol);
            oldbase[ol] = '\0';
            if (vibSigLoad(oldbase)) {
                strncpy(vibSigLoaded.name, bar + 1, sizeof(vibSigLoaded.name) - 1);
                vibSigLoaded.name[sizeof(vibSigLoaded.name) - 1] = '\0';
                char oldp[48], newp[48];
                snprintf(oldp, sizeof(oldp), "/vibsig/%s", oldbase);
                snprintf(newp, sizeof(newp), "/vibsig/%s.bin", bar + 1);
                File f = LittleFS.open(newp, "w");
                if (f) { f.write((const uint8_t*)&vibSigLoaded, sizeof(VibSignature)); f.close(); }
                if (strcmp(oldp, newp) != 0) LittleFS.remove(oldp);
                wifiPushVibSigList();
            }
        }

    // ── Latency ping ─────────────────────────────────────────────────────────
    //    The web dashboard sends {"ping":1} and times the round-trip to this
    //    {"pong":1} echo.  Routed through the normal command queue (same as
    //    every other command) so the measured value reflects real command
    //    responsiveness, not just raw network RTT.
    } else if (strcmp(key, "ping") == 0) {
        wsServer.textAll("{\"pong\":1}");
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
    float fov  = valid ? (mult * (avgDist * CTRLX + CTRLY)) : 0.0f;

    DynamicJsonDocument doc(3584);

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
    doc["calibError"]    = roundf(CALIB_ERROR * 1000.0f) / 1000.0f;
    doc["calibR2"]       = roundf(CALIB_R2 * 1000.0f) / 1000.0f;
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
    doc["buildDate"]   = __DATE__;
    doc["buildTime"]   = __TIME__;
    doc["sketchKB"]    = ESP.getSketchSize()      / 1024;
    doc["freeFlashKB"] = ESP.getFreeSketchSpace() / 1024;
    doc["chipInfo"]    = ESP.getChipModel();
    // sourceLines is managed by the HTML default (3866) — not sent here to save space

    // ── WiFi ─────────────────────────────────────────────────────────────────
    doc["wifiSSID"]   = WiFi.SSID();
    doc["wifiRSSI"]   = (int)WiFi.RSSI();

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
                    sigArr.add(slash ? slash + 1 : fn);
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
    float fov  = valid ? (mult * (avgDist * CTRLX + CTRLY)) : 0.0f;

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
    StaticJsonDocument<256> doc;
    doc["freeHeap"]      = ESP.getFreeHeap()    / 1024;
    doc["lowestFree"]    = ESP.getMinFreeHeap() / 1024;
    doc["psramFree"]     = ESP.getFreePsram()   / 1024;
    doc["psramTotal"]    = ESP.getPsramSize()   / 1024;
    doc["wifiSSID"]      = WiFi.SSID();
    doc["obj"]           = currentobj;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode   ? 1 : 0;

    serializeJson(doc, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// ACCESSORS  —  read-only view of wifi state for drawWifiIndicator() in the
//   main tab.  Because the main tab is concatenated BEFORE this tab, its code
//   sees these functions only via the forward declarations it contains.
//   (Arduino builds all .ino files into one TU in alphabetical order, with the
//   sketch-name file first.)
// ─────────────────────────────────────────────────────────────────────────────
bool        wifiIsConnected()   { return wifiConnected; }
bool        wifiIsPortal()      { return wifiServerMode == WMODE_PORTAL; }
const char* wifiGetPortalCode() { return portalCode; }
int    wifiGetRSSI()     { return wifiConnected ? (int)WiFi.RSSI() : 0; }
String wifiGetIP()       { if (wifiServerMode == WMODE_PORTAL) return AP_IP.toString();
                           return wifiConnected ? WiFi.localIP().toString() : ""; }
String wifiGetSSID()     { if (wifiServerMode == WMODE_PORTAL) return String(AP_SSID);
                           return wifiConnected ? WiFi.SSID() : ""; }

// Called from main.ino when a stack sequence finishes (the same moment the
// BLE HID key would have fired).  Pushes a one-shot WS event so the web UI
// can fire a browser Notification — BLE-free fallback for the stacker prompt.
// The JS side plays 3 scheduled AudioContext beeps from this single message.
void wifiNotifyStackComplete() {
    if (wsServer.count() == 0) return;
    wsServer.textAll("{\"stackDone\":1}");
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
