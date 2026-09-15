#include <SPI.h>
#include "RF24.h"

#ifdef printf_P
  #undef printf_P
#endif

#include <ezButton.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define CE_PIN     4
#define CSN_PIN    15
#define BUTTON_PIN 0

RF24 radio(CE_PIN, CSN_PIN);
ezButton button(BUTTON_PIN);

// ---------- Scan config ----------
#define NUM_CHANNELS     126
#define SAMPLES_PER_CH   40      // per scan
#define RESCAN_INTERVAL  15000   // ms between automatic re-scans
#define BUSY_THRESHOLD   8       // hits/40 above which a channel is "busy"

uint16_t spectrum[NUM_CHANNELS];

// ---------- Channel selection ----------
uint8_t  activeChannel   = 76;         // where our radio currently sits
bool     autoMode        = true;       // auto-pick quietest channel?
bool     manualOverride  = false;      // user pinned a channel?
uint8_t  manualChannel   = 76;

// Candidate channels to choose from. Add/remove to taste.
// Default: BLE advertising (37,38,39) + Wi-Fi 1,6,11 — the classic "safe" set.
const uint8_t candidateChannels[] = { 2, 26, 80, 12, 37, 62 };
const uint8_t NUM_CANDIDATES = sizeof(candidateChannels) / sizeof(candidateChannels[0]);

uint32_t lastScan = 0;

// ---------- Wi-Fi AP ----------
const char* ap_ssid     = "NodeMCU_Channel";
const char* ap_password = "12345678";

ESP8266WebServer server(80);

// ---------- Web UI ----------
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Channel Manager</title>
<style>
  body{font-family:Arial;background:#0d0d0d;color:#eee;text-align:center;
       margin:0;padding:24px}
  h1{margin:6px 0 20px}
  .card{background:#181818;border:1px solid #333;border-radius:10px;
        padding:16px;margin:10px auto;max-width:420px}
  .val{font-size:22px;font-weight:bold;color:#0f8}
  .row{display:flex;justify-content:space-between;padding:6px 0;
       border-bottom:1px solid #222}
  .row:last-child{border-bottom:none}
  .btn{display:inline-block;padding:12px 22px;font-size:15px;margin:8px 4px;
       border:none;border-radius:8px;cursor:pointer;color:#fff}
  .auto{background:#2196F3}.man{background:#f39c12}
  input{padding:10px;font-size:16px;width:90px;text-align:center;
        background:#000;color:#0f8;border:1px solid #333;border-radius:6px}
  .busy{color:#f44}.ok{color:#0f8}
</style>
</head>
<body>
<h1>2.4 GHz Channel Manager</h1>

<div class="card">
  <div class="row"><span>Mode</span><span class="val" id="mode">–</span></div>
  <div class="row"><span>Active channel</span><span class="val" id="ch">–</span></div>
  <div class="row"><span>Frequency</span><span class="val" id="mhz">–</span></div>
  <div class="row"><span>Busy score</span><span class="val" id="score">–</span></div>
</div>

<div class="card">
  <button class="btn auto" onclick="setMode('auto')">Auto — pick quietest</button>
  <button class="btn man" onclick="setMode('manual')">Manual — pin below</button>
  <form onsubmit="pinChannel(event)" style="margin-top:10px">
    <input type="number" id="pin" min="0" max="125" value="76">
    <button class="btn man" type="submit">Pin channel</button>
  </form>
</div>

<div class="card">
  <b>Candidate channel scores</b>
  <div id="cands" style="margin-top:8px"></div>
</div>

<script>
async function refresh() {
  try {
    const r = await fetch('/status');
    const s = await r.json();
    document.getElementById('mode').textContent  = s.mode;
    document.getElementById('ch').textContent    = s.ch;
    document.getElementById('mhz').textContent   = s.mhz + ' MHz';
    document.getElementById('score').textContent = s.score + ' / 40';
    document.getElementById('pin').value         = s.ch;

    let html = '';
    for (const c of s.candidates) {
      const cls = c.score > 8 ? 'busy' : 'ok';
      html += `<div class="row"><span>ch ${c.ch} (${c.mhz} MHz)</span>
               <span class="${cls}">${c.score}</span></div>`;
    }
    document.getElementById('cands').innerHTML = html;
  } catch(e) {}
  setTimeout(refresh, 1000);
}
refresh();

async function setMode(m) {
  await fetch('/mode?set=' + m);
}
async function pinChannel(e) {
  e.preventDefault();
  const v = document.getElementById('pin').value;
  await fetch('/pin?ch=' + v);
}
</script>
</body>
</html>
)=====";

void handleRoot() { server.send_P(200, "text/html", MAIN_page); }

void handleStatus() {
  uint16_t score = spectrum[activeChannel];
  int mhz = 2400 + activeChannel;

  String j = "{";
  j += "\"mode\":\""; j += (manualOverride ? "manual" : (autoMode ? "auto" : "locked")); j += "\",";
  j += "\"ch\":";    j += activeChannel; j += ",";
  j += "\"mhz\":";   j += mhz; j += ",";
  j += "\"score\":"; j += score; j += ",";
  j += "\"candidates\":[";
  for (uint8_t i = 0; i < NUM_CANDIDATES; i++) {
    uint8_t c = candidateChannels[i];
    j += "{\"ch\":"; j += c;
    j += ",\"mhz\":"; j += (2400 + c);
    j += ",\"score\":"; j += spectrum[c]; j += "}";
    if (i < NUM_CANDIDATES - 1) j += ",";
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleMode() {
  String m = server.arg("set");
  if (m == "auto")   { autoMode = true;  manualOverride = false; }
  if (m == "manual") { manualOverride = true; }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handlePin() {
  int c = server.arg("ch").toInt();
  if (c >= 0 && c <= 125) {
    manualChannel = (uint8_t)c;
    manualOverride = true;
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

void handleNotFound() { server.send(404, "text/plain", "404"); }

// ---------- Scan the full band once ----------
void scanBand() {
  for (int ch = 0; ch < NUM_CHANNELS; ch++) {
    radio.setChannel(ch);
    radio.startListening();
    uint16_t hits = 0;
    for (int s = 0; s < SAMPLES_PER_CH; s++) {
      if (radio.testRPD()) hits++;
      delayMicroseconds(20);
    }
    spectrum[ch] = hits;
  }
}

// ---------- Pick the quietest candidate ----------
uint8_t pickQuietest() {
  uint8_t best = candidateChannels[0];
  uint16_t bestScore = 0xFFFF;
  for (uint8_t i = 0; i < NUM_CANDIDATES; i++) {
    uint8_t c = candidateChannels[i];
    if (spectrum[c] < bestScore) {
      bestScore = spectrum[c];
      best = c;
    }
  }
  return best;
}

// ---------- Apply current selection to the radio ----------
void applyChannel() {
  uint8_t target = manualOverride ? manualChannel
                                   : (autoMode ? pickQuietest() : activeChannel);
  if (target != activeChannel) {
    activeChannel = target;
    Serial.print("Locked onto channel ");
    Serial.print(activeChannel);
    Serial.print(" (");
    Serial.print(2400 + activeChannel);
    Serial.println(" MHz)");
  }
  radio.setChannel(activeChannel);
  radio.startListening();
}

void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting channel manager...");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  button.setDebounceTime(50);

  for (int i = 0; i < NUM_CHANNELS; i++) spectrum[i] = 0;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP SSID: "); Serial.println(ap_ssid);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());

  server.on("/",       handleRoot);
  server.on("/status", handleStatus);
  server.on("/mode",   handleMode);
  server.on("/pin",    handlePin);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

  if (!radio.begin()) {
    Serial.println("RF24 failed!");
    while (1) {}
  }
  radio.setAutoAck(false);
  radio.setPALevel(RF24_PA_MIN);
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(activeChannel);
  radio.startListening();

  Serial.println("Initial band scan...");
  scanBand();
  applyChannel();
  lastScan = millis();
}

void loop() {
  server.handleClient();
  button.loop();

  if (millis() - lastScan > RESCAN_INTERVAL) {
    lastScan = millis();
    scanBand();
    applyChannel();
  }
}