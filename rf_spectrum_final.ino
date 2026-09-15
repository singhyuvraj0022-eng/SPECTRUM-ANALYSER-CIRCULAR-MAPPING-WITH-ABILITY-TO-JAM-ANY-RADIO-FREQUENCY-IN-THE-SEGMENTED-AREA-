// ═══════════════════════════════════════════════════════════════════
//  2.4 GHz Passive Spectrum Monitor — FINAL
//  ESP8266 (NodeMCU) + nRF24L01   |   RX ONLY, never transmits
//
//  Features:
//   ● Full 126-channel spectrum sweep (linear + radial view)
//   ● WiFi SSID scanner (ESP8266's own radio)
//   ● BLE channel activity detection
//   ● Manual channel lock mode
//   ● Channel heatmap
//   ● Start / Stop / Reset controls
//
//  Wiring:
//   VCC  → 3.3V ONLY (never 5V!)
//   GND  → GND
//   CE   → D2 (GPIO4)
//   CSN  → D8 (GPIO15)
//   SCK  → D5 (GPIO14)  hardware SPI
//   MOSI → D7 (GPIO13)  hardware SPI
//   MISO → D6 (GPIO12)  hardware SPI
//
//  ⚠ Add 10–100µF cap across VCC/GND at the module for stability
//
//  Connect to AP "RF-Spectrum" → open http://192.168.4.1
// ═══════════════════════════════════════════════════════════════════

#include <SPI.h>
#include "RF24.h"

// Fix RF24 / ESP8266 WebServer macro clash
#ifdef printf_P
  #undef printf_P
#endif

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define CE_PIN   4    // D2 = GPIO4
#define CSN_PIN  15   // D8 = GPIO15

RF24 radio(CE_PIN, CSN_PIN);
ESP8266WebServer server(80);

// ── Scan parameters ──────────────────────────────────────────────
const uint8_t  CHANNELS   = 126;     // 0–125 → 2400–2525 MHz
const uint16_t BASE_MHZ   = 2400;
const uint8_t  SAMPLES    = 16;      // reads per channel per sweep

uint8_t  hits[CHANNELS];             // current sweep results
uint8_t  peak[CHANNELS];             // all-time peak per channel
uint8_t  history[CHANNELS];          // previous sweep (for decay)
unsigned long sweepMs     = 0;
unsigned long totalSweeps = 0;

// ── BLE channel map (40 channels → nRF24 channel numbers) ───────
const uint8_t BLE_COUNT = 40;
const uint8_t bleChannels[BLE_COUNT] = {
   2,  4,  6,  8, 10, 12, 14, 16, 18, 20,
  22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
  42, 44, 46, 48, 50, 52, 54, 56, 58, 60,
  62, 64, 66, 68, 70, 72, 74, 76, 78, 80
};

// ── Control state ────────────────────────────────────────────────
bool     scanEnabled  = true;
bool     manualLock   = false;        // true = pinned to one channel
uint8_t  lockedCh     = 0;
uint8_t  lockedHits   = 0;           // hit counter in manual mode

// ── AP credentials ───────────────────────────────────────────────
const char* AP_SSID = "RF-Spectrum";
const char* AP_PASS = "spectrum123";  // min 8 chars for WPA2

// ── WiFi scan cache ──────────────────────────────────────────────
String wifiCache = "[]";
unsigned long lastWifiScan = 0;
const unsigned long WIFI_INTERVAL = 10000;


// ══════════════════════════════════════════════════════════════════
//  RF sweep — full 126-channel energy detection
// ══════════════════════════════════════════════════════════════════
void sweep() {
  unsigned long t0 = millis();
  memcpy(history, hits, CHANNELS);    // save previous for decay

  for (uint8_t ch = 0; ch < CHANNELS; ch++) {
    radio.setChannel(ch);
    delayMicroseconds(350);           // PLL settle
    uint8_t c = 0;
    for (uint8_t s = 0; s < SAMPLES; s++) {
      radio.startListening();
      delayMicroseconds(100);
      if (radio.testRPD()) c++;       // RPD ≈ −64 dBm
      radio.stopListening();
    }
    hits[ch] = c;
    if (c > peak[ch]) peak[ch] = c;
  }
  sweepMs = millis() - t0;
  totalSweeps++;
  radio.startListening();
}

// ══════════════════════════════════════════════════════════════════
//  Manual channel monitor — sit on one channel, count hits
// ══════════════════════════════════════════════════════════════════
void monitorLocked() {
  radio.setChannel(lockedCh);
  delayMicroseconds(350);
  uint8_t c = 0;
  for (uint8_t s = 0; s < 32; s++) {  // more samples in lock mode
    radio.startListening();
    delayMicroseconds(100);
    if (radio.testRPD()) c++;
    radio.stopListening();
  }
  lockedHits = c;
  radio.startListening();
}

// ══════════════════════════════════════════════════════════════════
//  WiFi scan (ESP8266's own radio)
// ══════════════════════════════════════════════════════════════════
void refreshWifi() {
  if (millis() - lastWifiScan < WIFI_INTERVAL) return;
  lastWifiScan = millis();

  int n = WiFi.scanNetworks(false, true);
  String j = "[";
  for (int i = 0; i < n && i < 20; i++) {
    if (i) j += ',';
    String ssid = WiFi.SSID(i);
    ssid.replace("\\", "\\\\");
    ssid.replace("\"", "\\\"");
    j += "{\"s\":\"" + ssid + "\""
       + ",\"r\":" + String(WiFi.RSSI(i))
       + ",\"c\":" + String(WiFi.channel(i))
       + ",\"e\":" + String(WiFi.encryptionType(i) != ENC_TYPE_NONE ? 1 : 0)
       + ",\"b\":\"" + WiFi.BSSIDstr(i) + "\"}";
  }
  j += "]";
  wifiCache = j;
  WiFi.scanDelete();
}


// ══════════════════════════════════════════════════════════════════
//  Web UI (PROGMEM)
// ══════════════════════════════════════════════════════════════════
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RF Spectrum Monitor</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
 --bg:#04070b;--c1:#0a1018;--c2:#0e1520;--bd:#142030;
 --t1:#d4ece4;--t2:#78a898;--t3:#3e6858;
 --g:#18e888;--y:#f0b830;--r:#ff4050;
 --bl:#4888d0;--pr:#9868e0;
}
body{background:var(--bg);color:var(--t1);
 font:11.5px/1.45 'Menlo','Cascadia Code','Fira Code',monospace;
 padding:8px 10px;-webkit-font-smoothing:antialiased}

/* header */
.hd{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:4px}
.hd h1{font-size:14px;color:var(--g);font-weight:700}
.hd .tg{font-size:8px;padding:2px 6px;border-radius:8px;
 border:1px solid var(--g);color:var(--g)}
.hd .mt{font-size:9px;color:var(--t3);margin-left:auto}

/* controls bar */
.ctrl{display:flex;gap:6px;flex-wrap:wrap;align-items:center;
 margin:6px 0 8px;padding:8px;background:var(--c1);
 border:1px solid var(--bd);border-radius:6px}
.ctrl button{padding:6px 14px;border:none;border-radius:5px;
 cursor:pointer;font:11px monospace;font-weight:600;color:#fff}
.btn-go{background:#18a868}.btn-stop{background:#d03040}
.btn-rst{background:#5060a0}
.ctrl input[type=number]{width:56px;padding:5px;font:11px monospace;
 background:#0a0e14;color:var(--t1);border:1px solid var(--bd);
 border-radius:4px;text-align:center}
.ctrl .sep{width:1px;height:24px;background:var(--bd);margin:0 4px}
.ctrl label{font-size:10px;color:var(--t2)}
.lock-info{font-size:10px;color:var(--y);margin-left:4px}

/* stats */
.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(100px,1fr));
 gap:5px;margin-bottom:8px}
.st{background:var(--c1);border:1px solid var(--bd);border-radius:5px;
 padding:7px 9px}
.st .v{font-size:18px;font-weight:800}
.st .l{font-size:8px;color:var(--t3);margin-top:1px;text-transform:uppercase;
 letter-spacing:.7px}
.vg{color:var(--g)}.vy{color:var(--y)}.vr{color:var(--r)}.vb{color:var(--bl)}

/* grid */
.row{display:grid;gap:7px;grid-template-columns:1fr 1fr;margin-bottom:7px}
@media(max-width:620px){.row{grid-template-columns:1fr}}
.full{grid-column:1/-1}
.pnl{background:var(--c1);border:1px solid var(--bd);
 border-radius:7px;padding:9px;overflow:hidden}
.pnl h2{font-size:9px;color:var(--t3);font-weight:600;
 text-transform:uppercase;letter-spacing:1px;margin-bottom:5px}
canvas{display:block;width:100%}

#barC{height:175px}
#radWrap{display:flex;align-items:center;justify-content:center}
#radC{max-width:340px;aspect-ratio:1/1}

/* legend */
.leg{display:flex;gap:8px;flex-wrap:wrap;margin-top:5px}
.leg span{font-size:8px;display:flex;align-items:center;gap:3px;color:var(--t2)}
.leg i{width:6px;height:6px;border-radius:50%;display:inline-block}

/* wifi list */
.wl{max-height:300px;overflow-y:auto;
 scrollbar-width:thin;scrollbar-color:var(--bd) transparent}
.wi{display:grid;grid-template-columns:18px 1fr 38px 50px;
 align-items:center;gap:5px;padding:4px 0;border-bottom:1px solid var(--bd)}
.wi:last-child{border:none}
.wi .nm{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:10px}
.wi .ch{font-size:8px;color:var(--t3);text-align:center}
.wi .br{height:4px;border-radius:2px;background:#0a0e14;overflow:hidden}
.wi .bf{height:100%;border-radius:2px}
.hid{color:var(--t3);font-style:italic}

/* heatmap */
.hm{display:flex;flex-wrap:wrap;gap:1px}
.hm .cx{width:6px;height:6px;border-radius:1px}

/* ble panel */
.ble-grid{display:grid;grid-template-columns:repeat(10,1fr);gap:2px}
.ble-cell{text-align:center;font-size:8px;padding:3px 0;
 border-radius:3px;background:#0a0e14;color:var(--t3)}
.ble-cell.active{background:var(--pr);color:#fff;font-weight:700}
.ble-cell.adv{border:1px solid var(--pr)}

/* tooltip */
#tip{position:fixed;background:#101820;color:var(--t1);
 font-size:9px;padding:3px 7px;border-radius:3px;
 pointer-events:none;display:none;z-index:99;border:1px solid var(--bd)}
</style></head><body>

<div class="hd">
 <h1>&#x1f4e1; RF Spectrum Monitor</h1>
 <span class="tg">PASSIVE RX</span>
 <span class="mt" id="meta">initializing&hellip;</span>
</div>

<!-- ── CONTROLS ── -->
<div class="ctrl">
 <button class="btn-go" onclick="ctrlScan('on')">&#x25b6; Start</button>
 <button class="btn-stop" onclick="ctrlScan('off')">&#x25a0; Stop</button>
 <button class="btn-rst" onclick="ctrlReset()">&#x21bb; Reset Peaks</button>
 <div class="sep"></div>
 <label>Lock ch:</label>
 <input type="number" id="chIn" min="0" max="125" value="0">
 <button class="btn-go" onclick="ctrlLock()">Lock</button>
 <button class="btn-stop" onclick="ctrlUnlock()">Unlock</button>
 <span class="lock-info" id="lockInfo"></span>
</div>

<!-- ── STATS ── -->
<div class="stats">
 <div class="st"><div class="v vg" id="vPk">&mdash;</div><div class="l">Peak Ch</div></div>
 <div class="st"><div class="v vg" id="vFr">&mdash;</div><div class="l">Peak Freq</div></div>
 <div class="st"><div class="v vy" id="vHt">&mdash;</div><div class="l">Hits/Max</div></div>
 <div class="st"><div class="v vg" id="vNt">&mdash;</div><div class="l">WiFi Nets</div></div>
 <div class="st"><div class="v vg" id="vAc">&mdash;</div><div class="l">Active Chs</div></div>
 <div class="st"><div class="v vb" id="vSw">&mdash;</div><div class="l">Sweeps</div></div>
</div>

<div class="row">
 <!-- LINEAR SPECTRUM -->
 <div class="pnl full">
  <h2>Linear Spectrum &mdash; 2400–2525 MHz</h2>
  <canvas id="barC" height="175"></canvas>
  <div class="leg">
   <span><i style="background:var(--g)"></i>Low</span>
   <span><i style="background:var(--y)"></i>Med</span>
   <span><i style="background:var(--r)"></i>High</span>
   <span><i style="background:var(--bl)"></i>WiFi</span>
   <span><i style="background:var(--pr)"></i>BT/BLE</span>
  </div>
 </div>

 <!-- RADIAL SPECTRUM -->
 <div class="pnl">
  <h2>Radial Spectrum</h2>
  <div id="radWrap"><canvas id="radC"></canvas></div>
 </div>

 <!-- WIFI NETWORKS -->
 <div class="pnl">
  <h2>WiFi Networks <span id="wCnt" style="color:var(--g)"></span></h2>
  <div class="wl" id="wList"><p style="color:var(--t3)">Scanning&hellip;</p></div>
 </div>

 <!-- BLE CHANNELS -->
 <div class="pnl full">
  <h2>BLE Channel Map (40 channels &mdash; purple = activity detected)</h2>
  <div class="ble-grid" id="bleMap"></div>
  <div class="leg" style="margin-top:6px">
   <span><i style="background:var(--pr)"></i>Activity detected</span>
   <span><i style="border:1px solid var(--pr);width:5px;height:5px"></i>Advertising ch (37/38/39)</span>
  </div>
 </div>

 <!-- HEATMAP -->
 <div class="pnl full">
  <h2>Channel Heatmap</h2>
  <div class="hm" id="hmap"></div>
 </div>
</div>

<div id="tip"></div>

<script>
const N=126,BASE=2400,S=16;
const WCHS=[1,6,11,14];
const wF=c=>c===14?2484:2412+(c-1)*5;
const BLE=[2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,
 42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80];
const BLE_ADV=[2,26,80]; // advertising channels (37,38,39)

const barC=document.getElementById('barC'),bx=barC.getContext('2d');
const radC=document.getElementById('radC'),rx=radC.getContext('2d');
const tip=document.getElementById('tip');

let rf=null, wf=[], st={scan:true,lock:false,lockCh:0,lockHits:0};

function hc(v){return v>.6?'#ff4050':v>.25?'#f0b830':'#18e888'}
function hca(v,a){
 if(v>.6)return`rgba(255,64,80,${a})`;
 if(v>.25)return`rgba(240,184,48,${a})`;
 return`rgba(24,232,136,${a})`}

function fit(c){
 const r=devicePixelRatio||1;
 const w=c.clientWidth,h=c.clientHeight||c.clientWidth;
 if(c.width!==w*r||c.height!==h*r){c.width=w*r;c.height=h*r;
  c.getContext('2d').scale(r,r)}
 return{w,h}}

/* ═══ CONTROLS ═══ */
function ctrlScan(s){fetch('/ctrl?scan='+s)}
function ctrlReset(){fetch('/ctrl?reset=1')}
function ctrlLock(){
 const ch=document.getElementById('chIn').value;
 fetch('/ctrl?lock='+ch)}
function ctrlUnlock(){fetch('/ctrl?unlock=1')}

/* ═══ LINEAR BAR ═══ */
function drawBar(){
 if(!rf)return;
 const{w,h}=fit(barC);bx.clearRect(0,0,w,h);
 const bw=w/N, ht=rf.hits;

 /* BT zone */
 bx.fillStyle='rgba(152,104,224,.04)';
 bx.fillRect((2-0)*bw,0,(80-2+1)*bw,h);

 for(let i=0;i<N;i++){
  const v=ht[i]/rf.samples;
  const bh=Math.max(v*(h-16),v>0?2:0);
  const g=bx.createLinearGradient(0,h-bh,0,h);
  g.addColorStop(0,hc(v));g.addColorStop(1,hca(v,.2));
  bx.fillStyle=g;
  bx.fillRect(i*bw+.5,h-bh,Math.max(bw-1,1),bh);
  if(v>.45){bx.fillStyle=hca(v,.1);
   bx.fillRect(i*bw,h-bh-3,bw,bh+3)}
 }

 /* locked channel marker */
 if(st.lock){
  const px=st.lockCh*bw;
  bx.strokeStyle='#fff';bx.lineWidth=1.5;
  bx.setLineDash([3,3]);
  bx.beginPath();bx.moveTo(px,0);bx.lineTo(px,h);bx.stroke();
  bx.setLineDash([]);
  bx.fillStyle='#fff';bx.font='bold 9px monospace';
  bx.textBaseline='top';bx.fillText('\u25bc LOCK',px+3,2);
 }

 /* WiFi markers */
 bx.font='8px monospace';bx.textBaseline='top';
 for(let ch=1;ch<=14;ch++){
  const f=wF(ch),px=(f-BASE)*bw;
  bx.strokeStyle='rgba(72,136,208,.2)';
  bx.beginPath();bx.moveTo(px,0);bx.lineTo(px,h);bx.stroke();
  if([1,6,11,14].includes(ch)){
   bx.fillStyle='#4888d0';bx.fillText('W'+ch,px+2,2)}
 }

 bx.fillStyle='#3e6858';bx.font='8px monospace';bx.textBaseline='bottom';
 bx.textAlign='left';bx.fillText('2400',2,h-1);
 bx.textAlign='right';bx.fillText('2525',w-2,h-1);
 bx.textAlign='left';
}

/* tooltip */
barC.onmousemove=e=>{
 if(!rf)return;
 const rc=barC.getBoundingClientRect();
 const ch=Math.floor((e.clientX-rc.left)/rc.width*N);
 if(ch<0||ch>=N){tip.style.display='none';return}
 const isBLE=BLE.includes(ch);
 tip.style.display='block';
 tip.style.left=(e.clientX+10)+'px';
 tip.style.top=(e.clientY-22)+'px';
 tip.textContent=`ch${ch} \u00b7 ${BASE+ch}MHz \u00b7 ${rf.hits[ch]}/${rf.samples}`
  +(isBLE?' \u00b7 BLE':'');
};
barC.onmouseleave=()=>tip.style.display='none';

/* ═══ RADIAL ═══ */
function drawRadial(){
 if(!rf)return;
 const{w,h}=fit(radC);rx.clearRect(0,0,w,h);
 const cx=w/2,cy=h/2;
 const rI=Math.min(cx,cy)*.15,rO=Math.min(cx,cy)*.84;
 const ht=rf.hits, step=Math.PI*2/N;

 [.25,.5,.75,1].forEach(f=>{
  const r=rI+(rO-rI)*f;
  rx.beginPath();rx.arc(cx,cy,r,0,Math.PI*2);
  rx.strokeStyle='rgba(20,32,48,.7)';rx.lineWidth=.5;rx.stroke()});

 /* BT arc */
 const aS=(2/N)*Math.PI*2-Math.PI/2;
 const aE=(80/N)*Math.PI*2-Math.PI/2;
 rx.beginPath();rx.arc(cx,cy,rO+6,aS,aE);
 rx.strokeStyle='rgba(152,104,224,.28)';rx.lineWidth=3;rx.stroke();

 /* WiFi dots */
 WCHS.forEach(wc=>{
  const a=((wF(wc)-BASE)/N)*Math.PI*2-Math.PI/2;
  rx.beginPath();rx.arc(cx+Math.cos(a)*(rO+14),cy+Math.sin(a)*(rO+14),2.5,0,Math.PI*2);
  rx.fillStyle='rgba(72,136,208,.45)';rx.fill()});

 /* spokes */
 for(let i=0;i<N;i++){
  const v=ht[i]/rf.samples;
  const a=i*step-Math.PI/2;
  const rE=rI+(rO-rI)*Math.max(v,.015);
  const cs=Math.cos(a),sn=Math.sin(a);
  rx.beginPath();
  rx.moveTo(cx+cs*rI,cy+sn*rI);
  rx.lineTo(cx+cs*rE,cy+sn*rE);
  rx.strokeStyle=hc(v);
  rx.lineWidth=Math.max(Math.PI*2*rI/N-.3,1.4);
  rx.lineCap='round';rx.stroke();
  if(v>.3){rx.beginPath();
   rx.arc(cx+cs*(rE+1),cy+sn*(rE+1),1.8,0,Math.PI*2);
   rx.fillStyle=hca(v,.55);rx.fill()}
 }

 /* locked channel indicator */
 if(st.lock){
  const a=(st.lockCh/N)*Math.PI*2-Math.PI/2;
  const cs=Math.cos(a),sn=Math.sin(a);
  rx.beginPath();
  rx.moveTo(cx+cs*(rI-4),cy+sn*(rI-4));
  rx.lineTo(cx+cs*(rO+10),cy+sn*(rO+10));
  rx.strokeStyle='rgba(255,255,255,.5)';rx.lineWidth=1;
  rx.setLineDash([2,3]);rx.stroke();rx.setLineDash([]);
 }

 rx.textAlign='center';rx.textBaseline='middle';
 rx.fillStyle='#18e888';rx.font='bold 13px monospace';
 rx.fillText('2.4G',cx,cy-7);
 rx.fillStyle='#3e6858';rx.font='9px monospace';
 rx.fillText(N+' ch',cx,cy+8);

 rx.fillStyle='#3e6858';rx.font='7px monospace';
 rx.textBaseline='bottom';rx.fillText('2400',cx,cy-rO-8);
 rx.textBaseline='top';rx.fillText('2463',cx,cy+rO+8);
 rx.textAlign='right';rx.textBaseline='middle';
 rx.fillText('2431',cx-rO-6,cy);
 rx.textAlign='left';rx.fillText('2494',cx+rO+6,cy);
}

/* ═══ HEATMAP ═══ */
function drawHeatmap(){
 if(!rf)return;
 let h='';
 for(let i=0;i<N;i++){
  const v=rf.hits[i]/rf.samples;
  h+=`<div class="cx" style="background:${hc(v)};opacity:${Math.max(v,.05)}"
   title="ch${i} ${BASE+i}MHz ${rf.hits[i]}/${rf.samples}"></div>`}
 document.getElementById('hmap').innerHTML=h;
}

/* ═══ BLE MAP ═══ */
function drawBLE(){
 if(!rf)return;
 let h='';
 for(let i=0;i<40;i++){
  const nrfCh=BLE[i];
  const v=rf.hits[nrfCh]||0;
  const isAdv=BLE_ADV.includes(nrfCh);
  const cls='ble-cell'+(v>0?' active':'')+(isAdv?' adv':'');
  const label=isAdv?('Ad'+(nrfCh===2?'37':nrfCh===26?'38':'39')):i;
  h+=`<div class="${cls}" title="BLE ch${i} = ${BASE+nrfCh}MHz, ${v}/${rf.samples}">${label}</div>`}
 document.getElementById('bleMap').innerHTML=h;
}

/* ═══ WIFI LIST ═══ */
function drawWifi(){
 const el=document.getElementById('wList');
 const cnt=document.getElementById('wCnt');
 if(!wf.length){el.innerHTML='<p style="color:var(--t3)">No networks</p>';
  cnt.textContent='';return}
 const sorted=[...wf].sort((a,b)=>b.r-a.r);
 cnt.textContent='('+sorted.length+')';
 document.getElementById('vNt').textContent=sorted.length;
 let html='';
 sorted.forEach(n=>{
  const pct=Math.min(Math.max((n.r+100)/55*100,4),100);
  const col=pct>65?'#18e888':pct>35?'#f0b830':'#ff4050';
  const ico=n.e?'\u{1f512}':'\u{1f513}';
  const nm=n.s||'<span class="hid">hidden</span>';
  html+=`<div class="wi">
   <span style="font-size:12px">${ico}</span>
   <span class="nm">${nm}</span>
   <span class="ch">ch${n.c}<br>${n.r}dB</span>
   <div><div class="br"><div class="bf" style="width:${pct}%;background:${col}"></div></div></div>
  </div>`});
 el.innerHTML=html;
}

/* ═══ STATS ═══ */
function updateStats(){
 if(!rf)return;
 document.getElementById('vPk').textContent='ch '+rf.peakCh;
 document.getElementById('vFr').textContent=(BASE+rf.peakCh)+' MHz';
 document.getElementById('vHt').textContent=rf.peakVal+'/'+rf.samples;
 let act=0;rf.hits.forEach(h=>{if(h>0)act++});
 document.getElementById('vAc').textContent=act+'/'+N;
 const ae=document.getElementById('vAc');
 ae.className='v '+(act>40?'vr':act>15?'vy':'vg');
 document.getElementById('vSw').textContent=rf.sweeps;

 /* lock status */
 const li=document.getElementById('lockInfo');
 if(st.lock){
  li.textContent='\u{1f512} ch'+st.lockCh+' ('+
   (BASE+st.lockCh)+'MHz) hits:'+st.lockHits+'/32';
 }else{li.textContent=''}

 document.getElementById('meta').textContent=
  (st.scan?'scanning':'paused')+' \u00b7 '+
  (st.lock?'locked ch'+st.lockCh:'sweeping')+' \u00b7 '+
  rf.sweepMs+'ms/sweep';
}

/* ═══ FETCH ═══ */
async function fetchRF(){
 try{
  const r=await fetch('/data');rf=await r.json();
  st.scan=rf.scan;st.lock=rf.lock;
  st.lockCh=rf.lockCh;st.lockHits=rf.lockHits;
  updateStats();drawBar();drawRadial();drawHeatmap();drawBLE();
 }catch(e){}}
async function fetchWifi(){
 try{const r=await fetch('/wifi');wf=await r.json();drawWifi()}catch(e){}}

fetchRF();fetchWifi();
setInterval(fetchRF,500);
setInterval(fetchWifi,10000);
window.addEventListener('resize',()=>{drawBar();drawRadial()});
</script>
</body></html>
)HTML";


// ══════════════════════════════════════════════════════════════════
//  HTTP endpoints
// ══════════════════════════════════════════════════════════════════

void handleRoot() {
  server.send_P(200, "text/html", PAGE);
}

void handleData() {
  uint8_t peakCh = 0, peakVal = 0;
  for (uint8_t i = 0; i < CHANNELS; i++) {
    if (hits[i] > peakVal) { peakVal = hits[i]; peakCh = i; }
  }
  String j = "{\"samples\":" + String(SAMPLES)
           + ",\"peakCh\":"  + String(peakCh)
           + ",\"peakVal\":" + String(peakVal)
           + ",\"sweepMs\":" + String(sweepMs)
           + ",\"sweeps\":"  + String(totalSweeps)
           + ",\"scan\":"    + String(scanEnabled ? "true" : "false")
           + ",\"lock\":"    + String(manualLock ? "true" : "false")
           + ",\"lockCh\":"  + String(lockedCh)
           + ",\"lockHits\":" + String(lockedHits)
           + ",\"hits\":[";
  for (uint8_t i = 0; i < CHANNELS; i++) {
    j += String(hits[i]);
    if (i < CHANNELS - 1) j += ',';
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleWifi() {
  server.send(200, "application/json", wifiCache);
}

void handleCtrl() {
  // Start / Stop
  if (server.hasArg("scan")) {
    String s = server.arg("scan");
    if (s == "on")  scanEnabled = true;
    if (s == "off") scanEnabled = false;
  }
  // Reset peaks
  if (server.hasArg("reset")) {
    memset(peak, 0, CHANNELS);
    totalSweeps = 0;
  }
  // Lock to channel
  if (server.hasArg("lock")) {
    int c = server.arg("lock").toInt();
    if (c >= 0 && c <= 125) {
      lockedCh   = (uint8_t)c;
      manualLock = true;
      radio.setChannel(lockedCh);
    }
  }
  // Unlock
  if (server.hasArg("unlock")) {
    manualLock = false;
  }
  server.send(200, "text/plain", "OK");
}

// ══════════════════════════════════════════════════════════════════
//  Setup
// ══════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n=== 2.4 GHz Spectrum Monitor — FINAL ==="));

  // ── Soft AP ────────────────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print(F("SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("Pass: ")); Serial.println(AP_PASS);
  Serial.print(F("IP:   ")); Serial.println(WiFi.softAPIP());

  // ── nRF24L01 ──────────────────────────────────────────────────
  if (!radio.begin()) {
    Serial.println(F("nRF24 FAIL — check wiring / 3.3V / capacitor"));
    while (true) delay(1000);
  }
  Serial.println(F("nRF24 OK"));

  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.setPayloadSize(5);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_2MBPS);     // widest RX bandwidth
  radio.setPALevel(RF24_PA_MIN);     // TX irrelevant — RX only
  radio.startListening();

  // ── HTTP routes ────────────────────────────────────────────────
  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.on("/wifi", handleWifi);
  server.on("/ctrl", handleCtrl);
  server.begin();
  Serial.println(F("Ready — connect to WiFi, open 192.168.4.1"));
}

// ══════════════════════════════════════════════════════════════════
//  Main loop
// ══════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  if (!scanEnabled) return;

  if (manualLock) {
    monitorLocked();    // sit on one channel, count hits
  } else {
    sweep();            // full 126-channel spectrum scan
  }

  refreshWifi();
}
