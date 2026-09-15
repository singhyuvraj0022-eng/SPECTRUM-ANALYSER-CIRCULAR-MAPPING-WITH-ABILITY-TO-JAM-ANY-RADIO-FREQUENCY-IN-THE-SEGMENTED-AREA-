// ═══════════════════════════════════════════════════════════════════
//  2.4 GHz Passive Spectrum Monitor — FINAL v2
//  ESP8266 (NodeMCU) + nRF24L01   |   RX ONLY, never transmits
//
//  Wiring:
//   VCC  → 3.3V ONLY       CE  → D2 (GPIO4)
//   GND  → GND             CSN → D8 (GPIO15)
//   SCK  → D5  MOSI → D7  MISO → D6   (hardware SPI)
//   ⚠ Add 10–100µF cap across nRF24 VCC/GND
//
//  AP: "RF-Spectrum"  Pass: "spectrum123"  → http://192.168.4.1
// ═══════════════════════════════════════════════════════════════════

#include <SPI.h>
#include "RF24.h"

#ifdef printf_P
  #undef printf_P
#endif

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

#define CE_PIN   4     // D2
#define CSN_PIN  15    // D8

RF24 radio(CE_PIN, CSN_PIN);
ESP8266WebServer server(80);

// ── Scan config ──────────────────────────────────────────────────
#define NUM_CH       126
#define SAMP_PER_CH  40
#define BASE_MHZ     2400

uint16_t spectrum[NUM_CH];
uint16_t peak[NUM_CH];
unsigned long sweepMs     = 0;
unsigned long totalSweeps = 0;

// ── Control state ────────────────────────────────────────────────
bool     scanEnabled = true;
bool     manualLock  = false;
uint8_t  lockedCh    = 0;
uint16_t lockedHits  = 0;

// ── AP ───────────────────────────────────────────────────────────
const char* AP_SSID = "RF-Spectrum";
const char* AP_PASS = "spectrum123";

// ── WiFi scan cache ──────────────────────────────────────────────
String   wifiCache     = "[]";
unsigned long lastWifi = 0;
bool     wifiScanning  = false;

// ══════════════════════════════════════════════════════════════════
//  Fast sweep — same approach as the working circular code
//  No stopListening/startListening per sample → ~100 ms total
// ══════════════════════════════════════════════════════════════════
void sweepFast() {
  unsigned long t0 = millis();
  for (int ch = 0; ch < NUM_CH; ch++) {
    radio.setChannel(ch);
    radio.startListening();
    uint16_t hits = 0;
    for (int s = 0; s < SAMP_PER_CH; s++) {
      if (radio.testRPD()) hits++;
      delayMicroseconds(20);
    }
    spectrum[ch] = hits;
    if (hits > peak[ch]) peak[ch] = hits;
    // yield every 20 channels so WiFi stack can send beacons
    if (ch % 20 == 0) yield();
  }
  sweepMs = millis() - t0;
  totalSweeps++;
}

// ══════════════════════════════════════════════════════════════════
//  Manual lock — sit on one channel, high sample count
// ══════════════════════════════════════════════════════════════════
void monitorLocked() {
  radio.setChannel(lockedCh);
  radio.startListening();
  uint16_t c = 0;
  for (int s = 0; s < 64; s++) {
    if (radio.testRPD()) c++;
    delayMicroseconds(20);
  }
  lockedHits = c;
}

// ══════════════════════════════════════════════════════════════════
//  WiFi scan — non-blocking (async)
// ══════════════════════════════════════════════════════════════════
void handleWifiScan() {
  // start a scan every 12 seconds
  if (!wifiScanning && millis() - lastWifi > 12000) {
    WiFi.scanNetworksAsync([](int n) {
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
           + "}";
      }
      j += "]";
      wifiCache = j;
      WiFi.scanDelete();
      wifiScanning = false;
    });
    wifiScanning = true;
    lastWifi = millis();
  }
}


// ══════════════════════════════════════════════════════════════════
//  Web UI
// ══════════════════════════════════════════════════════════════════
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RF Spectrum Monitor</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
:root{
 --bg:#04070a;--c1:#0a1018;--bd:#142030;
 --t1:#d4ece4;--t2:#78a898;--t3:#3e6858;
 --g:#18e888;--y:#f0b830;--r:#ff4050;
 --bl:#4888d0;--pr:#9868e0;
}
body{background:var(--bg);color:var(--t1);
 font:11.5px/1.45 'Menlo','Fira Code',monospace;
 padding:8px 10px}

.hd{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:4px}
.hd h1{font-size:14px;color:var(--g);font-weight:700}
.hd .tg{font-size:8px;padding:2px 6px;border-radius:8px;
 border:1px solid var(--g);color:var(--g)}
.hd .mt{font-size:9px;color:var(--t3);margin-left:auto}

.ctrl{display:flex;gap:5px;flex-wrap:wrap;align-items:center;
 margin:5px 0 7px;padding:7px;background:var(--c1);
 border:1px solid var(--bd);border-radius:6px}
.ctrl button{padding:5px 12px;border:none;border-radius:4px;
 cursor:pointer;font:11px monospace;font-weight:600;color:#fff}
.bg{background:#18a868}.bs{background:#d03040}.br{background:#5060a0}
.ctrl input[type=number]{width:52px;padding:4px;font:11px monospace;
 background:#080c12;color:var(--t1);border:1px solid var(--bd);
 border-radius:3px;text-align:center}
.ctrl label{font-size:10px;color:var(--t2)}
.ctrl .sp{width:1px;height:22px;background:var(--bd);margin:0 3px}
#lkI{font-size:10px;color:var(--y);margin-left:4px}

.stats{display:grid;grid-template-columns:repeat(auto-fit,minmax(95px,1fr));
 gap:4px;margin-bottom:7px}
.st{background:var(--c1);border:1px solid var(--bd);border-radius:5px;
 padding:6px 8px}
.st .v{font-size:17px;font-weight:800}
.st .l{font-size:8px;color:var(--t3);text-transform:uppercase;letter-spacing:.6px}
.vg{color:var(--g)}.vy{color:var(--y)}.vr{color:var(--r)}.vb{color:var(--bl)}

.row{display:grid;gap:6px;grid-template-columns:1fr 1fr;margin-bottom:6px}
@media(max-width:620px){.row{grid-template-columns:1fr}}
.full{grid-column:1/-1}
.pnl{background:var(--c1);border:1px solid var(--bd);
 border-radius:6px;padding:8px;overflow:hidden}
.pnl h2{font-size:9px;color:var(--t3);font-weight:600;
 text-transform:uppercase;letter-spacing:1px;margin-bottom:4px}
canvas{display:block;width:100%}

#barC{height:150px}
#radWrap{display:flex;justify-content:center}
#radC{max-width:360px;aspect-ratio:1/1}

.leg{display:flex;gap:8px;flex-wrap:wrap;margin-top:4px}
.leg span{font-size:8px;display:flex;align-items:center;gap:3px;color:var(--t2)}
.leg i{width:6px;height:6px;border-radius:50%;display:inline-block}

.wl{max-height:280px;overflow-y:auto;scrollbar-width:thin;
 scrollbar-color:var(--bd) transparent}
.wi{display:grid;grid-template-columns:16px 1fr 36px 48px;
 align-items:center;gap:4px;padding:4px 0;border-bottom:1px solid var(--bd)}
.wi:last-child{border:none}
.wi .nm{overflow:hidden;text-overflow:ellipsis;white-space:nowrap;font-size:10px}
.wi .ch{font-size:8px;color:var(--t3);text-align:center}
.wi .bb{height:4px;border-radius:2px;background:#080c12;overflow:hidden}
.wi .bf{height:100%;border-radius:2px}

.hm{display:flex;flex-wrap:wrap;gap:1px}
.hm .cx{width:5px;height:5px;border-radius:1px}

.bg2{display:grid;grid-template-columns:repeat(10,1fr);gap:2px}
.bc{text-align:center;font-size:7px;padding:3px 0;
 border-radius:2px;background:#080c12;color:var(--t3)}
.bc.on{background:var(--pr);color:#fff;font-weight:700}
.bc.ad{border:1px solid var(--pr)}

#tip{position:fixed;background:#101820;color:var(--t1);
 font-size:9px;padding:3px 6px;border-radius:3px;
 pointer-events:none;display:none;z-index:99;border:1px solid var(--bd)}
</style></head><body>

<div class="hd">
 <h1>&#x1f4e1; RF Spectrum Monitor</h1>
 <span class="tg">PASSIVE RX</span>
 <span class="mt" id="meta">boot&hellip;</span>
</div>

<div class="ctrl">
 <button class="bg" onclick="ctl('scan=on')">&#x25b6; Start</button>
 <button class="bs" onclick="ctl('scan=off')">&#x25a0; Stop</button>
 <button class="br" onclick="ctl('reset=1')">&#x21bb; Reset</button>
 <div class="sp"></div>
 <label>Lock ch:</label>
 <input type="number" id="chIn" min="0" max="125" value="0">
 <button class="bg" onclick="ctl('lock='+document.getElementById('chIn').value)">Lock</button>
 <button class="bs" onclick="ctl('unlock=1')">Unlock</button>
 <span id="lkI"></span>
</div>

<div class="stats">
 <div class="st"><div class="v vg" id="vPk">&mdash;</div><div class="l">Peak Ch</div></div>
 <div class="st"><div class="v vg" id="vFr">&mdash;</div><div class="l">Peak Freq</div></div>
 <div class="st"><div class="v vy" id="vHt">&mdash;</div><div class="l">Hits / Max</div></div>
 <div class="st"><div class="v vg" id="vNt">&mdash;</div><div class="l">WiFi Nets</div></div>
 <div class="st"><div class="v vg" id="vAc">&mdash;</div><div class="l">Active Chs</div></div>
 <div class="st"><div class="v vb" id="vSw">&mdash;</div><div class="l">Sweeps</div></div>
</div>

<div class="row">
 <!-- LINEAR -->
 <div class="pnl full">
  <h2>Linear Spectrum &mdash; 2400–2525 MHz</h2>
  <canvas id="barC" height="150"></canvas>
  <div class="leg">
   <span><i style="background:var(--g)"></i>Low</span>
   <span><i style="background:var(--y)"></i>Med</span>
   <span><i style="background:var(--r)"></i>High</span>
   <span><i style="background:var(--bl)"></i>WiFi</span>
   <span><i style="background:var(--pr)"></i>BT/BLE</span>
  </div>
 </div>

 <!-- RADIAL with radar sweep -->
 <div class="pnl">
  <h2>Radar Spectrum</h2>
  <div id="radWrap"><canvas id="radC"></canvas></div>
 </div>

 <!-- WIFI -->
 <div class="pnl">
  <h2>WiFi Networks <span id="wCnt" style="color:var(--g)"></span></h2>
  <div class="wl" id="wList"><p style="color:var(--t3)">Scanning&hellip;</p></div>
 </div>

 <!-- BLE MAP -->
 <div class="pnl full">
  <h2>BLE Channel Map (purple = activity)</h2>
  <div class="bg2" id="bleM"></div>
  <div class="leg" style="margin-top:5px">
   <span><i style="background:var(--pr)"></i>Active</span>
   <span><i style="border:1px solid var(--pr);width:5px;height:5px"></i>Advert ch 37/38/39</span>
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
const N=126,BASE=2400,S=40;
const WC=[1,6,11,14];
const wF=c=>c===14?2484:2412+(c-1)*5;
const BLE=[2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,32,34,36,38,40,
 42,44,46,48,50,52,54,56,58,60,62,64,66,68,70,72,74,76,78,80];
const ADV=[2,26,80];

const barC=document.getElementById('barC'),bx=barC.getContext('2d');
const radC=document.getElementById('radC'),rx=radC.getContext('2d');
const tip=document.getElementById('tip');

let rf=null, wf=[], st={};
let sweepAngle=0;

/* ═══ color ═══ */
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

function ctl(q){fetch('/ctrl?'+q)}

/* ═══ BAR SPECTRUM ═══ */
function drawBar(){
 if(!rf)return;
 const{w,h}=fit(barC);bx.clearRect(0,0,w,h);
 const bw=w/N,ht=rf.hits;

 bx.fillStyle='rgba(152,104,224,.04)';
 bx.fillRect(2*bw,0,(80-2+1)*bw,h);

 for(let i=0;i<N;i++){
  const v=ht[i]/S;
  const bh=Math.max(v*(h-14),v>0?2:0);
  const g=bx.createLinearGradient(0,h-bh,0,h);
  g.addColorStop(0,hc(v));g.addColorStop(1,hca(v,.15));
  bx.fillStyle=g;
  bx.fillRect(i*bw+.5,h-bh,Math.max(bw-1,1),bh);
 }

 if(st.lock){
  const px=st.lockCh*bw;
  bx.strokeStyle='#fff';bx.lineWidth=1.5;
  bx.setLineDash([3,3]);
  bx.beginPath();bx.moveTo(px,0);bx.lineTo(px,h);bx.stroke();
  bx.setLineDash([]);
  bx.fillStyle='#fff';bx.font='bold 8px monospace';
  bx.textBaseline='top';bx.fillText('\u25bc LOCK',px+3,2);
 }

 bx.font='7px monospace';bx.textBaseline='top';
 for(let ch=1;ch<=14;ch++){
  const f=wF(ch),px=(f-BASE)*bw;
  bx.strokeStyle='rgba(72,136,208,.18)';
  bx.beginPath();bx.moveTo(px,0);bx.lineTo(px,h);bx.stroke();
  if([1,6,11,14].includes(ch)){bx.fillStyle='#4888d0';bx.fillText('W'+ch,px+2,2)}
 }

 bx.fillStyle='#3e6858';bx.textBaseline='bottom';bx.textAlign='left';
 bx.fillText('2400',2,h-1);
 bx.textAlign='right';bx.fillText('2525',w-2,h-1);bx.textAlign='left';
}

barC.onmousemove=e=>{
 if(!rf)return;
 const rc=barC.getBoundingClientRect();
 const ch=Math.floor((e.clientX-rc.left)/rc.width*N);
 if(ch<0||ch>=N){tip.style.display='none';return}
 tip.style.display='block';
 tip.style.left=(e.clientX+10)+'px';
 tip.style.top=(e.clientY-20)+'px';
 tip.textContent=`ch${ch} \u00b7 ${BASE+ch}MHz \u00b7 ${rf.hits[ch]}/${S}`
  +(BLE.includes(ch)?' \u00b7 BLE':'');
};
barC.onmouseleave=()=>tip.style.display='none';

/* ═══ RADAR SPECTRUM (with rotating sweep line) ═══ */
function drawRadar(){
 if(!rf)return;
 const{w,h}=fit(radC);rx.clearRect(0,0,w,h);
 const cx=w/2,cy=h/2;
 const rI=Math.min(cx,cy)*.14;
 const rO=Math.min(cx,cy)*.46;
 const ht=rf.hits;

 /* guide rings */
 rx.strokeStyle='#1a1a1a';rx.lineWidth=.5;
 [.25,.5,.75,1].forEach(f=>{
  rx.beginPath();rx.arc(cx,cy,rI+(rO-rI)*f,0,Math.PI*2);rx.stroke()});

 /* radial spokes every 30° */
 for(let a=0;a<360;a+=30){
  const rad=(a-90)*Math.PI/180;
  rx.beginPath();
  rx.moveTo(cx+Math.cos(rad)*rI,cy+Math.sin(rad)*rI);
  rx.lineTo(cx+Math.cos(rad)*rO,cy+Math.sin(rad)*rO);
  rx.stroke();
 }

 /* BT arc */
 const aS=(2/N)*Math.PI*2-Math.PI/2;
 const aE=(80/N)*Math.PI*2-Math.PI/2;
 rx.beginPath();rx.arc(cx,cy,rO+5,aS,aE);
 rx.strokeStyle='rgba(152,104,224,.25)';rx.lineWidth=2.5;rx.stroke();

 /* WiFi dots */
 WC.forEach(wc=>{
  const a=((wF(wc)-BASE)/N)*Math.PI*2-Math.PI/2;
  rx.beginPath();rx.arc(cx+Math.cos(a)*(rO+12),cy+Math.sin(a)*(rO+12),2,0,Math.PI*2);
  rx.fillStyle='rgba(72,136,208,.4)';rx.fill()});

 /* spectrum wedges */
 const step=Math.PI*2/N;
 for(let i=0;i<N;i++){
  const v=Math.min(1,ht[i]/S);
  const a0=i*step-Math.PI/2;
  const a1=(i+1)*step-Math.PI/2;
  const rE=rI+(rO-rI)*v;

  if(v>0.01){
   const hue=120-Math.min(120,v*140);
   const lit=30+v*25;
   rx.fillStyle=`hsl(${hue},95%,${lit}%)`;
   rx.beginPath();
   rx.moveTo(cx+Math.cos(a0)*rI,cy+Math.sin(a0)*rI);
   rx.arc(cx,cy,rE,a0,a1);
   rx.lineTo(cx+Math.cos(a1)*rI,cy+Math.sin(a1)*rI);
   rx.arc(cx,cy,rI,a1,a0,true);
   rx.closePath();rx.fill();

   /* bright edge */
   rx.strokeStyle=`hsl(${hue},95%,65%)`;rx.lineWidth=.5;
   rx.beginPath();rx.arc(cx,cy,rE,a0,a1);rx.stroke();
  }
 }

 /* rotating sweep line */
 sweepAngle+=0.03;
 if(sweepAngle>Math.PI*2) sweepAngle-=Math.PI*2;
 const sa=sweepAngle-Math.PI/2;
 const grad=rx.createLinearGradient(cx,cy,
  cx+Math.cos(sa)*rO,cy+Math.sin(sa)*rO);
 grad.addColorStop(0,'rgba(0,255,140,0)');
 grad.addColorStop(1,'rgba(0,255,140,0.3)');
 rx.strokeStyle=grad;rx.lineWidth=1.5;
 rx.beginPath();rx.moveTo(cx,cy);
 rx.lineTo(cx+Math.cos(sa)*rO,cy+Math.sin(sa)*rO);
 rx.stroke();
 /* tip dot */
 rx.fillStyle='#0f8';rx.beginPath();
 rx.arc(cx+Math.cos(sa)*rO,cy+Math.sin(sa)*rO,2.5,0,Math.PI*2);
 rx.fill();

 /* locked channel line */
 if(st.lock){
  const la=(st.lockCh/N)*Math.PI*2-Math.PI/2;
  rx.strokeStyle='rgba(255,255,255,.4)';rx.lineWidth=1;
  rx.setLineDash([2,3]);
  rx.beginPath();
  rx.moveTo(cx+Math.cos(la)*rI,cy+Math.sin(la)*rI);
  rx.lineTo(cx+Math.cos(la)*(rO+8),cy+Math.sin(la)*(rO+8));
  rx.stroke();rx.setLineDash([]);
 }

 /* center text */
 rx.textAlign='center';rx.textBaseline='middle';
 rx.fillStyle='#18e888';rx.font='bold 12px monospace';
 rx.fillText('2.4G',cx,cy-6);
 rx.fillStyle='#3e6858';rx.font='8px monospace';
 rx.fillText(N+' ch',cx,cy+8);

 /* labels */
 rx.fillStyle='#555';rx.font='9px Arial';
 rx.textBaseline='bottom';rx.fillText('2400',cx,cy-rO-6);
 rx.textBaseline='top';rx.fillText('2460',cx,cy+rO+6);
 rx.textAlign='right';rx.textBaseline='middle';
 rx.fillText('2430',cx-rO-6,cy);
 rx.textAlign='left';rx.fillText('2500',cx+rO+6,cy);
}

/* ═══ HEATMAP ═══ */
function drawHeatmap(){
 if(!rf)return;let h='';
 for(let i=0;i<N;i++){const v=rf.hits[i]/S;
  h+=`<div class="cx" style="background:${hc(v)};opacity:${Math.max(v,.05)}"
   title="ch${i} ${BASE+i}MHz"></div>`}
 document.getElementById('hmap').innerHTML=h;
}

/* ═══ BLE MAP ═══ */
function drawBLE(){
 if(!rf)return;let h='';
 for(let i=0;i<40;i++){const nch=BLE[i];const v=rf.hits[nch]||0;
  const isAd=ADV.includes(nch);
  const cls='bc'+(v>0?' on':'')+(isAd?' ad':'');
  const lb=isAd?('Ad'+(nch===2?'37':nch===26?'38':'39')):i;
  h+=`<div class="${cls}" title="BLE${i}=${BASE+nch}MHz ${v}/${S}">${lb}</div>`}
 document.getElementById('bleM').innerHTML=h;
}

/* ═══ WIFI ═══ */
function drawWifi(){
 const el=document.getElementById('wList'),cnt=document.getElementById('wCnt');
 if(!wf.length){el.innerHTML='<p style="color:var(--t3)">No networks</p>';
  cnt.textContent='';return}
 const sorted=[...wf].sort((a,b)=>b.r-a.r);
 cnt.textContent='('+sorted.length+')';
 document.getElementById('vNt').textContent=sorted.length;
 let html='';
 sorted.forEach(n=>{
  const pct=Math.min(Math.max((n.r+100)/55*100,4),100);
  const col=pct>65?'#18e888':pct>35?'#f0b830':'#ff4050';
  const nm=n.s||'<span style="color:var(--t3);font-style:italic">hidden</span>';
  html+=`<div class="wi">
   <span style="font-size:11px">${n.e?'\u{1f512}':'\u{1f513}'}</span>
   <span class="nm">${nm}</span>
   <span class="ch">ch${n.c}<br>${n.r}dB</span>
   <div><div class="bb"><div class="bf" style="width:${pct}%;background:${col}"></div></div></div>
  </div>`});
 el.innerHTML=html;
}

/* ═══ STATS ═══ */
function updateStats(){
 if(!rf)return;
 document.getElementById('vPk').textContent='ch '+rf.peakCh;
 document.getElementById('vFr').textContent=(BASE+rf.peakCh)+' MHz';
 document.getElementById('vHt').textContent=rf.peakVal+'/'+S;
 let act=0;rf.hits.forEach(h=>{if(h>0)act++});
 const ae=document.getElementById('vAc');
 ae.textContent=act+'/'+N;
 ae.className='v '+(act>40?'vr':act>15?'vy':'vg');
 document.getElementById('vSw').textContent=rf.sweeps;

 const li=document.getElementById('lkI');
 li.textContent=st.lock?
  '\u{1f512} ch'+st.lockCh+' ('+( BASE+st.lockCh)+'MHz) '+st.lockHits+'/64 hits':'';

 document.getElementById('meta').textContent=
  (st.scan?'scanning':'paused')+' \u00b7 '+
  (st.lock?'locked ch'+st.lockCh:'sweeping')+' \u00b7 '+
  rf.sweepMs+'ms';
}

/* ═══ FETCH ═══ */
async function fetchRF(){
 try{
  const r=await fetch('/data');rf=await r.json();
  st.scan=rf.scan;st.lock=rf.lock;
  st.lockCh=rf.lockCh;st.lockHits=rf.lockHits;
  updateStats();drawBar();drawRadar();drawHeatmap();drawBLE();
 }catch(e){}}
async function fetchWifi(){
 try{const r=await fetch('/wifi');wf=await r.json();drawWifi()}catch(e){}}

/* animation loop for radar sweep even between data polls */
function animate(){drawRadar();requestAnimationFrame(animate)}

fetchRF();fetchWifi();
setInterval(fetchRF,600);
setInterval(fetchWifi,12000);
window.addEventListener('resize',()=>{drawBar();drawRadar()});
animate();
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
  uint8_t peakCh = 0;
  uint16_t peakVal = 0;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    if (spectrum[i] > peakVal) { peakVal = spectrum[i]; peakCh = i; }
  }
  String j = "{\"samples\":" + String(SAMP_PER_CH)
           + ",\"peakCh\":"  + String(peakCh)
           + ",\"peakVal\":" + String(peakVal)
           + ",\"sweepMs\":" + String(sweepMs)
           + ",\"sweeps\":"  + String(totalSweeps)
           + ",\"scan\":"    + String(scanEnabled ? "true" : "false")
           + ",\"lock\":"    + String(manualLock ? "true" : "false")
           + ",\"lockCh\":"  + String(lockedCh)
           + ",\"lockHits\":" + String(lockedHits)
           + ",\"hits\":[";
  for (uint8_t i = 0; i < NUM_CH; i++) {
    j += String(spectrum[i]);
    if (i < NUM_CH - 1) j += ',';
  }
  j += "]}";
  server.send(200, "application/json", j);
}

void handleWifi() {
  server.send(200, "application/json", wifiCache);
}

void handleCtrl() {
  if (server.hasArg("scan")) {
    String s = server.arg("scan");
    if (s == "on")  scanEnabled = true;
    if (s == "off") scanEnabled = false;
  }
  if (server.hasArg("reset")) {
    memset(peak, 0, sizeof(peak));
    memset(spectrum, 0, sizeof(spectrum));
    totalSweeps = 0;
  }
  if (server.hasArg("lock")) {
    int c = server.arg("lock").toInt();
    if (c >= 0 && c <= 125) {
      lockedCh   = (uint8_t)c;
      manualLock = true;
      radio.setChannel(lockedCh);
    }
  }
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
  Serial.println(F("\n=== 2.4 GHz Spectrum Monitor v2 ==="));

  memset(spectrum, 0, sizeof(spectrum));
  memset(peak, 0, sizeof(peak));

  // ── AP (start BEFORE radio so it's ready fast) ─────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS, 9);   // channel 9 to avoid overlap
  Serial.print(F("SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("Pass: ")); Serial.println(AP_PASS);
  Serial.print(F("IP:   ")); Serial.println(WiFi.softAPIP());

  // ── nRF24L01 ──────────────────────────────────────────────────
  if (!radio.begin()) {
    Serial.println(F("nRF24 FAIL — check wiring / 3.3V / cap"));
    while (true) delay(1000);
  }
  Serial.println(F("nRF24 OK"));

  radio.setAutoAck(false);
  radio.setCRCLength(RF24_CRC_DISABLED);
  radio.setPayloadSize(5);
  radio.setAddressWidth(3);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(0);
  radio.startListening();

  // ── HTTP ───────────────────────────────────────────────────────
  server.on("/",     handleRoot);
  server.on("/data", handleData);
  server.on("/wifi", handleWifi);
  server.on("/ctrl", handleCtrl);
  server.begin();
  Serial.println(F("Ready \u2014 connect to WiFi, open 192.168.4.1"));
}

// ══════════════════════════════════════════════════════════════════
//  Loop
// ══════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();
  yield();                  // give WiFi stack time

  if (scanEnabled) {
    if (manualLock) {
      monitorLocked();
    } else {
      sweepFast();
    }
  }

  handleWifiScan();         // non-blocking async scan
}
