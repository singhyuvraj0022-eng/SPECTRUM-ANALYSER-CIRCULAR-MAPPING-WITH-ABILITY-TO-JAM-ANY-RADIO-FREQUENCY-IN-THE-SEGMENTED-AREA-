/*
  RF RADAR scanner  -  NodeMCU (ESP8266) + 2 x nRF24L01+ (PA+LNA)
  ==========================================================================
  Listen-only 2.4 GHz scanner. It sends its measurements to rf_radar.py on
  the laptop over USB (115200 baud). The laptop does the radar maths.

  What each radio does
    ESP8266 Wi-Fi radio  monitor mode. For every Wi-Fi device it hears it
                         reports the signal strength (dBm) and the amount of
                         traffic. Only packet headers are looked at, never
                         the contents.
    nRF #A               sweeps 2400-2483 MHz and reports how busy every
                         1 MHz slice is (Wi-Fi, Bluetooth, anything else).
    nRF #B               sits on the Wi-Fi channel being watched and
                         measures its airtime (percent of time on air).

  Wiring
    The nRF modules share the hardware SPI pins:  SCK = D5, MISO = D6,
    MOSI = D7.  The CSN and CE pin of each module are found automatically at
    power-up (D8, D2, D1, D0, D4, D3 are tried) and printed on the serial
    port. To skip the search, fill in the four pin numbers below.

  The only thing this sketch ever transmits is one 1-byte test packet per
  nRF module at power-up. That packet is how the CE pin is identified.

  Board: "NodeMCU 1.0 (ESP-12E Module)", ESP8266 core 2.7 or newer.
  No extra libraries are needed.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
extern "C" {
#include "user_interface.h"
}

// ============================ SETTINGS ====================================

#define PIN_AUTO       255   // find the pin automatically
#define PIN_TIED_HIGH  254   // CE soldered straight to 3.3 V

// GPIO numbers of the nRF pins.   D0=16  D1=5  D2=4  D3=0  D4=2  D8=15
// Leave all four on PIN_AUTO unless the boot message reports a module as
// "not found" or with the wrong pins.
#define NRF_A_CSN  PIN_AUTO
#define NRF_A_CE   PIN_AUTO
#define NRF_B_CSN  PIN_AUTO
#define NRF_B_CE   PIN_AUTO

#define SERIAL_BAUD   115200
#define REPORT_MS     250     // one report (and one channel hop) every 250 ms
#define SPECTRUM_MS   1000    // band-sweep report interval

// ============================ INTERNALS ===================================

#define FW_VERSION    1

#define MAX_DEV       96      // Wi-Fi devices tracked at once
#define MAX_AP        16      // access points remembered (for their names)
#define MAX_LINES     20      // device lines per report (keeps 115200 baud happy)
#define DEV_STALE_MS  15000UL
#define U_BINS        40      // undecoded-frame histogram: -100..-20 dBm in 2 dB steps
#define U_LONG_BYTES  1000    // undecoded frames this long are bulk data, shorter ones are acks
#define SPEC_CH       84      // nRF channels 0..83 = 2400..2483 MHz
#define OUT_SIZE      4096
#define BURST         8       // nRF samples per pass through loop()
#define RX_SETTLE_US  200     // nRF needs 130 us to enter RX + 40 us for the detector

#define F_AP     0x01         // device is an access point / hotspot
#define F_STA    0x02         // device is a client of a network
#define F_PROBE  0x04         // device sent probe requests

// nRF24L01+ registers and commands
#define REG_CONFIG      0x00
#define REG_EN_AA       0x01
#define REG_SETUP_AW    0x03
#define REG_SETUP_RETR  0x04
#define REG_RF_CH       0x05
#define REG_RF_SETUP    0x06
#define REG_STATUS      0x07
#define REG_RPD         0x09
#define REG_RX_PW_P0    0x11
#define REG_RX_PW_P1    0x12
#define REG_DYNPD       0x1C
#define REG_FEATURE     0x1D
#define CMD_W_TX_PAYLOAD 0xA0
#define CMD_FLUSH_TX    0xE1
#define CMD_FLUSH_RX    0xE2
#define CMD_NOP         0xFF

#define CFG_POWER_DOWN  0x7D  // IRQs masked, CRC16, PRIM_RX, powered down
#define CFG_RX          0x7F  // same, powered up  -> receives while CE is high
#define CFG_TX          0x7E  // powered up as transmitter (CE self-test only)
#define ST_TX_DS        0x20

struct Nrf {
  uint8_t  csn;       // GPIO of CSN
  uint8_t  ce;        // GPIO of CE, PIN_TIED_HIGH, or PIN_AUTO when unknown
  bool     ok;        // module answers on SPI
  uint8_t  ch;        // current channel (2400 + ch MHz)
  uint32_t readyUs;   // no sampling before this time (after a retune)
};

struct Dev {
  uint8_t  mac[6];
  uint8_t  bssid[6];  // network this device belongs to (all zero = unknown)
  uint8_t  flags;
  int8_t   rssiMax;
  uint16_t upN;       // frames sent by this device since the last report
  uint16_t dnN;       // frames addressed to this device since the last report
  int32_t  rssiSum;   // sum of the RSSI of the frames it sent
  uint32_t upBytes;
  uint32_t dnBytes;
  uint32_t lastSeen;
  bool     used;
};

struct Ap {
  uint8_t  bssid[6];
  uint8_t  ch;
  int8_t   rssi;
  uint8_t  ssidLen;
  char     ssid[33];
  uint32_t lastSeen;
  uint32_t lastSent;
  bool     used;
};

static const uint8_t CAND[] = {15, 4, 5, 16, 2, 0};   // D8 D2 D1 D0 D4 D3
#define N_CAND (sizeof(CAND) / sizeof(CAND[0]))

static Nrf      nrf[2];
static Dev      devs[MAX_DEV];
static Dev      snap[MAX_LINES];
static Ap       aps[MAX_AP];
static uint16_t uN[2][U_BINS];       // [0] = short frames, [1] = long frames
static uint32_t uBytes[2][U_BINS];
static uint32_t tableFull = 0;

static uint16_t specHits[SPEC_CH];
static uint16_t specCnt[SPEC_CH];
static uint32_t bHits = 0, bN = 0;
static uint8_t  aVisit = 0;

static uint8_t  wifiCh = 1;       // channel the ESP8266 is listening on
static uint8_t  lockCh = 0;       // 0 = hop over channels 1..13
static uint8_t  devRot = 0;
static bool     filtOn = false;   // true = only devices of one network are tracked
static uint8_t  filt[6];
static uint32_t lastReport = 0, lastSpectrum = 0, lastHello = 0;

static char     outBuf[OUT_SIZE];
static uint16_t outHead = 0, outTail = 0;
static uint32_t outDropped = 0;
static char     line[720];
static char     cmdBuf[24];
static uint8_t  cmdLen = 0;

// ---------------------------------------------------------------------------
// Serial output. Everything is queued and sent only when the UART has room,
// so loop() never waits for the serial port and no Wi-Fi frames are missed.
// ---------------------------------------------------------------------------

static uint16_t outFree() {
  return (uint16_t)((outTail + OUT_SIZE - outHead - 1) % OUT_SIZE);
}

static void outLine(const char* s) {
  uint16_t n = (uint16_t)strlen(s);
  if ((uint32_t)n + 1 > outFree()) { outDropped++; return; }
  for (uint16_t i = 0; i < n; i++) {
    outBuf[outHead] = s[i];
    outHead = (uint16_t)((outHead + 1) % OUT_SIZE);
  }
  outBuf[outHead] = '\n';
  outHead = (uint16_t)((outHead + 1) % OUT_SIZE);
}

static void outPump() {
  int room = Serial.availableForWrite();
  while (room > 0 && outTail != outHead) {
    Serial.write((uint8_t)outBuf[outTail]);
    outTail = (uint16_t)((outTail + 1) % OUT_SIZE);
    room--;
  }
}

static const char* pinName(uint8_t gpio) {
  switch (gpio) {
    case 16: return "D0";
    case 5:  return "D1";
    case 4:  return "D2";
    case 0:  return "D3";
    case 2:  return "D4";
    case 15: return "D8";
    case PIN_TIED_HIGH: return "3V3";
    default: return "?";
  }
}

static bool isGpio(uint8_t pin) { return pin <= 16; }

// ---------------------------------------------------------------------------
// nRF24L01+ low level
// ---------------------------------------------------------------------------

static uint8_t nrfRead(uint8_t csn, uint8_t reg) {
  digitalWrite(csn, LOW);
  SPI.transfer(reg & 0x1F);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(csn, HIGH);
  return v;
}

static void nrfWrite(uint8_t csn, uint8_t reg, uint8_t val) {
  digitalWrite(csn, LOW);
  SPI.transfer(0x20 | (reg & 0x1F));
  SPI.transfer(val);
  digitalWrite(csn, HIGH);
}

// Sends a one-byte command and returns the STATUS register.
static uint8_t nrfCmd(uint8_t csn, uint8_t cmd) {
  digitalWrite(csn, LOW);
  uint8_t st = SPI.transfer(cmd);
  digitalWrite(csn, HIGH);
  return st;
}

// True if an nRF24 answers with this pin as its CSN. A register is written
// and read back three times with different values, so a floating MISO line
// cannot pass by accident.
static bool nrfProbe(uint8_t csn) {
  static const uint8_t pat[3] = {0x2A, 0x55, 0x13};
  for (uint8_t i = 0; i < 3; i++) {
    nrfWrite(csn, REG_RF_CH, pat[i]);
    if (nrfRead(csn, REG_RF_CH) != pat[i]) return false;
  }
  return true;
}

static bool isCsnPin(uint8_t pin) {
  return (nrf[0].ok && nrf[0].csn == pin) || (nrf[1].ok && nrf[1].csn == pin);
}

// Finds the CE pin of module m. The module is loaded with one test byte as a
// transmitter; the byte only leaves the chip (TX_DS flag) when CE goes high,
// so the candidate pin that makes the flag appear is CE.
static uint8_t nrfFindCe(uint8_t m) {
  uint8_t csn = nrf[m].csn;
  for (uint8_t i = 0; i < N_CAND; i++) {
    if (isCsnPin(CAND[i])) continue;
    pinMode(CAND[i], OUTPUT);
    digitalWrite(CAND[i], LOW);
  }
  nrfWrite(csn, REG_CONFIG, CFG_TX);
  delay(5);
  nrfWrite(csn, REG_EN_AA, 0x00);
  nrfWrite(csn, REG_SETUP_RETR, 0x00);
  nrfWrite(csn, REG_SETUP_AW, 0x03);
  nrfWrite(csn, REG_RF_CH, 80);
  nrfWrite(csn, REG_RF_SETUP, 0x08);          // 2 Mbit/s, lowest power
  nrfCmd(csn, CMD_FLUSH_TX);
  nrfWrite(csn, REG_STATUS, 0x70);
  digitalWrite(csn, LOW);
  SPI.transfer(CMD_W_TX_PAYLOAD);
  SPI.transfer(0x00);
  digitalWrite(csn, HIGH);
  delay(2);

  uint8_t found = PIN_AUTO;
  if (nrfCmd(csn, CMD_NOP) & ST_TX_DS) {
    found = PIN_TIED_HIGH;                    // sent without any pin moving
  } else {
    for (uint8_t i = 0; i < N_CAND && found == PIN_AUTO; i++) {
      if (isCsnPin(CAND[i])) continue;
      digitalWrite(CAND[i], HIGH);
      delay(2);
      digitalWrite(CAND[i], LOW);
      if (nrfCmd(csn, CMD_NOP) & ST_TX_DS) found = CAND[i];
    }
  }
  nrfCmd(csn, CMD_FLUSH_TX);
  nrfWrite(csn, REG_STATUS, 0x70);
  nrfWrite(csn, REG_CONFIG, CFG_POWER_DOWN);
  return found;
}

// Puts module m into receive mode on channel ch. No address or payload width
// is enabled, so it never receives packets; it is used only as a power
// detector (RPD bit: 1 while more than about -64 dBm is on the channel).
static void nrfSetupRx(uint8_t m, uint8_t ch) {
  uint8_t csn = nrf[m].csn;
  if (isGpio(nrf[m].ce)) {
    pinMode(nrf[m].ce, OUTPUT);
    digitalWrite(nrf[m].ce, LOW);
  }
  nrfWrite(csn, REG_CONFIG, CFG_POWER_DOWN);
  nrfWrite(csn, REG_EN_AA, 0x00);
  nrfWrite(csn, REG_SETUP_AW, 0x03);
  nrfWrite(csn, REG_SETUP_RETR, 0x00);
  nrfWrite(csn, REG_RF_SETUP, 0x09);          // 2 Mbit/s = widest receive filter
  nrfWrite(csn, REG_RX_PW_P0, 0x00);
  nrfWrite(csn, REG_RX_PW_P1, 0x00);
  nrfWrite(csn, REG_DYNPD, 0x00);
  nrfWrite(csn, REG_FEATURE, 0x00);
  nrfWrite(csn, REG_RF_CH, ch);
  nrfCmd(csn, CMD_FLUSH_RX);
  nrfCmd(csn, CMD_FLUSH_TX);
  nrfWrite(csn, REG_STATUS, 0x70);
  nrfWrite(csn, REG_CONFIG, CFG_RX);
  delay(5);                                   // crystal start-up
  nrf[m].ch = ch;
  nrf[m].readyUs = micros();
  if (nrfRead(csn, REG_CONFIG) != CFG_RX) nrf[m].ok = false;
}

static void nrfSetChannel(uint8_t m, uint8_t ch) {
  if (!nrf[m].ok || nrf[m].ch == ch) return;
  uint8_t csn = nrf[m].csn;
  if (nrf[m].ce == PIN_TIED_HIGH) {
    // CE cannot be dropped, so leave receive mode through power-down instead.
    nrfWrite(csn, REG_CONFIG, CFG_POWER_DOWN);
    nrfWrite(csn, REG_RF_CH, ch);
    nrfWrite(csn, REG_CONFIG, CFG_RX);
    nrf[m].readyUs = micros() + 2200;
  } else {
    nrfWrite(csn, REG_RF_CH, ch);             // CE is low between samples
  }
  nrf[m].ch = ch;
}

static void nrfDetect() {
  nrf[0].csn = NRF_A_CSN; nrf[0].ce = NRF_A_CE;
  nrf[1].csn = NRF_B_CSN; nrf[1].ce = NRF_B_CE;
  nrf[0].ok = nrf[1].ok = false;
  nrf[0].ch = nrf[1].ch = 0;
  nrf[0].readyUs = nrf[1].readyUs = 0;

  if (nrf[0].csn == PIN_AUTO && nrf[1].csn == PIN_AUTO) {
    // Every candidate idles high: a CSN line must be high while another
    // module is being addressed.
    for (uint8_t i = 0; i < N_CAND; i++) {
      pinMode(CAND[i], OUTPUT);
      digitalWrite(CAND[i], HIGH);
    }
    uint8_t n = 0;
    for (uint8_t i = 0; i < N_CAND && n < 2; i++) {
      if (!nrfProbe(CAND[i])) continue;
      nrf[n].csn = CAND[i];
      nrf[n].ok = true;
      nrfWrite(CAND[i], REG_RF_SETUP, 0x08);  // radio silent while searching
      nrfWrite(CAND[i], REG_CONFIG, CFG_POWER_DOWN);
      n++;
    }
  } else {
    for (uint8_t m = 0; m < 2; m++) {
      if (nrf[m].csn == PIN_AUTO) continue;
      pinMode(nrf[m].csn, OUTPUT);
      digitalWrite(nrf[m].csn, HIGH);
      if (isGpio(nrf[m].ce)) { pinMode(nrf[m].ce, OUTPUT); digitalWrite(nrf[m].ce, LOW); }
    }
    for (uint8_t m = 0; m < 2; m++) {
      if (nrf[m].csn == PIN_AUTO) continue;
      nrf[m].ok = nrfProbe(nrf[m].csn);
      if (nrf[m].ok) nrfWrite(nrf[m].csn, REG_CONFIG, CFG_POWER_DOWN);
    }
  }

  bool searched = false;
  for (uint8_t m = 0; m < 2; m++) {
    if (nrf[m].ok && nrf[m].ce == PIN_AUTO) { nrf[m].ce = nrfFindCe(m); searched = true; }
  }

  // Let go of every pin that turned out not to belong to an nRF module.
  if (searched || (NRF_A_CSN == PIN_AUTO && NRF_B_CSN == PIN_AUTO)) {
    for (uint8_t i = 0; i < N_CAND; i++) {
      uint8_t p = CAND[i];
      bool inUse = false;
      for (uint8_t m = 0; m < 2; m++) {
        if (nrf[m].ok && (nrf[m].csn == p || nrf[m].ce == p)) inUse = true;
      }
      if (!inUse) pinMode(p, INPUT);
    }
  }
}

// One pass = BURST power samples. CE is raised, the receiver settles, CE is
// dropped (which latches the detector) and the RPD bit is read.
static void nrfBurst() {
  if (!nrf[0].ok && !nrf[1].ok) return;
  for (uint8_t i = 0; i < BURST; i++) {
    uint32_t t = micros();
    bool rdy[2];
    for (uint8_t m = 0; m < 2; m++) {
      rdy[m] = nrf[m].ok && (int32_t)(t - nrf[m].readyUs) >= 0;
      if (rdy[m] && isGpio(nrf[m].ce)) digitalWrite(nrf[m].ce, HIGH);
    }
    delayMicroseconds(RX_SETTLE_US);
    for (uint8_t m = 0; m < 2; m++) {
      if (rdy[m] && isGpio(nrf[m].ce)) digitalWrite(nrf[m].ce, LOW);
    }
    if (rdy[0]) {                             // module A: band sweep
      uint8_t ch = nrf[0].ch;
      if (ch < SPEC_CH) {
        specHits[ch] += nrfRead(nrf[0].csn, REG_RPD) & 0x01;
        specCnt[ch]++;
      }
      // A retune is slow when CE is tied high, so take more samples per visit.
      uint8_t perVisit = (nrf[0].ce == PIN_TIED_HIGH) ? 6 : 1;
      if (++aVisit >= perVisit) {
        aVisit = 0;
        nrfSetChannel(0, (uint8_t)((ch + 1) % SPEC_CH));
      }
    }
    if (rdy[1]) {                             // module B: airtime of one channel
      bHits += nrfRead(nrf[1].csn, REG_RPD) & 0x01;
      bN++;
    }
  }
}

// ---------------------------------------------------------------------------
// Wi-Fi monitor mode
// ---------------------------------------------------------------------------

static bool isGroup(const uint8_t* a) { return (a[0] & 0x01) != 0; }
static bool macEq(const uint8_t* a, const uint8_t* b) { return memcmp(a, b, 6) == 0; }

static Dev* devGet(const uint8_t* mac, bool create, uint32_t now) {
  Dev* freeSlot = NULL;
  Dev* oldest = NULL;
  for (uint8_t i = 0; i < MAX_DEV; i++) {
    Dev* d = &devs[i];
    if (!d->used) { if (!freeSlot) freeSlot = d; continue; }
    if (macEq(d->mac, mac)) return d;
    if (!oldest || (int32_t)(d->lastSeen - oldest->lastSeen) < 0) oldest = d;
  }
  if (!create) return NULL;
  Dev* d = freeSlot;
  if (!d) {
    // Table full: reuse the entry that has been silent the longest.
    if (oldest && (uint32_t)(now - oldest->lastSeen) > DEV_STALE_MS &&
        oldest->upN == 0 && oldest->dnN == 0) {
      d = oldest;
    } else {
      tableFull++;
      return NULL;
    }
  }
  memset(d, 0, sizeof(Dev));
  memcpy(d->mac, mac, 6);
  d->rssiMax = -128;
  d->lastSeen = now;
  d->used = true;
  return d;
}

static void devTx(Dev* d, int8_t rssi, uint32_t bytes, uint32_t now) {
  if (d->upN < 0xFFFF) d->upN++;
  d->rssiSum += rssi;
  if (rssi > d->rssiMax) d->rssiMax = rssi;
  d->upBytes += bytes;
  d->lastSeen = now;
}

// Remembers the name and channel of an access point from its beacon.
static void apSeen(const uint8_t* bssid, int8_t rssi, uint8_t rxCh, const uint8_t* h, uint32_t now) {
  Ap* a = NULL;
  Ap* freeSlot = NULL;
  Ap* oldest = NULL;
  for (uint8_t i = 0; i < MAX_AP; i++) {
    Ap* x = &aps[i];
    if (!x->used) { if (!freeSlot) freeSlot = x; continue; }
    if (macEq(x->bssid, bssid)) { a = x; break; }
    if (!oldest || (int32_t)(x->lastSeen - oldest->lastSeen) < 0) oldest = x;
  }
  if (!a) {
    a = freeSlot;
    if (!a && oldest && (uint32_t)(now - oldest->lastSeen) > 20000UL) a = oldest;
    if (!a) return;
    memset(a, 0, sizeof(Ap));
    memcpy(a->bssid, bssid, 6);
    a->lastSent = now - 60000UL;
    a->used = true;
  }
  a->rssi = rssi;
  a->lastSeen = now;

  // Tagged fields start after the 24-byte header and 12 bytes of fixed fields.
  // The ESP8266 hands over the first 112 bytes of a management frame.
  const uint8_t* p = h + 36;
  const uint8_t* end = h + 112;
  uint8_t ch = 0;
  while (p + 2 <= end) {
    uint8_t id = p[0], l = p[1];
    if (p + 2 + l > end) break;
    if (id == 0 && l >= 1 && l <= 32 && p[2] != 0) {
      memcpy(a->ssid, p + 2, l);
      a->ssid[l] = 0;
      a->ssidLen = l;
    } else if (id == 3 && l == 1) {
      ch = p[2];
      break;
    }
    p += 2 + l;
  }
  // The channel written inside the beacon is the true one; a beacon is often
  // also heard one or two channels away from where it was sent.
  if (ch >= 1 && ch <= 14) a->ch = ch;
  else if (a->ch == 0) a->ch = rxCh;
}

/*
  Monitor-mode callback. The ESP8266 passes one of three buffer layouts:
    len == 12         radio information only. The frame was detected but could
                      not be decoded (LDPC coding, two antennas, 40 MHz). Only
                      its strength and length are known.
    len == 128        management frame: 12 bytes radio info + first 112 bytes
                      of the frame + count (2) + frame length (2).
    len == 50 + 10*n  data frame(s): 12 bytes radio info + first 36 bytes of
                      the frame + count n (2) + n * {length (2), seq (2), addr3 (6)}.
  Radio info: byte 0 = RSSI (signed dBm), byte 1 bits 6-7 = 11n flag,
  bytes 2-3 = legacy length (12 bit), bytes 5-6 = 11n length,
  byte 10 low nibble = channel.
*/
static void sniffCb(uint8_t* b, uint16_t len) {
  int8_t rssi = (int8_t)b[0];

  if (len == 12) {
    bool is11n = ((b[1] >> 6) & 0x03) != 0;
    uint16_t L = is11n ? (uint16_t)(b[5] | (b[6] << 8))
                       : (uint16_t)(b[2] | ((b[3] & 0x0F) << 8));
    int bin = ((int)rssi + 100) / 2;
    if (bin < 0) bin = 0;
    if (bin >= U_BINS) bin = U_BINS - 1;
    uint8_t cls = (L >= U_LONG_BYTES) ? 1 : 0;
    if (uN[cls][bin] < 0xFFFF) uN[cls][bin]++;
    uBytes[cls][bin] += L;
    return;
  }

  uint32_t bytes = 0;
  if (len == 128) {
    bytes = (uint16_t)(b[126] | (b[127] << 8));
  } else if (len >= 60 && (len % 10) == 0) {
    uint16_t cnt = (uint16_t)(b[48] | (b[49] << 8));
    uint16_t maxCnt = (uint16_t)((len - 50) / 10);
    if (cnt > maxCnt) cnt = maxCnt;
    for (uint16_t i = 0; i < cnt; i++) {
      bytes += (uint16_t)(b[50 + 10 * i] | (b[51 + 10 * i] << 8));
    }
  } else {
    return;
  }

  const uint8_t* h = b + 12;                  // 802.11 header
  uint8_t fc0 = h[0], fc1 = h[1];
  if (fc0 & 0x03) return;                     // protocol version is always 0
  uint8_t type = (fc0 >> 2) & 0x03;
  uint8_t sub = fc0 >> 4;
  const uint8_t* a1 = h + 4;                  // receiver
  const uint8_t* a2 = h + 10;                 // transmitter
  const uint8_t* a3 = h + 16;
  uint32_t now = millis();

  if (type == 1) {
    // Control frames that carry a transmitter address (block ack, RTS ...)
    // give extra strength readings for devices that are already known.
    if (sub >= 8 && sub <= 11) {
      Dev* d = devGet(a2, false, now);
      if (d) devTx(d, rssi, bytes, now);
    }
    return;
  }
  if (isGroup(a2)) return;

  // With a network filter set, frames of other networks are not tracked, so a
  // busy building cannot fill the device table.
  bool wanted = !filtOn || macEq(a1, filt) || macEq(a2, filt) || macEq(a3, filt);

  if (type == 0) {                            // management
    bool fromAp = (sub == 8 || sub == 5);     // beacon or probe response
    if (fromAp) apSeen(a2, rssi, b[10] & 0x0F, h, now);
    if (!wanted) return;
    Dev* d = devGet(a2, true, now);
    if (!d) return;
    devTx(d, rssi, bytes, now);
    if (fromAp) {
      d->flags |= F_AP;
      memcpy(d->bssid, a2, 6);
    } else if (sub == 4) {                    // probe request
      d->flags |= F_PROBE;
    } else if (!isGroup(a3)) {
      memcpy(d->bssid, a3, 6);
      d->flags |= macEq(a2, a3) ? F_AP : F_STA;
    }
    return;
  }

  if (type == 2 && wanted) {                  // data
    bool toDS = (fc1 & 0x01) != 0;
    bool fromDS = (fc1 & 0x02) != 0;
    Dev* tx = devGet(a2, true, now);
    if (tx) devTx(tx, rssi, bytes, now);
    if (toDS && !fromDS) {                    // client -> access point
      if (tx) { tx->flags |= F_STA; memcpy(tx->bssid, a1, 6); }
    } else if (fromDS && !toDS) {             // access point -> client
      if (tx) { tx->flags |= F_AP; memcpy(tx->bssid, a2, 6); }
      if (!isGroup(a1)) {
        Dev* rx = devGet(a1, true, now);
        if (rx) {
          rx->flags |= F_STA;
          memcpy(rx->bssid, a2, 6);
          if (rx->dnN < 0xFFFF) rx->dnN++;
          rx->dnBytes += bytes;
          rx->lastSeen = now;
        }
      }
    } else if (!toDS && !fromDS) {
      if (tx && !isGroup(a3)) memcpy(tx->bssid, a3, 6);
    }
  }
}

static void setWifiChannel(uint8_t ch) {
  wifiCh = ch;
  wifi_set_channel(ch);
  nrfSetChannel(1, (uint8_t)(7 + 5 * ch));    // centre of Wi-Fi channel ch
}

// ---------------------------------------------------------------------------
// Reports
//   H,RFRADAR,<version>,<csnA>,<ceA>,<csnB>,<ceB>     pins: 255 = none, 254 = 3V3
//   T,<ms>,<wifi channel>,<locked>,<B hits>,<B samples>,<lost>
//   D,<mac>,<bssid>,<flags>,<upN>,<rssiSum>,<rssiMax>,<upBytes>,<dnN>,<dnBytes>
//   U,<bin>:<frames>:<bytes>,...        undecoded frames under 1000 bytes, bin = (rssi+100)/2
//   W,<bin>:<frames>:<bytes>,...        undecoded frames of 1000 bytes or more
//   B,<bssid>,<channel>,<rssi>,<ssid as hex>
//   S,<samples per channel>,<84 x 2 hex digits>   busy fraction, FF = 100 %
//   E                                   end of report
//   # ...                               text for humans
// Commands from the laptop:  L<ch>  V  F<bssid>  ?
// ---------------------------------------------------------------------------

static void hello() {
  snprintf(line, sizeof(line), "H,RFRADAR,%d,%u,%u,%u,%u", FW_VERSION,
           (unsigned)(nrf[0].ok ? nrf[0].csn : 255), (unsigned)(nrf[0].ok ? nrf[0].ce : 255),
           (unsigned)(nrf[1].ok ? nrf[1].csn : 255), (unsigned)(nrf[1].ok ? nrf[1].ce : 255));
  outLine(line);
}

static void describeNrf(uint8_t m, const char* job) {
  if (!nrf[m].ok) {
    snprintf(line, sizeof(line), "# nRF %c: not found", 'A' + m);
  } else if (nrf[m].ce == PIN_AUTO) {
    snprintf(line, sizeof(line), "# nRF %c: CSN=%s(GPIO%u) but CE pin NOT found - set NRF_%c_CE in the sketch",
             'A' + m, pinName(nrf[m].csn), (unsigned)nrf[m].csn, 'A' + m);
  } else if (nrf[m].ce == PIN_TIED_HIGH) {
    snprintf(line, sizeof(line), "# nRF %c: CSN=%s(GPIO%u) CE=3V3 -> %s",
             'A' + m, pinName(nrf[m].csn), (unsigned)nrf[m].csn, job);
  } else {
    snprintf(line, sizeof(line), "# nRF %c: CSN=%s(GPIO%u) CE=%s(GPIO%u) -> %s",
             'A' + m, pinName(nrf[m].csn), (unsigned)nrf[m].csn,
             pinName(nrf[m].ce), (unsigned)nrf[m].ce, job);
  }
  outLine(line);
}

static void report(uint32_t now) {
  uint8_t nSnap = 0;
  static uint16_t sN[2][U_BINS];
  static uint32_t sB[2][U_BINS];

  // Copy and clear the counters in one go so the Wi-Fi callback can never
  // see them half-updated.
  noInterrupts();
  uint8_t last = devRot;
  for (uint8_t i = 0; i < MAX_DEV && nSnap < MAX_LINES; i++) {
    uint8_t idx = (uint8_t)((devRot + i) % MAX_DEV);
    Dev* d = &devs[idx];
    if (!d->used || (d->upN == 0 && d->dnN == 0)) continue;
    snap[nSnap++] = *d;
    d->upN = 0; d->dnN = 0; d->rssiSum = 0; d->upBytes = 0; d->dnBytes = 0;
    d->rssiMax = -128;
    last = idx;
  }
  // If the limit was hit, the next report starts after the last device sent.
  if (nSnap == MAX_LINES) devRot = (uint8_t)((last + 1) % MAX_DEV);
  memcpy(sN, uN, sizeof(uN));
  memcpy(sB, uBytes, sizeof(uBytes));
  memset(uN, 0, sizeof(uN));
  memset(uBytes, 0, sizeof(uBytes));
  interrupts();

  snprintf(line, sizeof(line), "T,%lu,%u,%u,%lu,%lu,%lu", (unsigned long)now,
           (unsigned)wifiCh, (unsigned)(lockCh ? 1 : 0),
           (unsigned long)bHits, (unsigned long)bN,
           (unsigned long)(outDropped + tableFull));
  outLine(line);
  bHits = 0; bN = 0;

  for (uint8_t i = 0; i < nSnap; i++) {
    Dev* d = &snap[i];
    snprintf(line, sizeof(line),
             "D,%02X%02X%02X%02X%02X%02X,%02X%02X%02X%02X%02X%02X,%u,%u,%ld,%d,%lu,%u,%lu",
             d->mac[0], d->mac[1], d->mac[2], d->mac[3], d->mac[4], d->mac[5],
             d->bssid[0], d->bssid[1], d->bssid[2], d->bssid[3], d->bssid[4], d->bssid[5],
             (unsigned)d->flags, (unsigned)d->upN, (long)d->rssiSum, (int)d->rssiMax,
             (unsigned long)d->upBytes, (unsigned)d->dnN, (unsigned long)d->dnBytes);
    outLine(line);
  }

  int n = 0;
  for (uint8_t cls = 0; cls < 2; cls++) {     // U = short frames, W = long frames
    n = 0;
    for (uint8_t i = 0; i < U_BINS; i++) {
      if (!sN[cls][i]) continue;
      if (n == 0) n = snprintf(line, sizeof(line), "%c", cls ? 'W' : 'U');
      n += snprintf(line + n, sizeof(line) - n, ",%u:%u:%lu", (unsigned)i, (unsigned)sN[cls][i],
                    (unsigned long)sB[cls][i]);
      if (n > (int)sizeof(line) - 32) { outLine(line); n = 0; }   // continue on a new line
    }
    if (n) outLine(line);
  }

  uint8_t sent = 0;
  for (uint8_t i = 0; i < MAX_AP && sent < 3; i++) {
    Ap* a = &aps[i];
    if (!a->used) continue;
    if ((uint32_t)(now - a->lastSeen) > 3000UL) continue;
    if ((uint32_t)(now - a->lastSent) < 2000UL) continue;
    a->lastSent = now;
    n = snprintf(line, sizeof(line), "B,%02X%02X%02X%02X%02X%02X,%u,%d,",
                 a->bssid[0], a->bssid[1], a->bssid[2], a->bssid[3], a->bssid[4], a->bssid[5],
                 (unsigned)a->ch, (int)a->rssi);
    for (uint8_t k = 0; k < a->ssidLen; k++) {
      n += snprintf(line + n, sizeof(line) - n, "%02X", (unsigned)(uint8_t)a->ssid[k]);
    }
    outLine(line);
    sent++;
  }

  if ((uint32_t)(now - lastSpectrum) >= SPECTRUM_MS && nrf[0].ok) {
    lastSpectrum = now;
    uint32_t total = 0;
    for (uint8_t c = 0; c < SPEC_CH; c++) total += specCnt[c];
    n = snprintf(line, sizeof(line), "S,%lu,", (unsigned long)(total / SPEC_CH));
    for (uint8_t c = 0; c < SPEC_CH; c++) {
      unsigned duty = specCnt[c] ? (unsigned)(((uint32_t)specHits[c] * 255UL + specCnt[c] / 2) / specCnt[c]) : 0;
      n += snprintf(line + n, sizeof(line) - n, "%02X", duty);
    }
    outLine(line);
    memset(specHits, 0, sizeof(specHits));
    memset(specCnt, 0, sizeof(specCnt));
  }

  outLine("E");
}

static int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// F<12 hex digits>: track only the network with this BSSID.  F alone: track all.
static void setFilter(const char* hex) {
  uint8_t m[6];
  bool valid = strlen(hex) == 12;
  for (uint8_t i = 0; valid && i < 6; i++) {
    int hi = hexVal(hex[2 * i]), lo = hexVal(hex[2 * i + 1]);
    if (hi < 0 || lo < 0) valid = false;
    else m[i] = (uint8_t)((hi << 4) | lo);
  }
  noInterrupts();
  filtOn = valid;
  if (valid) {
    memcpy(filt, m, 6);
    for (uint8_t i = 0; i < MAX_DEV; i++) {   // forget devices of other networks
      Dev* d = &devs[i];
      if (d->used && !macEq(d->mac, filt) && !macEq(d->bssid, filt)) d->used = false;
    }
  }
  interrupts();
}

static void doCmd(const char* s) {
  if (s[0] == 'L') {                          // L<channel>: stay on one channel
    int ch = atoi(s + 1);
    if (ch >= 1 && ch <= 13) { lockCh = (uint8_t)ch; setWifiChannel(lockCh); }
  } else if (s[0] == 'V') {                   // V: hop over channels 1..13, track everything
    lockCh = 0;
    setFilter("");
  } else if (s[0] == 'F') {
    setFilter(s + 1);
  } else if (s[0] == '?') {
    hello();
  }
}

static void serialIn() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      cmdBuf[cmdLen] = 0;
      if (cmdLen) doCmd(cmdBuf);
      cmdLen = 0;
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);                                 // nRF24 needs 100 ms after power-up
  outLine("");
  outLine("# RF RADAR scanner - listen only");

  SPI.begin();                                // SCK = D5, MISO = D6, MOSI = D7
  SPI.setFrequency(4000000);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  nrfDetect();
  if (nrf[0].ok) nrfSetupRx(0, 0);
  if (nrf[1].ok) nrfSetupRx(1, 12);
  describeNrf(0, "band sweep 2400-2483 MHz");
  describeNrf(1, "airtime of the Wi-Fi channel");

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wifi_set_sleep_type(NONE_SLEEP_T);          // receiver always on
  wifi_promiscuous_enable(0);
  wifi_set_promiscuous_rx_cb(sniffCb);
  wifi_promiscuous_enable(1);
  setWifiChannel(1);

  outLine("# Wi-Fi monitor mode on, hopping channels 1-13");
  hello();
  lastReport = millis();
  lastSpectrum = lastReport;
  lastHello = lastReport;
}

void loop() {
  nrfBurst();

  uint32_t now = millis();
  if ((uint32_t)(now - lastReport) >= REPORT_MS) {
    lastReport += REPORT_MS;
    if ((uint32_t)(now - lastReport) >= REPORT_MS) lastReport = now;   // fell behind
    report(now);
    if (!lockCh) setWifiChannel((uint8_t)(wifiCh % 13 + 1));           // survey: hop
    if ((uint32_t)(now - lastHello) >= 4000UL) { lastHello = now; hello(); }
  }

  serialIn();
  outPump();
}
