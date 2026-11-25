#!/usr/bin/env python3
"""
RF RADAR - laptop display for the NodeMCU + 2 x nRF24 scanner (rf_radar_fw.ino)
================================================================================

Install once:   pip install pyserial matplotlib numpy

Run:            python rf_radar.py --ssid "MyHotspot" --room 5.5x8.5
                python rf_radar.py                  (pick the network on screen)
                python rf_radar.py --demo           (no hardware, simulated room)

Before the first sweep
  1. Phones must be on a 2.4 GHz network (the scanner cannot hear 5 GHz):
     set a phone hotspot to the 2.4 GHz band and connect the phones to it.
  2. Reflector: cover a 50 x 25 cm piece of cardboard with kitchen foil, fold
     it into a V (two 25 cm panels at 90 degrees) and stand it up. Put the
     board inside the V with its antennas about 5 cm from the fold. The open
     side of the V is the "arrow". Board and reflector turn together.
  3. Dial: draw a clock face on paper under the board. Its 12 o'clock mark
     points at the wall shown at the top of the floor plan.

How it locates a device
  With the reflector behind it the board hears best along the arrow. You turn
  it through the 12 clock positions; at each one the ESP8266 measures the
  signal strength of every Wi-Fi device on the network.
    direction  = where a device is loudest (a cosine is fitted through all
                 12 readings, so one bad reading does not decide)
    distance   = from the strength on the arrow: about 22 dB weaker per 10x distance
    position   = direction + distance, kept inside the room
  A second sweep from another spot is combined with the first one.

How it judges activity
  Bytes sent to and from each device per second. Packets the ESP8266 can
  detect but not decode carry no address; they are credited to the device
  heard at that strength, so two phones that are equally loud can be mixed up
  (the list then shows a question mark).

Keys
  S        start a sweep            SPACE  board is turned / skip to next step
  A        auto-advance on/off      X      cancel the sweep
  click    on the floor plan = where the scanner sits
  1        the strongest phone is 1 m in front of the arrow (distance calibration)
  K        the busiest phone is streaming video right now (activity calibration)
  0        forget both calibrations
  C        clear results            R      scan for networks again
  F        full screen              Q      quit
"""

import argparse
import collections
import json
import math
import os
import queue
import random
import sys
import threading
import time
import traceback
import zlib

import numpy as np

FW_BAUD = 115200

# ---- sweep
STEPS_DEFAULT = 12        # headings per turn (12 = clock positions)
DWELL_DEFAULT = 8.0       # seconds measured at each heading
TURN_SECONDS = 5.0        # auto-advance: seconds allowed for turning the board
SETTLE = 0.6              # seconds ignored right after a turn
SURVEY_SECONDS = 7.0      # two passes over channels 1-13
MIN_PKTS = 3              # packets needed from a device at one heading
MAX_TRACK = 8             # devices shown at once (one colour each)

# ---- position model
RSSI_1M_DEFAULT = -40.0   # dBm heard from a phone 1 m in front of the arrow (key 1 calibrates)
PATH_N_DEFAULT = 2.2      # path-loss exponent indoors, line of sight
SIGMA_RSSI = 6.0          # dB uncertainty of strength -> distance
SAME_DB = 3.6             # two strengths closer than this cannot be told apart
ACK_RATIO = 60.0          # a download is at most this many times the bytes of its acknowledgements
GRID = 0.1                # m, floor-plan resolution

# ---- activity model
STREAM_REF_DEFAULT = 150e3  # bytes/s seen from a phone that streams video (key K calibrates)

ZERO_MAC = "000000000000"

# ---- colours (dark theme; device colours are a colour-blind-checked set, used in fixed order)
PAGE, SURFACE = "#0d0d0d", "#1a1a19"
INK, INK2, MUTED = "#f0efec", "#c3c2b7", "#898781"
GRIDC, AXISC = "#2c2c2a", "#383835"
CAT = ["#3987e5", "#d95926", "#199e70", "#c98500", "#d55181", "#008300", "#9085e9", "#e66767"]
GOOD, WARN, SERIOUS, CRIT = "#0ca30c", "#fab219", "#ec835a", "#d03b3b"
BLUE_TRACK = "#0d366b"

Q_NONE, Q_WEAK, Q_LOCKED = 0, 1, 2
Q_NAME = {Q_NONE: "no direction", Q_WEAK: "weak fix", Q_LOCKED: "LOCKED"}


# =============================================================================
# helpers
# =============================================================================

def clock_name(deg):
    h = int(round((deg % 360.0) / 30.0)) % 12
    return "%d o'clock" % (12 if h == 0 else h)


def fmt_dbm(v):
    return "\u2212%.0f dBm" % abs(v) if v < 0 else "%.0f dBm" % v


def fmt_rate(bytes_per_s):
    b = bytes_per_s * 8.0
    if b < 1000:
        return "under 1 kb/s"
    if b < 1e6:
        return "%.0f kb/s" % (b / 1e3)
    return "%.1f Mb/s" % (b / 1e6)


def mac_tail(mac):
    return "\u00b7\u00b7%s:%s" % (mac[8:10], mac[10:12])


def wmedian(samples):
    """Weighted median of (value, weight) pairs."""
    s = sorted(samples)
    total = sum(w for _, w in s)
    acc = 0.0
    for v, w in s:
        acc += w
        if acc >= total / 2.0:
            return v
    return s[-1][0]


def fit_direction(head_deg, vals, wts):
    """Fit  strength = a0 + A*cos(heading - theta)  through the readings.

    Returns theta (deg, clockwise from 12 o'clock), A (dB), a0 (dB) and the
    leave-one-out spread of theta (deg): how far a single bad reading can move
    the answer, which is used as the direction uncertainty.
    """
    h = np.radians(np.asarray(head_deg, float))
    y = np.asarray(vals, float)
    w = np.sqrt(np.asarray(wts, float))

    def solve(idx):
        X = np.column_stack([np.ones(len(idx)), np.cos(h[idx]), np.sin(h[idx])])
        c, *_ = np.linalg.lstsq(X * w[idx, None], y[idx] * w[idx], rcond=None)
        return c

    idx = np.arange(len(h))
    a0, a1, b1 = solve(idx)
    theta = math.degrees(math.atan2(b1, a1)) % 360.0
    amp = math.hypot(a1, b1)
    dev = []
    for i in idx:
        c = solve(np.delete(idx, i))
        t = math.degrees(math.atan2(c[2], c[1]))
        dev.append((t - theta + 180.0) % 360.0 - 180.0)
    n = len(dev)
    spread = math.sqrt((n - 1) / n * sum(d * d for d in dev)) if n > 1 else 180.0
    return theta, amp, a0, spread


# =============================================================================
# tracking core (no graphics in here)
# =============================================================================

class Dev:
    def __init__(self, mac):
        self.mac = mac
        self.bssid = ""
        self.flags = 0
        self.ch = 0
        self.rssi = None          # dBm at the current board heading (smoothed)
        self.last_tx = -1e9       # last time one of its own packets was decoded
        self.frames = 0
        self.clock = 0.0          # seconds this device has been observable (stops while the board turns)
        self.hist = collections.deque()   # (clock, up_bytes, dn_bytes, up_frames, decoded_bytes)
        self.solo = 0.0           # bytes/s it used while no other phone was heard at its strength
        self.dec_rate = 0.0       # bytes/s read with its address on them (smoothed)
        self.tag = None
        self.slot = None

    @property
    def is_ap(self):
        return bool(self.flags & 1)

    def rate(self, win):
        """(upload, download) in bytes/s over the last `win` seconds it was observable."""
        up = dn = 0.0
        for t, u, d, _, _ in reversed(self.hist):
            if self.clock - t >= win:
                break
            up += u
            dn += d
        span = min(win, max(self.clock, 1.0))
        return up / span, dn / span

    def decoded_share(self, win=12.0):
        """Part of its traffic that was read with its address on it (0..1)."""
        tot = dec = 0.0
        for t, u, d, _, k in reversed(self.hist):
            if self.clock - t >= win:
                break
            tot += u + d
            dec += k
        return dec / tot if tot > 0 else 1.0


class Fix:
    """Result of one sweep for one device."""

    def __init__(self, x, y, theta, sigma, peak, amp, heads, quality, curve):
        self.x, self.y = x, y            # where the scanner stood
        self.theta = theta               # direction, deg clockwise from 12 o'clock (None if unknown)
        self.sigma = sigma               # direction uncertainty, deg
        self.peak = peak                 # dBm when the arrow points at the device
        self.amp = amp                   # dB between loudest and average heading
        self.heads = heads               # headings with enough packets
        self.quality = quality
        self.curve = curve               # [(heading deg, dBm)] as measured


class Radar:
    CONNECT, SURVEY, PICK, LIVE, TURN, MEASURE = range(6)

    def __init__(self, room, steps=STEPS_DEFAULT, dwell=DWELL_DEFAULT, auto=False,
                 ssid=None, channel=None, send=None, settings_path=None):
        self.W, self.L = room
        self.steps = int(steps)
        self.dwell = float(dwell)
        self.auto = auto
        self.turn_s = TURN_SECONDS
        self.want_ssid = ssid
        self.want_ch = channel
        self.send = send or (lambda s: None)
        self.settings_path = settings_path
        self.rssi1m = RSSI_1M_DEFAULT
        self.path_n = PATH_N_DEFAULT
        self.stream_ref = STREAM_REF_DEFAULT
        self._load_settings()

        self.state = self.CONNECT
        self.t_state = 0.0
        self.devs = {}
        self.aps = {}
        self.hw = None
        self.log = collections.deque(maxlen=30)
        self.cur = None
        self.last_frame_t = -1e9
        self.n_reports = 0
        self.wifi_ch = 0
        self.lost = 0
        self.lock_ch = 0
        self.target = None            # set of BSSIDs being watched, None = everything on the channel
        self.target_name = ""
        self.last_cmd_t = -1e9
        self.spectrum = None
        self.spec_t = -1e9
        self.airtime = None
        self.air_src = ""
        self.air_hist = collections.deque(maxlen=240)
        self.net_frames = collections.Counter()
        self.pick = []
        self.tracked = []
        self.tag_no = {"AP": 0, "P": 0}
        self.pos = (self.W / 2.0, self.L / 2.0)
        self.k = 0
        self.obs = {}
        self.fixes = {}
        self.est = {}
        self.sweep_positions = []
        self.result_t = None
        self.toast = ("", -1e9)

    # ------------------------------------------------------------ settings
    def _load_settings(self):
        try:
            with open(self.settings_path) as f:
                s = json.load(f)
            self.rssi1m = float(s.get("rssi1m", self.rssi1m))
            self.path_n = float(s.get("path_n", self.path_n))
            self.stream_ref = float(s.get("stream_ref", self.stream_ref))
        except Exception:
            pass

    def _save_settings(self):
        if not self.settings_path:
            return
        try:
            with open(self.settings_path, "w") as f:
                json.dump({"rssi1m": self.rssi1m, "path_n": self.path_n,
                           "stream_ref": self.stream_ref}, f, indent=1)
        except Exception:
            pass

    # ------------------------------------------------------------ serial lines in
    def feed(self, line, now):
        line = line.strip()
        if not line:
            return
        c = line[0]
        try:
            if c == "T" and line[1:2] == ",":
                p = line.split(",")
                self.cur = {"ch": int(p[2]), "lock": int(p[3]), "bh": int(p[4]),
                            "bn": int(p[5]), "lost": int(p[6]), "D": [], "U": []}
            elif c == "D" and self.cur is not None:
                p = line.split(",")
                if len(p) == 10 and len(p[1]) == 12 and len(p[2]) == 12:
                    self.cur["D"].append((p[1], p[2], int(p[3]), int(p[4]), int(p[5]),
                                          int(p[6]), int(p[7]), int(p[8]), int(p[9])))
            elif c in "UW" and line[1:2] == "," and self.cur is not None:
                for item in line.split(",")[1:]:
                    b, n, by = item.split(":")
                    self.cur["U"].append((int(b), int(n), int(by), c == "W"))
            elif c == "B" and line[1:2] == ",":
                p = line.split(",")
                bssid, ch, rssi = p[1], int(p[2]), int(p[3])
                if len(bssid) == 12 and 1 <= ch <= 14:
                    ssid = bytes.fromhex(p[4]).decode("utf-8", "replace") if len(p) > 4 else ""
                    ap = self.aps.get(bssid)
                    if ap is None:
                        ap = self.aps[bssid] = {"ssid": "", "ch": ch, "rssi": rssi, "t": now}
                    if ssid:
                        ap["ssid"] = ssid
                    ap["ch"] = ch
                    # a beacon is also heard, weaker, on the neighbouring channels: keep the loudest
                    ap["rssi"] = max(rssi, ap["rssi"] - 1)
                    ap["t"] = now
            elif c == "S" and line[1:2] == ",":
                p = line.split(",")
                hx = p[2]
                vals = [int(hx[i:i + 2], 16) / 255.0 for i in range(0, len(hx) - 1, 2)]
                if len(vals) >= 84:
                    self.spectrum = np.array(vals[:84])
                    self.spec_t = now
            elif line == "E" and self.cur is not None:
                f, self.cur = self.cur, None
                self._commit(f, now)
            elif c == "H" and line.startswith("H,RFRADAR"):
                p = line.split(",")
                self.hw = {"ver": int(p[2]), "a": (int(p[3]), int(p[4])), "b": (int(p[5]), int(p[6]))}
            elif c == "#":
                self.log.append(line[1:].strip())
        except (ValueError, IndexError):
            return        # a damaged line (for example boot noise) is simply skipped

    def _locked(self, f):
        return bool(self.lock_ch and f["lock"] and f["ch"] == self.lock_ch)

    def _commit(self, f, now):
        """One 250 ms report from the scanner is complete."""
        self.last_frame_t = now
        ch = f["ch"]
        self.wifi_ch = ch
        self.lost = f["lost"]
        if self.state == self.CONNECT:
            self._begin_survey(now)
        locked = self._locked(f)

        # channel airtime: nRF B if fitted, otherwise taken from nRF A's band sweep
        if locked:
            duty = None
            if f["bn"] >= 50:
                duty, self.air_src = f["bh"] / f["bn"], "nRF B"
            elif self.spectrum is not None and now - self.spec_t < 3.0:
                c = 7 + 5 * ch
                duty, self.air_src = float(np.mean(self.spectrum[max(0, c - 8):c + 9])), "nRF A"
            if duty is not None:
                self.airtime = duty if self.airtime is None else 0.6 * self.airtime + 0.4 * duty
                self.air_hist.append((now, self.airtime))

        up_b, dn_b, up_n, ack_b = {}, {}, {}, {}
        for mac, bssid, fl, upn, rsum, _rmax, upb, dnn, dnb in f["D"]:
            if not (0 <= upn <= 20000 and 0 <= dnn <= 20000):
                continue
            d = self.devs.get(mac)
            if d is None:
                d = self.devs[mac] = Dev(mac)
            d.flags |= fl
            if bssid != ZERO_MAC:
                d.bssid = bssid
            if upn > 0:
                r = rsum / upn
                if not -105.0 <= r <= 0.0:
                    continue
                d.rssi = r if (d.rssi is None or now - d.last_tx > 2.0) else 0.5 * d.rssi + 0.5 * r
                d.last_tx = now
                d.frames += upn
                d.ch = ch
                if self.state == self.MEASURE and locked and now - self.t_state >= SETTLE:
                    self.obs.setdefault(mac, [[] for _ in range(self.steps)])[self.k].append((r, upn))
                if (d.flags & 2) and not d.is_ap:
                    self.net_frames[d.bssid] += upn
                if d.is_ap and mac not in self.aps:
                    self.aps[mac] = {"ssid": "", "ch": ch, "rssi": r, "t": now}
            up_b[mac] = up_b.get(mac, 0) + upb
            dn_b[mac] = dn_b.get(mac, 0) + dnb
            up_n[mac] = up_n.get(mac, 0) + upn
            if upn > 0 and upb / upn <= 400:
                ack_b[mac] = ack_b.get(mac, 0) + upb

        # Packets the ESP8266 could detect but not decode (modern phones use
        # codings it lacks) carry no address, only a strength and a length.
        #   long packet at the hotspot's strength  -> download, shared out below
        #   any other packet at a phone's strength -> sent by that phone
        # Two phones heard at the same strength cannot be told apart this way;
        # their packets are then split by what each used while it was alone.
        dec_b = {m: up_b.get(m, 0) + dn_b.get(m, 0) for m in set(up_b) | set(dn_b)}
        # A phone's strength is only known once it has been heard at the board's
        # present heading; while the board is being turned nothing can be credited.
        watching = locked and self.state != self.TURN
        t_still = self.t_state

        def heard(d):
            return d.rssi is not None and d.ch == ch and d.last_tx >= t_still and now - d.last_tx < 180.0

        if watching and self.target:
            mine = [d for d in self.devs.values() if self._member(d) and heard(d)]
            aps = [d for d in mine if d.is_ap]
            stas = [d for d in mine if not d.is_ap]
            pool = 0.0
            for b, n, by, is_long in f["U"]:
                r = 2 * b - 99.5
                near_ap = [a for a in aps if abs(a.rssi - r) <= SAME_DB]
                near = [d for d in stas if abs(d.rssi - r) <= SAME_DB]
                if near_ap and (is_long or not near):
                    pool += by
                    up_b[near_ap[0].mac] = up_b.get(near_ap[0].mac, 0) + by
                    continue
                if not near:
                    continue                      # nobody we watch is heard at this strength
                frac = len(near) / (len(near) + 1.0) if near_ap else 1.0
                pool += by * (1.0 - frac)
                if len(near) == 1:
                    ws = [1.0]
                else:
                    ws = [math.exp(-0.5 * ((d.rssi - r) / 1.5) ** 2) * (d.solo + 5.0 * d.dec_rate + 200.0)
                          for d in near]
                tot = sum(ws)
                for d, w in zip(near, ws):
                    up_b[d.mac] = up_b.get(d.mac, 0) + by * frac * w / tot
                    up_n[d.mac] = up_n.get(d.mac, 0) + n * frac * w / tot
                    if not is_long:
                        ack_b[d.mac] = ack_b.get(d.mac, 0) + by * frac * w / tot
            if pool > 0:
                # A download goes to whoever is acknowledging it. Share it by the
                # small packets each phone sent in this report (one stray packet
                # does not count), and never credit more than those could acknowledge.
                ev = {d.mac: max(0.0, ack_b.get(d.mac, 0.0) - 100.0) for d in stas}
                tot = sum(ev.values())
                for m, x in ev.items():
                    if x > 0:
                        dn_b[m] = dn_b.get(m, 0) + min(pool * x / tot, ACK_RATIO * x)
            for d in stas:
                d.dec_rate += (dec_b.get(d.mac, 0) / 0.25 - d.dec_rate) * 0.12     # about 2 s of memory
                alone = all(o is d or abs(o.rssi - d.rssi) > 2 * SAME_DB for o in stas)
                if alone:
                    inst = (up_b.get(d.mac, 0) + dn_b.get(d.mac, 0)) / 0.25
                    d.solo += (inst - d.solo) * 0.05          # about 5 s of memory
                else:
                    d.solo *= 0.996                           # fades over a minute while it cannot be checked

        # Traffic is logged against each device's own clock, which runs only
        # while the device can be observed, so turning the board leaves no hole.
        if watching:
            for d in self.devs.values():
                if not (d.is_ap or self.target is None or heard(d)):
                    continue
                d.clock += 0.25
                m = d.mac
                if m in dec_b or m in up_b or m in dn_b:
                    d.hist.append((d.clock, up_b.get(m, 0), dn_b.get(m, 0), up_n.get(m, 0), dec_b.get(m, 0)))
                while d.hist and d.clock - d.hist[0][0] > 20.0:
                    d.hist.popleft()

        if locked:
            self._update_tracked(now)
        elif self.lock_ch and now - self.last_cmd_t > 1.5:
            self._send_lock(now)      # the board was reset or missed the command

        # In a busy building thousands of devices pass by: forget the ones long gone.
        self.n_reports += 1
        if self.n_reports % 240 == 0:
            for mac in [m for m, d in self.devs.items() if m not in self.tracked and now - d.last_tx > 300.0]:
                del self.devs[mac]
            keep = set(self.pick) | (self.target or set())
            for b in [b for b, ap in self.aps.items() if b not in keep and now - ap["t"] > 600.0]:
                del self.aps[b]

    # ------------------------------------------------------------ network choice
    def _begin_survey(self, now):
        self.state = self.SURVEY
        self.t_state = now
        self.lock_ch = 0
        self.target = None
        self.net_frames.clear()
        self.pick = []
        self.send("V\n")
        if self.want_ch:
            self._lock(self.want_ch, None, "channel %d" % self.want_ch, now)

    def _match_ssid(self):
        want = self.want_ssid.strip().lower()
        m = [(ap["rssi"], b, ap) for b, ap in self.aps.items() if ap["ssid"].strip().lower() == want]
        if not m:
            return None
        m.sort(reverse=True)
        ch = m[0][2]["ch"]
        return ch, {b for _, b, ap in m if ap["ch"] == ch}, m[0][2]["ssid"]

    def _enter_pick(self, now):
        aps = sorted(self.aps.items(), key=lambda kv: -kv[1]["rssi"])[:9]
        self.pick = [b for b, _ in aps]
        self.state = self.PICK
        self.t_state = now

    def choose(self, i, now):
        if self.state != self.PICK or not 0 <= i < len(self.pick):
            return
        b = self.pick[i]
        ap = self.aps[b]
        self._lock(ap["ch"], {b}, ap["ssid"] or "hidden network", now)

    def rescan(self, now):
        if self.state in (self.TURN, self.MEASURE):
            return
        self.want_ch = self.want_ssid = None      # after R the network is picked from the list
        self._begin_survey(now)

    def _lock(self, ch, bssids, name, now):
        self.lock_ch = ch
        self.target = set(bssids) if bssids else None
        self.target_name = name
        self.state = self.LIVE
        self.t_state = now
        self.tracked = []
        self.tag_no = {"AP": 0, "P": 0}
        for d in self.devs.values():
            d.tag = d.slot = None
            d.hist.clear()
            d.solo = d.dec_rate = d.clock = 0.0
        self.obs, self.fixes, self.est, self.sweep_positions = {}, {}, {}, []
        self.result_t = None
        self.airtime = None
        self.air_hist.clear()
        self._send_lock(now)

    def _send_lock(self, now):
        self.send("L%d\n" % self.lock_ch)
        if self.target and len(self.target) == 1:
            self.send("F%s\n" % next(iter(self.target)))
        else:
            self.send("F\n")
        self.last_cmd_t = now

    def _member(self, d):
        return self.target is None or d.mac in self.target or d.bssid in self.target

    def _update_tracked(self, now):
        for mac in list(self.tracked):
            d = self.devs[mac]
            if now - d.last_tx > 120.0 and mac not in self.fixes:
                self.tracked.remove(mac)
                d.tag = d.slot = None
        need = 5 if self.target else 20
        cands = [d for d in self.devs.values()
                 if d.mac not in self.tracked and d.rssi is not None and self._member(d)
                 and d.ch == self.lock_ch and now - d.last_tx < 20.0 and d.frames >= need]
        cands.sort(key=lambda d: not d.is_ap)       # access point first, then in order of appearance
        for d in cands:
            if len(self.tracked) >= MAX_TRACK:
                break
            n_ap = sum(1 for m in self.tracked if self.devs[m].is_ap)
            if d.is_ap and n_ap >= 2:
                continue
            used = {self.devs[m].slot for m in self.tracked}
            d.slot = min(s for s in range(MAX_TRACK) if s not in used)
            kind = "AP" if d.is_ap else "P"
            self.tag_no[kind] += 1
            d.tag = ("AP" if self.tag_no["AP"] == 1 else "AP%d" % self.tag_no["AP"]) if d.is_ap \
                else "P%d" % self.tag_no["P"]
            self.tracked.append(d.mac)

    # ------------------------------------------------------------ activity
    def activity(self, d, now):
        """(label, level 0-3, bytes/s) from the traffic to and from a device."""
        fu, fd = d.rate(4.0)
        su, sd = d.rate(12.0)
        fast, slow = fu + fd, su + sd
        ref = self.stream_ref
        shown = max(slow, fast if fast > 3 * slow else 0.0)
        if slow >= 0.5 * ref:
            return ("UPLOADING" if su > 1.5 * sd else "STREAMING"), 3, shown
        if slow >= 0.08 * ref or fast >= 0.25 * ref:
            return "BROWSING", 2, shown
        if slow >= 0.004 * ref or fast >= 0.012 * ref:
            return "MESSAGING", 1, shown
        return "IDLE", 0, shown

    def shares_with(self, d, now):
        """Tags of other phones heard at the same strength as d, when d's traffic
        was credited by strength (not read by address) and so may be theirs."""
        if d.is_ap or d.rssi is None or d.decoded_share() >= 0.3:
            return []
        if sum(d.rate(12.0)) < 0.004 * self.stream_ref:
            return []
        out = []
        for m in self.tracked:
            o = self.devs[m]
            if o is not d and not o.is_ap and o.rssi is not None and now - o.last_tx < 180.0 \
                    and abs(o.rssi - d.rssi) <= SAME_DB:
                out.append(o.tag)
        return out

    # ------------------------------------------------------------ sweep
    def heading(self):
        return self.k * 360.0 / self.steps

    def start_sweep(self, now):
        if self.state != self.LIVE:
            return
        self.obs = {}
        self.k = 0
        self.state = self.TURN
        self.t_state = now

    def cancel_sweep(self, now):
        if self.state in (self.TURN, self.MEASURE):
            self.state = self.LIVE
            self.t_state = now

    def action(self, now):
        """SPACE / click on the banner: whatever the banner is asking for."""
        if self.state == self.LIVE:
            self.start_sweep(now)
        elif self.state == self.TURN:
            self.state = self.MEASURE
            self.t_state = now
        elif self.state == self.MEASURE:
            self._next_heading(now)

    def tick(self, now):
        if self.state == self.SURVEY:
            el = now - self.t_state
            if self.want_ssid:
                m = self._match_ssid()
                if m and el >= 3.5:
                    self._lock(m[0], m[1], m[2], now)
            elif el >= SURVEY_SECONDS and self.aps:
                self._enter_pick(now)
        elif self.state == self.PICK:
            for b in self.aps:                  # networks found later join the end of the list
                if b not in self.pick and len(self.pick) < 9:
                    self.pick.append(b)
        elif self.state == self.TURN:
            if self.auto and now - self.t_state >= self.turn_s:
                self.state = self.MEASURE
                self.t_state = now
        elif self.state == self.MEASURE:
            el = now - self.t_state
            # stay longer (up to 2.5x) while a phone that was talking has sent too few packets here
            if el >= self.dwell and (el >= 2.5 * self.dwell or not self.starved(now)):
                self._next_heading(now)

    def _next_heading(self, now):
        self.k += 1
        if self.k >= self.steps:
            self._finish_sweep(now)
        else:
            self.state = self.TURN
            self.t_state = now

    def starved(self, now):
        """Tags of the tracked devices with too few packets at the current heading."""
        out = []
        for mac in self.tracked:
            d = self.devs[mac]
            if now - d.last_tx > 30.0:
                continue                          # gone quiet: not worth waiting for
            got = sum(n for _, n in self.obs.get(mac, [[]] * self.steps)[self.k])
            if got < MIN_PKTS:
                out.append(d.tag)
        return out

    def curve(self, mac):
        """[(heading, dBm)] measured so far in the running sweep."""
        out = []
        for k, samples in enumerate(self.obs.get(mac, [])):
            if sum(n for _, n in samples) >= MIN_PKTS:
                out.append((k * 360.0 / self.steps, wmedian(samples)))
        return out

    def _fit(self, mac, pos):
        obs = self.obs.get(mac)
        if not obs:
            return None
        heads, vals, wts = [], [], []
        for k, samples in enumerate(obs):
            n = sum(c for _, c in samples)
            if n < MIN_PKTS:
                continue
            heads.append(k * 360.0 / self.steps)
            vals.append(wmedian(samples))
            wts.append(min(n, 30) / 30.0)
        if len(heads) < 3:
            return None
        curve = list(zip(heads, vals))
        gaps = [(heads[(i + 1) % len(heads)] - heads[i]) % 360.0 for i in range(len(heads))]
        if len(heads) < 6 or max(gaps) > 120.0:
            # too few headings for a direction: keep only the distance
            return Fix(pos[0], pos[1], None, 180.0, max(vals), 0.0, len(heads), Q_NONE, curve)
        theta, amp, a0, spread = fit_direction(heads, vals, wts)
        sigma = min(60.0, math.hypot(spread, 10.0))      # 10 deg floor: hand turning, reflections
        # Strength with the arrow on the device: the readings either side of the
        # fitted direction (the same thing key 1 measures at 1 m).
        step = 360.0 / self.steps
        near = [(1.0 - abs((h - theta + 180.0) % 360.0 - 180.0) / step, v) for h, v in curve
                if abs((h - theta + 180.0) % 360.0 - 180.0) < step]
        wsum = sum(w for w, _ in near)
        peak = sum(w * v for w, v in near) / wsum if wsum > 0.2 else a0 + amp
        if amp >= 2.5 and spread <= 20.0 and len(heads) >= 8:
            q = Q_LOCKED
        elif amp >= 1.5 and spread <= 45.0:
            q = Q_WEAK
        else:
            q = Q_NONE
        return Fix(pos[0], pos[1], theta, sigma, peak, amp, len(heads), q, curve)

    def _finish_sweep(self, now):
        pos = self.pos
        for mac in self.tracked:
            fx = self._fit(mac, pos)
            if fx is None:
                continue
            keep = [g for g in self.fixes.get(mac, []) if math.hypot(g.x - pos[0], g.y - pos[1]) > 0.3]
            self.fixes[mac] = (keep + [fx])[-4:]
        if not any(math.hypot(p[0] - pos[0], p[1] - pos[1]) <= 0.3 for p in self.sweep_positions):
            self.sweep_positions.append(pos)
        self.solve()
        self.state = self.LIVE
        self.t_state = now
        self.result_t = now

    def solve(self):
        """Turn the sweeps into a spot on the floor plan for every device."""
        gx = np.arange(GRID / 2, self.W, GRID)
        gy = np.arange(GRID / 2, self.L, GRID)
        X, Y = np.meshgrid(gx, gy)
        self.est = {}
        for mac, fixes in self.fixes.items():
            last = fixes[-1]
            dist = 10 ** ((self.rssi1m - last.peak) / (10 * self.path_n))
            directed = [f for f in fixes if f.quality != Q_NONE]
            if not directed:
                self.est[mac] = {"q": Q_NONE, "ring": (last.x, last.y, dist), "fix": last, "dist": dist}
                continue
            ll = np.zeros_like(X)
            for f in fixes:
                dx, dy = X - f.x, Y - f.y
                d = np.maximum(np.hypot(dx, dy), 0.25)
                pred = self.rssi1m - 10 * self.path_n * np.log10(d)
                ll -= 0.5 * ((f.peak - pred) / SIGMA_RSSI) ** 2
                if f.quality != Q_NONE:
                    brg = np.degrees(np.arctan2(dx, dy))
                    dth = (brg - f.theta + 180.0) % 360.0 - 180.0
                    ll -= 0.5 * (dth / f.sigma) ** 2
            p = np.exp(ll - ll.max())
            iy, ix = np.unravel_index(int(np.argmax(p)), p.shape)
            x, y = float(gx[ix]), float(gy[iy])
            pn = p / p.sum()
            mx, my = float((pn * X).sum()), float((pn * Y).sum())
            spread = math.sqrt(float((pn * ((X - mx) ** 2 + (Y - my) ** 2)).sum()))
            q = max(f.quality for f in fixes)
            self.est[mac] = {"q": q, "x": x, "y": y, "spread": spread, "img": p, "fix": last,
                             "dist": math.hypot(x - last.x, y - last.y), "n_pos": len(directed)}

    def summary(self):
        n = {Q_LOCKED: 0, Q_WEAK: 0, Q_NONE: 0}
        for mac in self.tracked:
            e = self.est.get(mac)
            if e:
                n[e["q"]] += 1
        return n

    def clear(self, now):
        if self.state != self.LIVE:
            return
        self.fixes, self.est, self.sweep_positions, self.obs = {}, {}, [], {}
        self.result_t = None

    def set_pos(self, x, y):
        if self.state != self.LIVE:
            return
        self.pos = (min(max(x, 0.1), self.W - 0.1), min(max(y, 0.1), self.L - 0.1))

    # ------------------------------------------------------------ calibration
    def cal_1m(self, now):
        c = [self.devs[m] for m in self.tracked
             if not self.devs[m].is_ap and now - self.devs[m].last_tx < 3.0]
        if self.state != self.LIVE or not c:
            self.toast = ("No phone is sending right now: open a video on it and try again", now)
            return
        d = max(c, key=lambda v: v.rssi)
        if not -70.0 <= d.rssi <= -15.0:
            self.toast = ("%s reads %s, which cannot be 1 m away: check the phone is the one in front" % (d.tag, fmt_dbm(d.rssi)), now)
            return
        self.rssi1m = d.rssi
        self._save_settings()
        if self.fixes:
            self.solve()
        self.toast = ("Distance calibrated: %s at 1 m = %s" % (d.tag, fmt_dbm(d.rssi)), now)

    def cal_reset(self, now):
        self.rssi1m, self.path_n, self.stream_ref = RSSI_1M_DEFAULT, PATH_N_DEFAULT, STREAM_REF_DEFAULT
        self._save_settings()
        if self.fixes:
            self.solve()
        self.toast = ("Calibration reset to the built-in values", now)

    def cal_stream(self, now):
        c = [self.devs[m] for m in self.tracked if not self.devs[m].is_ap]
        if self.state != self.LIVE or not c:
            return
        d = max(c, key=lambda v: sum(v.rate(12.0)))
        r = sum(d.rate(12.0))
        if r < 2000:
            self.toast = ("No phone is busy enough: let the video play for 15 s, then press K", now)
            return
        self.stream_ref = r
        self._save_settings()
        self.toast = ("Activity calibrated: streaming on %s = %s" % (d.tag, fmt_rate(r)), now)


# =============================================================================
# demo: a simulated room that speaks the scanner's serial protocol
# =============================================================================

class Simulator:
    def __init__(self, room, seed=4, ripple=1.0, decodable=0.35, null_rate=2.0, reflector=True):
        self.W, self.L = room
        W, L = room
        self.rng = random.Random(seed)
        self.ripple = ripple          # multiplier on the reflections (1.0 = about 1.5 dB rms)
        self.dec = decodable          # share of data packets the ESP8266 can decode
        self.null_rate = null_rate    # always-decodable packets per second from each phone
        self.reflector = reflector    # False = board without its foil reflector (hears all round)
        self.pos = (W / 2.0, L / 2.0)
        self.heading = 0.0
        self._cmd_heading = None
        self.lock = 0
        self.ch = 1
        self.filt = None
        self.t0 = None
        self.n = 0
        self.pending = []
        self._rip = {}
        self._air6 = 0.05
        self.ap = {"mac": "02DE40C0FFEE", "ssid": "DemoSpot", "ch": 6, "pos": (W - 0.7, L - 0.9), "off": 1.0}
        self.phones = [
            {"mac": "A6B1C2D30011", "pos": (1.1, L - 1.7), "off": 0.0, "mode": "stream"},
            {"mac": "CE55E6F70022", "pos": (W - 1.1, 2.1), "off": -2.0, "mode": "chat"},
            {"mac": "7A01B2C30033", "pos": (1.3, 1.2), "off": 1.5, "mode": "browse"},
        ]
        for i, p in enumerate(self.phones):
            p["st"] = {"next": 2.0 + 2.0 * i, "until": 0.0, "rate": 0.0}
            p["phase"] = 1.5 * i
        self.other_ap = {"mac": "3C8412345678", "ssid": "Campus-WiFi", "ch": 6, "pos": (W + 5.0, L * 0.4), "off": 4.0}
        self.other_sta = {"mac": "5E0000AA0044", "pos": (W + 3.0, L * 0.6), "off": 0.0}
        self.far_ap = {"mac": "0A1B2C3D4E5F", "ssid": "Lab-2G", "ch": 11, "pos": (-6.0, L + 3.0), "off": 4.0}

    def truth(self):
        t = {self.ap["mac"]: self.ap["pos"]}
        t.update({p["mac"]: p["pos"] for p in self.phones})
        return t

    def set_pose(self, pos, heading):
        """The demo 'operator' turns the board, a few degrees off like a real hand."""
        self.pos = pos
        if heading != self._cmd_heading:
            self._cmd_heading = heading
            self.heading = heading + self.rng.uniform(-4.0, 4.0)

    def command(self, s):
        for c in s.replace("\r", "\n").split("\n"):
            if c.startswith("L") and c[1:].isdigit() and 1 <= int(c[1:]) <= 13:
                self.lock = self.ch = int(c[1:])
            elif c.startswith("V"):
                self.lock, self.filt = 0, None
            elif c.startswith("F"):
                self.filt = c[1:] if len(c) == 13 else None
            elif c.startswith("?"):
                self.pending.append("H,RFRADAR,1,15,4,5,16")

    def _ripple(self, mac, heading):
        key = (mac, round(self.pos[0], 1), round(self.pos[1], 1))
        c = self._rip.get(key)
        if c is None:
            r = random.Random(zlib.crc32(repr(key).encode()))
            c = [(j, r.uniform(0.5, 1.0) * a, r.uniform(0, 2 * math.pi))
                 for j, a in ((1, 1.0), (2, 1.6), (3, 1.6), (4, 1.2), (5, 1.0))]
            self._rip[key] = c
        h = math.radians(heading)
        return self.ripple * sum(a * math.cos(j * h + ph) for j, a, ph in c)

    def _rssi(self, dev):
        dx, dy = dev["pos"][0] - self.pos[0], dev["pos"][1] - self.pos[1]
        d = max(0.3, math.hypot(dx, dy))
        a = math.radians(self.heading - math.degrees(math.atan2(dx, dy)))
        gain = 5.5 * math.cos(a) + 1.5 * math.cos(2 * a) - 7.0     # 0 dB on the arrow, -11 dB behind
        if not self.reflector:
            gain = -3.0
        return (RSSI_1M_DEFAULT + dev["off"] - 10 * PATH_N_DEFAULT * math.log10(d) + gain
                + self._ripple(dev["mac"], self.heading))

    def _traffic(self, ph, t):
        st, mode, rng = ph["st"], ph["mode"], self.rng
        if mode == "stream":                  # video player: fetch for 4 s, play for 5 s
            dn = 700e3 if (t + ph["phase"]) % 9.0 < 4.0 else 12e3
        elif mode == "browse":
            if t >= st["next"]:
                st["until"], st["rate"] = t + rng.uniform(1.5, 3.0), rng.uniform(80e3, 250e3)
                st["next"] = t + rng.uniform(7.0, 12.0)
            dn = st["rate"] if t < st["until"] else 1.5e3
        elif mode == "chat":
            if t >= st["next"]:
                st["until"], st["rate"] = t + 0.5, rng.uniform(8e3, 30e3)
                st["next"] = t + rng.uniform(3.0, 7.0)
            dn = st["rate"] if t < st["until"] else 300.0
        else:
            dn = 150.0
        return dn * 0.25 * rng.uniform(0.8, 1.2), (dn * 0.05 + 200.0) * 0.25 * rng.uniform(0.8, 1.2)

    def _split(self, n):
        """How many of n packets the ESP8266 decodes."""
        if n <= 0:
            return 0
        if n > 60:
            return max(0, int(round(n * self.dec + self.rng.gauss(0, math.sqrt(n * self.dec * (1 - self.dec))))))
        return sum(1 for _ in range(n) if self.rng.random() < self.dec)

    def _rsum(self, r, n):
        return int(round(n * r + self.rng.gauss(0, 1.5) * math.sqrt(n)))

    def _frame(self, t):
        rng = self.rng
        out = []
        ch = self.ch
        ubins = {"U": {}, "W": {}}

        def ubin(r, n, each):
            """n undecodable packets of `each` bytes heard at strength r."""
            parts = min(n, 12)                    # spread over a few 2 dB bins like real readings
            for i in range(parts):
                b = min(39, max(0, int((round(r + rng.gauss(0, 1.2)) + 100) // 2)))
                x = ubins["W" if each >= 1000 else "U"].setdefault(b, [0, 0])
                k = n // parts + (1 if i < n % parts else 0)
                x[0] += k
                x[1] += k * each

        dl = []
        total = 0.0
        if ch == 6:
            ap_r = self._rssi(self.ap)
            ap_n, ap_b = rng.choice((2, 2, 3)), 0
            ap_b += ap_n * 180
            for ph in self.phones:
                dn, up = self._traffic(ph, t)
                total += dn + up
                r = self._rssi(ph)
                n_dn = int(dn // 1400)
                dec = self._split(n_dn)
                dnb = dec * 1400
                if n_dn == 0 and dn > 40 and rng.random() < self.dec:
                    dec, dnb = 1, int(dn)
                ubin(ap_r, n_dn - dec, 1400)
                ap_n += dec
                ap_b += dnb
                n_ack = int(up // 90)
                dec_ack = self._split(n_ack)
                ubin(r, n_ack - dec_ack, 90)
                # power-save notices are sent at a basic rate, so they always decode
                udec = dec_ack + (1 if rng.random() < self.null_rate * 0.25 else 0)
                if udec or dec:
                    dl.append("D,%s,%s,2,%d,%d,%d,%d,%d,%d" % (
                        ph["mac"], self.ap["mac"], udec, self._rsum(r, udec) if udec else 0,
                        int(round(r)) + 2 if udec else -128, udec * 90, dec, dnb))
            dl.insert(0, "D,%s,%s,1,%d,%d,%d,%d,0,0" % (
                self.ap["mac"], self.ap["mac"], ap_n, self._rsum(ap_r, ap_n), int(round(ap_r)) + 2, ap_b))
            if self.filt is None:
                o, s = self.other_ap, self.other_sta
                dl.append("D,%s,%s,1,2,%d,%d,400,0,0" % (o["mac"], o["mac"], self._rsum(self._rssi(o), 2),
                                                          int(round(self._rssi(o)))))
                if rng.random() < 0.6:
                    dl.append("D,%s,%s,2,3,%d,%d,900,2,2800" % (s["mac"], o["mac"], self._rsum(self._rssi(s), 3),
                                                                 int(round(self._rssi(s)))))
            if self.n % 8 == 0:
                out.append("B,%s,6,%d,%s" % (self.ap["mac"], int(round(ap_r)), self.ap["ssid"].encode().hex().upper()))
            if self.n % 8 == 4:
                out.append("B,%s,6,%d,%s" % (self.other_ap["mac"], int(round(self._rssi(self.other_ap))),
                                             self.other_ap["ssid"].encode().hex().upper()))
        elif ch in (5, 7) and self.filt is None:
            r = self._rssi(self.ap) - 13
            dl.append("D,%s,%s,1,2,%d,%d,360,0,0" % (self.ap["mac"], self.ap["mac"], self._rsum(r, 2), int(round(r))))
            out.append("B,%s,6,%d,%s" % (self.ap["mac"], int(round(r)), self.ap["ssid"].encode().hex().upper()))
        elif ch == 11 and self.filt is None:
            r = self._rssi(self.far_ap)
            dl.append("D,%s,%s,1,2,%d,%d,400,0,0" % (self.far_ap["mac"], self.far_ap["mac"], self._rsum(r, 2),
                                                      int(round(r))))
            out.append("B,%s,11,%d,%s" % (self.far_ap["mac"], int(round(r)), self.far_ap["ssid"].encode().hex().upper()))

        air6 = min(0.95, 0.04 + total * 4 * 8 / 20e6)
        if ch == 6:
            self._air6 = air6
        air = {6: air6, 5: 0.7 * self._air6, 7: 0.7 * self._air6, 11: 0.08}.get(ch, 0.015)
        bh = int(round(1088 * min(1.0, air * rng.uniform(0.9, 1.1))))
        head = ["T,%d,%d,%d,%d,1088,0" % (int(t * 1000), ch, 1 if self.lock else 0, bh)]
        for key in ("U", "W"):
            if ubins[key]:
                dl.append(key + "".join(",%d:%d:%d" % (b, v[0], v[1]) for b, v in sorted(ubins[key].items())))
        out = head + dl + out
        if self.n % 4 == 0:
            vals = []
            for c in range(84):
                v = 0.012 * rng.random()
                if 28 <= c <= 46:
                    v += self._air6 * (1.0 if 30 <= c <= 44 else 0.5)
                if 53 <= c <= 71:
                    v += 0.08 * (1.0 if 55 <= c <= 69 else 0.5)
                vals.append(min(255, int(round(255 * v * rng.uniform(0.85, 1.15)))))
            out.append("S,51," + "".join("%02X" % v for v in vals))
        out.append("E")
        if self.n % 16 == 0:
            out.append("H,RFRADAR,1,15,4,5,16")
        if not self.lock:
            self.ch = self.ch % 13 + 1
        return out

    def lines_until(self, now):
        out = []
        if self.t0 is None:
            self.t0 = now
            for s in ("# RF RADAR scanner - listen only (SIMULATED)",
                      "# nRF A: CSN=D8(GPIO15) CE=D2(GPIO4) -> band sweep 2400-2483 MHz",
                      "# nRF B: CSN=D1(GPIO5) CE=D0(GPIO16) -> airtime of the Wi-Fi channel",
                      "H,RFRADAR,1,15,4,5,16"):
                out.append((now, s))
        for s in self.pending:
            out.append((now, s))
        self.pending = []
        if now - (self.t0 + 0.25 * self.n) > 5.0:           # the window was frozen: skip ahead
            self.n = int((now - self.t0) / 0.25) - 4
        while self.t0 + 0.25 * (self.n + 1) <= now:
            self.n += 1
            t = self.t0 + 0.25 * self.n
            for s in self._frame(0.25 * self.n):
                out.append((t, s))
        return out


# =============================================================================
# data sources
# =============================================================================

class DemoSource:
    def __init__(self, sim):
        self.sim = sim
        self.status = "simulated scanner"

    def poll(self, now):
        return self.sim.lines_until(now)

    def send(self, s):
        self.sim.command(s)

    def close(self):
        pass


def port_problem(port, err):
    s = str(err).lower()
    if "access is denied" in s or "busy" in s:
        return "%s is busy: close the Arduino Serial Monitor (only one program can use the port)" % port
    if "permission" in s:
        return "no permission to open %s (Linux: add yourself to the dialout group), or another program has it" % port
    if "filenotfound" in s or "no such file" in s or "cannot find" in s:
        return "%s not found: is the board plugged in?" % port
    return "cannot open %s (%s)" % (port, str(err)[:70])


class SerialSource:
    """Reads the scanner on a background thread and reconnects if the cable is pulled."""

    def __init__(self, port, record=None):
        import serial
        self._serial = serial
        self.port = port
        self.q = queue.Queue()
        self.ser = None
        self.status = "opening %s" % port
        self._stop = False
        self._rec = open(record, "w", buffering=1) if record else None
        threading.Thread(target=self._run, daemon=True).start()

    def _open(self):
        s = self._serial.Serial()
        s.port = self.port
        s.baudrate = FW_BAUD
        s.timeout = 0.2
        s.dtr = False        # keep the NodeMCU out of reset / flash mode
        s.rts = False
        s.open()
        return s

    def _run(self):
        buf = b""
        while not self._stop:
            if self.ser is None:
                try:
                    self.ser = self._open()
                    self.status = "%s open at %d baud" % (self.port, FW_BAUD)
                    self.ser.write(b"?\n")
                except Exception as e:
                    self.status = port_problem(self.port, e)
                    time.sleep(1.0)
                    continue
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except Exception:
                self.status = "%s disconnected, retrying" % self.port
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)
                continue
            if not data:
                continue
            buf += data
            if len(buf) > 16384:
                buf = b""
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "ignore").strip()
                t = time.monotonic()
                if self._rec:
                    self._rec.write("%.3f %s\n" % (t, line))
                if line.startswith("#"):
                    print(line)
                self.q.put((t, line))

    def poll(self, now):
        out = []
        try:
            while True:
                out.append(self.q.get_nowait())
        except queue.Empty:
            pass
        return out

    def send(self, s):
        try:
            if self.ser is not None:
                self.ser.write(s.encode("ascii"))
        except Exception:
            pass

    def close(self):
        self._stop = True
        if self._rec:
            self._rec.close()


def find_port():
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    usb = [p for p in ports if p.vid in (0x1A86, 0x10C4, 0x0403)
           or any(k in (p.description or "").lower() for k in ("ch340", "cp210", "usb-serial", "usb serial", "uart"))]
    pick = usb or (ports if len(ports) == 1 else [])
    if not pick:
        print("No scanner found. Serial ports on this computer:")
        for p in ports:
            print("   %s  %s" % (p.device, p.description))
        print("Plug in the NodeMCU, or name the port:  python rf_radar.py --port COM5")
        sys.exit(1)
    if len(pick) > 1:
        print("Several USB serial ports found, using %s. Others: %s" % (
            pick[0].device, ", ".join(p.device for p in pick[1:])))
    return pick[0].device


# =============================================================================
# window
# =============================================================================

def pick_font():
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in ("Segoe UI", "Helvetica Neue", "Arial", "DejaVu Sans"):
        if name in have:
            return name
    return "sans-serif"


class View:
    ROWS = 9

    def __init__(self, radar, source, clock, sim=None):
        self.r = radar
        self.src = source
        self.clock = clock
        self.sim = sim
        self._solved = None
        self._t0 = clock()
        self._build()

    # ------------------------------------------------------------ construction
    def _build(self):
        import matplotlib as mpl
        mpl.rcParams["toolbar"] = "None"
        for k in list(mpl.rcParams.keys()):
            if k.startswith("keymap."):
                mpl.rcParams[k] = []            # S, K, C ... are our keys, not matplotlib's
        mpl.rcParams["font.family"] = pick_font()
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Wedge, Circle, PathPatch
        from matplotlib import patheffects
        self.plt = plt
        r = self.r
        fig = self.fig = plt.figure(figsize=(12.4, 7.0), facecolor=PAGE)     # fits a 1366 x 768 laptop screen
        sim_note = " (simulated)" if self.sim is not None else ""
        try:
            fig.canvas.manager.set_window_title("RF Radar" + (" - DEMO, simulated data" if self.sim is not None else ""))
        except Exception:
            pass
        halo = [patheffects.withStroke(linewidth=3, foreground=SURFACE)]

        def panel(rect, title=None, sub=None):
            ax = fig.add_axes(rect, zorder=0)
            ax.set_facecolor(SURFACE)
            ax.set_xlim(0, 1)
            ax.set_ylim(0, 1)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
            t = None
            if title:
                t = fig.text(rect[0] + 0.009, rect[1] + rect[3] - 0.013, title, color=INK, fontsize=10.5,
                             fontweight="bold", va="top", ha="left")
            if sub:
                fig.text(rect[0] + rect[2] - 0.009, rect[1] + rect[3] - 0.015, sub, color=MUTED,
                         fontsize=8.5, va="top", ha="right")
            return ax, t

        # ---------------- banner
        ax, _ = panel((0.012, 0.888, 0.976, 0.100))
        self.ax_b = ax
        self.pill = Rectangle((0.010, 0.20), 0.108, 0.60, fc=AXISC, ec="none", transform=ax.transAxes)
        ax.add_patch(self.pill)
        self.pill_t = ax.text(0.064, 0.50, "", color=INK, fontsize=11, fontweight="bold", ha="center", va="center")
        self.main_t = ax.text(0.132, 0.62, "", color=INK, fontsize=16.5, fontweight="bold", ha="left", va="center")
        self.sub_t = ax.text(0.132, 0.27, "", color=INK2, fontsize=10.2, ha="left", va="center")
        ax.add_patch(Rectangle((0.132, 0.045), 0.615, 0.045, fc=GRIDC, ec="none"))
        self.prog = Rectangle((0.132, 0.045), 0.0, 0.045, fc=WARN, ec="none")
        ax.add_patch(self.prog)
        self.step_lbl = ax.text(0.772, 0.80, "sweep steps", color=MUTED, fontsize=8.5, ha="left", va="center")
        self.step_boxes = []
        dx = 0.214 / r.steps
        for i in range(r.steps):
            b = Rectangle((0.772 + i * dx, 0.30), dx * 0.78, 0.32, fc=AXISC, ec="none")
            ax.add_patch(b)
            self.step_boxes.append(b)

        # ---------------- direction finder (polar)
        panel((0.012, 0.215, 0.352, 0.660), "Direction finder" + sim_note, "signal strength as the board turns")
        ax = self.ax_r = fig.add_axes((0.040, 0.238, 0.296, 0.548), projection="polar", zorder=2)
        ax.set_facecolor(SURFACE)
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_rlabel_position(15)
        ax.set_xticks(np.radians(np.arange(0, 360, 30)))
        ax.set_xticklabels(["12"] + [str(i) for i in range(1, 12)], color=INK2, fontsize=10)
        ax.grid(color=GRIDC, linewidth=1.0, linestyle="-")
        ax.spines["polar"].set_color(AXISC)
        ax.tick_params(pad=2)
        step = 2 * math.pi / r.steps
        self.beam = ax.bar([0.0], [70.0], width=step, bottom=-95.0, color=INK, alpha=0.0, zorder=1)[0]
        self.beam_line, = ax.plot([0, 0], [-95, -25], color=INK, lw=1.5, alpha=0.0, zorder=3)
        self.rim = ax.bar(np.arange(r.steps) * step, [2.2] * r.steps, width=step * 0.80, bottom=-27.4,
                          color=AXISC, zorder=4)
        self.r_curve, self.r_ray, self.r_blip, self.r_lbl = [], [], [], []
        for s in range(MAX_TRACK):
            ln, = ax.plot([], [], color=CAT[s], lw=2, marker="o", ms=5.5, mec=SURFACE, mew=1.2, zorder=5,
                          solid_joinstyle="round")
            ray, = ax.plot([], [], color=CAT[s], lw=2, alpha=0.9, zorder=6, solid_capstyle="round")
            blip, = ax.plot([], [], marker="o", ms=11, color=CAT[s], mec=SURFACE, mew=2, ls="none", zorder=7)
            lbl = ax.text(0, 0, "", color=INK, fontsize=9.5, fontweight="bold", ha="center", va="center",
                          zorder=8, path_effects=halo)
            self.r_curve.append(ln)
            self.r_ray.append(ray)
            self.r_blip.append(blip)
            self.r_lbl.append(lbl)
        self.r_hint = fig.text(0.188, 0.512, "", color=MUTED, fontsize=10, ha="center", va="center", zorder=9)
        self._rlim = None
        self._scale = 1.0
        self._was_sweeping = False
        self._set_rlim(-90.0, -30.0)

        # ---------------- floor plan
        _, self.plan_title = panel((0.372, 0.215, 0.268, 0.660), "Floor plan" + sim_note)
        fig.text(0.372 + 0.268 - 0.009, 0.875 - 0.015, "%.1f m \u00d7 %.1f m" % (r.W, r.L), color=MUTED,
                 fontsize=8.5, va="top", ha="right")
        ax = self.ax_p = fig.add_axes((0.384, 0.232, 0.244, 0.585), zorder=2)
        ax.set_facecolor(SURFACE)
        ax.set_xlim(-0.7, r.W + 0.7)
        ax.set_ylim(-0.75, r.L + 0.85)
        ax.set_aspect("equal", adjustable="box", anchor="C")
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(False)
        self.room = Rectangle((0, 0), r.W, r.L, fc="#141413", ec=MUTED, lw=1.6, zorder=1)
        ax.add_patch(self.room)
        for i in range(1, int(math.ceil(r.W))):
            ax.plot([i, i], [0, r.L], color=GRIDC, lw=1, zorder=1.5)
        for i in range(1, int(math.ceil(r.L))):
            ax.plot([0, r.W], [i, i], color=GRIDC, lw=1, zorder=1.5)
        for i in range(0, int(r.W) + 1):
            ax.text(i, -0.32, "%d" % i, color=MUTED, fontsize=7.5, ha="center", va="center")
        for i in range(0, int(r.L) + 1):
            ax.text(-0.32, i, "%d" % i, color=MUTED, fontsize=7.5, ha="center", va="center")
        ax.text(r.W + 0.35, -0.32, "m", color=MUTED, fontsize=7.5, ha="center", va="center")
        ax.text(r.W / 2, r.L + 0.42, "12 o'clock wall", color=INK2, fontsize=9, ha="center", va="center")
        ny, nx = len(np.arange(GRID / 2, r.L, GRID)), len(np.arange(GRID / 2, r.W, GRID))
        self.plan_img = ax.imshow(np.zeros((ny, nx, 4)), extent=(0, r.W, 0, r.L), origin="lower", zorder=2,
                                  interpolation="bilinear", visible=False)
        self.p_ray, self.p_ring, self.p_glow, self.p_mark, self.p_tag, self.p_note = [], [], [], [], [], []
        for s in range(MAX_TRACK):
            ray, = ax.plot([], [], color=CAT[s], lw=1.5, alpha=0.75, zorder=3)
            ray.set_clip_path(self.room)
            ring = Circle((0, 0), 1.0, fc="none", ec=CAT[s], lw=2, alpha=0.8, zorder=3, visible=False)
            ax.add_patch(ring)
            ring.set_clip_path(self.room)
            glow, = ax.plot([], [], marker="o", ms=24, color=CAT[s], alpha=0.25, ls="none", zorder=5)
            mark, = ax.plot([], [], marker="o", ms=11, color=CAT[s], mec=SURFACE, mew=2, ls="none", zorder=6)
            tag = ax.text(0, 0, "", color=INK, fontsize=10.5, fontweight="bold", zorder=8, path_effects=halo)
            note = ax.text(0, 0, "", color=INK2, fontsize=8.3, zorder=8, path_effects=halo)
            self.p_ray.append(ray)
            self.p_ring.append(ring)
            self.p_glow.append(glow)
            self.p_mark.append(mark)
            self.p_tag.append(tag)
            self.p_note.append(note)
        self.wedge = Wedge((0, 0), 1.2, 0, 30, fc=INK, ec="none", alpha=0.15, zorder=4)
        ax.add_patch(self.wedge)
        self.wedge.set_clip_path(self.room)
        self.old_pos, = ax.plot([], [], marker="o", ms=8, mfc="none", mec=MUTED, mew=1.5, ls="none", zorder=4)
        self.scanner, = ax.plot([], [], marker="o", ms=12, mfc=SURFACE, mec=INK, mew=2.2, ls="none", zorder=7)
        self.scanner_dot, = ax.plot([], [], marker="o", ms=3.5, color=INK, ls="none", zorder=7.1)
        self.scanner_t = ax.text(0, 0, "scanner", color=INK2, fontsize=8.3, ha="center", va="top", zorder=8,
                                 path_effects=halo)

        # ---------------- device list
        ax, self.dev_title = panel((0.648, 0.215, 0.340, 0.660), "Devices")
        self.ax_d = ax
        self.dev_sub = fig.text(0.648 + 0.340 - 0.009, 0.875 - 0.015, "", color=MUTED, fontsize=8.5, va="top", ha="right")
        self.rows = []
        y0, rh = 0.892, 0.0985
        for i in range(self.ROWS):
            y = y0 - i * rh
            sw, = ax.plot([0.038], [y - 0.002], marker="o", ms=10, color=CAT[0], mec=SURFACE, mew=2, ls="none")
            row = {
                "sw": sw,
                "tag": ax.text(0.070, y, "", color=INK, fontsize=12, fontweight="bold", ha="left", va="center"),
                "name": ax.text(0.170, y, "", color=INK2, fontsize=10, ha="left", va="center"),
                "l2": ax.text(0.070, y - 0.040, "", color=INK2, fontsize=9.2, ha="left", va="center"),
                "r1": ax.text(0.975, y, "", color=INK, fontsize=10.5, fontweight="bold", ha="right", va="center"),
                "r2": ax.text(0.975, y - 0.040, "", color=INK2, fontsize=9.2, ha="right", va="center"),
                "qdot": ax.plot([0.038], [y - 0.040], marker="o", ms=6, color=MUTED, ls="none")[0],
                "meter": [],
            }
            for j in range(3):
                m = Rectangle((0.908 + j * 0.0235, y - 0.013), 0.0185, 0.026, fc=BLUE_TRACK, ec="none")
                ax.add_patch(m)
                row["meter"].append(m)
            row["sep"] = ax.plot([0.025, 0.975], [y - 0.070, y - 0.070], color=GRIDC, lw=1)[0]
            self.rows.append(row)

        # ---------------- band occupancy (nRF A)
        panel((0.012, 0.050, 0.628, 0.155), "2.4 GHz band", "share of time each 1 MHz slice is busy (nRF A)")
        ax = self.ax_s = fig.add_axes((0.046, 0.082, 0.582, 0.078), zorder=2)
        ax.set_facecolor(SURFACE)
        ax.set_xlim(2399.3, 2483.7)
        ax.set_ylim(0, 100)
        ax.set_yticks([0, 50, 100])
        ax.set_yticklabels(["0", "50", "100%"], color=MUTED, fontsize=8)
        ax.set_xticks([2400, 2412, 2437, 2462, 2483])
        ax.set_xticklabels(["2400", "ch 1", "ch 6", "ch 11", "2483 MHz"], color=MUTED, fontsize=8)
        ax.tick_params(length=0)
        ax.grid(axis="y", color=GRIDC, linewidth=1, linestyle="-")
        ax.set_axisbelow(True)
        for k, s in ax.spines.items():
            s.set_visible(k == "bottom")
            s.set_color(AXISC)
        self.watch = Rectangle((2400, 0), 20, 100, fc=INK, ec="none", alpha=0.07, visible=False, zorder=1)
        ax.add_patch(self.watch)
        self.watch_t = ax.text(2400, 96, "", color=INK2, fontsize=8.3, ha="center", va="top", zorder=4)
        self.bars = PathPatch(self._bars_path(np.zeros(84)), fc=CAT[0], ec="none", zorder=3)
        ax.add_patch(self.bars)
        self.spec_note = ax.text(2441.5, 50, "", color=MUTED, fontsize=9.5, ha="center", va="center", zorder=5)

        # ---------------- channel airtime (nRF B)
        ax, self.air_title = panel((0.648, 0.050, 0.340, 0.155), "Channel airtime")
        self.air_sub = fig.text(0.648 + 0.340 - 0.009, 0.205 - 0.015, "", color=MUTED, fontsize=8.5, va="top", ha="right")
        self.air_num = ax.text(0.030, 0.47, "", color=INK, fontsize=25, fontweight="bold", ha="left", va="center")
        self.air_lbl = ax.text(0.030, 0.185, "", color=INK2, fontsize=8.8, ha="left", va="center")
        ax.add_patch(Rectangle((0.030, 0.045), 0.94, 0.06, fc=BLUE_TRACK, ec="none"))
        self.air_fill = Rectangle((0.030, 0.045), 0.0, 0.06, fc=CAT[0], ec="none")
        ax.add_patch(self.air_fill)
        sp = self.ax_a = fig.add_axes((0.648 + 0.340 * 0.36, 0.050 + 0.155 * 0.26, 0.340 * 0.61, 0.155 * 0.42), zorder=2)
        sp.set_facecolor(SURFACE)
        sp.set_xlim(-60, 0)
        sp.set_ylim(0, 100)
        sp.set_xticks([])
        sp.set_yticks([])
        for k, s in sp.spines.items():
            s.set_visible(k == "bottom")
            s.set_color(AXISC)
        self.spark, = sp.plot([], [], color=CAT[0], lw=2, solid_joinstyle="round")
        self.spark_end, = sp.plot([], [], marker="o", ms=7, color=CAT[0], mec=SURFACE, mew=1.5, ls="none")

        # ---------------- footer
        fig.text(0.014, 0.022, "S sweep   SPACE next step   A auto-advance   X cancel   click plan = scanner position   "
                 "1 = phone at 1 m   K = phone is streaming   C clear   R rescan   Q quit",
                 color=MUTED, fontsize=8.6, ha="left", va="center")
        self.hw_t = fig.text(0.988, 0.022, "", color=MUTED, fontsize=8.6, ha="right", va="center")

        # Text sizes were chosen for a 14.2 x 8 inch window; they follow the real window size.
        from matplotlib.text import Text
        self._texts = [(t, t.get_fontsize()) for t in fig.findobj(Text)]
        self._scale = 1.0
        self.on_resize()

        fig.canvas.mpl_connect("key_press_event", self.on_key)
        fig.canvas.mpl_connect("button_press_event", self.on_click)
        fig.canvas.mpl_connect("close_event", self.on_close)
        fig.canvas.mpl_connect("resize_event", self.on_resize)
        self._interval = 200
        self.timer = fig.canvas.new_timer(interval=self._interval)
        self.timer.add_callback(self.on_timer)

    def on_resize(self, _ev=None):
        wi, hi = self.fig.get_size_inches()
        k = max(0.45, min(wi / 14.2, hi / 8.0))
        if abs(k - self._scale) < 0.015:
            return
        self._scale = k
        for t, base in self._texts:
            t.set_fontsize(base * k)
        lim, self._rlim = self._rlim, None        # the radial labels are rebuilt at the new size
        if lim:
            self._set_rlim(*lim)

    @staticmethod
    def _bars_path(heights):
        """All 84 band bars as one shape (one shape draws much faster than 84)."""
        from matplotlib.path import Path
        x0 = 2400 + np.arange(84) - 0.31
        x1 = x0 + 0.62
        h = np.asarray(heights, float)
        v = np.zeros((84 * 5, 2))
        v[0::5, 0], v[1::5, 0], v[2::5, 0], v[3::5, 0], v[4::5, 0] = x0, x0, x1, x1, x0
        v[1::5, 1], v[2::5, 1] = h, h
        codes = np.tile([Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY], 84)
        return Path(v, codes)

    def _set_rlim(self, lo, hi):
        """Radial scale of the direction finder (dBm at the centre and at the rim)."""
        if (lo, hi) == self._rlim:
            return
        self._rlim = (lo, hi)
        ax = self.ax_r
        ax.set_ylim(lo, hi)
        step = 5 if hi - lo <= 30 else 10
        ticks = list(range(int(lo) + step, int(hi), step))
        ax.set_yticks(ticks)
        ax.set_yticklabels([fmt_dbm(t) if t == ticks[-1] else fmt_dbm(t).replace(" dBm", "") for t in ticks],
                           color=MUTED, fontsize=8 * self._scale)
        for t in ax.get_yticklabels():
            t.set_bbox({"facecolor": SURFACE, "edgecolor": "none", "pad": 0.5, "alpha": 0.8})
        span = hi - lo
        self.beam.set_y(lo)
        self.beam.set_height(span)
        for b in self.rim:
            b.set_y(hi - 0.02 * span)
            b.set_height(0.02 * span)

    # ------------------------------------------------------------ events
    def on_key(self, ev):
        k = (ev.key or "").lower()
        r, now = self.r, self.clock()
        if k == "s":
            r.start_sweep(now)
        elif k in (" ", "space", "enter"):
            r.action(now)
        elif k == "a":
            r.auto = not r.auto
            r.toast = ("Auto-advance %s" % ("ON: each step starts by itself" if r.auto else "OFF: press SPACE at each step"), now)
        elif k in ("x", "escape"):
            r.cancel_sweep(now)
        elif k == "c":
            r.clear(now)
        elif k == "r":
            r.rescan(now)
        elif k == "k":
            r.cal_stream(now)
        elif k == "0" and r.state != r.PICK:
            r.cal_reset(now)
        elif k in "123456789" and len(k) == 1:
            if r.state == r.PICK:
                r.choose(int(k) - 1, now)
            elif k == "1":
                r.cal_1m(now)
        elif k == "f":
            try:
                self.fig.canvas.manager.full_screen_toggle()
            except Exception:
                pass
        elif k == "q":
            self.plt.close(self.fig)

    def on_click(self, ev):
        now = self.clock()
        if ev.inaxes is self.ax_p and ev.button == 1 and ev.xdata is not None:
            self.r.set_pos(ev.xdata, ev.ydata)
        elif ev.inaxes is self.ax_b:
            self.r.action(now)

    def on_close(self, _ev):
        try:
            self.timer.stop()
        except Exception:
            pass
        self.src.close()

    def on_timer(self):
        t0 = time.perf_counter()
        try:
            self.step(self.clock())
            self.fig.canvas.draw()
        except Exception:
            traceback.print_exc()
        # Redrawing takes a while on a slow laptop: pace the timer to it so
        # that keys and clicks are still answered promptly.
        want = int(min(700.0, max(200.0, 1500.0 * (time.perf_counter() - t0))))
        if abs(want - self._interval) > 40:
            self._interval = want
            try:
                self.timer.interval = want
            except Exception:
                pass

    def step(self, now):
        r = self.r
        if self.sim is not None:
            self.sim.set_pose(r.pos, r.heading() if r.state in (r.TURN, r.MEASURE) else 0.0)
        for t, line in self.src.poll(now):
            r.feed(line, t)
        r.tick(now)
        self.update(now)

    # ------------------------------------------------------------ drawing
    def _banner(self, now):
        r = self.r
        el = now - r.t_state
        if r.state != r.CONNECT and now - r.last_frame_t > 3.0:
            return "NO DATA", CRIT, "No data from the scanner", "Check the USB cable. %s" % self.src.status, None
        if r.state == r.CONNECT:
            sub = self.src.status
            if " open at " in sub and now - self._t0 > 6.0:
                sub = "The port is open but nothing arrives. Is rf_radar_fw.ino on the board? Press its RST button."
            return "CONNECTING", AXISC, "Connecting to the scanner\u2026", sub, None
        if r.state == r.SURVEY:
            if r.want_ssid and el > 12.0:
                return ("SCANNING", WARN, "Network \u201c%s\u201d not found on 2.4 GHz yet" % r.want_ssid[:24],
                        "Set the hotspot band to 2.4 GHz. The scanner cannot hear 5 GHz. "
                        "%d other networks seen. Press R to pick one from a list." % len(r.aps), None)
            if not r.aps and el > 12.0:
                return ("SCANNING", WARN, "No 2.4 GHz Wi-Fi network heard yet",
                        "Switch the hotspot on and set its band to 2.4 GHz. The scanner cannot hear 5 GHz.", None)
            return ("SCANNING", WARN, "Scanning Wi-Fi channels 1 to 13\u2026",
                    "%d networks found so far" % len(r.aps), min(1.0, el / SURVEY_SECONDS))
        if r.state == r.PICK:
            return ("CHOOSE", AXISC, "Choose the network to watch: press its number",
                    "The list is on the right. Your hotspot must be on the 2.4 GHz band to appear. R = scan again.", None)
        if r.steps == 12:
            clock = "%s (%d\u00b0)" % (clock_name(r.heading()), round(r.heading()))
        else:
            clock = "%d\u00b0" % round(r.heading())
        if r.state == r.TURN:
            sub = "Step %d of %d. Turn the whole board with its reflector, clockwise." % (r.k + 1, r.steps)
            if r.auto:
                left = max(0.0, r.turn_s - el)
                return ("TURN", INK, "Turn the arrow to %s" % clock,
                        sub + " Measuring starts in %d s (SPACE = now)." % math.ceil(left), None)
            return "TURN", INK, "Turn the arrow to %s, then press SPACE" % clock, sub, None
        if r.state == r.MEASURE:
            left = max(0.0, r.dwell - el)
            if el >= r.dwell:
                return ("SWEEPING", WARN, "Sweeping: measuring at %s. Keep the board still" % clock,
                        "Step %d of %d. Waiting for more packets from %s. SPACE = skip to the next step."
                        % (r.k + 1, r.steps, ", ".join(r.starved(now)) or "the phones"), 1.0)
            return ("SWEEPING", WARN, "Sweeping: measuring at %s. Keep the board still" % clock,
                    "Step %d of %d, %d s left. SPACE = skip to the next step." % (r.k + 1, r.steps, math.ceil(left)),
                    min(1.0, el / r.dwell))
        # LIVE
        name = "\u201c%s\u201d" % r.target_name if r.target else "this channel"
        watch = "%s on channel %d" % (name, r.lock_ch) if r.target else "everything on channel %d" % r.lock_ch
        if r.result_t is None:
            n = len(r.tracked)
            if n == 0:
                return ("WATCHING", AXISC, "Watching %s: waiting for devices" % watch,
                        "Use a phone on this network (open a video) so it starts sending.", None)
            return ("READY", AXISC, "Ready. Point the arrow at 12 o'clock and press S to sweep",
                    "Watching %s, %d device%s heard. Click the floor plan where the scanner sits."
                    % (watch, n, "" if n == 1 else "s"), None)
        s = r.summary()
        total = s[Q_LOCKED] + s[Q_WEAK] + s[Q_NONE]
        if total == 0:
            return ("NO RESULT", SERIOUS, "Sweep done, but no device sent enough packets",
                    "Keep the phones busy on %s during the sweep (play a video), then press S again." % name, None)
        parts = []
        if s[Q_LOCKED]:
            parts.append("%d locked" % s[Q_LOCKED])
        if s[Q_WEAK]:
            parts.append("%d weak" % s[Q_WEAK])
        if s[Q_NONE]:
            parts.append("%d distance only" % s[Q_NONE])
        ago = now - r.result_t
        age = "just now" if ago < 10 else ("%d s ago" % ago if ago < 90 else "%d min ago" % (ago / 60))
        located = s[Q_LOCKED] + s[Q_WEAK]
        if located == 0:
            return ("NO DIRECTION", SERIOUS, "Sweep done: strength did not change with direction",
                    "Is the foil reflector behind the board and turning with it? Activity and distance still work. S = sweep again.", None)
        return ("RESULT READY", GOOD, "Result ready: %d device%s located (%s)" % (located, "" if located == 1 else "s", ", ".join(parts)),
                "Measured %s. S = sweep again. For a tighter fix: move the scanner, click its new spot, press S." % age, None)

    def update(self, now):
        r = self.r
        sweeping = r.state in (r.TURN, r.MEASURE)
        phase = (now * 0.9) % 1.0

        # ---- banner
        pill, col, main, sub, prog = self._banner(now)
        toast, tt = r.toast
        if toast and now - tt < 5.0:
            sub = toast
        self.pill.set_facecolor(col)
        self.pill_t.set_text(pill)
        self.pill_t.set_color(INK if col in (AXISC, CRIT) else "#0b0b0b")
        self.main_t.set_text(main)
        self.main_t.set_fontsize((16.5 if len(main) <= 60 else 14.0) * self._scale)
        self.sub_t.set_text(sub)
        self.prog.set_width(0.615 * (prog or 0.0))
        self.prog.set_facecolor(WARN if r.state == r.MEASURE else MUTED)
        show_steps = sweeping or r.result_t is not None
        self.step_lbl.set_visible(show_steps)
        for i, b in enumerate(self.step_boxes):
            b.set_visible(show_steps)
            if sweeping:
                b.set_facecolor(INK if i < r.k else (WARN if i == r.k else AXISC))
            else:
                b.set_facecolor(GOOD if r.result_t is not None else AXISC)
        self.step_lbl.set_text("sweep steps: %d of %d" % (min(r.k + 1, r.steps), r.steps) if sweeping else "sweep complete")

        # ---- direction finder
        step = 2 * math.pi / r.steps
        th = math.radians(r.heading())
        curves = {}
        for mac in r.tracked:
            fx = r.fixes.get(mac, [None])[-1]
            curves[mac] = r.curve(mac) if sweeping else (fx.curve if fx else [])
        allv = [v for c in curves.values() for _, v in c]
        lo, hi = -90.0, -30.0
        if allv:
            lo = 5.0 * math.floor((min(allv) - 8.0) / 5.0)
            hi = 5.0 * math.ceil((max(allv) + 3.0) / 5.0)
            if sweeping and self._was_sweeping:     # only ever widen during a sweep, so the picture does not jump
                lo, hi = min(lo, self._rlim[0]), max(hi, self._rlim[1])
        self._was_sweeping = sweeping
        self._set_rlim(lo, hi)
        span = hi - lo
        if sweeping:
            self.beam.set_x(th - step / 2)
            self.beam.set_alpha(0.10 + (0.12 * (1 - phase) if r.state == r.MEASURE else 0.0))
            self.beam_line.set_data([th, th], [lo, hi])
            self.beam_line.set_alpha(0.9)
        else:
            self.beam.set_alpha(0.0)
            self.beam_line.set_alpha(0.0)
        for i, b in enumerate(self.rim):
            if sweeping:
                b.set_facecolor(INK2 if i < r.k else (WARN if i == r.k else AXISC))
            else:
                b.set_facecolor(AXISC)
        used = set()
        for mac in r.tracked:
            d = r.devs[mac]
            s = d.slot
            used.add(s)
            fx = r.fixes.get(mac, [None])[-1]
            curve = curves[mac]
            if curve:
                hh = [math.radians(h) for h, _ in curve]
                vv = [min(hi - 0.03 * span, max(lo + 0.02 * span, v)) for _, v in curve]
                if len(curve) == r.steps:
                    hh.append(hh[0])
                    vv.append(vv[0])
                self.r_curve[s].set_data(hh, vv)
            else:
                self.r_curve[s].set_data([], [])
            if fx and fx.theta is not None and fx.quality != Q_NONE and not sweeping:
                t = math.radians(fx.theta)
                pk = min(hi - 0.04 * span, max(lo + 0.05 * span, fx.peak))
                self.r_ray[s].set_data([t, t], [lo, pk])
                self.r_blip[s].set_data([t], [pk])
                out = pk + 0.11 * span
                self.r_lbl[s].set_position((t, out) if out <= hi - 0.05 * span else (t + 0.20, pk - 0.04 * span))
                self.r_lbl[s].set_text(d.tag)
            else:
                self.r_ray[s].set_data([], [])
                self.r_blip[s].set_data([], [])
                self.r_lbl[s].set_text("")
        for s in range(MAX_TRACK):
            if s not in used:
                self.r_curve[s].set_data([], [])
                self.r_ray[s].set_data([], [])
                self.r_blip[s].set_data([], [])
                self.r_lbl[s].set_text("")
        self.r_hint.set_text("" if (sweeping or r.fixes) else "The curves appear here\nduring a sweep")

        # ---- floor plan
        if self._solved is not r.est:
            self._solved = r.est
            from matplotlib.colors import to_rgb
            rgb = None                      # all blobs blended into one picture (one image is fast to draw)
            for mac, e in r.est.items():
                s = r.devs[mac].slot
                if s is None or "img" not in e:
                    continue
                a = 0.55 * np.clip(e["img"], 0, 1) ** 0.8
                if rgb is None:
                    rgb = np.zeros(a.shape + (3,))
                    alpha = np.zeros(a.shape)
                col = np.array(to_rgb(CAT[s]))
                out_a = a + alpha * (1 - a)
                rgb = (col * a[..., None] + rgb * (alpha * (1 - a))[..., None]) / np.maximum(out_a, 1e-9)[..., None]
                alpha = out_a
            if rgb is None:
                self.plan_img.set_visible(False)
            else:
                self.plan_img.set_data(np.dstack([rgb, alpha]))
                self.plan_img.set_visible(True)
        sx, sy = r.pos
        self.scanner.set_data([sx], [sy])
        self.scanner_dot.set_data([sx], [sy])
        self.scanner_t.set_position((sx, sy - 0.30))
        hw = 180.0 / r.steps
        hd = r.heading() if sweeping else 0.0
        self.wedge.set_center((sx, sy))
        self.wedge.set_theta1(90.0 - hd - hw)
        self.wedge.set_theta2(90.0 - hd + hw)
        if r.state == r.MEASURE:
            self.wedge.set_radius(0.7 + 2.6 * phase)
            self.wedge.set_alpha(0.06 + 0.20 * (1 - phase))
        else:
            self.wedge.set_radius(1.3)
            self.wedge.set_alpha(0.16 if (sweeping or r.result_t is None) else 0.0)
        old = [p for p in r.sweep_positions if math.hypot(p[0] - sx, p[1] - sy) > 0.3]
        self.old_pos.set_data([p[0] for p in old], [p[1] for p in old])
        self.scanner_t.set_visible(not any("x" in e and abs(e["x"] - sx) < 1.3 and -1.2 < e["y"] - sy < 0.5
                                           for e in r.est.values()))
        used = set()
        for mac in r.tracked:
            d = r.devs[mac]
            s = d.slot
            e = r.est.get(mac)
            if e is None:
                continue
            used.add(s)
            label, level, _ = r.activity(d, now)
            if e["q"] == Q_NONE:
                cx, cy, rad = e["ring"]
                self.p_ring[s].set_center((cx, cy))
                self.p_ring[s].set_radius(max(0.2, rad))
                self.p_ring[s].set_visible(True)
                self.p_mark[s].set_data([], [])
                self.p_glow[s].set_data([], [])
                self.p_ray[s].set_data([], [])
                # put the tag on the part of the ring that is inside the room, a different spot per device
                spot = None
                for j in range(24):
                    a = math.radians(35.0 + 45.0 * s + 15.0 * j)
                    px, py = cx + rad * math.sin(a), cy + rad * math.cos(a)
                    if 0.4 <= px <= r.W - 0.4 and 0.4 <= py <= r.L - 0.4:
                        spot = (px, py)
                        break
                self.p_tag[s].set_position(spot or (cx, cy + 0.5))
                self.p_tag[s].set_ha("center")
                self.p_tag[s].set_va("center")
                self.p_tag[s].set_text(d.tag if spot else "")
                self.p_note[s].set_text("")
                continue
            x, y = e["x"], e["y"]
            self.p_ring[s].set_visible(False)
            self.p_mark[s].set_data([x], [y])
            self.p_mark[s].set_marker("s" if d.is_ap else "o")
            self.p_mark[s].set_markersize(10.5 if d.is_ap else 10 + 1.6 * level)
            if level >= 2 and not d.is_ap:
                self.p_glow[s].set_data([x], [y])
                self.p_glow[s].set_markersize(20 + 16 * phase)
                self.p_glow[s].set_alpha(0.30 * (1 - phase))
            else:
                self.p_glow[s].set_data([], [])
            rx, ry = [], []
            for f in r.fixes.get(mac, []):
                if f.theta is not None and f.quality != Q_NONE:
                    t = math.radians(f.theta)
                    rx += [f.x, f.x + 30 * math.sin(t), float("nan")]
                    ry += [f.y, f.y + 30 * math.cos(t), float("nan")]
            self.p_ray[s].set_data(rx, ry)
            right = x > r.W - 1.6
            up = y < r.L - 0.9
            self.p_tag[s].set_position((x + (-0.24 if right else 0.24), y + (0.20 if up else -0.20)))
            self.p_tag[s].set_ha("right" if right else "left")
            self.p_tag[s].set_va("bottom" if up else "top")
            self.p_tag[s].set_text(d.tag)
            self.p_note[s].set_position((x + (-0.24 if right else 0.24), y + (0.16 if up else -0.62)))
            self.p_note[s].set_ha("right" if right else "left")
            self.p_note[s].set_va("top")
            self.p_note[s].set_text("hotspot" if d.is_ap else label.lower())
        for s in range(MAX_TRACK):
            if s not in used:
                self.p_ring[s].set_visible(False)
                self.p_mark[s].set_data([], [])
                self.p_glow[s].set_data([], [])
                self.p_ray[s].set_data([], [])
                self.p_tag[s].set_text("")
                self.p_note[s].set_text("")

        # ---- list on the right
        rows = []
        if r.state in (r.CONNECT, r.SURVEY, r.PICK):
            self.dev_title.set_text("Networks found" + (" (simulated)" if self.sim is not None else ""))
            self.dev_sub.set_text("2.4 GHz only")
            if r.state == r.PICK:
                order = r.pick
            else:
                order = [b for b, _ in sorted(r.aps.items(), key=lambda kv: -kv[1]["rssi"])[:self.ROWS]]
            for i, b in enumerate(order):
                ap = r.aps[b]
                n = r.net_frames.get(b, 0)
                name = ap["ssid"] or "(hidden name)"
                rows.append({"tag": "%d" % (i + 1) if r.state == r.PICK else "", "name": name if len(name) <= 24 else name[:23] + "\u2026",
                             "l2": "clients are active" if n >= 10 else "no client traffic seen",
                             "r1": "ch %d" % ap["ch"], "r2": fmt_dbm(ap["rssi"]), "color": None, "name_x": 0.115 if r.state == r.PICK else 0.070})
        else:
            self.dev_title.set_text("Devices" + (" (simulated)" if self.sim is not None else ""))
            self.dev_sub.set_text("on \u201c%s\u201d, channel %d" % (r.target_name, r.lock_ch) if r.target else "channel %d" % r.lock_ch)
            for mac in r.tracked:
                d = r.devs[mac]
                label, level, rate = r.activity(d, now)
                e = r.est.get(mac)
                ago = now - d.last_tx
                bits = [fmt_dbm(d.rssi) if ago < 20.0 else "%s %d s ago" % (fmt_dbm(d.rssi), ago)]
                shared = [] if sweeping else r.shares_with(d, now)
                qcol = None
                if e:
                    f = e["fix"]
                    if e["q"] != Q_NONE:
                        bits.append("%s (%d\u00b0)" % (clock_name(f.theta), round(f.theta)))
                        bits.append("%.1f m" % e["dist"])
                        bits.append("%s \u00b1%.1f m" % (Q_NAME[e["q"]], e["spread"]))
                        qcol = GOOD if e["q"] == Q_LOCKED else WARN
                    else:
                        bits.append("about %.1f m" % e["dist"])
                        bits.append("direction unknown")
                        qcol = MUTED
                if shared:
                    bits = [b for b in bits if "LOCKED" not in b and "weak fix" not in b and "unknown" not in b]
                ssid = r.aps.get(mac, {}).get("ssid", "")
                rows.append({"tag": d.tag, "name": (("\u201c%s\u201d hotspot" % ssid[:20]) if ssid else "hotspot") if d.is_ap
                             else "phone %s" % mac_tail(mac),
                             "l2": "  \u00b7  ".join(bits), "r1": "HOTSPOT" if d.is_ap else label + (" ?" if shared else ""),
                             "r2": ("may be %s's traffic" % "'s or ".join(shared[:2])) if shared else fmt_rate(rate),
                             "color": CAT[d.slot], "level": None if d.is_ap else level,
                             "square": d.is_ap, "qcol": qcol, "name_x": 0.170})
        for i, row in enumerate(self.rows):
            v = rows[i] if i < len(rows) else None
            row["sep"].set_visible(i < len(rows) and i < self.ROWS - 1)
            if v is None:
                for k in ("tag", "name", "l2", "r1", "r2"):
                    row[k].set_text("")
                row["sw"].set_visible(False)
                row["qdot"].set_visible(False)
                for m in row["meter"]:
                    m.set_visible(False)
                continue
            row["tag"].set_text(v["tag"])
            row["name"].set_text(v["name"])
            row["name"].set_x(v["name_x"])
            row["l2"].set_text(v["l2"])
            row["l2"].set_x(0.070 if v.get("qcol") else (0.070 if v["color"] else v["name_x"]))
            row["r1"].set_text(v["r1"])
            row["r2"].set_text(v["r2"])
            row["r1"].set_x(0.895 if v.get("level") is not None else 0.975)
            row["sw"].set_visible(v["color"] is not None)
            if v["color"]:
                row["sw"].set_color(v["color"])
                row["sw"].set_marker("s" if v.get("square") else "o")
            row["qdot"].set_visible(bool(v.get("qcol")))
            if v.get("qcol"):
                row["qdot"].set_color(v["qcol"])
            for j, m in enumerate(row["meter"]):
                m.set_visible(v.get("level") is not None)
                if v.get("level") is not None:
                    m.set_facecolor(CAT[0] if j < v["level"] else BLUE_TRACK)

        # ---- band occupancy
        a_ok = bool(r.hw and r.hw["a"][0] != 255)
        fresh = r.spectrum is not None and now - r.spec_t < 4.0
        if fresh:
            self.bars.set_path(self._bars_path(100.0 * r.spectrum))
            self.spec_note.set_text("")
        else:
            self.bars.set_path(self._bars_path(np.zeros(84)))
            self.spec_note.set_text("" if (r.hw is None or a_ok) else "nRF A not found: no band view (see the boot message)")
        if r.lock_ch:
            c = 2407 + 5 * r.lock_ch
            self.watch.set_x(c - 10)
            self.watch.set_visible(True)
            self.watch_t.set_position((c, 97))
            self.watch_t.set_text("watching ch %d" % r.lock_ch)
        else:
            self.watch.set_visible(False)
            self.watch_t.set_text("")

        # ---- airtime
        if r.lock_ch and r.airtime is not None:
            self.air_num.set_text("%d%%" % round(100 * r.airtime))
            self.air_lbl.set_text("of the time, channel %d is on the air" % r.lock_ch)
            self.air_sub.set_text("measured by %s, last 60 s" % r.air_src)
            self.air_fill.set_width(0.94 * min(1.0, r.airtime))
            xs = [t - now for t, _ in r.air_hist if now - t <= 60.0]
            ys = [100 * v for t, v in r.air_hist if now - t <= 60.0]
            self.spark.set_data(xs, ys)
            self.spark_end.set_data(xs[-1:], ys[-1:])
        else:
            self.air_num.set_text("\u2013")
            b_ok = bool(r.hw and r.hw["b"][0] != 255)
            self.air_lbl.set_text("starts when a network is being watched" if (r.hw is None or a_ok or b_ok)
                                  else "no nRF module found: airtime is off")
            self.air_sub.set_text("")
            self.air_fill.set_width(0.0)
            self.spark.set_data([], [])
            self.spark_end.set_data([], [])

        # ---- footer: what the firmware found on its pins
        if self.sim is not None:
            self.hw_t.set_text("DEMO MODE: simulated data, no scanner connected")
            self.hw_t.set_color(INK)
        elif r.hw:
            def pins(p):
                names = {16: "D0", 5: "D1", 4: "D2", 0: "D3", 2: "D4", 15: "D8", 254: "3V3", 255: "?"}
                return "missing" if p[0] == 255 else "%s/%s" % (names.get(p[0], "?"), names.get(p[1], "?"))
            self.hw_t.set_text("nRF A %s   nRF B %s   (CSN/CE)" % (pins(r.hw["a"]), pins(r.hw["b"])))

    def run(self):
        try:                                       # start maximised where the window system allows it
            w = self.fig.canvas.manager.window
            if hasattr(w, "showMaximized"):
                w.showMaximized()
            elif hasattr(w, "state"):
                try:
                    w.state("zoomed")
                except Exception:
                    w.attributes("-zoomed", True)
        except Exception:
            pass
        self.timer.start()
        self.plt.show()


# =============================================================================

def parse_room(s):
    for sep in ("x", "X", ",", "*"):
        if sep in s:
            a, b = s.split(sep, 1)
            w, l = float(a), float(b)
            if 1.0 <= w <= 60.0 and 1.0 <= l <= 60.0:
                return w, l
    raise argparse.ArgumentTypeError("room must look like 5.5x8.5 (width x length in metres)")


def main():
    ap = argparse.ArgumentParser(description="RF Radar: locate Wi-Fi devices in a room with the NodeMCU + nRF24 scanner")
    ap.add_argument("--port", help="serial port of the NodeMCU, e.g. COM5 or /dev/ttyUSB0 (found automatically if omitted)")
    ap.add_argument("--demo", action="store_true", help="no hardware: run with a simulated room")
    ap.add_argument("--ssid", help="name of the 2.4 GHz network to watch (your hotspot)")
    ap.add_argument("--channel", type=int, help="watch every device on this Wi-Fi channel instead of one network")
    ap.add_argument("--room", type=parse_room, default=(5.5, 8.5), help="room width x length in metres (default 5.5x8.5)")
    ap.add_argument("--pos", help="where the scanner sits, e.g. 2.7,4.2 (or click the floor plan)")
    ap.add_argument("--steps", type=int, default=STEPS_DEFAULT, choices=(8, 12, 24), help="headings per turn (default 12 = clock positions)")
    ap.add_argument("--dwell", type=float, default=DWELL_DEFAULT, help="seconds measured at each heading (default 8)")
    ap.add_argument("--auto", action="store_true", help="advance through the steps on a timer instead of SPACE")
    ap.add_argument("--rssi1m", type=float, help="dBm of a phone 1 m in front of the arrow (or press 1 to calibrate)")
    ap.add_argument("--pathn", type=float, help="path-loss exponent (default 2.2)")
    ap.add_argument("--record", help="save every line from the scanner to this file (for troubleshooting)")
    ap.add_argument("--speed", type=float, default=1.0, help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.channel is not None and not 1 <= a.channel <= 13:
        ap.error("--channel must be 1 to 13")

    here = os.path.dirname(os.path.abspath(__file__))
    sim = None
    if a.demo:
        sim = Simulator(a.room)
        src = DemoSource(sim)
        speed = max(0.1, a.speed)
        clock = lambda: time.monotonic() * speed
    else:
        import importlib.util
        if importlib.util.find_spec("serial") is None:
            print("pyserial is missing. Install it with:   pip install pyserial")
            sys.exit(1)
        src = SerialSource(a.port or find_port(), a.record)
        clock = time.monotonic

    radar = Radar(a.room, steps=a.steps, dwell=a.dwell, auto=a.auto or a.demo,
                  ssid=a.ssid or ("DemoSpot" if a.demo and not a.channel else None), channel=a.channel,
                  send=src.send, settings_path=None if a.demo else os.path.join(here, "rf_radar_settings.json"))
    if a.rssi1m is not None:
        radar.rssi1m = a.rssi1m
    if a.pathn is not None:
        radar.path_n = a.pathn
    if a.pos:
        try:
            x, y = (float(v) for v in a.pos.replace("x", ",").split(","))
            radar.pos = (min(max(x, 0.1), a.room[0] - 0.1), min(max(y, 0.1), a.room[1] - 0.1))
        except ValueError:
            ap.error("--pos must look like 2.7,4.2")
    view = View(radar, src, clock, sim)
    if a.demo:
        print("Demo: simulated room. Press S to sweep; the simulated board turns by itself.")
    view.run()


if __name__ == "__main__":
    main()
