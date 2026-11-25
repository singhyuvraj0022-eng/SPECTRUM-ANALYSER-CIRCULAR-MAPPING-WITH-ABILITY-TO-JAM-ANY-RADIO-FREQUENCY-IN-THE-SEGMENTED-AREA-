# RF Radar — the matched pair

These two files are one project and are meant to be used **together**. One runs on the
scanner hardware; the other runs on your laptop and draws the picture.

| File | Runs on | What it does |
|------|---------|--------------|
| [`rf_radar_fw.ino`](rf_radar_fw.ino) | NodeMCU (ESP8266) + 2 × nRF24L01+ | Listen-only 2.4 GHz scanner. Streams signal-strength and band-occupancy measurements over USB at 115200 baud. |
| [`rf_radar.py`](rf_radar.py) | Your laptop | Reads the scanner over USB and turns the measurements into a live radar display that locates Wi-Fi devices in a room. |

The firmware only measures and reports — the laptop side does all the radar maths.

## Download both

- **Whole folder / repo:** click the green **Code → Download ZIP** button on the repo home page; this folder comes with it.
- **Individual files:** open each file above and use the **Download raw file** button (top-right of the file view).
- **Git:** `git clone` the repo, then everything is in `rf_radar/`.

## Quick start

1. Flash `rf_radar_fw.ino` to the NodeMCU (board: *NodeMCU 1.0 (ESP-12E Module)*, ESP8266 core 2.7+). No extra libraries needed.
2. On the laptop, install the Python dependencies once:
   ```
   pip install pyserial matplotlib numpy
   ```
3. Plug in the board over USB and run:
   ```
   python rf_radar.py --ssid "MyHotspot" --room 5.5x8.5
   ```
   Or try it with no hardware:
   ```
   python rf_radar.py --demo
   ```

See the comment block at the top of each file for full wiring, setup, and on-screen key details.
