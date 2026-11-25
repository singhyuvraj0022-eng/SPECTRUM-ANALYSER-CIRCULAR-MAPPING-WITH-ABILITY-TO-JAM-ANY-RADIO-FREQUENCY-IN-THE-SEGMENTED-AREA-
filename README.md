# SPECTRUM-ANALYSER-CIRCULAR-MAPPING-WITH-ABILITY-TO-JAM-ANY-RADIO-FREQUENCY-IN-THE-SEGMENTED-AREA-

## RF Radar (main project)

The core scanner lives in [`rf_radar/`](rf_radar/) as a matched pair:

- [`rf_radar/rf_radar_fw.ino`](rf_radar/rf_radar_fw.ino) — NodeMCU firmware (the scanner)
- [`rf_radar/rf_radar.py`](rf_radar/rf_radar.py) — laptop display

See [`rf_radar/README.md`](rf_radar/README.md) for setup and how to download both together.

## Other sketches

- `2.4_ghz_channel_manager.ino`
- `_rfspectrum_monitor.ino`
- `rf_spectrum_final.ino`
