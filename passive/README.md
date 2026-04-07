# Passive CSI Collection — ISAC Mode

Passively sniffs Wi-Fi frames on a selected 2.4 GHz channel **without
connecting to any network**.  Every received data frame produces a CSI
sample with automatic traffic classification (`comm_class`).

## The problem this solves

In promiscuous mode the ESP32 sees every frame on the channel — beacons
every 100 ms from every AP, probe requests from phones, neighbour traffic,
AND the actual communication (YouTube, browsing) you care about.  Without
filtering, the useful CSI is buried under management-frame noise that all
looks "idle".

This firmware **suppresses management frames by default** and automatically
classifies each remaining data frame by traffic type using QoS TID and
frame size:

| `comm_class` | Meaning | Typical source |
|---|---|---|
| `video` | TID 4/5 or sig_len > 800 | YouTube, Netflix, video calls |
| `voice` | TID 6/7 | VoIP, FaceTime audio |
| `browsing` | TID 0/3 + sig_len > 200 | Web pages loading |
| `background` | TID 1/2 | Cloud sync, OS updates |
| `idle` | sig_len ≤ 100 | Keepalives, null data frames |
| `data` | everything else | Unclassified best-effort |
| `mgmt` | management frames | Beacons, probes (suppressed by default) |

## Quick-start: MacBook + iPhone hotspot

1. **Connect** your MacBook to your iPhone's Wi-Fi hotspot.

2. **Find the hotspot channel and MACs**.

   On the MacBook:
   ```bash
   # Hotspot BSSID and channel
   /System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I

   # Your MacBook Wi-Fi MAC (en0)
   ifconfig en0 | grep ether
   ```

3. **Configure and flash** the ESP32:
   ```bash
   cd passive
   idf.py menuconfig
   ```
   Under **ESP32 CSI Tool Config**:
   - Set **WiFi Channel** to match the hotspot channel.
   - Under **ISAC MAC Filter**, enter the iPhone BSSID and MacBook MAC.

   Then:
   ```bash
   idf.py flash monitor | python ../python_utils/serial_append_time.py > experiment.csv
   ```

4. **Alternatively, configure at runtime** (no reflash needed):

   In the monitor terminal type:
   ```
   SCAN
   ```
   This sweeps channels 1–13 and reports frame counts — the channel with
   the most frames is your hotspot's channel.  Then:
   ```
   CHANNEL: 6
   WATCHMAC: AA:BB:CC:DD:EE:FF
   WATCHMAC: 11:22:33:44:55:66
   ```

5. **Generate traffic**: Play a YouTube video on the MacBook, browse the
   web, or leave it idle.  The `comm_class` column updates automatically —
   no commands needed.

6. **Analyse** the collected CSV:
   ```bash
   python python_utils/isac_filter.py experiment.csv
   python python_utils/isac_filter.py experiment.csv --analyze
   python python_utils/isac_filter.py experiment.csv --timeline
   ```

## Runtime commands

Type these into `idf.py monitor` while the ESP32 is running:

| Command | Effect |
|---|---|
| `SCAN` | Sweep channels 1–13, find your hotspot's channel |
| `CHANNEL: <n>` | Switch to channel n (1–13) |
| `BANDWIDTH: 20\|40above\|40below` | Set HT20 or HT40 mode (default: 40above) |
| `WATCHMAC: <mac>` | Add a MAC to the filter list (up to 4) |
| `CLEARMAC` | Remove all MAC filters |
| `TAG: <label>` | Label subsequent rows (e.g. `person_present`) |
| `SHOWMGMT` | Include management frame CSI in output |
| `HIDEMGMT` | Suppress management frame CSI (default) |
| `SETTIME: <unix>` | Set the real-time clock |

## Troubleshooting

**"I only see `idle` or `data` — no `video` even during YouTube"**

- Make sure you are on the **correct channel** (use `SCAN`).
- Make sure **HT40** is enabled (`BANDWIDTH: 40above` or `40below`).
  Modern hotspots use 40 MHz channels; HT20 misses those frames entirely.
- Use `WATCHMAC:` with the hotspot BSSID.  Without it, neighbour traffic
  dilutes the output.
- If the hotspot MAC still does not appear, try `BANDWIDTH: 40below`.

**"I see too many MACs / too much noise"**

- Use `WATCHMAC:` to filter to just the two MACs you care about (hotspot +
  your device).
- Management frames are already suppressed by default.  If you previously
  ran `SHOWMGMT`, run `HIDEMGMT` to re-suppress.

## CSV column layout

See `isac_filter.py` header for the full 35-column layout.  Key columns:

| Index | Column | Use |
|---|---|---|
| 2 | `mac` | Sender MAC — identifies device vs router |
| 3 | `rssi` | Signal strength |
| 20 | `sig_len` | Frame size — larger during video streaming |
| 27 | `to_ds` | 1 = uplink (device → router) |
| 28 | `from_ds` | 1 = downlink (router → device) |
| 29 | `tid` | QoS Traffic ID (4/5 = Video) |
| 32 | `comm_class` | Automatic traffic classification |
| 33 | `env_label` | Optional physical environment label (TAG:) |
| 34 | `CSI_DATA` | Raw CSI complex values `[im re im re ...]` |
