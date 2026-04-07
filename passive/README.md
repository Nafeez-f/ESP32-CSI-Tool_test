# Passive CSI Collection — ISAC Mode

Passively sniffs Wi-Fi frames on a selected 2.4 GHz channel **without
connecting to any network**.  Every received frame produces a CSI sample
with automatic traffic classification (`comm_class`), giving you
continuous CSI at all times — during communication AND during idle periods.

## How it works

The ESP32 captures every frame on the channel and labels each one:

| `comm_class` | Meaning | Typical source |
|---|---|---|
| `video` | TID 4/5 or sig_len > 800 | YouTube, Netflix, video calls |
| `voice` | TID 6/7 | VoIP, FaceTime audio |
| `browsing` | TID 0/3 + sig_len > 200 | Web pages loading |
| `background` | TID 1/2 | Cloud sync, OS updates |
| `idle` | sig_len <= 100 (data frames) | Keepalives, null data frames |
| `data` | data frames, unclassified | Unclassified best-effort |
| `mgmt` | management frames | Beacons (~10/sec per AP), probes |

You get **continuous CSI** from management frames (beacons) even when no
data is flowing.  The `comm_class` column tells you exactly which frames
are communication traffic vs background — no manual labelling needed.

## Important setup notes

### iPhone hotspot: enable "Maximize Compatibility"

The ESP32 only supports 2.4 GHz.  Modern iPhones may default to 5 GHz
for Personal Hotspot.  On your iPhone, go to **Settings > Personal Hotspot**
and enable **"Maximize Compatibility"** to force 2.4 GHz.

### HT40 bandwidth is critical

Modern hotspots (including iPhone) use **HT40** (40 MHz channels) for data.
If the ESP32 is in HT20 mode, it will **only see legacy-rate frames**
(beacons, keepalives) — all the actual video/browsing data will be invisible.

This firmware defaults to **HT40-above** (`WIFI_SECOND_CHAN_ABOVE`).  If you
still see no HT frames, try:
```
BANDWIDTH: 40below
```
The `SCAN` command now **automatically tests HT20, HT40-above, and HT40-below**
for every channel and picks the best combination.

### How to tell if HT40 is working

Type `LISTMACS` and look at the **HT** column.  If all zeros, you are only
seeing legacy frames.  If non-zero, you are capturing the real HT/VHT traffic.

### MAC addresses: use `airport -I`, not `arp -a`

iPhones use **MAC randomization** — the MAC in the ARP table can differ from
the 802.11 BSSID in over-the-air frames.  On your MacBook:

```bash
# Correct way: shows the actual BSSID as seen in Wi-Fi frames
/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I

# Also get your MacBook Wi-Fi MAC
ifconfig en0 | grep ether
```

Or just use `LISTMACS` on the ESP32 to discover all MACs empirically.

## Quick-start: MacBook + iPhone hotspot

### 1. Prepare iPhone

Settings > Personal Hotspot > **Maximize Compatibility** = ON

### 2. Connect MacBook to the hotspot

### 3. Flash and run

```bash
cd passive
idf.py menuconfig   # Set WiFi Channel if you know it
idf.py flash monitor | python ../python_utils/serial_append_time.py > experiment.csv
```

### 4. Find the right channel and bandwidth (in the monitor)

```
SCAN
```

This now tests HT20 + HT40-above + HT40-below on every channel and picks
the combination with the most frames.  It automatically sets the channel
and bandwidth.

### 5. Identify your hotspot

```
LISTMACS
```

Look for the MAC with:
- Strong **RSSI** (close to your ESP32, e.g. -25 to -40 dBm)
- Non-zero **HT** column (capturing HT/VHT data frames)
- Both **Data** and **Mgmt** counts (beacons + data)

Then filter to it:
```
WATCHMAC: DA:80:83:E3:4A:00
WATCHMAC: CA:D5:BB:7F:0B:39
```

### 6. Generate traffic and observe

Play YouTube on the MacBook.  You should now see:
- `video` — YouTube data frames (large sig_len, TID 4/5)
- `mgmt` — beacons from hotspot (continuous, even during idle)
- `idle` — keepalive null-data frames from MacBook

### 7. Analyse

```bash
python python_utils/isac_filter.py experiment.csv --listmacs
python python_utils/isac_filter.py experiment.csv --analyze
python python_utils/isac_filter.py experiment.csv --timeline
```

## Runtime commands

| Command | Effect |
|---|---|
| `SCAN` | Sweep channels 1-13 with HT20/HT40 modes, auto-select best |
| `CHANNEL: <n>` | Switch to channel n (1-13), preserves bandwidth |
| `BANDWIDTH: 20\|40above\|40below` | Set bandwidth mode |
| `WATCHMAC: <mac>` | Add a MAC to the filter list (up to 4) |
| `CLEARMAC` | Remove all MAC filters |
| `LISTMACS` | Show all MACs with frame counts, HT status, RSSI |
| `TAG: <label>` | Label subsequent rows (e.g. `person_present`) |
| `SHOWMGMT` | Include management frame CSI (default) |
| `HIDEMGMT` | Suppress management frame CSI |
| `SETTIME: <unix>` | Set the real-time clock |

## Troubleshooting

**"I only see `idle` / `data` — no `video` even during YouTube"**

This almost always means you are missing the HT40 data frames:

1. Run `LISTMACS` and check the **HT** column.  If all zeros:
   - Try `BANDWIDTH: 40above` or `BANDWIDTH: 40below`
   - Or run `SCAN` which tests all modes automatically
2. Verify you are on the **correct channel** (use `SCAN` or `airport -I`)
3. Verify the iPhone hotspot is on **2.4 GHz** ("Maximize Compatibility" ON)

**"I see DA:80:83:E3:4A:00 but not the MAC from `arp -a`"**

Normal.  iPhone uses MAC randomization — the ARP MAC differs from the
802.11 BSSID.  Use `airport -I` or `LISTMACS` to find the real BSSID.

**"I see many MACs I don't recognize"**

These are neighbours' devices.  Use `WATCHMAC:` to filter to just your
hotspot BSSID and laptop MAC.

## CSV column layout

See `isac_filter.py` header for the full 35-column layout.  Key columns:

| Index | Column | Use |
|---|---|---|
| 2 | `mac` | Sender MAC |
| 3 | `rssi` | Signal strength |
| 5 | `sig_mode` | 0=legacy, 1=HT, 2=VHT (should be 1+ for real data) |
| 20 | `sig_len` | Frame size — larger during video streaming |
| 27 | `to_ds` | 1 = uplink (device -> router) |
| 28 | `from_ds` | 1 = downlink (router -> device) |
| 29 | `tid` | QoS Traffic ID (4/5 = Video) |
| 32 | `comm_class` | Automatic traffic classification |
| 33 | `env_label` | Optional physical environment label (TAG:) |
| 34 | `CSI_DATA` | Raw CSI complex values `[im re im re ...]` |
