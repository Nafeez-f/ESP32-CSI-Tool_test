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
| `idle` | sig_len ≤ 100 (data frames) | Keepalives, null data frames |
| `data` | data frames, unclassified | Unclassified best-effort |
| `mgmt` | management frames | Beacons (~10/sec per AP), probes |

You get **continuous CSI** from management frames (beacons) even when no
data is flowing.  The `comm_class` column tells you exactly which frames
are communication traffic vs background — no manual labelling needed.

## CRITICAL: iPhone hotspot must use 2.4 GHz

**The ESP32 only supports 2.4 GHz.**  Modern iPhones default to 5 GHz for
Personal Hotspot.  If your hotspot is on 5 GHz, the ESP32 will see beacons
and neighbour traffic but **none of your actual data traffic**.

**Fix:** On your iPhone, go to **Settings > Personal Hotspot** and enable
**"Maximize Compatibility"**.  This forces the hotspot to 2.4 GHz.

You will know it worked when you see the hotspot BSSID appearing in the
CSI output with `comm_class=video` during YouTube playback.

## Quick-start: MacBook + iPhone hotspot

### 1. Force 2.4 GHz on iPhone

Settings > Personal Hotspot > **Maximize Compatibility** = ON

### 2. Connect your MacBook to the hotspot

### 3. Find your MACs

On the MacBook, run `airport -I` to get the hotspot BSSID and channel:

```bash
/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I
```

Look for:
- **BSSID** — this is your iPhone hotspot's MAC (e.g. `da:80:83:e3:4a:00`)
- **channel** — the Wi-Fi channel number (e.g. `6`)

**Important:** Do NOT use `arp -a` for the BSSID.  The ARP table shows the
IP-layer MAC which can differ from the 802.11 BSSID (iPhones use MAC
randomization).  The `airport -I` BSSID is the one that appears in CSI rows.

Your MacBook's Wi-Fi MAC:
```bash
ifconfig en0 | grep ether
```

### 4. Flash and run

```bash
cd passive
idf.py menuconfig   # Set WiFi Channel, optionally set ISAC MAC filter
idf.py flash monitor | python ../python_utils/serial_append_time.py > experiment.csv
```

### 5. Identify MACs at runtime (no reflash needed)

If you are not sure which MAC is your hotspot, let the ESP32 collect for
30 seconds on the right channel, then type:

```
LISTMACS
```

This prints a table of every MAC seen, with data/mgmt frame counts, RSSI,
and direction.  Your hotspot BSSID will typically have:
- High **data** and **mgmt** counts
- Strong **RSSI** (e.g. -25 to -40 dBm if nearby)
- **Direction** = "AP/mgmt" for its beacons, "downlink" for data

Then lock onto it:
```
WATCHMAC: DA:80:83:E3:4A:00
WATCHMAC: CA:D5:BB:7F:0B:39
```

### 6. Generate traffic and observe

Play a YouTube video on the MacBook.  The `comm_class` column will show:
- `video` — frames carrying YouTube data (large, TID 4/5)
- `browsing` — web page loads
- `idle` — keepalive null-data frames
- `mgmt` — beacons (continuous, even during idle periods)

### 7. Analyse

```bash
python python_utils/isac_filter.py experiment.csv
python python_utils/isac_filter.py experiment.csv --analyze
python python_utils/isac_filter.py experiment.csv --timeline
python python_utils/isac_filter.py experiment.csv --listmacs
```

## Runtime commands

Type these into `idf.py monitor` while the ESP32 is running:

| Command | Effect |
|---|---|
| `SCAN` | Sweep channels 1-13, find your hotspot's channel |
| `CHANNEL: <n>` | Switch to channel n (1-13) |
| `BANDWIDTH: 20\|40above\|40below` | Set HT20 or HT40 mode (default: 40above) |
| `WATCHMAC: <mac>` | Add a MAC to the filter list (up to 4) |
| `CLEARMAC` | Remove all MAC filters |
| `LISTMACS` | Show all MACs seen with frame counts and RSSI |
| `TAG: <label>` | Label subsequent rows (e.g. `person_present`) |
| `SHOWMGMT` | Include management frame CSI in output (default) |
| `HIDEMGMT` | Suppress management frame CSI |
| `SETTIME: <unix>` | Set the real-time clock |

## Troubleshooting

**"I only see `idle` / `data` / `mgmt` — no `video` even during YouTube"**

1. **Check your iPhone is on 2.4 GHz** — enable "Maximize Compatibility" in
   Settings > Personal Hotspot.  This is the #1 cause.
2. **Check you are on the correct channel** — use `SCAN` or `airport -I`.
3. **Check HT40** — use `BANDWIDTH: 40above` (or `40below`).
4. **Use `LISTMACS`** to find your hotspot BSSID, then `WATCHMAC:` it.

**"I see many MACs I don't recognize"**

- These are neighbours' devices on the same channel.  Use `WATCHMAC:` with
  your hotspot BSSID and MacBook MAC to filter them out.
- Use `LISTMACS` to see which MACs are active and identify yours by RSSI.

**"The MAC from `arp -a` does not appear in CSI data"**

- iPhone uses MAC randomization.  The ARP MAC and 802.11 BSSID can differ.
- Use `airport -I` on macOS to get the correct BSSID.
- Or use `LISTMACS` on the ESP32 to discover it empirically.

## CSV column layout

See `isac_filter.py` header for the full 35-column layout.  Key columns:

| Index | Column | Use |
|---|---|---|
| 2 | `mac` | Sender MAC — identifies device vs router |
| 3 | `rssi` | Signal strength |
| 20 | `sig_len` | Frame size — larger during video streaming |
| 27 | `to_ds` | 1 = uplink (device -> router) |
| 28 | `from_ds` | 1 = downlink (router -> device) |
| 29 | `tid` | QoS Traffic ID (4/5 = Video) |
| 32 | `comm_class` | Automatic traffic classification |
| 33 | `env_label` | Optional physical environment label (TAG:) |
| 34 | `CSI_DATA` | Raw CSI complex values `[im re im re ...]` |
