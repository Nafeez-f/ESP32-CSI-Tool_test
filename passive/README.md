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

## What happens at boot (auto-scan)

On power-up, the firmware **automatically scans channels 1-13** trying
HT20, HT40-above, and HT40-below for each.  It picks the channel and
bandwidth with the most frames, then collects for 5 seconds and prints
a `LISTMACS` table showing every MAC it found.

**You do not need to type anything.**  Just flash, plug in, and pipe the
output to a file.  The auto-scan takes ~25 seconds, then CSI collection
starts automatically on the best channel.

To disable auto-scan (if you already know the channel): `idf.py menuconfig`
> ESP32 CSI Tool Config > uncheck "Auto-scan channels and bandwidth at boot".

## Quick-start

### 1. Flash and collect

```bash
cd passive
idf.py menuconfig    # optional: set channel, MAC filter, etc.
idf.py flash monitor | python ../python_utils/serial_append_time.py > experiment.csv
```

The monitor output is piped to a file.  **You cannot type commands into
the monitor when piping** — that's normal.  The auto-scan handles channel
and bandwidth selection for you automatically.

### 2. Watch the boot output

In the first ~25 seconds you'll see:
```
=== AUTO-SCAN: finding best channel + bandwidth ===
SCAN: ch  1  ->    12 frames/s  best_mode=HT20
SCAN: ch  6  ->   185 frames/s  best_mode=HT40a   <- BEST
...
SCAN: done. Best: channel 6, HT40-above (185 frames/s)
SCAN: now locked to channel 6 with HT40-above.

SCAN: collecting MACs for 5 seconds...

=== MACs seen on this channel ===
MAC                  Data   Mgmt    HT  RSSI MaxLen  Direction
-------------------------------------------------------------------
F0:A7:31:08:72:8E     142     52    98   -35   1480  downlink   <- your router
BA:87:BA:8E:58:DA      28      0    12   -30     28  uplink     <- your laptop
...
```

### 3. Identify your MACs

From your MacBook:
```bash
# Router MAC (BSSID)
/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport -I
# Look for "BSSID" line

# Your laptop MAC
ifconfig en0 | grep ether
```

**Do NOT use `arp -a`** — it can show a different MAC due to randomization.

In the LISTMACS table:
- **Router**: strong RSSI, high Data+Mgmt+HT counts, direction "downlink"
- **Laptop**: strong RSSI, direction "uplink", mostly idle/keepalive frames

### 4. (Optional) Filter to your MACs via menuconfig

If you want to filter CSI to only your router + laptop, set their MACs
in `idf.py menuconfig` > ESP32 CSI Tool Config > ISAC MAC Filter, then
reflash.  This removes neighbour traffic from the output.

### 5. Generate traffic and observe

Play YouTube on the MacBook.  The `comm_class` column will show:
- `video` — YouTube data frames (large sig_len, TID 4/5, HT/VHT)
- `mgmt` — beacons (continuous, even during idle)
- `idle` — keepalive null-data frames

### 6. Analyse

```bash
python python_utils/isac_filter.py experiment.csv --listmacs
python python_utils/isac_filter.py experiment.csv --analyze
python python_utils/isac_filter.py experiment.csv --timeline
```

## HT40 bandwidth: why it matters

Modern routers and hotspots use **HT40** (40 MHz channels).  Data frames
are transmitted across both the primary and secondary 20 MHz channels.  If
the ESP32 listens in HT20 mode, it misses these frames entirely — you only
see legacy-rate beacons and keepalives (all `sig_mode=0`, `MCS=0`).

Symptoms of wrong bandwidth:
- Only 4 `video` frames out of 2000+ total (almost all `idle`)
- All `MCS=0` even during active YouTube playback
- Very high retry rate (50-100%) on the few data frames that do appear
- `sig_mode=0` on everything (no HT frames)

The auto-scan fixes this by testing all three modes per channel.  The
secondary channel can be "above" (e.g. ch6+ch10) or "below" (e.g. ch6+ch2)
— depends on what the router negotiates with the client.

## Runtime commands

If you run `idf.py monitor` without piping (interactive mode), you can type:

| Command | Effect |
|---|---|
| `SCAN` | Re-scan channels with HT20/HT40 modes |
| `CHANNEL: <n>` | Switch to channel n (1-13), preserves bandwidth |
| `BANDWIDTH: 20\|40above\|40below` | Set bandwidth mode |
| `WATCHMAC: <mac>` | Add a MAC to the filter list (up to 4) |
| `CLEARMAC` | Remove all MAC filters |
| `LISTMACS` | Show all MACs with frame counts, HT status, RSSI |
| `TAG: <label>` | Label subsequent rows (e.g. `person_present`) |
| `SHOWMGMT` | Include management frame CSI (default) |
| `HIDEMGMT` | Suppress management frame CSI |
| `SETTIME: <unix>` | Set the real-time clock |

**Note:** Commands only work if the monitor is interactive (not piped).
When piping (`| python ... > file.csv`), use menuconfig or auto-scan instead.

## Troubleshooting

**"I only see `idle` — no `video` even during YouTube"**

1. Check the LISTMACS table at boot — is the **HT** column non-zero?
   - If all zeros: the auto-scan may have picked HT20.  Try reflashing
     after setting the channel manually in menuconfig, then test
     `BANDWIDTH: 40above` and `BANDWIDTH: 40below` interactively.
2. Verify you are on the correct channel (`airport -I` on macOS shows it)
3. If using iPhone hotspot, ensure "Maximize Compatibility" is ON

**"The MAC from `arp -a` does not appear in CSI data"**

Normal.  Use `airport -I` for the correct BSSID, or read the LISTMACS
table printed at boot.

## CSV column layout

| Index | Column | Use |
|---|---|---|
| 2 | `mac` | Sender MAC |
| 3 | `rssi` | Signal strength |
| 5 | `sig_mode` | 0=legacy, 1=HT, 2=VHT (must be 1+ for real data) |
| 20 | `sig_len` | Frame size — larger during video streaming |
| 27 | `to_ds` | 1 = uplink (device -> router) |
| 28 | `from_ds` | 1 = downlink (router -> device) |
| 29 | `tid` | QoS Traffic ID (4/5 = Video) |
| 32 | `comm_class` | Automatic traffic classification |
| 33 | `env_label` | Optional physical environment label (TAG:) |
| 34 | `CSI_DATA` | Raw CSI complex values `[im re im re ...]` |
