#!/usr/bin/env python3
"""
csi_analyzer.py — ISAC analysis tool for passive_csi firmware
==============================================================

Works with the CSV produced by passive_csi/main/main.cc.

CSV columns
-----------
  0  type         Always "CSI"
  1  mac          Transmitter MAC address
  2  frame_type   beacon | probe_resp | data | qos_data | null_data |
                  qos_null | ack | other_mgmt | other_ctrl | other
  3  to_ds        1 = uplink (device → AP)
  4  from_ds      1 = downlink (AP → device)
  5  sig_mode     0=legacy 802.11b/g  1=HT 802.11n  3=VHT 802.11ac
  6  mcs          Modulation/Coding Scheme (0-7 for HT)
  7  bandwidth    0=20MHz  1=40MHz
  8  sig_len      Frame length in bytes
  9  rssi         Signal strength (dBm)
  10 noise_floor  RF noise floor (0.25 dBm units)
  11 ampdu_cnt    A-MPDU aggregation count
  12 channel      Primary channel
  13 timestamp_us ESP32 microsecond timestamp
  14 retry        1 = retransmission
  15 tid          QoS TID (-1 if no QoS header)
  16 seq_num      802.11 sequence number
  17 CSI_DATA     [im0 re0 im1 re1 ...]

Usage
-----
  python csi_analyzer.py experiment.csv           # statistics
  python csi_analyzer.py experiment.csv --plot    # amplitude spectra
  python csi_analyzer.py experiment.csv --timeline  # heatmap over time
  python csi_analyzer.py experiment.csv --analyze   # 4-panel ISAC plot
  python csi_analyzer.py experiment.csv --macs AA:BB:CC --frame_type beacon
"""

import sys, re, math, argparse, statistics as _st
from collections import defaultdict

# ---- Column indices -------------------------------------------------------
C_MAC   = 1
C_FTYPE = 2
C_TO_DS = 3
C_FROM  = 4
C_SMODE = 5
C_MCS   = 6
C_BW    = 7
C_SLEN  = 8
C_RSSI  = 9
C_NF    = 10
C_AMPDU = 11
C_CH    = 12
C_TS    = 13
C_RETRY = 14
C_TID   = 15
C_SEQ   = 16
C_CSI   = 17

# Frame types that indicate real application traffic (not just idle beacons)
DATA_TYPES = {"data", "qos_data"}
# Frame types that are always present regardless of application traffic
BEACON_TYPES = {"beacon", "probe_resp"}
# Frame types that are small bookkeeping frames (no payload)
IDLE_TYPES = {"null_data", "qos_null", "ack", "rts", "cts", "block_ack", "block_ack_req"}

FRAME_TYPE_ORDER = [
    "beacon", "probe_resp", "qos_data", "data",
    "null_data", "qos_null", "other_mgmt", "other_ctrl", "other_data", "other"
]


# ---- CLI ------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="CSI analyser for passive_csi firmware",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument("csv_file")
    p.add_argument("--macs",       nargs="+", metavar="MAC",
                   help="Keep only these transmitter MACs")
    p.add_argument("--frame_type", metavar="TYPE",
                   help="Keep only this frame_type (e.g. beacon, qos_data)")
    p.add_argument("--stats",   action="store_true", default=True)
    p.add_argument("--plot",    action="store_true",
                   help="Mean CSI amplitude spectrum per frame type")
    p.add_argument("--timeline",action="store_true",
                   help="CSI amplitude heatmap over time")
    p.add_argument("--analyze", action="store_true",
                   help="4-panel ISAC time-domain plot")
    p.add_argument("--window",  type=float, default=1.0,
                   help="Bin width in seconds for --analyze (default 1.0)")
    p.add_argument("--out",     metavar="FILE",
                   help="Save filtered rows to this CSV")
    return p.parse_args()


# ---- Parsing --------------------------------------------------------------

def _i(s, d=-1):
    try: return int(s.strip())
    except: return d

def _f(s, d=float("nan")):
    try: return float(s.strip())
    except: return d

def parse_amplitudes(csi_col):
    m = re.search(r"\[([^\]]*)\]", csi_col)
    if not m: return []
    raw = [int(x) for x in m.group(1).split() if x]
    return [math.sqrt(raw[i]**2 + raw[i+1]**2) for i in range(0, len(raw)-1, 2)]

def load(path, watch_macs=None, frame_type_filter=None):
    watch = {m.strip().upper() for m in (watch_macs or [])}
    with open(path, errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line.startswith("CSI,"):
                continue
            parts = line.split(",", C_CSI + 1)
            if len(parts) < C_CSI + 1:
                continue

            mac = parts[C_MAC].strip().upper()
            if watch and mac not in watch:
                continue

            ftype = parts[C_FTYPE].strip()
            if frame_type_filter and ftype != frame_type_filter:
                continue

            ts_us = _f(parts[C_TS])
            if math.isnan(ts_us):
                continue

            amps = parse_amplitudes(parts[C_CSI])
            if not amps:
                continue

            to_ds   = _i(parts[C_TO_DS])
            from_ds = _i(parts[C_FROM])

            if   to_ds == 1 and from_ds == 0: direction = "uplink"
            elif to_ds == 0 and from_ds == 1: direction = "downlink"
            else:                              direction = "other"

            yield {
                "mac":       mac,
                "frame_type": ftype,
                "direction": direction,
                "sig_mode":  _i(parts[C_SMODE]),
                "mcs":       _i(parts[C_MCS]),
                "bandwidth": _i(parts[C_BW]),
                "sig_len":   _i(parts[C_SLEN]),
                "rssi":      _i(parts[C_RSSI]),
                "noise_floor": _i(parts[C_NF]),
                "ampdu_cnt": _i(parts[C_AMPDU]),
                "retry":     _i(parts[C_RETRY]),
                "tid":       _i(parts[C_TID]),
                "seq_num":   _i(parts[C_SEQ]),
                "ts_us":     ts_us,
                "ts_sec":    ts_us / 1e6,
                "amplitudes": amps,
                "raw":       line,
            }


# ---- Statistics -----------------------------------------------------------

def stats_by_type(rows_by_type):
    out = {}
    for ft, rows in rows_by_type.items():
        amps  = [a for r in rows for a in r["amplitudes"]]
        rssis = [r["rssi"] for r in rows]
        slens = [r["sig_len"] for r in rows]
        mcs   = [r["mcs"] for r in rows]
        retries = [r["retry"] for r in rows if r["retry"] >= 0]
        smodes = sorted(set(r["sig_mode"] for r in rows))

        ts = sorted(r["ts_sec"] for r in rows)
        if len(ts) > 1:
            ifis = [ts[i+1]-ts[i] for i in range(len(ts)-1)]
            mean_ifi = _st.mean(ifis)
            rate = 1/mean_ifi if mean_ifi > 0 else 0
        else:
            mean_ifi = rate = float("nan")

        seq_valid = [r["seq_num"] for r in sorted(rows, key=lambda x: x["ts_sec"])
                     if r["seq_num"] >= 0]
        lost = sum(max(0, (seq_valid[i]-seq_valid[i-1])%4096-1)
                   for i in range(1,len(seq_valid))
                   if (seq_valid[i]-seq_valid[i-1])%4096 < 200)

        n_down = sum(1 for r in rows if r["direction"]=="downlink")
        n_up   = sum(1 for r in rows if r["direction"]=="uplink")

        out[ft] = dict(
            count=len(rows), mean_amp=_st.mean(amps),
            std_amp=_st.stdev(amps) if len(amps)>1 else 0,
            mean_rssi=_st.mean(rssis), mean_sig_len=_st.mean(slens),
            mean_mcs=_st.mean(mcs),
            retry_pct=(_st.mean(retries)*100 if retries else -1),
            mean_ifi_ms=mean_ifi*1000, rate=rate,
            lost=lost, n_down=n_down, n_up=n_up, sig_modes=smodes,
        )
    return out


def print_stats(stats, rows_by_type):
    print("\n=== CSI statistics by frame type ===\n")
    order = FRAME_TYPE_ORDER + [k for k in sorted(stats) if k not in FRAME_TYPE_ORDER]
    for ft in order:
        if ft not in stats: continue
        s = stats[ft]
        macs = sorted({r["mac"] for r in rows_by_type[ft]})
        role = ("IDLE BASELINE" if ft in BEACON_TYPES else
                "COMMUNICATION" if ft in DATA_TYPES else
                "BOOKKEEPING"   if ft in IDLE_TYPES  else "")
        print(f"  frame_type : {ft}  [{role}]")
        print(f"  Frames     : {s['count']}")
        print(f"  MACs       : {', '.join(macs)}")
        print(f"  Direction  : {s['n_down']} downlink  {s['n_up']} uplink")
        print(f"  sig_mode   : {s['sig_modes']}  (0=b/g 1=HT-n 3=VHT-ac)")
        print(f"  MCS        : {s['mean_mcs']:.1f}   BW hint from bandwidth field")
        print(f"  sig_len    : {s['mean_sig_len']:.0f} bytes avg")
        print(f"  RSSI       : {s['mean_rssi']:.1f} dBm")
        print(f"  Retry      : {s['retry_pct']:.1f}%")
        print(f"  Lost frames: {s['lost']}  (seq_num gaps)")
        print(f"  Rate       : {s['rate']:.1f} frames/sec   IFI {s['mean_ifi_ms']:.1f} ms")
        print(f"  CSI amp    : mean={s['mean_amp']:.3f}  std={s['std_amp']:.3f}")
        print()


# ---- ISAC 4-panel plot ----------------------------------------------------

def plot_analyze(rows, window=1.0):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("pip install numpy matplotlib"); sys.exit(1)

    rows = sorted(rows, key=lambda r: r["ts_sec"])
    if len(rows) < 2: return

    t0 = rows[0]["ts_sec"]
    ts   = np.array([r["ts_sec"] - t0 for r in rows])
    sl   = np.array([r["sig_len"]     for r in rows], dtype=float)
    ret  = np.array([r["retry"]       for r in rows], dtype=float)
    mamp = np.array([sum(r["amplitudes"])/len(r["amplitudes"]) for r in rows])
    ftypes = [r["frame_type"] for r in rows]

    # Colours per frame type
    colors = {
        "beacon":      "#2196F3",  # blue
        "probe_resp":  "#64B5F6",  # light blue
        "qos_data":    "#F44336",  # red
        "data":        "#E91E63",  # pink
        "null_data":   "#9E9E9E",  # grey
        "qos_null":    "#BDBDBD",  # light grey
        "other_mgmt":  "#FF9800",  # orange
        "other_ctrl":  "#795548",  # brown
        "other":       "#607D8B",  # blue grey
    }

    edges   = np.arange(0, float(ts[-1]) + window, window)
    centers = edges[:-1] + window/2

    throughput  = np.zeros(len(centers))
    csi_std     = np.zeros(len(centers))
    retry_pct   = np.zeros(len(centers))
    fps         = np.zeros(len(centers))
    stacked     = {ft: np.zeros(len(centers)) for ft in colors}

    for i, (b0, b1) in enumerate(zip(edges[:-1], edges[1:])):
        mask = (ts >= b0) & (ts < b1)
        n = int(mask.sum())
        if n == 0: continue
        throughput[i] = sl[mask].sum()
        csi_std[i]    = float(np.std(mamp[mask])) if n > 1 else 0
        fps[i]        = n
        rv = ret[mask]; rv = rv[rv >= 0]
        retry_pct[i]  = float(np.mean(rv)*100) if len(rv) else 0
        for j, ft in enumerate(ftypes):
            if mask[j] and ft in stacked:
                stacked[ft][i] += sl[j]

    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle("ISAC — Communication + Sensing  (same frames, automatic classification)",
                 fontsize=13, fontweight="bold")

    # Panel 1: stacked throughput by frame type
    bottom = np.zeros(len(centers))
    for ft in FRAME_TYPE_ORDER:
        if ft not in stacked: continue
        v = stacked[ft] / 1000
        axes[0].bar(centers, v, bottom=bottom, width=window*0.85,
                    color=colors.get(ft,"#999"), alpha=0.85, label=ft)
        bottom += v
    axes[0].set_ylabel("Throughput (KB/s)"); axes[0].legend(fontsize=7, ncol=4)
    axes[0].set_title(
        "Panel 1 — Traffic load stacked by frame type\n"
        "Blue=beacon (always present)  Red=qos_data (YouTube/browsing)  Grey=null_data (idle polling)",
        fontsize=9)
    axes[0].grid(axis="y", alpha=0.3)

    # Panel 2: CSI sensing signal
    axes[1].bar(centers, csi_std, width=window*0.85, color="darkorange", alpha=0.85)
    axes[1].set_ylabel("CSI amp std dev")
    axes[1].set_title(
        "Panel 2 — Channel dynamics  (std of CSI amplitude per second)\n"
        "Spikes = physical change in the space  ← the SENSING signal",
        fontsize=9)
    axes[1].grid(axis="y", alpha=0.3)

    # Panel 3: retry rate
    axes[2].bar(centers, retry_pct, width=window*0.85, color="firebrick", alpha=0.85)
    axes[2].set_ylabel("Retry %")
    axes[2].set_title(
        "Panel 3 — Retransmission rate  (from 802.11 header)\n"
        "High = channel congested / obstructed / interference",
        fontsize=9)
    axes[2].grid(axis="y", alpha=0.3)

    # Panel 4: frame rate
    axes[3].bar(centers, fps, width=window*0.85, color="seagreen", alpha=0.85)
    axes[3].set_ylabel("Frames/sec")
    axes[3].set_title(
        "Panel 4 — CSI sampling rate\n"
        "Baseline ~10/sec (beacons only)  →  jumps to 30-50/sec during YouTube  "
        "← the ISAC coupling",
        fontsize=9)
    axes[3].set_xlabel("Time (seconds)")
    axes[3].grid(axis="y", alpha=0.3)

    plt.tight_layout()

    # Console table
    print(f"\n{'Time(s)':<9} {'KB/s':>7} {'CsiStd':>8} {'Retry%':>7} {'fps':>5}  DomType")
    print("-"*55)
    for i, bc in enumerate(centers):
        mask = (ts >= edges[i]) & (ts < edges[i+1])
        dom = max(set(f for j,f in enumerate(ftypes) if mask[j]),
                  key=lambda x: sum(1 for j,f in enumerate(ftypes) if mask[j] and f==x),
                  default="")
        print(f"{bc:<9.1f} {throughput[i]/1000:>7.1f} {csi_std[i]:>8.3f} "
              f"{retry_pct[i]:>7.1f} {fps[i]:>5.0f}  {dom}")
    plt.show()


# ---- Heatmap --------------------------------------------------------------

def plot_timeline(rows):
    try:
        import numpy as np, matplotlib.pyplot as plt, matplotlib.patches as mpatches
    except ImportError:
        print("pip install numpy matplotlib"); sys.exit(1)

    rows = sorted(rows, key=lambda r: r["ts_sec"])
    if not rows: return

    t0      = rows[0]["ts_sec"]
    min_len = min(len(r["amplitudes"]) for r in rows)
    ts      = np.array([r["ts_sec"]-t0 for r in rows])
    matrix  = np.array([r["amplitudes"][:min_len] for r in rows]).T
    ftypes  = [r["frame_type"] for r in rows]

    colors = {"beacon":"#2196F3","probe_resp":"#64B5F6",
              "qos_data":"#F44336","data":"#E91E63",
              "null_data":"#9E9E9E","qos_null":"#BDBDBD",
              "other_mgmt":"#FF9800","other_ctrl":"#795548","other":"#607D8B"}

    fig, axes = plt.subplots(2, 1, figsize=(14,8),
                              gridspec_kw={"height_ratios":[4,1]})
    im = axes[0].imshow(matrix, aspect="auto", origin="lower",
                        extent=[ts[0], ts[-1], 0, min_len],
                        cmap="viridis", interpolation="nearest")
    axes[0].set_ylabel("Subcarrier index")
    axes[0].set_title("CSI Amplitude Heatmap — vertical bands = physical change in the space")
    fig.colorbar(im, ax=axes[0], label="Amplitude")

    for i in range(len(ts)-1):
        axes[1].axvspan(ts[i], ts[i+1], color=colors.get(ftypes[i],"#ccc"), alpha=0.8)
    axes[1].set_xlim(ts[0], ts[-1]); axes[1].set_yticks([])
    axes[1].set_xlabel("Time (seconds)"); axes[1].set_title("Frame type")

    patches = [mpatches.Patch(color=colors.get(ft,"#ccc"), label=ft)
               for ft in sorted(set(ftypes))]
    axes[1].legend(handles=patches, fontsize=8, loc="upper right")
    plt.tight_layout(); plt.show()


# ---- Spectrum per frame type ----------------------------------------------

def plot_spectrum(rows_by_type):
    try:
        import numpy as np, matplotlib.pyplot as plt
    except ImportError:
        print("pip install numpy matplotlib"); sys.exit(1)

    colors = {"beacon":"#2196F3","probe_resp":"#64B5F6",
              "qos_data":"#F44336","data":"#E91E63",
              "null_data":"#9E9E9E","qos_null":"#BDBDBD",
              "other_mgmt":"#FF9800","other_ctrl":"#795548","other":"#607D8B"}

    fig, ax = plt.subplots(figsize=(12,5))
    for ft in FRAME_TYPE_ORDER:
        if ft not in rows_by_type: continue
        rows = rows_by_type[ft]
        mn = min(len(r["amplitudes"]) for r in rows)
        mat = np.array([r["amplitudes"][:mn] for r in rows])
        mu = mat.mean(0); sd = mat.std(0)
        c = colors.get(ft,"#999")
        ax.plot(mu, label=ft, color=c)
        ax.fill_between(range(mn), mu-sd, mu+sd, alpha=0.12, color=c)

    ax.set_xlabel("Subcarrier index"); ax.set_ylabel("Mean amplitude ± 1 std")
    ax.set_title("CSI amplitude spectrum per frame type\n"
                 "If beacon and qos_data curves differ → the channel changed with traffic")
    ax.legend(); plt.tight_layout(); plt.show()


# ---- File output ----------------------------------------------------------

def save_filtered(rows, path):
    hdr = ("type,mac,frame_type,to_ds,from_ds,sig_mode,mcs,bandwidth,"
           "sig_len,rssi,noise_floor,ampdu_cnt,channel,timestamp_us,"
           "retry,tid,seq_num,CSI_DATA")
    with open(path, "w") as f:
        f.write(hdr+"\n")
        for r in rows:
            f.write(r["raw"]+"\n")
    print(f"Wrote {len(rows)} rows to '{path}'")


# ---- Main -----------------------------------------------------------------

def main():
    args = parse_args()
    rows = list(load(args.csv_file, args.macs, args.frame_type))
    if not rows:
        print("No rows matched.", file=sys.stderr); sys.exit(1)

    print(f"Loaded {len(rows)} CSI frames.")

    rows_by_type = defaultdict(list)
    for r in rows: rows_by_type[r["frame_type"]].append(r)

    if args.stats:
        s = stats_by_type(rows_by_type)
        print_stats(s, rows_by_type)

    if args.out:
        save_filtered(rows, args.out)

    if args.analyze:
        plot_analyze(rows, args.window)

    if args.timeline:
        plot_timeline(rows)

    if args.plot:
        plot_spectrum(rows_by_type)


if __name__ == "__main__":
    main()
