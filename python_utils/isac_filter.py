#!/usr/bin/env python3
"""
isac_filter.py — ISAC post-processing utility for ESP32 CSI Tool (passive mode)
================================================================================

ISAC = Integrated Sensing and Communication.
The Wi-Fi frames carrying data (YouTube, browsing, etc.) are simultaneously
used to estimate the physical channel state (CSI).  This tool visualises BOTH
the communication signal AND the sensing signal from the same stream of frames.

KEY DESIGN PRINCIPLE
--------------------
comm_class is determined AUTOMATICALLY from frame metadata — no human commands
needed.  Just collect data and run this script.  The ESP32 reads TID and
sig_len from every frame header and classifies it on-device:

  mgmt       — management frame (beacon, probe, etc.) — suppressed by default
  video      — TID 4/5 or sig_len > 800 bytes  (YouTube, Netflix, video calls)
  voice      — TID 6/7                          (VoIP, FaceTime audio)
  browsing   — TID 0/3 and sig_len > 200 bytes  (web pages loading)
  background — TID 1/2                          (cloud sync, OS updates)
  idle       — sig_len <= 100 bytes             (only keepalives, null data)
  data       — everything else

env_label is OPTIONAL and set via the TAG: serial command.  Use it only when
you want to label the PHYSICAL ENVIRONMENT for sensing experiments, e.g.:
  person_present / empty_room / walking / sitting
Not needed for basic ISAC analysis.

CSV column layout
-----------------
  0  type            1  role            2  mac
  3  rssi            4  rate            5  sig_mode
  6  mcs             7  bandwidth       8  smoothing
  9  not_sounding   10  aggregation    11  stbc
  12 fec_coding     13 sgi             14 noise_floor
  15 ampdu_cnt      16 channel         17 secondary_channel
  18 local_timestamp 19 ant            20 sig_len
  21 rx_state       22 real_time_set   23 real_timestamp
  24 len
  25 seq_num    ← 802.11 sequence number (gap = lost frame)
  26 retry      ← 1=retransmission
  27 to_ds      ← 1=uplink (device→router)
  28 from_ds    ← 1=downlink (router→device)
  29 tid        ← QoS Traffic ID
  30 is_qos     ← 1=QoS data frame
  31 duration   ← NAV channel reservation (µs)
  32 comm_class ← AUTO: video/voice/browsing/background/idle/data
  33 env_label  ← OPTIONAL: physical environment label (TAG: command)
  34 CSI_DATA   ← [im re im re ...]

Usage
-----
# Just run it — no setup, no labels needed:
python isac_filter.py experiment.csv

# The ISAC plot (4 panels: throughput, CSI variation, retry, frame rate):
python isac_filter.py experiment.csv --analyze

# Filter to your router's MAC only (downlink frames):
python isac_filter.py experiment.csv --macs A4:C3:F0:85:AC:01 --analyze

# CSI heatmap over time:
python isac_filter.py experiment.csv --timeline

# Compare CSI amplitude spectra between traffic classes:
python isac_filter.py experiment.csv --plot

# Save only video-class rows:
python isac_filter.py experiment.csv --comm_class video --out video_only.csv
"""

import sys
import re
import math
import argparse
import statistics as _stats
from collections import defaultdict

# ---- Column indices ----------------------------------------------------------
COL_MAC        = 2
COL_RSSI       = 3
COL_RATE       = 4
COL_SIG_MODE   = 5
COL_MCS        = 6
COL_NOISE_FL   = 14
COL_AMPDU_CNT  = 15
COL_SIG_LEN    = 20
COL_REAL_TS    = 23
COL_LEN        = 24
COL_SEQ_NUM    = 25
COL_RETRY      = 26
COL_TO_DS      = 27
COL_FROM_DS    = 28
COL_TID        = 29
COL_IS_QOS     = 30
COL_DURATION   = 31
COL_COMM_CLASS = 32   # automatic, from firmware
COL_ENV_LABEL  = 33   # optional, from TAG: command
COL_CSI        = 34

COMM_CLASS_ORDER = ["video", "voice", "browsing", "data", "background", "idle", "mgmt"]


# ---- CLI -------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="ISAC CSI analysis — no labels or setup needed",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("csv_file", help="CSV file from the ESP32 passive mode")
    p.add_argument(
        "--macs", nargs="+", metavar="MAC",
        help="Keep only rows from these MACs (e.g. your router BSSID for downlink only)",
    )
    p.add_argument(
        "--comm_class", metavar="CLASS",
        help="Keep only rows with this comm_class: video/voice/browsing/background/idle/data",
    )
    p.add_argument(
        "--env_label", metavar="LABEL",
        help="Keep only rows with this env_label (from TAG: command). "
             "Use 'unlabelled' for rows with no tag.",
    )
    p.add_argument(
        "--out", metavar="FILE",
        help="Write filtered rows to this CSV file",
    )
    p.add_argument(
        "--analyze", action="store_true",
        help="4-panel ISAC time-domain plot: throughput, CSI variation, "
             "retry rate, frame rate. Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--plot", action="store_true",
        help="Mean CSI amplitude spectrum per comm_class. Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--timeline", action="store_true",
        help="CSI amplitude heatmap over time with comm_class colour strip. "
             "Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--subcarrier", type=int, default=None,
        help="Time-series for one subcarrier index (used with --plot)",
    )
    p.add_argument(
        "--window", type=float, default=1.0,
        help="Bin width in seconds for --analyze (default: 1.0)",
    )
    p.add_argument(
        "--stats", action="store_true", default=True,
        help="Print per-comm_class statistics (default: on)",
    )
    p.add_argument(
        "--listmacs", action="store_true",
        help="Print per-MAC summary table (frame counts, direction, RSSI, "
             "comm_class breakdown) to help identify your hotspot BSSID",
    )
    return p.parse_args()


# ---- Parsing helpers -------------------------------------------------------

def _int_or(s, default=-1):
    try:
        return int(s.strip())
    except (ValueError, AttributeError):
        return default

def _float_or(s, default=float("nan")):
    try:
        return float(s.strip())
    except (ValueError, AttributeError):
        return default

def parse_csi_amplitudes(csi_col):
    m = re.search(r"\[([^\]]*)\]", csi_col)
    if not m:
        return []
    raw = [int(x) for x in m.group(1).split() if x]
    return [math.sqrt(raw[i]**2 + raw[i+1]**2) for i in range(0, len(raw)-1, 2)]

def normalise_mac(mac):
    return mac.strip().upper()


# ---- CSV loading -----------------------------------------------------------

def load_csv(path, watch_macs=None, comm_class_filter=None, env_label_filter=None):
    watch_macs_upper = {normalise_mac(m) for m in (watch_macs or [])}

    with open(path, newline="", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line.startswith("CSI_DATA"):
                continue

            parts = line.split(",", COL_CSI + 1)
            if len(parts) < COL_CSI + 1:
                continue

            mac = normalise_mac(parts[COL_MAC])
            if watch_macs_upper and mac not in watch_macs_upper:
                continue

            comm_class = parts[COL_COMM_CLASS].strip()
            if comm_class_filter and comm_class != comm_class_filter:
                continue

            env_label = parts[COL_ENV_LABEL].strip()
            if env_label_filter is not None:
                want = "" if env_label_filter.lower() == "unlabelled" else env_label_filter
                if env_label != want:
                    continue

            real_ts = _float_or(parts[COL_REAL_TS])
            if math.isnan(real_ts):
                continue

            amps = parse_csi_amplitudes(parts[COL_CSI])
            if not amps:
                continue

            tid      = _int_or(parts[COL_TID])
            to_ds    = _int_or(parts[COL_TO_DS])
            from_ds  = _int_or(parts[COL_FROM_DS])

            if to_ds == 1 and from_ds == 0:
                direction = "uplink"
            elif to_ds == 0 and from_ds == 1:
                direction = "downlink"
            else:
                direction = "other"

            yield {
                "mac":            mac,
                "rssi":           _int_or(parts[COL_RSSI]),
                "rate":           _int_or(parts[COL_RATE]),
                "sig_mode":       _int_or(parts[COL_SIG_MODE]),
                "mcs":            _int_or(parts[COL_MCS]),
                "noise_floor":    _int_or(parts[COL_NOISE_FL]),
                "ampdu_cnt":      _int_or(parts[COL_AMPDU_CNT]),
                "sig_len":        _int_or(parts[COL_SIG_LEN]),
                "real_timestamp": real_ts,
                "seq_num":        _int_or(parts[COL_SEQ_NUM]),
                "retry":          _int_or(parts[COL_RETRY]),
                "to_ds":          to_ds,
                "from_ds":        from_ds,
                "tid":            tid,
                "is_qos":         _int_or(parts[COL_IS_QOS]),
                "duration":       _int_or(parts[COL_DURATION]),
                "comm_class":     comm_class if comm_class else "idle",
                "env_label":      env_label if env_label else "(unlabelled)",
                "direction":      direction,
                "amplitudes":     amps,
                "raw":            line,
            }


# ---- Statistics ------------------------------------------------------------

def compute_stats(rows_by_class):
    stats = {}
    for cls, rows in rows_by_class.items():
        all_amps   = [a for r in rows for a in r["amplitudes"]]
        rssis      = [r["rssi"] for r in rows]
        sig_lens   = [r["sig_len"] for r in rows]
        mcs_vals   = [r["mcs"] for r in rows]
        retry_vals = [r["retry"] for r in rows if r["retry"] >= 0]
        n_down     = sum(1 for r in rows if r["direction"] == "downlink")
        n_up       = sum(1 for r in rows if r["direction"] == "uplink")

        seq_valid  = [r["seq_num"] for r in sorted(rows, key=lambda x: x["real_timestamp"])
                      if r["seq_num"] >= 0]
        lost = sum(
            max(0, (seq_valid[i] - seq_valid[i-1]) % 4096 - 1)
            for i in range(1, len(seq_valid))
            if (seq_valid[i] - seq_valid[i-1]) % 4096 < 100
        )

        ts_sorted = sorted(r["real_timestamp"] for r in rows)
        if len(ts_sorted) > 1:
            ifis = [ts_sorted[i+1] - ts_sorted[i] for i in range(len(ts_sorted)-1)]
            mean_ifi = _stats.mean(ifis)
            csi_rate = 1.0 / mean_ifi if mean_ifi > 0 else 0
        else:
            mean_ifi = float("nan")
            csi_rate = float("nan")

        stats[cls] = {
            "count":        len(rows),
            "mean_amp":     _stats.mean(all_amps),
            "std_amp":      _stats.stdev(all_amps) if len(all_amps) > 1 else 0.0,
            "mean_rssi":    _stats.mean(rssis),
            "mean_sig_len": _stats.mean(sig_lens),
            "mean_mcs":     _stats.mean(mcs_vals),
            "retry_rate":   (_stats.mean(retry_vals) * 100) if retry_vals else -1,
            "n_downlink":   n_down,
            "n_uplink":     n_up,
            "lost_frames":  lost,
            "mean_ifi_ms":  mean_ifi * 1000 if not math.isnan(mean_ifi) else float("nan"),
            "csi_rate":     csi_rate,
        }
    return stats


def print_stats(stats, rows_by_class):
    print("\n=== ISAC communication-class statistics ===")
    print("(comm_class is automatic — no labels or commands needed)\n")
    for cls in COMM_CLASS_ORDER + [c for c in sorted(stats) if c not in COMM_CLASS_ORDER]:
        if cls not in stats:
            continue
        s = stats[cls]
        macs = {r["mac"] for r in rows_by_class[cls]}
        print(f"  comm_class : {cls}")
        print(f"  Frames     : {s['count']}")
        print(f"  MACs       : {', '.join(sorted(macs))}")
        print(f"  Direction  : {s['n_downlink']} downlink, {s['n_uplink']} uplink")
        print(f"  sig_len    : {s['mean_sig_len']:.0f} bytes avg"
              f"  (video≈1460, browsing≈400, idle≈14)")
        print(f"  MCS        : {s['mean_mcs']:.1f}   RSSI: {s['mean_rssi']:.1f} dBm")
        print(f"  Retry rate : {s['retry_rate']:.1f}%")
        print(f"  Lost frames: {s['lost_frames']}  (seq_num gaps)")
        print(f"  CSI rate   : {s['csi_rate']:.1f} samples/sec"
              f"  IFI: {s['mean_ifi_ms']:.1f} ms")
        print(f"  CSI amp    : mean={s['mean_amp']:.3f}  std={s['std_amp']:.3f}")
        print()


# ---- ISAC analysis plot ----------------------------------------------------

def plot_isac_analysis(rows, window_sec=1.0):
    """
    4-panel time-domain ISAC plot.

    Panel 1 — Communication load (KB/s) + traffic class breakdown
    Panel 2 — CSI amplitude variation (the SENSING signal)
    Panel 3 — Retransmission rate
    Panel 4 — CSI sampling rate (the ISAC coupling: more traffic = more sensing)

    No labels or TAG commands needed — comm_class comes from the frame headers.
    """
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("pip install numpy matplotlib", file=sys.stderr)
        sys.exit(1)

    rows = sorted(rows, key=lambda r: r["real_timestamp"])
    if len(rows) < 2:
        print("Need at least 2 rows.", file=sys.stderr)
        return

    t0       = rows[0]["real_timestamp"]
    ts       = np.array([r["real_timestamp"] - t0 for r in rows])
    sig_lens = np.array([r["sig_len"]  for r in rows], dtype=float)
    retries  = np.array([r["retry"]    for r in rows], dtype=float)
    mean_amps = np.array([sum(r["amplitudes"]) / len(r["amplitudes"]) for r in rows])
    classes  = [r["comm_class"] for r in rows]

    cls_colors = {
        "video":      "#e74c3c",
        "voice":      "#9b59b6",
        "browsing":   "#3498db",
        "data":       "#1abc9c",
        "background": "#f39c12",
        "idle":       "#95a5a6",
        "mgmt":       "#bdc3c7",
    }

    max_t = float(ts[-1])
    edges = np.arange(0, max_t + window_sec, window_sec)
    centers = edges[:-1] + window_sec / 2.0

    throughput   = np.zeros(len(centers))
    csi_variance = np.zeros(len(centers))
    retry_pct    = np.zeros(len(centers))
    frame_rate   = np.zeros(len(centers))
    cls_stacks   = {c: np.zeros(len(centers)) for c in cls_colors}

    for i, (b0, b1) in enumerate(zip(edges[:-1], edges[1:])):
        mask = (ts >= b0) & (ts < b1)
        n = int(mask.sum())
        if n == 0:
            continue
        throughput[i]   = sig_lens[mask].sum()
        csi_variance[i] = float(np.std(mean_amps[mask])) if n > 1 else 0.0
        frame_rate[i]   = n
        rv = retries[mask]; rv = rv[rv >= 0]
        retry_pct[i]    = float(np.mean(rv) * 100) if len(rv) > 0 else 0.0
        for j, cls in enumerate(classes):
            if mask[j] and cls in cls_stacks:
                cls_stacks[cls][i] += sig_lens[j]

    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle(
        "ISAC — Communication load + Channel sensing  (same frames, no labels needed)",
        fontsize=13, fontweight="bold"
    )

    # Panel 1: stacked throughput by comm_class
    bottom = np.zeros(len(centers))
    for cls in COMM_CLASS_ORDER:
        if cls in cls_stacks:
            vals = cls_stacks[cls] / 1000.0
            axes[0].bar(centers, vals, bottom=bottom, width=window_sec*0.85,
                        color=cls_colors.get(cls, "#cccccc"), alpha=0.85,
                        label=cls)
            bottom += vals
    axes[0].set_ylabel("Throughput (KB/s)", fontsize=9)
    axes[0].set_title(
        "Panel 1 — Communication load, stacked by traffic class\n"
        "Colours show what type of traffic produced each CSI sample — automatically",
        fontsize=9)
    axes[0].legend(loc="upper right", fontsize=8, ncol=3)
    axes[0].grid(axis="y", alpha=0.3)

    # Panel 2: CSI sensing signal
    axes[1].bar(centers, csi_variance, width=window_sec*0.85,
                color="darkorange", alpha=0.85)
    axes[1].set_ylabel("CSI amp std dev", fontsize=9)
    axes[1].set_title(
        "Panel 2 — Channel dynamics  (std of CSI amplitude per second)\n"
        "Spikes = physical change in the space (person moving, door opening, etc.)",
        fontsize=9)
    axes[1].grid(axis="y", alpha=0.3)

    # Panel 3: retry rate
    axes[2].bar(centers, retry_pct, width=window_sec*0.85,
                color="firebrick", alpha=0.85)
    axes[2].set_ylabel("Retry %", fontsize=9)
    axes[2].set_title(
        "Panel 3 — Retransmission rate  (from 802.11 frame header)\n"
        "High = channel congested / interfered / physically obstructed",
        fontsize=9)
    axes[2].grid(axis="y", alpha=0.3)

    # Panel 4: CSI sampling rate
    axes[3].bar(centers, frame_rate, width=window_sec*0.85,
                color="seagreen", alpha=0.85)
    axes[3].set_ylabel("CSI samples/sec", fontsize=9)
    axes[3].set_title(
        "Panel 4 — CSI sampling rate  (= number of data frames per second)\n"
        "More traffic → more frames → higher sensing resolution  "
        "← this is the ISAC coupling",
        fontsize=9)
    axes[3].set_xlabel("Time (seconds)", fontsize=10)
    axes[3].grid(axis="y", alpha=0.3)

    plt.tight_layout()

    print("\n=== ISAC time-bin summary ===")
    print(f"{'Time(s)':<9} {'KB/s':>7} {'CsiStd':>8} {'Retry%':>7} "
          f"{'fps':>5}  DominantClass")
    print("-" * 55)
    for i, bc in enumerate(centers):
        mask = (ts >= edges[i]) & (ts < edges[i+1])
        if mask.sum() > 0:
            cls_in = [classes[j] for j in range(len(ts)) if mask[j]]
            dom = max(set(cls_in), key=cls_in.count)
        else:
            dom = ""
        print(f"{bc:<9.1f} {throughput[i]/1000:>7.1f} {csi_variance[i]:>8.3f} "
              f"{retry_pct[i]:>7.1f} {frame_rate[i]:>5.0f}  {dom}")

    plt.show()


# ---- CSI heatmap -----------------------------------------------------------

def plot_csi_timeline(rows):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("pip install numpy matplotlib", file=sys.stderr)
        sys.exit(1)

    rows = sorted(rows, key=lambda r: r["real_timestamp"])
    if not rows:
        return

    t0      = rows[0]["real_timestamp"]
    min_len = min(len(r["amplitudes"]) for r in rows)
    ts      = np.array([r["real_timestamp"] - t0 for r in rows])
    matrix  = np.array([r["amplitudes"][:min_len] for r in rows]).T
    classes = [r["comm_class"] for r in rows]

    cls_colors = {"video":"#e74c3c","voice":"#9b59b6","browsing":"#3498db",
                  "data":"#1abc9c","background":"#f39c12","idle":"#95a5a6",
                  "mgmt":"#bdc3c7"}

    fig, axes = plt.subplots(2, 1, figsize=(14, 8),
                             gridspec_kw={"height_ratios": [4, 1]})

    im = axes[0].imshow(
        matrix, aspect="auto", origin="lower",
        extent=[ts[0], ts[-1], 0, min_len],
        cmap="viridis", interpolation="nearest"
    )
    axes[0].set_ylabel("Subcarrier index")
    axes[0].set_title(
        "CSI Amplitude Heatmap\n"
        "Vertical bands of change = something moved in the physical space"
    )
    fig.colorbar(im, ax=axes[0], label="Amplitude")

    for i in range(len(ts) - 1):
        axes[1].axvspan(ts[i], ts[i+1],
                        color=cls_colors.get(classes[i], "#cccccc"), alpha=0.8)
    axes[1].set_xlim(ts[0], ts[-1])
    axes[1].set_yticks([])
    axes[1].set_xlabel("Time (seconds)")
    axes[1].set_title("Traffic class (automatic)")

    unique_cls = sorted(set(classes))
    patches = [mpatches.Patch(color=cls_colors.get(c, "#cccccc"), label=c)
               for c in unique_cls]
    axes[1].legend(handles=patches, loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.show()


# ---- Mean spectrum ---------------------------------------------------------

def plot_mean_spectrum(rows_by_class, subcarrier=None):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("pip install numpy matplotlib", file=sys.stderr)
        sys.exit(1)

    cls_colors = {"video":"#e74c3c","voice":"#9b59b6","browsing":"#3498db",
                  "data":"#1abc9c","background":"#f39c12","idle":"#95a5a6",
                  "mgmt":"#bdc3c7"}

    if subcarrier is not None:
        fig, ax = plt.subplots(figsize=(12, 4))
        for cls, rows in rows_by_class.items():
            rows = sorted(rows, key=lambda r: r["real_timestamp"])
            t0 = rows[0]["real_timestamp"]
            ts  = [r["real_timestamp"] - t0 for r in rows]
            amp = [r["amplitudes"][subcarrier]
                   if subcarrier < len(r["amplitudes"]) else float("nan")
                   for r in rows]
            ax.scatter(ts, amp, s=4, label=cls,
                       color=cls_colors.get(cls, "#999999"))
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Amplitude (subcarrier {subcarrier})")
        ax.set_title(f"Subcarrier {subcarrier} over time by comm_class")
        ax.legend()
        plt.tight_layout()

    fig2, ax2 = plt.subplots(figsize=(12, 5))
    for cls in COMM_CLASS_ORDER:
        if cls not in rows_by_class:
            continue
        rows = rows_by_class[cls]
        min_len = min(len(r["amplitudes"]) for r in rows)
        matrix  = np.array([r["amplitudes"][:min_len] for r in rows])
        mean_a  = matrix.mean(axis=0)
        std_a   = matrix.std(axis=0)
        x = range(min_len)
        color = cls_colors.get(cls, "#999999")
        ax2.plot(x, mean_a, label=cls, color=color)
        ax2.fill_between(x, mean_a - std_a, mean_a + std_a, alpha=0.15, color=color)

    ax2.set_xlabel("Subcarrier index")
    ax2.set_ylabel("Mean amplitude ± 1 std")
    ax2.set_title("Mean CSI amplitude spectrum per comm_class (automatic classification)")
    ax2.legend()
    plt.tight_layout()
    plt.show()


# ---- Per-MAC summary -------------------------------------------------------

def print_mac_summary(rows_all):
    """Print a per-MAC table to help identify the hotspot BSSID."""
    from collections import defaultdict

    mac_info = defaultdict(lambda: {
        "count": 0, "rssi_sum": 0, "max_sig_len": 0, "ht_frames": 0,
        "classes": defaultdict(int), "n_up": 0, "n_down": 0, "n_mgmt": 0,
    })

    for r in rows_all:
        m = mac_info[r["mac"]]
        m["count"] += 1
        m["rssi_sum"] += r["rssi"]
        if r["sig_len"] > m["max_sig_len"]:
            m["max_sig_len"] = r["sig_len"]
        if r["sig_mode"] > 0:
            m["ht_frames"] += 1
        m["classes"][r["comm_class"]] += 1
        if r["direction"] == "uplink":
            m["n_up"] += 1
        elif r["direction"] == "downlink":
            m["n_down"] += 1
        if r["comm_class"] == "mgmt":
            m["n_mgmt"] += 1

    total_ht = sum(m["ht_frames"] for m in mac_info.values())

    print("\n=== Per-MAC summary (use this to find your hotspot BSSID) ===\n")
    print(f"{'MAC':<19s} {'Frames':>6s} {'HT':>5s} {'RSSI':>5s} {'MaxLen':>6s} "
          f"{'Up':>4s} {'Down':>4s} {'Mgmt':>5s}  Top classes")
    print("-" * 85)

    for mac in sorted(mac_info, key=lambda m: mac_info[m]["count"], reverse=True):
        m = mac_info[mac]
        avg_rssi = m["rssi_sum"] // m["count"] if m["count"] else 0
        top_cls = sorted(m["classes"].items(), key=lambda x: -x[1])
        cls_str = ", ".join(f"{c}={n}" for c, n in top_cls[:4])
        print(f"{mac:<19s} {m['count']:>6d} {m['ht_frames']:>5d} {avg_rssi:>5d} "
              f"{m['max_sig_len']:>6d} {m['n_up']:>4d} {m['n_down']:>4d} "
              f"{m['n_mgmt']:>5d}  {cls_str}")

    print("-" * 85)

    if total_ht == 0:
        print("WARNING: No HT/VHT frames in this capture (HT column all zeros)!")
        print("  You only captured legacy-rate frames (beacons, keepalives).")
        print("  The real data traffic was likely using HT40 which the ESP32 missed.")
        print("  Re-capture with: BANDWIDTH: 40above  or run SCAN (tests all modes).")
    else:
        print(f"HT column = 802.11n/ac frames ({total_ht} total). "
              f"Your hotspot has strong RSSI + high HT count.")

    print("Your laptop MAC shows uplink frames. "
          "Hotspot BSSID shows mgmt (beacons) + data.")
    print()


# ---- File output -----------------------------------------------------------

def write_filtered(rows, out_path, header_line):
    with open(out_path, "w") as fh:
        if header_line:
            fh.write(header_line + "\n")
        for r in rows:
            fh.write(r["raw"] + "\n")
    print(f"Wrote {len(rows)} rows to '{out_path}'")


# ---- Main ------------------------------------------------------------------

def main():
    args = parse_args()

    header_line = None
    with open(args.csv_file, errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("type,"):
                header_line = line
                break

    rows_all = list(load_csv(
        args.csv_file,
        watch_macs=args.macs,
        comm_class_filter=args.comm_class,
        env_label_filter=args.env_label,
    ))

    if not rows_all:
        print("No rows matched the given filters.", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(rows_all)} CSI frames.")

    rows_by_class = defaultdict(list)
    for r in rows_all:
        rows_by_class[r["comm_class"]].append(r)

    if args.listmacs:
        print_mac_summary(rows_all)

    if args.stats:
        stats = compute_stats(rows_by_class)
        print_stats(stats, rows_by_class)

    if args.out:
        write_filtered(rows_all, args.out, header_line)

    if args.analyze:
        plot_isac_analysis(rows_all, window_sec=args.window)

    if args.timeline:
        plot_csi_timeline(rows_all)

    if args.plot:
        plot_mean_spectrum(rows_by_class, args.subcarrier)


if __name__ == "__main__":
    main()
