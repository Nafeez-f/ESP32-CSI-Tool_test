#!/usr/bin/env python3
"""
isac_filter.py — ISAC post-processing utility for ESP32 CSI Tool (passive mode)
================================================================================

ISAC = Integrated Sensing and Communication.
The Wi-Fi frames that carry data (YouTube, file download, browsing) are
simultaneously used to estimate the physical channel state (CSI).  This tool
lets you visualise BOTH the communication signal AND the sensing signal from the
same stream of frames captured by the passive ESP32.

----------------------------------------------------------------------
Key fields for ISAC and what they represent
----------------------------------------------------------------------
  mac           Who sent the frame.
                  router BSSID → downlink frame (data flowing TO your PC)
                  PC Wi-Fi MAC → uplink frame (ACKs / requests FROM your PC)

  sig_len       Physical layer frame size in bytes.
                  ~1500 during active YouTube download (near Ethernet MTU)
                  ~14   during idle (only tiny ACKs)
                  → Use this as the per-frame communication load indicator.

  ampdu_cnt     Number of MPDUs aggregated into this A-MPDU transmission.
                  0-1 during idle, 8-32+ during heavy download.
                  → Multiplied by sig_len gives a rough bytes-on-air count.

  rate / mcs    Modulation and Coding Scheme index.
                  Higher = better channel conditions = faster throughput.
                  → Rises during active streaming when the channel is clear.

  rssi          Received signal strength (dBm).
                  → Coarse proxy for distance / obstruction.

  noise_floor   Background noise at the moment of reception (dBm).

  real_timestamp Seconds since epoch (steady clock).
                  Difference between consecutive rows = inter-frame interval.
                  → Shorter IFI = more data flowing = higher CSI sampling rate.
                  This is the key ISAC link: MORE COMMUNICATION → MORE CSI SAMPLES
                  → BETTER SENSING TEMPORAL RESOLUTION.

  CSI_DATA      Complex channel coefficients: interleaved [Im0 Re0 Im1 Re1 ...]
                  Amplitude[k] = sqrt(Im_k^2 + Re_k^2)  (multipath strength)
                  Phase[k]     = atan2(Im_k, Re_k)       (multipath delay)
                  → The sensing signal. Changes when someone moves in the space.

  activity      Your runtime label (TAG: command). Used to split data for
                  supervised learning or comparison plots.

----------------------------------------------------------------------
Usage examples
----------------------------------------------------------------------
# Filter to two MACs and print per-activity statistics:
python isac_filter.py data.csv --macs AA:BB:CC:DD:EE:FF 11:22:33:44:55:66

# THE ISAC PLOT — 3-panel communication+sensing time-domain view:
python isac_filter.py data.csv --analyze

# Restrict the ISAC analysis to downlink only (router MAC):
python isac_filter.py data.csv --macs 11:22:33:44:55:66 --analyze

# CSI heatmap over time (see which subcarriers change with activity):
python isac_filter.py data.csv --timeline

# Save only "youtube" rows:
python isac_filter.py data.csv --activity youtube --out youtube.csv

# Mean amplitude spectrum per activity (static comparison):
python isac_filter.py data.csv --plot

----------------------------------------------------------------------
CSV column layout (33 fixed columns before the CSI bracket)
----------------------------------------------------------------------
  0  type            1  role            2  mac
  3  rssi            4  rate            5  sig_mode
  6  mcs             7  bandwidth       8  smoothing
  9  not_sounding   10  aggregation    11  stbc
  12 fec_coding     13 sgi             14 noise_floor
  15 ampdu_cnt      16 channel         17 secondary_channel
  18 local_timestamp 19 ant            20 sig_len
  21 rx_state       22 real_time_set   23 real_timestamp
  24 len
  25 seq_num    ← 802.11 sequence number; gap = lost frame = missing CSI sample
  26 retry      ← 1=retransmission; high rate = bad channel / interference
  27 to_ds      ← 1=uplink frame (MacBook → router)
  28 from_ds    ← 1=downlink frame (router → MacBook)
  29 tid        ← QoS Traffic ID: 4/5=Video, 0/3=BestEffort, 1/2=Background, 6/7=Voice
  30 is_qos     ← 1=QoS data frame (always 1 for 802.11n/ac traffic)
  31 duration   ← NAV channel reservation in µs
  32 activity   ← your runtime TAG label
  33 CSI_DATA   ← bracket column [im re im re ...]
"""

import sys
import re
import math
import argparse
import statistics as _stats
from collections import defaultdict

# ---- Column indices ----------------------------------------------------------
COL_TYPE       = 0
COL_ROLE       = 1
COL_MAC        = 2
COL_RSSI       = 3
COL_RATE       = 4
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
COL_ACTIVITY   = 32
COL_CSI        = 33

TID_LABELS = {
    0: "BestEffort",  1: "Background",  2: "Background",  3: "BestEffort",
    4: "Video",       5: "Video",        6: "Voice",        7: "Voice",
}


# ---- CLI -------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(
        description="ISAC CSI filter and analyser for ESP32 passive mode",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("csv_file", help="Path to the CSI CSV collected from the ESP32")
    p.add_argument(
        "--macs", nargs="+", metavar="MAC",
        help="Keep only rows whose sender MAC is in this list (case-insensitive). "
             "Tip: use your router BSSID for downlink-only analysis.",
    )
    p.add_argument(
        "--activity", metavar="LABEL",
        help="Keep only rows whose activity label matches LABEL. "
             "Use 'unlabelled' to keep rows with no label.",
    )
    p.add_argument(
        "--out", metavar="FILE",
        help="Write filtered rows to this CSV file.",
    )
    p.add_argument(
        "--stats", action="store_true", default=True,
        help="Print per-activity statistics (default: on).",
    )
    p.add_argument(
        "--analyze", action="store_true",
        help="ISAC analysis: 3-panel plot of communication load, CSI variation, "
             "and CSI sampling rate over time. Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--plot", action="store_true",
        help="Mean amplitude spectrum per activity (static comparison). "
             "Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--timeline", action="store_true",
        help="CSI amplitude heatmap over time — see which subcarriers change "
             "with activity. Requires numpy + matplotlib.",
    )
    p.add_argument(
        "--subcarrier", type=int, default=None,
        help="When using --plot, also show a time-series for this subcarrier index.",
    )
    p.add_argument(
        "--window", type=float, default=1.0,
        help="Bin width in seconds for the --analyze plot (default: 1.0).",
    )
    return p.parse_args()


# ---- CSV parsing -----------------------------------------------------------

def parse_csi_amplitudes(csi_col):
    """Return list of per-subcarrier amplitudes from the raw bracket column."""
    m = re.search(r"\[([^\]]*)\]", csi_col)
    if not m:
        return []
    raw = [int(x) for x in m.group(1).split() if x]
    amps = []
    for i in range(0, len(raw) - 1, 2):
        amps.append(math.sqrt(raw[i] ** 2 + raw[i + 1] ** 2))
    return amps


def normalise_mac(mac):
    return mac.strip().upper()


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


def load_csv(path, watch_macs, activity_filter):
    """
    Yield filtered rows as dicts with all parsed fields.
    """
    watch_macs_upper = {normalise_mac(m) for m in (watch_macs or [])}

    with open(path, newline="", errors="replace") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line.startswith("CSI_DATA"):
                continue

            # Split on first (COL_CSI+1) commas; the CSI bracket contains spaces
            parts = line.split(",", COL_CSI + 1)
            if len(parts) < COL_CSI + 1:
                continue

            mac = normalise_mac(parts[COL_MAC])
            if watch_macs_upper and mac not in watch_macs_upper:
                continue

            activity = parts[COL_ACTIVITY].strip()
            if activity_filter is not None:
                want = "" if activity_filter.lower() == "unlabelled" else activity_filter
                if activity != want:
                    continue

            rssi      = _int_or(parts[COL_RSSI])
            rate      = _int_or(parts[COL_RATE])
            mcs       = _int_or(parts[COL_MCS])
            noise_fl  = _int_or(parts[COL_NOISE_FL])
            ampdu_cnt = _int_or(parts[COL_AMPDU_CNT])
            sig_len   = _int_or(parts[COL_SIG_LEN])
            real_ts   = _float_or(parts[COL_REAL_TS])
            # 802.11 MAC header fields (-1 = not yet cached on firmware side)
            seq_num   = _int_or(parts[COL_SEQ_NUM])
            retry     = _int_or(parts[COL_RETRY])
            to_ds     = _int_or(parts[COL_TO_DS])
            from_ds   = _int_or(parts[COL_FROM_DS])
            tid       = _int_or(parts[COL_TID])
            is_qos    = _int_or(parts[COL_IS_QOS])
            duration  = _int_or(parts[COL_DURATION])

            if math.isnan(real_ts):
                continue

            amps = parse_csi_amplitudes(parts[COL_CSI])
            if not amps:
                continue

            # Derive frame direction from ds bits
            if to_ds == 1 and from_ds == 0:
                direction = "uplink"
            elif to_ds == 0 and from_ds == 1:
                direction = "downlink"
            elif to_ds == 1 and from_ds == 1:
                direction = "wds"       # wireless distribution system (rare)
            else:
                direction = "ibss"      # ad-hoc / unknown

            tid_label = TID_LABELS.get(tid, "unknown") if tid >= 0 else "unknown"

            yield {
                "mac":           mac,
                "rssi":          rssi,
                "rate":          rate,
                "mcs":           mcs,
                "noise_floor":   noise_fl,
                "ampdu_cnt":     ampdu_cnt,
                "sig_len":       sig_len,
                "real_timestamp": real_ts,
                # 802.11 MAC header
                "seq_num":       seq_num,
                "retry":         retry,
                "to_ds":         to_ds,
                "from_ds":       from_ds,
                "tid":           tid,
                "tid_label":     tid_label,
                "is_qos":        is_qos,
                "duration":      duration,
                "direction":     direction,
                # labels
                "activity":      activity if activity else "(unlabelled)",
                "amplitudes":    amps,
                "raw":           line,
            }


# ---- Statistics ------------------------------------------------------------

def compute_stats(rows_by_activity):
    stats = {}
    for act, rows in rows_by_activity.items():
        all_amps   = [a for r in rows for a in r["amplitudes"]]
        rssis      = [r["rssi"] for r in rows]
        sig_lens   = [r["sig_len"] for r in rows]
        mcs_vals   = [r["mcs"] for r in rows]
        retry_vals = [r["retry"] for r in rows if r["retry"] >= 0]
        tid_vals   = [r["tid"] for r in rows if r["tid"] >= 0]

        # Direction breakdown
        n_up   = sum(1 for r in rows if r["direction"] == "uplink")
        n_down = sum(1 for r in rows if r["direction"] == "downlink")

        # TID label breakdown
        tid_counts = {}
        for r in rows:
            lbl = r["tid_label"]
            tid_counts[lbl] = tid_counts.get(lbl, 0) + 1

        # Sequence number gap analysis (detect lost frames = missing CSI)
        seq_valid = [r["seq_num"] for r in sorted(rows, key=lambda x: x["real_timestamp"])
                     if r["seq_num"] >= 0]
        lost_frames = 0
        for i in range(1, len(seq_valid)):
            gap = (seq_valid[i] - seq_valid[i-1]) % 4096
            if 1 < gap < 100:   # >1 = gap; <100 = not a wrap or unrelated frame
                lost_frames += gap - 1

        stats[act] = {
            "count":        len(rows),
            "mean_amp":     _stats.mean(all_amps),
            "std_amp":      _stats.stdev(all_amps) if len(all_amps) > 1 else 0.0,
            "mean_rssi":    _stats.mean(rssis),
            "mean_sig_len": _stats.mean(sig_lens),
            "mean_mcs":     _stats.mean(mcs_vals),
            "retry_rate":   (_stats.mean(retry_vals) * 100) if retry_vals else -1,
            "n_uplink":     n_up,
            "n_downlink":   n_down,
            "tid_counts":   tid_counts,
            "lost_frames":  lost_frames,
        }
    return stats


def print_stats(stats, rows_by_activity):
    print("\n=== ISAC per-activity statistics ===")
    for act in sorted(stats):
        s = stats[act]
        rows = rows_by_activity[act]
        macs = {r["mac"] for r in rows}

        # Estimate inter-frame interval
        ts_sorted = sorted(r["real_timestamp"] for r in rows)
        if len(ts_sorted) > 1:
            ifis = [ts_sorted[i+1] - ts_sorted[i] for i in range(len(ts_sorted)-1)]
            mean_ifi = _stats.mean(ifis)
            csi_rate = 1.0 / mean_ifi if mean_ifi > 0 else 0
        else:
            mean_ifi = float("nan")
            csi_rate = float("nan")

        tid_str = ", ".join(f"{k}:{v}" for k, v in sorted(s["tid_counts"].items(),
                                                            key=lambda x: -x[1]))
        print(f"\n  Activity       : '{act}'")
        print(f"  Frames         : {s['count']}")
        print(f"  MACs seen      : {', '.join(sorted(macs))}")
        print(f"  Direction      : {s['n_downlink']} downlink (router→device), "
              f"{s['n_uplink']} uplink (device→router)")
        print(f"  Traffic type   : {tid_str}")
        print(f"    TID key: Video=4/5, BestEffort=0/3, Background=1/2, Voice=6/7")
        print(f"  Retry rate     : {s['retry_rate']:.1f}%"
              f"  ← high = bad channel / interference / congestion")
        print(f"  Lost frames    : {s['lost_frames']}"
              f"  ← sequence number gaps = missing CSI samples")
        print(f"  Mean RSSI      : {s['mean_rssi']:.1f} dBm")
        print(f"  Mean sig_len   : {s['mean_sig_len']:.0f} bytes"
              f"  ← ~1460=active download, ~14=idle ACKs only")
        print(f"  Mean MCS       : {s['mean_mcs']:.1f}"
              f"          ← 0=1Mbps legacy, 7=65Mbps (802.11n best)")
        print(f"  Mean IFI       : {mean_ifi*1000:.1f} ms"
              f"      ← inter-frame interval")
        print(f"  CSI rate       : {csi_rate:.1f} samples/sec"
              f"  ← sensing resolution (driven by traffic)")
        print(f"  CSI amp mean   : {s['mean_amp']:.3f}")
        print(f"  CSI amp std    : {s['std_amp']:.3f}")
    print()


# ---- ISAC analysis plot (the key new visualisation) -----------------------

def plot_isac_analysis(rows, window_sec=1.0):
    """
    3-panel ISAC time-domain plot.

    Panel 1 — Communication load (bytes on air per second):
      sum(sig_len) per time bin.  Spikes during YouTube download,
      near-zero during idle.  This is the "C" in ISAC.

    Panel 2 — Channel sensing signal (CSI amplitude variation per second):
      std(mean_amplitude_per_frame) within each time bin.
      Rises when something physically changes in the space (person moving,
      objects shifting).  This is the "S" in ISAC.

    Panel 3 — CSI sampling rate (frames per second):
      Directly driven by communication traffic.  More data flowing →
      more frames → more CSI samples → better sensing resolution.
      This panel makes visible the ISAC coupling: communication enables sensing.

    How to read this plot for your YouTube experiment:
      - During active streaming: Panel 1 high, Panel 3 high.
      - When someone walks near the router+PC: Panel 2 spikes.
      - Both (1) and (2) are observable simultaneously from the same frames.
      That co-observation is ISAC.
    """
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("ERROR: numpy and matplotlib required. pip install numpy matplotlib",
              file=sys.stderr)
        sys.exit(1)

    # Sort by timestamp and normalise to t=0
    rows = sorted(rows, key=lambda r: r["real_timestamp"])
    if len(rows) < 2:
        print("Not enough rows for ISAC analysis (need at least 2).", file=sys.stderr)
        return

    t0 = rows[0]["real_timestamp"]
    ts = np.array([r["real_timestamp"] - t0 for r in rows])
    sig_lens  = np.array([r["sig_len"]  for r in rows], dtype=float)
    ampdu_cnt = np.array([r["ampdu_cnt"] for r in rows], dtype=float)
    rssis     = np.array([r["rssi"]     for r in rows], dtype=float)
    # Mean amplitude per frame (single number summarising the CSI vector)
    mean_amps = np.array([
        sum(r["amplitudes"]) / len(r["amplitudes"]) for r in rows
    ])
    activities = [r["activity"] for r in rows]

    retries   = np.array([r["retry"]   for r in rows], dtype=float)
    tid_arr   = np.array([r["tid"]     for r in rows], dtype=float)

    max_t = float(ts[-1])
    bin_edges = np.arange(0, max_t + window_sec, window_sec)
    bin_centers = bin_edges[:-1] + window_sec / 2.0

    throughput    = []   # bytes per second (Panel 1)
    csi_variance  = []   # amplitude std per second (Panel 2)
    frame_rate    = []   # frames per second (Panel 3)
    retry_pct     = []   # retransmission % per second (Panel 4)
    video_pct     = []   # fraction of frames with Video TID (4 or 5) (Panel 1 overlay)

    for b_start, b_end in zip(bin_edges[:-1], bin_edges[1:]):
        mask = (ts >= b_start) & (ts < b_end)
        n = int(mask.sum())
        if n == 0:
            throughput.append(0.0)
            csi_variance.append(0.0)
            frame_rate.append(0)
            retry_pct.append(0.0)
            video_pct.append(0.0)
            continue

        throughput.append(float(sig_lens[mask].sum()))

        amps_in_bin = mean_amps[mask]
        csi_variance.append(float(np.std(amps_in_bin)) if n > 1 else 0.0)

        frame_rate.append(n)

        valid_retry = retries[mask]
        valid_retry = valid_retry[valid_retry >= 0]
        retry_pct.append(float(np.mean(valid_retry) * 100) if len(valid_retry) > 0 else 0.0)

        valid_tid = tid_arr[mask]
        valid_tid = valid_tid[valid_tid >= 0]
        vpct = float(np.mean((valid_tid == 4) | (valid_tid == 5)) * 100) if len(valid_tid) > 0 else 0.0
        video_pct.append(vpct)

    throughput   = np.array(throughput)
    csi_variance = np.array(csi_variance)
    frame_rate   = np.array(frame_rate, dtype=float)
    retry_pct    = np.array(retry_pct)
    video_pct    = np.array(video_pct)

    # Activity colour bands
    unique_acts = sorted(set(activities))
    cmap = plt.cm.tab10
    act_colors = {a: cmap(i / max(len(unique_acts), 1)) for i, a in enumerate(unique_acts)}

    fig, axes = plt.subplots(4, 1, figsize=(14, 13), sharex=True)
    fig.suptitle(
        "ISAC Analysis — Communication + Sensing from the same Wi-Fi frames",
        fontsize=13, fontweight="bold"
    )

    def add_activity_bands(ax):
        """Shade background by activity label."""
        if len(unique_acts) == 1 and "(unlabelled)" in unique_acts:
            return
        prev_act = activities[0]
        seg_start = ts[0]
        for i in range(1, len(ts)):
            if activities[i] != prev_act or i == len(ts) - 1:
                ax.axvspan(seg_start, ts[i], alpha=0.10,
                           color=act_colors[prev_act])
                prev_act = activities[i]
                seg_start = ts[i]

    # --- Panel 1: Communication load + Video TID overlay ---
    ax1_twin = axes[0].twinx()
    axes[0].bar(bin_centers, throughput / 1000.0, width=window_sec * 0.85,
                color="steelblue", alpha=0.7, label="KB/s")
    ax1_twin.plot(bin_centers, video_pct, color="navy", linewidth=1.2,
                  linestyle="--", label="Video TID %")
    add_activity_bands(axes[0])
    axes[0].set_ylabel("Throughput (KB/s)", fontsize=9)
    ax1_twin.set_ylabel("Video TID %", fontsize=9, color="navy")
    axes[0].set_title(
        "Panel 1 — Communication load  (sig_len sum per second)\n"
        "Dashed = % of frames with Video TID (4/5) — rises when YouTube is active",
        fontsize=9
    )
    axes[0].grid(axis="y", alpha=0.3)

    # --- Panel 2: CSI sensing signal ---
    axes[1].bar(bin_centers, csi_variance, width=window_sec * 0.85,
                color="darkorange", alpha=0.8)
    add_activity_bands(axes[1])
    axes[1].set_ylabel("CSI amp std dev", fontsize=9)
    axes[1].set_title(
        "Panel 2 — Channel dynamics / SENSING signal  (std of CSI amplitude per second)\n"
        "Spikes = something physically moved in the space (person, hand, door opening)",
        fontsize=9
    )
    axes[1].grid(axis="y", alpha=0.3)

    # --- Panel 3: Retry rate ---
    axes[2].bar(bin_centers, retry_pct, width=window_sec * 0.85,
                color="firebrick", alpha=0.8)
    add_activity_bands(axes[2])
    axes[2].set_ylabel("Retry %", fontsize=9)
    axes[2].set_title(
        "Panel 3 — Retransmission rate  (retry flag from 802.11 header)\n"
        "High = bad channel / interference / heavy congestion on the AP",
        fontsize=9
    )
    axes[2].grid(axis="y", alpha=0.3)

    # --- Panel 4: CSI sampling rate ---
    axes[3].bar(bin_centers, frame_rate, width=window_sec * 0.85,
                color="seagreen", alpha=0.8)
    add_activity_bands(axes[3])
    axes[3].set_ylabel("CSI samples/sec", fontsize=9)
    axes[3].set_title(
        "Panel 4 — CSI sampling rate  (number of frames per second)\n"
        "More communication → more frames → higher sensing temporal resolution"
        " — this is the ISAC coupling",
        fontsize=9
    )
    axes[3].set_xlabel("Time (seconds)", fontsize=10)
    axes[3].grid(axis="y", alpha=0.3)

    # Legend for activity bands
    if not (len(unique_acts) == 1 and "(unlabelled)" in unique_acts):
        patches = [mpatches.Patch(color=act_colors[a], alpha=0.4, label=a)
                   for a in unique_acts]
        axes[0].legend(handles=patches, loc="upper right", fontsize=8,
                       title="Activity labels")

    plt.tight_layout()

    # Print summary table
    print("\n=== ISAC time-bin summary ===")
    print(f"{'Time(s)':<9} {'KB/s':>7} {'VideoTID%':>9} {'CsiStd':>8} "
          f"{'Retry%':>7} {'fps':>5}  Activity")
    print("-" * 60)
    for i, bc in enumerate(bin_centers):
        mask = (ts >= bin_edges[i]) & (ts < bin_edges[i+1])
        if mask.sum() > 0:
            acts_in_bin = [activities[j] for j in range(len(ts)) if mask[j]]
            dom_act = max(set(acts_in_bin), key=acts_in_bin.count)
        else:
            dom_act = ""
        print(f"{bc:<9.1f} {throughput[i]/1000:>7.1f} {video_pct[i]:>9.1f} "
              f"{csi_variance[i]:>8.3f} {retry_pct[i]:>7.1f} "
              f"{frame_rate[i]:>5.0f}  {dom_act}")

    plt.show()


# ---- CSI timeline heatmap --------------------------------------------------

def plot_csi_timeline(rows):
    """
    Heatmap: X axis = time, Y axis = subcarrier index, colour = amplitude.
    Shows exactly which subcarriers change when and correlates with activity.
    Useful for identifying which subcarriers are most sensitive to your target.
    """
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: numpy and matplotlib required. pip install numpy matplotlib",
              file=sys.stderr)
        sys.exit(1)

    rows = sorted(rows, key=lambda r: r["real_timestamp"])
    if not rows:
        return

    t0 = rows[0]["real_timestamp"]
    min_len = min(len(r["amplitudes"]) for r in rows)
    ts    = np.array([r["real_timestamp"] - t0 for r in rows])
    matrix = np.array([r["amplitudes"][:min_len] for r in rows]).T  # (subcarriers, time)

    activities = [r["activity"] for r in rows]
    unique_acts = sorted(set(activities))
    cmap_acts = plt.cm.tab10
    act_colors = {a: cmap_acts(i / max(len(unique_acts), 1)) for i, a in enumerate(unique_acts)}

    fig, axes = plt.subplots(2, 1, figsize=(14, 8),
                              gridspec_kw={"height_ratios": [4, 1]})

    # Heatmap
    im = axes[0].imshow(
        matrix, aspect="auto", origin="lower",
        extent=[ts[0], ts[-1], 0, min_len],
        cmap="viridis", interpolation="nearest"
    )
    axes[0].set_ylabel("Subcarrier index")
    axes[0].set_title(
        "CSI Amplitude Heatmap over Time\n"
        "Vertical bands of change = physical event in the channel"
    )
    fig.colorbar(im, ax=axes[0], label="Amplitude")

    # Activity colour strip
    for i in range(len(ts) - 1):
        axes[1].axvspan(ts[i], ts[i+1], color=act_colors[activities[i]], alpha=0.7)
    axes[1].set_xlim(ts[0], ts[-1])
    axes[1].set_yticks([])
    axes[1].set_xlabel("Time (seconds)")
    axes[1].set_title("Activity label")

    import matplotlib.patches as mpatches
    patches = [mpatches.Patch(color=act_colors[a], label=a) for a in unique_acts]
    axes[1].legend(handles=patches, loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.show()


# ---- Mean spectrum per activity -------------------------------------------

def plot_mean_spectrum(rows_by_activity, subcarrier=None):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: numpy and matplotlib required. pip install numpy matplotlib",
              file=sys.stderr)
        sys.exit(1)

    activities = sorted(rows_by_activity.keys())
    colors = plt.cm.tab10.colors

    if subcarrier is not None:
        fig, ax = plt.subplots(figsize=(12, 4))
        for idx, act in enumerate(activities):
            rows = sorted(rows_by_activity[act], key=lambda r: r["real_timestamp"])
            t0 = rows[0]["real_timestamp"]
            ts = [r["real_timestamp"] - t0 for r in rows]
            amps = [r["amplitudes"][subcarrier]
                    if subcarrier < len(r["amplitudes"]) else float("nan")
                    for r in rows]
            ax.scatter(ts, amps, s=4, label=act, color=colors[idx % len(colors)])
        ax.set_xlabel("Time (s)")
        ax.set_ylabel(f"Amplitude (subcarrier {subcarrier})")
        ax.set_title(f"Subcarrier {subcarrier} amplitude over time by activity")
        ax.legend()
        plt.tight_layout()

    fig2, ax2 = plt.subplots(figsize=(12, 5))
    for idx, act in enumerate(activities):
        rows = rows_by_activity[act]
        min_len = min(len(r["amplitudes"]) for r in rows)
        import numpy as np
        matrix = np.array([r["amplitudes"][:min_len] for r in rows])
        mean_amp = matrix.mean(axis=0)
        std_amp  = matrix.std(axis=0)
        x = range(min_len)
        ax2.plot(x, mean_amp, label=act, color=colors[idx % len(colors)])
        ax2.fill_between(x, mean_amp - std_amp, mean_amp + std_amp,
                         alpha=0.15, color=colors[idx % len(colors)])
    ax2.set_xlabel("Subcarrier index")
    ax2.set_ylabel("Mean amplitude ± 1 std")
    ax2.set_title("Mean CSI amplitude spectrum per activity\n"
                  "(shaded = ±1 standard deviation across frames)")
    ax2.legend()
    plt.tight_layout()
    plt.show()


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

    rows_all = list(load_csv(args.csv_file, args.macs, args.activity))

    if not rows_all:
        print("No rows matched the given filters.", file=sys.stderr)
        sys.exit(1)

    print(f"Matched {len(rows_all)} CSI frames.")

    rows_by_activity = defaultdict(list)
    for r in rows_all:
        rows_by_activity[r["activity"]].append(r)

    if args.stats:
        stats = compute_stats(rows_by_activity)
        print_stats(stats, rows_by_activity)

    if args.out:
        write_filtered(rows_all, args.out, header_line)

    if args.analyze:
        plot_isac_analysis(rows_all, window_sec=args.window)

    if args.timeline:
        plot_csi_timeline(rows_all)

    if args.plot:
        plot_mean_spectrum(rows_by_activity, args.subcarrier)


if __name__ == "__main__":
    main()
