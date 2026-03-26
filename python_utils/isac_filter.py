#!/usr/bin/env python3
"""
isac_filter.py — ISAC post-processing utility for ESP32 CSI Tool (passive mode)
================================================================================
Filters a collected CSI CSV by MAC address and/or activity label, then shows
per-activity amplitude statistics and an optional comparison plot.

Usage examples
--------------
# Show per-activity amplitude statistics for two MACs (PC + router):
python isac_filter.py data.csv \
    --macs AA:BB:CC:DD:EE:FF 11:22:33:44:55:66

# Filter only rows tagged "youtube" and save to a new CSV:
python isac_filter.py data.csv --activity youtube --out youtube.csv

# Plot subcarrier amplitude per activity (requires matplotlib + numpy):
python isac_filter.py data.csv --plot

CSV column layout (26 fixed columns before the CSI bracket):
  0  type | 1  role | 2  mac | 3  rssi | 4  rate | 5  sig_mode | 6  mcs
  7  bandwidth | 8  smoothing | 9  not_sounding | 10 aggregation
  11 stbc | 12 fec_coding | 13 sgi | 14 noise_floor | 15 ampdu_cnt
  16 channel | 17 secondary_channel | 18 local_timestamp | 19 ant
  20 sig_len | 21 rx_state | 22 real_time_set | 23 real_timestamp
  24 len | 25 activity | 26 CSI_DATA (bracket column)
"""

import sys
import re
import math
import argparse
import csv
from collections import defaultdict

COL_TYPE      = 0
COL_ROLE      = 1
COL_MAC       = 2
COL_RSSI      = 3
COL_REAL_TS   = 23
COL_LEN       = 24
COL_ACTIVITY  = 25
COL_CSI       = 26  # "[im re im re ...]"


def parse_args():
    p = argparse.ArgumentParser(
        description="ISAC CSI filter and analyser",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("csv_file", help="Path to the CSI CSV collected from the ESP32")
    p.add_argument(
        "--macs", nargs="+", metavar="MAC",
        help="Keep only rows whose sender MAC is in this list (case-insensitive). "
             "Leave blank to keep all MACs.",
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
        help="Print per-activity amplitude statistics (default: on).",
    )
    p.add_argument(
        "--plot", action="store_true",
        help="Show a per-activity mean-amplitude plot (requires numpy + matplotlib).",
    )
    p.add_argument(
        "--subcarrier", type=int, default=None,
        help="When plotting, also show a time-series for this single subcarrier index.",
    )
    return p.parse_args()


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


def load_csv(path, watch_macs, activity_filter):
    """
    Yield filtered rows as dicts with keys:
      mac, rssi, real_timestamp, activity, amplitudes (list of floats)
    """
    watch_macs_upper = {normalise_mac(m) for m in (watch_macs or [])}

    with open(path, newline="") as fh:
        for raw_line in fh:
            line = raw_line.strip()
            if not line.startswith("CSI_DATA"):
                continue

            # Split carefully: the CSI bracket may contain spaces
            # Split on the first 27 commas only
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

            try:
                rssi = int(parts[COL_RSSI])
                real_ts = float(parts[COL_REAL_TS])
            except ValueError:
                continue

            amps = parse_csi_amplitudes(parts[COL_CSI])
            if not amps:
                continue

            yield {
                "mac": mac,
                "rssi": rssi,
                "real_timestamp": real_ts,
                "activity": activity if activity else "(unlabelled)",
                "amplitudes": amps,
                "raw": line,
            }


def compute_stats(rows_by_activity):
    """Return per-activity dict of {mean_amp, std_amp, count, mean_rssi}."""
    import statistics

    stats = {}
    for act, rows in rows_by_activity.items():
        all_amps = [a for r in rows for a in r["amplitudes"]]
        rssis = [r["rssi"] for r in rows]
        stats[act] = {
            "count": len(rows),
            "mean_amp": statistics.mean(all_amps),
            "std_amp": statistics.stdev(all_amps) if len(all_amps) > 1 else 0.0,
            "mean_rssi": statistics.mean(rssis),
        }
    return stats


def print_stats(stats, rows_by_activity):
    print("\n=== ISAC per-activity CSI statistics ===")
    for act in sorted(stats):
        s = stats[act]
        macs = {r["mac"] for r in rows_by_activity[act]}
        print(f"\n  Activity : '{act}'")
        print(f"  Frames   : {s['count']}")
        print(f"  MACs seen: {', '.join(sorted(macs))}")
        print(f"  Mean RSSI: {s['mean_rssi']:.1f} dBm")
        print(f"  Amp mean : {s['mean_amp']:.3f}")
        print(f"  Amp std  : {s['std_amp']:.3f}")
    print()


def plot_results(rows_by_activity, subcarrier=None):
    try:
        import numpy as np
        import matplotlib.pyplot as plt
    except ImportError:
        print("ERROR: numpy and matplotlib are required for --plot. "
              "Install with: pip install numpy matplotlib", file=sys.stderr)
        sys.exit(1)

    activities = sorted(rows_by_activity.keys())
    colors = plt.cm.tab10.colors

    if subcarrier is not None:
        # Time-series plot for one subcarrier
        fig, ax = plt.subplots(figsize=(12, 4))
        for idx, act in enumerate(activities):
            rows = rows_by_activity[act]
            ts = [r["real_timestamp"] for r in rows]
            amps = [r["amplitudes"][subcarrier]
                    if subcarrier < len(r["amplitudes"]) else float("nan")
                    for r in rows]
            ax.scatter(ts, amps, s=4, label=act, color=colors[idx % len(colors)])
        ax.set_xlabel("Timestamp (s)")
        ax.set_ylabel(f"Amplitude (subcarrier {subcarrier})")
        ax.set_title(f"ISAC — subcarrier {subcarrier} amplitude over time")
        ax.legend()
        plt.tight_layout()

    # Mean amplitude spectrum per activity
    fig2, ax2 = plt.subplots(figsize=(12, 5))
    for idx, act in enumerate(activities):
        rows = rows_by_activity[act]
        # Align all amplitude vectors to the shortest length
        min_len = min(len(r["amplitudes"]) for r in rows)
        matrix = np.array([r["amplitudes"][:min_len] for r in rows])
        mean_amp = matrix.mean(axis=0)
        ax2.plot(mean_amp, label=act, color=colors[idx % len(colors)])
    ax2.set_xlabel("Subcarrier index")
    ax2.set_ylabel("Mean amplitude")
    ax2.set_title("ISAC — mean CSI amplitude spectrum per activity")
    ax2.legend()
    plt.tight_layout()
    plt.show()


def write_filtered(rows, out_path, original_header):
    with open(out_path, "w") as fh:
        if original_header:
            fh.write(original_header + "\n")
        for r in rows:
            fh.write(r["raw"] + "\n")
    print(f"Wrote {len(rows)} rows to '{out_path}'")


def main():
    args = parse_args()

    # Read optional header line
    header_line = None
    with open(args.csv_file) as fh:
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

    if args.plot:
        plot_results(rows_by_activity, args.subcarrier)


if __name__ == "__main__":
    main()
