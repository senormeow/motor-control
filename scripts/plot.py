#!/usr/bin/env python3
"""
Display recorded scope waveforms from CSV or NPZ files produced by record.py.

Usage:
  python plot.py scope.csv
  python plot.py mytest.npz
  python plot.py scope.csv --channels ch1 ch3
  python plot.py scope.csv --tstart -0.001 --tend 0.001
"""
import argparse
import sys

import numpy as np
import matplotlib
for _backend in ("TkAgg", "Qt5Agg", "GTK3Agg", "WebAgg"):
    try:
        matplotlib.use(_backend)
        break
    except Exception:
        continue
import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, newline="") as f:
        header = f.readline().strip().split(",")
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    return header, data


def load_npz(path):
    d = np.load(path)
    keys = list(d.keys())
    # put 't' first if present
    if "t" in keys:
        keys.remove("t")
        keys = ["t"] + keys
    data = np.column_stack([d[k] for k in keys])
    return keys, data


def main():
    ap = argparse.ArgumentParser(description="Plot recorded scope waveforms.")
    ap.add_argument("file", help="CSV or NPZ file from record.py")
    ap.add_argument("--channels", nargs="+", default=None,
                    help="Column names to plot (default: all except t)")
    ap.add_argument("--tstart", type=float, default=None,
                    help="Start time in seconds")
    ap.add_argument("--tend", type=float, default=None,
                    help="End time in seconds")
    ap.add_argument("--title", default=None,
                    help="Plot title (default: filename)")
    args = ap.parse_args()

    if args.file.lower().endswith(".npz"):
        header, data = load_npz(args.file)
    else:
        header, data = load_csv(args.file)

    if "t" not in header:
        print("ERROR: no 't' column found in file", file=sys.stderr)
        sys.exit(1)

    t_idx = header.index("t")
    t = data[:, t_idx]
    channel_names = [h for h in header if h != "t"]
    channel_cols = [header.index(h) for h in channel_names]

    if args.channels:
        missing = [c for c in args.channels if c not in channel_names]
        if missing:
            print(f"ERROR: unknown channels: {missing}. Available: {channel_names}",
                  file=sys.stderr)
            sys.exit(1)
        channel_names = args.channels
        channel_cols = [header.index(c) for c in channel_names]

    # Time window mask
    mask = np.ones(len(t), dtype=bool)
    if args.tstart is not None:
        mask &= t >= args.tstart
    if args.tend is not None:
        mask &= t <= args.tend
    t = t[mask]

    fig, axes = plt.subplots(len(channel_names), 1, sharex=True,
                             figsize=(12, 2.5 * len(channel_names)))
    if len(channel_names) == 1:
        axes = [axes]

    fig.suptitle(args.title or args.file)

    for ax, name, col in zip(axes, channel_names, channel_cols):
        v = data[mask, col]
        ax.plot(t * 1e3, v, linewidth=0.6)
        ax.set_ylabel(f"{name}\n(V)")
        ax.grid(True, alpha=0.3)
        stats = f"min={v.min():+.3f}  max={v.max():+.3f}  mean={v.mean():+.4f}  rms={np.sqrt((v**2).mean()):.4f}"
        ax.set_title(stats, fontsize=8, loc="right")

    axes[-1].set_xlabel("Time (ms)")
    fig.tight_layout()
    try:
        plt.show()
    except Exception:
        out = args.file.rsplit(".", 1)[0] + "_plot.png"
        fig.savefig(out, dpi=150)
        print(f"Saved to {out}")


if __name__ == "__main__":
    main()
