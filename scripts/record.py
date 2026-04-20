#!/usr/bin/env python3
"""
Record multi-channel scope data from a Rigol DHO series scope to CSV or NPZ.

Tested on DHO924S, firmware 00.01.05. Should work on other DHO800/900/1000/4000
series with minor or no modification.

Prerequisites on the scope:
  - Acquire -> Mem Depth -> set to a fixed value (e.g. 10M), not Auto
  - Channels you want captured are displayed and centered appropriately
  - Press Stop when you want to capture

Usage examples:
  # Record all enabled channels to scope.csv
  python record.py

  # Specify scope IP and output file
  python record.py --ip 192.168.1.50 --out mytest.csv

  # Custom channel labels (one per enabled channel, in CH order)
  python record.py --labels sin cos VAB VCB

  # NPZ output (10x smaller than CSV, much faster to load later)
  python record.py --out mytest.npz --format npz

  # Capture only specific channels, ignoring display state
  python record.py --channels 1,3

  # Verbose debug output
  python record.py --verbose
"""
import argparse
import csv
import sys
import time

import numpy as np
import pyvisa


DEFAULT_SCOPE_IP = "192.168.11.159"
DEFAULT_OUTFILE = "scope.csv"
WORD_CHUNK = 125000  # 250 kB per query; Rigol caps single-shot WAV:DATA?


def query_float(scope, cmd):
    return float(scope.query(cmd).strip())


def get_enabled_channels(scope):
    """Query which channels are currently enabled on the scope display."""
    enabled = []
    for ch in (1, 2, 3, 4):
        resp = scope.query(f":CHAN{ch}:DISP?").strip()
        # Rigol returns "1" or "ON" for enabled, "0" or "OFF" for disabled
        if resp in ("1", "ON") or resp.upper() == "ON":
            enabled.append(ch)
    return enabled


def dump_raw_header(scope, ch, verbose=False):
    """Peek at the first few bytes coming back to confirm WORD vs BYTE format."""
    scope.write(f":WAV:SOUR CHAN{ch}")
    time.sleep(0.05)
    scope.write(":WAV:FORM WORD")
    time.sleep(0.05)
    scope.write(":WAV:STAR 1")
    scope.write(":WAV:STOP 10")
    time.sleep(0.05)
    scope.write(":WAV:DATA?")
    raw = scope.read_raw()
    if verbose:
        print(f"  Raw header bytes: {raw[:20].hex()}")

    # Parse TMC header: #N<N digits of length>
    if raw[0:1] != b"#":
        print(f"  WARNING: no # header, got {raw[:2]!r}")
        return None
    ndig = int(chr(raw[1]))
    length = int(raw[2:2 + ndig])
    if verbose:
        print(f"  TMC header says {length} bytes payload for 10 samples")
    if length == 20:
        if verbose:
            print("  -> WORD format confirmed (2 bytes/sample)")
        return "WORD"
    elif length == 10:
        if verbose:
            print("  -> BYTE format (1 byte/sample) - WORD didn't stick")
        return "BYTE"
    else:
        print(f"  -> Unexpected payload size: {length}")
        return None


def fetch_channel(scope, ch, total_points, verbose=False):
    """Fetch a single channel in WORD format (12-bit), chunked."""
    scope.write(f":WAV:SOUR CHAN{ch}")
    time.sleep(0.05)
    scope.write(":WAV:FORM WORD")
    time.sleep(0.05)

    fmt = scope.query(":WAV:FORM?").strip()
    if "WORD" not in fmt.upper():
        raise RuntimeError(f"CH{ch}: expected WORD format, got {fmt!r}")

    yinc = query_float(scope, ":WAV:YINC?")
    yorig = query_float(scope, ":WAV:YOR?")
    yref = query_float(scope, ":WAV:YREF?")
    if verbose:
        print(f"  CH{ch}: yinc={yinc:.6g}  yorig={yorig:.6g}  yref={yref:.6g}")

    if yref < 1000:
        print(f"  WARNING: yref={yref} looks wrong for WORD mode "
              f"(expected ~2048 or ~32768)")

    result = np.zeros(total_points, dtype=np.uint16)
    pos = 0
    start = 1
    while start <= total_points:
        stop = min(start + WORD_CHUNK - 1, total_points)
        scope.write(f":WAV:STAR {start}")
        scope.write(f":WAV:STOP {stop}")
        data = scope.query_binary_values(
            ":WAV:DATA?",
            datatype="H",
            container=np.ndarray,
            header_fmt="ieee",
            is_big_endian=False,
        )
        n = len(data)
        if n == 0:
            print(f"\n  WARNING: empty response at start={start}, stopping")
            break
        result[pos:pos + n] = data
        pos += n
        start += n
        pct = 100.0 * pos / total_points
        print(f"  CH{ch}: {pos}/{total_points} ({pct:.1f}%)", end="\r")
    print()

    if pos < total_points:
        result = result[:pos]

    if verbose:
        print(f"  CH{ch}: raw codes min={result.min()} max={result.max()} "
              f"mean={result.mean():.0f}")

    volts = (result.astype(np.float64) - yorig - yref) * yinc
    return volts


def parse_channels_arg(s):
    """Parse '1,2,4' or '1' into a list of ints."""
    if s is None:
        return None
    chans = []
    for part in s.split(","):
        part = part.strip()
        if part:
            n = int(part)
            if n < 1 or n > 4:
                raise argparse.ArgumentTypeError(f"Channel must be 1-4, got {n}")
            chans.append(n)
    return chans


def write_csv(outfile, t_arr, channel_data, labels):
    """Write time + labeled channel columns to CSV."""
    header = ["t"] + labels
    with open(outfile, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        # Build row iterators to avoid materializing the full matrix
        arrays = [t_arr] + list(channel_data)
        for row in zip(*arrays):
            w.writerow(row)


def write_npz(outfile, t_arr, channel_data, labels):
    """Write time + channels to compressed NPZ."""
    kwargs = {"t": t_arr}
    for label, data in zip(labels, channel_data):
        kwargs[label] = data
    np.savez_compressed(outfile, **kwargs)


def main():
    ap = argparse.ArgumentParser(
        description="Record Rigol DHO scope waveforms to CSV or NPZ.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("Usage examples:")[1] if "Usage examples:" in __doc__ else None,
    )
    ap.add_argument("--ip", default=DEFAULT_SCOPE_IP,
                    help=f"Scope IP address (default: {DEFAULT_SCOPE_IP})")
    ap.add_argument("--out", default=DEFAULT_OUTFILE,
                    help=f"Output filename (default: {DEFAULT_OUTFILE})")
    ap.add_argument("--format", choices=["csv", "npz", "auto"], default="auto",
                    help="Output format. 'auto' infers from --out extension (default: auto)")
    ap.add_argument("--channels", type=parse_channels_arg, default=None,
                    help="Comma-separated channel list (e.g. '1,3'). "
                         "Default: auto-detect enabled channels.")
    ap.add_argument("--labels", nargs="+", default=None,
                    help="Column labels for enabled channels, in CH order. "
                         "Default: ch1, ch2, ... Example: --labels sin cos VAB VCB")
    ap.add_argument("--verbose", "-v", action="store_true",
                    help="Print extra debug info (yref/yinc, raw headers).")

    # Legacy positional support: record.py [ip] [outfile]
    # argparse doesn't handle mixed positional+flagged cleanly, so we sniff argv
    # before parsing if no flags are used.
    if len(sys.argv) >= 2 and not sys.argv[1].startswith("-"):
        # Positional mode for backwards compat
        positional = []
        while len(sys.argv) > 1 and not sys.argv[1].startswith("-"):
            positional.append(sys.argv.pop(1))
        if len(positional) >= 1:
            sys.argv.extend(["--ip", positional[0]])
        if len(positional) >= 2:
            sys.argv.extend(["--out", positional[1]])

    args = ap.parse_args()

    # Determine output format
    out_format = args.format
    if out_format == "auto":
        if args.out.lower().endswith(".npz"):
            out_format = "npz"
        else:
            out_format = "csv"

    rm = pyvisa.ResourceManager("@py")
    scope = rm.open_resource(f"TCPIP::{args.ip}::INSTR")
    scope.timeout = 60000
    scope.chunk_size = 1024 * 1024

    try:
        idn = scope.query("*IDN?").strip()
        print(f"Connected: {idn}")

        # Setup order matters on DHO firmware
        scope.write(":STOP")
        time.sleep(0.3)
        scope.write(":WAV:FORM WORD")
        time.sleep(0.1)
        scope.write(":WAV:MODE RAW")
        time.sleep(0.1)
        # Re-assert format after MODE change in case it got clobbered
        scope.write(":WAV:FORM WORD")
        time.sleep(0.1)

        fmt = scope.query(":WAV:FORM?").strip()
        mode = scope.query(":WAV:MODE?").strip()
        print(f"Waveform format: {fmt}, mode: {mode}")
        if "WORD" not in fmt.upper():
            raise RuntimeError(f"WORD format not set, got {fmt!r}")

        mdepth = scope.query(":ACQ:MDEP?").strip()
        print(f"Acquire memory depth: {mdepth}")
        if mdepth.upper() == "AUTO":
            print(">>> Memory depth is AUTO. Set to a fixed value (e.g. 10M)")
            print(">>> on the scope's Acquire menu for consistent captures.")
        else:
            try:
                md = int(float(mdepth))
                if md < 100000:
                    print(f">>> Memory depth {md} is low. Increase via Acquire menu.")
            except Exception:
                pass

        # Decide which channels to pull
        if args.channels is not None:
            channels_to_fetch = args.channels
            print(f"Fetching user-specified channels: {channels_to_fetch}")
        else:
            channels_to_fetch = get_enabled_channels(scope)
            if not channels_to_fetch:
                raise RuntimeError("No channels enabled on scope. Enable at least one "
                                   "channel on the scope or use --channels.")
            print(f"Auto-detected enabled channels: {channels_to_fetch}")

        # Validate or build labels
        if args.labels is not None:
            if len(args.labels) != len(channels_to_fetch):
                raise ValueError(
                    f"--labels count ({len(args.labels)}) must match channel count "
                    f"({len(channels_to_fetch)}). Channels: {channels_to_fetch}"
                )
            labels = args.labels
        else:
            labels = [f"ch{c}" for c in channels_to_fetch]
        print(f"Column labels: {labels}")

        # Select first channel so :WAV:POIN? returns the real count
        scope.write(f":WAV:SOUR CHAN{channels_to_fetch[0]}")
        time.sleep(0.05)

        total_points = int(float(scope.query(":WAV:POIN?").strip()))
        xinc = query_float(scope, ":WAV:XINC?")
        xorig = query_float(scope, ":WAV:XOR?")
        fs = 1.0 / xinc
        duration = total_points * xinc
        print(f"Fetching {total_points} points/channel at {fs:.1f} Sa/s  "
              f"({duration*1000:.1f} ms record)")

        # Sanity-check the format with a 10-sample probe
        if args.verbose:
            print("\nProbing first channel for format sanity...")
        detected = dump_raw_header(scope, channels_to_fetch[0], verbose=args.verbose)
        if detected == "BYTE":
            raise RuntimeError(
                "Scope is returning BYTE data despite WORD request. "
                "Try power-cycling the scope, or update firmware."
            )

        print()
        t0 = time.time()
        channel_data = []
        for ch in channels_to_fetch:
            print(f"Fetching CH{ch} ...")
            channel_data.append(fetch_channel(scope, ch, total_points, verbose=args.verbose))
        elapsed = time.time() - t0
        print(f"\nFetched all channels in {elapsed:.1f} s")

        n = min(len(v) for v in channel_data)
        channel_data = [v[:n] for v in channel_data]
        t_arr = xorig + np.arange(n) * xinc

        print(f"\nWriting {n} rows to {args.out} (format: {out_format}) ...")
        if out_format == "csv":
            write_csv(args.out, t_arr, channel_data, labels)
        elif out_format == "npz":
            write_npz(args.out, t_arr, channel_data, labels)
        else:
            raise ValueError(f"Unknown output format: {out_format}")
        print(f"Done. Saved {args.out}")

        print("\nPer-channel stats (V):")
        for ch, label, v in zip(channels_to_fetch, labels, channel_data):
            print(f"  CH{ch} ({label}): min={v.min():+.4f}  max={v.max():+.4f}  "
                  f"mean={v.mean():+.5f}  rms={np.sqrt((v**2).mean()):.4f}")

    finally:
        scope.close()
        rm.close()


if __name__ == "__main__":
    main()
