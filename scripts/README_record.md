# Rigol DHO Scope Recorder

Captures multi-channel waveform data from a Rigol DHO-series scope to CSV or
NPZ. Tested on DHO924S with firmware 00.01.05; should work on other DHO800/
900/1000/4000 series scopes with minor or no modification.

## Dependencies

    pip install pyvisa pyvisa-py numpy

Uses the `pyvisa-py` pure-Python backend — no NI-VISA required.

## Quick start

1. Set up the scope (see [Scope setup](#scope-setup))
2. Spin up whatever you're measuring, press Stop on the scope
3. `python record.py` → saves `scope.csv` in the current directory

## Usage

    python record.py [options]

### Options

| Flag | Description | Default |
|------|-------------|---------|
| `--ip ADDR` | Scope IP address | `192.168.11.159` |
| `--out FILE` | Output filename | `scope.csv` |
| `--format {csv,npz,auto}` | Output format (auto infers from extension) | `auto` |
| `--channels LIST` | Comma-separated channel list (e.g. `1,3`) | auto-detect enabled |
| `--labels NAME [NAME ...]` | Column labels, one per channel in CH order | `ch1`, `ch2`, … |
| `--verbose`, `-v` | Print extra diagnostic info | off |

### Legacy positional form

For backwards compatibility, `python record.py [ip] [outfile]` still works:

    python record.py 192.168.1.50 mytest.csv

## Examples

**Default capture** — all enabled channels to `scope.csv`:

    python record.py

**Custom IP and output**:

    python record.py --ip 192.168.1.50 --out bench_run.csv

**Named channels** (resolver + back-EMF measurement):

    python record.py --labels sin cos VAB VCB

**Subset of channels** (only CH1 and CH3):

    python record.py --channels 1,3 --labels gate_high phase_voltage

**NPZ format** for large captures (10× smaller, faster to load):

    python record.py --out switching_test.npz

**Verbose mode** for debugging scope communication:

    python record.py -v

## Output formats

### CSV

Text, human-readable, easy to inspect. First column is `t` (seconds),
subsequent columns are per-channel voltages using the provided labels.

    t,sin,cos,VAB,VCB
    -1.0000000,-0.14267,0.00507,0.02667,-0.06133
    -0.9999980,-0.14240,0.00533,0.02640,-0.06160
    ...

At 10M points × 4 channels: ~500 MB file, 2-3 minutes to write.
Fine for small captures; use NPZ for anything over 1M points.

### NPZ

NumPy's compressed archive format. Binary, typed, ~10× smaller than CSV.

    # Write:
    python record.py --out mytest.npz

    # Read:
    import numpy as np
    data = np.load("mytest.npz")
    t = data["t"]
    sin = data["sin"]
    # Access any labeled column by name

At 10M × 4ch: ~60 MB file, writes in seconds, loads in a fraction of a second.

## Scope setup

Do these on the scope before running the script. Some are critical; others
just improve capture quality.

### Critical

- **Memory depth**: Acquire → Mem Depth → fixed value (not Auto).
  10M works well for most tests. Script will warn if set below 100k.
- **Acquisition mode**: Normal (not Peak Detect, not Average).
- **Stop before capture**: press Stop so the waveform buffer is static.

### Recommended

- **Time base**: set for the capture duration you want. At 10M memory:
  - 100 ms/div → ~8 MSa/s, good for slow signals
  - 1 ms/div → 800 MSa/s, good for switching edges
  - 10 µs/div → 80 GSa/s (interpolated), gate drive analysis
- **Vertical**: center each enabled channel on 0V with signal present.
  Script can't fix badly-positioned channels — they'll just clip.
- **Channel display**: disable channels you don't want captured.
  The script auto-detects enabled channels and only pulls those.

### Scope channel probe setup

Script reads `yinc`, `yorig`, `yref` from the scope directly, so whatever
probe attenuation (×1, ×10, etc.) you've configured is applied automatically.
Voltage output is in volts at the probe tip.

## Channel detection

By default, the script queries `:CHAN{n}:DISP?` on each of the 4 channels
and only fetches channels that are currently displayed. This means:

- You control what gets captured from the scope front panel
- Disabled channels don't slow down the transfer
- If you want all 4 channels regardless of display state, use
  `--channels 1,2,3,4`

If no channels are enabled, the script errors out with a reminder.

## Known gotchas

### ROLL mode aliases fast signals
If your time base is slow enough to trigger ROLL mode (typically >100 ms/div
on a DHO), the scope reduces sample rate dramatically. Signals above ~1 kHz
will alias badly. For anything with AC content above audio frequencies,
stay out of ROLL mode.

### WORD format must be set before `:WAV:MODE RAW`
On DHO firmware 00.01.05, issuing `:WAV:MODE RAW` silently resets the
waveform format to BYTE. The script enforces the correct order and re-asserts
WORD afterward. A 10-byte sanity probe confirms the scope is actually
sending 16-bit data before committing to the full capture.

### Rigol chunked transfers
The scope caps single-query transfers at around 250k points. The script
uses 125k-point chunks (250 kB in WORD format) with `:WAV:STAR` / `:WAV:STOP`
range queries. For 10M × 4 channels this takes ~10 seconds.

### Transfer speed
Rough numbers at 1 Gbit LAN:
- 1 M points × 4 channels: ~2 seconds
- 10 M points × 4 channels: ~10 seconds  
- 50 M points × 4 channels: ~50 seconds

CSV writing is typically slower than the transfer itself for large captures
— use NPZ if transfer speed matters.

### Channel offset drift
Rigol DHO channels can have ~20 mV DC offset at the input. The script does
not null this; if you need zero-centered data, either:
- Run the scope's self-calibration (Utility → Self-Cal, probes disconnected)
- Null in post-processing: subtract each channel's mean over several full
  cycles of AC content

## SCPI commands used

    *IDN?                              # Identify scope
    :STOP                              # Halt acquisition
    :WAV:FORM WORD                     # 12-bit samples (set before MODE)
    :WAV:MODE RAW                      # Full memory, not just screen
    :ACQ:MDEP?                         # Query memory depth
    :CHAN{n}:DISP?                     # Is channel enabled?
    :WAV:SOUR CHAN{n}                  # Select channel for transfer
    :WAV:POIN?                         # Available point count
    :WAV:XINC?                         # Time per sample
    :WAV:XOR?                          # Time origin
    :WAV:YINC?                         # Volts per code
    :WAV:YOR?                          # Voltage origin
    :WAV:YREF?                         # Code corresponding to 0 V
    :WAV:STAR {n}                      # Start point for next transfer
    :WAV:STOP {n}                      # End point for next transfer
    :WAV:DATA?                         # Binary block transfer

These are standard for Rigol DHO800/900/1000/4000 and most MSO series.
Check your scope's programming manual if adapting to a different model.

## Voltage conversion

For the Rigol DHO series in WORD mode:

    volts = (code - yref - yorig) * yinc

- `code` is the raw 16-bit sample (0-65535, with center around 32768)
- `yref` is the code representing 0V on the display (typically 32768)
- `yorig` is the channel offset in codes (nonzero if you've shifted the trace)
- `yinc` is volts per code

This may differ slightly on other Rigol series (DS1000Z, MSO5000). Check
with a known DC signal and adjust the formula if readings are off.

## Adapting to other scope models

### Other Rigol DHO scopes
Should work as-is. The SCPI command set is stable across DHO800/900/1000/4000.

### Older Rigol (DS1000Z, MSO5000)
Most commands work, but check:
- WORD format behavior (may need different byte order)
- Memory depth command (may be `:ACQ:MEM` instead of `:ACQ:MDEP`)
- yref/yorig interpretation (older scopes use 8-bit encoding even in WORD mode)

### Non-Rigol
The overall flow (set format, set mode, iterate channels, chunked transfers)
applies to any LXI scope. Main adaptations:
- SCPI command names per manufacturer (Siglent, Keysight, Tektronix differ)
- Binary header format (most use IEEE 488.2 definite-length blocks)
- Voltage conversion formula

## Troubleshooting

**"WORD format not set, got 'BYTE'"**
The format didn't stick. Try adding more `time.sleep()` between the format
and mode commands, or power-cycle the scope. Some firmware versions are
pickier about command timing.

**"yref=128 looks wrong for WORD mode"**
The scope reported BYTE-format scaling constants despite being asked for
WORD. This happens when `:WAV:FORM WORD` is set after `:WAV:MODE RAW`
on older firmware. The script tries to guard against this, but if you see
this warning, check the firmware version and the command order.

**Transfer times out**
Increase `scope.timeout` (currently 60 seconds) at the top of `main()`.
Very large captures over slow networks can exceed this.

**Empty or short response from `:WAV:DATA?`**
The scope may not have acquired data (Run state, not Stop). Ensure Stop
was pressed before running the script. The script issues `:STOP` at start,
but if the scope wasn't armed, there won't be anything to transfer.

**Values look wrong (constant at ±yref*yinc)**
Channel is probably off-screen (trace above or below display area). The
scope clips code values to 0 or 65535 in those cases. Adjust vertical
position/scale on the scope so the signal is visible on screen.
