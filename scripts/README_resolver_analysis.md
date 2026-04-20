# EM57 Resolver-to-Electrical Angle Offset Analysis

Analyzes scope captures to determine the offset between resolver angle
and motor electrical angle for the Nissan Leaf EM57.

See [../scope/README.md](../scope/README.md) for how to capture the data.
See [../../docs/hardware/em57-motor.md](../../docs/hardware/em57-motor.md)
for the final offset value and its firmware application.

## Expected input

CSV from record.py with this specific channel layout:

| Column    | Source                                              |
|-----------|-----------------------------------------------------|
| ch1_sin   | Resolver sin channel from the motor                 |
| ch2_cos   | Resolver cos channel from the motor                 |
| ch3_VAB   | Scope probe: tip on phase A, ground on phase B      |
| ch4_VCB   | Scope probe: tip on phase C, ground on phase B      |

Capture conditions:
- Motor spun by hand (rotor rotating, no input power)
- At least 5 electrical cycles captured (500+ ms at slow hand-spin)
- Sample rate ≥40 kHz to resolve 10 kHz resolver carrier

## Usage

    python analyze_resolver.py [input.csv]

Default input: scope.csv

## Method

1. DC-null all channels (software offset correction)
2. Reconstruct V_A, V_B, V_C from line-to-line measurements
   assuming V_A + V_B + V_C = 0
3. Clarke transform → electrical angle from back-EMF
4. Pythagorean-identity demodulation of resolver envelopes
   (|env| = sqrt(signal² + (d_signal/dt/ω_carrier)²))
5. Sign recovery via alternating polarity at envelope zero crossings
6. Test all 4 polarity combinations, report lowest-std offset

## Expected output

    Offset ≈ ±90° with std < 2° for a good capture
    theta_electrical = theta_resolver + offset

For the EM57: expect −90° (or +270°, same thing).
See hardware docs for derivation and cross-checks.

## Writes analysis.png with:

- Raw and demodulated resolver channels
- Reconstructed phase voltages  
- Clarke alpha/beta
- Back-EMF angle vs resolver angle over the capture
