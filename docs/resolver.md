## Resolver specifications (as measured/used on this unit)

### Excitation
- Frequency: 10 kHz sinusoidal
- Amplitude: 10 V (unclear whether peak-to-peak or ±10 V; confirm against RDC spec before driving)

### Output format
- Two channels: Sin and Cos
- Amplitude-modulated carriers at excitation frequency
- Envelope peak ~2 V (measured from this unit; will scale with excitation amplitude)
- Sin channel wired to scope CH3
- Cos channel wired to scope CH4

### Electrical characteristics
- 4 pole pairs (resolver electrical cycles = motor electrical cycles = 4× mechanical revolutions)
- 1:1 tracking with motor electrical angle
- Resolver carrier frequency as measured: 9993 Hz (matches 10 kHz spec)

## Resolver-to-Electrical Angle Offset

**θ_electrical = θ_resolver − 90°**

Equivalently: when resolver reads 0°, the rotor d-axis is at electrical −90° from phase A.
Equivalently: when resolver reads 90°, rotor d-axis aligns with phase A (positive d-axis).

This reflects Nissan's deliberate factory mounting where resolver zero corresponds
to the rotor q-axis aligned with phase A.

### Verification methods (all agree)
| Method | Result | Precision |
|--------|--------|-----------|
| Back-EMF Clarke analysis (2 s @ 500 kSa/s) | −89.82° | ±1.57° |
| 20 A two-phase DC pulse alignment | −87° average | ±3° |
| High-current DC pulse (+A,−B,−C) | −78° | ±30° (cog-limited) |
| Physical design expectation | −90° exact | (design value) |

Commit **−90°** to firmware (cardinal design value; measurements land within noise).

## Phase labeling (scope wiring convention)

Phases labeled A, B, C match the physical motor bus bar terminals.
- Back-EMF measurement showed electrical direction when shaft rotated
  clockwise (facing motor shaft) gave negative omega in Clarke (alpha=V_A, beta=(V_B−V_C)/√3)
- This means rotating CW-facing-shaft corresponds to DECREASING electrical angle
  in standard ABC convention
- For FOC: positive Iq with (θ_electrical = θ_resolver − 90°) will produce
  torque in the CW-facing-shaft direction
- If opposite direction is desired (for vehicle "forward"): either swap two phase
  wires OR negate angle in firmware

## Recorded on Rigol with 
- CH1 = Resolver Sin
- CH2 = Resolver Cos
- CH3 = V_AB (phase A minus phase B)
- CH4 = V_CB (phase C minus phase B)

## Mechanical conventions used

- "Clockwise" in all experiments means: viewing the motor shaft end directly, rotating CW
- Shaft end convention: the output shaft end (not the back/resolver end)
- 12 cogging "clicks" felt per electrical cycle = 48 cogs per mechanical revolution
- One cog click ≈ 7.5° mechanical = 30° electrical

## DC pulse test behavior (IPM saliency notes)

At low-to-moderate currents (~10-20 A), the motor exhibits IPM saliency effects:

- Rotor may lock 180° from MMF direction instead of with it (reluctance torque dominance)
- Polarity flipping sometimes produces no rotation (reluctance minimum is 180°-symmetric)
- Two-phase tests have ±3° final alignment accuracy once cogging is overcome
- ~20 A provides 4:1 torque margin over cogging (12.5 Nm vs ~3 Nm cogging)
