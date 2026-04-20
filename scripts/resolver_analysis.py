#!/usr/bin/env python3
"""
EM57 resolver offset analysis - robust demod version.
"""
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

src = sys.argv[1] if len(sys.argv) > 1 else 'scope.csv'

t, s, c, vab, vcb = [], [], [], [], []
with open(src) as f:
    r = csv.reader(f); next(r)
    for row in r:
        if len(row) < 5: continue
        t.append(float(row[0])); s.append(float(row[1])); c.append(float(row[2]))
        vab.append(float(row[3])); vcb.append(float(row[4]))

t = np.array(t); s = np.array(s); c = np.array(c)
vab = np.array(vab); vcb = np.array(vcb)

dt = np.median(np.diff(t)); fs = 1.0/dt
print(f"fs={fs:.0f} Hz, N={len(t)}, duration={t[-1]-t[0]:.3f}s")

# DC null
for name, arr in [('CH1 sin', s), ('CH2 cos', c), ('CH3 VAB', vab), ('CH4 VCB', vcb)]:
    print(f"  {name}: {arr.mean():+.4f} V -> nulled")
s -= s.mean(); c -= c.mean(); vab -= vab.mean(); vcb -= vcb.mean()

# Phase reconstruction
v_a = (2*vab - vcb)/3
v_b = -(vab + vcb)/3
v_c = (2*vcb - vab)/3

alpha = v_a
beta = (v_b - v_c)/np.sqrt(3)

omega_emf = np.polyfit(t, np.unwrap(np.arctan2(beta, alpha)), 1)[0]
print(f"\nBack-EMF f_elec = {omega_emf/(2*np.pi):+.3f} Hz")
print(f"Direction: {'forward ABC' if omega_emf > 0 else 'reverse ACB'}")

theta_emf = np.arctan2(beta, alpha) + np.pi/2
theta_emf = (theta_emf + np.pi) % (2*np.pi) - np.pi

# Carrier frequency from FFT
freqs = np.fft.rfftfreq(len(s), dt)
spec = np.abs(np.fft.rfft(s))
mask = (freqs > 5000) & (freqs < 20000)
f_carrier = freqs[mask][np.argmax(spec[mask])]
print(f"Resolver carrier: {f_carrier:.1f} Hz")
wc = 2*np.pi*f_carrier

# --- Robust envelope recovery ---
# For s(t) = m(t)*sin(wc*t + phi):
#   s^2 + (ds/dt / wc)^2 = m(t)^2  (Pythagorean identity)
# Gives magnitude; sign recovered from zero-crossing tracking.
ds = np.gradient(s, dt)
dc = np.gradient(c, dt)

s_mag = np.sqrt(s**2 + (ds/wc)**2)
c_mag = np.sqrt(c**2 + (dc/wc)**2)

# Low-pass the magnitudes to smooth out numerical noise (cutoff well below carrier)
b_lpf, a_lpf = butter(4, f_carrier/5, btype='low', fs=fs)
s_mag = filtfilt(b_lpf, a_lpf, s_mag)
c_mag = filtfilt(b_lpf, a_lpf, c_mag)

# Sign recovery by alternating-sign across envelope zero-crossings (minima of |env|)
def alternating_sign(mag, min_prominence=None):
    if min_prominence is None:
        min_prominence = 0.2 * (mag.max() - mag.min())
    mins, _ = find_peaks(-mag, prominence=min_prominence)
    sgn = np.ones_like(mag); cur = 1.0; last = 0
    for m in mins:
        sgn[last:m] = cur; cur = -cur; last = m
    sgn[last:] = cur
    return sgn, mins

s_sign, s_mins = alternating_sign(s_mag)
c_sign, c_mins = alternating_sign(c_mag)
print(f"Sin envelope zero-crossings: {len(s_mins)}")
print(f"Cos envelope zero-crossings: {len(c_mins)}")

sin_env = s_sign * s_mag
cos_env = c_sign * c_mag

# Try all 4 polarity combos (initial sign of s_sign and c_sign is arbitrary)
best = None
print("\nPolarity tests:")
for ss in (1, -1):
    for cs in (1, -1):
        theta_res = np.arctan2(ss*sin_env, cs*cos_env)
        omega_res = np.polyfit(t, np.unwrap(theta_res), 1)[0]
        if np.sign(omega_res) != np.sign(omega_emf):
            print(f"  sin*{ss:+d}, cos*{cs:+d}: direction mismatch (skip)")
            continue
        diff = (theta_emf - theta_res + np.pi) % (2*np.pi) - np.pi
        off = np.arctan2(np.mean(np.sin(diff)), np.mean(np.cos(diff)))
        dcen = (diff - off + np.pi) % (2*np.pi) - np.pi
        std = np.sqrt(np.mean(dcen**2))
        print(f"  sin*{ss:+d}, cos*{cs:+d}: offset={np.degrees(off):+7.2f}° std={np.degrees(std):5.2f}°")
        if best is None or std < best[3]:
            best = (ss, cs, off, std, theta_res)

ss, cs, off, std, theta_res = best
print(f"\n{'='*60}")
print(f"BEST: sin*{ss:+d}, cos*{cs:+d}")
print(f"Offset = {np.degrees(off):+.2f}°  std = {np.degrees(std):.2f}°")
print(f"Firmware: theta_electrical = theta_resolver + ({np.degrees(off):+.1f}°)")
print(f"{'='*60}")

# Plots
fig, ax = plt.subplots(5, 1, figsize=(15, 14), sharex=True)
ax[0].plot(t, s, 'y', lw=0.3, alpha=0.3)
ax[0].plot(t, ss*sin_env, 'r', lw=1, label=f'sin demod (*{ss:+d})')
ax[0].set_ylabel('Sin'); ax[0].grid(True); ax[0].legend(loc='upper right')

ax[1].plot(t, c, 'c', lw=0.3, alpha=0.3)
ax[1].plot(t, cs*cos_env, 'b', lw=1, label=f'cos demod (*{cs:+d})')
ax[1].set_ylabel('Cos'); ax[1].grid(True); ax[1].legend(loc='upper right')

ax[2].plot(t, v_a, 'r', lw=0.5, label='V_A')
ax[2].plot(t, v_b, 'g', lw=0.5, label='V_B')
ax[2].plot(t, v_c, 'b', lw=0.5, label='V_C')
ax[2].set_ylabel('EMF'); ax[2].grid(True); ax[2].legend(loc='upper right')

ax[3].plot(t, alpha, 'r', lw=0.5, label='alpha')
ax[3].plot(t, beta, 'b', lw=0.5, label='beta')
ax[3].set_ylabel('Clarke'); ax[3].grid(True); ax[3].legend(loc='upper right')

ax[4].plot(t, np.degrees(theta_emf), 'k', lw=0.5, label='θ_e (EMF)')
ax[4].plot(t, np.degrees(theta_res), 'm', lw=0.5, alpha=0.6, label='θ_e (resolver)')
ax[4].set_ylabel('deg'); ax[4].set_xlabel('t (s)')
ax[4].grid(True); ax[4].legend(loc='upper right')

plt.tight_layout()
plt.savefig('analysis.png', dpi=100)
print("Saved analysis.png")
