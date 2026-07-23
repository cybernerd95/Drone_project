# Drone Micro-Doppler Radar Simulation — Project Summary

## Goal
Simulate a quadcopter drone's radar signature (range, speed, and rotor
micro-Doppler) in MATLAB using Phased Array System Toolbox, in a way that
produces research-quality, visually readable outputs.

## Files produced so far

1. **`droneMotion.m`** — computes the 9 scatterers of the drone each pulse:
   scatterer #1 = drone body, scatterers #2–9 = the two blade tips of each
   of the 4 rotors. Returns position, velocity, and radar look angle
   (`rangeangle`) for every scatterer.
2. **`drone_microdoppler_main.m`** — single straight-line-flight version.
   Full radar chain: LFM chirp waveform → transmitter → URA array
   (Radiator) → FreeSpace propagation → RadarTarget (RCS) → URA array
   (Collector) → receiver → matched filter → Range-Doppler map →
   micro-Doppler spectrogram.
3. **`drone_microdoppler_waypoints_main.m`** — extended version: drone
   flies A → B → C (each leg ≥100 m, a few seconds long), by updating
   `phased.Platform.Velocity` (a tunable property) at each waypoint
   crossing. Adds a range track plot, radial-speed track plot, a
   range-compensated (range-tracked) spectrogram over the whole flight,
   and two Range-Doppler snapshots (mid-leg-1, mid-leg-2).

## Chronological changes / fixes made

- Fixed `droneMotion.m` originally advancing `phased.Platform` with a
  **hardcoded** `1/20000` instead of the actual `1/prf` — replaced with an
  explicit `dt` argument.
- Replaced `cart2sph`-based look-angle computation (only correct because
  radar sat at the origin) with `rangeangle(scatterPos, radarpos)` — the
  general, toolbox-native approach, matching MATLAB's own helicopter
  micro-Doppler example.
- Fixed the Range-Doppler map's `ylim` clipping the target out of the
  displayed window (target was at ~700 m range but the plot was
  hard-limited to ±200 m).
- Replaced the unmodulated rectangular pulse (`PulseWidth = 2 µs`, which
  gives ~300 m range resolution — far too coarse) with a **linear FM
  (LFM) chirp** (`SweepBandwidth = 5 MHz`, `PulseWidth = 10 µs`), giving
  ~30 m range resolution and a proper pulse-compression gain.
- Added a waypoint-following version (A→B→C) by updating the platform's
  `Velocity` property mid-simulation, plus range-tracking logic (finding
  the peak-energy range bin every pulse) since a moving target can't use
  one fixed range bin.

## ⚠️ Known unresolved issue

In the waypoint version, the **standalone `phased.MatchedFilter` object**
used to build `mfSignal` (for the range-track plot and the
range-compensated spectrogram) does **not** cancel its own internal group
delay. This shows up as the detected range being offset ~1,500 m too high
(exactly `chirp_length_in_samples × c/(2·fs)` = 100 samples × 15 m/sample).
The **Range-Doppler snapshot plots are correct** because those use
`phased.RangeDopplerResponse`, which handles this delay internally.
**Fix (not yet applied):** construct the matched filter with
`'GroupDelaySource','Auto'`, or manually shift `mfSignal` up by
`length(mfcoeff)-1` samples before indexing into range.

## Current modeling assumptions / what is NOT included

- **Radar is stationary and non-scanning.** `radarpos = [0;0;0]`,
  `radarvel = [0;0;0]`, and the array (`phased.URA`) has no boresight
  steering, rotation, or mechanical/electronic scan pattern applied — it's
  a fixed staring array, not a rotating (PPI-style) radar.
- **No atmosphere/weather modeling.** `phased.FreeSpace` only applies
  basic free-space path loss and propagation delay — no atmospheric
  attenuation, no turbulence, no rain/humidity loss, and no wind. The
  drone's flight path is pure kinematics (piecewise-constant velocity
  between waypoints); there's no gust/turbulence perturbation on its
  motion.
- **Array combining is simple summation**, not real digital beamforming.
  `phased.Radiator`/`phased.Collector` apply the array's manifold
  (element phase/gain vs. angle) per element, and the receive branch does
  `sum(echo,2)` — a fixed, unweighted (conventional/delay-and-sum-style)
  combination. There's no per-element digital sampling retained, no
  adaptive weighting, and no angle-of-arrival estimation.

## Algorithms currently used (for citation)

| Stage | Algorithm |
|---|---|
| Waveform | Linear Frequency Modulation (LFM) chirp |
| Pulse compression | Matched filtering (correlation with time-reversed conjugate of the transmit chirp) |
| Range processing | FFT-based range compression (via `phased.RangeDopplerResponse`) |
| Doppler processing | FFT across slow-time (pulse-to-pulse) samples, 128-point Doppler FFT |
| Micro-Doppler analysis | Short-Time Fourier Transform (STFT), via `pspectrum(...,'spectrogram')` |
| Target range detection | Simple peak/argmax energy detection per pulse (**not** CFAR — no adaptive noise-threshold detector is implemented) |
| Kinematics | Piecewise-constant-velocity point-mass model (`phased.Platform`, `'MotionModel','Velocity'`), velocity updated at waypoints |
| Array/antenna model | Uniform Rectangular Array (URA) manifold via `phased.Radiator`/`phased.Collector`, unweighted element summation on receive |

## Next step requested
Extend this to a **digital array radar** version: retain individual
element channels (instead of summing them on receive) and apply an
explicit digital beamforming algorithm, plus revisit whether the radar
should be modeled as rotating/scanning and whether atmospheric/wind
effects should be added.
