# Photographed large-track model — R45, 20 October 2026

This is an approximate reconstruction, **not physical test feedback**. Firmware
remains R45. No speed, controller gains, stop conditions or frozen ZIPs changed.

## Reconstruction and method

- `tests/photo_track.json` traces IMG_5189.jpeg in its displayed orientation.
  A planar homography uses the four estimated mat corners; the annotated
  1.27 m top and 2.48 m right segments set the two scales. The paper is wrinkled,
  the photo is oblique and the measurement endpoints are approximate. The
  inferred mat dimensions are not surveyed dimensions.
- The derived route is **14.067 m**, sampled at 2 mm, with alternating bends,
  both interior lobes and exactly **two geometric crossings**. The start is
  halfway along the long straight. Both travel directions are exercised.
- Actual frozen R44 and workspace R45 code run against the same native fake
  hardware. Motor commands drive a first-order differential-drive model; motion
  generates integer encoders, quantized gyro readings and five optical readings.
- Optical readings come from distances to the **union of every black stroke**.
  There is no oracle choosing the intended branch. The weighted position uses
  the pinned Pololu 1.1.3 library's 200/50 thresholds and lost-line behavior.
  The nominal sensor row is assumed 30 mm forward, 10 mm pitch, with a 4 mm
  optical edge transition and a 20 mm line. These dimensions/responses need
  measurement; pitch/width variants are included.
- Separate route-order scoring detects shortcuts and premature estimated
  finishes. `complete` requires three circuits at the stop command, within
  6 mm discretization tolerance. It does not simulate rolling after the stop.
  `route_fraction` also exposes late finishes. It is never fed into steering.
- The model has **no tire slip, grip saturation, pitch/bounce, mat wrinkles,
  lighting changes, battery sag or measured hardware latency**. Kinematic
  tracking success cannot establish that the robot can hold a physical corner.

## Results

All **72 diagnostic runs** are retained in [report.json](report.json): 36 cases
per revision. They cover both directions; Safe/Mid/Beast; motor gain/response,
sensor noise and 3% motor imbalance, line width/pitch, 5/10 ms replay, ±10%
course scale, 2 ms optical delay, replay encoder scale changes of ±0.5/1%,
and gyro scale factors 1.001/1.005/0.995.

| Nominal Beast result | R44 | R45 |
| --- | --- | --- |
| Forward replay laps, model seconds | 15.975 / 15.805 | 15.620 / 15.510 |
| Reverse replay laps, model seconds | 16.055 / 15.930 | 15.690 / 15.570 |
| Maximum crossing wheel-average drop per sample | 332 PWM | 2 PWM |
| Acceleration during a detected crossing | 0 | 0 |
| Full route covered on three estimated laps | Yes, both directions | Yes, both directions |

The approximately 2% nominal model time reduction supports R45's existing
crossing correction; it is **not a prediction of physical lap time**. Normal
corner steering allocation can still reduce wheel average more than the routine
throttle ramp; the 2 PWM figure applies only to these detected crossing samples.

Both revisions complete all **30 cases without injected gyro scale error**.
R45 meets the full-route criterion in **32/36**, versus R44 **31/36**. This
parameter matrix is not a probability distribution or a physical completion rate.
The remaining cases must not be hidden behind the nominal passes:

- With **+0.1% gyro scale**, both revisions declare three laps about 30–32 mm
  before three independently scored circuits. These fail the strict route check.
- With **+0.5% gyro scale**, R45's first estimated closure occurs at 0.9956 of
  the route. Resetting the lap frame repeats that early anchor. Both directions
  eventually hit the unchanged **Map mismatch** stop on replay lap 3, while
  modeled axle offset is still under 7 mm. R44 stops in one direction and falsely
  declares completion early in the other. This is a plausible failure mechanism,
  **not a measurement or diagnosis of the user's gyro**.
- With **−0.5% gyro scale**, R45 completes, but the final estimated finish is
  about 0.19–0.20 m late. R44 stops with mismatch in one direction. Completing
  three circuits does not imply precise finish localization.
- The nominal map uses **390/512 bins**; the largest geometry variant uses 429.
  No modeled case reaches Map full. This does not establish the cause of the
  user's physical failure or measure runtime stack margin.

The modeled route works in both directions. No global reversal is required to
explain the modeled failure. Confirm the actual robot's loaded revision/mode
and lap boundaries before changing map order or introducing re-localization.

## Reproduce and validation

No extra Python dependencies or robot connection are required; use native g++.

```text
python tests/photo_track.py --quick --check --output <temporary-output-folder>
python tests/photo_track.py --check --traces --output <temporary-output-folder>
python tests/run_tests.py --focused
```

The quick CI regression checks true route completion in both directions,
exercises crossings, rejects crossing acceleration and bounds the nominal
crossing drop. The full run records failures rather than asserting that all
uncertainty cases pass. SHA256 hashes identify the exact two firmware sources.
CSV traces are optional; [track.svg](track.svg) and [crossing.svg](crossing.svg)
are regenerated without plotting dependencies.

Local validation: the 72 runs completed, nominal checks passed, and all four
focused native executables passed. The pinned Leonardo build remains
**28662/28672 flash, 1508/2560 static RAM**. Only ten flash bytes are spare;
firmware expansion needs a separately verified space-saving change. R45 and
all previous release ZIPs remain unchanged.

## Next physical test

Use frozen R45, recalibrate on the mat and start midway along the long straight.
Complete Safe first. Film one whole learning lap and the beginning of replay:
check whether each estimated lap changes at the same physical start point and
whether that point walks along the straight. Then record exact stop text,
revision, mode and which crossing/bend failed. That distinguishes map phase
drift from insufficient physical corner grip without weakening the mismatch stop.
