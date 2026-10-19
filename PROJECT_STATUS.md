# Status: 19 October 2026

- Target: stock Pololu 3pi+ 32U4 OLED Standard, four matching AAA NiMH cells.
- Latest candidate: R44. No physical R43/R44 testing has happened.
- Confirmed large-mat failure: **Map mismatch**, plus one incorrect corner.
  RAM exhaustion and Map full were not confirmed. The mat has S-bends/crossings.
- R44 retains the last valid optical error during a crossing, preventing its
  ambiguous position from creating a derivative kick on exit. No tuning changes.
- R43 adds a 30 ms mismatch-caution request cap, boost inhibition and live-yaw
  assistance, plus signed fine-map diagnostics. The 200 ms mismatch stop remains.
  Pooled maps retain magnitude-only acceptance. R42's capacity changes remain.
- Local build: 28670/28672 flash, 1508/2560 static RAM, no linked heap allocators.
  Only two flash bytes remain with the pinned toolchain; do not drop safeguards
  to make an oversized build fit. Peak stack/physical timing are unmeasured.
- Four focused native executables pass, including 108 crossing trace pairs.
  Historical R42/R43 paired reports have 33 passing R43 cases. Most course
  results match R42; delayed/noisy jitter case 3 is
  slower and has larger sensor SD. It needs review, not a blanket speed claim.
- The old testStraightStability noise40/output-drop assertion fails identically
  in frozen R42 and R43. That historical check is not included among the passes.
- R42/R43 and older release ZIPs are preserved. R44 should remain a test candidate
  until LAB_TEST.md contains physical results.
- Private source: https://github.com/altLov3r/robodays. Cloud CI and access
  instructions are in CLOUD.md. This desktop chat is still a local executor.
- Overnight heartbeat: robodays-overnight-review, every two hours until
  09:00 Europe/Sofia on 20 October 2026. Local PC/app must remain available.

## Useful remaining work

Investigate short disagreement under sensor delay/jitter without weakening the
persistent wrong-turn stop or tuning only to one model. Audit any flash-saving
refactor against the same behavior and bounds. Do not widen finish capture or
invent map re-localization without evidence: the first lab check must verify
that learning covers the entire physical course before replay starts.

Prefer one justified fix with a regression over new release churn. Record
software limitations and keep the prior candidate available for comparison.

## Evening review: S-bend and lap transitions

- Added 120 moving, prescribed S-bend agreement cases in
  `tests/test_race_session.cpp`: both directions, all three map resolutions,
  -2..+2 bin offsets, 600/1400 mm/s inputs, 5 ms or 5/13/7/10 ms ticks,
  and partial-bin wrap. Maximum pending disagreement: 30 ms; no mismatch stop.
  The expanded cases include positions beyond 65535 counts. They isolate map
  agreement; pose closure is suppressed and physical learning is not tested.
- Added actual-controller lap-boundary cases in `tests/test_corner_exit.cpp`.
  A clear straight keeps qualified launch; a corner at bin zero or crossing
  revokes it. No firmware defect was reproduced by these checks.
- Firmware remains R43, with unchanged tuning and safeguards. No new ZIP or
  release was justified. R42/R43 frozen ZIP hashes remain unchanged.
- All four focused native executables pass. The pinned Leonardo build remains
  28664/28672 flash and 1508/2560 static RAM. No paired model rerun was needed
  for this test-only change. The delayed/noisy case 3 limitation remains open.
- Next physical test: recalibrate on the large mat, start midway along the long
  straight, verify one entire learning lap, then run Safe. Record exact stop,
  error/derivative, voltage and which part of the corner/crossing failed.

## Later review: R44 crossing-history correction

- Reproduced a distinct controller bug: four black sensors can report a biased
  position even while crossing straight. That value entered derivative history
  despite steering being disabled during the crossing. With the real exit line
  centered, one 20 ms fixture commanded 136/0 or 0/136 instead of 68/68 PWM.
- R44 retains the last valid error during a crossing; the derivative filter
  relaxes normally. On exit, current valid optical readings resume. Raw gyro,
  encoder mapping, crossing speed, braking/launch settings and all stops remain.
- Added 54 centered/asymmetric trace pairs per control period, covering 5/10 ms,
  1/4/12 crossing samples, three entry and exit errors, and both bias directions.
  Frozen R43 fails all 108 pairs; R44 passes all. Maximum extra exit differential
  falls from 190/168 PWM to zero. Reports are in `releases/*R44-crossings*.json`.
- Focused native checks and pinned Leonardo build pass; flash 28670/28672,
  static RAM 1508/2560. Native regression checks confirm No line immediately
  after a crossing and Map mismatch at 200 ms during one. No new dependencies.
- Cloud checks for the R44 firmware commit also pass:
  https://github.com/altLov3r/robodays/actions/runs/39306146651.
- Paired checks are limited to this crossing issue; full trajectories were not
  rerun. No physical speed, grip, map agreement or completion claim follows.
  The reported large-mat mismatch is still not conclusively diagnosed.
- Next lab test remains a complete learning lap and Safe session on the actual
  mat, watching crossing exits and recording exact stop/E/D/location/voltage.

## Late review: learning at crossings

- Added 18 learning-stage centered/asymmetric crossing pairs in
  `tests/test_corner_exit.cpp`: both gyro polarities, 10/80/180 ms crossings,
  and starting positions 0/65530/131066 counts. The latter positions trigger
  the first and second map compactions during the crossing.
- R44 continues accumulating nonzero gyro-derived bends during ambiguous optical
  readings. Maps, curvature accumulators, control traces, width/polarity and
  speed-scale estimates match the centered reference. No false learning-noise
  fallback occurs; existing genuine-noise fallback tests remain passing.
- All four focused native executables pass. Firmware and R44 tuning remain
  unchanged; no new release or paired trajectory rerun is justified. Source
  bytes match the frozen R44 ZIP, and R42/R43/R44 ZIP hashes are preserved.
  The pinned R44 build remains 28670/28672 flash and 1508/2560 static RAM.
- These are scripted controller/storage inputs with pose closure suppressed.
  The next physical test remains the entire learning lap followed by Safe,
  watching crossing exits and recording exact stop, E/D, location and voltage.
