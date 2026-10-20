# Status: 20 October 2026

- Target: stock Pololu 3pi+ 32U4 OLED Standard, four matching AAA NiMH cells.
- Latest candidate: R45; physical R45 testing is pending.
- Photo-course review: [72 paired model runs](reports/photo-track-r45/README.md)
  cover an approximate 14.067 m trace with two crossings. R45's nominal crossing
  drop is at most 2 PWM versus R44's 332; both nominal directions cover three
  true circuits. Gyro-scale sensitivity exposes unresolved lap-anchor drift and
  Map mismatch. Full-route criteria pass 32/36 R45 cases and 31/36 R44 cases;
  remaining failures are retained. Firmware/tuning and release ZIPs are unchanged.
- New report: learning follows the spirals; replay laps 2/3 fail, with hard
  crossing braking and brief acceleration. Tested revision/mode are unconfirmed.
  A global map reversal was suspected by the user, not established.
- R45 removes the forced crossing snap to 65 PWM, retains map/gyro steering
  through overlaps and follows live turn direction when a mapped bend opposes it.
- Confirmed large-mat failure: **Map mismatch**, plus one incorrect corner.
  RAM exhaustion and Map full were not confirmed. The mat has S-bends/crossings.
- R44 retains the last valid optical error during a crossing, preventing its
  ambiguous position from creating a derivative kick on exit. No tuning changes.
- R43 adds a 30 ms mismatch-caution request cap, boost inhibition and live-yaw
  assistance, plus signed fine-map diagnostics. The 200 ms mismatch stop remains.
  Pooled maps retain magnitude-only acceptance. R42's capacity changes remain.
- Local build: 28662/28672 flash, 1508/2560 static RAM, no linked heap allocators.
  Only ten flash bytes remain with the pinned toolchain; do not drop safeguards
  to make an oversized build fit. Peak stack/physical timing are unmeasured.
- Four focused native executables pass, including 108 crossing trace pairs.
  Historical R42/R43 paired reports have 33 passing R43 cases. Most course
  results match R42; delayed/noisy jitter case 3 is
  slower and has larger sensor SD. It needs review, not a blanket speed claim.
- The old testStraightStability noise40/output-drop assertion fails identically
  in frozen R42 and R43. That historical check is not included among the passes.
- R42/R43/R44 and older release ZIPs are preserved. R45 should remain a test candidate
  until LAB_TEST.md contains physical results.
- Private source: https://github.com/altLov3r/robodays. Cloud CI and access
  instructions are in CLOUD.md. This desktop chat is still a local executor.
- Overnight heartbeat robodays-overnight-review was paused at the user's
  request. The current R45 work is a new manual request; do not restart it.

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

## 20 October review: unequal-wheel map boundaries

- Added 18 independent half-count storage-oracle cases in
  `tests/test_large_map.cpp`. Unequal wheel deltas exercise odd sums of 7/255/999
  counts around both compactions, the last partial bins and maximum capacity.
  Final learning closure is prescribed with one wheel moving one count.
- Every used map bin is known and matches the independent signed fine-bin or
  pooled peak-magnitude reference within one curvature unit for float/Q3 rounding. Learned
  length and bin count remain bounded. Planner wrap before/at/after the learned
  length matches the dense distance oracle, including odd half-count positions.
- All four focused native executables pass. No firmware defect was reproduced;
  R44, its tuning and all frozen releases remain unchanged. The prescribed
  pose/clock isolate storage arithmetic; these cases do not establish closure,
  reachable physical track length, grip or lap performance.
- Firmware remains byte-identical to frozen R44: the pinned build is still
  28670/28672 flash and 1508/2560 static RAM. No paired trajectory rerun needed.
- Next physical test remains one entire learning lap, then Safe on the large
  mat; capture exact stop, E/D, voltage and corner/crossing location.

## R45: physical replay report and targeted correction

- Reproduced a crossing discontinuity: 280 PWM became 65 in one control tick.
  R45 holds at most the delivered entry average, with normal mapped braking.
  It excludes optical PD at an ambiguous crossing while retaining gyro/map
  steering; strong launch still waits for valid optical/yaw requalification.
- Reproduced opposite replay steering before mismatch caution: valid line error
  and gyro requested a positive turn, but R44 commanded -16 PWM differential;
  the mirrored case also failed. An opposing stored bend suppressed assistance
  and left yaw damping fighting the requested turn. Live-direction fallback
  corrects this without changing normal matching-map prediction or map order.
- Braking now anchors hidden internal throttle to delivered power when the
  independent speed request requires braking, and throughout crossings. R44's
  400 internal / 280 delivered fixture briefly accelerates under a brake request;
  R45 immediately decelerates at the normal rate. Straight jitter must not ratchet
  throttle down; its independent request retains the existing policy.
- Both gyro polarities and turn directions pass. All 30 targeted command
  fixtures now pass; frozen R44 fails all 30. Routine crossing drop changes
  295 -> 2/3 PWM at 5/10 ms; wrong-way and canceled-crossing-turn cases become zero.
  Eight changing-radius course runs complete with effectively unchanged timing
  and offsets. Reports: `releases/*R45-replay*.json`; these are not track predictions.
- All four focused native executables and the pinned Leonardo build pass:
  28662/28672 flash, 1508/2560 static RAM. Native tests retain line-loss,
  button, battery, 200 ms map-mismatch and dark/air stops. No speed/gain increase.
- Cloud source checks also pass with identical resource totals:
  https://github.com/altLov3r/robodays/actions/runs/39429678913.
- Next physical test: R45 startup, complete learning lap, then Safe replay.
  Watch spiral entry and overlap exit; capture mode, voltage, lap, exact stop
  and E/D. Persistent spatial phase drift still needs real track evidence.
