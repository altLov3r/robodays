# Status: 19 October 2026

- Target: stock Pololu 3pi+ 32U4 OLED Standard, four matching AAA NiMH cells.
- Latest candidate: R43. No physical R43 testing has happened.
- Confirmed large-mat failure: **Map mismatch**, plus one incorrect corner.
  RAM exhaustion and Map full were not confirmed. The mat has S-bends/crossings.
- R43 adds a 30 ms mismatch-caution request cap, boost inhibition and live-yaw
  assistance, plus signed fine-map diagnostics. The 200 ms mismatch stop remains.
  Pooled maps retain magnitude-only acceptance. R42's capacity changes remain.
- Local build: 28664/28672 flash, 1508/2560 static RAM, no linked heap allocators.
  Only eight flash bytes remain with the pinned toolchain; do not drop safeguards
  to make an oversized build fit. Peak stack/physical timing are unmeasured.
- Four focused native executables pass. Both paired reports have 33 passing
  current cases. Most course results match R42; delayed/noisy jitter case 3 is
  slower and has larger sensor SD. It needs review, not a blanket speed claim.
- The old testStraightStability noise40/output-drop assertion fails identically
  in frozen R42 and R43. That historical check is not included among the passes.
- R42 and older release ZIPs are preserved. R43 should remain a test candidate
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
