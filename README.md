# Pololu 3pi+ Standard line follower

**R45 test candidate - continuous crossings and correct replay turn direction.**
For the stock **Pololu 3pi+ 32U4 OLED Standard, 30:1 MP motors**, black line on
white floor. Replace both sketch files together. R42 through R44 remain available.

The private repository is [altLov3r/robodays](https://github.com/altLov3r/robodays).
See [CLOUD.md](CLOUD.md) for access from another PC and the separate Codex Cloud
setup. Record physical tests in [LAB_TEST.md](LAB_TEST.md).

## What changed

- R45 fixes the reported crossing brake/surge: crossing requests cannot exceed
  the previously delivered wheel average, and mapped corners still request
  braking through the normal ramp. The unconditional snap to 65 PWM is removed.
  Crossing exits still require settled optical/yaw samples before strong boost.
- Map/gyro steering continues across crossings while ambiguous optical steering
  is excluded. An overlap no longer unconditionally cancels an ongoing turn.
- A stale opposite-signed mapped bend previously removed turn assistance while
  yaw damping could command a turn against valid line feedback. R45 uses live
  direction in that conflict. Normal agreeing-map prediction and optical vetoes
  remain. Map order, speed/gain settings and the 200 ms mismatch stop are unchanged.
- Braking now starts from the delivered wheel average when internal throttle
  has been hidden by steering trim. This prevents a brake request from briefly
  accelerating the wheels while the internal value winds down. Ordinary
  straight acceleration retains its existing behavior.
- The new physical report is spiral failures on replay laps 2/3, with crossing
  braking/brief acceleration. Tested revision/mode and a global map reversal
  remain unconfirmed. The slow learning lap reportedly follows the spirals.
- R44 keeps the last valid optical error while four or five sensors see black.
  An asymmetric crossing has no unique line center; feeding it into the
  derivative previously caused a turn after the real line was centered again.
  The existing derivative filter relaxes normally during the crossing. Valid
  line readings resume on exit. R45 adds the continuous crossing behavior above.
- The following map changes were introduced in R42/R43 and remain in R44.
- The owner clarified the big-track stop was **Map mismatch**, with one
  incorrect corner. A Map full or RAM failure was not confirmed.
- After **30 ms of continuous disagreement**, requests are capped at **180 PWM**,
  straight boost is inhibited and assistance uses live yaw. Shorter spikes
  retain the normal path. The normal 300 PWM/s throttle braking ramp remains;
  actual output can stay above the request while it ramps down.
- Fine maps now check turn direction. Pooled maps retain magnitude-based
  acceptance because a merged bin can contain both S-bend directions. The local
  two-bin window, +16 tolerance and **200 ms stop** remain. An agreeing sample
  clears caution; straight boost must pass the usual alignment qualification.
- When full during learning, adjacent bins merge **in place**, retaining the
  strongest stored bend. Capacity doubles twice, reaching about **73.6 m**.
  Opposite S-bends cannot cancel each other during this merge. Small tracks
  keep the original map resolution and normal speed budgets.
- Map position and lookahead now use **32-bit distance**, supporting positions
  beyond 65,535 counts and correctly wrapping partial final bins.
- Coarse bins can contain both turn directions. After compression, predictive
  speed limits use the pooled map, while turn assistance uses live yaw.
  The early trailing-bin release is disabled for pooled maps.
- Each lap's timeout is **180 s**; the existing **240 s whole-session cap**
  remains. These time limits can stop a run before the storage limit is reached.
- Shared agreement code, a bounded byte timer and narrower base-bin indexing
  reduce overhead. No new dependencies, heap allocation or display writes
  while racing. R41's corner budget, entry reserve and line-loss stop remain.

## Validation and limits

- A new [photo-traced course model](reports/photo-track-r45/README.md) includes
  both crossings and independently checks physical route coverage. Across 72
  paired diagnostic runs, R45 completes the nominal course in both directions
  and reduces its largest nominal crossing drop from 332 to 2 PWM. Injected
  gyro scale error still produces lap-anchor drift, premature finishes or Map
  mismatch. All failures are retained; these are not physical speed/grip results.
- R44/R45 paired checks cover **30 replay control fixtures** at 5/10 ms.
  R44 fails all 30; R45 passes all. Wrong-way turn cases fall from eight to zero,
  canceled crossing turns from eight to zero, and maximum routine crossing drop
  from 295 PWM to 2/3 PWM. This bound excludes stops and urgent steering allocation.
  The hidden-throttle braking fixture changes from an output increase to normal
  deceleration at both control periods.
- Eight changing-radius course-model runs complete, with essentially unchanged
  lap times and offsets. These models do not reproduce the photographed track
  or establish physical tire grip. No speed increase is claimed.
- All **108 crossing trace pairs** pass at 5/10 ms. Under identical valid entry
  and exit readings, centered and asymmetric crossing inputs now produce the
  same control history. Frozen R43 differs in all 108 pairs, with up to 190/168
  PWM extra exit differential. These are command fixtures, not physical tests.
- Immediate No line and the persistent 200 ms Map mismatch stop remain active
  after/during a crossing. R45 awaits physical testing on the larger mat.
- Four native test executables pass. New checks cover both compactions, exact
  capacity, sample chunks from 1 to 500 counts, partial-bin flush/wrap, opposite
  bend peaks, live-yaw steering and the final bounded Map full stop.
- New checks cover direction, pooled S-bends, caution thresholds, boost,
  ordinary braking, live assistance, recovery and timer increments to 246 ms.
- Historical paired R42/R43 reports contain **23 motion cases + 10 fixtures**
  at 5/10 ms. All R43 cases complete; most course results match R42. In noisy delayed
  jitter case 3, mean speed falls from 1245 to about 1204 mm/s and sensor SD
  rises from 1.884 to about 2.478 mm. This is not a general speed/stability gain.
- The older `testStraightStability()` check fails its legacy output-drop bound
  for noise40 at 10 ms in both frozen R42 and R43, with identical data. It is
  not counted as passing. Steering allocation can reduce actual wheel average
  faster than the common ramp. The full historical trajectory suite was not run.
- Leonardo build: **28,662/28,672 bytes flash**, **1508/2560 static RAM**.
  The map remains 512 bytes; no linked malloc/calloc/realloc/free.
  The 1052 bytes available for stack/locals are not a measured stack margin.
  Only ten flash bytes remain; use the pinned toolchain.

For the photographed track, start near the **middle of the long straight**,
well away from a crossing or turn, and point along the line. Recalibrate on the
actual mat and complete a **Safe** session first. Crossings retain map/gyro
steering and prohibit acceleration. The photos show S-bends, intersections and mat wrinkles;
capacity fixes alone do not establish physical tracking or finish accuracy.
Confirm lap 1 covers the whole course before replay begins. This candidate has
not run on the robot and does not re-localize a drifting map or change finish
detection. If it stops, record the exact text and its error/derivative pair.

## Install and upload

1. Extract `releases/PololuLineFollower-R45.zip`.
2. Open `PololuLineFollower/PololuLineFollower.ino` in Arduino IDE. Keep
   `RaceSession.h` beside it and replace **both files together**. Startup shows
   **R45**; mismatched planner revisions fail compilation.
3. Install **Pololu3piPlus32U4** through Library Manager and accept dependencies.
4. Select **Arduino Leonardo**, ATmega32U4 at 16 MHz, and the robot's USB port.
   The OLED edition is required; this is not firmware for Turtle/Hyper motors.
5. Upload, disconnect USB, and recalibrate. Development never flashes the robot
   automatically. Keep the previous release ZIP for rollback.

## Run

- **A:** automatic calibration. Allow the two-second hands-clear pause, then
  five slow, encoder-bounded turns with stopped pauses before reversal.
  Keep the robot on the line/white-floor boundary; do not turn it by hand.
- After calibration the OLED shows five sensor values and alignment arrows.
  Values run from 0 (white) to 1000 (black). Aim for `CENTERED` and the green
  LED, then point the robot along a straight. The screen cannot check heading.
- **C while stopped:** Safe â†’ Mid â†’ Beast â†’ Safe. Power-on defaults to Safe.
- **B:** release to arm, then keep the robot still during the two-second gyro
  bias measurement. The first lap learns slowly; the next two use the map.
- The robot stops after **three estimated laps** and holds the results.
  Another B starts a fresh learning session; R45 retains its completed-session
  reference until power-off. A reruns calibration.
- Any A/B/C button stops driving. During release-to-arm, the initiating A/B
  button may remain held. There is no blind lost-line recovery or auto restart.

Start from the same point and heading for comparisons. Use an open, flat test
area and first check a complete Safe session after upload. Then compare Beast
sessions on the same track and similar battery charge. Record completed/attempted
sessions, laps 2/3, scale/width, PWM, limiter and A/L/S shares.

Use four matching AAA NiMH cells with correct polarity and the stock covers on.
Charge the cells externally; USB does not charge them. No 10440 lithium cells.
The 4.40 V loaded cutoff is a stop threshold, not a target for timed comparisons.

## Modes and planning

All values are **PWM commands out of 400**, not measured speeds.

| Stage | Straight target | Wheel ceiling |
| --- | ---: | ---: |
| Learning, every mode | 80 | 100 |
| Safe replay | 120 | 140 |
| Mid replay | 180 | 220 |
| Beast replay | 400 | 400 |

Sustained noisy Beast learning selects a Mid fallback for both replay laps.
One isolated derivative spike cannot latch it, and a replay error cannot change
the session's motor ceiling. Errors during replay still reduce speed locally.

The 512-byte map adapts during learning:

| Capacity with stock encoder/wheel scale | Bin width | Stored distance per bin |
| --- | --- | --- |
| About 18.4 m | 128 counts | 36 mm |
| About 36.8 m | 256 counts | 72 mm |
| About 73.6 m | 512 counts | 144 mm |

Fine-bin curvature is averaged by traveled distance. Compression preserves
maximum magnitude across completed fine bins, rather than averaging opposite
turns together. Coarse bins therefore trade detail/pace for coverage. Their
signed peak is not used to predict steering direction, and their trailing bend
is not released early. Map resolution is frozen during replay.
Every B discards the old map and starts at fine resolution. Learning also
estimates speed/PWM scale and gyro polarity/effective wheel spacing.
A full maximum-resolution map stops with Map full; timeouts still apply.

Planning combines mapped corner lookahead, live gyro curvature, optical error
and filtered error derivative. The nominal lateral budgets are 1200/2400/5600 mm/s^2 for
Safe/Mid/qualified Beast, with a 12.5% drivetrain margin. Beast uses a lower
budget without trusted geometry. These are tuning assumptions, not measured
friction limits. Both replay laps use the same budget. Wheel ceilings and
differential steering room also constrain speed. No reverse wheel commands are used during line following.

Planned braking is 250 PWM/s for normal profiles, or 200 for noisy-learning
fallback.
The common output ramp brakes at 300 PWM/s. Large line errors/crossings retain
an urgent slowdown; faults stop immediately. Steering allocation can also reduce
wheel-average output immediately. The code therefore does not promise a limit
on every physical deceleration or eliminate pitching. Running bumper stops
remain disabled as requested; startup/calibration bumper checks remain active.

**Finish detection has no physical marker.** Encoder/gyro pose estimates can
drift with slip or repeated geometry. This code stops after three detected
returns; it cannot guarantee exactly three physical laps or exact finish timing.
Use a distinct finish reference if exact counting is required. Small loops,
branches, crossings and nearby parallel sections need separate track validation.

## Read the results

| Display | Meaning |
| --- | --- |
| `820/84` | Learned speed-scale Q8 / effective wheel width in mm. Divide 820 by 256 for model mm/s per PWM. |
| `A/L/S 33/22/16` | Integer replay tick percentages for acceleration or throttle restoration / optical line cap / steering allocation. |
| `LAP 1/2/3` | Estimated lap times; lap 1 is learning. |
| `PWM 280/290` | Peak actual wheel-average command / peak common throttle before steering trim. Independent maxima. |
| `LIMIT: M/Y/L/A/S/F` | Most frequent replay limiter: map / live yaw / optical line / acceleration / steering / satisfied cruise target. |
| `LIMIT: G` | Insufficient learned gyro geometry/polarity confidence. |
| `MID FALLBACK` | Sustained noisy learning selected the fallback. |

Tick-count ties choose M, L, A, S, F, then Y. A dominant `A` does not establish
how much time more acceleration could save. Shares use floor-rounded percentages
and need not total 100 because other limiter categories also exist.

A run stopped early holds its reason and last valid `error/filteredDerivative`
pair. After `NO LINE`, that pair is the preceding valid sample, not a measured
off-track position. The screen stays until A/B/C.

| Stop | Check |
| --- | --- |
| `BAD CAL`, `CENTRE!`, `NO LINE` | Calibration, line/white contrast, placement and sensor height. |
| `BAT LOW/HIGH` | Cell charge/type, polarity and contacts. |
| `STALLED`, `ENC DIR/ERR` | Free wheels, assembly, encoders and power. |
| `DARK/AIR` | Wide crossing, floor/sensor geometry or loss of contrast. |
| `LOOP LAG`, `IMU ERROR`, `GYRO STALE/RANGE` | Timing, IMU/I2C and unexpected motion. |
| `HOLD STILL`, `GYRO CAL` | Remain still throughout the start countdown. |
| `MAP FULL/GAP/MISMATCH`, `NO FINISH` | Map capacity, grip, loop geometry and return estimate. |
| `MOTION ERROR` | Motion outside encoder/gyro estimator bounds. |
| `USB OUT`, `STOPPED`, `TIME END` | Disconnect USB, deliberate cancellation or session timeout. |

Do not bypass a stop to force a finish. A nose strike or departure still needs
physical inspection/tuning; compilation cannot establish track stability.

## Validation

Run the focused checks with:

```powershell
python tests/run_tests.py --focused
```

They compile the actual sketch against fake hardware and check calibration,
startup/UI/buttons, mode ceilings, battery/USB/IMU/encoder/stall faults, watchdog
upload handoff, fractional ramps, timing rollover, predictive line limits, yaw
feedback, corner holds, output bounds, diagnostics and the three-lap stop latch.

Earlier planner, mismatch-window, launch, mode, fault, line-loss, calibration
and repeated-start checks remain. test_large_map.cpp checks pooling,
large positions, partial wrap, capacity/fault bounds and mixed turn directions.
Acceleration tests validate each sample before reading the qualified launch
state, matching the consolidated production control path.

Additional native checks replay 120 prescribed S-bend profiles through all map
resolutions, two travel rates, both directions, offsets of up to two bins and
fixed/jittered ticks, including partial-bin wrap. Maximum pending disagreement
was 30 ms; no mismatch stop occurred. Lap-boundary checks retain launch on a
clear straight and revoke it on a corner or crossing. These fixtures do not
validate learning, pose closure or grip on the photographed mat.

Learning-stage crossing checks also compare 18 centered/asymmetric input pairs,
including both map-compaction boundaries. Gyro-derived bends are recorded during
the crossing, and map/geometry estimates match without a false noise fallback.
These input fixtures suppress pose closure and do not establish physical grip.

For the R45 replay regression, compare frozen R44 with current firmware:

```powershell
python tests/compare_motion.py --baseline R44 --replay-only --output releases/PololuLineFollower-R45-replay.json
python tests/compare_motion.py --baseline R44 --replay-only --period-ms 10 --output releases/PololuLineFollower-R45-replay-10ms.json
```

The same current harness compiles against frozen R44 and current firmware.
Each report retains R44's failing control baseline and the four changing-radius
course comparisons. Both reports are included in the R45 ZIP. Earlier reports
and known failures remain in their frozen releases. No unrelated trajectory
models were rerun for R45.
The full historical trajectory suite was not run. Models do not calibrate
physical tire grip, battery sag or the shape of the photographed mat.

Warnings-enabled OLED/Leonardo builds use **28,662/28,672 bytes flash** and
**1508/2560 bytes static RAM**, without linked malloc/calloc/realloc/free.

Validation toolchain: Arduino AVR Boards 1.8.8, Pololu3piPlus32U4 1.1.3,
PololuOLED 2.0.0, FastGPIO 2.2.0, Pushbutton 2.0.0 and USBPause 2.0.0.
Different compiler/library versions can change resource usage. No library files
were modified. The compact Pololu-derived font and its MIT notice are embedded
in the sketch; emitted text renders in uppercase.

No hardware upload or physical R45 test was performed during development.
Firmware PWM bounds do not establish physical motor/gearbox protection or
collision prevention. Verify on your track before relying on competition runs.

## References

- [Pololu 3pi+ user guide](https://www.pololu.com/docs/0J83/all)
- [Official Arduino library](https://github.com/pololu/pololu-3pi-plus-32u4-arduino-library)
- [OLED library](https://github.com/pololu/pololu-oled-arduino)
