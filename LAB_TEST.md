# Lab test record

Latest physical correction (2026-10-19): the large track stopped with **Map
mismatch**, and the robot took a corner incorrectly once. A Map full stop or
RAM shortage was not confirmed. The track includes S-bends and crossings.

2026-10-20 report: the slow learning lap follows the spiral curves; failures
occur on replay laps 2/3. At overlaps the robot brakes and briefly accelerates.
The user suspects reversed mapping; that is not confirmed. Tested revision and
mode were not supplied. R45 addresses reproduced replay/crossing control faults.

## Open on the lab PC

1. Sign in to GitHub as the repository owner or an authorized collaborator.
2. Open https://github.com/altLov3r/robodays.
3. Open `releases/` and download the intended frozen ZIP. Extract it and open
   `PololuLineFollower/PololuLineFollower.ino`, keeping `RaceSession.h` beside it.
4. Use Arduino Leonardo and the pinned library versions listed in README.md.

For editing, clone `https://github.com/altLov3r/robodays.git`. Before each lab
session, pull the latest commits after saving your own changes. The GitHub copy
contains firmware and test history; it does not upload to the robot automatically.

## First tests

R45 is an untested replay/crossing candidate. R43's map-disagreement caution
remains; frozen R42/R43/R44 are available for rollback. Software checks do not
confirm any candidate on this larger mat.

- Recalibrate on the actual mat. Start midway along the long straight, pointing
  along the line and away from crossings. Keep battery placement consistent.
- Confirm lap 1 covers the entire course before the display counts lap 2.
- Watch whether successive estimated lap changes occur at the same physical
  start point. The new approximate photo-course model can reproduce Map mismatch
  from gyro-scale/lap-anchor drift while still tracking the line. This is a
  sensitivity result, not a confirmed physical diagnosis; see
  `reports/photo-track-r45/README.md`.
- Complete a Safe session before Mid/Beast. Leave the map/line-loss stops enabled.
- Watch overlaps: speed should stay bounded by entry speed and mapped braking,
  without the previous snap to 65 PWM. Watch the first spiral after each crossing.
- Record the startup revision, selected mode and whether a failure is on the
  learning lap or replay. A video of a complete attempt would help locate phase drift.
- If a stop occurs, record its exact text and displayed error/derivative values.
  Record whether it happened at entry, middle, exit, crossing or an S-bend reversal.
- Report completion as well as lap time. Software models do not verify this mat.

| Version | Mode | Battery before/after | Lap 1/2/3 | Exact stop | Error/derivative | Location |
| --- | --- | --- | --- | --- | --- | --- |
| | | | | | | |

Preserve failed-run data. Do not change gains, battery arrangement and starting
position together: those changes make the next comparison difficult to interpret.
