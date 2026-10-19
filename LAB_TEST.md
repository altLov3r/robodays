# Lab test record

Latest physical correction (2026-10-19): the large track stopped with **Map
mismatch**, and the robot took a corner incorrectly once. A Map full stop or
RAM shortage was not confirmed. The track includes S-bends and crossings.

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

R44 is an untested crossing-history candidate. R43's map-disagreement caution
remains; frozen R42/R43 are available for rollback. Software checks do not
confirm any candidate on this larger mat.

- Recalibrate on the actual mat. Start midway along the long straight, pointing
  along the line and away from crossings. Keep battery placement consistent.
- Confirm lap 1 covers the entire course before the display counts lap 2.
- Complete a Safe session before Mid/Beast. Leave the map/line-loss stops enabled.
- Watch crossing exits: R44 removes an artificial steering-history spike, but
  the mat still needs a physical test of line alignment and corner transitions.
- If a stop occurs, record its exact text and displayed error/derivative values.
  Record whether it happened at entry, middle, exit, crossing or an S-bend reversal.
- Report completion as well as lap time. Software models do not verify this mat.

| Version | Mode | Battery before/after | Lap 1/2/3 | Exact stop | Error/derivative | Location |
| --- | --- | --- | --- | --- | --- | --- |
| | | | | | | |

Preserve failed-run data. Do not change gains, battery arrangement and starting
position together: those changes make the next comparison difficult to interpret.
