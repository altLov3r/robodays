"""Compare a frozen release and current firmware in the same limited motion models."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
from zipfile import ZipFile


root = Path(__file__).resolve().parent.parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--baseline", default="R42")
parser.add_argument("--candidate", type=Path, default=root, help="Workspace containing candidate sketch files.")
parser.add_argument("--period-ms", type=int, choices=(5, 10), default=5)
parser.add_argument("--stress", action="store_true", help="Include motor/geometry/noise variations.")
parser.add_argument("--crossings-only", action="store_true", help="Compare crossing command traces only; no trajectory models.")
parser.add_argument("--replay-only", action="store_true", help="Compare reported replay faults and four changing-radius courses.")
parser.add_argument("--output", type=Path, required=True)
options = parser.parse_args()
compiler = shutil.which("g++") or "C:/msys64/mingw64/bin/g++.exe"
environment = os.environ.copy()
environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment["PATH"]

harness = r'''
#include <sstream>
#define main regression_main
#include "test_line_follower.cpp"
#undef main

int main()
{
  for (unsigned test = 0; test < MODEL_CASES; test++)
  {
    std::ostringstream diagnostic;
    std::streambuf * output = std::cout.rdbuf(diagnostic.rdbuf());
    if (test == 31)
    {
      readyFastReplay();
      race.mismatchMs = 30;
      const int target = baseSpeedForLine(0, 0, false);
      const int launch = race.launchCurveLimit;
      memset(race.curves, -80, sizeof(race.curves));
      race.filteredCurve = 240;
      const int assist = race.steeringFeedforward(100, 0);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":31,\"kind\":\"pending_mismatch\",\"target_pwm\":" << target
        << ",\"launch_limit\":" << launch << ",\"live_assist\":" << assist << ",\"complete\":"
        << (target <= 180 && launch == 0 && assist > 0) << "}\n";
    }
    else if (test == 32)
    {
      RaceSession session;
      session.reset(BEAST, 0);
      session.mapReady = true;
      session.completed = 1;
      session.learnedCounts = 20000;
      session.binsUsed = 157;
      memset(session.curves, 30, sizeof(session.curves));
      session.filteredCurve = -240;
      unsigned elapsed = 0;
      while (session.fault == RaceSession::NONE && elapsed < 300)
      {
        elapsed += 5;
        session.update(2, 2, -2 * RaceSession::MM_PER_COUNT * 30 / 6000, 5, 80, elapsed);
      }
      std::cout.rdbuf(output);
      std::cout << "{\"case\":32,\"kind\":\"mismatch\",\"fault\":" << int(session.fault)
        << ",\"elapsed_ms\":" << elapsed << ",\"complete\":"
        << (session.fault == RaceSession::MAP_MISMATCH && elapsed == 200) << "}\n";
    }
    else if (test == 29)
    {
      RaceSession session;
      session.reset(BEAST, 0);
      for (unsigned bin = 0; bin < 600 && session.fault == RaceSession::NONE; bin++)
      {
        // Map-storage fixture, not a synthetic lap/pose prediction.
        session.x = session.y = 1000;
        session.heading = 0;
        session.leftStartZone = session.returnArmed = false;
        session.update(128, 128, 0, 10, 80, 10);
      }
      if (session.fault == RaceSession::NONE)
      {
        session.x = -0.1f; session.y = session.heading = 0;
        session.leftStartZone = session.returnArmed = true;
        session.update(1, 1, 0, 10, 80, 20);
      }
      std::cout.rdbuf(output);
      std::cout << "{\"case\":29,\"kind\":\"large_map\",\"fault\":" << int(session.fault)
        << ",\"learned_counts\":" << session.learnedCounts << ",\"bins\":" << session.binsUsed
        << ",\"map_bytes\":" << sizeof(session.curves) << ",\"complete\":"
        << (session.fault == RaceSession::NONE && session.mapReady && session.learnedCounts == 76801)
        << "}\n";
    }
    else if (test == 30)
    {
      const auto data = testRoundedSquare(BEAST, 5000, 200, 1, 3.25f, 40, 1, 86, 0,
        0.25f, 0x51a7u, false, MODEL_PERIOD);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":30,\"kind\":\"square\",\"lap2\":" << data.lapTimes[1]
        << ",\"lap3\":" << data.lapTimes[2] << ",\"max_offset\":"
        << std::max(data.maxCrossTrack[1], data.maxCrossTrack[2])
        << ",\"complete\":" << (race.completed == 3 && state == IDLE && race.fault == RaceSession::NONE)
        << ",\"fault\":" << int(race.fault) << "}\n";
    }
    else if (test == 28)
    {
      RaceSession session;
      session.reset(BEAST, 0);
      session.mapReady = true;
      session.completed = 1;
      session.learnedCounts = 20000;
      session.binsUsed = (session.learnedCounts + 127) / 128;
      session.speedScaleQ8 = 820;
      session.trackWidthMm = 84;
      session.turnVotes = 8;
      session.speedMmS = 1000;
      memset(session.curves, 0, sizeof(session.curves));
      for (unsigned bin = 20; bin < 30; bin++) { session.curves[bin] = 30; }
      session.distanceTwice = 20 * RaceSession::BIN_COUNTS * 2;
      const int corner = session.plannedCruise(false);
      session.distanceTwice = (20 * RaceSession::BIN_COUNTS - 720) * 2;
      const int entry = session.plannedCruise(false);
      memset(session.curves, 0, sizeof(session.curves));
      const int straight = session.plannedCruise(true);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":28,\"kind\":\"corner_entry\",\"corner_pwm\":" << corner
        << ",\"early_entry_pwm\":" << entry << ",\"straight_pwm\":" << straight
        << ",\"complete\":" << (entry <= corner && corner <= 293 && straight == 400) << "}\n";
    }
    else if (test == 27)
    {
      RaceSession session;
      const auto replay = [&](uint16_t scale)
      {
        session.reset(BEAST, 0);
        session.mapReady = true;
        session.completed = 2;
        session.learnedCounts = 20000;
        session.binsUsed = (session.learnedCounts + 127) / 128;
        session.speedScaleQ8 = scale;
        session.trackWidthMm = 84;
        session.turnVotes = 8;
        memset(session.curves, 30, sizeof(session.curves));
      };
      replay(820);
      const int referenceCorner = session.plannedCruise(false), referenceYaw = session.yawScale();
      session.distanceTwice = (session.learnedCounts - 20) * 2;
      session.x = -1;
      session.leftStartZone = session.returnArmed = true;
      assert(session.update(20, 20, 0, 10, 80, 10) == RaceSession::SESSION_FINISHED);
      replay(700);
      const int laterCorner = session.plannedCruise(false), laterYaw = session.yawScale();
      memset(session.curves, 0, sizeof(session.curves));
      const int straight = session.plannedCruise(true);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":27,\"kind\":\"repeat_session\",\"reference_corner_pwm\":" << referenceCorner
        << ",\"later_corner_pwm\":" << laterCorner << ",\"reference_yaw_scale\":" << referenceYaw
        << ",\"later_yaw_scale\":" << laterYaw << ",\"straight_pwm\":" << straight
        << ",\"complete\":" << (laterCorner <= referenceCorner && laterYaw <= referenceYaw && straight == 400)
        << "}\n";
    }
    else if (test == 25 || test == 26)
    {
      RaceSession session;
      session.reset(BEAST, 0);
      session.mapReady = true;
      session.completed = 1;
      session.binsUsed = 32;
      session.learnedCounts = 32 * RaceSession::BIN_COUNTS;
      session.distanceTwice = 10 * RaceSession::BIN_COUNTS * 2;
      memset(session.curves, 0, sizeof(session.curves));
      session.curves[12] = 30;
      const int curve = test == 25 ? 30 : 90;
      session.filteredCurve = curve * 8;
      unsigned elapsed = 0;
      while (session.fault == RaceSession::NONE && elapsed < 300)
      {
        elapsed += 5;
        session.update(2, 2, 2 * RaceSession::MM_PER_COUNT * curve / 6000, 5, 100, elapsed);
      }
      const bool complete = session.fault == (test == 25 ? RaceSession::NONE : RaceSession::MAP_MISMATCH);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":" << test << ",\"kind\":\"mismatch\",\"fault\":" << int(session.fault)
        << ",\"elapsed_ms\":" << elapsed << ",\"complete\":" << complete << "}\n";
    }
    else if (test == 24)
    {
      readyFastReplay();
      setStraightHistory(race, 0, 0);
      forwardCommand = leftCommand = rightCommand = 200;
      motors.setSpeeds(200, 200);
      unsigned elapsed = 0;
      while (state == RUNNING && forwardCommand < 300 && elapsed < 1000)
      { tick(5); elapsed += 5; }
      std::cout.rdbuf(output);
      std::cout << "{\"case\":24,\"kind\":\"settling\",\"time_ms\":" << elapsed
        << ",\"complete\":" << (state == RUNNING && forwardCommand >= 300) << "}\n";
    }
    else if (test == 23)
    {
      readyFastReplay();
      setStraightHistory(race, 0, 0);
      race.curves[0] = race.curves[race.binsUsed - 1] = 30;
      Fake::directCounts = true;
      Fake::leftCounts = Fake::rightCounts = 2;
      forwardCommand = 400;
      leftCommand = rightCommand = 280;
      motors.setSpeeds(280, 280);
      tick(5);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":23,\"kind\":\"release\",\"before_pwm\":280,\"after_pwm\":"
        << (leftCommand + rightCommand) / 2 << ",\"complete\":" << (state == RUNNING) << "}\n";
    }
    else if (test == 8)
    {
      readyFastReplay();
      Fake::position = 2100;
      lastError = 100;
      forwardCommand = leftCommand = rightCommand = 200;
      motors.setSpeeds(200, 200);
      unsigned elapsed = 0;
      while (state == RUNNING && forwardCommand < 300 && elapsed < 1000)
      { tick(5); elapsed += 5; }
      std::cout.rdbuf(output);
      std::cout << "{\"case\":8,\"kind\":\"command\",\"time_ms\":" << elapsed
        << ",\"reached_pwm\":" << forwardCommand
        << ",\"complete\":" << (state == RUNNING && forwardCommand >= 300) << "}\n";
    }
    else if (test < 4 || test >= 21)
    {
      const int noise = test == 0 || test >= 21 ? 40 : 100;
      const float mismatch = test == 1 ? 0.03f : (test == 2 ? -0.03f : 0);
      const auto data = testDynamicStraight(noise, mismatch, 1, test == 3, test == 3, false,
        MODEL_PERIOD, test >= 21 ? (test == 21 ? 1 : -1) : 0);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":" << test << ",\"kind\":\"straight\",\"speed\":" << data.meanSpeed
        << ",\"sensor_sd\":" << data.sensorStdDev << ",\"max_offset\":" << data.steadyMaxLateral
        << ",\"steady_brake\":" << data.steadyMaxDrop
        << ",\"peak_rejoin_offset\":" << data.maxLateral
        << ",\"complete\":" << (state == RUNNING && race.fault == RaceSession::NONE) << "}\n";
    }
    else
    {
      struct Course { float straight, radius, direction, gain, width, mismatch, response; int noise; };
      const Course courses[] = {
        {600,200, 1,3.25f,86,0,0.25f,40},
        {600,200,-1,3.25f,86,0,0.25f,40},
        {600,200, 1,3.25f,86,0.03f,0.25f,100},
        {400,100, 1,3.25f,86,0,0.25f,40},
        {600,200, 1,4.0f,86,0,0.15f,100},
        {600,200,-1,4.0f,86,0.03f,0.15f,100},
        {400,100, 1,3.75f,86,0,0.15f,100},
        {400,100,-1,3.75f,86,-0.03f,0.15f,100},
        {300,80,  1,3.25f,75,0.03f,0.20f,100},
        {300,80, -1,3.25f,100,-0.03f,0.20f,100},
        {800,250, 1,2.75f,86,0.05f,0.12f,120},
        {800,250,-1,2.75f,86,-0.05f,0.12f,120},
      };
      const auto & course = courses[test >= 17 ? test - 17 : (test < 8 ? test - 4 : test - 5)];
      const int shift = test >= 17 ? (test % 2 ? -1 : 1) : 0;
      const auto data = testRoundedSquare(BEAST, course.straight, course.radius,
        course.direction, course.gain, course.noise, 1, course.width, course.mismatch,
        course.response, 0x51a7u, false, MODEL_PERIOD, shift);
      std::cout.rdbuf(output);
      std::cout << "{\"case\":" << test << ",\"kind\":\"square\",\"lap2\":" << data.lapTimes[1]
        << ",\"lap3\":" << data.lapTimes[2] << ",\"max_offset\":"
        << std::max(data.maxCrossTrack[1], data.maxCrossTrack[2])
        << ",\"brake\":" << std::max(data.maxCommonDrop[1], data.maxCommonDrop[2])
        << ",\"straight_speed\":" << (data.straightSpeed[1] + data.straightSpeed[2]) / 2
        << ",\"entry_speed\":" << (data.cornerSpeed[1] + data.cornerSpeed[2]) / 2
        << ",\"map_shift_bins\":" << shift
        << ",\"complete\":" << (race.completed == 3 && state == IDLE && race.fault == RaceSession::NONE)
        << ",\"fault\":" << int(race.fault) << "}\n";
    }
  }
}
'''

results = {}
if options.replay_only:
    harness = r'''
#include <sstream>
#define main regression_main
#include "test_line_follower.cpp"
#undef main
int main()
{
  const auto faults = testReplayFaults(MODEL_PERIOD);
  std::cout << "{\"case\":0,\"kind\":\"replay_faults\",\"fixtures\":" << faults.cases
    << ",\"failed\":" << faults.failed << ",\"wrong_turns\":" << faults.wrongTurns
    << ",\"lost_crossing_turns\":" << faults.lostCrossingTurns
    << ",\"max_crossing_drop\":" << faults.maxCrossingDrop
    << ",\"max_exit_rise\":" << faults.maxExitRise
    << ",\"braking_change\":" << faults.brakingChange
    << ",\"complete\":" << (faults.failed == 0) << "}\n";
  for (unsigned test = 1; test <= 4; test++)
  {
    std::ostringstream diagnostic;
    auto output = std::cout.rdbuf(diagnostic.rdbuf());
    const int direction = test <= 2 ? -1 : 1;
    const float warp = test % 2 ? -0.6f : 0.6f;
    const auto data = testRoundedSquare(BEAST,600,150,direction,3.75f,40,1,86,0,
      0.25f,0x51a7u,false,MODEL_PERIOD,0,warp);
    std::cout.rdbuf(output);
    std::cout << "{\"case\":" << test << ",\"kind\":\"square\",\"corner_warp\":" << warp
      << ",\"direction\":" << direction << ",\"lap2\":" << data.lapTimes[1]
      << ",\"lap3\":" << data.lapTimes[2] << ",\"max_offset\":"
      << std::max(data.maxCrossTrack[1],data.maxCrossTrack[2])
      << ",\"complete\":" << (race.completed == 3 && state == IDLE && race.fault == RaceSession::NONE)
      << ",\"fault\":" << int(race.fault) << "}\n";
  }
}
'''
if options.crossings_only:
    harness = r'''
#define main regression_main
#include "test_line_follower.cpp"
#undef main
int main()
{
  const auto data = testCrossingHistory(MODEL_PERIOD);
  std::cout << "{\"case\":0,\"kind\":\"crossing\",\"pairs\":" << data.cases
    << ",\"failed_pairs\":" << data.failed
    << ",\"max_exit_delta_difference\":" << data.maxExitDeltaDifference
    << ",\"max_exit_motor_difference\":" << data.maxExitMotorDifference
    << ",\"complete\":" << (data.failed == 0) << "}\n";
}
'''
with tempfile.TemporaryDirectory(prefix="robodays-motion-") as temporary:
    workspace = Path(temporary).resolve()
    assert workspace.is_relative_to(Path(tempfile.gettempdir()).resolve())
    for label in (options.baseline, "current"):
        folder = workspace / label
        source = folder / "PololuLineFollower"
        source.mkdir(parents=True)
        for name in ("PololuLineFollower.ino", "RaceSession.h"):
            if label == "current":
                shutil.copy2(options.candidate / "PololuLineFollower" / name, source / name)
            else:
                with ZipFile(root / "releases" / f"PololuLineFollower-{label}.zip") as archive:
                    (source / name).write_bytes(archive.read(f"PololuLineFollower/{name}"))
        tests = folder / "tests"
        tests.mkdir()
        shutil.copy2(root / "tests/test_line_follower.cpp", tests / "test_line_follower.cpp")
        shutil.copytree(root / "tests/fakes", tests / "fakes")
        case_count = 1 if options.crossings_only else (5 if options.replay_only else (33 if options.stress else 9))
        source_text = harness.replace("MODEL_PERIOD", str(options.period_ms)).replace("MODEL_CASES", str(case_count))
        (tests / "motion.cpp").write_text(source_text, encoding="utf-8")
        executable = tests / "motion.exe"
        build = subprocess.run([compiler, "-std=c++11", "-O2", "-Wall", "-Wextra", "-Werror",
                        "-I", str(tests / "fakes"), str(tests / "motion.cpp"), "-o", str(executable)],
                       env=environment, capture_output=True, text=True)
        if build.returncode:
            raise SystemExit(build.stderr)
        run = subprocess.run([str(executable)], env=environment,
                             capture_output=True, text=True, timeout=60)
        if run.returncode:
            raise SystemExit(run.stdout + run.stderr)
        results[label] = [json.loads(line) for line in run.stdout.splitlines() if line.startswith("{")]
        assert len(results[label]) == case_count
    options.output.write_text(json.dumps({
        "scope": (f"54 centered/asymmetric crossing command-trace pairs, {options.period_ms} ms; no trajectory or grip model."
                  if options.crossings_only else
                  f"15 replay control fixtures and four changing-radius course models at {options.period_ms} ms; no calibrated grip or photographed-track prediction."
                  if options.replay_only else
                  f"{23 if options.stress else 8} uncalibrated motion cases, {options.period_ms} ms replay (jitter case 10-20 ms), plus {'command, throttle-release, settling, three mismatch, repeated-session, corner-entry, large-map and pending-mismatch fixtures' if options.stress else 'one command fixture'}; no physical lap prediction or calibrated tire friction."),
        "results": results,
    }, indent=2) + "\n", encoding="utf-8")

for old, new in zip(results[options.baseline], results["current"]):
    assert old["case"] == new["case"]
    if old["complete"] and not new["complete"]:
        raise SystemExit(f"Model regression in case {old['case']}; see {options.output}")
    if new["kind"] == "replay_faults":
        print(f"Replay fixtures: {old['failed']}->{new['failed']} failed; wrong turns "
              f"{old['wrong_turns']}->{new['wrong_turns']}; crossing drop "
              f"{old['max_crossing_drop']}->{new['max_crossing_drop']} PWM")
    elif new["kind"] == "crossing":
        print(f"Crossing pairs: {old['failed_pairs']} -> {new['failed_pairs']} differing traces; "
              f"extra exit differential {old['max_exit_motor_difference']} -> {new['max_exit_motor_difference']} PWM")
    elif new["kind"] == "pending_mismatch":
        if not new["complete"]:
            raise SystemExit(f"Pending mismatch control failed; see {options.output}")
        print(f"Case 31: pending target {old['target_pwm']}->{new['target_pwm']} PWM; "
              f"launch {old['launch_limit']}->{new['launch_limit']}; "
              f"live assist {old['live_assist']}->{new['live_assist']}")
    elif new["kind"] == "large_map":
        if not new["complete"]:
            raise SystemExit(f"Large map failed; see {options.output}")
        print(f"Case 29: storage complete {old['complete']} -> {new['complete']}; "
              f"fault {old['fault']} -> {new['fault']}; {new['map_bytes']} map bytes")
    elif new["kind"] == "corner_entry":
        if not new["complete"]:
            raise SystemExit(f"Corner entry reserve failed; see {options.output}")
        print(f"Case 28: early entry {old['early_entry_pwm']} -> {new['early_entry_pwm']} PWM; "
              f"corner {old['corner_pwm']} -> {new['corner_pwm']}; straight target {new['straight_pwm']}")
    elif new["kind"] == "repeat_session":
        if not new["complete"]:
            raise SystemExit(f"Repeated-session guard failed; see {options.output}")
        print(f"Case 27: lower later estimate, corner PWM {old['reference_corner_pwm']}->{old['later_corner_pwm']} vs "
              f"{new['reference_corner_pwm']}->{new['later_corner_pwm']}; "
              f"yaw scale {old['reference_yaw_scale']}->{old['later_yaw_scale']} vs "
              f"{new['reference_yaw_scale']}->{new['later_yaw_scale']}; straight target {new['straight_pwm']}")
    elif new["kind"] == "mismatch":
        if not new["complete"]:
            raise SystemExit(f"Mismatch fixture {new['case']} failed; see {options.output}")
        print(f"Case {new['case']}: mismatch fault {old['fault']} -> {new['fault']}; "
              f"elapsed {old['elapsed_ms']} -> {new['elapsed_ms']} ms")
    elif new["kind"] == "settling":
        print(f"Case 24: pending-straight 200->300 command {old['time_ms']} -> {new['time_ms']} ms")
    elif new["kind"] == "release":
        print(f"Case 23: hidden-throttle release {old['before_pwm']}->{old['after_pwm']} vs "
              f"{new['before_pwm']}->{new['after_pwm']} PWM")
    elif new["kind"] == "command":
        print(f"Case 8: fixed-input command recovery {old['time_ms']} -> {new['time_ms']} ms")
    elif new["kind"] == "square":
        print(f"Case {new['case']}: complete {old['complete']} -> {new['complete']}; "
              f"model laps {old['lap2']}/{old['lap3']} -> {new['lap2']}/{new['lap3']} ms; "
              f"max offset {old['max_offset']:.2f} -> {new['max_offset']:.2f} mm")
    else:
        print(f"Case {new['case']}: model speed {old['speed']:.1f} -> {new['speed']:.1f}; "
              f"sensor SD {old['sensor_sd']:.3f} -> {new['sensor_sd']:.3f} mm")

if not all(case["complete"] for case in results["current"]):
    raise SystemExit(f"Current model/fixture incomplete; see {options.output}")
