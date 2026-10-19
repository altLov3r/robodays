#define main regression_main
#include "test_line_follower.cpp"
#undef main

void pendingStraight()
{
  readyFastReplay();
  race.straightStableMs = 0;
  race.launchCurveLimit = 32;
}

void testContinuousAlignment()
{
  for (const uint16_t period : {5, 7, 10, 20, 50})
  {
    pendingStraight();
    const unsigned required = Settings::STRAIGHT_SETTLE_MS;
    const unsigned increment = std::min<unsigned>(period, 10);
    unsigned total = 0;
    while (total < required)
    {
      qualifyStraight(0, 0, false, period);
      total = std::min(total + increment, required);
      assert(race.straightStableMs == total);
      assert(accelerationForTest(0, 0, false) == (total == required ? 24 : 6));
    }
    qualifyStraight(200, 0, false, 5);
    assert(race.straightStableMs == 0);
    qualifyStraight(0, 0, false, 5);
    assert(race.straightStableMs == 5 && accelerationForTest(0, 0, false) == 6);
  }
  struct Sample { int error, delta; bool acquire, hold; };
  const Sample samples[] = {
    {159,0,true,true}, {160,0,false,true}, {179,0,false,true}, {180,0,false,false},
    {0,59,true,true}, {0,60,false,true}, {0,79,false,true}, {0,80,false,false},
    {49,50,true,true}, {50,50,false,true}, {30,70,false,false}
  };
  for (const auto & sample : samples)
  for (const int sign : {-1, 1})
  {
    pendingStraight();
    qualifyStraight(sign * sample.error, sign * sample.delta, false, 5);
    assert(race.straightStableMs == (sample.acquire ? 5 : 0));
    race.straightStableMs = 40;
    qualifyStraight(sign * sample.error, sign * sample.delta, false, 5);
    assert(race.straightStableMs == (sample.hold ? 40 : 0));
  }
  for (unsigned condition = 0; condition < 5; condition++)
  {
    readyFastReplay();
    if (condition == 0) { race.launchCurveLimit = 0; }
    if (condition == 1) { race.launchCurveLimit = 96; }
    if (condition == 2) { race.exitQuietMs = 39; }
    if (condition == 3) { race.filteredCurve = -32; }
    qualifyStraight(0, 0, condition == 4, 5);
    assert(race.straightStableMs == 0);
  }
  readyFastReplay();
  race.abort();
  assert(race.straightStableMs == 0);
  readyFastReplay();
  race.reset(BEAST, Fake::now);
  assert(race.straightStableMs == 0);
}

void testHiddenThrottleRelease()
{
  for (const int direction : {-1, 1})
  {
    pendingStraight();
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 2;
    race.curves[0] = race.curves[race.binsUsed - 1] = direction * 30;
    forwardCommand = 400;
    leftCommand = rightCommand = 280;
    motors.setSpeeds(280, 280);
    const unsigned writes = Fake::displayWrites;
    tick(5);
    // R36 jumped from 280 to 400 when the line/yaw sample briefly centered.
    assert(state == RUNNING && forwardCommand == 400);
    assert(leftCommand == 283 && rightCommand == 283);
    assert(race.straightStableMs == 0 && speedLimiter == ACCEL_LIMIT);
    for (const int error : {100, -100, 200, -200, 0})
    {
      const int previous = (leftCommand + rightCommand) / 2;
      Fake::position = 2000 + direction * error;
      tick(5);
      assert(state == RUNNING && race.straightStableMs == 0);
      assert((leftCommand + rightCommand) / 2 <= previous + 3);
      assert(leftCommand >= 0 && leftCommand <= 400 && rightCommand >= 0 && rightCommand <= 400);
    }
    assert(Fake::displayWrites == writes);
  }
  readyFastReplay();
  forwardCommand = 400;
  leftCommand = rightCommand = 300;
  motors.setSpeeds(300, 300);
  race.straightStableMs = 30;
  tick(5);
  // Crossing the qualification threshold must also ramp out hidden throttle.
  assert(race.straightStableMs == 35 && forwardCommand == 400);
  assert(leftCommand == 312 && rightCommand == 312);

  readyFastReplay();
  forwardCommand = leftCommand = rightCommand = 200;
  motors.setSpeeds(200, 200);
  tick(5);
  assert(leftCommand == 212 && rightCommand == 212);
  advance(100);
  assert(leftCommand == 400 && rightCommand == 400);
}

void testDeliveredTurnAssistance()
{
  readyFastReplay();
  memset(race.curves, 30, sizeof(race.curves));
  race.filteredCurve = 240;
  forwardCommand = 400;
  leftCommand = rightCommand = 200;
  motors.setSpeeds(200, 200);
  Fake::position = 2100; lastError = 100;
  tick(5);
  // Assist is 43 at the delivered base of 200, not 86 at hidden throttle 400.
  assert((leftCommand - rightCommand) / 2 == 73);
  assert((leftCommand + rightCommand) / 2 == 203);

  for (const bool bend : {false, true})
  {
    readyFastReplay();
    if (bend) { memset(race.curves, 30, sizeof(race.curves)); race.filteredCurve = 240; }
    forwardCommand = leftCommand = rightCommand = 200;
    motors.setSpeeds(200, 200);
    Fake::position = 2040; lastError = 40; filteredDelta = 56;
    tick(5);
    assert(filteredDelta == 40);
    // Straight optical D=64; bend optical D=96, with its existing turn assistance.
    assert((leftCommand - rightCommand) / 2 == (bend ? 76 : 9));
  }
}

void modelReplay(RaceSession & session, uint16_t scale)
{
  session.reset(BEAST, 0);
  session.mapReady = true;
  session.completed = 1;
  session.learnedCounts = 20000;
  session.binsUsed = (session.learnedCounts + 127) / 128;
  session.speedScaleQ8 = scale;
  session.trackWidthMm = 84;
  session.turnVotes = 8;
  memset(session.curves, 30, sizeof(session.curves));
}

void completeReferenceSession(RaceSession & session, uint32_t now = 10)
{
  session.completed = 2;
  session.distanceTwice = (session.learnedCounts - 20) * 2;
  session.x = -1;
  session.y = session.heading = 0;
  session.leftStartZone = session.returnArmed = true;
  assert(session.update(20, 20, 0, 10, 80, now) == RaceSession::SESSION_FINISHED);
  assert(session.fault == RaceSession::NONE && !session.running);
}

void testCompletedDriveReference()
{
  RaceSession session, reference, unguarded;
  modelReplay(session, 820);
  modelReplay(reference, 820);
  modelReplay(unguarded, 700);
  const int corner = reference.plannedCruise(false);
  const unsigned yaw = reference.yawScale();
  assert(unguarded.plannedCruise(false) > corner && unguarded.yawScale() > yaw);
  assert(session.completedScaleQ8 == 0);
  completeReferenceSession(session);
  assert(session.completedScaleQ8 == 820);
  session.abort();
  for (const uint16_t scale : {820, 810, 780, 700, 640, 820, 900})
  {
    modelReplay(session, scale);
    modelReplay(reference, std::max<uint16_t>(820, scale));
    assert(session.completedScaleQ8 == 820 && session.speedScaleQ8 == scale);
    for (const int curve : {0, 8, 20, 30, 60, 100})
    {
      memset(session.curves, curve, sizeof(session.curves));
      memset(reference.curves, curve, sizeof(reference.curves));
      for (const float speed : {0.0f, 400.0f, 1000.0f, 1500.0f})
      {
        session.speedMmS = reference.speedMmS = speed;
        assert(session.plannedCruise(false) == reference.plannedCruise(false));
        assert(session.yawScale() == reference.yawScale());
      }
    }
  }
  // Changing a floor while the cache is warm must update its full model.
  modelReplay(session, 700);
  session.completedScaleQ8 = 900;
  modelReplay(reference, 900);
  assert(session.yawScale() == reference.yawScale());
  assert(session.plannedCruise(false) == reference.plannedCruise(false));
  session.completedScaleQ8 = 820;
  modelReplay(reference, 820);
  assert(session.plannedCruise(false) == reference.plannedCruise(false));
  assert(session.yawScale() == reference.yawScale());

  // Safe, Mid, noisy and untrusted Beast retain their original conversion.
  for (const DriveMode mode : {SAFE, MID, BEAST})
  for (const bool noisy : {false, true})
  for (const int votes : {0, 3, 8, -8})
  {
    if (mode == BEAST && !noisy && abs(votes) >= 4) { continue; }
    modelReplay(session, 700);
    modelReplay(reference, 700);
    session.mode = reference.mode = mode;
    session.noisyLearning = reference.noisyLearning = noisy;
    session.turnVotes = reference.turnVotes = votes;
    assert(session.plannedCruise(false) == reference.plannedCruise(false));
    assert(session.yawScale() == reference.yawScale());
    completeReferenceSession(session);
    assert(session.completedScaleQ8 == 820);
  }
  modelReplay(session, 900);
  session.abort();
  assert(session.completedScaleQ8 == 820);
  modelReplay(session, 900);
  session.fault = RaceSession::MAP_MISMATCH;
  assert(session.update(20, 20, 0, 10, 80, 10) == RaceSession::NO_EVENT);
  assert(session.completedScaleQ8 == 820);
  modelReplay(session, 900);
  completeReferenceSession(session);
  assert(session.completedScaleQ8 == 900);
  modelReplay(session, 700);
  completeReferenceSession(session);
  assert(session.completedScaleQ8 == 900);
  session = RaceSession();
  assert(session.completedScaleQ8 == 0);
}

void testRepeatedSessionStartup()
{
  resetRobot();
  calibrateRobot();
  selectedMode = BEAST;
  for (unsigned attempt = 0; attempt < 7; attempt++)
  {
    Fake::gyroRaw = int(attempt) * 10 - 30;
    press(1);
    while (state == COUNTDOWN) { tick(); }
    assert(state == RUNNING && gyroBias == Fake::gyroRaw);
    assert(race.running && !race.mapReady && race.completed == 0 && race.fault == RaceSession::NONE);
    assert(race.speedScaleQ8 == 1024 && race.trackWidthMm == 86 && race.turnVotes == 0);
    assert(race.distanceTwice == 0 && race.binsUsed == 0 && race.filteredCurve == 0);
    assert(race.straightStableMs == 0 && race.exitQuietMs == 0 && race.mismatchMs == 0);
    assert(forwardCommand == 0 && leftCommand == 0 && rightCommand == 0 && filteredDelta == 0);
    assert(race.completedScaleQ8 == (attempt ? 820 : 0));
    for (const auto curve : race.curves) { assert(curve == RaceSession::UNKNOWN_CURVE); }
    for (const auto lap : race.lapTimes) { assert(lap == 0); }
    modelReplay(race, attempt ? 700 : 820);
    race.lapStarted = Fake::now;
    completeReferenceSession(race, Fake::now + 10);
    showResults();
    assert(Fake::leftMotor == 0 && Fake::rightMotor == 0 && race.completedScaleQ8 == 820);
    advance(100);
  }
}

void testPendingMismatchControl()
{
  readyFastReplay();
  for (const uint8_t pending : {0, 5, 20, 29})
  {
    race.mismatchMs = pending;
    assert(baseSpeedForLine(0, 0, false) == 400);
  }
  race.mismatchMs = 30;
  assert(baseSpeedForLine(0, 0, false) == 180 && speedLimiter == MAP_LIMIT);
  assert(race.launchCurveLimit == 0);
  qualifyStraight(0, 0, false, 5);
  assert(race.straightStableMs == 0 && accelerationForTest(0, 0, false) == 6);
  // A disagreed fine map must not command its stored, opposing turn direction.
  for (const int sign : {-1, 1})
  {
    memset(race.curves, -sign * 80, sizeof(race.curves));
    race.filteredCurve = sign * 30 * 8;
    assert(race.steeringFeedforward(100, 0) * sign > 0);
  }

  for (const int sign : {-1, 1})
  {
    readyFastReplay();
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 10;
    const float turn = 10 * RaceSession::MM_PER_COUNT * sign * 30 / 6000;
    Fake::gyroRaw = lroundf(turn / (0.07f * 3.14159265f / 180 * 0.005f));
    race.filteredCurve = sign * 240;
    forwardCommand = leftCommand = rightCommand = 300;
    motors.setSpeeds(300, 300);
    const unsigned displayWrites = Fake::displayWrites;
    for (unsigned elapsed = 5; elapsed <= 200; elapsed += 5)
    {
      const int previous = (leftCommand + rightCommand) / 2;
      tick(5);
      if (elapsed < 200)
      {
        assert(state == RUNNING && race.mismatchMs == elapsed);
        assert(race.straightStableMs == 0 && race.profile().limit == 400);
        assert(previous - (leftCommand + rightCommand) / 2 <= 2);
        if (elapsed >= 30) { assert(baseSpeedForLine(0, 0, false) <= 180); }
        assert(Fake::displayWrites == displayWrites);
      }
    }
    expectStopped("Map mismatch");
    assert(race.fault == RaceSession::MAP_MISMATCH && race.mismatchMs == 200);
  }
  readyFastReplay();
  race.mismatchMs = 80;
  race.straightStableMs = 0;
  forwardCommand = leftCommand = rightCommand = 200;
  motors.setSpeeds(200, 200);
  tick(5); // A fresh agreeing sample clears the pending mismatch.
  assert(state == RUNNING && race.mismatchMs == 0 && race.straightStableMs == 5);
  assert(leftCommand == 203 && rightCommand == 203);
}

void testLapBoundaryQualification()
{
  // Clear straight, corner at bin zero, and crossing at the estimated finish.
  for (const int condition : {0, 1, 2})
  {
    readyFastReplay();
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 2;
    race.straightStableMs = Settings::STRAIGHT_SETTLE_MS;
    race.distanceTwice = (race.learnedCounts - 1) * 2;
    race.x = -0.1f;
    race.y = race.heading = 0;
    race.leftStartZone = race.returnArmed = true;
    if (condition == 1) { race.curves[0] = race.curves[1] = 30; }
    if (condition == 2) { for (auto & reading : Fake::readings) { reading = 900; } }
    tick(5);
    assert(state == RUNNING && race.completed == 2 && race.distanceCounts() == 0);
    assert(race.straightStableMs == (condition ? 0 : Settings::STRAIGHT_SETTLE_MS));
    assert(accelerationForTest(0, 0, condition == 2) == (condition ? 6 : 24));
    assert(leftCommand >= 0 && leftCommand <= 400 && rightCommand >= 0 && rightCommand <= 400);
  }
}

void testCrossingStops()
{
  readyFastReplay();
  for (auto & reading : Fake::readings) { reading = 1000; }
  tick(5);
  for (auto & reading : Fake::readings) { reading = 0; }
  tick(5);
  expectStopped("No line");

  readyFastReplay();
  Fake::directCounts = true;
  Fake::leftCounts = Fake::rightCounts = 4;
  const float turn = 4 * RaceSession::MM_PER_COUNT * 30 / 6000;
  Fake::gyroRaw = lroundf(turn / (0.07f * 3.14159265f / 180 * 0.005f));
  race.filteredCurve = 240;
  race.mismatchMs = 190;
  for (auto & reading : Fake::readings) { reading = 1000; }
  tick(5);
  assert(state == RUNNING && race.mismatchMs == 195);
  tick(5);
  expectStopped("Map mismatch");
  assert(race.mismatchMs == 200);
}

void testCrossingLearning()
{
  unsigned cases = 0;
  for (const int polarity : {-1, 1})
  for (const uint32_t start : {0u, 65530u, 131066u})
  for (const unsigned duration : {1u, 8u, 18u})
  {
    RaceSession reference;
    std::vector<int> referenceTrace;
    for (const bool asymmetric : {false, true})
    {
      readyRun();
      selectedMode = BEAST;
      race.reset(BEAST, Fake::now);
      // Storage/controller fixture, including samples that trigger compaction.
      race.distanceTwice = start * 2;
      race.mapShift = start > 65536 ? 1 : 0;
      race.binsUsed = (start + race.mapBinCounts() - 1) / race.mapBinCounts();
      memset(race.curves, 0, race.binsUsed);
      race.mapWeight = (start % RaceSession::BIN_COUNTS) * 2;
      race.x = race.y = 1000; // Suppress pose closure; no synthetic lap claim.
      lastError = filteredDelta = 0;
      forwardCommand = leftCommand = rightCommand = 80;
      motors.setSpeeds(80, 80);
      Fake::directCounts = true;
      Fake::leftCounts = 12;
      Fake::rightCounts = 8;
      const float turn = polarity * 4 * RaceSession::MM_PER_COUNT / 86;
      Fake::gyroRaw = lroundf(turn / (0.07f * 3.14159265f / 180 * 0.01f));
      std::vector<int> trace;
      for (unsigned frame = 0; frame < duration + 16; frame++)
      {
        const bool crossing = frame < duration;
        const int side = frame % 2 ? 1 : -1;
        Fake::position = 2000 + (crossing && asymmetric ? side * 500 : 0);
        for (unsigned i = 0; i < 5; i++)
        { Fake::readings[i] = crossing || i == 2 ? 1000 : 0; }
        if (crossing && asymmetric) { Fake::readings[side < 0 ? 4 : 0] = 0; }
        tick(10);
        assert(state == RUNNING && !race.mapReady && race.fault == RaceSession::NONE);
        assert(!race.noisyLearning && race.learningNoise == 0);
        if (frame + 1 == duration)
        { assert(race.mapCurveSum * polarity > 0 && race.filteredCurve * polarity > 0); }
        const int values[] = {lastError, filteredDelta, leftCommand, rightCommand};
        for (const int value : values) { trace.push_back(value); }
      }
      assert(race.distanceCounts() == start + 10 * (duration + 16));
      assert(race.mapShift == (start ? (start > 65536 ? 2 : 1) : 0));
      assert(race.turnVotes * polarity > 0);
      bool recordedTurn = false;
      for (unsigned bin = 0; bin < race.binsUsed; bin++)
      {
        if (race.curves[bin] != RaceSession::UNKNOWN_CURVE && abs(race.curves[bin]) >= 20)
        { recordedTurn = true; }
      }
      assert(recordedTurn); // Crossing handling must not blank gyro-derived bends.
      if (!asymmetric) { reference = race; referenceTrace = trace; }
      else
      {
        assert(trace == referenceTrace);
        assert(memcmp(race.curves, reference.curves, sizeof(race.curves)) == 0);
        assert(race.mapWeight == reference.mapWeight && race.mapCurveSum == reference.mapCurveSum);
        assert(race.filteredCurve == reference.filteredCurve && race.turnVotes == reference.turnVotes);
        assert(race.trackWidthMm == reference.trackWidthMm && race.speedScaleQ8 == reference.speedScaleQ8);
        cases++;
      }
    }
  }
  std::cout << "Learning crossings: " << cases
    << " centered/asymmetric pairs preserve map, geometry and fallback state.\n";
}

int main()
{
  testContinuousAlignment();
  testHiddenThrottleRelease();
  testDeliveredTurnAssistance();
  testCompletedDriveReference();
  testRepeatedSessionStartup();
  testPendingMismatchControl();
  testLapBoundaryQualification();
  testCrossingStops();
  testCrossingLearning();
  std::cout << "PASS: persistent straight alignment, rebound/hidden-throttle release, stronger settled launch, "
    "delivered-base turn assistance, straight-only derivative gain, completed-session model guard, "
    "seven consecutive button/gyro/map resets, pending-mismatch slowdown/live steering/boost inhibition "
    "and unchanged persistent stop, lap-boundary launch qualification.\n";
  return 0;
}
