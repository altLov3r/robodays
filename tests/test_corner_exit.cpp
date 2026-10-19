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

int main()
{
  testContinuousAlignment();
  testHiddenThrottleRelease();
  testDeliveredTurnAssistance();
  testCompletedDriveReference();
  testRepeatedSessionStartup();
  std::cout << "PASS: persistent straight alignment, rebound/hidden-throttle release, stronger settled launch, "
    "delivered-base turn assistance, straight-only derivative gain, completed-session model guard "
    "and seven consecutive button/gyro/map resets.\n";
  return 0;
}
