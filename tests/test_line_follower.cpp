#include <iostream>
#include <algorithm>
#include <limits>
#include <vector>
#include "../PololuLineFollower/PololuLineFollower.ino"

// The paired harness also compiles this file with historical firmware.
template<class Session>
auto setStraightHistory(Session & session, uint8_t value, int) ->
  decltype(session.straightStableMs = value, void()) { session.straightStableMs = value; }
template<class Session>
void setStraightHistory(Session &, uint8_t, long) {}

int16_t readAcceleration(int16_t (*step)(), int16_t, int16_t, bool) { return step(); }
int16_t readAcceleration(int16_t (*step)(int16_t, int16_t, bool), int16_t e, int16_t d, bool c)
{ return step(e, d, c); }
int16_t accelerationForTest(int16_t e, int16_t d, bool c)
{ return readAcceleration(&accelerationStep, e, d, c); }

void tick(uint32_t ms = 10)
{
  Fake::now += ms;
  loop();
}

void advance(uint32_t ms)
{
  for (uint32_t i = 0; i < ms; i += 10) { tick(); }
}

void resetRobot(uint32_t startTime = 0)
{
  Fake::now = startTime;
  Fake::microOffsetUs = 0;
  for (uint8_t i = 0; i < 3; i++) { Fake::buttons[i] = false; }
  Fake::usb = false;
  Fake::serialBaud = 57600;
  Fake::battery = 5000;
  Fake::imuConnected = true;
  Fake::gyroFresh = true;
  Fake::gyroRaw = 0;
  Fake::imuError = 0;
  Fake::wireTimeout = false;
  selectedMode = SAFE;
  race = RaceSession();
  gyroSamples = 0;
  gyroSum = 0;
  gyroBias = 0;
  Fake::bump = 0;
  Fake::position = 2000;
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = i == 2 ? 1000 : 0; }
  Fake::goodCalibration = true;
  Fake::scriptedCalibration = false;
  Fake::calibrationReads = 0;
  Fake::motorSignChangedWithoutStop = false;
  Fake::leftCounts = Fake::rightCounts = 10;
  Fake::directCounts = false;
  Fake::encoderErrors[0] = Fake::encoderErrors[1] = false;
  state = IDLE;
  calibrated = buttonsArmed = false;
  calibrationRequested = calibrationPausing = false;
  calibrationPhase = 0;
  calibrationLeftTravel = calibrationRightTravel = 0;
  calibrationPhaseSince = lastPreview = lastIdleUpdate = 0;
  countdownSeconds = 0;
  batteryMv = 0;
  leftStallMs = rightStallMs = 0;
  stateSince = lastControl = lastBattery = lastEncoderCheck = darkSince = 0;
  lastSensorMicros = 0;
  darkActive = false;
  runDiagnostics = false;
  leftCommand = rightCommand = lastError = filteredDelta = 0;
  forwardCommand = 0;
  throttleRemainder = 0;
  peakDrivePwm = peakThrottlePwm = 0;
  memset(limiterTicks, 0, sizeof(limiterTicks));
  lineSensors.resetCalibration();
  setup();
  advance(70);
}

void press(uint8_t button)
{
  Fake::buttons[button] = true;
  tick();
  Fake::buttons[button] = false;
  advance(70);
}

void beginAutoCalibration()
{
  const unsigned nonzeroCalls = Fake::nonzeroMotorCalls;
  press(0);
  assert(state == COUNTDOWN);
  assert(Fake::nonzeroMotorCalls == nonzeroCalls);
  advance(2000);
  assert(state == CALIBRATING);
}

void calibrateRobot()
{
  const unsigned nonzeroCalls = Fake::nonzeroMotorCalls;
  beginAutoCalibration();
  const uint32_t started = Fake::now;
  while (state == CALIBRATING && Fake::now - started <= Settings::CALIBRATION_MS)
  {
    const bool wasPausing = calibrationPausing;
    const uint32_t pauseStarted = calibrationPhaseSince;
    tick();
    if (wasPausing && Fake::now - pauseStarted < Settings::CALIBRATION_PAUSE_MS)
    {
      assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
    }
  }
  assert(state == IDLE && calibrated);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
  assert(Fake::nonzeroMotorCalls > nonzeroCalls);
  assert(calibrationPhase == 5 && !Fake::motorSignChangedWithoutStop);
  advance(100);
  assert(Fake::message.find("CENTERED") != std::string::npos);
  advance(70);
}

void startRobot()
{
  press(1);
  assert(state == COUNTDOWN);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
  advance(2000);
  assert(state == RUNNING);
  advance(400);
  assert(Fake::leftMotor > 0 && Fake::rightMotor > 0);
}

void readyRun()
{
  resetRobot();
  calibrateRobot();
  startRobot();
}

void expectStopped(const char * message)
{
  assert(state == IDLE);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
  assert(Fake::message.find(message) == 0);
  advance(500);
  assert(state == IDLE && Fake::leftMotor == 0 && Fake::rightMotor == 0);
}

void testFixedCalibrationStorage()
{
  resetRobot();
  assert(!calibrationOK());
  uint16_t * const minimum = lineSensors.calibrationOn.minimum;
  uint16_t * const maximum = lineSensors.calibrationOn.maximum;
  assert(minimum && maximum && minimum != maximum);
  Fake::scriptedCalibration = true;
  for (unsigned block = 0; block < 5; block++)
  {
    for (unsigned sample = 0; sample < 10; sample++)
    for (unsigned sensor = 0; sensor < 5; sensor++)
    {
      uint16_t value = 100 + 5 * sensor;
      if (block == 1 && sample != sensor) { value = 1500 + 2 * sensor; }
      if (block == 2) { value = sample == sensor ? 1999 : 50 + 5 * sensor; }
      if (block == 3) { value = 1400 + sample * 3 + sensor; }
      if (block == 4) { value = 20 + sample * 2 + sensor; }
      Fake::calibrationReadings[sample][sensor] = value;
    }
    lineSensors.calibrate();
    assert(Fake::calibrationReads == (block + 1) * 10);
    for (unsigned sensor = 0; sensor < 5; sensor++)
    {
      // Isolated dark/bright outliers in blocks 1/2 cannot change either extreme.
      assert(maximum[sensor] == (block < 3 ? 100 + 5 * sensor : 1400 + sensor));
      assert(minimum[sensor] == (block < 4 ? 100 + 5 * sensor : 38 + sensor));
    }
    assert(calibrationOK() == (block >= 3));
  }
  for (unsigned repeat = 0; repeat < 100; repeat++)
  {
    lineSensors.resetCalibration();
    assert(lineSensors.calibrationOn.minimum == minimum && lineSensors.calibrationOn.maximum == maximum);
    assert(!calibrationOK());
    for (unsigned sensor = 0; sensor < 5; sensor++)
    { assert(minimum[sensor] == Settings::SENSOR_TIMEOUT_US && maximum[sensor] == 0); }
    lineSensors.calibrate();
    assert(!calibrationOK()); // One narrow raw-reading band is not a valid calibration.
  }
  { RaceLineSensors separate; assert(separate.calibrationOn.minimum != minimum); }
  resetRobot();
}

// Differential-drive model: motor PWM changes motion, which changes sensor error,
// encoder counts and gyro readings. No imposed lap times or perfect line tracking.
struct SteeringSnapshot
{
  int error, delta, forward, live, mapped, votes, width, feedforward;
};

SteeringSnapshot steeringSnapshot()
{
  const int error = int(Fake::position) - 2000;
  const int mapped = race.mapReady && race.learnedCounts ?
    race.curves[(race.distanceCounts() % race.learnedCounts) / RaceSession::BIN_COUNTS] :
    RaceSession::UNKNOWN_CURVE;
  return {error, filteredDelta, forwardCommand, race.filteredCurve / 8,
    mapped, race.turnVotes, race.trackWidthMm, race.steeringFeedforward(forwardCommand, error)};
}

struct MotionMetrics
{
  bool fastReplayUsed = false;
  uint32_t lapTimes[3] = {};
  unsigned straightSamples[3] = {}, approachSamples[3] = {}, cornerSamples[3] = {};
  unsigned curveSamples[3] = {};
  unsigned brakingBeforeCorner[3] = {};
  float straightSpeed[3] = {}, straightPwm[3] = {}, straightDifferential[3] = {};
  float straightLateralMean[3] = {}, straightLateralAbs[3] = {}, straightLateralRms[3] = {};
  float straightLateralStdDev[3] = {}, straightDifferentialVariation[3] = {};
  float straightCommonVariation[3] = {};
  unsigned straightVariationSamples[3] = {};
  float approachSpeed[3] = {}, cornerSpeed[3] = {};
  float curveSpeed[3] = {}, curvePwm[3] = {};
  float peakStraightPwm[3] = {}, peakStraightSpeed[3] = {};
  float maxCommonDrop[3] = {}, maxWheelDrop[3] = {};
  SteeringSnapshot braking[3] = {}, tracking[3] = {};
  float maxCrossTrack[3] = {};
};

MotionMetrics testRoundedSquare(DriveMode mode, float straight, float radius,
  float direction, float motorGain = 2.5f, int sensorNoise = 0,
  float gyroPolarity = 1, float wheelBase = 86, float gainMismatch = 0,
  float motorResponse = 0.25f, uint32_t noiseSeed = 0x51a7u, bool strictChecks = true,
  uint16_t replayPeriod = 10, int mapShiftBins = 0)
{
  struct Point { float x, y, along, remaining; bool curve; };
  std::vector<Point> track;
  float tx = -straight / 2, ty = 0, angle = 0;
  for (int side = 0; side < 4; side++)
  {
    for (float s = 0; s < straight; s += 1)
    {
      track.push_back({tx, ty, s, straight - s, false});
      const float ds = fminf(1, straight - s);
      tx += ds * cosf(angle); ty += ds * sinf(angle);
    }
    const float arc = radius * 3.14159265f / 2;
    for (float s = 0; s < arc; s += 1)
    {
      track.push_back({tx, ty, s, arc - s, true});
      const float ds = fminf(1, arc - s), turn = direction * ds / radius;
      tx += ds * cosf(angle + turn / 2); ty += ds * sinf(angle + turn / 2);
      angle += turn;
    }
  }
  resetRobot();
  calibrateRobot();
  selectedMode = mode;
  press(1);
  advance(2000);
  assert(state == RUNNING);
  Fake::directCounts = true;
  MotionMetrics metrics;
  uint32_t noiseState = noiseSeed;
  float px = 0, py = 0, heading = 0, vl = 0, vr = 0, fl = 0, fr = 0;
  float travel = 0, lapTravel = 0, distances[3] = {};
  bool previousStraight = false;
  for (uint32_t frame = 0; frame < 20000 && state == RUNNING; frame++)
  {
    // Stock-sized robot, first-order motor response, integer encoder quantization.
    const uint16_t period = race.mapReady && mode == BEAST && !race.noisyLearning ? replayPeriod : 10;
    const float seconds = period * 0.001f;
    const float response = 1 - powf(1 - motorResponse, period / 10.0f);
    vl += (Fake::leftMotor * motorGain * (1 + gainMismatch / 2) - vl) * response;
    vr += (Fake::rightMotor * motorGain * (1 - gainMismatch / 2) - vr) * response;
    const float step = (vl + vr) * seconds / 2, turn = (vl - vr) * seconds / wheelBase;
    px += step * cosf(heading + turn / 2); py += step * sinf(heading + turn / 2);
    heading += turn;
    travel += step;
    fl += vl * seconds / RaceSession::MM_PER_COUNT;
    fr += vr * seconds / RaceSession::MM_PER_COUNT;
    Fake::leftCounts = static_cast<int16_t>(fl); fl -= Fake::leftCounts;
    Fake::rightCounts = static_cast<int16_t>(fr); fr -= Fake::rightCounts;
    Fake::gyroRaw = static_cast<int16_t>(lroundf(turn * gyroPolarity /
      (0.07f * 3.14159265f / 180 * seconds)));
    const float sx = px + 30 * cosf(heading), sy = py + 30 * sinf(heading);
    float nearest = 1e20f, centerNearest = 1e20f, error = 0;
    const Point * centerPoint = nullptr;
    for (const Point & p : track)
    {
      const float dx = p.x - sx, dy = p.y - sy, d2 = dx * dx + dy * dy;
      if (d2 < nearest)
      {
        nearest = d2;
        error = (-dx * sinf(heading) + dy * cosf(heading)) * 100;
      }
      const float cx = p.x - px, cy = p.y - py, c2 = cx * cx + cy * cy;
      if (c2 < centerNearest) { centerNearest = c2; centerPoint = &p; }
    }
    const float actualLateral = error / 100;
    noiseState = noiseState * 1664525u + 1013904223u;
    if (sensorNoise) { error += int(noiseState >> 16) % (sensorNoise * 2 + 1) - sensorNoise; }
    Fake::position = static_cast<uint16_t>(fmaxf(0, fminf(4000, 2000 + error)));
    for (int i = 0; i < 5; i++)
    {
      Fake::readings[i] = nearest < 22 * 22 && abs(int(Fake::position) - i * 1000) < 1000 ? 900 : 0;
    }
    const uint8_t previousLap = race.completed;
    const int16_t previousLeft = Fake::leftMotor, previousRight = Fake::rightMotor;
    tick(period);
    // Session/fault stops intentionally brake immediately; quantify normal driving.
    if (state == RUNNING && previousLap < 3)
    {
      if (race.mode == BEAST && race.mapReady && !race.noisyLearning)
      {
        assert(race.completed == 1 || race.completed == 2);
        metrics.fastReplayUsed = true;
      }
      const float common = (Fake::leftMotor + Fake::rightMotor) * 0.5f;
      const float oldCommon = (previousLeft + previousRight) * 0.5f;
      if (oldCommon - common > metrics.maxCommonDrop[previousLap])
      {
        metrics.maxCommonDrop[previousLap] = oldCommon - common;
        metrics.braking[previousLap] = steeringSnapshot();
      }
      metrics.maxWheelDrop[previousLap] = fmaxf(metrics.maxWheelDrop[previousLap],
        fmaxf(previousLeft - Fake::leftMotor, previousRight - Fake::rightMotor));
      if (sqrtf(nearest) > metrics.maxCrossTrack[previousLap])
      {
        metrics.maxCrossTrack[previousLap] = sqrtf(nearest);
        metrics.tracking[previousLap] = steeringSnapshot();
      }
      const float speed = (vl + vr) * 0.5f;
      const bool onStraight = !centerPoint->curve && centerPoint->along > fminf(100, straight / 4) &&
        centerPoint->remaining > fminf(100, straight / 4);
      if (onStraight)
      {
        metrics.straightSamples[previousLap]++;
        metrics.straightPwm[previousLap] += common;
        metrics.straightSpeed[previousLap] += speed;
        metrics.straightDifferential[previousLap] += abs(Fake::leftMotor - Fake::rightMotor);
        metrics.peakStraightPwm[previousLap] = fmaxf(metrics.peakStraightPwm[previousLap], common);
        metrics.peakStraightSpeed[previousLap] = fmaxf(metrics.peakStraightSpeed[previousLap], speed);
        metrics.straightLateralMean[previousLap] += actualLateral;
        metrics.straightLateralAbs[previousLap] += fabsf(actualLateral);
        metrics.straightLateralRms[previousLap] += actualLateral * actualLateral;
        if (previousStraight)
        {
          metrics.straightVariationSamples[previousLap]++;
          metrics.straightDifferentialVariation[previousLap] +=
            abs((Fake::leftMotor - Fake::rightMotor) - (previousLeft - previousRight));
          metrics.straightCommonVariation[previousLap] += fabsf(common - oldCommon);
        }
      }
      previousStraight = onStraight && race.completed == previousLap;
      if (!centerPoint->curve && centerPoint->remaining > 20 && centerPoint->remaining < 200 &&
        common < oldCommon) { metrics.brakingBeforeCorner[previousLap]++; }
      if (!centerPoint->curve && centerPoint->remaining >= 40 && centerPoint->remaining <= 80)
      {
        metrics.approachSamples[previousLap]++;
        metrics.approachSpeed[previousLap] += speed;
      }
      if (centerPoint->curve && centerPoint->along >= 20 && centerPoint->along <= 40)
      {
        metrics.cornerSamples[previousLap]++;
        metrics.cornerSpeed[previousLap] += speed;
      }
      if (centerPoint->curve)
      {
        metrics.curveSamples[previousLap]++;
        metrics.curveSpeed[previousLap] += speed;
        metrics.curvePwm[previousLap] += common;
      }
    }
    if (race.completed != previousLap)
    {
      distances[previousLap] = travel - lapTravel;
      lapTravel = travel;
      if (!previousLap && mapShiftBins)
      {
        const unsigned shift = mapShiftBins > 0 ? unsigned(mapShiftBins) : race.binsUsed + mapShiftBins;
        assert(shift < race.binsUsed);
        std::rotate(race.curves, race.curves + shift, race.curves + race.binsUsed);
      }
    }
  }
  for (int lap = 0; lap < 3; lap++)
  {
    metrics.lapTimes[lap] = race.lapTimes[lap];
    if (metrics.straightSamples[lap])
    {
      metrics.straightPwm[lap] /= metrics.straightSamples[lap];
      metrics.straightSpeed[lap] /= metrics.straightSamples[lap];
      metrics.straightDifferential[lap] /= metrics.straightSamples[lap];
      metrics.straightLateralMean[lap] /= metrics.straightSamples[lap];
      metrics.straightLateralAbs[lap] /= metrics.straightSamples[lap];
      const float squared = metrics.straightLateralRms[lap] / metrics.straightSamples[lap];
      metrics.straightLateralRms[lap] = sqrtf(squared);
      metrics.straightLateralStdDev[lap] = sqrtf(fmaxf(0,
        squared - metrics.straightLateralMean[lap] * metrics.straightLateralMean[lap]));
    }
    if (metrics.straightVariationSamples[lap])
    {
      metrics.straightDifferentialVariation[lap] /= metrics.straightVariationSamples[lap];
      metrics.straightCommonVariation[lap] /= metrics.straightVariationSamples[lap];
    }
    if (metrics.approachSamples[lap]) { metrics.approachSpeed[lap] /= metrics.approachSamples[lap]; }
    if (metrics.cornerSamples[lap]) { metrics.cornerSpeed[lap] /= metrics.cornerSamples[lap]; }
    if (metrics.curveSamples[lap])
    {
      metrics.curveSpeed[lap] /= metrics.curveSamples[lap];
      metrics.curvePwm[lap] /= metrics.curveSamples[lap];
    }
  }
  std::cout << "Square mode " << int(mode) << ", " << straight << "/" << radius
    << ", direction " << direction << ", motor gain " << motorGain << ", noise " << sensorNoise
    << ", gyro " << gyroPolarity << ", axle " << wheelBase << ", mismatch " << gainMismatch
    << ", response " << motorResponse
    << ": " << race.lapTimes[0] << ", "
    << race.lapTimes[1] << ", " << race.lapTimes[2] << " ms; "
    << "replay straight " << metrics.straightPwm[1] << "/" << metrics.peakStraightPwm[1]
    << " PWM (" << metrics.straightSpeed[1] << " mm/s), approach/corner "
    << metrics.approachSpeed[1] << "/" << metrics.cornerSpeed[1] << " mm/s, "
    << "curve mean " << metrics.curveSpeed[1] << " mm/s / " << metrics.curvePwm[1] << " PWM, "
    << "pre-corner brake samples " << metrics.brakingBeforeCorner[1] << ", "
    << "common/wheel drop " << metrics.maxCommonDrop[1] << "/" << metrics.maxWheelDrop[1]
    << ", straight differential " << metrics.straightDifferential[1]
    << ", straight lateral RMS/SD " << metrics.straightLateralRms[1] << "/"
    << metrics.straightLateralStdDev[1] << " mm, PWM diff/common variation "
    << metrics.straightDifferentialVariation[1] << "/" << metrics.straightCommonVariation[1]
    << ", at max drop e/d/f " << metrics.braking[1].error << "/"
    << metrics.braking[1].delta << "/" << metrics.braking[1].forward
    << ", cross-track " << metrics.maxCrossTrack[1] << " mm; "
    << "fast replay used " << metrics.fastReplayUsed << ", completed " << int(race.completed)
    << ", fault " << int(race.fault) << std::endl;
  for (int lap = 1; lap < 3; lap++)
  {
    if (metrics.maxCommonDrop[lap] <= Settings::DRIVE_BRAKE_STEP && metrics.maxCrossTrack[lap] < 8)
    { continue; }
    for (const SteeringSnapshot & sample : {metrics.braking[lap], metrics.tracking[lap]})
    {
      std::cout << "  lap " << lap + 1 << " brake/track snapshot: error " << sample.error
        << ", delta " << sample.delta << ", forward " << sample.forward
        << ", live/map " << sample.live << "/" << sample.mapped
        << ", votes " << sample.votes << ", width " << sample.width
        << ", feedforward " << sample.feedforward << std::endl;
    }
  }
  if (!strictChecks) { return metrics; }
  assert(race.completed == 3 && state == IDLE && race.fault == RaceSession::NONE);
  for (int lap = 0; lap < 3; lap++)
  {
    assert(metrics.straightSamples[lap] && metrics.approachSamples[lap] &&
      metrics.cornerSamples[lap] && metrics.curveSamples[lap]);
  }
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
  assert(race.lapTimes[1] < race.lapTimes[0]);
  assert(race.lapTimes[2] < race.lapTimes[0]);
  // Quantization/localization can move the estimated crossing by a few frames.
  assert(race.lapTimes[2] <= race.lapTimes[1] + 100);
  for (int lap = 1; lap < 3; lap++)
  {
    assert(fabsf(distances[lap] - distances[0]) < distances[0] * 0.03f);
    assert(metrics.maxCommonDrop[lap] <= Settings::DRIVE_BRAKE_STEP);
    if (radius <= 75)
    {
      assert(metrics.maxCrossTrack[lap] < 8);
    }
  }
  return metrics;
}

struct StraightMetrics
{
  float sensorRms = 0, sensorMean = 0, centerRms = 0, centerMean = 0;
  float sensorStdDev = 0, centerStdDev = 0;
  float differentialVariation = 0, commonVariation = 0, meanSpeed = 0;
  float maxLateral = 0, maxCommonDrop = 0, finalOffset = 0;
  float steadyMaxLateral = 0, steadyMaxDrop = 0;
  unsigned earlyUrgentBrakes = 0, steadyUrgentBrakes = 0;
  uint32_t lastOutsideMeanBand = 0, maxBrakeTime = 0;
  int maxBrakeError = 0, maxBrakeDelta = 0;
  unsigned samples = 0;
};

StraightMetrics testDynamicStraight(int noise, float mismatch = 0,
  float polarity = 1, bool periodJitter = false, bool sensorDelay = false, bool strictChecks = true,
  uint16_t replayPeriod = 10, int cornerExit = 0)
{
  readyRun();
  selectedMode = BEAST;
  race.reset(BEAST, Fake::now);
  race.mapReady = true;
  race.completed = 1;
  race.learnedCounts = 50000;
  race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
  memset(race.curves, 0, sizeof(race.curves));
  race.speedScaleQ8 = 960; // These tests use 3.75 mm/s per PWM.
  race.turnVotes = polarity > 0 ? 8 : -8;
  advance(1000);
  if (cornerExit)
  {
    forwardCommand = 400;
    leftCommand = 280 + cornerExit * 40;
    rightCommand = 280 - cornerExit * 40;
    motors.setSpeeds(leftCommand, rightCommand);
    race.filteredCurve = cornerExit * 160;
    race.exitQuietMs = 0;
    setStraightHistory(race, 0, 0);
  }
  Fake::directCounts = true;
  float heading = cornerExit * 0.04f, py = cornerExit ? -30 * sinf(heading) : 3;
  float vl = Fake::leftMotor * 3.75f, vr = Fake::rightMotor * 3.75f;
  float fl = 0, fr = 0, delayedError = -300;
  StraightMetrics metrics;
  struct Trace { uint32_t at; float center; };
  std::vector<Trace> trace;
  uint32_t noiseState = 0x9c35u, elapsed = 0;
  const uint16_t periods[] = {10, 15, 10, 20, 15, 10};
  for (unsigned frame = 0; elapsed < 5000 && state == RUNNING; frame++)
  {
    const uint16_t period = periodJitter ? periods[frame % 6] : replayPeriod;
    const float seconds = period * 0.001f;
    const float response = 1 - powf(0.75f, period / 10.0f);
    vl += (Fake::leftMotor * 3.75f * (1 + mismatch / 2) - vl) * response;
    vr += (Fake::rightMotor * 3.75f * (1 - mismatch / 2) - vr) * response;
    const float step = (vl + vr) * seconds / 2, turn = (vl - vr) * seconds / 86;
    py += step * sinf(heading + turn / 2);
    heading += turn;
    fl += vl * seconds / RaceSession::MM_PER_COUNT;
    fr += vr * seconds / RaceSession::MM_PER_COUNT;
    Fake::leftCounts = static_cast<int16_t>(fl); fl -= Fake::leftCounts;
    Fake::rightCounts = static_cast<int16_t>(fr); fr -= Fake::rightCounts;
    Fake::gyroRaw = static_cast<int16_t>(lroundf(turn * polarity /
      (0.07f * 3.14159265f / 180 * seconds)));
    const float sensorOffset = py + 30 * sinf(heading);
    const float actualError = -sensorOffset * cosf(heading) * 100;
    const float observed = sensorDelay ? delayedError : actualError;
    delayedError = actualError;
    noiseState = noiseState * 1664525u + 1013904223u;
    const int jitter = noise ? int(noiseState >> 16) % (noise * 2 + 1) - noise : 0;
    Fake::position = static_cast<uint16_t>(fmaxf(0, fminf(4000, 2000 + observed + jitter)));
    for (int i = 0; i < 5; i++)
    {
      Fake::readings[i] = fabsf(sensorOffset) < 22 &&
        abs(int(Fake::position) - i * 1000) < 1000 ? 900 : 0;
    }
    const int16_t previousLeft = Fake::leftMotor, previousRight = Fake::rightMotor;
    tick(period);
    elapsed += period;
    trace.push_back({elapsed, py});
    metrics.maxLateral = fmaxf(metrics.maxLateral, fabsf(sensorOffset));
    if (state != RUNNING) { break; }
    const float common = (Fake::leftMotor + Fake::rightMotor) * 0.5f;
    const float oldCommon = (previousLeft + previousRight) * 0.5f;
    const float drop = (oldCommon - common) * 10 / period;
    if (drop > metrics.maxCommonDrop)
    {
      metrics.maxCommonDrop = drop;
      metrics.maxBrakeTime = elapsed;
      metrics.maxBrakeError = int(Fake::position) - 2000;
      metrics.maxBrakeDelta = filteredDelta;
    }
    if (drop > Settings::DRIVE_BRAKE_STEP + 0.001f)
    {
      if (elapsed < 2000) { metrics.earlyUrgentBrakes++; }
      else { metrics.steadyUrgentBrakes++; }
    }
    if (elapsed >= 2000)
    {
      metrics.samples++;
      metrics.steadyMaxLateral = fmaxf(metrics.steadyMaxLateral, fabsf(sensorOffset));
      metrics.steadyMaxDrop = fmaxf(metrics.steadyMaxDrop, drop);
      metrics.sensorRms += sensorOffset * sensorOffset;
      metrics.sensorMean += sensorOffset;
      metrics.centerRms += py * py;
      metrics.centerMean += py;
      metrics.meanSpeed += (vl + vr) / 2;
      metrics.differentialVariation += abs((Fake::leftMotor - Fake::rightMotor) -
        (previousLeft - previousRight)) * 10.0f / period;
      metrics.commonVariation += fabsf(common - oldCommon) * 10 / period;
    }
  }
  metrics.finalOffset = py;
  if (metrics.samples)
  {
    metrics.sensorRms = sqrtf(metrics.sensorRms / metrics.samples);
    metrics.sensorMean /= metrics.samples;
    metrics.centerRms = sqrtf(metrics.centerRms / metrics.samples);
    metrics.centerMean /= metrics.samples;
    metrics.sensorStdDev = sqrtf(fmaxf(0, metrics.sensorRms * metrics.sensorRms -
      metrics.sensorMean * metrics.sensorMean));
    metrics.centerStdDev = sqrtf(fmaxf(0, metrics.centerRms * metrics.centerRms -
      metrics.centerMean * metrics.centerMean));
    metrics.meanSpeed /= metrics.samples;
    metrics.differentialVariation /= metrics.samples;
    metrics.commonVariation /= metrics.samples;
    for (const Trace & sample : trace)
    {
      if (fabsf(sample.center - metrics.centerMean) > 0.5f)
      { metrics.lastOutsideMeanBand = sample.at; }
    }
  }
  std::cout << "Dynamic straight noise " << noise << ", mismatch " << mismatch
    << ", gyro " << polarity << ", jitter/delay " << periodJitter << "/" << sensorDelay
    << ": RMS sensor/center " << metrics.sensorRms << "/" << metrics.centerRms
    << " mm, mean sensor/center " << metrics.sensorMean << "/" << metrics.centerMean
    << " mm, SD sensor/center " << metrics.sensorStdDev << "/" << metrics.centerStdDev
    << " mm, PWM diff/common variation " << metrics.differentialVariation << "/"
    << metrics.commonVariation << ", speed " << metrics.meanSpeed << " mm/s, max offset "
    << metrics.maxLateral << " mm, brake initial/steady " << metrics.maxCommonDrop
    << "/" << metrics.steadyMaxDrop << ", urgent initial/steady " << metrics.earlyUrgentBrakes
    << "/" << metrics.steadyUrgentBrakes << ", max brake at/e/d " << metrics.maxBrakeTime
    << "/" << metrics.maxBrakeError << "/" << metrics.maxBrakeDelta
    << ", last outside mean+/-0.5mm " << metrics.lastOutsideMeanBand << " ms"
    << ", state/fault " << int(state) << "/" << int(race.fault) << std::endl;
  if (!strictChecks) { return metrics; }
  assert(state == RUNNING && race.fault == RaceSession::NONE && metrics.samples > 100);
  assert(metrics.steadyMaxLateral < 12 && fabsf(metrics.finalOffset) < 6);
  assert(metrics.steadyMaxDrop <= Settings::DRIVE_BRAKE_STEP + 0.001f);
  return metrics;
}

void testStraightStability()
{
  // Physical steady error, not injected readout noise. Frozen R6 had sensor SD
  // 0.962 mm with 3% motor imbalance and 3.283 mm with imbalance/jitter/delay.
  const StraightMetrics quiet = testDynamicStraight(0);
  assert(quiet.sensorStdDev < 0.6f && quiet.meanSpeed > 1450);
  const StraightMetrics noise40 = testDynamicStraight(40);
  assert(noise40.sensorStdDev < 0.17f && noise40.meanSpeed > 1450);
  const StraightMetrics noise100 = testDynamicStraight(100);
  assert(noise100.sensorStdDev < 0.76f && noise100.meanSpeed > 1400);
  const StraightMetrics bias3 = testDynamicStraight(40, 0.03f);
  assert(bias3.sensorStdDev < 0.5f && fabsf(bias3.sensorMean) < 1.1f && bias3.meanSpeed > 1430);
  const StraightMetrics bias8 = testDynamicStraight(100, 0.08f, -1);
  assert(bias8.sensorStdDev < 0.9f && fabsf(bias8.sensorMean) < 2.4f && bias8.meanSpeed > 1325);
  const StraightMetrics delayed = testDynamicStraight(40, 0.03f, 1, false, true);
  assert(delayed.sensorStdDev < 1.2f && delayed.meanSpeed > 1350);
  const StraightMetrics jitter = testDynamicStraight(100, 0, 1, true);
  assert(jitter.sensorStdDev < 1.15f && jitter.meanSpeed > 1400);
  const StraightMetrics stress = testDynamicStraight(40, 0.08f, -1, true, true);
  assert(stress.sensorStdDev < 2.5f && fabsf(stress.sensorMean) < 2.6f && stress.meanSpeed > 1250);
}

void testNoisyStraight()
{
  readyRun();
  // Mid has spare output headroom, isolating alternating steering corrections
  // from full-speed steering headroom reservation.
  selectedMode = MID;
  race.reset(MID, Fake::now);
  race.mapReady = true;
  race.completed = 1;
  race.learnedCounts = 50000;
  race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
  memset(race.curves, 0, sizeof(race.curves));
  float sumCommand = 0, sumRequested = 0;
  for (unsigned frame = 0; frame < 400; frame++)
  {
    // A line centered within 1 mm; independent readout jitter must not ratchet
    // both motor commands down through alternating differential corrections.
    Fake::position = 2000 + (frame % 2 ? 100 : -100);
    tick();
    assert(state == RUNNING);
    if (frame >= 200)
    {
      sumCommand += (Fake::leftMotor + Fake::rightMotor) * 0.5f;
      sumRequested += baseSpeedForLine(Fake::position - 2000, filteredDelta, false);
    }
  }
  std::cout << "Noisy straight: delivered/requested common PWM " << sumCommand / 200
    << "/" << sumRequested / 200 << std::endl;
  assert(sumCommand >= sumRequested * 0.95f);
}

void testSmoothBraking()
{
  readyRun();
  selectedMode = BEAST;
  race.reset(BEAST, Fake::now);
  race.mapReady = true;
  race.completed = 1;
  race.turnVotes = 8;
  race.learnedCounts = 50000;
  race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
  memset(race.curves, 0, sizeof(race.curves));
  advance(1500);
  assert(Fake::leftMotor == 400 && Fake::rightMotor == 400);
  // A suddenly conservative map request still applies the routine brake ramp.
  // Fault/button/no-line stops have separate immediate-zero regressions below.
  memset(race.curves, 127, sizeof(race.curves));
  for (unsigned frame = 0; frame < 110; frame++)
  {
    const int16_t previous = Fake::leftMotor;
    tick();
    assert(state == RUNNING && Fake::leftMotor == Fake::rightMotor);
    assert(Fake::leftMotor <= previous);
    assert(previous - Fake::leftMotor <= Settings::DRIVE_BRAKE_STEP);
  }
  assert(Fake::leftMotor == race.plannedCruise(true));
  for (const uint32_t period : {20u, 40u})
  {
    const int16_t before = Fake::leftMotor;
    memset(race.curves, 0, sizeof(race.curves));
    tick(period);
    assert(Fake::leftMotor - before == static_cast<int>(6 * period / Settings::CONTROL_MS));
    advance(1500);
    assert(Fake::leftMotor == 400 && Fake::rightMotor == 400);
    memset(race.curves, 127, sizeof(race.curves));
    tick(period);
    assert(400 - Fake::leftMotor == static_cast<int>(Settings::DRIVE_BRAKE_STEP * period / Settings::CONTROL_MS));
    advance(1500);
  }
}

void testAccelerationConfidence()
{
  const int16_t expectedSteps[] = {6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6};
  for (unsigned scenario = 0; scenario < 18; scenario++)
  {
    readyRun();
    selectedMode = scenario == 1 ? MID : (scenario == 2 ? SAFE : BEAST);
    race.reset(selectedMode, Fake::now);
    if (scenario != 0)
    {
      race.mapReady = true;
      race.completed = 1;
      race.learnedCounts = 50000;
      race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
      memset(race.curves, 0, sizeof(race.curves));
      race.turnVotes = 8;
      race.exitQuietMs = 40;
    }
    if (scenario == 4) { Fake::position = 2150; lastError = 150; }
    if (scenario == 5) { filteredDelta = 80; }
    if (scenario == 6) { race.filteredCurve = 64; }
    if (scenario == 7) { race.noisyLearning = true; }
    if (scenario == 8) { race.curves[0] = 40; }
    if (scenario == 9) { race.curves[1] = 40; }
    if (scenario == 10) { race.turnVotes = 0; }
    if (scenario == 11) { Fake::position = 2080; lastError = 80; }
    if (scenario == 12) { filteredDelta = 60; }
    if (scenario >= 13) { race.completed = 2; }
    if (scenario == 14) { race.noisyLearning = true; }
    if (scenario == 15) { race.curves[1] = 40; }
    if (scenario == 16) { Fake::position = 2080; lastError = 80; }
    if (scenario == 17) { race.turnVotes = 0; }
    forwardCommand = leftCommand = rightCommand = scenario == 0 ? 60 : 100;
    motors.setSpeeds(leftCommand, rightCommand);
    const int16_t before = forwardCommand;
    tick();
    assert(state == RUNNING);
    assert(forwardCommand - before == expectedSteps[scenario]);
  }
}

void testNoisyLearningFallback()
{
  readyRun();
  selectedMode = BEAST;
  race.reset(BEAST, Fake::now);
  lastError = filteredDelta = 0;
  Fake::position = 2300;
  tick(10);
  assert(state == RUNNING && filteredDelta == 150 && !race.noisyLearning && race.learningNoise == 1);
  Fake::position = 2000;
  advance(50);
  assert(state == RUNNING && !race.noisyLearning && race.learningNoise == 0);
  for (unsigned frame = 0; frame < 12; frame++)
  {
    Fake::position = frame % 2 ? 2500 : 1500;
    tick(10);
    assert(state == RUNNING);
  }
  assert(race.noisyLearning);
  Fake::position = 2000;
  advance(100);
  assert(state == RUNNING && race.noisyLearning);
  race.mapReady = true;
  race.completed = 1;
  race.learnedCounts = 50000;
  race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
  memset(race.curves, 0, sizeof(race.curves));
  race.turnVotes = 8;
  lastError = filteredDelta = 0;
  forwardCommand = leftCommand = rightCommand = 100;
  motors.setSpeeds(100, 100);
  tick(10);
  assert(state == RUNNING && forwardCommand == 106);
  assert(race.profile().cruise == 180 && race.profile().limit == 220);
  assert(race.steeringLimit() == 110);
}

void readyFastReplay(bool attack = false)
{
  readyRun();
  selectedMode = BEAST;
  race.reset(BEAST, Fake::now);
  race.mapReady = true;
  race.completed = attack ? 2 : 1;
  race.learnedCounts = 50000;
  race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
  memset(race.curves, 0, sizeof(race.curves));
  race.turnVotes = 8;
  forwardCommand = leftCommand = rightCommand = 100;
  race.exitQuietMs = 40; // Helper represents an already settled replay straight.
  setStraightHistory(race, 40, 0);
  throttleRemainder = 0;
  peakDrivePwm = peakThrottlePwm = 0;
  memset(limiterTicks, 0, sizeof(limiterTicks));
  motors.setSpeeds(100, 100);
  lastControl = lastEncoderCheck = Fake::now;
  lastSensorMicros = micros();
}

void testTightLaunchGate()
{
  for (const int limit : {0, 32, 96})
  for (const uint8_t quiet : {20, 39, 40, 89})
  for (const int yaw : {-96,-32,-31,0,31,32,96})
  for (const int error : {-2000,-180,-179,-100,0,100,179,180,2000})
  for (const int delta : {-8000,-80,-79,-40,0,40,79,80,8000})
  {
    readyFastReplay();
    race.launchCurveLimit = limit;
    race.exitQuietMs = quiet;
    race.filteredCurve = yaw;
    const bool boost = limit == 32 && quiet >= 40 && abs(yaw) < 32 &&
      abs(error) < 180 && abs(delta) < 80 && abs(error + 3 * delta) < 240;
    qualifyStraight(error, delta, false, 0);
    assert(accelerationForTest(error, delta, false) == (boost ? 24 : 6));
    qualifyStraight(-error, -delta, false, 0);
    assert(accelerationForTest(-error, -delta, false) == (boost ? 24 : 6));
    qualifyStraight(error, delta, true, 0);
    assert(accelerationForTest(error, delta, true) == 6);
  }
  for (const int error : {0, 100})
  {
    readyFastReplay();
    Fake::position = 2000 + error;
    lastError = error;
    forwardCommand = leftCommand = rightCommand = 200;
    motors.setSpeeds(200, 200);
    unsigned elapsed = 0;
    const unsigned writes = Fake::displayWrites;
    while (forwardCommand < 300 && elapsed < 200) { tick(5); elapsed += 5; }
    assert(state == RUNNING && elapsed == 45u);
    assert(Fake::displayWrites == writes);
  }
}

void testStraightRecoveryGate()
{
  for (const bool lapThree : {false, true})
  for (const uint16_t period : {5,7,8,10,20,40})
  {
    readyFastReplay(lapThree);
    Fake::position = 2110;
    lastError = 110;
    unsigned elapsed = 0;
    while (elapsed < 100)
    {
      unsigned dt = period;
      if (100 - elapsed < dt) { dt = 100 - elapsed; }
      if (100 - elapsed - dt > 0 && 100 - elapsed - dt < 5) { dt = 100 - elapsed; }
      tick(dt); elapsed += dt;
      assert(state == RUNNING && forwardCommand == int(100 + 24 * elapsed / 10));
      assert(throttleRemainder == int(24 * elapsed % 10));
    }
  }
  for (unsigned scenario = 0; scenario < 7; scenario++)
  {
    readyFastReplay();
    Fake::position = 2100; lastError = 100;
    if (scenario == 0) { race.exitQuietMs = 34; }
    if (scenario == 1) { race.curves[0] = race.curves[race.binsUsed - 1] = 30; }
    if (scenario == 2) { race.curves[1] = 30; }
    if (scenario == 3) { race.turnVotes = 3; }
    if (scenario == 4) { Fake::gyroRaw = 600; }
    if (scenario == 5) { race.noisyLearning = true; }
    if (scenario == 6) { selectedMode = race.mode = MID; }
    tick(scenario >= 5 ? 10 : 5);
    assert(state == RUNNING && forwardCommand == (scenario >= 5 ? 106 : 103));
  }
}

void testCurveRecovery()
{
  for (const bool lapThree : {false, true})
  for (const uint16_t period : {5, 7, 8, 10, 20, 40})
  {
    readyFastReplay(lapThree);
    memset(race.curves, 30, sizeof(race.curves));
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 10;
    forwardCommand = leftCommand = rightCommand = 140;
    motors.setSpeeds(140, 140);
    unsigned elapsed = 0;
    while (elapsed < 100)
    {
      unsigned dt = period;
      if (100 - elapsed < dt) { dt = 100 - elapsed; }
      if (100 - elapsed - dt > 0 && 100 - elapsed - dt < 5) { dt = 100 - elapsed; }
      tick(dt);
      elapsed += dt;
      assert(state == RUNNING && race.launchCurveLimit == 0);
      assert(forwardCommand == int(140 + 6 * elapsed / 10));
      assert(forwardCommand <= race.plannedCruise(true));
    }
    advance(100);
    assert(forwardCommand == 234 && race.plannedCruise(true) == 234);
  }
}

void testLimiterShares()
{
  readyFastReplay();
  race.completed = 3;
  showResults();
  assert(Fake::displayRows[2].find("A/L/S 0/0/0") == 0);
  const uint16_t counts[] = {10000, 8000, 12000, 6000, 0, 0};
  memcpy(limiterTicks, counts, sizeof(counts));
  showResults();
  assert(Fake::displayRows[2].find("A/L/S 33/22/16") == 0);
  assert(Fake::displayRows[7].find("Limit: A") == 0);
}

void testReplayConsistency()
{
  for (const bool lapThree : {false, true})
  {
    readyFastReplay(lapThree);
    memset(race.curves, 30, sizeof(race.curves));
    forwardCommand = leftCommand = rightCommand = 234;
    motors.setSpeeds(234, 234);
    tick(5);
    assert(state == RUNNING && forwardCommand == 234);
    assert(race.plannedCruise(true) == 234 && race.profile().limit == 400);
    race.completed = 3;
    showResults();
    assert(Fake::displayRows[7].find('+') == std::string::npos);
  }
}

void testLimiterTelemetry()
{
  for (unsigned reason = 0; reason < 6; reason++)
  {
    readyFastReplay();
    forwardCommand = leftCommand = rightCommand = 400;
    if (reason == MAP_LIMIT) { memset(race.curves, 127, sizeof(race.curves)); }
    if (reason == YAW_LIMIT)
    {
      race.filteredCurve = 240;
      race.curves[2] = 9; // Accepted neighboring bend; isolate live-yaw limiting.
    }
    if (reason == LINE_LIMIT)
    {
      Fake::position = 2500;
      lastError = 500;
      forwardCommand = leftCommand = rightCommand = baseSpeedForLine(500, 0, false);
    }
    if (reason == ACCEL_LIMIT) { forwardCommand = leftCommand = rightCommand = 100; }
    if (reason == STEER_LIMIT) { Fake::position = 2080; lastError = 80; }
    tick(5);
    assert(state == RUNNING && speedLimiter == reason);
    for (unsigned bucket = 0; bucket < 6; bucket++)
    { assert(limiterTicks[bucket] == (bucket == reason ? 1 : 0)); }
    race.completed = 3;
    showResults();
    assert(Fake::displayRows[7].find(std::string("Limit: ") + "MLASFY"[reason]) == 0);
    assert(Fake::displayRows[1].find(std::to_string(race.speedScaleQ8) + "/" +
      std::to_string(race.trackWidthMm)) == 0);
  }
  for (const int votes : {-4, -3, 0, 3, 4})
  {
    readyFastReplay();
    race.turnVotes = votes;
    limiterTicks[ACCEL_LIMIT] = 5;
    race.completed = 3;
    showResults();
    assert(Fake::displayRows[7].find(abs(votes) < 4 ? "Limit: G" : "Limit: A") == 0);
  }

  readyFastReplay();
  forwardCommand = leftCommand = rightCommand = 400;
  Fake::position = 2800;
  lastError = 800;
  tick(5);
  assert(forwardCommand < 200 && peakThrottlePwm == forwardCommand);
  Fake::position = 2000;
  lastError = filteredDelta = 0;
  advance(500);
  assert(state == RUNNING && !race.noisyLearning);
  assert(Fake::leftMotor == 400 && Fake::rightMotor == 400);
}

void checkCornerEntryYaw(DriveMode mode, bool noisy, int polarity,
  int direction, int yawDirection, int offset)
{
  readyFastReplay();
  race.mode = selectedMode = mode;
  race.noisyLearning = noisy;
  race.turnVotes = polarity * 8;
  forwardCommand = leftCommand = rightCommand = 70;
  const int error = direction * offset;
  Fake::position = 2000 + error;
  lastError = error;
  Fake::gyroRaw = polarity * yawDirection * 600;
  const bool fast = mode == BEAST && !noisy;
  tick(fast ? 5 : 10);
  assert(state == RUNNING);
  const int16_t proportional = direction * (offset > 24 ? offset - 24 : 0) * Settings::KP_256 / 256;
  const int16_t measured = polarity * Fake::gyroRaw *
    (0.07f * 3.14159265f / 180000.0f) * (128000UL * 86 / 1024);
  const bool half = offset >= 400 || (fast && offset >= 160 && direction == yawDirection);
  const int16_t yawError = -measured;
  const int16_t expectedDamping = half ? yawError / 2 : yawError;
  const int16_t room = std::min(forwardCommand, int16_t(race.profile().limit - forwardCommand));
  const int16_t limit = std::min(room, race.steeringLimit());
  const int16_t expected = std::max(int16_t(-limit), std::min(limit, int16_t(proportional + expectedDamping)));
  assert(leftCommand - rightCommand == 2 * expected);
  assert(leftCommand + rightCommand == 2 * forwardCommand);
  assert(Fake::leftMotor >= 0 && Fake::leftMotor <= race.profile().limit);
  assert(Fake::rightMotor >= 0 && Fake::rightMotor <= race.profile().limit);
}

void testCornerEntryYawPriority()
{
  for (const int polarity : {-1, 1})
  {
    for (const int direction : {-1, 1})
    {
      for (const int yawDirection : {-1, 1})
      {
        for (const int offset : {0, 80, 159, 160, 200, 399, 400, 500})
        {
          for (const auto mode : {SAFE, MID, BEAST})
          { checkCornerEntryYaw(mode, false, polarity, direction, yawDirection, offset); }
          checkCornerEntryYaw(BEAST, true, polarity, direction, yawDirection, offset);
        }
      }
    }
  }
}

void testR24YawDamping()
{
  for (const int polarity : {-1, 1})
  for (const int direction : {-1, 1})
  for (const int error : {-300, -120, -40, 40, 120, 300})
  for (const int delta : {-40, 0, 40})
  {
    readyFastReplay();
    race.turnVotes = polarity * 8;
    race.filteredCurve = direction * 240;
    memset(race.curves, direction * 30, sizeof(race.curves));
    forwardCommand = leftCommand = rightCommand = 200;
    motors.setSpeeds(200, 200);
    Fake::position = 2000 + error;
    lastError = error;
    filteredDelta = delta * 7 / 5;
    const int steerError = error > 0 ? error - 24 : error + 24;
    const int steerDelta = delta > 12 ? delta - 12 : (delta < -12 ? delta + 12 : 0);
    const int feedback = (steerError * 32 + steerDelta * 96) / 256;
    const int wheelDirection = polarity * direction;
    const int opposing = wheelDirection > 0 ? -error : error;
    const int feedforward = opposing >= 80 ? 0 :
      (opposing > 0 ? wheelDirection * 43 * (80 - opposing) / 80 : wheelDirection * 43);
    const int desired = feedback + feedforward;
    const float conversion = (0.07f * 3.14159265f / 180000.0f) * (128000UL * 86 / 1024);
    Fake::gyroRaw = polarity * lroundf(desired / conversion);
    tick(5);
    assert(state == RUNNING);
    // R24 damps around mapped yaw; it does not add half the optical PD again.
    const int expected = desired + (feedforward - desired) / 2;
    assert(abs((leftCommand - rightCommand) / 2 - expected) <= 1);
  }
  for (const int error : {-200, 200})
  {
    readyFastReplay();
    forwardCommand = leftCommand = rightCommand = 200;
    Fake::position = 2000 + error;
    lastError = error;
    tick(5);
    assert((leftCommand - rightCommand) / 2 == (error < 0 ? -22 : 22));
  }
}

void testR24OpticalEnvelope()
{
  struct Case { int error, delta, expected; };
  const Case cases[] = {{200,100,234}, {200,-100,234}, {100,100,234},
    {300,100,234}, {300,-100,234}, {600,200,139}, {1200,0,65}, {0,100,234},
    {450,100,234}, {400,200,206}, {500,200,173}, {600,-200,139}, {600,0,233}};
  for (const int direction : {-1, 1})
  for (const auto & item : cases)
  {
    readyFastReplay();
    memset(race.curves, 30, sizeof(race.curves));
    forwardCommand = leftCommand = rightCommand = item.expected;
    motors.setSpeeds(item.expected, item.expected);
    Fake::position = 2000 + direction * item.error;
    lastError = direction * item.error;
    filteredDelta = direction * item.delta * 7 / 5; // Five ms filter produces the specified delta.
    const unsigned writes = Fake::displayWrites;
    tick(5);
    assert(state == RUNNING && filteredDelta == direction * item.delta);
    assert(forwardCommand == item.expected);
    assert((leftCommand + rightCommand) / 2 == item.expected);
    assert(Fake::displayWrites == writes);
  }
  for (const int demand : {0, 1, 100, 300, 999, 1000, 8000, 18000})
  for (int base = 65; base <= 400; base++)
  {
    const int output = trackingSpeed(base, demand);
    assert(output >= 65 && output <= base);
    if (demand >= 1000) { assert(output == 65); }
  }
  // Large derivatives still bypass routine braking using the R24 optical cap.
  for (const int direction : {-1, 1})
  {
    readyFastReplay();
    forwardCommand = leftCommand = rightCommand = 400;
    motors.setSpeeds(400, 400);
    Fake::position = 2000 + direction * 500;
    lastError = direction * 500;
    filteredDelta = direction * 350; // 250 after the five-ms filter.
    tick(5);
    assert(state == RUNNING && forwardCommand == 139 && throttleRemainder == 0);
    assert(leftCommand >= 0 && rightCommand >= 0 && leftCommand + rightCommand == 278);
  }
  for (unsigned scenario = 0; scenario < 4; scenario++)
  {
    readyFastReplay();
    if (scenario == 0) { race.mapReady = false; }
    if (scenario == 1) { race.mode = SAFE; }
    if (scenario == 2) { race.mode = MID; }
    if (scenario == 3) { race.noisyLearning = true; }
    const int cruise = race.profile().cruise;
    assert(baseSpeedForLine(200, 100, false) == trackingSpeed(cruise, 180));
  }
}

void testFailureReadings()
{
  readyFastReplay();
  lastError = -900;
  filteredDelta = -180;
  for (auto & reading : Fake::readings) { reading = 0; }
  tick(5);
  expectStopped("No line");
  assert(runDiagnostics && Fake::displayRows[2].find("-900/-180") == 0);
  const unsigned writes = Fake::displayWrites;
  advance(1000);
  assert(Fake::displayWrites == writes && Fake::leftMotor == 0 && Fake::rightMotor == 0);
  press(2);
  assert(!runDiagnostics && state == IDLE);
}

void testFullThrottleSteering()
{
  for (const bool attack : {false, true})
  {
    for (const int error : {-100, -80, -40, 40, 80, 100})
    {
      readyFastReplay(attack);
      forwardCommand = leftCommand = rightCommand = 400;
      Fake::position = 2000 + error;
      lastError = error; // No artificial derivative kick.
      tick(5);
      const int turn = (abs(error) - Settings::STEERING_DEADBAND) * Settings::KP_256 / 256;
      assert(state == RUNNING && forwardCommand == 400);
      assert(error > 0 ? Fake::leftMotor == 400 : Fake::rightMotor == 400);
      assert(error > 0 ? Fake::rightMotor == 400 - 2 * turn : Fake::leftMotor == 400 - 2 * turn);
      assert(peakDrivePwm == 400 - turn);
      // Countersteering at saturation must work on the very next tick.
      Fake::position = 2000 - error;
      lastError = -error;
      tick(5);
      assert(forwardCommand == 400);
      assert(error > 0 ? Fake::rightMotor == 400 : Fake::leftMotor == 400);
      Fake::position = 2000;
      lastError = 0;
      tick(5);
      assert(Fake::leftMotor == 400 && Fake::rightMotor == 400);
      assert(forwardCommand == 400 && peakDrivePwm == 400);
      Fake::buttons[1] = true;
      tick(1);
      expectStopped("Stopped");
      assert(peakDrivePwm == 400); // Results survive the stop.
    }
  }
}

void testExitLaunch()
{
  for (const bool lapThree : {false, true})
  for (const int direction : {-1, 1})
  {
    readyFastReplay(lapThree);
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 10;
    Fake::gyroRaw = direction * 600;
    const unsigned writes = Fake::displayWrites;
    tick(5);
    // Fresh turning revokes the full launch before the curvature filter catches up.
    assert(state == RUNNING && abs(race.filteredCurve) < 64);
    assert(race.exitQuietMs == 0 && race.launchCurveLimit == 0 && forwardCommand == 103);
    assert(Fake::displayWrites == writes);
  }
  for (const uint8_t quietBefore : {0, 10, 14, 15, 19, 20})
  for (const bool moving : {false, true})
  {
    readyFastReplay();
    race.exitQuietMs = quietBefore;
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = moving ? 10 : 0;
    tick(5);
    const bool qualified = moving && quietBefore >= 15;
    assert(state == RUNNING && race.launchCurveLimit == (qualified ? 32 : 0));
    assert(forwardCommand == 103);
  }
  for (unsigned variant = 0; variant < 4; variant++)
  {
    const bool mature = variant >= 2;
    const bool lapThree = variant % 2;
    for (const int direction : {-1, 1})
    {
      for (unsigned layout = 0; layout < 7; layout++)
      {
        for (const int yaw : {36, 37, 73, 74, 110, 111})
        {
          readyFastReplay(lapThree);
          race.turnVotes = direction * 8;
          const unsigned previous = race.binsUsed - 1;
          if (layout != 6) { race.curves[previous] = direction * 30; }
          if (layout != 1 && layout != 6) { race.curves[0] = direction * 30; }
          if (layout == 2 || layout == 5) { race.curves[1] = direction * 30; }
          if (layout == 3) { race.curves[previous] = RaceSession::UNKNOWN_CURVE; }
          if (layout == 4) { race.curves[previous] = -direction * 30; }
          if (layout == 5) { race.curves[previous] = 0; }
          race.filteredCurve = direction * yaw;
          race.exitQuietMs = mature ? 40 : 0;
          const unsigned writes = Fake::displayWrites;
          tick(5);
          const unsigned limit = mature && (layout == 1 || layout == 6) ? 32 : 0;
          const bool strong = unsigned(yaw - yaw / 7) < limit;
          assert(state == RUNNING && race.launchCurveLimit == limit);
          assert(forwardCommand == (strong ? 112 : 103));
          assert(throttleRemainder == 0);
          assert(Fake::displayWrites == writes);
          assert(leftCommand >= 0 && leftCommand <= 400 && rightCommand >= 0 && rightCommand <= 400);
        }
      }
    }
  }
  for (unsigned scenario = 0; scenario < 8; scenario++)
  {
    readyFastReplay();
    race.curves[0] = race.curves[race.binsUsed - 1] = 30;
    race.filteredCurve = 110;
    race.exitQuietMs = 40;
    if (scenario < 4)
    {
      const int error = scenario < 2 ? (scenario ? 80 : 79) : (scenario == 2 ? -79 : -80);
      Fake::position = 2000 + error;
      lastError = error;
    }
    if (scenario == 4 || scenario == 5) { filteredDelta = scenario == 4 ? 40 : 41; }
    if (scenario >= 6) { race.turnVotes = scenario == 6 ? 3 : -3; }
    tick(5);
    assert(state == RUNNING);
    assert(forwardCommand == 103 && race.launchCurveLimit == 0);
  }
  readyFastReplay();
  race.launchCurveLimit = 96;
  forwardCommand = leftCommand = rightCommand = 40;
  for (unsigned sensor = 0; sensor < 5; sensor++) { Fake::readings[sensor] = 1000; }
  tick(5);
  assert(state == RUNNING && forwardCommand == 43); // Crossing cannot use a stale exit permission.
}

void testCornerExitLineConfirmation()
{
  struct Sample { int16_t error, delta; bool clear; };
  const Sample samples[] = {{0, 0, true}, {79, 0, true}, {80, 0, false},
    {0, 29, true}, {0, 30, false}, {160, 0, false}, {0, 100, false}};
  for (const bool lapThree : {false, true})
  for (const int turn : {-1, 1})
  for (const int sign : {-1, 1})
  for (const auto & sample : samples)
  {
    readyFastReplay(lapThree);
    race.curves[0] = race.curves[race.binsUsed - 1] = turn * 30;
    race.exitQuietMs = 20;
    Fake::directCounts = true;
    Fake::leftCounts = Fake::rightCounts = 10;
    Fake::position = 2000 + sign * sample.error;
    lastError = sign * sample.error;
    filteredDelta = sign * sample.delta * 7 / 5;
    forwardCommand = leftCommand = rightCommand = 260;
    motors.setSpeeds(260, 260);
    const unsigned writes = Fake::displayWrites;
    tick(5);
    assert(state == RUNNING && race.exitQuietMs >= 20);
    assert(filteredDelta == sign * sample.delta);
    assert(forwardCommand == (sample.clear ? 263 : 259));
    assert(race.launchCurveLimit == 0);
    assert((leftCommand + rightCommand) / 2 == forwardCommand);
    assert(Fake::displayWrites == writes);
    if (!sample.clear)
    {
      // Once the actual line sample settles, release promptly without a slow-lap latch.
      Fake::position = 2000;
      lastError = filteredDelta = 0;
      tick(5);
      assert(state == RUNNING && forwardCommand == 262 && race.launchCurveLimit == 0);
    }
  }
}

void testFastControl()
{
  readyFastReplay();
  const unsigned calls = Fake::motorCalls, writes = Fake::displayWrites;
  tick(4);
  assert(Fake::motorCalls == calls && forwardCommand == 100);
  tick(1);
  assert(Fake::motorCalls == calls + 1 && forwardCommand == 112);
  assert(throttleRemainder == 0);
  tick(5);
  assert(forwardCommand == 124 && throttleRemainder == 0);
  assert(Fake::displayWrites == writes);

  readyFastReplay(true);
  forwardCommand = leftCommand = rightCommand = 400;
  for (uint16_t position = 0; position <= 4000; position += 100)
  {
    Fake::position = position;
    tick(5);
    assert(state == RUNNING);
    assert(Fake::leftMotor >= 0 && Fake::leftMotor <= 400);
    assert(Fake::rightMotor >= 0 && Fake::rightMotor <= 400);
  }

  for (unsigned scenario = 0; scenario < 4; scenario++)
  {
    readyFastReplay();
    if (scenario == 0) { race.mode = SAFE; }
    if (scenario == 1) { race.mode = MID; }
    if (scenario == 2) { race.noisyLearning = true; }
    if (scenario == 3) { race.reset(BEAST, Fake::now); forwardCommand = 60; }
    const unsigned before = Fake::motorCalls;
    tick(5);
    assert(Fake::motorCalls == before);
    tick(5);
    assert(Fake::motorCalls == before + 1);
  }

  // Identical elapsed time must give identical ramp progress despite short ticks.
  for (const bool attack : {false, true})
  {
    for (const uint16_t period : {5, 7, 8, 10, 20, 40})
    {
      readyFastReplay(attack);
      uint16_t elapsed = 0;
      while (elapsed < 200)
      {
        uint16_t dt = period;
        if (200 - elapsed < dt) { dt = 200 - elapsed; }
        // Do not finish with a sub-control-period partial tick.
        if (200 - elapsed - dt > 0 && 200 - elapsed - dt < 5) { dt = 200 - elapsed; }
        tick(dt);
        elapsed += dt;
        assert(state == RUNNING);
        const int expected = 100 + 24 * elapsed / 10;
        assert(forwardCommand == (expected > 400 ? 400 : expected));
      }
    }
  }
  for (const uint16_t period : {5, 7, 8, 10, 20, 40})
  {
    readyFastReplay();
    forwardCommand = leftCommand = rightCommand = 400;
    memset(race.curves, 127, sizeof(race.curves));
    uint16_t elapsed = 0;
    while (elapsed < 100)
    {
      uint16_t dt = period;
      if (100 - elapsed < dt) { dt = 100 - elapsed; }
      if (100 - elapsed - dt > 0 && 100 - elapsed - dt < 5) { dt = 100 - elapsed; }
      tick(dt);
      elapsed += dt;
      assert(state == RUNNING && forwardCommand == 400 - 3 * elapsed / 10);
    }
  }

  readyFastReplay();
  tick(5);
  assert(forwardCommand == 112 && throttleRemainder == 0);
  forwardCommand = 200;
  memset(race.curves, 127, sizeof(race.curves));
  tick(5);
  assert(forwardCommand == 199 && throttleRemainder == -5);
  tick(5);
  assert(forwardCommand == 197 && throttleRemainder == 0);
  memset(race.curves, 0, sizeof(race.curves));
  tick(5);
  assert(forwardCommand == 200 && throttleRemainder == 0);
  forwardCommand = 398;
  tick(5);
  assert(forwardCommand == 400 && throttleRemainder == 0);
  memset(race.curves, 127, sizeof(race.curves));
  tick(5);
  assert(forwardCommand == 399 && throttleRemainder == -5);

  readyFastReplay(true);
  forwardCommand = leftCommand = rightCommand = 400;
  Fake::position = 2710;
  lastError = 710;
  tick(5);
  assert(state == RUNNING && forwardCommand < 300);
  assert(throttleRemainder == 0);
  Fake::buttons[0] = true;
  tick(1);
  expectStopped("Stopped");
  assert(throttleRemainder == 0);

  for (const int sign : {-1, 1})
  {
    readyFastReplay();
    for (int sample = 1; sample <= 4; sample++)
    {
      Fake::position = 2000 + sign * sample * 50;
      tick(5);
    }
    assert(abs(filteredDelta - sign * 75) <= 5);
  }

  for (const int sign : {-1, 1})
  {
    readyFastReplay();
    Fake::microOffsetUs = 1000;
    Fake::position = 2000 + sign * 60;
    tick(5);
    assert(filteredDelta == sign * 28); // 60 position units in 6 ms.
    Fake::microOffsetUs = 0;
    Fake::position = 2000 + sign * 100;
    tick(5);
    assert(filteredDelta == sign * 48); // 40 position units in 4 ms.

    readyFastReplay();
    Fake::microOffsetUs = uint32_t(0) - 4000;
    Fake::position = sign > 0 ? 4000 : 0;
    lastError = -sign * 2000;
    tick(5);
    assert(filteredDelta == sign * 2285); // Extreme 1 ms slope saturates safely.
    assert(Fake::leftMotor >= 0 && Fake::leftMotor <= 400);
    assert(Fake::rightMotor >= 0 && Fake::rightMotor <= 400);
  }
  readyFastReplay();
  Fake::microOffsetUs = (65500u - static_cast<uint16_t>(micros())) & 0xffffu;
  lastSensorMicros = micros();
  gyroLastMicros = micros();
  Fake::position = 2050;
  tick(5);
  assert(filteredDelta == 28); // Acquisition clock crosses its 16-bit wrap.

  readyFastReplay();
  Fake::microOffsetUs = std::numeric_limits<uint32_t>::max() - micros() - 1000;
  lastSensorMicros = micros();
  gyroLastMicros = micros();
  Fake::position = 2050;
  tick(5);
  assert(filteredDelta == 28); // Full micros() rollover also remains valid.

  readyFastReplay();
  Fake::microOffsetUs = uint32_t(0) - 5000;
  Fake::position = 2050;
  tick(5);
  assert(filteredDelta == 28); // Zero microsecond delta uses the valid loop interval.

  int16_t curvature[2] = {};
  float speed[2] = {};
  for (unsigned index = 0; index < 2; index++)
  {
    RaceSession filtered;
    filtered.reset(BEAST, 0);
    filtered.mapReady = true;
    filtered.completed = 1;
    filtered.learnedCounts = 50000;
    filtered.binsUsed = 391;
    memset(filtered.curves, 0, sizeof(filtered.curves));
    const uint16_t dt = index ? 10 : 5;
    for (uint16_t now = dt; now <= 40; now += dt)
    {
      const int16_t counts = dt * 2;
      const float turn = counts * RaceSession::MM_PER_COUNT * 320 / 48000;
      filtered.update(counts, counts, turn, dt, 100, now);
      assert(filtered.fault == RaceSession::NONE);
    }
    curvature[index] = filtered.filteredCurve;
    speed[index] = filtered.speedMmS;
  }
  assert(abs(curvature[0] - curvature[1]) <= 8);
  assert(fabsf(speed[0] - speed[1]) <= 15);

  for (const int sign : {-1, 1})
  {
    int16_t reference = 0;
    for (const uint16_t timestampOffset : {0, 1000, 2000})
    {
      readyFastReplay();
      Fake::gyroRaw = sign * 600;
      gyroLastMicros = micros() - timestampOffset;
      tick(5);
      const int16_t differential = leftCommand - rightCommand;
      if (!timestampOffset) { reference = differential; }
      assert(reference * sign < 0 && differential == reference);
    }
  }

  readyFastReplay();
  tick(5);
  tick(7);
  race.completed = 3;
  showResults();
  assert(Fake::displayRows[6].find("PWM " + std::to_string(peakDrivePwm) + "/" + std::to_string(peakThrottlePwm)) == 0);
  race.noisyLearning = true;
  showResults();
  assert(Fake::displayRows[7].find("MID FALLBACK") == 0);

  readyRun();
  selectedMode = BEAST;
  race.reset(BEAST, Fake::now);
  race.distanceTwice = (4096 - 10) * 2;
  race.binsUsed = 32;
  memset(race.curves, 0, sizeof(race.curves));
  race.x = -1;
  race.leftStartZone = race.returnArmed = true;
  Fake::directCounts = true;
  Fake::leftCounts = Fake::rightCounts = 10;
  const unsigned learningWrites = Fake::displayWrites;
  tick(10);
  assert(state == RUNNING && race.mapReady && race.completed == 1);
  const unsigned learningCalls = Fake::motorCalls;
  tick(5);
  assert(Fake::motorCalls == learningCalls + 1);
  assert(Fake::displayWrites == learningWrites);

  readyFastReplay(true);
  race.distanceTwice = (race.learnedCounts - 10) * 2;
  race.x = -1;
  race.leftStartZone = race.returnArmed = true;
  Fake::directCounts = true;
  Fake::leftCounts = Fake::rightCounts = 10;
  forwardCommand = leftCommand = rightCommand = 400;
  motors.setSpeeds(400, 400);
  tick(5);
  assert(state == IDLE && race.completed == 3 && !race.running);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0 && throttleRemainder == 0);
  assert(Fake::displayRows[0].find("3 LAPS EST. DONE") == 0);
  const unsigned stoppedCalls = Fake::nonzeroMotorCalls;
  advance(10000);
  assert(state == IDLE && race.completed == 3 && Fake::nonzeroMotorCalls == stoppedCalls);
}

int main(int argc, char ** argv)
{
  const bool focused = argc > 1 && strcmp(argv[1], "--focused") == 0;
  testFixedCalibrationStorage();
  resetRobot();
  assert(Fake::displayRows[0].find("3pi+ R" + std::to_string(RACE_PLANNER_REVISION)) == 0);
  assert(Fake::displayRows[2].find("A CAL B RUN C MODE") == 0);
  assert(Fake::displayRows[5].find("Battery: 5.00V") == 0);
  assert(Fake::displayRows[7].find("A CAL B RUN C MODE") == 0);
  advance(20000);
  assert(state == IDLE && Fake::leftMotor == 0 && Fake::rightMotor == 0);
  press(1);
  expectStopped("Cal A!");

  resetRobot();
  press(2);
  assert(selectedMode == MID && state == IDLE && Fake::leftMotor == 0);
  press(2);
  assert(selectedMode == BEAST && state == IDLE && Fake::rightMotor == 0);
  press(2);
  assert(selectedMode == SAFE);

  resetRobot();
  press(0);
  assert(Fake::displayRows[2].find("Hands clear") == 0);
  advance(1000);
  assert(state == COUNTDOWN && Fake::leftMotor == 0 && Fake::rightMotor == 0);
  assert(Fake::displayRows[2].find("Hands clear") == 0);
  advance(1000);
  assert(state == CALIBRATING);
  assert(Fake::displayRows[0].find("AUTO CAL") == 0);
  Fake::buttons[2] = true;
  tick();
  expectStopped("Stopped");

  resetRobot();
  Fake::goodCalibration = false;
  press(0);
  advance(10100);
  assert(!calibrated);
  expectStopped("Bad cal");

  resetRobot();
  calibrateRobot();
  lineSensors.calibrationOn.maximum[4] = lineSensors.calibrationOn.minimum[4];
  assert(!calibrationOK());
  lineSensors.calibrationOn.maximum = nullptr;
  assert(!calibrationOK());

  resetRobot();
  press(0);
  press(2);
  assert(!calibrated);
  expectStopped("Stopped");

  resetRobot();
  Fake::buttons[0] = true;
  advance(5000);
  assert(state == WAIT_RELEASE && Fake::leftMotor == 0 && Fake::rightMotor == 0);
  Fake::buttons[0] = false;
  advance(70);
  assert(state == COUNTDOWN);
  press(1);
  expectStopped("Stopped");

  resetRobot();
  Fake::usb = true;
  press(0);
  expectStopped("USB out");

  resetRobot();
  Fake::battery = 4300;
  press(0);
  expectStopped("Bat low");

  for (uint8_t button = 0; button < 3; button++)
  {
    resetRobot();
    beginAutoCalibration();
    advance(100);
    Fake::buttons[button] = true;
    tick();
    expectStopped("Stopped");
    assert(!calibrated);
  }

  resetRobot();
  beginAutoCalibration();
  Fake::usb = true;
  tick();
  expectStopped("USB out");

  resetRobot();
  beginAutoCalibration();
  Fake::battery = 4300;
  advance(100);
  expectStopped("Bat low");

  resetRobot();
  beginAutoCalibration();
  Fake::leftCounts = 0;
  advance(1000);
  expectStopped("Stalled");

  resetRobot();
  beginAutoCalibration();
  Fake::rightCounts = -10;
  advance(100);
  expectStopped("Enc dir");

  resetRobot();
  beginAutoCalibration();
  Fake::encoderErrors[0] = true;
  tick();
  expectStopped("Enc err");

  resetRobot();
  beginAutoCalibration();
  Fake::leftCounts = Fake::rightCounts = 1;
  advance(5000);
  expectStopped("Cal time");

  resetRobot();
  beginAutoCalibration();
  stateSince = Fake::now - Settings::CALIBRATION_MS;
  tick();
  expectStopped("Cal time");

  resetRobot();
  calibrateRobot();
  assert(Fake::greenLed);
  for (uint8_t i = 0; i < 5; i++)
  { assert(Fake::displayRows[i + 2].find(std::to_string(i) + ":") == 0); }
  assert(Fake::displayRows[4].find("1000") != std::string::npos);
  assert(Fake::displayRows[7].find("CENTERED") == 0);
  Fake::position = 1700;
  advance(100);
  assert(!Fake::greenLed && Fake::displayRows[7].find("SHIFT LEFT") == 0);
  Fake::position = 2300;
  advance(100);
  assert(Fake::displayRows[7].find("SHIFT RIGHT") == 0);
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = 0; }
  advance(100);
  assert(!Fake::greenLed && Fake::displayRows[7].find("NO LINE") == 0);
  assert(Fake::displayRows[7].find("NO LINE") == 0);
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = 1000; }
  advance(100);
  assert(!Fake::greenLed && Fake::displayRows[7].find("WIDE/OFF") == 0);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);

  resetRobot();
  calibrateRobot();
  Fake::usb = true;
  press(1);
  expectStopped("USB out");

  // Do not cancel/feed the core's watchdog when it requests the bootloader.
  resetRobot();
  Fake::usb = true;
  tick();
  assert(!watchdogEnabled);
  Fake::serialBaud = 1200;
  const unsigned resetsBeforeUpload = Fake::watchdogResets;
  const unsigned changesBeforeUpload = Fake::watchdogChanges;
  advance(500);
  assert(Fake::watchdogResets == resetsBeforeUpload);
  assert(Fake::watchdogChanges == changesBeforeUpload);
  Fake::usb = false;
  tick();
  assert(watchdogEnabled && Fake::watchdogResets > resetsBeforeUpload);

  resetRobot();
  calibrateRobot();
  Fake::bump = 2;
  press(1);
  expectStopped("Bumped");

  resetRobot();
  beginAutoCalibration();
  Fake::bump = 1;
  tick(1);
  expectStopped("Bumped");

  resetRobot();
  calibrateRobot();
  press(1);
  assert(state == COUNTDOWN);
  Fake::bump = 3;
  tick(1);
  expectStopped("Bumped");

  resetRobot();
  calibrateRobot();
  press(1);
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = 0; }
  advance(2000);
  expectStopped("Centre!");

  for (uint16_t voltage : {uint16_t(4300), uint16_t(6700)})
  {
    resetRobot();
    calibrateRobot();
    Fake::battery = voltage;
    press(1);
    expectStopped(voltage < 4400 ? "Bat low" : "Bat high");
  }

  resetRobot();
  calibrateRobot();
  Fake::position = 3500;
  press(1);
  expectStopped("Centre!");

  resetRobot();
  calibrateRobot();
  Fake::buttons[1] = true;
  advance(4000);
  assert(state == WAIT_RELEASE && Fake::leftMotor == 0);
  Fake::buttons[1] = false;
  advance(70);
  assert(state == COUNTDOWN);
  press(2);
  expectStopped("Stopped");

  resetRobot();
  calibrateRobot();
  press(1);
  Fake::usb = true;
  tick();
  expectStopped("USB out");

  // Calibration ramp bounds; actual driving motor limits are exercised below.
  for (int16_t current = 0; current <= 180; current++)
  {
    for (int16_t target = 0; target <= 180; target++)
    {
      const int16_t next = rampCommand(current, target);
      assert(next >= 0 && next <= 180 && next <= current + 3);
      if (target <= current) { assert(next == target); }
    }
  }

  // Straight pace is higher, while large offsets/rapid changes demand corner pace.
  race.reset(selectedMode, Fake::now);
  assert(baseSpeedForLine(0, 0, false) == 80);
  assert(baseSpeedForLine(100, 60, false) == 80);
  assert(baseSpeedForLine(2000, 0, false) == 65);
  assert(baseSpeedForLine(0, 1000, false) == 65);
  assert(baseSpeedForLine(0, 0, true) == 65);
  int16_t previousBase = 80;
  for (int16_t error = 0; error <= 2000; error += 25)
  {
    const int16_t base = baseSpeedForLine(error, 0, false);
    assert(base >= 65 && base <= previousBase);
    assert(base == baseSpeedForLine(-error, 0, false));
    previousBase = base;
  }

  // An already cautious map target is a cap, not another multiplicative penalty.
  for (const auto mode : {SAFE, MID, BEAST})
  {
    race.reset(mode, Fake::now);
    race.mapReady = true;
    race.completed = 1;
    race.learnedCounts = 20000;
    race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
    memset(race.curves, 127, sizeof(race.curves));
    const int16_t live = race.profile().cruise - (race.profile().cruise - 65) * 400 / 1000;
    const int16_t mapped = race.plannedCruise(true);
    assert(baseSpeedForLine(500, 0, false) == (live < mapped ? live : mapped));
    assert(baseSpeedForLine(2000, 0, false) == 65);
    assert(baseSpeedForLine(0, 0, true) == 65);
  }
  race.reset(selectedMode, Fake::now);

  readyRun();
  const unsigned writesBeforeDriving = Fake::displayWrites;
  advance(500);
  assert(Fake::leftMotor == 80 && Fake::rightMotor == 80);
  assert(Fake::displayWrites == writesBeforeDriving);
  assert(Fake::displayRows[0].find_first_not_of(' ') == std::string::npos);
  assert(!display.automatic);
  Fake::position = 4000;
  advance(200);
  assert(Fake::leftMotor <= 100 && Fake::rightMotor <= 10);
  assert(Fake::leftMotor > Fake::rightMotor);

  for (const auto mode : {SAFE, MID, BEAST})
  {
    readyRun();
    selectedMode = mode;
    race.reset(mode, Fake::now);
    // Supply a known flat map to exercise the actual replay motor-output path.
    race.mapReady = true;
    race.completed = 1;
    race.learnedCounts = 20000;
    race.binsUsed = (race.learnedCounts + RaceSession::BIN_COUNTS - 1) / RaceSession::BIN_COUNTS;
    memset(race.curves, 0, sizeof(race.curves));
    assert(baseSpeedForLine(100, 60, false) == driveProfile(mode, false).cruise);
    advance(1500);
    assert(state == RUNNING);
    assert(Fake::leftMotor == driveProfile(mode, false).cruise);
    assert(Fake::rightMotor == driveProfile(mode, false).cruise);
  }

  readyRun();
  int16_t previousForward = forwardCommand;
  for (uint16_t position = 0; position <= 4000; position += 25)
  {
    Fake::position = position;
    tick();
    assert(state == RUNNING);
    assert(forwardCommand <= previousForward + Settings::DRIVE_ACCEL_STEP);
    assert(Fake::leftMotor >= 0 && Fake::leftMotor <= race.profile().limit);
    assert(Fake::rightMotor >= 0 && Fake::rightMotor <= race.profile().limit);
    previousForward = forwardCommand;
  }
  Fake::position = 3000;
  advance(100);
  assert(Fake::leftMotor > Fake::rightMotor); // Line right -> steer right.
  Fake::position = 1000;
  advance(100);
  assert(Fake::leftMotor < Fake::rightMotor);

  readyRun();
  Fake::position = 2100;
  tick(10);
  assert(filteredDelta == 50);
  readyRun();
  Fake::position = 2200;
  tick(20);
  assert(filteredDelta == 80); // Same slope; a longer interval advances the filter further.
  Fake::position = 2000;
  tick(20);
  assert(filteredDelta < 0); // Negative derivative must stay signed.

  for (uint8_t button = 0; button < 3; button++)
  {
    readyRun();
    Fake::buttons[button] = true;
    tick(1);
    expectStopped("Stopped");
    // Holding a stop button must never restart/calibrate the robot.
    advance(10000);
    assert(state == IDLE);
  }

  readyRun();
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = 0; }
  tick();
  expectStopped("No line");

  // Track bumps, including sustained contact, must not abort a running lap.
  for (const auto mode : {SAFE, MID, BEAST})
  {
    readyRun();
    race.mode = selectedMode = mode;
    for (const uint8_t bump : {1, 2, 3, 0})
    {
      Fake::bump = bump;
      advance(500);
      assert(state == RUNNING && Fake::leftMotor > 0 && Fake::rightMotor > 0);
      assert(Fake::displayRows[0].find("Bumped") == std::string::npos);
    }
    Fake::bump = 3;
    Fake::buttons[2] = true;
    tick(1);
    expectStopped("Stopped");
  }

  readyRun();
  Fake::usb = true;
  tick(1);
  expectStopped("USB out");

  readyRun();
  Fake::battery = 4300;
  advance(100);
  expectStopped("Bat low");

  readyRun();
  Fake::leftCounts = 0;
  advance(500);
  expectStopped("Stalled");

  readyRun();
  Fake::rightCounts = 0;
  advance(500);
  expectStopped("Stalled");

  readyRun();
  Fake::rightCounts = -10;
  advance(100);
  expectStopped("Enc dir");

  readyRun();
  Fake::encoderErrors[1] = true;
  tick();
  expectStopped("Enc err");

  readyRun();
  for (uint8_t i = 0; i < 5; i++) { Fake::readings[i] = 1000; }
  advance(100);
  assert(state == RUNNING && Fake::leftMotor == Fake::rightMotor);
  advance(120);
  expectStopped("Dark/air");

  readyRun();
  tick(51);
  expectStopped("Loop lag");

  readyRun();
  stateSince = Fake::now - Settings::MAX_RUN_MS;
  tick();
  expectStopped("Time end");

  // Start before uint32 millis rollover; complete calibration/run across it.
  resetRobot(std::numeric_limits<uint32_t>::max() - 9000);
  calibrateRobot();
  startRobot();
  advance(3000);
  assert(state == RUNNING);
  stateSince = Fake::now - Settings::MAX_RUN_MS;
  tick();
  expectStopped("Time end");

  readyRun();
  Fake::gyroFresh = false;
  advance(50);
  expectStopped("Gyro stale");

  readyRun();
  Fake::imuError = 1;
  tick();
  expectStopped("IMU error");

  readyRun();
  Fake::wireTimeout = true;
  tick();
  expectStopped("IMU error");

  readyRun();
  Fake::gyroRaw = 30000;
  tick();
  expectStopped("Gyro range");

  resetRobot();
  calibrateRobot();
  Fake::gyroFresh = false;
  press(1);
  advance(2000);
  expectStopped("Gyro cal");

  resetRobot();
  calibrateRobot();
  press(1);
  advance(1000);
  Fake::gyroRaw = 500;
  advance(1000);
  expectStopped("Hold still");

  readyRun();
  race.lapStarted = Fake::now - RaceSession::LAP_TIMEOUT_MS;
  tick();
  expectStopped("No finish");

  testNoisyStraight();
  testSmoothBraking();
  testAccelerationConfidence();
  testTightLaunchGate();
  testStraightRecoveryGate();
  testCurveRecovery();
  testLimiterShares();
  testReplayConsistency();
  testNoisyLearningFallback();
  testLimiterTelemetry();
  testCornerEntryYawPriority();
  testR24YawDamping();
  testR24OpticalEnvelope();
  testFailureReadings();
  testFullThrottleSteering();
  testExitLaunch();
  testCornerExitLineConfirmation();
  testFastControl();
  if (focused)
  {
    std::cout << "PASS: settled straight launch, R24 steering, startup, fixed calibration, OLED/limiter shares, buttons/faults, "
      "tight launch gates, fractional ramps, guarded exits and timed braking; full-course models skipped.\n";
    return 0;
  }

  // Full sketch: synthetic circular motion with realistic gyro/count agreement.
  // This validates software transitions, not real-world localization accuracy.
  readyRun();
  selectedMode = BEAST;
  race.reset(selectedMode, Fake::now);
  bool sawReplay = false;
  const unsigned writesDuringRace = Fake::displayWrites;
  for (uint16_t frame = 0; frame < 6000 && state == RUNNING; frame++)
  {
    const int16_t left = Fake::leftCounts * Fake::leftMotor / 60;
    const int16_t right = Fake::rightCounts * Fake::rightMotor / 60;
    const float step = (left + right) * 0.5f * RaceSession::MM_PER_COUNT;
    const float turn = step / 300.0f;
    Fake::gyroRaw = static_cast<int16_t>(lroundf(turn / (0.07f * 3.14159265f / 180 * 0.01f)));
    tick();
    if (state == RUNNING) { assert(Fake::displayWrites == writesDuringRace); }
    if (race.completed == 1 && state == RUNNING) { sawReplay = true; }
  }
  assert(sawReplay && race.completed == 3 && state == IDLE);
  assert(Fake::displayRows[0].find("3 LAPS EST. DONE") == 0);
  assert(Fake::leftMotor == 0 && Fake::rightMotor == 0);
  assert(race.lapTimes[0] > 0 && race.lapTimes[1] > 0 && race.lapTimes[2] > 0);
  advance(10000);
  assert(race.completed == 3 && state == IDLE && Fake::leftMotor == 0);

  MotionMetrics reference[3];
  for (const auto mode : {SAFE, MID, BEAST})
  {
    reference[mode] = testRoundedSquare(mode, 500, 150, 1);
    testRoundedSquare(mode, 500, 150, -1);
    testRoundedSquare(mode, 200, 75, 1);
    testRoundedSquare(mode, 500, 150, 1, 3.75f);
  }
  assert(reference[BEAST].straightSpeed[1] > reference[SAFE].straightSpeed[1] * 1.4f);
  assert(reference[BEAST].lapTimes[1] < reference[SAFE].lapTimes[1]);
  MotionMetrics sizedTracks[3][2];
  for (const auto mode : {MID, BEAST})
  {
    testRoundedSquare(mode, 500, 300, 1);
    assert(race.turnVotes >= 4);
    testRoundedSquare(mode, 500, 300, -1);
    assert(race.turnVotes >= 4);
    testRoundedSquare(mode, 200, 75, -1);
    testRoundedSquare(mode, 200, 75, 1, 3.75f);
    testRoundedSquare(mode, 200, 75, -1, 3.75f);
    testRoundedSquare(mode, 200, 75, 1, 2.5f, 40);
    testRoundedSquare(mode, 200, 75, -1, 2.5f, 40);
    uint8_t gainIndex = 0;
    for (const float gain : {2.5f, 3.75f})
    {
      sizedTracks[mode][gainIndex++] = testRoundedSquare(mode, 600, 200, 1, gain);
      testRoundedSquare(mode, 600, 200, -1, gain);
    }
    testRoundedSquare(mode, 600, 200, 1, 2.5f, 40);
  }
  // Frozen R5 controller, same geometry, motor models and physical lap checks.
  const uint32_t r5Replay[2][2] = {{10650, 10590}, {7430, 7390}};
  for (int gain = 0; gain < 2; gain++)
  {
    for (int lap = 1; lap < 3; lap++)
    {
      assert(sizedTracks[BEAST][gain].lapTimes[lap] * 100 < r5Replay[gain][lap - 1] * 80);
      assert(sizedTracks[BEAST][gain].lapTimes[lap] * 100 < sizedTracks[MID][gain].lapTimes[lap] * 85);
    }
  }
  // R8's best flying laps in the same models were 4.610 s and 4.420 s.
  const uint32_t r8Best[2] = {4610, 4420};
  const uint8_t maximumPercent[2] = {95, 80};
  for (int gain = 0; gain < 2; gain++)
  {
    const MotionMetrics & course = sizedTracks[BEAST][gain];
    const uint32_t best = course.lapTimes[1] < course.lapTimes[2] ?
      course.lapTimes[1] : course.lapTimes[2];
    assert(course.fastReplayUsed);
    assert(best * 100 <= r8Best[gain] * maximumPercent[gain]);
  }
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 40, -1);
  assert(race.turnVotes <= -4 && abs(int(race.trackWidthMm) - 86) < 10);
  testRoundedSquare(BEAST, 600, 200, -1, 3.75f, 40, -1, 100);
  assert(race.turnVotes <= -4 && abs(int(race.trackWidthMm) - 100) < 10);
  testRoundedSquare(BEAST, 500, 300, -1, 2.5f, 40, -1);
  assert(race.turnVotes <= -4 && abs(int(race.trackWidthMm) - 86) < 10);
  const MotionMetrics noisyLong = testRoundedSquare(BEAST, 1400, 150, 1, 2.5f, 40);
  for (int lap = 1; lap < 3; lap++)
  {
    assert(noisyLong.peakStraightPwm[lap] >= 360);
    assert(noisyLong.brakingBeforeCorner[lap] >= 4);
    assert(noisyLong.approachSpeed[lap] < noisyLong.peakStraightSpeed[lap] * 0.85f);
    assert(noisyLong.maxCrossTrack[lap] < 15);
    assert(noisyLong.maxCommonDrop[lap] <= Settings::DRIVE_BRAKE_STEP);
  }
  const MotionMetrics fastLong = testRoundedSquare(BEAST, 2200, 150, 1, 3.75f, 40);
  for (int lap = 1; lap < 3; lap++)
  {
    assert(fastLong.peakStraightPwm[lap] >= 360);
    assert(fastLong.brakingBeforeCorner[lap] >= 4);
    assert(fastLong.approachSpeed[lap] < fastLong.peakStraightSpeed[lap] * 0.85f);
    assert(fastLong.maxCommonDrop[lap] <= Settings::DRIVE_BRAKE_STEP);
  }
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 40, 1, 86, 0.03f);
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 100, -1, 86, 0.08f);
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 40, 1, 86, 0.08f);
  // Additional motor-response cases pass the frozen R8 quality thresholds.
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 0, 1, 86, 0, 0.15f);
  testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 0, 1, 86, 0, 0.35f);
  testRoundedSquare(BEAST, 200, 75, 1, 2.5f, 0, 1, 86, 0, 0.35f);
  // Stress the conservative path without relaxing any course/braking bounds.
  for (const uint32_t seed : {0x51a7u, 0u, 1u, 2u, 3u})
  {
    testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 175, 1, 86, 0, 0.25f, seed);
    testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 200, 1, 86, 0, 0.25f, seed);
    testRoundedSquare(BEAST, 600, 200, 1, 3.75f, 200, 1, 86, 0.03f, 0.25f, seed);
    testRoundedSquare(BEAST, 200, 75, 1, 3.75f, 200, 1, 86, 0, 0.25f, seed);
    testRoundedSquare(BEAST, 2200, 150, 1, 3.75f, 200, 1, 86, 0, 0.25f, seed);
  }
  testStraightStability();

  std::cout << "PASS: startup, automatic calibration bounds/reversal pauses, "
    "OLED screens/countdown/centering, modes, gyro faults, three-lap latch, "
    "arming, motor limits, time-normalized steering, "
    "noise-tolerant common throttle, timed soft braking, rounded-square motion cases, "
    "physical straight stability with imbalance/noise/delay/jitter, "
    "all stop paths, USB upload watchdog handoff, no automatic restart, "
    "and millis rollover.\n";
  return 0;
}
