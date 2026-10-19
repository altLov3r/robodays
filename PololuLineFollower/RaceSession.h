#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

constexpr uint8_t RACE_PLANNER_REVISION = 43;

enum DriveMode : uint8_t { SAFE, MID, BEAST };

struct DriveProfile
{
  int16_t cruise;
  int16_t limit;
};

inline DriveProfile driveProfile(DriveMode mode, bool learning)
{
  if (learning) { return {80, 100}; }
  if (mode == MID) { return {180, 220}; }
  if (mode == BEAST) { return {400, 400}; }
  return {120, 140};
}

// Distance-indexed signed curvature, without an absolute position reference.
// Loop closure is an estimate: wheel slip/gyro drift can prevent or fool it.
class RaceSession
{
public:
  static constexpr uint16_t BIN_COUNTS = 128;
  static constexpr uint16_t MAP_CAPACITY = 512;
  static constexpr uint8_t MAX_MAP_SHIFT = 2;
  static_assert(BIN_COUNTS == 128, "Map indexing uses seven base distance bits.");
  static_assert((uint32_t(MAP_CAPACITY) << MAX_MAP_SHIFT) < 65536UL,
    "Base-bin indices must fit in 16 bits.");
  static constexpr uint32_t LAP_TIMEOUT_MS = 180000;
  static constexpr uint8_t MAP_CAUTION_MS = 30;
  static constexpr float MM_PER_COUNT = 32.0f * 3.14159265f / 358.3f;
  static constexpr int8_t UNKNOWN_CURVE = -128;
  enum Fault : uint8_t { NONE, MAP_FULL, MAP_GAP, MAP_MISMATCH, NO_FINISH, BAD_MOTION };
  enum Event : uint8_t { NO_EVENT, LAP_FINISHED, SESSION_FINISHED };

  DriveMode mode = SAFE;
  Fault fault = NONE;
  bool running = false;
  bool mapReady = false;
  uint8_t completed = 0;
  uint32_t lapTimes[3] = {};
  uint32_t learnedCounts = 0;
  uint16_t binsUsed = 0;
  uint8_t mapShift = 0;
  int8_t curves[MAP_CAPACITY]; // Signed curvature * 6000: radius 75 mm is +/-80.
  uint32_t distanceTwice = 0;
  uint32_t lapStarted = 0;
  float x = 0, y = 0, heading = 0, speedMmS = 0;
  bool leftStartZone = false;
  bool returnArmed = false;
  uint8_t mismatchMs = 0; // Stops at 200; a validated tick adds at most 50 ms.
  int16_t filteredCurve = 0; // Curvature in eighths to retain filter precision.
  uint16_t speedScaleQ8 = 1024; // Measured mm/s per PWM, initially 4.0.
  // A clean Beast finish bounds later compensation; retained until power-off.
  uint16_t completedScaleQ8 = 0;
  uint8_t trackWidthMm = 86;
  int8_t turnVotes = 0;
  int16_t learningDifference = 0;
  float learningTurn = 0;
  uint8_t learningSamples = 0;
  int32_t mapCurveSum = 0;
  uint16_t mapWeight = 0;
  uint8_t learningNoise = 0;
  bool noisyLearning = false;
  mutable bool liveLimited = false; // Telemetry only; never changes the speed bound.
  mutable uint8_t launchCurveLimit = 0; // Q3 yaw limit; zero disables strong launches.
  uint8_t exitQuietMs = 0;
  uint8_t straightStableMs = 0;

  void reset(DriveMode selected, uint32_t now)
  {
    mode = selected;
    fault = NONE;
    running = true;
    mapReady = false;
    completed = 0;
    learnedCounts = 0;
    binsUsed = 0;
    mapShift = 0;
    speedMmS = 0;
    filteredCurve = 0;
    speedScaleQ8 = 1024;
    trackWidthMm = 86;
    turnVotes = 0;
    learningDifference = 0;
    learningTurn = 0;
    learningSamples = 0;
    mapCurveSum = 0;
    mapWeight = 0;
    learningNoise = 0;
    noisyLearning = false;
    liveLimited = false;
    launchCurveLimit = 0;
    exitQuietMs = 0;
    straightStableMs = 0;
    cachedSpeedScale = 0;
    memset(curves, 128, sizeof(curves));
    memset(lapTimes, 0, sizeof(lapTimes));
    resetLap(now);
  }

  void abort()
  {
    running = false;
    straightStableMs = 0;
    if (completed < 3) { mapReady = false; }
  }

  DriveProfile profile() const { return driveProfile(noisyLearning ? MID : mode, !mapReady); }
  int16_t steeringLimit() const
  { return !mapReady ? 90 : (noisyLearning ? 110 : (mode == BEAST ? 140 : (mode == MID ? 110 : 90))); }
  uint32_t distanceCounts() const { return distanceTwice / 2; }
  bool mapUncertain() const { return mismatchMs >= MAP_CAUTION_MS; }
  // Bias [-3,3] to [0,6] for a single byte comparison on the AVR.
  __attribute__((noinline)) bool trustedGeometry() const
  { return static_cast<uint8_t>(turnVotes + 3) > 6; }
  uint16_t mapBinCounts() const { return BIN_COUNTS << mapShift; }
  // Map-local positions fit after removing the seven fixed base-distance bits.
  __attribute__((noinline)) uint16_t mapBin(uint32_t point) const
  { return static_cast<uint16_t>(point >> 7) >> mapShift; }
  __attribute__((noinline)) uint32_t mapPosition() const
  { return distanceCounts() % learnedCounts; }

  void noteTracking(int16_t, int16_t delta, bool)
  {
    if (!running || mapReady || mode != BEAST) { return; }
    // Reject isolated spikes. Sustained noise still selects the fallback before
    // replay; its motor ceiling must never change during a racing lap.
    if (abs(delta) >= 150)
    { if (learningNoise < 8) { learningNoise++; } }
    else if (learningNoise) { learningNoise--; }
    if (learningNoise == 8) { noisyLearning = true; }
  }

  __attribute__((noinline)) void resetLap(uint32_t now)
  {
    distanceTwice = 0;
    lapStarted = now;
    x = y = heading = 0;
    leftStartZone = returnArmed = false;
    mismatchMs = 0;
  }

  Event update(int16_t left, int16_t right, float turnRadians,
    uint16_t elapsedMs, int16_t drivePwm, uint32_t now)
  {
    if (!running || fault != NONE || completed >= 3) { return NO_EVENT; }
    if (!elapsedMs || elapsedMs > 50 || left < -2 || right < -2 ||
      left > 500 || right > 500 || !isfinite(turnRadians) || fabsf(turnRadians) > 0.7f)
    {
      fault = BAD_MOTION;
      return NO_EVENT;
    }
    if (now - lapStarted >= LAP_TIMEOUT_MS)
    {
      fault = NO_FINISH;
      return NO_EVENT;
    }
    int16_t sum = left + right; // Validated wheel deltas bound this to -4..1000.
    if (sum < 0) { sum = 0; }
    distanceTwice += sum;
    const float step = sum * 0.5f * MM_PER_COUNT;
    // Encoder quantization must not make the braking horizon jump every sample.
    const float filterWeight = elapsedMs / (30.0f + elapsedMs);
    speedMmS += (step * 1000.0f / elapsedMs - speedMmS) * filterWeight;
    const float previousX = x;
    x += step * cosf(heading + turnRadians * 0.5f);
    y += step * sinf(heading + turnRadians * 0.5f);
    heading += turnRadians;
    if (heading > 3.14159265f) { heading -= 6.28318530f; }
    if (heading < -3.14159265f) { heading += 6.28318530f; }

    // Filter signed curvature: alternating steering jitter is not a sustained bend.
    float curve = sum > 0 ? turnRadians * 48000.0f / step : 0;
    if (curve > 1016) { curve = 1016; }
    if (curve < -1016) { curve = -1016; }
    const int16_t sampleCurve = static_cast<int16_t>(curve);
    const bool moving = sum >= 4; // At least 0.5 mm with the stock encoder scale.
    const int16_t liveCurve = moving ? sampleCurve : 0;
    if (mapReady && moving && abs(liveCurve) < 32)
    {
      if (exitQuietMs < 40) { exitQuietMs += elapsedMs; }
    }
    else { exitQuietMs = 0; }
    filteredCurve += static_cast<int32_t>(liveCurve - filteredCurve) *
      elapsedMs / (30 + elapsedMs);
    const int8_t bend = filteredCurve / 8;
    const uint8_t magnitude = abs(bend);
    const uint32_t distance = distanceCounts();
    if (!mapReady)
    {
      // Estimate the stock drivetrain's speed scale on settled learning straights.
      if (magnitude <= 3 && drivePwm >= 80 && speedMmS > 100)
      {
        const uint16_t sample = static_cast<uint32_t>(speedMmS) * 256 / drivePwm;
        if (sample >= 256 && sample <= 1536)
        { speedScaleQ8 = (speedScaleQ8 * 7 + sample) / 8; }
      }
      // Learn yaw polarity and effective wheel spacing from encoder/gyro agreement.
      // Do not assume the simulator's axis convention or exact tyre geometry.
      // Integrate paired measurements: slow/broad turns are below the per-tick
      // angle threshold, and single encoder counts otherwise bias the width.
      if (magnitude >= 8)
      {
        learningDifference += left - right;
        learningTurn += turnRadians;
        learningSamples++;
        if (fabsf(learningTurn) >= 0.04f || learningSamples >= 20)
        {
          if (fabsf(learningTurn) >= 0.04f)
          {
            const float width = learningDifference * MM_PER_COUNT / learningTurn;
            const float magnitudeWidth = fabsf(width);
            if (magnitudeWidth >= 59.5f && magnitudeWidth < 110.5f)
            {
              const uint16_t sample = static_cast<uint16_t>(magnitudeWidth + 0.5f);
              trackWidthMm = (trackWidthMm * 7 + sample + 4) / 8;
              if (width > 0 && turnVotes < 8) { turnVotes++; }
              if (width < 0 && turnVotes > -8) { turnVotes--; }
            }
          }
          learningDifference = 0;
          learningTurn = 0;
          learningSamples = 0;
        }
      }
      else
      {
        learningDifference = 0;
        learningTurn = 0;
        learningSamples = 0;
      }
      // Integrate signed turning by distance, splitting a sample at bin edges.
      // A single steering spike must not become a permanent corner speed cap.
      uint32_t point = distanceTwice - sum;
      uint16_t bin = mapBin(point / 2);
      if (bin > binsUsed) { fault = MAP_GAP; return NO_EVENT; }
      uint16_t remaining = sum;
      while (remaining)
      {
        if (bin >= MAP_CAPACITY)
        {
          if (mapShift == MAX_MAP_SHIFT) { fault = MAP_FULL; return NO_EVENT; }
          // In-place peak pooling cannot cancel opposite S-bends into a straight.
          for (uint16_t i = 0; i < MAP_CAPACITY / 2; i++)
          { curves[i] = peakCurve(curves[2 * i], curves[2 * i + 1]); }
          memset(curves + MAP_CAPACITY / 2, 128, MAP_CAPACITY / 2);
          binsUsed = MAP_CAPACITY / 2;
          mapShift++;
          bin /= 2;
        }
        if (!mapWeight) { mapCurveSum = 0; }
        uint16_t span = BIN_COUNTS * 2 - point % (BIN_COUNTS * 2);
        if (span > remaining) { span = remaining; }
        mapCurveSum += static_cast<int32_t>(sampleCurve) * span;
        mapWeight += span;
        const int8_t average = mapCurveSum / mapWeight / 8;
        if (!mapShift) { curves[bin] = average; }
        else if (mapWeight == BIN_COUNTS * 2)
        { curves[bin] = peakCurve(curves[bin], average); }
        binsUsed = bin + 1;
        if (mapWeight == BIN_COUNTS * 2) { mapWeight = 0; }
        point += span;
        remaining -= span;
        if (remaining) { bin = mapBin(point / 2); }
      }
    }
    else
    {
      if (distance > learnedCounts + learnedCounts * 15 / 100)
      {
        fault = NO_FINISH;
        return NO_EVENT;
      }
      if (!updateMapAgreement(bend, elapsedMs)) { return NO_EVENT; }
    }

    if (x * x + y * y > 150.0f * 150.0f) { leftStartZone = true; }
    if (leftStartZone && x < -50 && x > -300 && fabsf(y) < 150 && fabsf(heading) < 0.6f)
    {
      returnArmed = true;
    }
    // Cross the starting plane forwards, near the initial position and heading.
    const bool closure = returnArmed && previousX < 0 && x >= 0 &&
      fabsf(y) <= 80 && fabsf(heading) <= 0.25f;
    if (closure && distance * MM_PER_COUNT < 750)
    {
      fault = NO_FINISH; // Do not silently combine multiple small loops into one lap.
      return NO_EVENT;
    }
    if (!closure || (mapReady && distance < learnedCounts * 85 / 100)) { return NO_EVENT; }
    if (!mapReady)
    {
      learnedCounts = distance;
      if (mapShift && mapWeight)
      {
        const uint16_t bin = mapBin((distanceTwice - 1) / 2);
        curves[bin] = peakCurve(curves[bin], mapCurveSum / mapWeight / 8);
      }
      // A crossing exactly on a bin boundary must not retain a zero-length bin.
      binsUsed = mapBin(learnedCounts - 1) + 1;
      mapReady = true;
    }
    lapTimes[completed] = now - lapStarted;
    completed++;
    if (completed == 3)
    {
      if (mode == BEAST && !noisyLearning && trustedGeometry() && speedScaleQ8 > completedScaleQ8)
      { completedScaleQ8 = speedScaleQ8; }
      running = false;
      return SESSION_FINISHED;
    }
    resetLap(now); // Anchor the next lap at the estimated crossing; do not accumulate pose error.
    return LAP_FINISHED;
  }

  // Share the live/map calculation to keep AVR flash use bounded.
  __attribute__((noinline)) uint32_t cornerSquared(uint8_t curvature, uint32_t lateralScale) const
  {
    const DriveProfile active = profile();
    if (curvature <= 3) { return static_cast<uint32_t>(active.cruise) * active.cruise; }
    const uint16_t geometry = trackWidthMm * curvature;
    int16_t wheelCap = static_cast<uint32_t>(active.limit - 8) * 12000 / (12000 + geometry);
    const int16_t turnCap = static_cast<uint32_t>(steeringLimit() - 12) * 12000 / geometry;
    if (wheelCap > turnCap) { wheelCap = turnCap; }
    if (wheelCap > active.cruise) { wheelCap = active.cruise; }
    const uint32_t wheelSquared = static_cast<uint32_t>(wheelCap) * wheelCap;
    uint32_t allowed = lateralScale * 6000 / curvature;
    if (allowed > wheelSquared) { allowed = wheelSquared; }
    return allowed < 4225 ? 4225 : allowed;
  }

  __attribute__((noinline)) int16_t steeringFeedforward(int16_t base, int16_t error, uint8_t * yawWeight = nullptr) const
  {
    // Share map lookup: damping weight 0 disables it, 1 is half, 2 is full.
    if (yawWeight) { *yawWeight = 0; }
    if (!mapReady || !learnedCounts || !trustedGeometry()) { return 0; }
    const uint16_t bin = mapBin(mapPosition());
    const bool mappedStraight = mapPhase(bin) & 1;
    const bool cautious = mapUncertain();
    const bool measuredAssistOnly = mapShift || cautious || noisyLearning || (mode == BEAST && mappedStraight);
    if (yawWeight)
    {
      // Weaken damping before a bend, including the lap's partial final bin.
      *yawWeight = mappedStraight && abs(filteredCurve) < 64 ? 2 : 1;
    }
    int8_t mapped = curves[bin];
    int16_t live = filteredCurve / 8;
    // A pooled bin can contain both turn directions: steer from live yaw only.
    if ((mapShift || cautious) && mapped != UNKNOWN_CURVE) { mapped = live; }
    int16_t confidence = abs(live) * 8;
    if (confidence < abs(error)) { confidence = abs(error); }
    if (abs(live) < 8)
    {
      // Optical lookahead sees corner entry before the chassis starts rotating.
      if (measuredAssistOnly || abs(error) < 60) { return 0; }
      live = (error > 0) == (turnVotes > 0) ? 8 : -8;
    }
    // A confirmed live bend may lead or trail the learned bin by a few cm.
    for (int8_t neighbor = -1; !mapShift && !cautious && neighbor <= 1; neighbor++)
    {
      int16_t index = static_cast<int16_t>(bin) + neighbor;
      if (index < 0) { index = binsUsed - 1; }
      if (index >= static_cast<int16_t>(binsUsed)) { index = 0; }
      const int8_t candidate = curves[index];
      if (candidate != UNKNOWN_CURVE && static_cast<int16_t>(candidate) * live > 0 &&
        (mapped == UNKNOWN_CURVE || static_cast<int16_t>(mapped) * live <= 0 || abs(candidate) > abs(mapped)))
      { mapped = candidate; }
    }
    if (mapped == UNKNOWN_CURVE || static_cast<int16_t>(mapped) * live <= 0) { return 0; }
    int16_t bend = mapped;
    // Ramp the desired turn with measured entry confidence, without delaying
    // feedback or imposing a lagging slew limit on the wheel commands.
    if (!noisyLearning && confidence < 128)
    { bend = confidence <= 48 ? 0 : bend * (confidence - 48) / 80; }
    // Behind a bend, borrowed map curvature must not sustain more turn than
    // measured yaw. Entry and mapped bends retain their predictive assistance.
    if (measuredAssistOnly && abs(bend) > abs(live))
    { bend = live; }
    if (turnVotes < 0) { bend = -bend; }
    // Fade stale turn assistance continuously as the line asks to countersteer.
    // A binary veto caused assistance to switch on/off around corner exits.
    const int16_t opposing = bend > 0 ? -error : error;
    if (opposing >= 80) { return 0; }
    const int16_t assist = static_cast<int32_t>(base) * trackWidthMm * bend / 12000;
    return opposing > 0 ? static_cast<int32_t>(assist) * (80 - opposing) / 80 : assist;
  }

  __attribute__((noinline)) uint8_t mapPhase(uint16_t bin) const
  {
    const uint16_t previous = bin ? bin - 1 : binsUsed - 1;
    const uint16_t next = bin + 1 < binsUsed ? bin + 1 : 0;
    const uint8_t prior = abs(curves[previous]);
    if (abs(curves[next]) > 3) { return 0; }
    uint8_t phase = abs(curves[bin]) <= 3 ? 1 : 0;
    // The byte range 4..127 excludes straight bins and the unknown sentinel.
    if (curves[bin] != UNKNOWN_CURVE &&
      static_cast<uint8_t>(prior - 4) < 124 &&
      (!curves[bin] || (curves[previous] < 0) == (curves[bin] < 0))) { phase |= 2; }
    return phase; // Bit 0: straight; bit 1: mapped exit.
  }

  uint16_t yawScale() const
  {
    refreshDriveModel();
    return cachedYawScale;
  }

  __attribute__((noinline)) int16_t plannedCruise(bool trackingClear) const
  {
    liveLimited = false;
    const DriveProfile active = profile();
    if (!mapReady || !learnedCounts || !binsUsed)
    { launchCurveLimit = 0; return active.cruise; }
    // Leave extra entry distance for spatial averaging and map phase error;
    // keep planned deceleration below the unchanged 300 PWM/s output ramp.
    const bool sprint = mode == BEAST && !noisyLearning && trustedGeometry();
    refreshDriveModel();
    const uint16_t brakeGainQ8 = cachedBrakeGainQ8;
    const uint32_t lateralScale = cachedLateralScale;
    uint16_t guard = 125 + static_cast<uint16_t>(speedMmS * 0.04f / MM_PER_COUNT);
    if (sprint) { guard += guard / 2 + 64 + 2 * BIN_COUNTS; } // About 142 mm + 60 ms of travel.
    uint32_t targetSquared = static_cast<uint32_t>(active.cruise) * active.cruise;
    const uint16_t window = cachedHorizon + guard;
    const uint32_t liveSquared = cornerSquared(abs(filteredCurve / 8), lateralScale);
    if (liveSquared < targetSquared) { targetSquared = liveSquared; liveLimited = true; }
    uint32_t point = mapPosition();
    uint16_t offset = 0;
    uint8_t previousCurve = 0;
    // Release only the trailing bin of a mapped multi-bin bend. Live yaw
    // still caps speed; future bins retain the full corner-entry guard.
    const uint8_t phase = sprint ? mapPhase(mapBin(point)) : 0;
    // Quiet yaw alone can also mean understeer: require the current line sample.
    const bool settled = trackingClear && exitQuietMs >= 20;
    const bool exiting = !mapShift && (phase & 2) && settled;
    // Releasing a trailing curve bin must never qualify it for straight boost.
    launchCurveLimit = !mapUncertain() && (phase & 1) && exitQuietMs >= 20 ? 32 : 0;
    while (offset <= window)
    {
      const uint16_t bin = mapBin(point);
      const uint16_t clear = offset > guard ? offset - guard : 0;
      const uint32_t braking = static_cast<uint32_t>(clear) * brakeGainQ8 / 256;
      // Even the minimum corner target cannot constrain speed beyond this point.
      if (4225 + braking >= targetSquared) { break; }
      // Straights cannot lower the target. A repeated evaluated curvature only
      // loosens its bound as braking distance grows, even across straight bins.
      const uint8_t curvature = abs(curves[bin]);
      if (curvature > 3 && curvature != previousCurve && (offset || !exiting))
      {
        previousCurve = curvature;
        const uint32_t allowed = (curvature == 128 ? 4225 :
          cornerSquared(curvature, lateralScale)) + braking;
        if (allowed < targetSquared) { targetSquared = allowed; liveLimited = false; }
      }
      // The last map bin can be partial: visit bin zero immediately after wrap.
      const uint16_t nextBin = mapBinCounts() - (static_cast<uint16_t>(point) & (mapBinCounts() - 1));
      const uint32_t toStart = learnedCounts - point;
      const uint16_t step = toStart < nextBin ? static_cast<uint16_t>(toStart) : nextBin;
      offset += step;
      point += step;
      if (point == learnedCounts) { point = 0; }
    }
    // Integer floor square root; the motor ceiling bounds the result to 400.
    uint16_t root = 0;
    for (uint16_t bit = 256; bit; bit >>= 1)
    {
      const uint16_t candidate = root | bit;
      if (static_cast<uint32_t>(candidate) * candidate <= targetSquared) { root = candidate; }
    }
    return root;
  }

private:
  bool updateMapAgreement(int8_t bend, uint8_t elapsedMs)
  {
    const uint16_t bin = mapBin(mapPosition());
    uint16_t index = bin;
    // Gyro filtering and replay distance drift can shift a bend by two bins.
    // This tolerance is diagnostic only; the braking map is not shifted.
    for (uint8_t neighbor = 0; neighbor < 2; neighbor++)
    {
      index = index ? index - 1 : binsUsed - 1;
    }
    uint8_t expected = 0;
    for (uint8_t neighbor = 0; neighbor < 5; neighbor++)
    {
      const int8_t candidate = curves[index];
      // Fine maps retain direction; pooled S-bends retain magnitude only.
      if (candidate != UNKNOWN_CURVE && (mapShift || (candidate < 0) == (bend < 0)))
      {
        const uint8_t magnitude = abs(candidate);
        if (magnitude > expected) { expected = magnitude; }
      }
      if (++index == binsUsed) { index = 0; }
    }
    if (abs(bend) > expected + 16)
    {
      mismatchMs += elapsedMs;
      if (mismatchMs >= 200) { fault = MAP_MISMATCH; return false; }
    }
    else { mismatchMs = 0; }
    return true;
  }

  __attribute__((noinline)) static int8_t peakCurve(int8_t a, int8_t b)
  {
    const uint8_t magnitudeA = a < 0 ? -a : a;
    const uint8_t magnitudeB = b < 0 ? -b : b;
    return magnitudeA == 128 || (magnitudeB != 128 && magnitudeB > magnitudeA) ? b : a;
  }

  mutable uint16_t cachedSpeedScale = 0;
  mutable uint8_t cachedWidth = 0, cachedProfile = 0;
  mutable uint16_t cachedBrakeGainQ8 = 0, cachedYawScale = 0, cachedHorizon = 0;
  mutable uint32_t cachedLateralScale = 0;

  __attribute__((noinline)) void refreshDriveModel() const
  {
    const bool trusted = trustedGeometry();
    const uint8_t key = mode |
      (noisyLearning ? 4 : 0) | (trusted ? 8 : 0);
    const bool sprint = key == (BEAST | 8);
    // A lower learning estimate must not raise corner PWM/yaw gain after a
    // proven session. Stronger measured drivetrains still reduce both normally.
    const uint16_t modelScale = sprint &&
      speedScaleQ8 < completedScaleQ8 ? completedScaleQ8 : speedScaleQ8;
    if (cachedSpeedScale == modelScale && cachedWidth == trackWidthMm && cachedProfile == key)
    { return; }
    const uint16_t scale = modelScale + modelScale / 8;
    // Leave braking reserve below the 300 PWM/s output ramp.
    const uint16_t braking = noisyLearning ? 28672U : 35840U;
    cachedBrakeGainQ8 = sprint ?
      (static_cast<uint32_t>(braking) * 256 / scale) : (braking / scale) * 256;
    const uint16_t lateral = mode == BEAST ?
      (noisyLearning ? 2400 : (trusted ? 5600 : 3600)) : (mode == MID ? 2400 : 1200);
    cachedLateralScale = (static_cast<uint32_t>(lateral) * 65536) /
      (static_cast<uint32_t>(scale) * scale);
    cachedYawScale = 128000UL * trackWidthMm / modelScale;
    // The horizon is for replay even if yaw conversion was requested during learning.
    const int16_t cruise = driveProfile(noisyLearning ? MID : mode, false).cruise;
    cachedHorizon = (static_cast<uint32_t>(cruise) * cruise - 4225) * 256 / cachedBrakeGainQ8;
    cachedSpeedScale = modelScale;
    cachedWidth = trackWidthMm;
    cachedProfile = key;
  }
};
