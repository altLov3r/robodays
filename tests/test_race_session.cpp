#include <assert.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include "../PololuLineFollower/RaceSession.h"

void replayMap(RaceSession & race, DriveMode mode)
{
  race.reset(mode, 0);
  race.mapReady = true;
  race.completed = 1;
  race.learnedCounts = 20000;
  race.binsUsed = race.learnedCounts / RaceSession::BIN_COUNTS + 1;
  memset(race.curves, 0, sizeof(race.curves));
}

void testCircle(DriveMode mode, float sign, uint32_t clockStart)
{
  RaceSession race;
  race.reset(mode, clockStart);
  uint32_t now = clockStart;
  uint8_t finishes = 0;
  float fractionalCounts = 0;
  for (uint16_t frame = 0; frame < 1500 && race.running; frame++)
  {
    fractionalCounts += 5 / RaceSession::MM_PER_COUNT;
    const int16_t counts = static_cast<int16_t>(fractionalCounts);
    fractionalCounts -= counts;
    const float step = counts * RaceSession::MM_PER_COUNT;
    now += 10;
    const auto event = race.update(counts, counts, sign * step / 300, 10, 0, now);
    assert(race.fault == RaceSession::NONE);
    if (event != RaceSession::NO_EVENT)
    {
      finishes++;
      assert(race.completed == finishes && race.mapReady);
      assert(race.binsUsed > 5 && race.binsUsed <= RaceSession::MAP_CAPACITY);
      for (uint16_t i = 0; i < race.binsUsed; i++) { assert(race.curves[i] != RaceSession::UNKNOWN_CURVE); }
      assert(race.learnedCounts * RaceSession::MM_PER_COUNT > 1850);
      assert(race.learnedCounts * RaceSession::MM_PER_COUNT < 1920);
      assert(race.lapTimes[finishes - 1] >= 3700 && race.lapTimes[finishes - 1] <= 3900);
      if (finishes < 3) { assert(event == RaceSession::LAP_FINISHED && race.running); }
      else { assert(event == RaceSession::SESSION_FINISHED && !race.running); }
    }
  }
  assert(finishes == 3);
  for (uint8_t extra = 0; extra < 100; extra++)
  {
    now += 10;
    assert(race.update(20, 20, 0, 10, 0, now) == RaceSession::NO_EVENT);
    assert(race.completed == 3 && !race.running);
  }
}

void finishReplay(RaceSession & race)
{
  race.distanceTwice = (race.learnedCounts - 10) * 2;
  race.x = -1;
  race.y = race.heading = 0;
  race.leftStartZone = race.returnArmed = true;
  assert(race.update(20, 20, 0, 10, 100, 10) == RaceSession::LAP_FINISHED);
  assert(race.completed == 2);
}

// Independent dense-distance oracle: no bin skipping, run grouping or pruning.
int16_t denseCruise(const RaceSession & race, uint8_t revision = 41,
  bool * liveLimited = nullptr, bool releaseExit = true)
{
  const DriveProfile profile = race.profile();
  const bool r16 = revision == 16;
  const uint16_t scale = race.speedScaleQ8 + race.speedScaleQ8 / 8;
  const bool sprint = race.mode == BEAST && !race.noisyLearning && abs(race.turnVotes) >= 4;
  const uint16_t braking = (race.noisyLearning || (sprint && revision == 27) ? 28672U :
    (sprint && revision == 28 ? 32256U :
    (sprint && revision < 22 ? (r16 ? 39424U : 32256U) : 35840U))) / scale;
  const uint32_t brakeQ8 = revision >= 35 && sprint ?
    (revision >= 36 && revision < 38 ? 40140UL : 35840UL) * 256 / scale : uint32_t(braking) * 256;
  const uint16_t sprintLateral = revision >= 41 ? 5600 : (revision >= 36 && revision < 38 ? 6300 : (r16 ? 9000 : (revision == 17 ? 6500 :
    (revision == 18 ? 5000 : (revision < 22 || revision == 27 || revision == 28 ? 5400 : 6000)))));
  const uint16_t lateral = race.mode == BEAST ?
    (race.noisyLearning ? 2400 : (abs(race.turnVotes) < 4 ? 3600 : sprintLateral)) :
    (race.mode == MID ? 2400 : 1200);
  const uint32_t lateralScale = (static_cast<uint32_t>(lateral) * 65536 / scale) / scale;
  const uint16_t travel = static_cast<uint16_t>(race.speedMmS * 0.04f / RaceSession::MM_PER_COUNT);
  uint16_t guard = r16 && sprint ?
    64 + static_cast<uint16_t>(race.speedMmS * 0.02f / RaceSession::MM_PER_COUNT) :
    (revision >= 18 && sprint ? 251 + (3 * travel + 1) / 2 : 125 + travel);
  if (sprint && revision >= 29) { guard += RaceSession::BIN_COUNTS / 2; }
  else if (sprint && revision >= 27)
  { guard += RaceSession::BIN_COUNTS + static_cast<uint16_t>(race.speedMmS * 0.02f / RaceSession::MM_PER_COUNT); }
  if (sprint && revision >= 38) { guard += RaceSession::BIN_COUNTS / 2; }
  if (sprint && revision >= 41) { guard += RaceSession::BIN_COUNTS; }
  uint32_t target = static_cast<uint32_t>(profile.cruise) * profile.cruise;
  uint32_t mapped = target;
  const auto corner = [&](uint8_t curve) -> uint32_t
  { return race.cornerSquared(curve, lateralScale); };
  const uint16_t horizon = (target - 4225) * 256 / brakeQ8 + guard;
  const uint32_t initial = race.distanceCounts() % race.learnedCounts;
  const uint16_t current = initial / RaceSession::BIN_COUNTS;
  const uint32_t live = corner(abs(race.filteredCurve / 8));
  if (live < target) { target = live; }
  const uint16_t previous = current ? current - 1 : race.binsUsed - 1;
  const uint16_t next = (current + 1) % race.binsUsed;
  const int8_t prior = race.curves[previous];
  const int8_t bendNow = race.curves[current];
  const bool exit = releaseExit && ((revision >= 21 && revision < 27) || (revision >= 32 && race.exitQuietMs >= 20)) && sprint && bendNow != RaceSession::UNKNOWN_CURVE &&
    abs(race.curves[next]) <= 3 && abs(prior) >= 4 && abs(prior) <= 127 &&
    (!bendNow || (prior < 0) == (bendNow < 0));
  const uint32_t end = std::min((initial / RaceSession::BIN_COUNTS + 1) * RaceSession::BIN_COUNTS,
    race.learnedCounts);
  for (uint16_t offset = 0; offset <= horizon; offset++)
  {
    if (exit && initial + offset < end) { continue; }
    const uint32_t point = (race.distanceCounts() + offset) % race.learnedCounts;
    const int8_t bend = race.curves[point / RaceSession::BIN_COUNTS];
    uint32_t allowed = bend == RaceSession::UNKNOWN_CURVE ? 4225 :
      corner(abs(bend));
    if (offset > guard) { allowed += static_cast<uint32_t>(offset - guard) * brakeQ8 / 256; }
    if (allowed < mapped) { mapped = allowed; }
    if (allowed < target) { target = allowed; }
  }
  if (liveLimited)
  { *liveLimited = live < static_cast<uint32_t>(profile.cruise) * profile.cruise && live <= mapped; }
  uint16_t result = sqrtf(target);
  while (static_cast<uint32_t>(result + 1) * (result + 1) <= target) { result++; }
  while (static_cast<uint32_t>(result) * result > target) { result--; }
  return result;
}

void testPlannerGrouping()
{
  uint32_t random = 0x31f2u;
  const int8_t bends[] = {0, 3, -3, 8, -8, 16, -16, 40, -40, 80, -80, 127, -127, -128};
  for (unsigned sample = 0; sample < 256; sample++)
  {
    RaceSession race;
    replayMap(race, static_cast<DriveMode>(sample % 3));
    race.binsUsed = sample % 8 == 0 ? 512 : 7 + sample % 90;
    race.learnedCounts = race.binsUsed * uint32_t(RaceSession::BIN_COUNTS) - (sample % 3 ? 17 : 0);
    race.noisyLearning = race.mode == BEAST && sample % 7 == 0;
    race.turnVotes = sample % 4 ? 8 : 0;
    race.trackWidthMm = 60 + sample % 51;
    race.speedScaleQ8 = 256 + sample * 17 % 1281;
    race.speedMmS = sample * 37 % 1600;
    race.filteredCurve = sample % 5 ? 0 : bends[sample % 13] * 8;
    race.exitQuietMs = sample % 51;
    for (uint16_t bin = 0; bin < race.binsUsed; bin++)
    {
      if (bin % 7 == 0) { random = random * 1664525u + 1013904223u; }
      race.curves[bin] = sample % 8 == 0 ? 0 : bends[(random >> 12) % 14];
      if (sample % 8 == 1) { race.curves[bin] = bin % 2 ? 40 : -40; }
    }
    race.distanceTwice = (sample % 2 ? race.learnedCounts - 1 : random % race.learnedCounts) * 2;
    bool liveLimited = false;
    assert(race.plannedCruise(true) == denseCruise(race, 41, &liveLimited));
    assert(race.liveLimited == liveLimited);
    const uint16_t bin = race.mapPosition() / RaceSession::BIN_COUNTS;
    const int8_t current = race.curves[bin];
    const int8_t next = race.curves[(bin + 1) % race.binsUsed];
    const bool sprint = race.mode == BEAST && !race.noisyLearning && abs(race.turnVotes) >= 4;
    const bool straight = abs(current) <= 3 && abs(next) <= 3;
    assert(race.launchCurveLimit == (sprint && straight && race.exitQuietMs >= 20 ? 32 : 0));
    assert(race.plannedCruise(true) <= denseCruise(race, 40));
    const int16_t r26 = denseCruise(race, 26);
    assert(race.plannedCruise(true) <= denseCruise(race, 35));
    if (race.mode != BEAST || race.noisyLearning || abs(race.turnVotes) < 4)
    { assert(race.plannedCruise(true) == r26); }
    const int16_t settledTarget = race.plannedCruise(true);
    assert(race.plannedCruise(false) == denseCruise(race, 41, nullptr, false));
    assert(race.launchCurveLimit == (sprint && straight && race.exitQuietMs >= 20 ? 32 : 0));
    assert(race.plannedCruise(false) <= settledTarget);
  }
  for (const uint32_t finish : {4096u, 4116u})
  {
    RaceSession boundary;
    boundary.reset(BEAST, 0);
    boundary.distanceTwice = (finish - 10) * 2;
    boundary.binsUsed = (finish - 10) / RaceSession::BIN_COUNTS + 1;
    memset(boundary.curves, 0, sizeof(boundary.curves));
    boundary.x = -1;
    boundary.leftStartZone = boundary.returnArmed = true;
    assert(boundary.update(10, 10, 0, 10, 80, 10) == RaceSession::LAP_FINISHED);
    assert(boundary.learnedCounts == finish);
    assert(boundary.binsUsed == (finish - 1) / RaceSession::BIN_COUNTS + 1);
  }
}

void testSprintEnvelope()
{
  RaceSession race;
  replayMap(race, BEAST);
  race.turnVotes = 8;
  memset(race.curves, 30, sizeof(race.curves));
  assert(race.plannedCruise(true) == 234);
  assert(race.plannedCruise(true) == denseCruise(race));
  const int16_t pace = race.plannedCruise(true);
  // No session-wide pace ratchet: line recovery is handled by the live cap.
  race.noteTracking(400, 180, true);
  assert(race.plannedCruise(true) == pace && !race.noisyLearning);
  race.completed = 2;
  assert(race.plannedCruise(true) == pace);
  race.turnVotes = 0;
  assert(race.plannedCruise(true) < pace);
  race.noisyLearning = true;
  assert(race.profile().limit == 220 && race.plannedCruise(true) < pace);
}

void testCornerEntryBraking()
{
  for (const int direction : {-1, 1})
  {
    for (const int speed : {500, 1000, 1500})
    {
      RaceSession race;
      replayMap(race, BEAST);
      race.turnVotes = 8;
      race.speedMmS = speed;
      race.curves[100] = direction * 30;
      assert(race.plannedCruise(true) == 400); // Long clear straights retain full pace.
      race.curves[100] = 0;
      for (unsigned bin = 40; bin < 50; bin++) { race.curves[bin] = direction * 30; }
      int16_t previous = 400;
      bool earlierBraking = false;
      for (unsigned position = 0; position <= 40 * RaceSession::BIN_COUNTS; position += 32)
      {
        race.distanceTwice = position * 2;
        const int16_t target = race.plannedCruise(true);
        assert(target == denseCruise(race) && target <= denseCruise(race, 35));
        const int16_t r40 = denseCruise(race, 40);
        assert(target <= r40);
        if (target < r40) { earlierBraking = true; }
        assert(target <= previous && target >= 234);
        const float distanceMm = (40 * RaceSession::BIN_COUNTS - position) * RaceSession::MM_PER_COUNT;
        const float scale = (race.speedScaleQ8 + race.speedScaleQ8 / 8) / 256.0f;
        assert(target * long(target) <= 235L * 235 + 2 * 250 * distanceMm / scale);
        previous = target;
      }
      assert(previous == 234 && earlierBraking);
      const unsigned travel = static_cast<uint16_t>(speed * 0.04f / RaceSession::MM_PER_COUNT);
      const unsigned guard = 251 + (3 * travel + 1) / 2 + 2 * RaceSession::BIN_COUNTS;
      race.distanceTwice = (40 * RaceSession::BIN_COUNTS - guard) * 2;
      assert(race.plannedCruise(true) == 234 && denseCruise(race, 40) > 234);
      // Reach the lower corner pace one map bin earlier than R40.

      // A delayed map must not postpone slowing until the physical corner.
      // Shift its bend two bins late and still reach corner pace 40 ms early.
      memset(race.curves, 0, sizeof(race.curves));
      for (unsigned bin = 42; bin < 52; bin++) { race.curves[bin] = direction * 30; }
      race.distanceTwice = (40 * RaceSession::BIN_COUNTS - travel) * 2;
      assert(race.plannedCruise(true) == 234);
      assert(denseCruise(race, 17) > 252);

      // Three late map bins: request bend pace with 20 ms left before entry.
      memset(race.curves, 0, sizeof(race.curves));
      for (unsigned bin = 43; bin < 53; bin++) { race.curves[bin] = direction * 30; }
      const unsigned approach = static_cast<uint16_t>(speed * 0.02f / RaceSession::MM_PER_COUNT);
      race.distanceTwice = (40 * RaceSession::BIN_COUNTS - approach) * 2;
      assert(race.plannedCruise(true) == 234);
      race.distanceTwice = (43 * RaceSession::BIN_COUNTS - guard) * 2;
      assert(race.plannedCruise(true) == 234 && denseCruise(race, 26) > 234);
    }
  }
}

void testBoundedBeastPace()
{
  for (const DriveMode mode : {SAFE, MID, BEAST})
  for (const bool noisy : {false, true})
  for (const int votes : {3, 8, -8})
  for (const uint16_t scale : {256, 820, 1024, 1536})
  {
    RaceSession race;
    replayMap(race, mode);
    race.noisyLearning = noisy;
    race.turnVotes = votes;
    race.speedScaleQ8 = scale;
    // At the bend itself, braking distance cannot mask a larger corner budget.
    race.binsUsed = 4;
    race.learnedCounts = 4 * RaceSession::BIN_COUNTS;
    for (int curve = 4; curve <= 127; curve++)
    {
      memset(race.curves, votes < 0 ? -curve : curve, sizeof(race.curves));
      const int old = denseCruise(race, 35);
      const int revised = race.plannedCruise(true);
      if (mode == BEAST && !noisy && abs(votes) >= 4)
      { assert(revised <= old && revised == denseCruise(race)); }
      else { assert(revised == old); }
    }
  }
}

void testReportedScalePlan()
{
  for (const int direction : {-1, 1})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.turnVotes = direction * 8;
    race.speedScaleQ8 = 820;
    race.trackWidthMm = 84;
    race.speedMmS = 1000;
    race.exitQuietMs = 40;
    for (unsigned bin = 20; bin <= 28; bin++) { race.curves[bin] = direction * 30; }
    race.distanceTwice = (20 * RaceSession::BIN_COUNTS - 2140) * 2;
    assert(race.plannedCruise(true) == 376 && denseCruise(race, 35) == 393);
    race.distanceTwice = 20 * RaceSession::BIN_COUNTS * 2;
    assert(race.plannedCruise(true) == 293 && denseCruise(race, 40) == 303);
    race.distanceTwice = 28 * RaceSession::BIN_COUNTS * 2;
    assert(race.plannedCruise(true) == 400 && race.launchCurveLimit == 0);
    race.distanceTwice = 29 * RaceSession::BIN_COUNTS * 2;
    assert(race.plannedCruise(true) == 400 && race.launchCurveLimit == 32);
  }
}





void testDriveModelCache()
{
  RaceSession race;
  replayMap(race, BEAST);
  race.curves[20] = race.curves[21] = 30;
  race.speedMmS = 1000;
  race.distanceTwice = 600;
  for (const DriveMode mode : {BEAST, SAFE, MID, BEAST})
  for (const bool noisy : {false, true, false})
  for (const int votes : {8, 3, -8, 0, 8})
  for (const uint16_t scale : {256, 820, 842, 1024, 1536})
  for (const uint8_t width : {60, 84, 110})
  {
    race.mode = mode;
    race.noisyLearning = noisy;
    race.turnVotes = votes;
    race.speedScaleQ8 = scale;
    race.trackWidthMm = width;
    const uint16_t expectedYaw = 128000UL * width / scale;
    assert(race.yawScale() == expectedYaw);
    const int16_t expectedPlan = denseCruise(race);
    for (unsigned repeat = 0; repeat < 2; repeat++)
    {
      assert(race.plannedCruise(true) == expectedPlan);
      assert(race.yawScale() == expectedYaw);
    }
    // Kinematic inputs remain live while the fixed drivetrain values are cached.
    race.distanceTwice += 2;
    assert(race.plannedCruise(true) == denseCruise(race));
  }
  race.reset(BEAST, 100);
  assert(race.yawScale() == 128000UL * 86 / 1024);
  assert(race.plannedCruise(true) == 80);
  race.mapReady = true;
  race.learnedCounts = 20000;
  race.binsUsed = 157;
  memset(race.curves, 0, sizeof(race.curves));
  race.curves[10] = 30;
  assert(race.plannedCruise(true) == denseCruise(race));
}

void testBrakePrecision()
{
  for (const uint16_t scale : {256, 820, 842, 1024, 1536})
  for (const int direction : {-1, 1})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.turnVotes = direction * 8;
    race.speedScaleQ8 = scale;
    race.speedMmS = 1000;
    for (unsigned bin = 30; bin < 40; bin++) { race.curves[bin] = direction * 30; }
    const uint16_t modelScale = scale + scale / 8;
    const uint32_t lateral = 5600UL * 65536 / (uint32_t(modelScale) * modelScale);
    const uint32_t corner = race.cornerSquared(30, lateral);
    const uint32_t guard = 251 + (3 * uint32_t(1000 * 0.04f / RaceSession::MM_PER_COUNT) + 1) / 2 + 256;
    for (unsigned distance = 0; distance <= 30 * RaceSession::BIN_COUNTS; distance += 19)
    {
      race.distanceTwice = distance * 2;
      const int16_t target = race.plannedCruise(true);
      assert(target == denseCruise(race) && target <= denseCruise(race, 35));
      const uint32_t available = 30 * RaceSession::BIN_COUNTS - distance;
      const uint32_t clear = available > guard ? available - guard : 0;
      // Never exceed the 250 PWM/s braking envelope, including fractional rounding.
      assert(uint32_t(target) * target <= corner + uint64_t(clear) * 35840 / modelScale);
    }
    race.distanceTwice = 30 * RaceSession::BIN_COUNTS * 2;
    assert(race.plannedCruise(true) <= denseCruise(race, 40));
  }
}

void testMappedCornerExit()
{
  for (const int direction : {-1, 1})
  for (const unsigned bin : {0u, 27u, 156u, 511u})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.turnVotes = direction * 8;
    race.speedScaleQ8 = 820;
    race.trackWidthMm = 84;
    race.speedMmS = 1000;
    race.learnedCounts = bin == 511 ? 65535 : 20000;
    race.binsUsed = (race.learnedCounts - 1) / RaceSession::BIN_COUNTS + 1;
    const unsigned previous = bin ? bin - 1 : race.binsUsed - 1;
    const unsigned next = (bin + 1) % race.binsUsed;
    race.curves[previous] = race.curves[bin] = direction * 30;
    const auto withoutExit = [&]() { return denseCruise(race, 41, nullptr, false); };
    for (const unsigned progress : {0u, 16u, 31u, 32u, 126u})
    {
      if (bin * RaceSession::BIN_COUNTS + progress >= race.learnedCounts) { continue; }
      race.distanceTwice = (bin * RaceSession::BIN_COUNTS + progress) * 2;
      for (const unsigned quiet : {0u, 19u, 20u, 40u})
      for (const int live : {0, 8, 16, 30, 45})
      {
        race.exitQuietMs = quiet;
        race.filteredCurve = direction * live * 8;
        const int16_t held = withoutExit();
        const int16_t revised = race.plannedCruise(true);
        assert(revised == denseCruise(race) && revised >= held && revised <= 400);
        assert(race.launchCurveLimit == 0);
        if (quiet < 20) { assert(revised == held); }
        if (quiet >= 20 && live == 0) { assert(revised == 400 && held == 293); }
        if (live >= 30) { assert(revised <= 293); } // Live turning still caps a released map bin.
        assert(race.plannedCruise(false) == held && race.launchCurveLimit == 0);
      }
    }
    race.exitQuietMs = 20;
    race.filteredCurve = 0;
    race.distanceTwice = bin * RaceSession::BIN_COUNTS * 2;
    const unsigned future = (bin + 2) % race.binsUsed;
    race.curves[future] = direction * 30;
    assert(race.plannedCruise(true) == withoutExit()); // Next corner keeps its entry guard.
    race.curves[future] = 0;
    for (const int prior : {0, 3, -direction * 30, -128})
    {
      race.curves[previous] = prior;
      assert(race.plannedCruise(true) == withoutExit() && race.launchCurveLimit == 0);
    }
    race.curves[previous] = direction * 30;
    race.curves[next] = RaceSession::UNKNOWN_CURVE;
    assert(race.plannedCruise(true) == withoutExit() && race.launchCurveLimit == 0);
    race.curves[next] = 0;
    race.curves[bin] = RaceSession::UNKNOWN_CURVE;
    assert(race.plannedCruise(true) == 65 && !race.liveLimited);
    race.curves[bin] = direction * 30;
    for (const DriveMode mode : {SAFE, MID, BEAST})
    {
      race.mode = mode;
      race.noisyLearning = mode == BEAST;
      assert(race.plannedCruise(true) == withoutExit() && race.launchCurveLimit == 0);
    }
    race.noisyLearning = false;
    race.turnVotes = 3;
    assert(race.plannedCruise(true) == withoutExit());
    race.liveLimited = true;
    race.launchCurveLimit = 96;
    race.mapReady = false;
    assert(race.plannedCruise(true) == 80 && !race.liveLimited && race.launchCurveLimit == 0);
    race.reset(BEAST, 0);
    assert(race.launchCurveLimit == 0);
  }
  RaceSession full;
  replayMap(full, BEAST);
  full.turnVotes = 8;
  full.learnedCounts = 65536;
  full.binsUsed = 512;
  full.curves[510] = full.curves[511] = 30;
  full.curves[2] = 30;
  for (const uint32_t distance : {65408u, 65535u, 65536u, 65537u, 131071u})
  {
    full.distanceTwice = distance * 2;
    assert(full.mapPosition() == distance % full.learnedCounts);
    assert(full.plannedCruise(true) == denseCruise(full));
  }
}

void testLearningNoiseRejection()
{
  RaceSession race;
  race.reset(BEAST, 0);
  for (unsigned spike = 0; spike < 100; spike++)
  {
    race.noteTracking(0, spike % 2 ? -300 : 300, false);
    assert(!race.noisyLearning);
    race.noteTracking(0, 0, false);
    assert(race.learningNoise == 0 && !race.noisyLearning);
  }
  for (unsigned burst = 0; burst < 5; burst++)
  {
    for (unsigned sample = 0; sample < 7; sample++)
    { race.noteTracking(0, 150, false); assert(!race.noisyLearning); }
    for (unsigned quiet = 0; quiet < 7; quiet++) { race.noteTracking(0, 0, false); }
    assert(race.learningNoise == 0 && !race.noisyLearning);
  }
  for (unsigned sample = 0; sample < 1000; sample++) { race.noteTracking(0, -150, false); }
  assert(race.noisyLearning && race.learningNoise == 8); // Saturates instead of wrapping.
  for (unsigned quiet = 0; quiet < 100; quiet++) { race.noteTracking(0, 0, false); }
  assert(race.noisyLearning && race.learningNoise == 0); // Genuine fallback remains latched.

  race.reset(BEAST, 0);
  for (unsigned sample = 0; sample < 7; sample++) { race.noteTracking(0, 150, false); }
  race.mapReady = true;
  for (unsigned sample = 0; sample < 20; sample++) { race.noteTracking(0, 2000, true); }
  assert(!race.noisyLearning && race.learningNoise == 7);
  race.reset(BEAST, 0);
  race.abort();
  for (unsigned sample = 0; sample < 20; sample++) { race.noteTracking(0, 2000, true); }
  assert(!race.noisyLearning && race.learningNoise == 0);
}

void testExitQualification()
{
  const float step = 20 * RaceSession::MM_PER_COUNT;
  for (const int direction : {-1, 1})
  {
    for (const uint16_t period : {1, 5, 7, 8, 10, 20, 40, 50})
    {
      RaceSession race;
      replayMap(race, BEAST);
      uint32_t now = 0;
      for (unsigned frame = 1; frame <= 64; frame++)
      {
        now += period;
        race.update(20, 20, direction * 31.5f * step / 48000, period, 100, now);
        assert(race.fault == RaceSession::NONE);
        assert((race.exitQuietMs >= 40) == (now >= 40));
        assert(race.exitQuietMs <= 89);
      }
      // Raw yaw crosses the quiet threshold while the filtered yaw still lags.
      race.distanceTwice = 100 * RaceSession::BIN_COUNTS * 2;
      race.turnVotes = direction * 8;
      race.curves[99] = race.curves[100] = direction * 30;
      now += 10;
      race.update(20, 20, direction * 32.5f * step / 48000, 10, 100, now);
      assert(race.exitQuietMs == 0 && abs(race.filteredCurve) < 96);
      race.plannedCruise(true);
      assert(race.launchCurveLimit == 0);
      for (unsigned quiet = 1; quiet <= 4; quiet++)
      {
        now += 10;
        race.update(20, 20, 0, 10, 100, now);
        race.plannedCruise(true);
        assert(race.exitQuietMs == std::min(quiet * 10, 40u));
        assert(race.launchCurveLimit == 0); // A trailing curved bin cannot launch.
      }
      race.curves[100] = 0;
      race.plannedCruise(true);
      assert(race.launchCurveLimit == 32); // Only the mapped straight may qualify.
      race.curves[101] = direction * 30;
      race.plannedCruise(true);
      assert(race.launchCurveLimit == 0); // Disable the launch before another bend.
      now += 10;
      race.update(0, 0, 0, 10, 100, now);
      assert(race.exitQuietMs == 0); // Stationary readings cannot qualify an exit.
      race.reset(BEAST, 0);
      assert(race.exitQuietMs == 0);
      race.exitQuietMs = 20;
      race.update(20, 20, 0, 10, 80, 10);
      assert(race.exitQuietMs == 0); // Learning cannot prime the replay boost.
    }
  }
  for (const int16_t left : {-2, 0, 500})
  {
    for (const int16_t right : {-2, 0, 500})
    {
      RaceSession race;
      race.reset(BEAST, 0);
      race.update(left, right, 0, 10, 80, 10);
      assert(race.fault == RaceSession::NONE);
      assert(race.distanceTwice == static_cast<uint32_t>(std::max(0, int(left) + right)));
    }
  }
}

void testSpatialMap()
{
  for (const int direction : {-1, 1})
  {
    RaceSession race;
    race.reset(BEAST, 0);
    const float step = 64 * RaceSession::MM_PER_COUNT;
    race.update(64, 64, direction * step * 72 / 6000, 10, 80, 10);
    race.update(64, 64, -direction * step * 72 / 6000, 10, 80, 20);
    assert(race.fault == RaceSession::NONE && race.binsUsed == 1);
    assert(abs(race.curves[0]) <= 1); // Equal/opposite yaw has zero net bend.
    assert(race.mapWeight == 0);

    // A reading spanning a boundary contributes only its distance in each bin.
    race.reset(BEAST, 0);
    const float longStep = 96 * RaceSession::MM_PER_COUNT;
    race.update(96, 96, direction * longStep * 16 / 6000, 10, 80, 10);
    race.update(96, 96, direction * longStep * 48 / 6000, 10, 80, 20);
    assert(race.fault == RaceSession::NONE && race.binsUsed == 2);
    assert(abs(int(race.curves[0]) - direction * 24) <= 1);
    assert(abs(int(race.curves[1]) - direction * 48) <= 1);
    assert(race.mapWeight == 128);

    // A real short corner must survive spatial averaging.
    race.reset(BEAST, 0);
    race.update(128, 128, direction * 128 * RaceSession::MM_PER_COUNT * 80 / 6000,
      10, 80, 10);
    assert(race.fault == RaceSession::NONE && abs(int(race.curves[0]) - direction * 80) <= 1);

    for (const int counts : {1, 2, 4, 8, 16, 32, 64, 128})
    {
      race.reset(BEAST, 0);
      for (int total = counts; total <= 256; total += counts)
      {
        const float yaw = direction * counts * RaceSession::MM_PER_COUNT * 30 / 6000;
        race.update(counts, counts, yaw, 10, 80, total * 10);
        assert(race.fault == RaceSession::NONE && race.mapWeight <= 256);
      }
      assert(race.binsUsed == 2);
      for (unsigned bin = 0; bin < 2; bin++)
      { assert(abs(int(race.curves[bin]) - direction * 30) <= 1); }
    }
    race.reset(BEAST, 0);
    assert(race.mapWeight == 0 && race.mapCurveSum == 0);
    race.update(0, 0, 0, 10, 0, 10);
    assert(race.fault == RaceSession::NONE && race.binsUsed == 0);
  }
}

void testCornerExitAssist()
{
  RaceSession race;
  for (const int polarity : {-1, 1})
  {
    for (const int direction : {-1, 1})
    {
      for (const int width : {60, 86, 110})
      {
        for (const int base : {100, 250, 400})
        {
          for (const int feedback : {-100, -79, 0, 59, 60, 100, 180})
          {
            int previous = 0;
            for (const int live : {0, 1, 7, 8, 12, 16, 30, 80, 127})
            {
              replayMap(race, BEAST);
              race.turnVotes = polarity * 8;
              race.trackWidthMm = width;
              race.distanceTwice = 2 * RaceSession::BIN_COUNTS;
              race.curves[0] = direction * 40;
              race.filteredCurve = direction * live * 8;
              const int error = polarity * direction * feedback;
              // Mode does not enter the original assist math; Mid bypasses
              // only the new Beast exit cap, providing the unchanged reference.
              race.mode = MID;
              const int original = race.steeringFeedforward(base, error);
              race.mode = BEAST;
              const int revised = race.steeringFeedforward(base, error);
              assert(abs(revised) <= abs(original) && revised * original >= 0);
              assert(abs(revised) <= base * long(width) * live / 12000);
              assert(abs(revised) >= previous);
              if (live < 8) { assert(revised == 0); }
              previous = abs(revised);
              for (const unsigned curvedBin : {1u, 2u})
              {
                race.curves[curvedBin] = direction * 40;
                race.mode = MID;
                const int entry = race.steeringFeedforward(base, error);
                race.mode = BEAST;
                assert(race.steeringFeedforward(base, error) == entry);
                race.curves[curvedBin] = 0;
              }
              race.curves[0] = RaceSession::UNKNOWN_CURVE;
              assert(race.steeringFeedforward(base, error) == 0);
            }
          }
        }
      }
    }
  }
}

void testMapMismatchPhaseTolerance()
{
  const float step = 2 * RaceSession::MM_PER_COUNT;
  for (const int direction : {-1, 1})
  for (const int start : {0, 10, 31})
  for (int offset = -3; offset <= 3; offset++)
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.binsUsed = 32;
    race.learnedCounts = 32 * RaceSession::BIN_COUNTS;
    race.distanceTwice = start * RaceSession::BIN_COUNTS * 2;
    race.curves[(start + offset + 32) % 32] = direction * 30;
    race.filteredCurve = direction * 240;
    for (unsigned now = 5; now <= 300 && race.fault == RaceSession::NONE; now += 5)
    { race.update(2, 2, direction * step * 30 / 6000, 5, 100, now); }
    if (abs(offset) <= 2)
    { assert(race.fault == RaceSession::NONE && race.mismatchMs == 0); }
    else { assert(race.fault == RaceSession::MAP_MISMATCH && race.mismatchMs == 200); }
  }
  for (const int direction : {-1, 1})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.curves[2] = direction * 30;
    race.filteredCurve = direction * 90 * 8;
    for (unsigned now = 5; now <= 200; now += 5)
    {
      race.update(2, 2, direction * step * 90 / 6000, 5, 100, now);
      assert(race.fault == (now < 200 ? RaceSession::NONE : RaceSession::MAP_MISMATCH));
    }
  }
  for (const unsigned bins : {1u, 2u, 3u, 4u, 5u})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.binsUsed = bins;
    race.learnedCounts = bins * RaceSession::BIN_COUNTS;
    race.curves[bins - 1] = 30;
    race.filteredCurve = 240;
    for (unsigned now = 5; now <= 300; now += 5)
    { race.update(2, 2, step * 30 / 6000, 5, 100, now); }
    assert(race.fault == RaceSession::NONE && race.mismatchMs == 0);
  }
  RaceSession race;
  replayMap(race, BEAST);
  race.filteredCurve = 240;
  for (unsigned now = 5; now <= 100; now += 5)
  { race.update(2, 2, step * 30 / 6000, 5, 100, now); }
  assert(race.fault == RaceSession::NONE && race.mismatchMs == 100);
  race.curves[2] = 30;
  race.update(2, 2, step * 30 / 6000, 5, 100, 105);
  assert(race.fault == RaceSession::NONE && race.mismatchMs == 0);
}

void testSignedMismatchAndTimerBounds()
{
  for (const int sign : {-1, 1})
  for (const unsigned shift : {0u, 1u, 2u})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.mapShift = shift;
    race.learnedCounts = uint32_t(race.binsUsed) * race.mapBinCounts();
    memset(race.curves, sign * 30, sizeof(race.curves));
    race.filteredCurve = -sign * 240;
    const float turn = -sign * 2 * RaceSession::MM_PER_COUNT * 30 / 6000;
    for (unsigned now = 5; now <= 200; now += 5)
    {
      race.update(2, 2, turn, 5, 80, now);
      if (shift)
      { assert(race.fault == RaceSession::NONE && race.mismatchMs == 0); }
      else
      {
        assert(race.mismatchMs == now);
        assert(race.fault == (now < 200 ? RaceSession::NONE : RaceSession::MAP_MISMATCH));
      }
    }
    replayMap(race, BEAST);
    race.curves[0] = sign * 30;
    race.curves[1] = -sign * 30;
    race.filteredCurve = -sign * 240;
    for (unsigned now = 5; now <= 300; now += 5)
    { race.update(2, 2, turn, 5, 80, now); }
    assert(race.fault == RaceSession::NONE && race.mismatchMs == 0);
  }
  RaceSession race;
  replayMap(race, BEAST);
  race.filteredCurve = 240;
  unsigned now = 0;
  for (const uint16_t dt : {49, 49, 49, 49, 50})
  {
    now += dt;
    race.update(100, 100, 100 * RaceSession::MM_PER_COUNT * 30 / 6000, dt, 80, now);
    assert(race.mismatchMs == now);
    assert(race.fault == (now < 200 ? RaceSession::NONE : RaceSession::MAP_MISMATCH));
  }
  assert(race.mismatchMs == 246); // Largest validated additions cannot wrap the byte timer.
  for (int votes = -128; votes <= 127; votes++)
  {
    race.turnVotes = votes;
    assert(race.trustedGeometry() == (abs(votes) >= 4));
  }
}

void testMovingSBendAgreement()
{
  unsigned cases = 0, maxPending = 0;
  for (const unsigned shift : {0u, 1u, 2u})
  for (const int direction : {-1, 1})
  for (int phase = -2; phase <= 2; phase++)
  for (const unsigned speed : {600u, 1400u})
  for (const bool jitter : {false, true})
  {
    RaceSession race;
    replayMap(race, BEAST);
    race.mapShift = shift;
    race.binsUsed = 257;
    const unsigned width = race.mapBinCounts();
    race.learnedCounts = uint32_t(race.binsUsed) * width - 17;
    for (unsigned bin = 0; bin < race.binsUsed; bin++)
    {
      const unsigned section = bin % 24;
      race.curves[bin] = section < 8 ? 0 : direction * (section < 16 ? 30 : -30);
    }
    const auto actualBend = [&](uint32_t point) -> int
    {
      int32_t shifted = int32_t(point % race.learnedCounts) + phase * int(width);
      if (shifted < 0) { shifted += race.learnedCounts; }
      if (shifted >= int32_t(race.learnedCounts)) { shifted -= race.learnedCounts; }
      return race.curves[unsigned(shifted) / width];
    };
    race.filteredCurve = actualBend(0) * 8;
    uint32_t now = 0;
    const unsigned periods[] = {5, 13, 7, 10};
    for (unsigned frame = 0; race.distanceCounts() < race.learnedCounts + 2 * width; frame++)
    {
      const unsigned dt = jitter ? periods[frame % 4] : 5;
      const unsigned counts = unsigned(std::lround(speed * dt / (1000 * RaceSession::MM_PER_COUNT)));
      const int bend = actualBend(race.distanceCounts() + counts / 2);
      const float turn = counts * RaceSession::MM_PER_COUNT * bend / 6000;
      // Agreement-only fixture: suppress pose closure, preserving distance wrap.
      race.x = race.y = 1000;
      race.heading = 0;
      race.leftStartZone = race.returnArmed = false;
      now += dt;
      assert(race.update(counts, counts, turn, dt, 100, now) == RaceSession::NO_EVENT);
      assert(race.fault == RaceSession::NONE);
      maxPending = std::max<unsigned>(maxPending, race.mismatchMs);
    }
    cases++;
  }
  std::cout << "Moving S-bend agreement: " << cases
    << " cases, max pending mismatch " << maxPending << " ms.\n";
}

int main(int argc, char ** argv)
{
  const bool focused = argc > 1 && strcmp(argv[1], "--focused") == 0;
  testPlannerGrouping();
  testSprintEnvelope();
  testCornerEntryBraking();
  testBoundedBeastPace();
  testReportedScalePlan();
  testDriveModelCache();
  testBrakePrecision();
  testMappedCornerExit();
  testExitQualification();
  testLearningNoiseRejection();
  testSpatialMap();
  testCornerExitAssist();
  testMapMismatchPhaseTolerance();
  testSignedMismatchAndTimerBounds();
  testMovingSBendAgreement();
  for (const auto mode : {SAFE, MID, BEAST})
  {
    RaceSession race;
    race.reset(mode, 0);
    assert(race.profile().limit == 100 && race.plannedCruise(true) == 80);
    if (!focused)
    {
      testCircle(mode, 1, 0);
      testCircle(mode, -1, std::numeric_limits<uint32_t>::max() - 1000);
    }
  }
  assert(driveProfile(SAFE, false).cruise == 120 && driveProfile(SAFE, false).limit == 140);
  assert(driveProfile(MID, false).cruise == 180 && driveProfile(MID, false).limit == 220);
  assert(driveProfile(BEAST, false).cruise == 400 && driveProfile(BEAST, false).limit == 400);
  assert(driveProfile(static_cast<DriveMode>(99), false).limit == 140);

  // Tight and broad corners remain distinguishable, with different mode budgets.
  int16_t tightPace[3] = {};
  for (const auto mode : {SAFE, MID, BEAST})
  {
    RaceSession curved;
    replayMap(curved, mode);
    memset(curved.curves, 80, sizeof(curved.curves)); // 75 mm radius.
    tightPace[mode] = curved.plannedCruise(true);
    assert(tightPace[mode] >= 65);
    memset(curved.curves, 40, sizeof(curved.curves)); // 150 mm radius.
    const int16_t mediumPace = curved.plannedCruise(true);
    memset(curved.curves, 20, sizeof(curved.curves)); // 300 mm radius.
    assert(tightPace[mode] < mediumPace && mediumPace < curved.plannedCruise(true));
  }
  assert(tightPace[SAFE] < tightPace[MID] && tightPace[MID] < tightPace[BEAST]);
  for (const uint16_t scale : {256, 640, 1024, 1536})
  {
    int16_t previousMode[128] = {};
    for (const auto mode : {SAFE, MID, BEAST})
    {
      RaceSession envelope;
      replayMap(envelope, mode);
      envelope.speedScaleQ8 = scale;
      int16_t previousCorner = envelope.profile().cruise;
      for (uint8_t curvature = 1; curvature <= 127; curvature++)
      {
        memset(envelope.curves, curvature, sizeof(envelope.curves));
        const int16_t pace = envelope.plannedCruise(true);
        assert(pace >= 65 && pace <= previousCorner);
        assert(pace >= previousMode[curvature]);
        previousCorner = pace;
        previousMode[curvature] = pace;
      }
    }
  }

  // Rounded square: each side has a straight followed by a curved section.
  for (const auto mode : {SAFE, MID, BEAST})
  {
    RaceSession square;
    replayMap(square, mode);
    square.binsUsed = 400;
    square.learnedCounts = 400 * RaceSession::BIN_COUNTS;
    square.speedMmS = 450;
    for (uint16_t bin = 0; bin < square.binsUsed; bin++)
    {
      square.curves[bin] = bin % 100 < 80 ? 0 : 80;
    }
    assert(square.plannedCruise(true) == driveProfile(mode, false).cruise);
    square.distanceTwice = 79 * RaceSession::BIN_COUNTS * 2;
    const int16_t approach = square.plannedCruise(true);
    assert(approach >= 65 && approach < driveProfile(mode, false).cruise);
    square.distanceTwice = 80 * RaceSession::BIN_COUNTS * 2;
    const int16_t corner = square.plannedCruise(true);
    assert(corner >= 65 && corner <= approach);
    square.completed = 2;
    assert(square.plannedCruise(true) == corner); // No extra lap-3 penalty away from the finish.
  }

  RaceSession race;
  replayMap(race, BEAST);
  assert(race.plannedCruise(true) == 400);
  race.speedMmS = 1000;
  race.curves[6] = 127;
  const int16_t approaching = race.plannedCruise(true);
  assert(approaching > 65 && approaching < 400); // Gradual braking before a bend.
  race.distanceTwice = 6 * RaceSession::BIN_COUNTS * 2;
  const int16_t tightCorner = race.plannedCruise(true);
  assert(tightCorner >= 65 && tightCorner < approaching);
  race.distanceTwice = 0;
  race.curves[6] = 0;
  race.curves[100] = 127;
  assert(race.plannedCruise(true) == 400);
  // Replay policy has no lap-3 penalty or permanent tracking-error veto.
  for (const auto mode : {SAFE, MID, BEAST})
  {
    replayMap(race, mode);
    for (uint16_t bin = 0; bin < race.binsUsed; bin++) { race.curves[bin] = (int(bin % 3) - 1) * 40; }
    for (uint32_t distance = 0; distance < race.learnedCounts * 115 / 100; distance += 73)
    {
      race.distanceTwice = distance * 2;
      race.speedMmS = distance % 1500;
      race.completed = 1;
      const int16_t lapTwo = race.plannedCruise(true);
      race.completed = 2;
      assert(race.plannedCruise(true) == lapTwo);
    }
  }

  // Do not skip bin zero when the learned length ends partway through a bin.
  replayMap(race, BEAST);
  race.learnedCounts = 200 * RaceSession::BIN_COUNTS + 20;
  race.binsUsed = 201;
  race.distanceTwice = (race.learnedCounts - 10) * 2;
  race.curves[0] = 127;
  assert(race.plannedCruise(true) >= 65 && race.plannedCruise(true) <= tightCorner);

  race.speedMmS = 500;
  race.resetLap(10);
  assert(race.speedMmS == 500); // Continuous laps retain the measured moving speed.
  race.reset(SAFE, 20);
  assert(race.speedMmS == 0); // A fresh session starts stationary.

  // Alternating yaw corrections on a straight must not paint a maximum-risk
  // corner into the map; a sustained turn still must be recognized.
  RaceSession jitter, bend;
  jitter.reset(BEAST, 0);
  bend.reset(BEAST, 0);
  for (uint16_t frame = 1; frame <= 120; frame++)
  {
    jitter.update(10, 10, frame % 2 ? 0.03f : -0.03f, 10, 100, frame * 10);
    bend.update(10, 10, 0.03f, 10, 100, frame * 10);
  }
  assert(jitter.fault == RaceSession::NONE && bend.fault == RaceSession::NONE);
  for (uint16_t bin = 0; bin < jitter.binsUsed; bin++) { assert(abs(jitter.curves[bin]) <= 18); }
  assert(bend.curves[bend.binsUsed - 1] >= 60);

  // Learn the speed/PWM ratio from physical encoder motion instead of assuming
  // every battery, motor and floor produces the same speed at a given PWM.
  race.reset(BEAST, 0);
  for (uint16_t frame = 1; frame <= 100; frame++)
  { race.update(10, 10, 0, 10, 100, frame * 10); }
  const int expectedScale = static_cast<int>(10 * RaceSession::MM_PER_COUNT * 100 * 256 / 100);
  assert(abs(int(race.speedScaleQ8) - expectedScale) < 12);

  replayMap(race, BEAST);
  race.curves[10] = 127;
  race.speedScaleQ8 = 640;
  const int16_t slowDrivetrain = race.plannedCruise(true);
  race.speedScaleQ8 = 1280;
  assert(race.plannedCruise(true) < slowDrivetrain); // Faster hardware brakes sooner.

  // Gyro convention is learned, so reversed sensor axes still yield the same
  // physical wheel assistance. Weak yaw needs optical confirmation of entry.
  for (const int polarity : {-1, 1})
  {
    race.reset(BEAST, 0);
    for (uint16_t frame = 1; frame <= 120; frame++)
    {
      const float yaw = polarity * (24 - 16) * RaceSession::MM_PER_COUNT / 86;
      race.update(24, 16, yaw, 10, 100, frame * 10);
    }
    assert(race.fault == RaceSession::NONE && race.trackWidthMm >= 84 && race.trackWidthMm <= 88);
    assert(race.turnVotes * polarity >= 4);
    race.mapReady = true;
    race.learnedCounts = 20000;
    race.binsUsed = race.learnedCounts / RaceSession::BIN_COUNTS + 1;
    memset(race.curves, polarity * 30, sizeof(race.curves));
    race.filteredCurve = polarity * 30 * 8;
    assert(race.steeringFeedforward(100, 0) >= 20 && race.steeringFeedforward(100, 0) <= 22);
    assert(race.steeringFeedforward(100, -81) == 0); // Current feedback vetoes stale turns.
    race.filteredCurve = -polarity * 30 * 8;
    assert(race.steeringFeedforward(100, 0) == 0);
    race.filteredCurve = 0;
    assert(race.steeringFeedforward(100, 0) == 0);
    race.filteredCurve = polarity * 30 * 8;
    memset(race.curves, 128, sizeof(race.curves));
    assert(race.steeringFeedforward(100, 0) == 0);
    assert(race.plannedCruise(true) == 65);
  }

  // Small but consistent turn samples must calibrate geometry at learning pace.
  for (const int polarity : {-1, 1})
  {
    race.reset(BEAST, 0);
    float leftFraction = 0, rightFraction = 0;
    for (uint16_t frame = 1; frame <= 180; frame++)
    {
      const float turn = 0.004f; // Every sample is below the old 0.01 rad gate.
      leftFraction += (2.0f + 43 * turn) / RaceSession::MM_PER_COUNT;
      rightFraction += (2.0f - 43 * turn) / RaceSession::MM_PER_COUNT;
      const int16_t left = leftFraction, right = rightFraction;
      leftFraction -= left;
      rightFraction -= right;
      race.update(left, right, polarity * turn, 10, 80, frame * 10);
      assert(race.fault == RaceSession::NONE && race.learningSamples < 20);
    }
    assert(race.turnVotes * polarity >= 4);
    assert(abs(int(race.trackWidthMm) - 86) <= 4);
    race.reset(BEAST, 0);
    assert(race.learningSamples == 0 && race.learningDifference == 0 && race.learningTurn == 0);
  }

  for (const int polarity : {-1, 1})
  {
    for (const int direction : {-1, 1})
    {
      replayMap(race, BEAST);
      race.turnVotes = polarity * 8;
      const int sign = polarity * direction;
      memset(race.curves, direction * 30, sizeof(race.curves));
      for (const int error : {0, 59, 60, 61, 99, 100, 101})
      {
        race.filteredCurve = 0;
        const int16_t assist = race.steeringFeedforward(100, sign * error);
        assert(error < 60 ? assist == 0 : assist * sign > 0);
        assert(race.steeringFeedforward(100, -sign * error) == 0);
      }
      memset(race.curves, 0, sizeof(race.curves));
      assert(race.steeringFeedforward(100, sign * 2000) == 0);
      memset(race.curves, 128, sizeof(race.curves));
      assert(race.steeringFeedforward(100, sign * 2000) == 0);
      memset(race.curves, direction * 30, sizeof(race.curves));
      race.turnVotes = polarity * 3;
      assert(race.steeringFeedforward(100, sign * 2000) == 0);
    }
  }

  for (const int polarity : {-1, 1})
  {
    for (const int direction : {-1, 1})
    {
      replayMap(race, BEAST);
      race.turnVotes = polarity * 8;
      race.trackWidthMm = 110;
      memset(race.curves, direction * 127, sizeof(race.curves));
      const int sign = polarity * direction;
      for (const int confidence : {63, 127, 128, 129})
      {
        const int bend = confidence < 128 ? 127 * (confidence - 48) / 80 : 127;
        const int expected = 400L * 110 * bend / 12000;
        assert(race.steeringFeedforward(400, sign * confidence) == sign * expected);
      }
      race.filteredCurve = -direction * 8 * 8;
      assert(race.steeringFeedforward(400, sign * 1000) == 0);
    }
  }

  race.reset(BEAST, 0);
  race.binsUsed = 1;
  race.curves[0] = 0;
  race.filteredCurve = 1016;
  race.learningDifference = 10000;
  race.learningTurn = 0.039f;
  race.learningSamples = 19;
  race.update(500, -2, 0.002f, 10, 80, 10); // Candidate width exceeds uint16_t.
  assert(race.fault == RaceSession::NONE && race.trackWidthMm == 86 && race.turnVotes == 0);
  assert(race.learningSamples == 0 && race.learningDifference == 0 && race.learningTurn == 0);

  race.reset(BEAST, 0);
  for (uint16_t frame = 1; frame <= 10; frame++)
  { race.update(2, 2, 0.001f, 10, 80, frame * 10); }
  race.learningSamples = 0;
  race.learningDifference = 0;
  race.learningTurn = 0;
  for (uint16_t frame = 11; frame <= 30; frame++)
  { race.update(2, 2, 0.001f, 10, 80, frame * 10); }
  assert(race.fault == RaceSession::NONE && race.turnVotes == 0);
  assert(race.learningSamples == 0 && race.learningDifference == 0 && race.learningTurn == 0);

  // Noisy learning selects a fixed conservative envelope before replay begins.
  for (const auto mode : {SAFE, MID, BEAST})
  {
    for (const int delta : {-151, -150, -149, 149, 150, 151})
    {
      race.reset(mode, 0);
      const bool noisy = mode == BEAST && abs(delta) >= 150;
      for (unsigned sample = 1; sample <= 8; sample++)
      {
        race.noteTracking(0, delta, false);
        assert(race.noisyLearning == (noisy && sample == 8));
      }
      race.noteTracking(0, 0, false);
      assert(race.noisyLearning == noisy);
      race.mapReady = true;
      race.completed = 1;
      race.learnedCounts = 20000;
      race.binsUsed = race.learnedCounts / RaceSession::BIN_COUNTS + 1;
      memset(race.curves, 0, sizeof(race.curves));
      race.turnVotes = 8;
      const DriveProfile expected = driveProfile(noisy ? MID : mode, false);
      assert(race.profile().cruise == expected.cruise && race.profile().limit == expected.limit);
      if (noisy) { assert(race.steeringLimit() == 110 && race.plannedCruise(true) == 180); }
      finishReplay(race);
      race.reset(mode, 0);
      assert(!race.noisyLearning && race.learningNoise == 0);
    }
  }
  replayMap(race, BEAST);
  race.noteTracking(0, 200, false);
  assert(!race.noisyLearning); // Never lower the motor ceiling mid-replay.
  for (const int sign : {-1, 1})
  {
    replayMap(race, BEAST);
    race.noisyLearning = true;
    race.turnVotes = 8;
    memset(race.curves, sign * 40, sizeof(race.curves));
    assert(race.steeringFeedforward(180, sign * 100) == 0);
    race.filteredCurve = sign * 16 * 8;
    assert(race.steeringFeedforward(180, 0) == sign * (180L * 86 * 16 / 12000));
  }

  // Strong yaw damping requires confirmed current/next straights and live agreement.
  replayMap(race, BEAST);
  uint8_t yawWeight = 99;
  race.steeringFeedforward(400, 0, &yawWeight);
  assert(yawWeight == 0); // Gyro polarity has not been learned.
  race.turnVotes = 4;
  race.steeringFeedforward(400, 0, &yawWeight);
  assert(yawWeight == 2);
  for (const int curvature : {-128, -4, 4, 127})
  {
    race.curves[1] = curvature;
    race.steeringFeedforward(400, 0, &yawWeight);
    assert(yawWeight == 1); // Weaken ahead of a bend or an unknown bin.
  }
  race.curves[1] = 0;
  for (const int live : {-64, -63, 63, 64})
  {
    race.filteredCurve = live;
    race.steeringFeedforward(400, 0, &yawWeight);
    assert(yawWeight == (abs(live) < 64 ? 2 : 1));
  }
  race.filteredCurve = 0;
  race.learnedCounts = 10 * RaceSession::BIN_COUNTS + 10;
  race.binsUsed = 11;
  race.distanceTwice = (race.learnedCounts - 1) * 2;
  race.curves[0] = -4;
  race.steeringFeedforward(400, 0, &yawWeight);
  assert(yawWeight == 1);
  race.curves[0] = 0;
  race.steeringFeedforward(400, 0, &yawWeight);
  assert(yawWeight == 2);
  race.learnedCounts = 0;
  race.steeringFeedforward(400, 0, &yawWeight);
  assert(yawWeight == 0);

  // Fade both turn directions continuously, including products wider than AVR int.
  for (const int polarity : {-1, 1})
  {
    for (const int direction : {-1, 1})
    {
      replayMap(race, BEAST);
      race.turnVotes = polarity * 8;
      race.trackWidthMm = 110;
      memset(race.curves, direction * 127, sizeof(race.curves));
      race.filteredCurve = direction * 127 * 8;
      const int sign = polarity * direction;
      const int full = 400L * 110 * 127 / 12000;
      for (int opposing = 0; opposing <= 80; opposing++)
      {
        assert(race.steeringFeedforward(400, -sign * opposing) ==
          sign * full * (80 - opposing) / 80);
      }
      assert(race.steeringFeedforward(400, sign * 2000) == sign * full);
      race.filteredCurve = direction * 7 * 8;
      assert(race.steeringFeedforward(400, 0) == 0);
      race.filteredCurve = direction * 8 * 8;
      assert(race.steeringFeedforward(400, 0) * sign > 0);
    }
  }

  replayMap(race, BEAST);
  for (uint8_t frame = 1; frame <= 30; frame++)
  {
    race.update(10, 10, 0.03f, 10, 100, frame * 10);
  }
  assert(race.fault == RaceSession::MAP_MISMATCH);

  replayMap(race, MID);
  race.distanceTwice = race.learnedCounts * 2 * 116 / 100;
  race.update(10, 10, 0, 10, 0, 10);
  assert(race.fault == RaceSession::NO_FINISH);

  replayMap(race, MID);
  race.distanceTwice = race.learnedCounts;
  race.x = -1;
  race.leftStartZone = race.returnArmed = true;
  assert(race.update(10, 10, 0, 10, 0, 10) == RaceSession::NO_EVENT);
  assert(race.completed == 1); // A geometrically early crossing is not another lap.

  race.reset(SAFE, 0);
  race.mapShift = RaceSession::MAX_MAP_SHIFT;
  race.binsUsed = RaceSession::MAP_CAPACITY;
  race.distanceTwice = (RaceSession::MAP_CAPACITY * uint32_t(race.mapBinCounts()) - 1) * 2;
  race.update(10, 10, 0, 10, 0, 10);
  assert(race.fault == RaceSession::MAP_FULL);

  race.reset(SAFE, 0);
  race.distanceTwice = 4096;
  race.update(0, 0, 0, 10, 0, 10);
  assert(race.fault == RaceSession::MAP_GAP);

  race.reset(SAFE, 0);
  race.update(0, 0, 0, 10, 0, RaceSession::LAP_TIMEOUT_MS);
  assert(race.fault == RaceSession::NO_FINISH);

  race.reset(SAFE, 0);
  race.update(0, 0, std::numeric_limits<float>::quiet_NaN(), 10, 0, 10);
  assert(race.fault == RaceSession::BAD_MOTION);

  race.reset(SAFE, 0);
  uint32_t now = 0;
  for (uint16_t frame = 0; frame < 400 && race.fault == RaceSession::NONE; frame++)
  {
    now += 10;
    const float step = 18 * RaceSession::MM_PER_COUNT;
    race.update(18, 18, step / 100, 10, 0, now);
  }
  assert(race.fault == RaceSession::NO_FINISH && race.completed == 0);

  race.reset(SAFE, 0);
  race.abort();
  race.update(10, 10, 0, 10, 0, 10);
  assert(!race.running && !race.mapReady && race.completed == 0);

  if (focused)
  {
    std::cout << "PASS: guarded R24 exits, dense planner bounds, wrap/boundary bins, geometry, mode limits, "
      "spatial mapping, sprint/fallback limits and faults; trajectory models skipped.\n";
    return 0;
  }
  std::cout << "PASS: rounded-square replay pace, gradual braking, all mode limits, "
    "CW/CCW three-lap estimates, rollover, "
    "map bounds/mismatch, filtered curvature, learned motor scale, wraparound lookahead, "
    "two-lap sprint policy, noisy-learning fallback, timeouts and completion latch.\n";
}
