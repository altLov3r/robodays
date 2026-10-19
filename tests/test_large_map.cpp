#include <assert.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include "../PololuLineFollower/RaceSession.h"

void record(RaceSession & race, uint16_t counts, int curvature = 0)
{
  // Storage fixture: suppress geometric closure, without imposing a lap result.
  race.x = race.y = 1000;
  race.heading = 0;
  race.leftStartZone = race.returnArmed = false;
  const float turn = counts * RaceSession::MM_PER_COUNT * curvature / 6000;
  assert(race.update(counts, counts, turn, 10, 80, 10) == RaceSession::NO_EVENT);
}

void finishLearning(RaceSession & race, uint16_t counts)
{
  race.x = -0.1f;
  race.y = race.heading = 0;
  race.leftStartZone = race.returnArmed = true;
  assert(race.update(counts, counts, 0, 10, 80, 20) == RaceSession::LAP_FINISHED);
  assert(race.fault == RaceSession::NONE && race.mapReady && race.completed == 1);
}

int densePooledPlan(const RaceSession & race)
{
  const unsigned width = 128u << race.mapShift;
  const unsigned scale = race.speedScaleQ8 + race.speedScaleQ8 / 8;
  const unsigned brakeQ8 = 35840UL * 256 / scale;
  const unsigned lateral = 5600UL * 65536 / (uint32_t(scale) * scale);
  unsigned guard = 125 + unsigned(race.speedMmS * 0.04f / RaceSession::MM_PER_COUNT);
  guard += guard / 2 + 64 + 256;
  uint32_t target = std::min<uint32_t>(160000,
    race.cornerSquared(abs(race.filteredCurve / 8), lateral));
  const unsigned horizon = (160000UL - 4225) * 256 / brakeQ8 + guard;
  for (unsigned ahead = 0; ahead <= horizon; ahead++)
  {
    const unsigned bin = ((race.distanceCounts() + ahead) % race.learnedCounts) / width;
    const int curve = race.curves[bin];
    uint32_t allowed = curve == RaceSession::UNKNOWN_CURVE ? 4225 :
      race.cornerSquared(abs(curve), lateral);
    if (ahead > guard) { allowed += uint32_t(ahead - guard) * brakeQ8 / 256; }
    target = std::min(target, allowed);
  }
  return unsigned(std::sqrt(double(target)));
}

void testCapacityAndPartialWrap()
{
  RaceSession race;
  race.reset(BEAST, 0);
  for (unsigned i = 0; i < 512; i++) { record(race, 128); }
  assert(race.fault == RaceSession::NONE && race.mapShift == 0 && race.binsUsed == 512);
  record(race, 1);
  assert(race.fault == RaceSession::NONE && race.mapShift == 1 && race.binsUsed == 257);
  assert(race.mapWeight == 2 && race.curves[256] == RaceSession::UNKNOWN_CURVE);
  finishLearning(race, 1);
  assert(race.learnedCounts == 65538 && race.binsUsed == 257 && race.curves[256] == 0);
  race.turnVotes = 8;
  race.trackWidthMm = 84;
  race.speedScaleQ8 = 820;
  race.speedMmS = 1000;
  race.curves[0] = 60;
  race.curves[255] = -30;
  for (const uint32_t distance : {0u, 64000u, 65407u, 65408u, 65535u, 65536u, 65537u, 65538u, 131075u})
  {
    race.distanceTwice = distance * 2;
    assert(race.mapPosition() == distance % race.learnedCounts);
    assert(race.plannedCruise(true) == densePooledPlan(race));
  }
  race.reset(BEAST, 0);
  assert(race.mapShift == 0 && race.mapBinCounts() == 128 && race.binsUsed == 0);
}

void testBothCompactionsAndLimit()
{
  RaceSession race;
  race.reset(BEAST, 0);
  for (unsigned bin = 0; bin < 2048; bin++)
  {
    record(race, 128, bin % 2 ? -80 : 20);
    assert(race.fault == RaceSession::NONE && race.binsUsed <= 512);
    const unsigned expectedShift = bin < 512 ? 0 : (bin < 1024 ? 1 : 2);
    assert(race.mapShift == expectedShift);
    if (expectedShift)
    {
      for (unsigned stored = 0; stored < race.binsUsed - 1u; stored++)
      { assert(race.curves[stored] <= -79 && race.curves[stored] >= -80); }
    }
  }
  assert(race.mapShift == 2 && race.mapBinCounts() == 512 && race.binsUsed == 512);
  assert(race.distanceCounts() == 262144 && sizeof(race.curves) == 512);
  record(race, 1);
  assert(race.fault == RaceSession::MAP_FULL && race.binsUsed == 512);
}

void testIrregularSampleBoundaries()
{
  for (const uint16_t sample : {1, 7, 31, 127, 128, 251, 500})
  {
    RaceSession race;
    race.reset(BEAST, 0);
    // Different chunks straddle both compaction boundaries and partial base bins.
    while (race.distanceCounts() < 140003)
    {
      record(race, std::min<uint32_t>(sample, 140003 - race.distanceCounts()));
      assert(race.fault == RaceSession::NONE && race.binsUsed <= 512);
    }
    finishLearning(race, 1);
    assert(race.mapShift == 2 && race.learnedCounts == 140004);
    assert(race.binsUsed == (race.learnedCounts + 511) / 512);
    for (unsigned bin = 0; bin < race.binsUsed; bin++) { assert(race.curves[bin] == 0); }
  }
}

void testPooledSteeringAndExit()
{
  for (const int polarity : {-1, 1})
  for (const int direction : {-1, 1})
  {
    RaceSession race;
    race.reset(BEAST, 0);
    race.mapShift = 1;
    race.mapReady = true;
    race.completed = 1;
    race.learnedCounts = 70001;
    race.binsUsed = (race.learnedCounts + 255) / 256;
    race.turnVotes = 8 * polarity;
    race.trackWidthMm = 84;
    race.speedScaleQ8 = 820;
    memset(race.curves, -80 * direction, sizeof(race.curves));
    race.filteredCurve = direction * 30 * 8;
    assert(race.steeringFeedforward(100, 0) * direction * polarity > 0);
    race.filteredCurve = 0;
    assert(race.steeringFeedforward(100, 200 * direction * polarity) == 0);
    memset(race.curves, 0, sizeof(race.curves));
    race.curves[race.binsUsed - 1] = race.curves[0] = direction * 30;
    race.exitQuietMs = 40;
    // A pooled bin is not evidence that the physical bend has already ended.
    assert(race.plannedCruise(true) == 293 && race.launchCurveLimit == 0);
  }
}

int main()
{
  testCapacityAndPartialWrap();
  testBothCompactionsAndLimit();
  testIrregularSampleBoundaries();
  testPooledSteeringAndExit();
  std::cout << "PASS: fixed 512-byte map, both capacity expansions, 32-bit/partial wrap, "
    "opposite-bend peaks, chunk boundaries, live-yaw steering and bounded full-map fault.\n";
}
