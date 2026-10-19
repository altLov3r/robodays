#include <assert.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
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
  for (const uint8_t shift : {0, 1, 2})
  for (const uint32_t point : {0u, 127u, 128u, 65535u, 65536u, 131071u, 131072u, 262144u})
  {
    race.mapShift = shift;
    assert(race.mapBin(point) == point / (128u << shift));
  }
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

void testUnequalWheelBoundaries()
{
  unsigned cases = 0;
  for (const unsigned chunk : {7u, 255u, 999u})
  for (const uint32_t lengthTwice : {131071u, 131073u, 262143u, 262145u, 524287u, 524288u})
  {
    RaceSession race;
    race.reset(BEAST, 0);
    std::vector<int32_t> sums(2048, 0);
    std::vector<unsigned> weights(2048, 0);
    uint32_t point = 0;
    const auto accumulate = [&](unsigned span, int bend)
    {
      // Independent half-count oracle: no production splitting or compaction.
      for (unsigned i = 0; i < span; i++, point++)
      {
        sums[point / 256] += bend * 8;
        weights[point / 256]++;
      }
    };
    for (unsigned frame = 0; point < lengthTwice - 1; frame++)
    {
      const unsigned span = std::min<uint32_t>(chunk, lengthTwice - 1 - point);
      const int bend = frame % 3 == 0 ? 24 : (frame % 3 == 1 ? -12 : 0);
      const float turn = span * 0.5f * RaceSession::MM_PER_COUNT * bend / 6000;
      // Storage-only fixture: hold pose and clock, independent of lap estimation.
      race.x = race.y = 1000;
      race.heading = 0;
      race.leftStartZone = race.returnArmed = false;
      assert(race.update((span + 1) / 2, span / 2, turn, 10, 80, 10) == RaceSession::NO_EVENT);
      assert(race.fault == RaceSession::NONE);
      accumulate(span, bend);
      assert(race.distanceTwice == point);
    }
    // Prescribe closure with a final unequal-wheel half count.
    race.x = -0.01f;
    race.y = race.heading = 0;
    race.leftStartZone = race.returnArmed = true;
    assert(race.update(0, 1, 0, 10, 80, 20) == RaceSession::LAP_FINISHED);
    accumulate(1, 0);
    assert(race.fault == RaceSession::NONE && race.mapReady);
    assert(race.learnedCounts == lengthTwice / 2);
    assert(race.mapShift == (lengthTwice > 262144 ? 2 : (lengthTwice > 131072 ? 1 : 0)));
    assert(race.binsUsed == (race.learnedCounts + race.mapBinCounts() - 1) / race.mapBinCounts());
    for (unsigned bin = 0; bin < race.binsUsed; bin++)
    {
      int peak = 0;
      for (unsigned fine = bin << race.mapShift; fine < (bin + 1) << race.mapShift; fine++)
      {
        if (!weights[fine]) { continue; }
        const int average = sums[fine] / int(weights[fine]) / 8;
        peak = std::max(peak, abs(average));
        if (!race.mapShift) { assert(abs(int(race.curves[bin]) - average) <= 1); }
      }
      assert(race.curves[bin] != RaceSession::UNKNOWN_CURVE);
      assert(abs(abs(race.curves[bin]) - peak) <= 1); // Float-to-Q3 rounding allowance.
    }
    race.turnVotes = 8;
    race.speedScaleQ8 = 820;
    race.speedMmS = 1000;
    race.filteredCurve = 0;
    for (const uint32_t distance : {race.learnedCounts - 1, race.learnedCounts, race.learnedCounts + 1})
    {
      race.distanceTwice = distance * 2 + 1;
      assert(race.mapPosition() == distance % race.learnedCounts);
      assert(race.plannedCruise(true) == densePooledPlan(race));
    }
    cases++;
  }
  std::cout << "Unequal-wheel half-count storage/wrap: " << cases << " oracle cases pass.\n";
}

int main()
{
  testCapacityAndPartialWrap();
  testBothCompactionsAndLimit();
  testIrregularSampleBoundaries();
  testPooledSteeringAndExit();
  testUnequalWheelBoundaries();
  std::cout << "PASS: fixed 512-byte map, both capacity expansions, 32-bit/partial wrap, "
    "opposite-bend peaks, chunk boundaries, live-yaw steering and bounded full-map fault.\n";
}
