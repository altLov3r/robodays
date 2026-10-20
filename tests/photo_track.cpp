#include <fstream>
#include <map>
#include <sstream>
#define main regression_main
#include "test_line_follower.cpp"
#undef main

namespace Photo
{
struct Point { double x, y; };
double squared(Point a, Point b) { return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y); }
Point operator+(Point a, Point b) { return {a.x+b.x, a.y+b.y}; }
Point operator*(Point a, double scale) { return {a.x*scale, a.y*scale}; }

struct Track
{
  std::vector<Point> points;
  std::map<std::pair<int, int>, std::vector<unsigned>> cells;
  static std::pair<int, int> cell(Point p)
  { return {int(floor(p.x / 50)), int(floor(p.y / 50))}; }

  double distance(Point p) const
  {
    const auto key = cell(p);
    double nearest = 1e20;
    for (int x = key.first-1; x <= key.first+1; x++)
    for (int y = key.second-1; y <= key.second+1; y++)
    {
      const auto found = cells.find({x, y});
      if (found == cells.end()) { continue; }
      for (unsigned index : found->second)
      {
        const Point a = points[index], b = points[(index+1) % points.size()];
        const double dx = b.x-a.x, dy = b.y-a.y;
        const double t = fmax(0, fmin(1, ((p.x-a.x)*dx + (p.y-a.y)*dy) / (dx*dx+dy*dy)));
        nearest = fmin(nearest, squared(p, {a.x+t*dx, a.y+t*dy}));
      }
    }
    return sqrt(nearest);
  }

  void index()
  {
    for (unsigned i = 0; i < points.size(); i++) { cells[cell(points[i])].push_back(i); }
  }
};

// Match the pinned Pololu library's weighted black-line estimator. The firmware
// sees the union of all strokes; route order is used ONLY for independent scoring.
uint16_t position(const uint16_t * values, uint16_t previous)
{
  bool onLine = false;
  uint32_t sum = 0, weighted = 0;
  for (unsigned i = 0; i < 5; i++)
  {
    if (values[i] > 200) { onLine = true; }
    if (values[i] > 50) { weighted += uint32_t(values[i]) * i * 1000; sum += values[i]; }
  }
  return onLine ? weighted / sum : (previous < 2000 ? 0 : 4000);
}

void checkSensorModel()
{
  const uint16_t centered[] = {0, 0, 1000, 0, 0}, all[] = {1000, 1000, 1000, 1000, 1000};
  const uint16_t asymmetric[] = {1000, 1000, 1000, 1000, 0}, lost[] = {0, 0, 0, 0, 0};
  assert(position(centered, 0) == 2000 && position(all, 0) == 2000);
  assert(position(asymmetric, 0) == 1500 && position(lost, 1000) == 0);
  assert(position(lost, 3000) == 4000);
  Track crossing;
  for (int x = -100; x <= 100; x++) { crossing.points.push_back({double(x), 0}); }
  for (int y = -100; y <= 100; y++) { crossing.points.push_back({0, double(y)}); }
  crossing.index();
  assert(crossing.distance({0, 35}) < .001 && crossing.distance({35, 0}) < .001);
  assert(fabs(crossing.distance({10, 10}) - 10) < .001);
}
}

int main(int argc, char ** argv)
{
  using namespace Photo;
  assert(argc == 16);
  checkSensorModel();
  const DriveMode mode = static_cast<DriveMode>(atoi(argv[3]));
  const int direction = atoi(argv[4]);
  const double gain = atof(argv[5]), response10 = atof(argv[6]), pitch = atof(argv[7]);
  const double width = atof(argv[8]);
  const int noise = atoi(argv[9]);
  const double mismatch = atof(argv[10]), scale = atof(argv[12]);
  const unsigned replayPeriod = atoi(argv[11]);
  const double encoderBias = atof(argv[13]), gyroScale = atof(argv[14]);
  const double sensorDelay = atof(argv[15]) * .001;
  Track track;
  std::ifstream input(argv[1]);
  Point p;
  while (input >> p.x >> p.y) { track.points.push_back(p * scale); }
  assert(track.points.size() > 1000);
  if (direction < 0) { std::reverse(track.points.begin()+1, track.points.end()); }
  track.index();
  std::ostringstream ignored;
  auto * output = std::cout.rdbuf(ignored.rdbuf());
  resetRobot(); calibrateRobot(); selectedMode = mode; press(1); advance(2000);
  assert(state == RUNNING);
  Fake::directCounts = true;
  const uint32_t started = Fake::now;
  Point robot = track.points[0];
  double heading = atan2(track.points[1].y-robot.y, track.points[1].x-robot.x);
  double vl = 0, vr = 0, fl = 0, fr = 0, travel = 0;
  const int count = int(track.points.size());
  int progress = 0;
  uint32_t seed = 0x51a7u;
  unsigned crossingSamples[3] = {}, partialSamples[3] = {}, cautionSamples[3] = {};
  double maxOffset[3] = {}, maxDrop[3] = {}, lapProgress[3] = {};
  double maxCrossingDrop[3] = {}, maxCrossingBoost[3] = {};
  unsigned trueLapTimes[3] = {}, lastLapMs = 0;
  std::string modelStop;
  std::ofstream trace(argv[2]);
  trace << "ms,x,y,heading,progress,estimated_lap,position,dark,left,right,live_curve,map_curve,pending_ms,offset\n";
  unsigned frame = 0;
  for (; frame < 50000 && state == RUNNING; frame++)
  {
    const unsigned dt = race.mapReady && mode == BEAST && !race.noisyLearning ? replayPeriod : 10;
    const double seconds = dt * .001, response = 1-pow(1-response10, dt / 10.0);
    vl += (Fake::leftMotor*gain*(1+mismatch/2)-vl)*response;
    vr += (Fake::rightMotor*gain*(1-mismatch/2)-vr)*response;
    const double step = (vl+vr)*seconds/2, turn = (vl-vr)*seconds/86;
    robot = robot + Point{cos(heading+turn/2), sin(heading+turn/2)}*step;
    heading += turn; travel += step;
    // Model a replay-only change in measured travel (e.g. changed effective tyre
    // scale). Physical pose still uses true wheel travel. This is sensitivity,
    // not a measured slip estimate or an imposed firmware map offset.
    const double countScale = race.mapReady ? 1+encoderBias : 1;
    fl += vl*seconds*countScale/RaceSession::MM_PER_COUNT;
    fr += vr*seconds*countScale/RaceSession::MM_PER_COUNT;
    Fake::leftCounts = int(fl); fl -= Fake::leftCounts;
    Fake::rightCounts = int(fr); fr -= Fake::rightCounts;
    Fake::gyroRaw = int(lround(turn * gyroScale / (.07*3.14159265/180*seconds)));
    // Reconstruct the earlier optical pose within this constant-velocity step.
    const double opticalHeading = heading-turn*sensorDelay/seconds;
    const Point opticalRobot = robot + Point{cos(heading-turn*sensorDelay/seconds/2),
      sin(heading-turn*sensorDelay/seconds/2)}*(-(vl+vr)*sensorDelay/2);
    const Point forward{cos(opticalHeading), sin(opticalHeading)}, lateral{-sin(opticalHeading), cos(opticalHeading)};
    const Point sensor = opticalRobot + forward*30;
    unsigned dark = 0;
    for (int i = 0; i < 5; i++)
    {
      const double distance = track.distance(sensor + lateral*((i-2)*pitch));
      // A 4 mm finite spot softens the edge; sensor noise is calibrated-reading
      // noise, not an injected ideal line position. Width/pitch are assumptions.
      int reading = int(lround(1000*fmax(0, fmin(1, (width/2+2-distance)/4))));
      seed = seed*1664525u+1013904223u;
      if (noise) { reading += int(seed >> 16) % (2*noise+1) - noise; }
      Fake::readings[i] = std::max(0, std::min(1000, reading));
      if (reading > 750) { dark++; }
    }
    Fake::position = position(Fake::readings, Fake::position);
    // Local route continuity prevents an intersection shortcut from being scored
    // as a lap. No route coordinate or desired curvature is sent to the robot.
    int nearest = progress;
    double best = 1e20;
    for (int k = progress-100; k <= progress+100; k++)
    {
      const double distance = squared(robot, track.points[(k % count + count) % count]);
      if (distance < best) { best = distance; nearest = k; }
    }
    const int oldProgress = progress;
    progress = nearest;
    if (progress/count > oldProgress/count && progress/count <= 3)
    {
      const unsigned elapsed = Fake::now+dt-started;
      trueLapTimes[progress/count-1] = elapsed-lastLapMs;
      lastLapMs = elapsed;
    }
    const unsigned lap = race.completed;
    const int delivered = (Fake::leftMotor+Fake::rightMotor)/2;
    tick(dt);
    if (lap < 3)
    {
      if (dark >= 4) { crossingSamples[lap]++; }
      else if (dark == 3) { partialSamples[lap]++; }
      if (race.mapUncertain()) { cautionSamples[lap]++; }
      maxOffset[lap] = fmax(maxOffset[lap], sqrt(best));
      if (state == RUNNING)
      {
        const double drop = delivered-(Fake::leftMotor+Fake::rightMotor)/2.0;
        maxDrop[lap] = fmax(maxDrop[lap], drop);
        if (dark >= 4)
        {
          maxCrossingDrop[lap] = fmax(maxCrossingDrop[lap], drop);
          maxCrossingBoost[lap] = fmax(maxCrossingBoost[lap], -drop);
        }
      }
      if (race.completed != lap) { lapProgress[lap] = double(progress)/count; }
    }
    if (frame % 4 == 0 || state != RUNNING)
    {
      const int mapped = race.mapReady && race.learnedCounts ?
        race.curves[race.mapBin(race.distanceCounts()%race.learnedCounts)] : -128;
      trace << Fake::now-started << ',' << robot.x << ',' << robot.y << ',' << heading << ','
        << double(progress)/count << ',' << int(race.completed) << ',' << Fake::position << ',' << dark
        << ',' << Fake::leftMotor << ',' << Fake::rightMotor << ',' << race.filteredCurve/8 << ','
        << mapped << ',' << int(race.mismatchMs) << ',' << sqrt(best) << '\n';
    }
    if (sqrt(best) > 80) { modelStop = "Route departure >80mm"; break; }
  }
  std::cout.rdbuf(output);
  std::string stop = race.completed == 3 ? "Finished" : Fake::displayRows[0];
  if (!modelStop.empty()) { stop = modelStop; }
  while (!stop.empty() && stop.back() == ' ') { stop.pop_back(); }
  if (frame == 50000) { stop = "Model timeout"; }
  // Six millimetres allows trace discretization; a finish tens of centimetres
  // early must fail even when the display says all three laps completed.
  const bool complete = race.completed == 3 && progress >= 3*count-3;
  std::cout << "{\"complete\":" << (complete ? "true" : "false") << ",\"stop\":\"" << stop
    << "\",\"estimated_laps\":" << int(race.completed) << ",\"true_laps\":" << std::max(0, progress/count)
    << ",\"route_fraction\":" << double(progress)/count << ",\"travel_mm\":" << travel
    << ",\"learned_mm\":" << race.learnedCounts*RaceSession::MM_PER_COUNT
    << ",\"bins\":" << race.binsUsed << ",\"map_shift\":" << int(race.mapShift)
    << ",\"fault\":" << int(race.fault) << ",\"noisy_learning\":" << (race.noisyLearning ? "true" : "false")
    << ",\"scale_q8\":" << race.speedScaleQ8 << ",\"track_width_mm\":" << int(race.trackWidthMm);
  const auto array = [](const char * name, const double * values)
  { std::cout << ",\"" << name << "\":[" << values[0] << ',' << values[1] << ',' << values[2] << ']'; };
  double laps[3], trueTimes[3], crossings[3], partials[3], caution[3];
  for (unsigned i = 0; i < 3; i++)
  {
    laps[i] = race.lapTimes[i]; trueTimes[i] = trueLapTimes[i];
    crossings[i] = crossingSamples[i]; partials[i] = partialSamples[i]; caution[i] = cautionSamples[i];
  }
  array("estimated_lap_ms", laps); array("true_lap_ms", trueTimes);
  array("lap_route_fraction", lapProgress); array("crossing_samples", crossings);
  array("three_dark_samples", partials); array("caution_samples", caution);
  array("max_axle_offset_mm", maxOffset); array("max_running_common_drop_pwm", maxDrop);
  array("max_crossing_common_drop_pwm", maxCrossingDrop);
  array("max_crossing_common_boost_pwm", maxCrossingBoost);
  std::cout << "}\n";
}
