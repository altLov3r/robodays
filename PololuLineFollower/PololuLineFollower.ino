// Pololu 3pi+ 32U4 Standard (30:1 MP motors), black line on white floor.
// OLED edition only. No external wiring changes.

#ifdef ARDUINO
// Component headers avoid linking the unused buzzer timer ISR into this sketch.
#include <Pololu3piPlus32U4Motors.h>
#include <Pololu3piPlus32U4Encoders.h>
#include <Pololu3piPlus32U4LineSensors.h>
#include <Pololu3piPlus32U4BumpSensors.h>
#include <Pololu3piPlus32U4Buttons.h>
#include <Pololu3piPlus32U4OLED.h>

// Pololu 5x8 glyphs for the emitted letters, digits and punctuation.
/*
Copyright (c) 2021 Pololu Corporation (www.pololu.com)

Permission is hereby granted, free of charge, to any person
obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without
restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following
conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/
const uint8_t raceOledFont[][5] PROGMEM = {
  {0b00001000, 0b00001000, 0b00111110, 0b00001000, 0b00001000},
  {0b00000000, 0b01010000, 0b00110000, 0b00000000, 0b00000000},
  {0b00001000, 0b00001000, 0b00001000, 0b00001000, 0b00001000},
  {0b00000000, 0b01100000, 0b01100000, 0b00000000, 0b00000000},
  {0b00100000, 0b00010000, 0b00001000, 0b00000100, 0b00000010},
  {0b00111110, 0b01010001, 0b01001001, 0b01000101, 0b00111110},
  {0b00000000, 0b01000010, 0b01111111, 0b01000000, 0b00000000},
  {0b01000010, 0b01100001, 0b01010001, 0b01001001, 0b01000110},
  {0b00100001, 0b01000001, 0b01000101, 0b01001011, 0b00110001},
  {0b00011000, 0b00010100, 0b00010010, 0b01111111, 0b00010000},
  {0b00100111, 0b01000101, 0b01000101, 0b01000101, 0b00111001},
  {0b00111100, 0b01001010, 0b01001001, 0b01001001, 0b00110000},
  {0b00000011, 0b00000001, 0b01110001, 0b00001001, 0b00000111},
  {0b00110110, 0b01001001, 0b01001001, 0b01001001, 0b00110110},
  {0b00000110, 0b01001001, 0b01001001, 0b00101001, 0b00011110},
  {0b00000000, 0b00110110, 0b00110110, 0b00000000, 0b00000000},
  {0b00000000, 0b01010110, 0b00110110, 0b00000000, 0b00000000},
  {0b00001000, 0b00010100, 0b00100010, 0b01000001, 0b00000000},
  {0b00010100, 0b00010100, 0b00010100, 0b00010100, 0b00010100},
  {0b00000000, 0b01000001, 0b00100010, 0b00010100, 0b00001000},
  {0b00000010, 0b00000001, 0b01010001, 0b00001001, 0b00000110},
  {0b00110010, 0b01001001, 0b01111001, 0b01000001, 0b00111110},
  {0b01111110, 0b00010001, 0b00010001, 0b00010001, 0b01111110},
  {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b00110110},
  {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00100010},
  {0b01111111, 0b01000001, 0b01000001, 0b01000001, 0b00111110},
  {0b01111111, 0b01001001, 0b01001001, 0b01001001, 0b01000001},
  {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000001},
  {0b00111110, 0b01000001, 0b01001001, 0b01001001, 0b01111010},
  {0b01111111, 0b00001000, 0b00001000, 0b00001000, 0b01111111},
  {0b00000000, 0b01000001, 0b01111111, 0b01000001, 0b00000000},
  {0b00100000, 0b01000000, 0b01000001, 0b00111111, 0b00000001},
  {0b01111111, 0b00001000, 0b00010100, 0b00100010, 0b01000001},
  {0b01111111, 0b01000000, 0b01000000, 0b01000000, 0b01000000},
  {0b01111111, 0b00000010, 0b00001100, 0b00000010, 0b01111111},
  {0b01111111, 0b00000100, 0b00001000, 0b00010000, 0b01111111},
  {0b00111110, 0b01000001, 0b01000001, 0b01000001, 0b00111110},
  {0b01111111, 0b00001001, 0b00001001, 0b00001001, 0b00000110},
  {0b00111110, 0b01000001, 0b01010001, 0b00100001, 0b01011110},
  {0b01111111, 0b00001001, 0b00011001, 0b00101001, 0b01000110},
  {0b01000110, 0b01001001, 0b01001001, 0b01001001, 0b00110001},
  {0b00000001, 0b00000001, 0b01111111, 0b00000001, 0b00000001},
  {0b00111111, 0b01000000, 0b01000000, 0b01000000, 0b00111111},
  {0b00011111, 0b00100000, 0b01000000, 0b00100000, 0b00011111},
  {0b00111111, 0b01000000, 0b00111000, 0b01000000, 0b00111111},
  {0b01100011, 0b00010100, 0b00001000, 0b00010100, 0b01100011},
  {0b00000111, 0b00001000, 0b01110000, 0b00001000, 0b00000111},
  {0b01100001, 0b01010001, 0b01001001, 0b01000101, 0b01000011},
  {0b00000000, 0b00000000, 0b01001111, 0b00000000, 0b00000000},
};

namespace Pololu3piPlus32U4
{
// Board helpers from Pololu3piPlus32U4.h (library 1.1.3).
inline void ledRed(bool on) { FastGPIO::Pin<17>::setOutput(!on); }
inline void ledYellow(bool on) { FastGPIO::Pin<13>::setOutput(on); }
inline void ledGreen(bool on) { FastGPIO::Pin<IO_D5>::setOutput(!on); }
inline bool usbPowerPresent() { return USBSTA >> VBUS & 1; }
inline uint16_t readBatteryMillivolts()
{
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) { sum += analogRead(A1); }
  return (static_cast<uint32_t>(sum) * 1875 + 511) / 1024;
}
}
#else
#include <Pololu3piPlus32U4.h>
#endif
#include <Pololu3piPlus32U4IMU.h>
#include <avr/wdt.h>
#include "RaceSession.h"

static_assert(RACE_PLANNER_REVISION == 42, "Update both sketch files together.");

using namespace Pololu3piPlus32U4;

#ifdef ARDUINO
// Fold lowercase text to uppercase and reject glyphs outside the compact table.
template<> uint8_t PololuSH1106Main<Pololu3piPlus32U4::OLEDCore>::getGlyphColumn(uint8_t glyph, uint8_t pixelX)
{
  if (glyph >= 'a' && glyph <= 'z') { glyph -= 'a' - 'A'; }
  uint8_t index;
  if (glyph == '!') { index = 48; }
  else if (glyph >= '+' && glyph <= 'Z') { index = glyph - '+'; }
  else { return 0; }
  return pgm_read_byte(&raceOledFont[index][pixelX]);
}
// Start directly in the used OLED layout; do not link the unused 8x2 renderer.
template<> PololuSH1106Main<Pololu3piPlus32U4::OLEDCore>::PololuSH1106Main()
{
  noAutoDisplay();
  setLayout21x8();
}
#endif
OLED display;

namespace Settings
{
constexpr int16_t MOTOR_LIMIT = 400; // Absolute library limit; profiles impose lower limits.
constexpr int16_t CURVE_SPEED = 65;
constexpr int16_t ACCEL_STEP = 3;
constexpr int16_t DRIVE_ACCEL_STEP = 6;
constexpr int16_t STRAIGHT_ACCEL_STEP = 24;
constexpr uint8_t STRAIGHT_SETTLE_MS = 35;
constexpr int16_t DRIVE_BRAKE_STEP = 3;
constexpr int16_t STEERING_DEADBAND = 24;
constexpr int16_t KP_256 = 32;       // P = 0.125
constexpr int16_t KD_256 = 96;       // D = 0.375 per 10 ms sample
constexpr int16_t STRAIGHT_KD_256 = 64;
constexpr uint16_t CONTROL_MS = 10;
constexpr uint16_t FAST_CONTROL_MS = 5;
constexpr int16_t CALIBRATION_SPEED = 60;
constexpr uint16_t CALIBRATION_MS = 8000; // Absolute motion timeout.
constexpr uint16_t CALIBRATION_PHASE_MS = 2000;
constexpr uint16_t CALIBRATION_PAUSE_MS = 150;
constexpr uint16_t PREVIEW_MS = 100;
constexpr int16_t CENTER_TOLERANCE = 100;
constexpr int16_t START_TOLERANCE = 350;
constexpr uint16_t MIN_CALIBRATION_SPAN = 200;
constexpr uint16_t LINE_THRESHOLD = 250;
constexpr uint16_t SENSOR_TIMEOUT_US = 2000;
constexpr uint16_t BATTERY_MIN_MV = 4400;
constexpr uint16_t BATTERY_MAX_MV = 6600;
constexpr uint16_t STALL_MS = 400;
constexpr uint32_t MAX_RUN_MS = 240000; // Whole-session backup, not a lap counter.
}

static_assert(Settings::MOTOR_LIMIT <= 400, "Never exceed the library motor limit.");
static_assert(Settings::CONTROL_MS == 10, "Keep the 10 ms reference; tune FAST_CONTROL_MS instead.");
static_assert(Settings::FAST_CONTROL_MS >= 5 && Settings::FAST_CONTROL_MS <= Settings::CONTROL_MS,
  "Fast control must remain between 5 and 10 ms.");
static_assert(Settings::CALIBRATION_SPEED > 0 &&
  Settings::CALIBRATION_SPEED <= 60, "Keep calibration slow: at most 60/400.");

// Fixed storage avoids heap allocation for the only calibration mode we use.
// Retain Pololu's rule: all ten readings must support a new extreme.
class RaceLineSensors : public LineSensors
{
public:
  RaceLineSensors() { resetCalibration(); }
  RaceLineSensors(const RaceLineSensors &) = delete;
  RaceLineSensors & operator=(const RaceLineSensors &) = delete;
  ~RaceLineSensors()
  {
    // The base destructor owns heap pointers; our arrays belong to this object.
    calibrationOn.minimum = calibrationOn.maximum = nullptr;
    calibrationOff.minimum = calibrationOff.maximum = nullptr;
  }

  void resetCalibration()
  {
    calibrationOn.minimum = minimum;
    calibrationOn.maximum = maximum;
    calibrationOn.initialized = true;
    for (uint8_t i = 0; i < 5; i++)
    { minimum[i] = Settings::SENSOR_TIMEOUT_US; maximum[i] = 0; }
  }

  // Based on Pololu LineSensors.cpp, under the MIT notice above.
  void calibrate()
  {
    uint16_t values[5], low[5], high[5];
    for (uint8_t sample = 0; sample < 10; sample++)
    {
      read(values);
      for (uint8_t i = 0; i < 5; i++)
      {
        if (!sample || values[i] < low[i]) { low[i] = values[i]; }
        if (!sample || values[i] > high[i]) { high[i] = values[i]; }
      }
    }
    for (uint8_t i = 0; i < 5; i++)
    {
      if (low[i] > maximum[i]) { maximum[i] = low[i]; }
      if (high[i] < minimum[i]) { minimum[i] = high[i]; }
    }
  }

private:
  uint16_t minimum[5], maximum[5];
};

Motors motors;
Encoders encoders;
RaceLineSensors lineSensors;
BumpSensors bumpSensors;
ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;
IMU imu;
RaceSession race;
DriveMode selectedMode = SAFE;
bool imuReady = false;
uint16_t gyroSamples = 0;
int32_t gyroSum = 0;
int16_t gyroMinimum = 32767, gyroMaximum = -32768;
float gyroBias = 0;
uint32_t gyroLastMicros = 0;
uint32_t gyroLastFresh = 0;
int16_t latestLeftCounts = 0, latestRightCounts = 0;

enum RunState : uint8_t { IDLE, CALIBRATING, WAIT_RELEASE, COUNTDOWN, RUNNING };
RunState state = IDLE;
bool calibrated = false;
bool buttonsArmed = false;
bool watchdogEnabled = false;
bool calibrationRequested = false;
bool calibrationPausing = false;
uint8_t calibrationPhase = 0;
int32_t calibrationLeftTravel = 0;
int32_t calibrationRightTravel = 0;
uint32_t calibrationPhaseSince = 0;
uint32_t lastPreview = 0;
uint32_t lastIdleUpdate = 0;
uint8_t countdownSeconds = 0;
uint16_t sensorValues[5];
uint16_t batteryMv = 0;
uint16_t leftStallMs = 0;
uint16_t rightStallMs = 0;
uint32_t stateSince = 0;
uint32_t lastButtonHeld = 0;
uint32_t lastControl = 0;
uint16_t lastSensorMicros = 0;
uint32_t lastBattery = 0;
uint32_t lastEncoderCheck = 0;
uint32_t darkSince = 0;
bool darkActive = false;
bool runDiagnostics = false;
int16_t leftCommand = 0;
int16_t rightCommand = 0;
int16_t forwardCommand = 0;
int16_t lastError = 0;
int16_t filteredDelta = 0;
int8_t throttleRemainder = 0;
int16_t peakDrivePwm = 0;
int16_t peakThrottlePwm = 0;
enum SpeedLimiter : uint8_t { MAP_LIMIT, LINE_LIMIT, ACCEL_LIMIT, STEER_LIMIT, FULL_SPEED, YAW_LIMIT };
uint16_t limiterTicks[6] = {};
SpeedLimiter speedLimiter = FULL_SPEED;

const __FlashStringHelper * modeName(DriveMode mode)
{
  if (mode == SAFE) { return F("SAFE"); }
  if (mode == MID) { return F("MID"); }
  return F("BEAST");
}

const __FlashStringHelper * buttonMenu() { return F("A CAL B RUN C MODE"); }
// Share repeated flash strings to leave room for the corner controller.
const __FlashStringHelper * calibrationTitle() { return F("AUTO CAL"); }
const __FlashStringHelper * holdStillMessage() { return F("Hold still"); }
const __FlashStringHelper * handsClearMessage() { return F("Hands clear"); }
const __FlashStringHelper * imuErrorMessage() { return F("IMU error"); }
const __FlashStringHelper * calibrationTimeoutMessage() { return F("Cal time"); }
const __FlashStringHelper * encoderErrorMessage() { return F("Enc err"); }
const __FlashStringHelper * encoderDirectionMessage() { return F("Enc dir"); }
const __FlashStringHelper * stalledMessage() { return F("Stalled"); }
const __FlashStringHelper * stoppedMessage() { return F("Stopped"); }
const __FlashStringHelper * releaseMessage() { return F("Release"); }

void flushDisplay()
{
  display.display();
  // Pololu display() re-enables automatic writes; keep drawing buffered.
  display.noAutoDisplay();
}

void clearDisplayRow(uint8_t row)
{
  display.gotoXY(0, row);
  for (uint8_t column = 0; column < 20; column++) { display.print(' '); }
  display.gotoXY(0, row);
}

__attribute__((noinline)) void printPair(int16_t first, int16_t second)
{
  display.print(first);
  display.print('/');
  display.print(second);
}

__attribute__((noinline)) void printLimiterShare(SpeedLimiter limiter, uint16_t total)
{
  display.print(static_cast<uint16_t>(total ? 100UL * limiterTicks[limiter] / total : 0));
}

void showStatus(const __FlashStringHelper * first,
  const __FlashStringHelper * second)
{
  display.clear();
  display.print(first);
  display.gotoXY(0, 2);
  if (runDiagnostics) { printPair(lastError, filteredDelta); }
  else { display.print(second); }
  flushDisplay();
}

// Display millivolts/milliseconds in base units with two decimal places.
__attribute__((noinline)) void printMilliValue(uint32_t value)
{
  display.print(static_cast<uint16_t>(value / 1000));
  display.print('.');
  const uint8_t hundredths = static_cast<uint16_t>(value % 1000) / 10;
  if (hundredths < 10) { display.print('0'); }
  display.print(hundredths);
}

void showIdleInfo(uint32_t now)
{
  if (now - lastIdleUpdate < 500) { return; }
  lastIdleUpdate = now;
  batteryMv = readBatteryMillivolts();
  display.gotoXY(0, 5);
  display.print(F("Battery: "));
  printMilliValue(batteryMv);
  display.print(F("V  "));
  display.gotoXY(0, 6);
  display.print(modeName(selectedMode));
  display.print(F("     "));
  display.gotoXY(0, 7);
  display.print(buttonMenu());
  flushDisplay();
}

void showCountdown(uint32_t)
{
  if (countdownSeconds) { return; }
  countdownSeconds = 1;
  showStatus(calibrationRequested ? calibrationTitle() : modeName(selectedMode),
    calibrationRequested ? handsClearMessage() : holdStillMessage());
}

void showCalibrationScreen()
{
  showStatus(calibrationTitle(), handsClearMessage());
}

void stopRobot(const __FlashStringHelper * reason)
{
  // Stop before display work, and require a new released-then-pressed button.
  motors.setSpeeds(0, 0);
  runDiagnostics = state == RUNNING && race.completed < 3;
  forwardCommand = leftCommand = rightCommand = 0;
  throttleRemainder = 0;
  race.abort();
  state = IDLE;
  calibrationRequested = false;
  buttonsArmed = false;
  lastButtonHeld = millis();
  ledGreen(false);
  ledYellow(false);
  ledRed(true);
  showStatus(reason, releaseMessage());
  lastIdleUpdate = millis() - 500;
}

void showResults()
{
  stopRobot(F("3 LAPS EST. DONE"));
  display.gotoXY(0, 1);
  printPair(race.speedScaleQ8, race.trackWidthMm);
  for (uint8_t i = 0; i < 3; i++)
  {
    display.gotoXY(0, i + 3);
    display.print(F("Lap "));
    display.print(i + 1);
    display.print(F(": "));
    printMilliValue(race.lapTimes[i]);
    display.print('s');
  }
  display.gotoXY(0, 6);
  display.print(F("PWM "));
  printPair(peakDrivePwm, peakThrottlePwm);
  display.gotoXY(0, 7);
  uint8_t dominant = 0;
  uint16_t total = limiterTicks[0]; // The 240 s session cap bounds replay to 48,000 five-ms ticks.
  for (uint8_t i = 1; i < 6; i++)
  {
    total += limiterTicks[i];
    if (limiterTicks[i] > limiterTicks[dominant]) { dominant = i; }
  }
  display.print(race.noisyLearning ? F("MID FALLBACK") : F("Limit: "));
  if (!race.noisyLearning) { display.print(!race.trustedGeometry() ? 'G' : "MLASFY"[dominant]); }
  display.gotoXY(0, 2);
  display.print(F("A/L/S "));
  printLimiterShare(ACCEL_LIMIT, total);
  display.print('/');
  printLimiterShare(LINE_LIMIT, total);
  display.print('/');
  printLimiterShare(STEER_LIMIT, total);
  flushDisplay();
  ledRed(false);
}

void selectNextMode()
{
  selectedMode = static_cast<DriveMode>((selectedMode + 1) % 3);
  race.abort();
  race.completed = 0;
  showStatus(modeName(selectedMode), buttonMenu());
  lastIdleUpdate = millis() - 500;
  lastPreview = millis() - Settings::PREVIEW_MS;
}

bool batteryOK()
{
  batteryMv = readBatteryMillivolts();
  lastBattery = millis();
  if (batteryMv < Settings::BATTERY_MIN_MV)
  {
    stopRobot(F("Bat low"));
    return false;
  }
  if (batteryMv > Settings::BATTERY_MAX_MV)
  {
    stopRobot(F("Bat high"));
    return false;
  }
  return true;
}

bool driveInterlocks()
{
  if (usbPowerPresent())
  {
    stopRobot(F("USB out"));
    return false;
  }
  if (state != RUNNING && bumpSensors.read())
  {
    stopRobot(F("Bumped"));
    return false;
  }
  return true;
}

bool calibrationOK()
{
  const auto & data = lineSensors.calibrationOn;
  if (!data.initialized || !data.minimum || !data.maximum) { return false; }
  for (uint8_t i = 0; i < 5; i++)
  {
    if (data.maximum[i] < data.minimum[i] ||
      data.maximum[i] - data.minimum[i] < Settings::MIN_CALIBRATION_SPAN)
    {
      return false;
    }
  }
  return true;
}

uint16_t peakReading()
{
  uint16_t peak = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    if (sensorValues[i] > peak) { peak = sensorValues[i]; }
  }
  return peak;
}

bool startPositionOK()
{
  const uint16_t position = lineSensors.readLineBlack(sensorValues);
  if (peakReading() < Settings::LINE_THRESHOLD ||
    position < 2000 - Settings::START_TOLERANCE ||
    position > 2000 + Settings::START_TOLERANCE ||
    sensorValues[0] > 500 || sensorValues[4] > 500)
  {
    stopRobot(F("Centre!"));
    return false;
  }
  return true;
}

void beginCalibration()
{
  motors.setSpeeds(0, 0);
  if (!driveInterlocks() || !batteryOK()) { return; }
  race.completed = 0;
  calibrated = false;
  calibrationRequested = true;
  state = WAIT_RELEASE;
  stateSince = millis();
  ledRed(false);
  ledGreen(false);
  ledYellow(true);
  showStatus(calibrationTitle(), releaseMessage());
}

void requestStart()
{
  if (!calibrated)
  {
    stopRobot(F("Cal A!"));
    return;
  }
  if (!imuReady) { stopRobot(imuErrorMessage()); return; }
  if (!driveInterlocks() || !batteryOK() || !startPositionOK()) { return; }
  calibrationRequested = false;
  state = WAIT_RELEASE;
  stateSince = millis();
  ledRed(false);
  ledYellow(true);
  showStatus(modeName(selectedMode), releaseMessage());
}

void startGyroCalibration()
{
  gyroSamples = 0;
  gyroSum = 0;
  gyroMinimum = 32767;
  gyroMaximum = -32768;
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

bool imuOK()
{
  if (imu.getLastError() || Wire.getWireTimeoutFlag())
  {
    stopRobot(imuErrorMessage());
    return false;
  }
  return true;
}

bool sampleGyroCalibration(uint32_t now)
{
  if (now - stateSince < 250 || gyroSamples >= 256) { return true; }
  const int16_t left = encoders.getCountsAndResetLeft();
  const int16_t right = encoders.getCountsAndResetRight();
  if (left > 2 || left < -2 || right > 2 || right < -2)
  {
    stopRobot(holdStillMessage());
    return false;
  }
  const bool fresh = imu.gyroDataReady();
  if (!imuOK()) { return false; }
  if (!fresh) { return true; }
  imu.readGyro();
  if (!imuOK()) { return false; }
  gyroSum += imu.g.z;
  if (imu.g.z < gyroMinimum) { gyroMinimum = imu.g.z; }
  if (imu.g.z > gyroMaximum) { gyroMaximum = imu.g.z; }
  gyroSamples++;
  return true;
}

bool readTurnDelta(float & turn)
{
  const bool fresh = imu.gyroDataReady();
  if (!imuOK()) { return false; }
  if (fresh)
  {
    imu.readGyro();
    if (!imuOK()) { return false; }
    gyroLastFresh = millis();
  }
  if (millis() - gyroLastFresh > 30) { stopRobot(F("Gyro stale")); return false; }
  const float raw = imu.g.z - gyroBias;
  if (fabsf(raw) > 28000) { stopRobot(F("Gyro range")); return false; }
  const uint32_t nowMicros = micros();
  turn = raw * (0.07f * 3.14159265f / 180000000.0f) * (nowMicros - gyroLastMicros);
  gyroLastMicros = nowMicros;
  return true;
}

void beginRun()
{
  if (!driveInterlocks() || !batteryOK() || !startPositionOK()) { return; }
  if (gyroSamples < 128) { stopRobot(F("Gyro cal")); return; }
  if (static_cast<int32_t>(gyroMaximum) - gyroMinimum > 80)
  {
    stopRobot(holdStillMessage());
    return;
  }
  gyroBias = static_cast<float>(gyroSum) / gyroSamples;
  // Seed D from the actual start position: no derivative kick at startup.
  lastError = static_cast<int16_t>(lineSensors.readLineBlack(sensorValues)) - 2000;
  filteredDelta = 0;
  darkActive = false;
  leftStallMs = rightStallMs = 0;
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  encoders.checkErrorLeft();
  encoders.checkErrorRight();
  forwardCommand = leftCommand = rightCommand = 0;
  throttleRemainder = 0;
  peakDrivePwm = 0;
  peakThrottlePwm = 0;
  memset(limiterTicks, 0, sizeof(limiterTicks));
  display.clear();
  flushDisplay();
  // All display writes finish before timing the run.
  state = RUNNING;
  stateSince = lastControl = lastEncoderCheck = millis();
  race.reset(selectedMode, stateSince);
  lastSensorMicros = micros();
  gyroLastMicros = micros();
  gyroLastFresh = millis();
  ledYellow(false);
  ledGreen(true);
}

void resetCalibrationPhase(uint32_t now)
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
  calibrationLeftTravel = calibrationRightTravel = 0;
  leftStallMs = rightStallMs = 0;
  calibrationPausing = false;
  calibrationPhaseSince = lastControl = now;
}

void startCalibrationMotion()
{
  if (!driveInterlocks() || !batteryOK()) { return; }
  lineSensors.resetCalibration();
  encoders.checkErrorLeft();
  encoders.checkErrorRight();
  calibrationPhase = 0;
  forwardCommand = leftCommand = rightCommand = 0;
  showCalibrationScreen();
  state = CALIBRATING;
  stateSince = millis();
  resetCalibrationPhase(stateSince);
}

void printPaddedValue(uint16_t value)
{
  if (value < 1000) { display.print(' '); }
  if (value < 100) { display.print(' '); }
  if (value < 10) { display.print(' '); }
  display.print(value);
}

void showSensorPreview(uint32_t now)
{
  if (!calibrated || now - lastPreview < Settings::PREVIEW_MS) { return; }
  lastPreview = now;
  const uint16_t position = lineSensors.readLineBlack(sensorValues);
  const int16_t error = static_cast<int16_t>(position) - 2000;
  const bool lineVisible = peakReading() >= Settings::LINE_THRESHOLD;
  const bool narrowLine = sensorValues[0] <= 500 && sensorValues[4] <= 500;
  const bool centered = lineVisible && narrowLine &&
    error >= -Settings::CENTER_TOLERANCE && error <= Settings::CENTER_TOLERANCE;
  // Keep row 0 (including any stop reason) visible. No graphics buffer needed.
  for (uint8_t i = 0; i < 5; i++)
  {
    clearDisplayRow(i + 2);
    display.print(i);
    display.print(':');
    printPaddedValue(sensorValues[i]);
  }
  clearDisplayRow(7);
  if (!lineVisible) { display.print(F("NO LINE")); }
  else if (!narrowLine) { display.print(F("WIDE/OFF")); }
  else if (centered) { display.print(F("CENTERED")); }
  else { display.print(error < 0 ? F("SHIFT LEFT") : F("SHIFT RIGHT")); }
  flushDisplay();
  ledGreen(centered);
}

int16_t rampCommand(int16_t current, int16_t target)
{
  // Slow increases, immediate reductions. Emergency stops bypass the ramp.
  if (target > current + Settings::ACCEL_STEP)
  {
    return current + Settings::ACCEL_STEP;
  }
  return target;
}

void calibrateAutomatically(uint32_t now)
{
  if (now - stateSince >= Settings::CALIBRATION_MS)
  {
    stopRobot(calibrationTimeoutMessage());
    return;
  }
  if (calibrationPausing)
  {
    if (now - calibrationPhaseSince >= Settings::CALIBRATION_PAUSE_MS)
    {
      // Flush residual motion while stopped before changing direction.
      resetCalibrationPhase(now);
    }
    return;
  }
  const uint32_t elapsed = now - lastControl;
  if (elapsed < Settings::CONTROL_MS) { return; }
  if (elapsed > 100 || now - calibrationPhaseSince >= Settings::CALIBRATION_PHASE_MS)
  {
    stopRobot(calibrationTimeoutMessage());
    return;
  }
  lastControl = now;
  const bool leftError = encoders.checkErrorLeft();
  const bool rightError = encoders.checkErrorRight();
  if (leftError || rightError) { stopRobot(encoderErrorMessage()); return; }

  // Right, left, right, left, then back towards the original heading.
  // Counts limit travel independently of battery/load; they are not exact angles.
  const int16_t direction = (calibrationPhase % 2 == 0) ? 1 : -1;
  const int16_t target = (calibrationPhase == 0 || calibrationPhase == 4) ? 100 : 200;
  const int32_t leftDelta =
    static_cast<int32_t>(encoders.getCountsAndResetLeft()) * direction;
  const int32_t rightDelta =
    static_cast<int32_t>(encoders.getCountsAndResetRight()) * -direction;
  if (leftDelta < -2 || rightDelta < -2)
  {
    stopRobot(encoderDirectionMessage());
    return;
  }
  calibrationLeftTravel += leftDelta;
  calibrationRightTravel += rightDelta;
  const int16_t leftMagnitude = leftCommand < 0 ? -leftCommand : leftCommand;
  const int16_t rightMagnitude = rightCommand < 0 ? -rightCommand : rightCommand;
  leftStallMs = (leftMagnitude >= 45 && leftDelta < 1) ?
    leftStallMs + static_cast<uint16_t>(elapsed) : 0;
  rightStallMs = (rightMagnitude >= 45 && rightDelta < 1) ?
    rightStallMs + static_cast<uint16_t>(elapsed) : 0;
  if (leftStallMs >= Settings::STALL_MS || rightStallMs >= Settings::STALL_MS)
  {
    stopRobot(stalledMessage());
    return;
  }

  if (calibrationLeftTravel >= target && calibrationRightTravel >= target)
  {
    motors.setSpeeds(0, 0);
    leftCommand = rightCommand = 0;
    calibrationPhase++;
    if (calibrationPhase == 5)
    {
      calibrated = calibrationOK();
      stopRobot(calibrated ? modeName(selectedMode) : F("Bad cal"));
      ledRed(!calibrated);
      lastPreview = now - Settings::PREVIEW_MS;
      return;
    }
    calibrationPausing = true;
    calibrationPhaseSince = now;
    showCalibrationScreen(); // Refresh only with motors stopped between sweeps.
    return;
  }

  const int16_t leftTarget = calibrationLeftTravel >= target ? 0 : Settings::CALIBRATION_SPEED;
  const int16_t rightTarget = calibrationRightTravel >= target ? 0 : Settings::CALIBRATION_SPEED;
  leftCommand = direction * rampCommand(leftMagnitude, leftTarget);
  rightCommand = -direction * rampCommand(rightMagnitude, rightTarget);
  motors.setSpeeds(leftCommand, rightCommand);
  lineSensors.calibrate(); // Ten bounded reads; stop buttons checked next loop.
}

bool checkEncoders(uint32_t now)
{
  const bool leftError = encoders.checkErrorLeft();
  const bool rightError = encoders.checkErrorRight();
  if (leftError || rightError)
  {
    stopRobot(encoderErrorMessage());
    return false;
  }
  const uint32_t elapsed = now - lastEncoderCheck;
  lastEncoderCheck = now;
  const int16_t leftCounts = encoders.getCountsAndResetLeft();
  const int16_t rightCounts = encoders.getCountsAndResetRight();
  latestLeftCounts = leftCounts;
  latestRightCounts = rightCounts;
  if (leftCounts < -2 || rightCounts < -2)
  {
    stopRobot(encoderDirectionMessage());
    return false;
  }
  leftStallMs = (leftCommand >= 45 && leftCounts < 1) ?
    leftStallMs + static_cast<uint16_t>(elapsed) : 0;
  rightStallMs = (rightCommand >= 45 && rightCounts < 1) ?
    rightStallMs + static_cast<uint16_t>(elapsed) : 0;
  if (leftStallMs >= Settings::STALL_MS || rightStallMs >= Settings::STALL_MS)
  {
    stopRobot(stalledMessage());
    return false;
  }
  return true;
}

__attribute__((noinline)) int16_t trackingSpeed(int16_t base, int16_t demand)
{
  if (demand > 1000) { demand = 1000; }
  return base - static_cast<int32_t>(base - Settings::CURVE_SPEED) * demand / 1000;
}

bool lineSettled(int16_t error, int16_t delta)
{
  return abs(error) < 80 && abs(delta) < 30;
}

int16_t accelerationStep()
{
  // qualifyStraight validates this sample immediately before the output ramp.
  return race.straightStableMs >= Settings::STRAIGHT_SETTLE_MS ?
    Settings::STRAIGHT_ACCEL_STEP : Settings::DRIVE_ACCEL_STEP;
}

int16_t baseSpeedForLine(int16_t error, int16_t delta, bool crossing)
{
  if (crossing) { speedLimiter = LINE_LIMIT; return Settings::CURVE_SPEED; }
  const int16_t offset = abs(error);
  const int16_t change = abs(delta);
  // Small centered sensor jitter should steer without cutting straight throttle.
  int16_t demand = (offset > 100 ? offset - 100 : 0) +
    (change > 60 ? (change - 60) * 2 : 0);
  // Map and live sensing independently cap speed; do not apply the same bend
  // penalty twice by scaling an already-reduced mapped target again.
  const int16_t cruise = race.profile().cruise;
  const int16_t live = trackingSpeed(cruise, demand);
  const int16_t mapped = race.plannedCruise(lineSettled(error, delta));
  speedLimiter = mapped < live ? (race.liveLimited ? YAW_LIMIT : MAP_LIMIT) : LINE_LIMIT;
  if (mapped == cruise && live == cruise) { speedLimiter = FULL_SPEED; }
  return live < mapped ? live : mapped;
}

__attribute__((noinline)) void qualifyStraight(int16_t error, int16_t delta, bool crossing, uint16_t dt)
{
  const bool holding = race.straightStableMs >= Settings::STRAIGHT_SETTLE_MS;
  if (crossing || race.launchCurveLimit != 32 || race.exitQuietMs < 40 ||
    abs(race.filteredCurve) >= 32 || abs(error) >= (holding ? 180 : 160) ||
    abs(delta) >= (holding ? 80 : 60) || abs(error + 3 * delta) >= (holding ? 240 : 200))
  { race.straightStableMs = 0; }
  else if (!holding)
  {
    const uint16_t stable = race.straightStableMs + (dt > 10 ? 10 : dt);
    race.straightStableMs = stable < Settings::STRAIGHT_SETTLE_MS ? stable : Settings::STRAIGHT_SETTLE_MS;
  }
}

__attribute__((noinline)) void followLine(uint32_t now)
{
  const uint32_t elapsed = now - lastControl;
  const bool fastReplay = race.mapReady && race.mode == BEAST && !race.noisyLearning;
  if (elapsed < (fastReplay ? Settings::FAST_CONTROL_MS : Settings::CONTROL_MS)) { return; }
  if (elapsed > 50)
  {
    stopRobot(F("Loop lag"));
    return;
  }
  lastControl = now;
  const int16_t position = lineSensors.readLineBlack(sensorValues);
  // The 50 ms loop bound plus the 2 ms sensor timeout fit a 16-bit us clock.
  const uint16_t sensorNow = micros();
  uint16_t sensorElapsed = sensorNow - lastSensorMicros;
  lastSensorMicros = sensorNow;
  if (!sensorElapsed) { sensorElapsed = elapsed * 1000; }
  if (peakReading() < Settings::LINE_THRESHOLD)
  {
    stopRobot(F("No line"));
    return;
  }
  uint8_t darkCount = 0;
  for (uint8_t i = 0; i < 5; i++)
  {
    if (sensorValues[i] > 750) { darkCount++; }
  }
  const bool crossing = darkCount >= 4;
  if (crossing)
  {
    if (!darkActive) { darkSince = now; darkActive = true; }
    if (now - darkSince >= 200)
    {
      stopRobot(F("Dark/air"));
      return;
    }
  }
  else { darkActive = false; }

  const int16_t error = position - 2000;
  // Use acquisition timestamps: ADC and sensor reads can shift within a tick.
  const int32_t slope = static_cast<int32_t>(error - lastError) *
    (Settings::CONTROL_MS * 1000L) / static_cast<int32_t>(sensorElapsed);
  const int16_t normalizedDelta = slope > 8000 ? 8000 : (slope < -8000 ? -8000 : slope);
  const int16_t dt = elapsed;
  // Preserve the 10 ms response while keeping a similar time constant at 5 ms.
  if (dt >= 30) { filteredDelta = normalizedDelta; }
  else
  {
    filteredDelta += static_cast<int32_t>(normalizedDelta - filteredDelta) *
      (2 * dt) / (30 + dt);
  }
  lastError = error;
  const int16_t steerError = error > Settings::STEERING_DEADBAND ? error - Settings::STEERING_DEADBAND :
    (error < -Settings::STEERING_DEADBAND ? error + Settings::STEERING_DEADBAND : 0);
  const int16_t steerDelta = filteredDelta > 12 ? filteredDelta - 12 :
    (filteredDelta < -12 ? filteredDelta + 12 : 0);
  int16_t correction = 0;
  if (!checkEncoders(now)) { return; }
  float turn = 0;
  if (!readTurnDelta(turn)) { return; }
  race.noteTracking(error, filteredDelta, crossing); // Include the lap's final sample.
  const RaceSession::Event event = race.update(latestLeftCounts, latestRightCounts,
    turn, static_cast<uint16_t>(elapsed), (leftCommand + rightCommand) / 2, now);
  if (race.fault != RaceSession::NONE)
  {
    if (race.fault == RaceSession::MAP_FULL) { stopRobot(F("Map full")); }
    else if (race.fault == RaceSession::MAP_GAP) { stopRobot(F("Map gap")); }
    else if (race.fault == RaceSession::MAP_MISMATCH) { stopRobot(F("Map mismatch")); }
    else if (race.fault == RaceSession::NO_FINISH) { stopRobot(F("No finish")); }
    else { stopRobot(F("Motion error")); }
    return;
  }
  if (event == RaceSession::SESSION_FINISHED) { showResults(); return; }
  uint8_t yawWeight = 0;
  if (!crossing)
  {
    const int16_t assist = race.steeringFeedforward((leftCommand + rightCommand) / 2, error, &yawWeight);
    const int16_t derivativeGain = fastReplay && yawWeight == 2 &&
      abs(error) < 160 && abs(filteredDelta) < 60 ? Settings::STRAIGHT_KD_256 : Settings::KD_256;
    correction = (static_cast<int32_t>(steerError) * Settings::KP_256 +
      static_cast<int32_t>(steerDelta) * derivativeGain) / 256;
    correction += assist;
    if (yawWeight)
    {
      // Dampen actual yaw with the gyro, relative to the confirmed mapped turn.
      // Optical derivative noise must not be the only source of yaw damping.
      const uint16_t yawScale = race.yawScale();
      // Use measured rate directly: gyro integration and loop millis have
      // different timestamps, which otherwise modulate damping on short ticks.
      int16_t measuredTurn = (imu.g.z - gyroBias) *
        (0.07f * 3.14159265f / 180000.0f) * yawScale;
      if (race.turnVotes < 0) { measuredTurn = -measuredTurn; }
      // Let an optically requested turn develop before the map catches up.
      // Keep the existing recovery damping when yaw goes the wrong way.
      if (abs(error) >= 400 ||
        (fastReplay && abs(error) >= 160 && measuredTurn && (error < 0) == (measuredTurn < 0))) { yawWeight = 1; }
      // Restore R24 damping around mapped yaw; keep optical PD at its tuned gain.
      const int16_t damping = assist - measuredTurn;
      correction += damping * yawWeight / 2;
    }
  }
  const int16_t steeringLimit = race.steeringLimit();
  if (correction > steeringLimit) { correction = steeringLimit; }
  if (correction < -steeringLimit) { correction = -steeringLimit; }
  int16_t requested = baseSpeedForLine(error, filteredDelta, crossing);
  qualifyStraight(error, filteredDelta, crossing, dt);
  const int16_t ceiling = race.profile().limit;
  const int16_t steeringBase = ceiling - abs(correction);
  if (!fastReplay && requested > steeringBase)
  { requested = steeringBase; speedLimiter = STEER_LIMIT; }
  // Ramp the common throttle, not each wheel: alternating corrections must not
  // ratchet both wheels down. Large tracking errors bypass routine soft braking.
  int16_t change = requested - forwardCommand;
  // Boost only after continuous optical/yaw alignment on a mapped straight.
  const int16_t acceleration = accelerationStep();
  // Carry fractional PWM across short ticks: 5 ms braking alternates 1 and 2,
  // retaining 300 PWM/s instead of silently truncating it to 200 PWM/s.
  int16_t budget = (change > 0 ? acceleration : -Settings::DRIVE_BRAKE_STEP) * dt;
  if ((budget > 0) == (throttleRemainder > 0)) { budget += throttleRemainder; }
  const int16_t ramp = budget / static_cast<int16_t>(Settings::CONTROL_MS);
  throttleRemainder = budget % static_cast<int16_t>(Settings::CONTROL_MS);
  if (change > 0 && change > ramp) { change = ramp; }
  else if (change < 0 && change < ramp) { change = ramp; }
  else { throttleRemainder = 0; }
  forwardCommand += change;
  if ((crossing || abs(error) >= 700 || abs(filteredDelta) >= 250) && forwardCommand > requested)
  { forwardCommand = requested; throttleRemainder = 0; }
  if (forwardCommand > peakThrottlePwm) { peakThrottlePwm = forwardCommand; }
  int16_t base = forwardCommand;
  // At full Beast throttle, reserve steering immediately by holding the outer
  // wheel at its ceiling. Do not feed this transient trim into the throttle
  // ramp: alternating corrections must not ratchet the next command down.
  if (fastReplay && base > steeringBase) { base = steeringBase; }
  // Steering unwind must not release hidden common throttle in one jump.
  const int16_t releaseLimit = (leftCommand + rightCommand) / 2 + acceleration * dt / Settings::CONTROL_MS;
  if (fastReplay && base > releaseLimit) { base = releaseLimit; }
  const int16_t room = base < ceiling - base ? base : ceiling - base;
  if (correction > room) { correction = room; }
  if (correction < -room) { correction = -room; }
  leftCommand = base + correction;
  rightCommand = base - correction;
  if (base > peakDrivePwm) { peakDrivePwm = base; }
  if (race.mapReady)
  {
    if (base < forwardCommand) { speedLimiter = base < steeringBase ? ACCEL_LIMIT : STEER_LIMIT; }
    else if (forwardCommand < requested) { speedLimiter = ACCEL_LIMIT; }
    limiterTicks[speedLimiter]++;
  }
  motors.setSpeeds(leftCommand, rightCommand);
}

void setup()
{
  MCUSR = 0;
  wdt_disable();
  watchdogEnabled = false;
  motors.setSpeeds(0, 0);
  lineSensors.setTimeout(Settings::SENSOR_TIMEOUT_US);
  // Keep the bumper untouched during startup so its baseline is meaningful.
  bumpSensors.calibrate();
  encoders.init();
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(2500, true);
  Wire.clearWireTimeoutFlag();
  imuReady = imu.init();
  if (imuReady)
  {
    // Gyro only: auto-increment and block-data-update prevent torn axis reads.
    imu.writeReg(LSM6DS33_ADDR, LSM6DS33_REG_CTRL3_C, 0x44);
    imuReady = imu.getLastError() == 0;
  }
  if (imuReady)
  {
    imu.configureForTurnSensing();
    imuReady = imu.getLastError() == 0 && !Wire.getWireTimeoutFlag();
  }
  display.noAutoDisplay();
  ledRed(false);
  ledGreen(false);
  ledYellow(false);
  showStatus(F("3pi+ R42"), buttonMenu());
  lastIdleUpdate = millis() - 500;
  showIdleInfo(millis());
  lastButtonHeld = millis();
}

void serviceWatchdog()
{
  if (!usbPowerPresent())
  {
    if (!watchdogEnabled)
    {
      wdt_enable(WDTO_250MS);
      watchdogEnabled = true;
    }
    wdt_reset();
  }
  else if (Serial.baud() != 1200 && watchdogEnabled)
  {
    wdt_disable();
    watchdogEnabled = false;
  }
  // Arduino uses its own watchdog reset for a 1200-baud upload request.
  // When USB is at 1200 baud, neither feed nor reconfigure that watchdog.
}

void loop()
{
  serviceWatchdog();
  const uint32_t now = millis();
  const bool a = buttonA.isPressed();
  const bool b = buttonB.isPressed();
  const bool c = buttonC.isPressed();
  const bool anyButton = a || b || c;
  if (anyButton) { lastButtonHeld = now; }

  if (state == IDLE)
  {
    if (!anyButton && now - lastButtonHeld >= 60) { buttonsArmed = true; }
    if (anyButton && buttonsArmed)
    {
      buttonsArmed = false;
      runDiagnostics = false;
      if (c) { selectNextMode(); }
      else if (a) { beginCalibration(); }
      else if (b) { requestStart(); }
    }
    if (state == IDLE)
    {
      if (race.completed == 3 || runDiagnostics) { return; }
      if (calibrated) { showSensorPreview(now); }
      else { showIdleInfo(now); }
    }
    return;
  }
  if (state == WAIT_RELEASE)
  {
    if (c || (calibrationRequested ? b : a))
    {
      stopRobot(stoppedMessage());
      return;
    }
    if (!driveInterlocks()) { return; }
    if (!anyButton && now - lastButtonHeld >= 60)
    {
      state = COUNTDOWN;
      stateSince = now;
      countdownSeconds = 0;
      if (!calibrationRequested) { startGyroCalibration(); }
      showCountdown(now);
    }
    return;
  }
  if (anyButton) { stopRobot(stoppedMessage()); return; }
  if (!driveInterlocks()) { return; }
  if (now - lastBattery >= 100 && !batteryOK()) { return; }
  if (state == COUNTDOWN)
  {
    if (!calibrationRequested && !sampleGyroCalibration(now)) { return; }
    if (now - stateSince >= 2000)
    {
      if (calibrationRequested) { startCalibrationMotion(); }
      else { beginRun(); }
    }
    else { showCountdown(now); }
    return;
  }
  if (state == CALIBRATING)
  {
    calibrateAutomatically(now);
    return;
  }
  if (now - stateSince >= Settings::MAX_RUN_MS)
  {
    stopRobot(F("Time end"));
    return;
  }
  followLine(now);
}
