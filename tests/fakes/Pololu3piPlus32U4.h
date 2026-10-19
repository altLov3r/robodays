#pragma once
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <string>

using __FlashStringHelper = char;
#define F(value) value

namespace Fake
{
static uint32_t now = 0;
static uint32_t microOffsetUs = 0;
static bool buttons[3] = {false, false, false};
static bool usb = false;
static uint16_t battery = 5000;
static uint8_t bump = 0;
static uint16_t readings[5] = {0, 0, 1000, 0, 0};
static uint16_t position = 2000;
static bool goodCalibration = true;
static bool scriptedCalibration = false;
static uint16_t calibrationReadings[10][5] = {};
static unsigned calibrationReads = 0;
static int16_t leftCounts = 10;
static int16_t rightCounts = 10;
static bool directCounts = false;
static bool encoderErrors[2] = {false, false};
static int16_t leftMotor = 0;
static int16_t rightMotor = 0;
static unsigned motorCalls = 0;
static unsigned nonzeroMotorCalls = 0;
static unsigned watchdogResets = 0;
static unsigned watchdogChanges = 0;
static uint32_t serialBaud = 57600;
static bool imuConnected = true, gyroFresh = true, wireTimeout = false;
static int16_t gyroRaw = 0;
static uint8_t imuError = 0;
static bool greenLed = false;
static std::string message;
static std::string displayRows[8];
static unsigned displayWrites = 0;
static bool motorSignChangedWithoutStop = false;
}

static uint8_t MCUSR = 0;
inline uint32_t millis() { return Fake::now; }
inline uint32_t micros() { return Fake::now * uint32_t(1000) + Fake::microOffsetUs; }
struct FakeSerial
{
  uint32_t baud() { return Fake::serialBaud; }
};
static FakeSerial Serial;

namespace Pololu3piPlus32U4
{
struct OLED
{
  uint8_t x = 0, y = 0;
  bool automatic = true;
  void clear()
  {
    Fake::message.clear();
    for (auto & row : Fake::displayRows) { row.assign(21, ' '); }
    x = y = 0;
  }
  void gotoXY(uint8_t column, uint8_t row)
  {
    assert(column < 21 && row < 8);
    x = column;
    y = row;
    Fake::message += "|";
  }
  void print(const char * text)
  {
    Fake::message += text;
    while (*text)
    {
      assert(x < 21 && y < 8);
      Fake::displayRows[y][x++] = *text++;
    }
  }
  void print(char value) { const char text[] = {value, 0}; print(text); }
  void print(int value) { print(std::to_string(value).c_str()); }
  void print(unsigned int value) { print(std::to_string(value).c_str()); }
  void display() { Fake::displayWrites++; automatic = true; }
  void displayPartial(uint8_t row, uint8_t column, uint8_t width)
  {
    assert(row < 8 && column + width <= 21);
    Fake::displayWrites++;
  }
  void setLayout21x8() {}
  void noAutoDisplay() { automatic = false; }
};
using LCD = OLED;
struct Motors
{
  void setSpeeds(int16_t left, int16_t right)
  {
    assert(left >= -60 && left <= 400);
    assert(right >= -60 && right <= 400);
    if (left < 0 || right < 0)
    {
      assert(left <= 60 && right <= 60 && int32_t(left) * right <= 0);
    }
    if ((int32_t(left) * Fake::leftMotor < 0) ||
      (int32_t(right) * Fake::rightMotor < 0))
    {
      Fake::motorSignChangedWithoutStop = true;
    }
    Fake::leftMotor = left;
    Fake::rightMotor = right;
    Fake::motorCalls++;
    if (left || right) { Fake::nonzeroMotorCalls++; }
  }
};
struct Encoders
{
  void init() {}
  int16_t getCountsAndResetLeft()
  { return Fake::directCounts ? Fake::leftCounts : Fake::leftCounts * Fake::leftMotor / 60; }
  int16_t getCountsAndResetRight()
  { return Fake::directCounts ? Fake::rightCounts : Fake::rightCounts * Fake::rightMotor / 60; }
  bool checkErrorLeft()
  {
    const bool value = Fake::encoderErrors[0];
    Fake::encoderErrors[0] = false;
    return value;
  }
  bool checkErrorRight()
  {
    const bool value = Fake::encoderErrors[1];
    Fake::encoderErrors[1] = false;
    return value;
  }
};
struct LineSensors
{
  struct CalibrationData
  {
    bool initialized = false;
    uint16_t * minimum = nullptr;
    uint16_t * maximum = nullptr;
  } calibrationOn, calibrationOff;
  ~LineSensors()
  {
    // Match the real library's ownership contract at derived-object teardown.
    free(calibrationOn.minimum);
    free(calibrationOn.maximum);
    free(calibrationOff.minimum);
    free(calibrationOff.maximum);
  }
  void setTimeout(uint16_t) {}
  void read(uint16_t * values)
  {
    for (uint8_t i = 0; i < 5; i++)
    {
      values[i] = Fake::scriptedCalibration ?
        Fake::calibrationReadings[Fake::calibrationReads % 10][i] :
        (Fake::goodCalibration && (Fake::calibrationReads / 10) % 2 ? 1500 : 100);
    }
    Fake::calibrationReads++;
  }
  uint16_t readLineBlack(uint16_t * values)
  {
    for (uint8_t i = 0; i < 5; i++) { values[i] = Fake::readings[i]; }
    return Fake::position;
  }
};
struct BumpSensors
{
  void calibrate() {}
  uint8_t read() { return Fake::bump; }
};
struct ButtonA { bool isPressed() { return Fake::buttons[0]; } };
struct ButtonB { bool isPressed() { return Fake::buttons[1]; } };
struct ButtonC { bool isPressed() { return Fake::buttons[2]; } };
inline void ledRed(bool) {}
inline void ledGreen(bool on) { Fake::greenLed = on; }
inline void ledYellow(bool) {}
inline bool usbPowerPresent() { return Fake::usb; }
inline uint16_t readBatteryMillivolts() { return Fake::battery; }
}
