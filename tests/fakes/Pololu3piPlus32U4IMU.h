#pragma once
#include "Pololu3piPlus32U4.h"
#define LSM6DS33_ADDR 0x6B
#define LSM6DS33_REG_CTRL3_C 0x12

struct FakeWire
{
  void begin() {}
  void setClock(uint32_t) {}
  void setWireTimeout(uint32_t, bool) {}
  bool getWireTimeoutFlag() { return Fake::wireTimeout; }
  void clearWireTimeoutFlag() { Fake::wireTimeout = false; }
};
static FakeWire Wire;

namespace Pololu3piPlus32U4
{
struct IMU
{
  struct { int16_t x = 0, y = 0, z = 0; } g;
  bool init() { return Fake::imuConnected; }
  void enableDefault() {}
  void writeReg(uint8_t, uint8_t, uint8_t) {}
  void configureForTurnSensing() {}
  bool gyroDataReady() { return Fake::gyroFresh; }
  void readGyro() { g.z = Fake::gyroRaw; }
  uint8_t getLastError() { return Fake::imuError; }
};
}
