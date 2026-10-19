#pragma once
#define WDTO_250MS 4
inline void wdt_disable() { Fake::watchdogChanges++; }
inline void wdt_enable(int) { Fake::watchdogChanges++; }
inline void wdt_reset() { Fake::watchdogResets++; }
