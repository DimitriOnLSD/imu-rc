#ifndef TIMER_H
#define TIMER_H

#include <Ticker.h>

#define UPDATE_RATE_MENU 10  // Hz
#define UPDATE_RATE_IMU 50   // Hz
#define UPDATE_RATE_BATT 0.2 // Hz
#define UPDATE_RATE_RSSI 5   // Hz
#define SEND_RATE_BLE 20     // Hz

Ticker menuTicker;
Ticker imuTicker;
Ticker battTicker;
Ticker bleTicker;
Ticker rssiTicker;

bool updateBattery = false;
bool updateDisplay = false;
bool updateData = false;
bool sendData = false;
bool updateRSSI = false;

#include "timer.cpp"

#endif