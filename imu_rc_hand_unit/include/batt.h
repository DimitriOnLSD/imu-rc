#ifndef BATT_H
#define BATT_H

void setupBattery();

uint16_t enableBattRead();

double batteryVoltage(uint16_t analogBits);

uint8_t batteryPercentage(double voltage);

uint8_t batteryBars(uint16_t analogBits);

bool checkCharging();

#include "batt.cpp"

#endif
