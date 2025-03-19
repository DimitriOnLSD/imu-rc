#ifndef LED_H
#define LED_H

/*
OFF         : No LED color
BATTERY     : Red
LOW BATTERY : Red blinking
CHARGING    : Yellow
CHARGED     : Green
SCANNING    : Blue blinking
PAIRING     : Blue
IMU RESET   : White blinking
*/

void setupLED();
void setColor(const char *color);
void SetLEDStatus(const char *color, const char *mode);

#include "led.cpp"

#endif
