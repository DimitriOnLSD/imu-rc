#ifndef INT_H
#define INT_H

char input;                               // Input character
volatile bool buttonPress = false;        // Flag for any button press
volatile bool buttonPressed[4] = {false}; // Flags for specific button presses

#include "int.cpp"

#endif
