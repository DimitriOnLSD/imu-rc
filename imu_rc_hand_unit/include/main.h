#ifndef MAIN_H
#define MAIN_H

#ifdef DEV_BOARD
#define PIN_NEOPIXEL 48     // LED_BUILTIN
#define PIN_BTN_U 15        // K1
#define PIN_BTN_D 7         // K2
#define PIN_BTN_S 6         // K3
#define PIN_BTN_B 5         // K4
#define PIN_BATTERY_READ 16 // ADC2_5
#define PIN_BATTERY_ENABLE_READ 17
#define PIN_CHARGER_STATUS 18
#else
#define PIN_RGB_RED 6
#define PIN_RGB_GREEN 5
#define PIN_RGB_BLUE 4
#define PIN_BTN_B 18
#define PIN_BTN_U 15
#define PIN_BTN_S 17
#define PIN_BTN_D 16
#define PIN_BATTERY_READ 23 // won't work in v1.1
#define PIN_BATTERY_ENABLE_READ 24 // won't work in v1.1
#define PIN_CHARGER_STATUS 7 // won't work in v1.1
#endif
///////////////////////////////////////////////////////////////////////////////
#define THRESHOLD_MIN_STEER 8.0
#define THRESHOLD_MIN_ROTATE 12.0 
#define THRESHOLD_MAX_ROTATE 45.0
#define THRESHOLD_MIN 20.0
#define THRESHOLD_MAX 90.0
///////////////////////////////////////////////////////////////////////////////
#define DUTY_CYCLE_MIN 175
#define DUTY_CYCLE_MAX 255

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include "I2Cdev.h"
#include "SparkFunLSM6DSO.h"

#ifdef DEV_BOARD
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

// BLE variables
String dataToSend;
uint8_t serverBatteryPercentage = 0;
bool clientDisconnected = false;      // Flag to indicate a previous disconnection from the client
bool clientIsConnected = false;       // Flag to indicate a connection
bool clientLostConnection = false;    // Flag to indicate a lost connection
bool clientConnectionAttempt = false; // Flag to indicate a connection attempt
bool clientIsScanning = false;        // Flag to indicate the client is scanning

// Car logic variables
uint8_t controlType = 0;    // Control type for the car
bool carIsStopped = true;   // Flag to indicate the car is stopped
bool carIsRotating = false; // Flag to indicate the car is rotating
bool canMoveXAxis = false;  // Flag to enable movement in the X-axis
bool canMoveYAxis = true;   // Flag to enable movement in the Y-axis
bool movingXAxis = false;   // Flag to indicate movement in the X-axis
bool movingYAxis = false;   // Flag to indicate movement in the Y-axis
float sensitivity = 1.0f;   // Sensitivity for the inertial sensor
const float sens_max = 1.5f;
const float sens_min = 1.0f;

// Handheld device variables
uint8_t battPercentage = 0;               // Battery percentage
bool readDataWithScreen = false;          // Flag to read data with the screen

void debugPrint(bool printData, double yaw, double pitch, double roll, String data);
void setupSerial();
void setupI2C();
double constraintAngle(double angle, double limit);
bool positiveTilt(double var);
bool negativeTilt(double var);
void motorCommand(uint8_t leftSpeed, uint8_t rightSpeed, String direction);
void functionCommand(String command);
float increase(float value, float increment, float limit);
float decrease(float value, float decrement, float limit);

#include "timer.h"
#include "led.h"
#include "menu.h"
#include "ble.h"
#include "batt.h"
#include "imu.h"
#include "int.h"

#endif
