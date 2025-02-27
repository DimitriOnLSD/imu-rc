#ifndef MAIN_H
#define MAIN_H

#define SERVICE_UUID BLEUUID("3e3f1b05-90a8-43af-aa6b-335b28282b57")
#define CHARACTERISTIC_UUID BLEUUID("3e2c8a88-90fd-4b6b-b947-997e80d03d5b")
///////////////////////////////////////////////////////////////////////////////
#define SCAN_DURATION 5            // Seconds
#define DISPLAY_MENU_FREQUENCY 10  // Hz
#define BLE_DATA_FREQUENCY 20
#define BATT_DATA_FREQUENCY 0.2
///////////////////////////////////////////////////////////////////////////////
#if (DEV_BOARD == true)
#define PIN_NEOPIXEL 48 // LED_BUILTIN
#define PIN_BUTTON_UP 5 // K1
#define PIN_BUTTON_DOWN 6 // K2
#define PIN_BUTTON_SELECT 7 // K3
#define PIN_BUTTON_BACK 15 // K4
#define PIN_BATTERY_READ 16 // ADC2_5
#define PIN_BATTERY_ENABLE_READ 17
#define PIN_CHARGER_STATUS 18
#else
#define PIN_RGB_RED 6
#define PIN_RGB_GREEN 5
#define PIN_RGB_BLUE 4
#define PIN_BUTTON_BACK 15
#define PIN_BUTTON_UP 16
#define PIN_BUTTON_SELECT 17
#define PIN_BUTTON_DOWN 18
#define PIN_BATTERY_READ 17 // ADC2_6
#define PIN_BATTERY_ENABLE_READ 21
#define PIN_CHARGER_STATUS 7
#endif
///////////////////////////////////////////////////////////////////////////////
#define THRESHOLD_MIN_STEER 5.0
#define THRESHOLD_MIN 10.0  // Threshold for the angular position. Variable in Euler angles
#define THRESHOLD_MAX 40.0
///////////////////////////////////////////////////////////////////////////////
#define DUTY_CYCLE_MIN 170  // PWM duty cycle
#define DUTY_CYCLE_MAX 255
///////////////////////////////////////////////////////////////////////////////
#define OLED_SCREEN_WIDTH 128  // Pixels
#define OLED_SCREEN_HEIGHT 64
///////////////////////////////////////////////////////////////////////////////
#define OLED_RESET -1
#define OLED_SCREEN_ADDRESS 0x3C
///////////////////////////////////////////////////////////////////////////////
#define BATTERY_POS_X 100  // Pixels
#define BATTERY_POS_Y 0
#define BATTERY_WIDTH 23
#define BATTERY_HEIGHT 12
#define BATTERY_PADDING 2
#define BATTERY_BAR_WIDTH 4
#define BATTERY_BAR_HEIGHT 8
#define BATTERY_NOTCH_W 2
#define BATTERY_NOTCH_H 6
///////////////////////////////////////////////////////////////////////////////
#define RSSI_NUM_BARS 4  // Pixels
#define RSSI_BAR_POS_X 0
#define RSSI_BAR_POS_Y 12
#define RSSI_BAR_WIDTH 4
#define RSSI_BAR_SPACING 1
///////////////////////////////////////////////////////////////////////////////
#define SNSV_NUM_BARS 20
#define SNSV_BAR_POS_X 16
#define SNSV_BAR_POS_Y 48
#define SNSV_BAR_WIDTH 4
#define SNSV_BAR_SPACING 1
///////////////////////////////////////////////////////////////////////////////
#define TOP_BAR_LINE_X 0  // Pixels
#define TOP_BAR_LINE_Y 14
#define TOP_BAR_LINE_W 128
#define TOP_BAR_LINE_H 1

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
#include "MPU6050_6Axis_MotionApps20.h"

#if (DEV_BOARD == true)
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif

// Define the display, Bluetooth Low-Energy and IMU Sensor objects
Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET);
BLEScan *pBLEScan;
BLEClient *pClient;
BLERemoteCharacteristic *pRemoteCharacteristic;
MPU6050 mpu;

bool clientDisconnected = false;
bool clientIsConnected = false;
bool clientLostConnection = false;
bool clientConnectionAttempt = false;
bool clientIsScanning = false;
bool printOnce = true;
bool carIsStopped = true;    // Flag to indicate the car is stopped
bool carIsRotating = false;  // Flag to indicate the car is rotating
bool canMoveXAxis = false;   // Flag to enable movement in the X-axis
bool canMoveYAxis = true;    // Flag to enable movement in the Y-axis
bool movingXAxis = false;    // Flag to indicate movement in the X-axis
bool movingYAxis = false;    // Flag to indicate movement in the Y-axis

uint8_t barsBattery = 0;
uint8_t RSSI = 0;

String dataToSend;
uint8_t serverBatteryPercentage = 0;
uint8_t controlType = 0;  // Control type for the car

float sensitivity = 1.0f;  // Sensitivity for the inertial sensor
const float sens_max = 2.0f;
const float sens_min = 0.1f;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
bool resetYPR = false;   // Flag for resetting Yaw/Pitch/Roll
uint8_t MPUIntStatus;    // Holds actual interrupt status byte from MPU
uint8_t devStatus;       // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64];  // FIFO storage buffer

/*---Orientation/Motion Variables---*/
Quaternion q;         // [w, x, y, z]         Quaternion container
VectorInt16 aa;       // [x, y, z]            Accel sensor measurements
VectorInt16 gy;       // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            Gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
double yaw;
double pitch;
double roll;

/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Function to print data to the serial monitor
void debugPrint(bool printData, double yaw, double pitch, double roll, String data) {
#if (DEBUG == true)
  if (printData) {
    Serial.print("ypr\t");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.println(data);
  } else {
    Serial.println(data);
  }
#endif
}

// Function to setup the serial monitor
void setupSerial() {
#if (DEBUG == true)
  Serial.begin(115200);
#endif
}

// Function to setup the I2C communication
void setupI2C() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// Function to constrain the angle
double constraintAngle(double angle, double limit) {
  if (angle > limit) {
    return limit;
  } else if (angle < -limit) {
    return -limit;
  } else {
    return angle;
  }
}

// Function to check if the tilt is positive or negative
bool positiveTilt(double var) {
  return (var > THRESHOLD_MIN);
}
bool negativeTilt(double var) {
  return (var < -THRESHOLD_MIN);
}

// Function to send motor commands to the server
void motorCommand(uint8_t leftSpeed, uint8_t rightSpeed, String direction) {
  dataToSend = direction + "," + leftSpeed + "," + rightSpeed;
}
void functionCommand(String command) {
  dataToSend = command;
}

// Function to increase or decrease a value
float increase(float value, float increment, float limit) {
  value += increment;
  if (value > limit) {
    value = limit;
  }
  return value;
}
float decrease(float value, float decrement, float limit) {
  value -= decrement;
  if (value < limit) {
    value = limit;
  }
  return value;
}

#include "led.h"
#include "menu.h"
#include "ble.h"
#include "batt.h"
#include "imu.h"

#endif