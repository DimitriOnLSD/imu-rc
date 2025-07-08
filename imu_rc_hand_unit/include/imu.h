#ifndef IMU_H
#define IMU_H

#include "SparkFunLSM6DSO.h"
#include "Wire.h"

// LSM6DSO sensor object (using I2C with default address 0x6B)
LSM6DSO imu;

// Sensor status
bool imuReady = false;

// Reading variables
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float tempF;

// Yaw, pitch, and roll (manually calculated if desired)
double yaw = 0.0;
double pitch = 0.0;
double roll = 0.0;

// Sensor initialization
void setupLSM6DSO();
bool resetLSM();
bool readLSM6DSO();

#include "imu.cpp"

#endif
