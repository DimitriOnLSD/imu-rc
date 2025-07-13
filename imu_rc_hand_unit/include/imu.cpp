void setupLSM6DSO() {
  debugPrint(false, 0, 0, 0, "Initializing I2C for LSM6DSO...");

  Wire.begin();

  if (imu.begin()) {
    debugPrint(false, 0, 0, 0, "LSM6DSO connection successful");
    imuReady = true;
  } else {
    debugPrint(false, 0, 0, 0, "LSM6DSO connection failed");
    while (true);  // Stop execution if sensor fails to initialize
  }

  // Load default/basic settings
  if (imu.initialize(BASIC_SETTINGS)) {
    debugPrint(false, 0, 0, 0, "LSM6DSO settings loaded");
  } else {
    debugPrint(false, 0, 0, 0, "LSM6DSO failed to load settings");
  }
}

bool resetLSM() {
  debugPrint(false, 0, 0, 0, "Resetting LSM6DSO...");

  imuReady = false;

  // Try reinitializing the sensor
  if (!imu.begin()) {
    debugPrint(false, 0, 0, 0, "LSM6DSO re-init failed");
    return false;
  }

  if (!imu.initialize(BASIC_SETTINGS)) {
    debugPrint(false, 0, 0, 0, "LSM6DSO failed to reload settings");
    return false;
  }

  imuReady = true;
  return true;
}

bool readLSM6DSO() {
  if (!imuReady) return false;

  accelX = imu.readFloatAccelX();
  accelY = imu.readFloatAccelY();
  accelZ = imu.readFloatAccelZ();

  pitch = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  roll  = atan2(accelY, accelZ) * 180 / PI;
  yaw = 0.0;

  return true;
}