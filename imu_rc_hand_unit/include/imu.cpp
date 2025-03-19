void setupMPU6050() {
  // Initialize the MPU6050
  debugPrint(false, 0, 0, 0, "Initializing I2C for MPU6050...");
  mpu.initialize();

  // Verify MPU6050 connection
  debugPrint(false, 0, 0, 0, "Testing MPU6050 connections...");
  if (mpu.testConnection() == false) {
    debugPrint(false, 0, 0, 0, "MPU6050 connection failed");
    while (true)
      ;
  } else {
    debugPrint(false, 0, 0, 0, "MPU6050 connection successful");
  }

  // Initializate and configure the DMP
  debugPrint(false, 0, 0, 0, "Initializing DMP...");
  mpu.dmpInitialize();

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  DMPReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();  // Get expected DMP packet size for later comparison
}

// Interrupt detection routine
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

// Function to reset the MPU6050
bool resetMPU() {
  debugPrint(false, 0, 0, 0, "Resetting MPU6050...");
  q = Quaternion(1, 0, 0, 0);
  gravity = VectorFloat(0, 0, 0);

  DMPReady = false;
  mpu.dmpInitialize();

  // Reset offsets and re-enable DMP
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);

  DMPReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  if (packetSize == -1)
    return false;
  return true;
}