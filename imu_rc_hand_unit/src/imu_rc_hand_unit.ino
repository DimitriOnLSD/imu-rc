#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// UUIDs of the BLE server's service and characteristic we want to connect to
#define SERVICE_UUID BLEUUID("3e3f1b05-90a8-43af-aa6b-335b28282b57")
#define CHARACTERISTIC_UUID BLEUUID("3e2c8a88-90fd-4b6b-b947-997e80d03d5b")

#define INTERRUPT_PIN 2
#define THRESHOLD 10.0  // Ângulo de inclinação para acionar o movimento
#define THRESHOLD_STEERING 5.0
#define THRESHOLD_MAX 40.0
#define BLE_DATA_FREQUENCY 20  // Frequência de envio de dados em Hz
#define SPEED_MAX 255
#define SPEED_MIN 170

#define DEBUG true

BLEScan *pBLEScan;
BLEClient *pClient;
BLERemoteCharacteristic *pRemoteCharacteristic;
MPU6050 mpu;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;   // Set true if DMP init was successful
bool resetYPR = false;   // Flag para controlar o reset
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

/*-Packet structure for InvenSense teapot demo-*/
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*---Variables for BLE communication---*/
String dataToSend;
bool stopped = true;        // Flag to indicate the car is stopped
bool rotating = false;      // Flag to indicate the car is rotating
bool canMoveXAxis = false;  // Flag to enable movement in the X-axis
bool canMoveYAxis = true;   // Flag to enable movement in the Y-axis
bool movingXAxis = false;   // Flag to indicate movement in the X-axis
bool movingYAxis = false;   // Flag to indicate movement in the Y-axis

/*---Function to limit the angle---*/
double constraintAngle(double angle, double limit) {
  if      (angle > limit)  { return limit;  } 
  else if (angle < -limit) { return -limit; } 
  else                     { return angle;  }
}

/*---Function prototypes---*/
bool positiveTilt(double var) { return (var > THRESHOLD); }
bool negativeTilt(double var) { return (var < -THRESHOLD);}

/*---Function to set the speed of the motors---*/
void motorCommand(uint8_t leftSpeed, uint8_t rightSpeed, String direction) { dataToSend = direction + "," + leftSpeed + "," + rightSpeed; }
void functionCommand(String command) { dataToSend = command; }

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() { MPUInterrupt = true;}

// Callback function for notifications
static void notifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  uint8_t *pData,
  size_t length,
  bool isNotify) {

  String notificationValue = "";
  for (size_t i = 0; i < length; i++) {
    notificationValue += (char)pData[i];
  }
#if (DEBUG == true)
  Serial.print("Notification received: ");
  Serial.println(notificationValue);
#endif
}

// Function to connect to the server and return the connected BLE client
BLEClient *connectToServer(BLEAddress serverAddress) {
  // Create a new BLE client instance
  pClient = BLEDevice::createClient();

  // Attempt to connect to the server using its address
#if (DEBUG == true)
  Serial.print("Connecting to IMU-RC Car: ");
  Serial.println(serverAddress.toString().c_str());
#endif
  if (pClient->connect(serverAddress)) {
#if (DEBUG == true)
    Serial.println("Connected to IMU-RC Car.");
#endif
  } else {
#if (DEBUG == true)
    Serial.println("Failed to connect to IMU-RC Car.");
#endif
    return nullptr;
  }

  // Check if the service exists on the server
  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
#if (DEBUG == true)
    Serial.println("Failed to find IMU-RC Car service.");
#endif
    pClient->disconnect();
    return nullptr;
  }

  // Get the characteristic from the service
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
#if (DEBUG == true)
    Serial.println("Failed to find IMU-RC Car characteristic.");
#endif
    pClient->disconnect();
    return nullptr;
  }

  // Register for notifications
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
#if (DEBUG == true)
    Serial.println("Registered for notifications.");
#endif
  }

  // Return the connected client
  return pClient;
}

// Scan for devices
void scanForServer() {
#if (DEBUG == true)
  Serial.println("Scanning for IMU-RC Car servers...");
#endif

  // Start BLE scan and look for devices
  BLEScanResults results = pBLEScan->start(5);
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);

    // Check if the device advertises the service UUID we're looking for
    if (device.haveServiceUUID() && device.isAdvertisingService(SERVICE_UUID)) {
#if (DEBUG == true)
      Serial.println("Found IMU-RC Car. Attempting to connect...");
#endif

      // Connect to the server
      BLEClient *pClient = connectToServer(device.getAddress());
      if (pClient != nullptr) {
        break;  // If connection successful, exit scan loop
      }
    }
  }

  pBLEScan->clearResults();  // Clear scan results
}

void resetMPU() {
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
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

#if (DEBUG == true)
  Serial.begin(115200);
  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
#endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);

#if (DEBUG == true)
  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
#endif
  devStatus = mpu.dmpInitialize();

  /* Making sure it worked (returns 0 if so) */
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);

    mpu.setDMPEnabled(true);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();  // Get expected DMP packet size for later comparison
  }

  // Initialize BLE device as a client
  BLEDevice::init("IMU-RC Hand Unit");
  pBLEScan = BLEDevice::getScan();  // Create BLE scan object
  pBLEScan->setActiveScan(true);    // Enable active scanning

  // Start scanning for BLE servers
  scanForServer();
}

void loop() {
  if (!DMPReady)
    return;

  static unsigned long lastSendTime = 0;
  unsigned long currentTime = millis();

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    if (resetYPR) {
      resetMPU();
      resetYPR = false;
    }

    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    double yaw =   ypr[0] * 180 / M_PI;
    double pitch = ypr[1] * 180 / M_PI;
    double roll =  ypr[2] * 180 / M_PI;

    // Send data to the server at a fixed frequency
    if (currentTime - lastSendTime > 1 / BLE_DATA_FREQUENCY * 1000) {
      lastSendTime = currentTime;

      // Assure client is connected
      if (pClient != nullptr && pClient->isConnected()) {

        if (stopped) {
          if      (positiveTilt(pitch) && !negativeTilt(roll) && !positiveTilt(roll)) { motorCommand(0, SPEED_MAX, "LEFT"); canMoveYAxis = false; } 
          else if (negativeTilt(pitch) && !negativeTilt(roll) && !positiveTilt(roll)) { motorCommand(SPEED_MAX, 0, "RIGHT"); canMoveYAxis = false; } 
          else {
            stopped = false;
            canMoveYAxis = true;
          }
        } else {
          stopped = true;
          canMoveYAxis = true;
          canMoveXAxis = true;
          motorCommand(0, 0, "STOP");
        }

        if (canMoveYAxis) {
          if (positiveTilt(roll)) {
            stopped = false;

            roll = constraintAngle(roll, THRESHOLD_MAX);
            uint8_t speed = (uint8_t)map(abs(roll), THRESHOLD, THRESHOLD_MAX, SPEED_MIN, SPEED_MAX);
            motorCommand(speed, speed, "REVERSE");
          } else if (negativeTilt(roll)) {
            stopped = false;

            roll = constraintAngle(roll, THRESHOLD_MAX);
            pitch = constraintAngle(pitch, THRESHOLD_MAX);
            uint8_t speed = (uint8_t)map(abs(roll), THRESHOLD, THRESHOLD_MAX, SPEED_MIN, SPEED_MAX);
            uint8_t LS = positiveTilt(pitch) ? speed - map(abs(pitch), THRESHOLD_STEERING, THRESHOLD_MAX, 0, speed) : speed;
            uint8_t RS = negativeTilt(pitch) ? speed - map(abs(pitch), THRESHOLD_STEERING, THRESHOLD_MAX, 0, speed) : speed;
            motorCommand(LS, RS, "FORWARD");
          }
        }

        // Check for incoming serial data
        if (Serial.available() > 0) {
          char key = Serial.read();  // Read the incoming byte

          // Map keys to motor commands
          switch (key) {
            case 'w': motorCommand(SPEED_MAX, SPEED_MAX, "FORWARD"); break;
            case 's': motorCommand(SPEED_MAX, SPEED_MAX, "REVERSE"); break;
            case 'a': motorCommand(SPEED_MAX, SPEED_MAX, "LEFT");    break;
            case 'd': motorCommand(SPEED_MAX, SPEED_MAX, "RIGHT");   break;
            case 'z': motorCommand(0        , 0        , "STOP");    break;
            case 'b': functionCommand("REQUEST BATTERY");            break;
            case 'r': resetYPR = true;                               break;
            default: return;
          }
        }

        // Send data to the server
        if (pRemoteCharacteristic != nullptr && !dataToSend.isEmpty()) {
          pRemoteCharacteristic->writeValue(dataToSend.c_str(), dataToSend.length());
        }

#if (DEBUG == true)
        Serial.print("ypr\t"); 
        Serial.print(yaw);   Serial.print("\t");
        Serial.print(pitch); Serial.print("\t");
        Serial.print(roll);  Serial.print("\t");
        Serial.println(dataToSend);
#endif
      } else {
#if (DEBUG == true)
        Serial.println("Connection with IMU-RC lost. Scanning...");
#endif
        // If the client is not connected, scan for servers
        scanForServer();
      }
    }
  }
}