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

/* DEBUG PURPOSE ONLY */
#define DEBUG true
///////////////////////////////////////////////////////////////////////////////
#define SERVICE_UUID BLEUUID("3e3f1b05-90a8-43af-aa6b-335b28282b57")
#define CHARACTERISTIC_UUID BLEUUID("3e2c8a88-90fd-4b6b-b947-997e80d03d5b")
///////////////////////////////////////////////////////////////////////////////
#define SCAN_DURATION 5 // Seconds
#define DISPLAY_MENU_FREQUENCY 10 // Hz
#define BLE_DATA_FREQUENCY 20
///////////////////////////////////////////////////////////////////////////////
#define PIN_RGB_RED 1
#define PIN_RGB_GREEN 2
#define PIN_RGB_BLUE 3
#define PIN_BUTTON_BACK 4
#define PIN_BUTTON_UP 5
#define PIN_BUTTON_SELECT 6
#define PIN_BUTTON_DOWN 7
#define PIN_BATTERY_ENABLE_READ 8
#define PIN_BATTERY_READ 9
///////////////////////////////////////////////////////////////////////////////
#define THRESHOLD 10.0 // Threshold for the angular position. Variable in Euler angles
#define THRESHOLD_STEERING 5.0 
#define THRESHOLD_MAX 40.0 
///////////////////////////////////////////////////////////////////////////////
#define DUTY_CYCLE_MIN 170 // PWM duty cycle
#define DUTY_CYCLE_MAX 255
///////////////////////////////////////////////////////////////////////////////
#define OLED_SCREEN_WIDTH 128 // Pixels
#define OLED_SCREEN_HEIGHT 64
///////////////////////////////////////////////////////////////////////////////
#define OLED_RESET -1
#define OLED_SCREEN_ADDRESS 0x3C
///////////////////////////////////////////////////////////////////////////////
#define BATTERY_POS_X 100 // Pixels
#define BATTERY_POS_Y 0
#define BATTERY_WIDTH 23
#define BATTERY_HEIGHT 12
#define BATTERY_PADDING 2
#define BATTERY_BAR_WIDTH 4
#define BATTERY_BAR_HEIGHT 8
#define BATTERY_NOTCH_W 2
#define BATTERY_NOTCH_H 6
///////////////////////////////////////////////////////////////////////////////
#define RSSI_NUM_BARS 4 // Pixels
#define RSSI_BAR_POS_X 0
#define RSSI_BAR_POS_Y 12
#define RSSI_BAR_WIDTH 4
#define RSSI_BAR_SPACING 1
///////////////////////////////////////////////////////////////////////////////
#define TOP_BAR_LINE_X 0 // Pixels
#define TOP_BAR_LINE_Y 14
#define TOP_BAR_LINE_W 128
#define TOP_BAR_LINE_H 1

// Define the display, Bluetooth Low-Energy and IMU Sensor objects
Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET);
BLEScan *pBLEScan;
BLEClient *pClient;
BLERemoteCharacteristic *pRemoteCharacteristic;
MPU6050 mpu;

// Define menu structures
enum MenuState { MAIN_MENU,
                 CONNECTED_MENU,
                 SETTINGS_MENU };
MenuState currentMenu = MAIN_MENU;

#define MAIN_MENU_ITEMS 3
const char *mainMenuText[MAIN_MENU_ITEMS] = {
  "Connect",
  "LED info",
  "Settings"
};
int selectedMainMenuItem = 0;

#define CONNECTED_MENU_ITEMS 4
const char *connectedMenuText[CONNECTED_MENU_ITEMS] = {
  "Control type",
  "Car stats",
  "Settings",
  "Disconnect"
};
int selectedConnectedMenuItem = 0;

#define SETTINGS_MENU_ITEMS 3
const char *settingsMenuText[SETTINGS_MENU_ITEMS] = {
  "Inertial sensitivity",
  "Reset IMU",
  "Data print"
};
int selectedSettingsMenuItem = 0;

bool stopped = true;        // Flag to indicate the car is stopped
bool rotating = false;      // Flag to indicate the car is rotating
bool canMoveXAxis = false;  // Flag to enable movement in the X-axis
bool canMoveYAxis = true;   // Flag to enable movement in the Y-axis
bool movingXAxis = false;   // Flag to indicate movement in the X-axis
bool movingYAxis = false;   // Flag to indicate movement in the Y-axis

float batteryVoltage = 0; // Battery voltage in volts
uint8_t batteryPercentage = 0; // Battery percentage

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

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;  // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

/*---Function prototypes---*/
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

void setupIMU() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment on this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();

#if (DEBUG == true)
  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if (mpu.testConnection() == false) {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  } else {
    Serial.println("MPU6050 connection successful");
  }
#endif

#if (DEBUG == true)
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
}

/*---Callback function for notifications---*/
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  String notificationValue = "";
  for (size_t i = 0; i < length; i++) {
    notificationValue += (char)pData[i];
  }
#if (DEBUG == true)
  Serial.print("Notification received: ");
  Serial.println(notificationValue);
#endif
}

/*---Function to connect to the server and return the connected BLE client---*/
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

/*---Scan for devices---*/
void scanForServer() {
#if (DEBUG == true)
  Serial.println("Scanning for IMU-RC Car servers...");
#endif

  // Start BLE scan and look for devices
  BLEScanResults results = pBLEScan->start(SCAN_DURATION);
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

/*---Setup BLE device---*/
void setupBLE() {
  // Initialize BLE device as a client
  BLEDevice::init("IMU-RC Hand Unit");
  pBLEScan = BLEDevice::getScan();  // Create BLE scan object
  pBLEScan->setActiveScan(true);    // Enable active scanning

  // Start scanning for BLE servers
  scanForServer();
}

/*---Function to limit the angle---*/
double constraintAngle(double angle, double limit) {
  if (angle > limit) {
    return limit;
  } else if (angle < -limit) {
    return -limit;
  } else {
    return angle;
  }
}

/*---Function prototypes---*/
bool positiveTilt(double var) {
  return (var > THRESHOLD);
}
bool negativeTilt(double var) {
  return (var < -THRESHOLD);
}

/*---Function to set the speed of the motors---*/
void motorCommand(uint8_t leftSpeed, uint8_t rightSpeed, String direction) {
  dataToSend = direction + "," + leftSpeed + "," + rightSpeed;
}
void functionCommand(String command) {
  dataToSend = command;
}

// Draw battery icon
void drawBattery(int x, int y, int bars) {
  display.drawRect(x, y, BATTERY_WIDTH, BATTERY_HEIGHT, SSD1306_WHITE);
  for (int i = 0; i < bars; i++) {
    display.fillRect(x + BATTERY_PADDING + i * (BATTERY_BAR_WIDTH + 1), y + BATTERY_PADDING, BATTERY_BAR_WIDTH, BATTERY_BAR_HEIGHT, SSD1306_WHITE);
  }
  display.fillRect(x + BATTERY_WIDTH, y + (BATTERY_HEIGHT / 2) - (BATTERY_NOTCH_H / 2), BATTERY_NOTCH_W, BATTERY_NOTCH_H, SSD1306_WHITE);
}

// Draw signal strength
void drawSignalStrength(int numBars) {
  for (int i = 0; i < RSSI_NUM_BARS; i++) {
    if (i < numBars) {
      display.fillRect(RSSI_BAR_POS_X + (RSSI_BAR_WIDTH + RSSI_BAR_SPACING) * i, RSSI_BAR_POS_Y - ((RSSI_BAR_WIDTH - 2) * 2 + i * 2), RSSI_BAR_WIDTH, (RSSI_BAR_WIDTH - 2) * 2 + i * 2, SSD1306_WHITE);
    } else {
      display.drawRect(RSSI_BAR_POS_X + (RSSI_BAR_WIDTH + RSSI_BAR_SPACING) * i, RSSI_BAR_POS_Y - ((RSSI_BAR_WIDTH - 2) * 2 + i * 2), RSSI_BAR_WIDTH, (RSSI_BAR_WIDTH - 2) * 2 + i * 2, SSD1306_WHITE);
    }
  }
}

// Draw menu
void drawMenu(const char *menuItems[], int numItems, int selected) {
  display.setTextSize(1);
  for (int i = 0; i < numItems; i++) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(menuItems[i], 0, 0, &x1, &y1, &w, &h);
    int xPos = (OLED_SCREEN_WIDTH - w) / 2;
    int yPos = 16 + i * 12;
    if (i == selected) {
      display.fillRect(0, yPos - 2, OLED_SCREEN_WIDTH, h + 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    } else {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(xPos, yPos);
    display.print(menuItems[i]);
  }
}

void setupMenu() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
}

int getBatteryLevel() {
  // Read battery voltage
  digitalWrite(PIN_BATTERY_ENABLE_READ, HIGH);
  uint16_t analogBits = analogRead(PIN_BATTERY_READ);
  digitalWrite(PIN_BATTERY_ENABLE_READ, LOW);

  batteryVoltage = analogBits * 4.2 / 4095;
  batteryPercentage = map(batteryVoltage, 3.7, 4.2, 0, 100);
  batteryPercentage = constrain(batteryPercentage, 0, 100);

  int bars = (float)map(analogBits, 0, 4095, 0, 4);

  return bars;
}

void setupBattery() {
  // Set battery pin as input
  pinMode(PIN_BATTERY_READ, INPUT);
  pinMode(PIN_BATTERY_ENABLE_READ, OUTPUT);
  digitalWrite(PIN_BATTERY_ENABLE_READ, LOW);
}

void setup() {
#if (DEBUG == true)
  Serial.begin(115200);
#endif
  setupBattery();
  setupMenu();
  setupIMU();
  setupBLE();

  display.clearDisplay();
  display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);

  drawBattery(BATTERY_POS_X, BATTERY_POS_Y, getBatteryLevel());
  // drawSignalStrength(getRSSI());
  drawSignalStrength(3);

  display.display();
}

void loop() {
  static unsigned long lastSendTime = 0;
  static unsigned long lastMenuTime = 0;
  unsigned long currentTime = millis();

  // Save battery by having a fixed data send frequency
  if (currentTime - lastSendTime > 1 / BLE_DATA_FREQUENCY * 1000) {
    lastSendTime = currentTime;

    // Assure MPU is ready
    if (!DMPReady)
      return;

    // Assure client is connected
    if (pClient != nullptr && pClient->isConnected()) {

      // Fetch data from the MPU
      if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {

        // Reset MPU if requested
        if (resetYPR) {
          resetMPU();
          resetYPR = false;
        }

        // Get quaternion and gravity
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert yaw, pitch and roll to degrees
        double yaw = ypr[0] * 180 / M_PI;
        double pitch = ypr[1] * 180 / M_PI;
        double roll = ypr[2] * 180 / M_PI;

        // If the car is stopped, apply rotation logic. If not, apply movement logic
        if (stopped) {
          if (positiveTilt(pitch) && !negativeTilt(roll) && !positiveTilt(roll)) {
            motorCommand(0, DUTY_CYCLE_MAX, "LEFT");
            canMoveYAxis = false;
          } else if (negativeTilt(pitch) && !negativeTilt(roll) && !positiveTilt(roll)) {
            motorCommand(DUTY_CYCLE_MAX, 0, "RIGHT");
            canMoveYAxis = false;
          } else {
            stopped = false;
            canMoveYAxis = true;
          }
        } else {
          stopped = true;
          canMoveYAxis = true;
          canMoveXAxis = true;
          motorCommand(0, 0, "STOP");
        }

        // If postive tilt, move forward. If negative tilt, move backward
        if (canMoveYAxis) {
          if (positiveTilt(roll)) {
            stopped = false;

            roll = constraintAngle(roll, THRESHOLD_MAX);
            uint8_t speed = (uint8_t)map(abs(roll), THRESHOLD, THRESHOLD_MAX, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
            motorCommand(speed, speed, "REVERSE");
          } else if (negativeTilt(roll)) {
            stopped = false;

            roll = constraintAngle(roll, THRESHOLD_MAX);
            pitch = constraintAngle(pitch, THRESHOLD_MAX);
            uint8_t speed = (uint8_t)map(abs(roll), THRESHOLD, THRESHOLD_MAX, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
            uint8_t LS = positiveTilt(pitch) ? speed - map(abs(pitch), THRESHOLD_STEERING, THRESHOLD_MAX, 0, speed) : speed;
            uint8_t RS = negativeTilt(pitch) ? speed - map(abs(pitch), THRESHOLD_STEERING, THRESHOLD_MAX, 0, speed) : speed;
            motorCommand(LS, RS, "FORWARD");
          }
        }

        // Send data to the server
        if (pRemoteCharacteristic != nullptr && !dataToSend.isEmpty()) {
          pRemoteCharacteristic->writeValue(dataToSend.c_str(), dataToSend.length());
        }

#if (DEBUG == true)
        Serial.print("ypr\t");
        Serial.print(yaw);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.print(roll);
        Serial.print("\t");
        Serial.println(dataToSend);
#endif
      }
    } else {
#if (DEBUG == true)
      Serial.println("Connection with IMU-RC lost. Scanning...");
#endif
      // If the client is not connected, scan for servers
      scanForServer();
    }
  }

  // Save battery by having a fixed menu display frequency
  if (currentTime - lastMenuTime > 1 / DISPLAY_MENU_FREQUENCY * 1000) {
    lastMenuTime = currentTime;

    if (Serial.available()) {
      char input = Serial.read();
      switch (currentMenu) {
        case MAIN_MENU:
          if (input == 'u') {
            selectedMainMenuItem = (selectedMainMenuItem - 1 + MAIN_MENU_ITEMS) % MAIN_MENU_ITEMS;
          } else if (input == 'd') {
            selectedMainMenuItem = (selectedMainMenuItem + 1) % MAIN_MENU_ITEMS;
          } else if (input == 's') {  // Select menu
            if (selectedMainMenuItem == 0) {
              currentMenu = CONNECTED_MENU;
            } else if (selectedMainMenuItem == 2) {
              currentMenu = SETTINGS_MENU;
            }
          }
          break;
        case CONNECTED_MENU:
          if (input == 'u') {
            selectedConnectedMenuItem = (selectedConnectedMenuItem - 1 + CONNECTED_MENU_ITEMS) % CONNECTED_MENU_ITEMS;
          } else if (input == 'd') {
            selectedConnectedMenuItem = (selectedConnectedMenuItem + 1) % CONNECTED_MENU_ITEMS;
          } else if (input == 'b') {  // Back to main menu
            currentMenu = MAIN_MENU;
          }
          break;
        case SETTINGS_MENU:
          if (input == 'u') {
            selectedSettingsMenuItem = (selectedSettingsMenuItem - 1 + SETTINGS_MENU_ITEMS) % SETTINGS_MENU_ITEMS;
          } else if (input == 'd') {
            selectedSettingsMenuItem = (selectedSettingsMenuItem + 1) % SETTINGS_MENU_ITEMS;
          } else if (input == 'b') {  // Back to main menu
            currentMenu = MAIN_MENU;
          }
          break;
      }
    }

    display.clearDisplay();
    drawBattery(BATTERY_POS_X, BATTERY_POS_Y, 3);
    drawSignalStrength(3);
    display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);

    // Render the current menu
    switch (currentMenu) {
      case MAIN_MENU:
        drawMenu(mainMenuText, MAIN_MENU_ITEMS, selectedMainMenuItem);
        break;
      case CONNECTED_MENU:
        drawMenu(connectedMenuText, CONNECTED_MENU_ITEMS, selectedConnectedMenuItem);
        break;
      case SETTINGS_MENU:
        drawMenu(settingsMenuText, SETTINGS_MENU_ITEMS, selectedSettingsMenuItem);
        break;
    }

    display.display();
  }
}