#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "3e3f1b05-90a8-43af-aa6b-335b28282b57"
#define CHARACTERISTIC_UUID "3e2c8a88-90fd-4b6b-b947-997e80d03d5b"

// Define pins for 4 motors using DRV8833 driver
#define AIN1 35  // RIGHT LEFT motor - input 1
#define AIN2 45  // RIGHT LEFT motor - input 2
#define BIN1 48  // FRONT LEFT motor - input 1 ALTERADO COM BIN2
#define BIN2 47  // FRONT LEFT motor - input 2 ALTERADO COM BIN1
#define AIN3 15  // FRONT RIGHT motor - input 1 ALTERADO COM AIN4
#define AIN4 16  // FRONT RIGHT motor - input 2 ALTERADO COM AIN3
#define BIN3 6   // REAR RIGHT motor - input 1
#define BIN4 7   // REAR RIGHT motor - input 2

#define STNDBYA 36
#define STNDBYB 17

// PWM channels for each pin
#define AIN1_CH 0
#define AIN2_CH 1
#define BIN1_CH 2
#define BIN2_CH 3
#define AIN3_CH 4
#define AIN4_CH 5
#define BIN3_CH 6
#define BIN4_CH 7

#define RESOLUTION 8
#define FREQUENCY 1000

#define SPEED_MAX 255
#define SPEED_MIN 0
#define SPEED_STEP 10

#define SEND_BATTERY_FREQUENCY 1

#define DEBUG

// Global variable to store received data
String receivedData = "";
String function = "";
uint8_t speed_LM = 0;
uint8_t speed_RM = 0;
uint8_t batteryPercentage = 0;
double batteryVoltage = 0.0;

// BLECharacteristic pointer for notifications
BLECharacteristic *pCharacteristic;

// BLE characteristic callback class
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value.length() > 0) {
      // Directly set the global receivedData
      receivedData = String(value.c_str());
      
      // Parsing logic for function and motor speeds
      int firstDelimiterIndex = receivedData.indexOf(',');
      int secondDelimiterIndex = receivedData.indexOf(',', firstDelimiterIndex + 1);

      if (firstDelimiterIndex > 0 && secondDelimiterIndex > firstDelimiterIndex) {
        function = receivedData.substring(0, firstDelimiterIndex);
        String leftMotorSpeed = receivedData.substring(firstDelimiterIndex + 1, secondDelimiterIndex);
        String rightMotorSpeed = receivedData.substring(secondDelimiterIndex + 1);

        speed_LM = leftMotorSpeed.toInt();
        speed_RM = rightMotorSpeed.toInt();
      } else {
        Serial.println("Invalid data format.");
      }
    }
  }
};

// Function to control movement based on state
void move(uint8_t state) {
  switch (state) {
    case 0: // STOP
      ledcWrite(AIN1_CH, 0); ledcWrite(AIN2_CH, 0);
      ledcWrite(BIN1_CH, 0); ledcWrite(BIN2_CH, 0);
      ledcWrite(AIN3_CH, 0); ledcWrite(AIN4_CH, 0);
      ledcWrite(BIN3_CH, 0); ledcWrite(BIN4_CH, 0);
      break;

    case 1: // FORWARD
      ledcWrite(AIN1_CH, speed_LM); ledcWrite(AIN2_CH, 0);
      ledcWrite(BIN1_CH, speed_LM); ledcWrite(BIN2_CH, 0);
      ledcWrite(AIN3_CH, speed_RM); ledcWrite(AIN4_CH, 0);
      ledcWrite(BIN3_CH, speed_RM); ledcWrite(BIN4_CH, 0);
      break;

    case 2: // REVERSE
      ledcWrite(AIN1_CH, 0); ledcWrite(AIN2_CH, speed_LM);
      ledcWrite(BIN1_CH, 0); ledcWrite(BIN2_CH, speed_LM);
      ledcWrite(AIN3_CH, 0); ledcWrite(AIN4_CH, speed_RM);
      ledcWrite(BIN3_CH, 0); ledcWrite(BIN4_CH, speed_RM);
      break;

    case 3: // ROTATE RIGHT (left motors forward, right motors backward)
      ledcWrite(AIN1_CH, speed_LM); ledcWrite(AIN2_CH, 0);
      ledcWrite(BIN1_CH, speed_LM); ledcWrite(BIN2_CH, 0);
      ledcWrite(AIN3_CH, 0); ledcWrite(AIN4_CH, speed_RM);
      ledcWrite(BIN3_CH, 0); ledcWrite(BIN4_CH, speed_RM);
      break;

    case 4: // ROTATE LEFT (left motors backward, right motors forward)
      ledcWrite(AIN1_CH, 0); ledcWrite(AIN2_CH, speed_LM);
      ledcWrite(BIN1_CH, 0); ledcWrite(BIN2_CH, speed_LM);
      ledcWrite(AIN3_CH, speed_RM); ledcWrite(AIN4_CH, 0);
      ledcWrite(BIN3_CH, speed_RM); ledcWrite(BIN4_CH, 0);
      break;
  }
}

void enableMotorDrivers() {
  pinMode(STNDBYA, OUTPUT);
  pinMode(STNDBYB, OUTPUT);
  digitalWrite(STNDBYA, HIGH);
  digitalWrite(STNDBYB, HIGH);
}

// Function to configure motor pins and attach PWM channels
void setupPins() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(AIN3, OUTPUT); pinMode(AIN4, OUTPUT);
  pinMode(BIN3, OUTPUT); pinMode(BIN4, OUTPUT);

  ledcSetup(AIN1_CH, FREQUENCY, RESOLUTION); ledcAttachPin(AIN1, AIN1_CH);
  ledcSetup(AIN2_CH, FREQUENCY, RESOLUTION); ledcAttachPin(AIN2, AIN2_CH);
  ledcSetup(BIN1_CH, FREQUENCY, RESOLUTION); ledcAttachPin(BIN1, BIN1_CH);
  ledcSetup(BIN2_CH, FREQUENCY, RESOLUTION); ledcAttachPin(BIN2, BIN2_CH);
  ledcSetup(AIN3_CH, FREQUENCY, RESOLUTION); ledcAttachPin(AIN3, AIN3_CH);
  ledcSetup(AIN4_CH, FREQUENCY, RESOLUTION); ledcAttachPin(AIN4, AIN4_CH);
  ledcSetup(BIN3_CH, FREQUENCY, RESOLUTION); ledcAttachPin(BIN3, BIN3_CH);
  ledcSetup(BIN4_CH, FREQUENCY, RESOLUTION); ledcAttachPin(BIN4, BIN4_CH);

  move(0); // Stop all motors initially
}

void setup() {
  setupPins();
  enableMotorDrivers();

#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Initialize the BLE device
  BLEDevice::init("IMU-RC Car");

  // Create a BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create a BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic with read, write, and notify properties
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ | 
      BLECharacteristic::PROPERTY_WRITE | 
      BLECharacteristic::PROPERTY_NOTIFY);

  // Set callback to handle client writes
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Set an initial value for the characteristic
  pCharacteristic->setValue("IMU-RC Car V1.0.0");

  // Start the service
  pService->start();

  // Start advertising the BLE service
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // Helpful for iPhone connections
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

#ifdef DEBUG
  Serial.println("Waiting for a client connection...");
#endif
}

void sendNotificationToClient(String data) {
  // Ensure the client is connected before notifying
  if (pCharacteristic != nullptr) {
    pCharacteristic->setValue(data.c_str());  // Update the characteristic value
    pCharacteristic->notify();                // Send notification to the client
#ifdef DEBUG
    Serial.print("Notified client: ");
    Serial.println(data);
#endif
  }
}

void loop() {
  unsigned long currentTime = millis();
  static unsigned long lastSendTime = 0;

  if (!receivedData.isEmpty()) {  // Check if receivedData is not empty
#ifdef DEBUG
    Serial.println("Processing command: " + receivedData);
#endif
    // Motor control logic
    if (function.equals("FORWARD")) {
      move(1);
    } else if (function.equals("REVERSE")) {
      move(2);
    } else if (function.equals("RIGHT")) {
      move(3);
    } else if (function.equals("LEFT")) {
      move(4);
    } else if (function.equals("STOP")) {
      move(0);
    } else {
#ifdef DEBUG
      Serial.println("Invalid command received!");
#endif
      sendNotificationToClient("Invalid command");
    }
    receivedData = "";  // Clear after processing to avoid re-triggering
  }
  
  if (currentTime - lastSendTime > 1 / SEND_BATTERY_FREQUENCY * 1000) {
    lastSendTime = currentTime;
    sendNotificationToClient(String(batteryPercentage, 0));
  }

  batteryPercentage += 1;
  if (batteryPercentage > 100) {
    batteryPercentage = 0;
  }
  

  // future development

  // uint8_t command;
  // switch(command) {
  //   case 0: // STOP
  //     move(0);
  //     break;
  //   case 1: // FORWARD
  //     move(1);
  //     break;
  //   case 2: // REVERSE
  //     move(2);
  //     break;
  //   case 3: // RIGHT
  //     move(3);
  //     break;
  //   case 4: // LEFT
  //     move(4);
  //     break;
  //   case 5: // REQUEST BATTERY
  //     sendNotificationToClient(String(batteryPercentage, 0));
  //     break;
  //   default:
  //     sendNotificationToClient("Invalid command");
  //     break;
  // }
}