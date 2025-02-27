#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "3e3f1b05-90a8-43af-aa6b-335b28282b57"
#define CHARACTERISTIC_UUID "3e2c8a88-90fd-4b6b-b947-997e80d03d5b"

// Define the pins for the motor control
#define MOTOR_A1_PIN 4
#define MOTOR_A2_PIN 16
#define MOTOR_B1_PIN 17
#define MOTOR_B2_PIN 18

#define MOTOR_A1_CHANNEL 0
#define MOTOR_A2_CHANNEL 1
#define MOTOR_B1_CHANNEL 2
#define MOTOR_B2_CHANNEL 3

#define RESOLUTION 8
#define FREQUENCY 5000

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

void move(uint8_t state) {
  switch (state) {
    case 0:                            // Stop (coast)
      ledcWrite(MOTOR_A1_CHANNEL, 0);  // R
      ledcWrite(MOTOR_A2_CHANNEL, 0);  // R
      ledcWrite(MOTOR_B1_CHANNEL, 0);  // L
      ledcWrite(MOTOR_B2_CHANNEL, 0);  // L
      break;
    case 1:                                   // Forward
      ledcWrite(MOTOR_A1_CHANNEL, speed_LM);  // R
      ledcWrite(MOTOR_A2_CHANNEL, 0);         // R
      ledcWrite(MOTOR_B1_CHANNEL, 0);         // L
      ledcWrite(MOTOR_B2_CHANNEL, speed_RM);  // L
      break;
    case 2:                                   // Reverse
      ledcWrite(MOTOR_A1_CHANNEL, 0);         // R
      ledcWrite(MOTOR_A2_CHANNEL, speed_LM);  // R
      ledcWrite(MOTOR_B1_CHANNEL, speed_RM);  // L
      ledcWrite(MOTOR_B2_CHANNEL, 0);         // L
      break;
    case 3:                                   // Rotate right
      ledcWrite(MOTOR_A1_CHANNEL, speed_LM);  // R
      ledcWrite(MOTOR_A2_CHANNEL, 0);         // R
      ledcWrite(MOTOR_B1_CHANNEL, speed_RM);  // L
      ledcWrite(MOTOR_B2_CHANNEL, 0);         // L
      break;
    case 4:                                   // Rotate left
      ledcWrite(MOTOR_A1_CHANNEL, 0);         // R
      ledcWrite(MOTOR_A2_CHANNEL, speed_LM);  // R
      ledcWrite(MOTOR_B1_CHANNEL, 0);         // L
      ledcWrite(MOTOR_B2_CHANNEL, speed_RM);  // L
      break;
  }
}

// Function to setup the motor control pins
void setupPins() {
  pinMode(MOTOR_A1_PIN, OUTPUT);
  pinMode(MOTOR_A2_PIN, OUTPUT);
  pinMode(MOTOR_B1_PIN, OUTPUT);
  pinMode(MOTOR_B2_PIN, OUTPUT);
  ledcSetup(MOTOR_A1_CHANNEL, FREQUENCY, RESOLUTION);
  ledcSetup(MOTOR_A2_CHANNEL, FREQUENCY, RESOLUTION);
  ledcSetup(MOTOR_B1_CHANNEL, FREQUENCY, RESOLUTION);
  ledcSetup(MOTOR_B2_CHANNEL, FREQUENCY, RESOLUTION);
  ledcAttachPin(MOTOR_A1_PIN, MOTOR_A1_CHANNEL);
  ledcAttachPin(MOTOR_A2_PIN, MOTOR_A2_CHANNEL);
  ledcAttachPin(MOTOR_B1_PIN, MOTOR_B1_CHANNEL);
  ledcAttachPin(MOTOR_B2_PIN, MOTOR_B2_CHANNEL);
  move(0);
}

void setup() {
  setupPins();

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