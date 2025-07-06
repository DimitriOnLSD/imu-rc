#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Define the UUIDs for the service and characteristic
#define SERVICE_UUID        "3e3f1b05-90a8-43af-aa6b-335b28282b57"
#define CHARACTERISTIC_UUID "3e2c8a88-90fd-4b6b-b947-997e80d03d5b"

// Define motor control pins for DRV8833 driver
#define AIN1 35  // RIGHT LEFT motor - input 1
#define AIN2 45  // RIGHT LEFT motor - input 2
#define BIN1 48  // FRONT LEFT motor - input 1 ALTERADO COM BIN2
#define BIN2 47  // FRONT LEFT motor - input 2 ALTERADO COM BIN1
#define AIN3 15  // FRONT RIGHT motor - input 1 ALTERADO COM AIN4
#define AIN4 16  // FRONT RIGHT motor - input 2 ALTERADO COM AIN3
#define BIN3 6   // REAR RIGHT motor - input 1
#define BIN4 7   // REAR RIGHT motor - input 2

// Define standby pins for DRV8833 driver
#define STNDBYA 36
#define STNDBYB 17

// Define PWM channels for motor control
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

// Define motor control states
#define SPEED_MAX 255
#define SPEED_MIN 0
#define SPEED_STEP 10

// Define standby states
#define SEND_BATTERY_FREQUENCY 1 // Frequency to send battery percentage (Hz)
#define BATTERY_VOLTAGE_PIN 20  // Pin for battery voltage measurement
#define BATTERY_VOLTAGE_ENABLE_PIN 21 // Pin to enable battery voltage measurement
#define BATTERY_READ_DELAY 10 // Delay for battery voltage measurement in milliseconds

// Uncomment the following line to enable debug messages
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
  // Ensure speed values are within bounds
  speed_LM = constrain(speed_LM, SPEED_MIN, SPEED_MAX);
  speed_RM = constrain(speed_RM, SPEED_MIN, SPEED_MAX);
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

void disableSleepMode() {
  digitalWrite(STNDBYA, HIGH);
  digitalWrite(STNDBYB, HIGH);
}

void enableSleepMode() {
  digitalWrite(STNDBYA, LOW);
  digitalWrite(STNDBYB, LOW);
}

// Function to configure motor pins and attach PWM channels
void setupMotorPins() {
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(AIN3, OUTPUT); pinMode(AIN4, OUTPUT);
  pinMode(BIN3, OUTPUT); pinMode(BIN4, OUTPUT);

  pinMode(STNDBYA, OUTPUT);
  pinMode(STNDBYB, OUTPUT);

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

// Function to configure battery voltage measurement pins
void setupBatteryPins() {
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(BATTERY_VOLTAGE_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, LOW); // Keep battery voltage measurement disabled initially
}

void enableBatteryVoltageMeasurement() {
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, HIGH); // Enable battery voltage measurement
}

uint8_t measureBatteryVoltage() {
  int rawADC = analogRead(BATTERY_VOLTAGE_PIN);
  batteryVoltage = (rawADC / 4095.0) * 3.3;
  digitalWrite(BATTERY_VOLTAGE_ENABLE_PIN, LOW); // Disable battery voltage measurement

  float minVoltage = 2.91;
  float maxVoltage = 3.30;
  return constrain(100.0 * (batteryVoltage - minVoltage) / (maxVoltage - minVoltage), 0, 100);
}

void setup() {
  setupMotorPins();   // Configure motor pins and attach PWM channels
  disableSleepMode(); // Disable deep sleep mode to keep the device awake
  setupBatteryPins(); // Configure battery voltage measurement pins

#ifdef DEBUG
  Serial.begin(115200);
#endif

  BLEDevice::init("IMU-RC Car");                               // Initialize the BLE device
  BLEServer *pServer = BLEDevice::createServer();              // Create a BLE Server
  BLEService *pService = pServer->createService(SERVICE_UUID); // Create a BLE Service
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);                      // Create a BLE Characteristic with read, write, and notify properties
  pCharacteristic->setCallbacks(new MyCallbacks());           // Set callback to handle client writes
  pCharacteristic->setValue("IMU-RC Car");                    // Set an initial value for the characteristic
  pService->start();                                          // Start the service
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // Start advertising the BLE service
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Helpful for iPhone connections
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
  
  if (currentTime - lastSendTime > 1 / SEND_BATTERY_FREQUENCY * (1000 - BATTERY_READ_DELAY)) {
    enableBatteryVoltageMeasurement(); // Enable battery voltage measurement
    if (currentTime - lastSendTime > 1 / SEND_BATTERY_FREQUENCY * 1000) {
      lastSendTime = currentTime;
      batteryPercentage = measureBatteryVoltage(); // Measure battery voltage and get percentage
      sendNotificationToClient(String(batteryPercentage, 0)); // Send battery percentage to the client
    }
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