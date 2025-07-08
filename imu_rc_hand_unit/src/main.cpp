/* DEBUG AND DEVELOPMENT PURPOSE ONLY */
#define DEBUG
// #define DEV_BOARD

#include "main.h"

bool resetYPR = false;

// Function to print data to the serial monitor
void debugPrint(bool printData, double yaw, double pitch, double roll, String data) {
#ifdef DEBUG
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
#ifdef DEBUG
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
  if (value > limit) { value = limit; }
  return value;
}
float decrease(float value, float decrement, float limit) {
  value -= decrement;
  if (value < limit) { value = limit; }
  return value;
}

void setup() {
  setupTimer();
  setupFNButtons();
  setupSerial();
  setupLED();
  setupBattery();
  setupI2C();
  setupOLED();
  setupLSM6DSO();
  setupBLE();

  display.clearDisplay();
  displayHUD(batteryBars(enableBattRead()), 0);
  display.display();
}

void loop() {
  if (buttonPress) {
    for (int i = 0; i < 4; i++) {
      if (buttonPressed[i]) {
        switch (i) {
          case 0: input = 'w'; attachInterrupt(digitalPinToInterrupt(PIN_BTN_U), handleUP, FALLING); break;
          case 1: input = 's'; attachInterrupt(digitalPinToInterrupt(PIN_BTN_D), handleDN, FALLING); break;
          case 2: input = 32;  attachInterrupt(digitalPinToInterrupt(PIN_BTN_S), handleSL, FALLING); break;
          case 3: input = 9;   attachInterrupt(digitalPinToInterrupt(PIN_BTN_B), handleBK, FALLING); break;
        }
        buttonPressed[i] = false;  // Reset flag
      }
    }
    buttonPress = false;
    updateDisplay = true;
  } else if (Serial.available()) {
    input = Serial.read();
    updateDisplay = true;
  } else {
    input = 0;
  }

  if (updateBattery) {
    // Read battery level
    uint16_t battData = enableBattRead();
    double battVoltage = batteryVoltage(battData);
    battPercentage = batteryPercentage(battVoltage);
    barsBattery = batteryBars(battData);

    char buffer[50];
    snprintf(buffer, sizeof(buffer), "Battery level: %d%%", battPercentage);
    debugPrint(false, 0, 0, 0, buffer);

    // make interrupt for charge pin and flag for this function
    // Check if the battery is low
    updateBattery = false;
  }

  if (updateRSSI && clientIsConnected) {
    RSSI = pClient->getRssi();
    debugPrint(false, 0, 0, 0, "RSSI: " + String(RSSI));
    RSSIbars = getSignalBars(RSSI);
    updateRSSI = false;
  } else if (!clientIsConnected) {
    RSSI = 0; // Assure the RSSI value is 0 when not connected
    RSSIbars = 0;
  }

  if (updateDisplay) {
    // Handle the current menu
    switch (currentMenu) {
      case MAIN_MENU:
        switch (input) {
          case 87:
          case 119: // W
            navigateUp(selectedMainMenuItem, MAIN_MENU_ITEMS);
            break;
          case 53:
          case 115: // S
            navigateDown(selectedMainMenuItem, MAIN_MENU_ITEMS);
            break;
          case 32: // SPACE
            previousMenu = currentMenu;
            switch (selectedMainMenuItem) {
              case 0:  // Scan
                clientIsScanning = true;
                currentMenu = SCANNING;
                break;
              case 1:  // Settings
                currentMenu = SETTINGS_MENU;
                break;
            }
            break;
        }
        break;
      case CONNECTED_MENU:
        switch (input) {
          case 87:
          case 119: // W
            navigateUp(selectedConnectedMenuItem, CONNECTED_MENU_ITEMS);
            break;
          case 53:
          case 115: // S
            navigateDown(selectedConnectedMenuItem, CONNECTED_MENU_ITEMS);
            break;
          case 32: // SPACE
            previousMenu = currentMenu;
            switch (selectedConnectedMenuItem) {
              case 0:  // Control type
                currentMenu = CONTROL_TYPE_MENU;
                break;
              case 1:  // Car stats
                currentMenu = CAR_STATS;
                break;
              case 2:  // Settings
                currentMenu = SETTINGS_MENU;
                break;
              case 3:  // Disconnect
                pClient->disconnect();
                debugPrint(false, 0, 0, 0, "Disconnected from IMU-RC Car.");
                clientDisconnected = true;
                clientIsConnected = false;
                previousMenu = MAIN_MENU; // Prevent returning to the connected menu
                currentMenu = MAIN_MENU;
                break;
            }
            break;
        }
        break;
      case SETTINGS_MENU:
        switch (input) {
          case 87:
          case 119: // W
            navigateUp(selectedSettingsMenuItem, SETTINGS_MENU_ITEMS);
            break;
          case 53:
          case 115: // S
            navigateDown(selectedSettingsMenuItem, SETTINGS_MENU_ITEMS);
            break;
          case 9: // TAB
            currentMenu = clientIsConnected ? CONNECTED_MENU : MAIN_MENU;
            break;
          case 32: // SPACE
            previousMenu = currentMenu;
            switch (selectedSettingsMenuItem) {
              case 0:  // Set inertial sensitivity
                currentMenu = INERTIAL_SENSITIVITY;
                break;
              case 1:  // Reset IMU
                resetYPR = true;
                currentMenu = RESET;
                break;
              case 2:  // Print data
                currentMenu = DATA_PRINT;
                break;
              case 3:  // LED info
                currentMenu = LED_INFO;
                break;
            }
            break;
        }
        break;
      case CONTROL_TYPE_MENU:
        switch (input) {
          case 87:
          case 119: // W
            navigateUp(selectedControlTypeMenuItem, CONTROL_TYPE_MENU_ITEMS);
            break;
          case 53:
          case 115: // S
            navigateDown(selectedControlTypeMenuItem, CONTROL_TYPE_MENU_ITEMS);
            break;
          case 9: // TAB
            currentMenu = previousMenu;
            break;
          case 32: // SPACE
            switch (selectedControlTypeMenuItem) {
              case 0:  // Dual Motor
                controlType = 0;
                break;
              case 1:  // Quad Motor
                controlType = 1;
                break;
              case 2:  // Omni-directional
                controlType = 2;
                break;
            }
            currentMenu = previousMenu;
            break;
        }
        break;
      case SCANNING:
        currentMenu = scanForServer() ? CONNECTED_MENU : NOT_FOUND;
        break;
      case INERTIAL_SENSITIVITY:
        switch (input) {
          case 32: // SPACE
          case 9: // TAB
            currentMenu = previousMenu;
            break;
          case 87:
          case 119: // W
            sensitivity = increase(sensitivity, 0.1f, sens_max);
            break;
          case 53:
          case 115: // S
            sensitivity = decrease(sensitivity, 0.1f, sens_min);
            break;
        }
        break;
      case DATA_PRINT:
        readDataWithScreen = true;
      case NOT_FOUND:
      case FAILED:
        clientIsScanning = false;
      case RESET_COMPLETED:
      case LED_INFO:
      case CAR_STATS:
      case LOST_CONNECTION:
        switch (input) {
          case 32: // SPACE
          case 9: // TAB
            currentMenu = currentMenu == LOST_CONNECTION ? MAIN_MENU : previousMenu;
            break;
        }
        break;
    }

    display.clearDisplay();
    
    displayHUD(barsBattery, RSSIbars);

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
      case CONTROL_TYPE_MENU:
        drawMenu(controlTypeMenuText, CONTROL_TYPE_MENU_ITEMS, selectedControlTypeMenuItem);
        break;
      case SCANNING:
        drawText("Scanning...", 1, -1, -1);
        SetLEDStatus("BLUE", "ON");
        break;
      case NOT_FOUND:
        drawText("No servers found.", 1, -1, -1);
        break;
      case FAILED:
        drawText("Failed to connect.", 1, -1, -1);
        break;
      case CAR_STATS:
        drawText("Battery level: " + String(serverBatteryPercentage), 1, -1, 17);
        drawText("RSSI: " + String(RSSI), 1, -1, 27);
        break;
      case INERTIAL_SENSITIVITY:
        drawSensitivityBar(sensitivity);
        drawText("Sensitivity: " + String(sensitivity, 1), 1, -1, 57);
        break;
      case LED_INFO:
        drawText("RED:ON", 1, -1, 17);
        drawText("YELLOW:CHARGING", 1, -1, 27);
        drawText("GREEN:CHARGED", 1, -1, 37);
        drawText("BLUE:SCAN", 1, -1, 47);
        drawText("WHITE:RESET", 1, -1, 57);
        break;
      case DATA_PRINT:
        drawText("yaw: " + String((double)yaw, 1), 1, -1, 17);
        drawText("pitch: " + String((double)pitch, 1), 1, -1, 27);
        drawText("roll: " + String((double)roll, 1), 1, -1, 37);
        drawText("data: " + dataToSend, 1, -1, 47);
        break;
      case LOST_CONNECTION:
        drawText("Lost connection.", 1, -1, -1);
        break;
      case RESET:
        drawText("Resetting...", 1, -1, -1);
        SetLEDStatus("WHITE", "ON");
        break;
      case RESET_COMPLETED:
        drawText("LSM6DSO", 1, -1, 24);
        drawText("reset completed!", 1, -1, 34);
        break;
      case RESET_FAILED:
        drawText("Failed to reset", 1, -1, 24);
        drawText("LSM6DSO.", 1, -1, 34);
        break;
    }

    display.display();
    updateDisplay = false;
  }

  if ((updateData && clientIsConnected) || (updateData && readDataWithScreen) || resetYPR) {
    // Assure MPU is ready
    if (!imuReady)
      return;

    // Reset MPU if requested
    if (resetYPR) {
      imuTicker.detach();  // Stop the IMU timer
      currentMenu = resetLSM() ? RESET_COMPLETED : RESET_FAILED;
      imuTicker.attach(1.0 / UPDATE_RATE_IMU, updateIMU);  // Re-enable IMU timer
      resetYPR = false;
    }

    if (readLSM6DSO()) {
      debugPrint(true, yaw, pitch, roll, dataToSend);
    }

    updateData = false;
    readDataWithScreen = false;
  }

  if (sendData) {
    if (pClient != nullptr && pClient->isConnected()) {

      // ROTAÇÃO PARADO
      if (carIsStopped) {
        if (positiveTilt(roll) && abs(roll) >= THRESHOLD_MIN_ROTATE && !negativeTilt(pitch) && !positiveTilt(pitch)) {

          roll = constraintAngle(roll, THRESHOLD_MAX_ROTATE);
          float baseSpeed = map(abs(roll), THRESHOLD_MIN_ROTATE, THRESHOLD_MAX_ROTATE, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          uint8_t rotateSpeed = constrain((baseSpeed * sensitivity), DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          motorCommand(0, rotateSpeed, "LEFT");
          canMoveYAxis = false;

        } else if (negativeTilt(roll) && abs(roll) >= THRESHOLD_MIN_ROTATE && !negativeTilt(pitch) && !positiveTilt(pitch)) {

          roll = constraintAngle(roll, THRESHOLD_MAX_ROTATE);
          float baseSpeed = map(abs(roll), THRESHOLD_MIN_ROTATE, THRESHOLD_MAX_ROTATE, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          uint8_t rotateSpeed = constrain((baseSpeed * sensitivity), DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          motorCommand(rotateSpeed, 0, "RIGHT");
          canMoveYAxis = false;

        } else {
          carIsStopped = false;
          canMoveYAxis = true;
        }

      } else {
        carIsStopped = true;
        canMoveYAxis = true;
        canMoveXAxis = true;
        motorCommand(0, 0, "STOP");
      }

      // MOVIMENTO Y (FRENTE / TRÁS) E CURVA (ROLL COMBINADO)
      if (canMoveYAxis) {
        if (negativeTilt(pitch) && abs(pitch) >= THRESHOLD_MIN) {
          carIsStopped = false;

          pitch = constraintAngle(pitch, THRESHOLD_MAX);
          float baseSpeed = map(abs(pitch), THRESHOLD_MIN, THRESHOLD_MAX, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
          uint8_t speed = constrain((sensitivity * baseSpeed), DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          motorCommand(speed, speed, "REVERSE");

        } else if (positiveTilt(pitch) && abs(pitch) >= THRESHOLD_MIN) {
          carIsStopped = false;

          pitch = constraintAngle(pitch, THRESHOLD_MAX);
          roll = constraintAngle(roll, THRESHOLD_MAX);

          float baseSpeed = map(abs(pitch), THRESHOLD_MIN, THRESHOLD_MAX, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
          uint8_t speed = constrain((sensitivity * baseSpeed), DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

          uint8_t steerDiff = (uint8_t)map(abs(roll), THRESHOLD_MIN_STEER, THRESHOLD_MAX, 0, DUTY_CYCLE_MAX);
          steerDiff = constrain(steerDiff, 0, speed);

          uint8_t LS = positiveTilt(roll) ? speed - steerDiff : speed;
          uint8_t RS = negativeTilt(roll) ? speed - steerDiff : speed;

          motorCommand(LS, RS, "FORWARD");
        }
      }

      // ENVIO DE DADOS BLE
      if (pRemoteCharacteristic != nullptr && !dataToSend.isEmpty()) {
        pRemoteCharacteristic->writeValue(dataToSend.c_str(), dataToSend.length());
      }

    } else {
      if (clientIsConnected && !clientDisconnected && clientConnectionAttempt) {
        clientLostConnection = true;
        clientIsConnected = false;
        clientConnectionAttempt = false;
        currentMenu = LOST_CONNECTION;
        debugPrint(false, 0, 0, 0, "Lost connection to IMU-RC Car.");
      }
    }

    sendData = false;
  }

  // Set LED status based on battery level
  if (checkCharging()) {
    battPercentage >= 95 ? SetLEDStatus("GREEN", "ON") : SetLEDStatus("YELLOW", "ON");
  } else {
    if (currentMenu != SCANNING && currentMenu != RESET) {
      battPercentage <= 20 ? SetLEDStatus("RED", "BLINK") : SetLEDStatus("RED", "ON");
    }
  }
}
