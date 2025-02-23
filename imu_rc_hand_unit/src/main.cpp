#include "main.h"

void setup() {
#if (DEBUG == true)
  Serial.begin(115200);
#endif
  // setupBattery();
  // setupLED();
  setupI2C();
  setupOLED();
  setupMPU6050();
  setupBLE();

  display.clearDisplay();
  display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);

  drawBattery(BATTERY_POS_X, BATTERY_POS_Y, getBatteryLevel());
  drawMenu(mainMenuText, MAIN_MENU_ITEMS, selectedMainMenuItem);

  display.display();
}

void loop() {
  double yaw;
  double pitch;
  double roll;
  static unsigned long lastSendTime = 0;
  static unsigned long lastMenuTime = 0;
  unsigned long currentTime = millis();
  char input;
  uint8_t barsBattery = getBatteryLevel();
  uint8_t RSSI = 0;

#if (DISABLE_DISPLAY == false)
  // Save battery by having a fixed menu display frequency
  if (currentTime - lastMenuTime > 1 / DISPLAY_MENU_FREQUENCY * 1000) {
    lastMenuTime = currentTime;

    // Set LED status based on battery level
    if (digitalRead(PIN_CHARGER_STATUS) == HIGH) {
      // Charging and reached 100% or charging and not reached 100%
      if (batteryPercentage == 100) {
        SetLEDStatus("GREEN", "ON");
      } else {
        SetLEDStatus("YELLOW", "ON");
      }
    } else {
      // Not charging. When below 20%, blink red. Otherwise, solid red.
      if (batteryPercentage <= 20) {
        SetLEDStatus("RED", "BLINK");
      } else {
        SetLEDStatus("RED", "ON");
      }
    }

    if (Serial.available() || clientIsScanning) {
      if (!clientIsScanning)
        input = Serial.read();

      switch (currentMenu) {
        case MAIN_MENU:
          switch (input) {
            case 'w':  // Navigate up
              navigateUp(selectedMainMenuItem, MAIN_MENU_ITEMS);
              break;
            case 's':  // Navigate down
              navigateDown(selectedMainMenuItem, MAIN_MENU_ITEMS);
              break;
            case 32:  // Select menu
              previousMenu = currentMenu;
              switch (selectedMainMenuItem) {
                case 0:  // Start scanning for BLE servers
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
            case 'w':  // Navigate up
              navigateUp(selectedConnectedMenuItem, CONNECTED_MENU_ITEMS);
              break;
            case 's':  // Navigate down
              navigateDown(selectedConnectedMenuItem, CONNECTED_MENU_ITEMS);
              break;
            case 32: // Select menu
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
                  previousMenu = MAIN_MENU;
                  currentMenu = MAIN_MENU;
                  break;
              }
              break;
            default:
              RSSI = pClient->getRssi();
              break;
          }
          break;
        case SETTINGS_MENU:
          switch (input) {
            case 'w':  // Navigate up
              navigateUp(selectedSettingsMenuItem, SETTINGS_MENU_ITEMS);
              break;
            case 's':  // Navigate down
              navigateDown(selectedSettingsMenuItem, SETTINGS_MENU_ITEMS);
              break;
            case 9:  // Back to main menu
              currentMenu = clientIsConnected ? CONNECTED_MENU : MAIN_MENU;
              break;
            case 32:  // Select menu
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
                case 3: // LED info
                  currentMenu = LED_INFO;
                  break;
              }
              break;
          }
          break;
        case CONTROL_TYPE_MENU:
          switch (input) {
            case 'w':  // Navigate up
              navigateUp(selectedControlTypeMenuItem, CONTROL_TYPE_MENU_ITEMS);
              break;
            case 's':  // Navigate down
              navigateDown(selectedControlTypeMenuItem, CONTROL_TYPE_MENU_ITEMS);
              break;
            case 9:  // Back
              currentMenu = previousMenu;
              break;
            case 32:  // Select menu
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
          switch(input) {
            case 32:
            case 9:
              currentMenu = previousMenu;
              break;
            case 'w':
              sensitivity += 0.1f;
              break;
            case 's':
              sensitivity -= 0.1f;
              break;
          }
          break;
        case NOT_FOUND:
        case FAILED:
          clientIsScanning = false;
        case DATA_PRINT:
        case RESET_COMPLETED:
        case LED_INFO:
        case CAR_STATS:
        case LOST_CONNECTION:
          switch(input) {
            case 32:
            case 9:
              currentMenu = currentMenu == LOST_CONNECTION ? MAIN_MENU : previousMenu;
              break;
          }
          break;       
      }
    }

    sensitivity = (float) constrain(sensitivity, 0.1f, 2.0f);

    display.clearDisplay();
    display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);
    drawBattery(BATTERY_POS_X, BATTERY_POS_Y, barsBattery);      
    drawSignalStrength(RSSI);
    
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
        SetLEDStatus("BLUE", "BLINK");
        break;
      case NOT_FOUND:
        drawText("No servers found.", 1, -1, -1);
        break;
      case FAILED:
        drawText("Failed to connect.", 1, -1, -1);
        break;
      case CAR_STATS:
        drawText("Battery level:", 1, -1, -1);
        break;
      case INERTIAL_SENSITIVITY:
        drawSensitivityBar(sensitivity);
        drawText("Sensitivity: " + String(sensitivity, 1), 1, -1, 56);
        break;
      case LED_INFO:
        drawText("RED:ON",          1, -1, 16);      
        drawText("YELLOW:CHARGING", 1, -1, 26); 
        drawText("GREEN:CHARGED",   1, -1, 36); 
        drawText("BLUE:SCAN",       1, -1, 46); 
        drawText("WHITE:RESET",     1, -1, 56);
        break;
      case DATA_PRINT:  
        drawText("yaw: "   + String((float) yaw,   1), 1, -1, 16);
        drawText("pitch: " + String((float) pitch, 1), 1, -1, 24);
        drawText("roll: "  + String((float) roll,  1), 1, -1, 32);
        drawText("data: "  + dataToSend, 1, -1, 40);
        break;	
      case LOST_CONNECTION:
        drawText("Lost connection.", 1, -1, -1);
        break;
      case RESET:
        drawText("Resetting...", 1, -1, -1);
        debugPrint(false, 0, 0, 0, "Resetting MPU6050...");
        SetLEDStatus("WHITE", "ON");
        break;
      case RESET_COMPLETED:
        drawText("MPU-6050", 1, -1, 24);
        drawText("reset completed!", 1, -1, 34);
        break;
      case RESET_FAILED:
        drawText("Failed to reset", 1, -1, 24);
        drawText("MPU-6050.", 1, -1, 34);
        break;
    }

    display.display();
  }
#endif

#if (DISABLE_BLE == false)
  // Save battery by having a fixed data send frequency
  if (currentTime - lastSendTime > 1 / BLE_DATA_FREQUENCY * 1000) {
    lastSendTime = currentTime;

    // Assure MPU is ready
    if (!DMPReady)
      return;

    // Reset MPU if requested
    if (resetYPR) {
      currentMenu = resetMPU() ? RESET_COMPLETED : RESET_FAILED;
      resetYPR = false;
    }

    // Assure client is connected
    if (pClient != nullptr && pClient->isConnected()) {

      // Fetch data from the MPU
      if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {

        // Get quaternion and gravity
        mpu.dmpGetQuaternion(&q, FIFOBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert yaw, pitch and roll to degrees
        yaw = ypr[0] * 180 / M_PI;
        pitch = ypr[1] * 180 / M_PI;
        roll = ypr[2] * 180 / M_PI;

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

        // Print data to the serial monitor
        debugPrint(true, yaw, pitch, roll, dataToSend);
      }
    } else {
      if (clientIsConnected && !clientDisconnected && clientConnectionAttempt) {
        clientLostConnection = true;
        clientIsConnected = false;
        clientConnectionAttempt = false;
        currentMenu = LOST_CONNECTION;
        if (printOnce) {
          debugPrint(false, 0, 0, 0, "Lost connection to IMU-RC Car.");
          printOnce = false;
        }
      }
    }
  }
#endif
}
