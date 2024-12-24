#include "main.h"

void setup() {
#if (DEBUG == true)
  Serial.begin(115200);
#endif
  setupBattery();
  setupLED();
  setupI2CDevices();
  setupBLE();

  display.clearDisplay();
  display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);

  drawBattery(BATTERY_POS_X, BATTERY_POS_Y, getBatteryLevel());
  drawSignalStrength(0);

  display.display();
}

void loop() {
  static unsigned long lastSendTime = 0;
  static unsigned long lastMenuTime = 0;
  unsigned long currentTime = millis();
  uint8_t barsBattery = getBatteryLevel();

  // Save battery by having a fixed menu display frequency
  if (currentTime - lastMenuTime > 1 / DISPLAY_MENU_FREQUENCY * 1000) {
    lastMenuTime = currentTime;

    // Set LED status based on battery level
    if (digitalRead(PIN_CHARGER_STATUS) == HIGH) {
      if (batteryPercentage == 100) {
        SetLEDStatus("GREEN", "ON");
      } else {
        SetLEDStatus("YELLOW", "ON");
      }
    } else {
      if (batteryPercentage < 20) {
        SetLEDStatus("RED", "BLINK");
      } else {
        SetLEDStatus("RED", "ON");
      }
    }

    drawBattery(BATTERY_POS_X, BATTERY_POS_Y, barsBattery);

    if (Serial.available()) {
      char input = Serial.read();

      switch (currentMenu) {
        case MAIN_MENU:
          switch (input) {
            case 'u':  // Navigate up
              selectedMainMenuItem = (selectedMainMenuItem - 1 + MAIN_MENU_ITEMS) % MAIN_MENU_ITEMS;
              break;
            case 'd':  // Navigate down
              selectedMainMenuItem = (selectedMainMenuItem + 1) % MAIN_MENU_ITEMS;
              break;
            case 's':  // Select menu
              switch (selectedMainMenuItem) {
                case 0:  // Start scanning for BLE servers
                  if (scanForServer()) {
                    currentMenu = CONNECTED_MENU;  // Move to connected menu if connectedv
                  } else {
                    currentMenu = MAIN_MENU;  // Stay in main menu if not connected
                  }
                  break;
                case 1:  // Go to LED info menu
                  /*
                  - OFF: No LED color
                  - BATTERY: Red
                  - LOW BATTERY: Red blinking
                  - CHARGING: Yellow
                  - CHARGED: Green
                  - SCANNING: Blue blinking
                  - PAIRING: Blue
                  - IMU RESET: White blinking
                  */
                  break;
                case 2:  // Go to settings menu
                  previousMenu = MAIN_MENU;
                  currentMenu = SETTINGS_MENU;
                  break;
              }
              break;
          }
          break;

        case CONNECTED_MENU:
          drawSignalStrength(3);  // Get actual signal strength

          switch (input) {
            case 'u':  // Navigate up
              selectedConnectedMenuItem = (selectedConnectedMenuItem - 1 + CONNECTED_MENU_ITEMS) % CONNECTED_MENU_ITEMS;
              break;
            case 'd':  // Navigate down
              selectedConnectedMenuItem = (selectedConnectedMenuItem + 1) % CONNECTED_MENU_ITEMS;
              break;
            case 'b':  // Back to main menu
              currentMenu = MAIN_MENU;
              break;
            case 's':
              switch (selectedConnectedMenuItem) {
                case 0:  // Control type
                  break;
                case 1:  // Car stats
                  break;
                case 2:  // Settings
                  previousMenu = CONNECTED_MENU;
                  currentMenu = SETTINGS_MENU;
                  break;
                case 3:  // Disconnect
                  pClient->disconnect();
                  currentMenu = MAIN_MENU;
                  break;
              }
              break;

            case SETTINGS_MENU:
              switch (input) {
                case 'u':  // Navigate up
                  selectedSettingsMenuItem = (selectedSettingsMenuItem - 1 + SETTINGS_MENU_ITEMS) % SETTINGS_MENU_ITEMS;
                  break;
                case 'd':  // Navigate down
                  selectedSettingsMenuItem = (selectedSettingsMenuItem + 1) % SETTINGS_MENU_ITEMS;
                  break;
                case 'b':  // Back to main menu
                  if (previousMenu == CONNECTED_MENU) {
                    currentMenu = CONNECTED_MENU;
                  } else {
                    currentMenu = MAIN_MENU;
                  }
                  break;
                case 's':  // Select menu
                  switch (selectedSettingsMenuItem) {
                    case 0:  // Set inertial sensitivity
                      break;
                    case 1:  // Reset IMU
                      resetYPR = true;
                      break;
                    case 2:  // Print data
                      break;
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
  }

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

        // Print data to the serial monitor
        debugPrint(true, yaw, pitch, roll, dataToSend);
      }
    } else {
      debugPrint(false, 0, 0, 0, "Disconnected from IMU-RC Car.");
    }
  }
}
