#ifndef MENU_H
#define MENU_H

#define OLED_SCREEN_WIDTH 128  // Pixels
#define OLED_SCREEN_HEIGHT 64
///////////////////////////////////////////////////////////////////////////////
#define OLED_RESET -1
#define OLED_SCREEN_ADDRESS 0x3C
///////////////////////////////////////////////////////////////////////////////
#define BATTERY_POS_X 100  // Pixels
#define BATTERY_POS_Y 0
#define BATTERY_WIDTH 23
#define BATTERY_HEIGHT 12
#define BATTERY_PADDING 2
#define BATTERY_BAR_WIDTH 4
#define BATTERY_BAR_HEIGHT 8
#define BATTERY_NOTCH_W 2
#define BATTERY_NOTCH_H 6
///////////////////////////////////////////////////////////////////////////////
#define RSSI_NUM_BARS 4  // Pixels
#define RSSI_BAR_POS_X 0
#define RSSI_BAR_POS_Y 12
#define RSSI_BAR_WIDTH 4
#define RSSI_BAR_SPACING 1
#define RSSI_MIN 175
#define RSSI_MAX 215
///////////////////////////////////////////////////////////////////////////////
#define SNSV_NUM_BARS 5
#define SNSV_BAR_POS_X 42
#define SNSV_BAR_POS_Y 42
#define SNSV_BAR_WIDTH 6
#define SNSV_BAR_SPACING 2
///////////////////////////////////////////////////////////////////////////////
#define TOP_BAR_LINE_X 0  // Pixels
#define TOP_BAR_LINE_Y 14
#define TOP_BAR_LINE_W 128
#define TOP_BAR_LINE_H 1

Adafruit_SSD1306 display(OLED_SCREEN_WIDTH, OLED_SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t barsBattery = 4; // Default battery bars
uint8_t RSSI = 0;
uint8_t RSSIbars = 0;

// Define menu structures
enum MenuState {
  MAIN_MENU,
  CONNECTED_MENU,
  SETTINGS_MENU,
  CONTROL_TYPE_MENU,
  SCANNING,
  CONNECTING,
  NOT_FOUND,
  FAILED,
  CAR_STATS,
  INERTIAL_SENSITIVITY,
  DATA_PRINT,
  LED_INFO,
  LOST_CONNECTION,
  RESET,
  RESET_COMPLETED,
  RESET_FAILED
};
MenuState currentMenu = MAIN_MENU;
MenuState previousMenu = MAIN_MENU;

#define MAIN_MENU_ITEMS 2
const char *mainMenuText[MAIN_MENU_ITEMS] = {
  "Scan",
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

#define SETTINGS_MENU_ITEMS 4
const char *settingsMenuText[SETTINGS_MENU_ITEMS] = {
  "Inertial sensitivity",
  "Reset IMU",
  "Data print",
  "LED info"
};
int selectedSettingsMenuItem = 0;

#define CONTROL_TYPE_MENU_ITEMS 3
const char *controlTypeMenuText[CONTROL_TYPE_MENU_ITEMS] = {
  "Dual Motor",
  "Quad Motor",
  "Omni-directional"
};
int selectedControlTypeMenuItem = 0;

#include "menu.cpp"

#endif
