// Draw menu
void drawMenu(const char *menuItems[], int numItems, int selected)
{
  display.setTextSize(1);
  for (int i = 0; i < numItems; i++)
  {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(menuItems[i], 0, 0, &x1, &y1, &w, &h);
    int xPos = (OLED_SCREEN_WIDTH - w) / 2;
    int yPos = 16 + i * 12;
    if (i == selected)
    {
      display.fillRect(0, yPos - 2, OLED_SCREEN_WIDTH, h + 4, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    }
    else
    {
      display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);
    }
    display.setCursor(xPos, yPos);
    display.print(menuItems[i]);
  }
}

// Draw text
void drawText(String text, int size, int x, int y)
{
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text.c_str(), 0, 0, &x1, &y1, &w, &h);
  // if x or y is -1, center the text
  int xPos = x == -1 ? (OLED_SCREEN_WIDTH - w) / 2 : x;
  int yPos = y == -1 ? (OLED_SCREEN_HEIGHT - h) / 2 : y;

  display.setCursor(xPos, yPos);
  display.print(text);
}

// Draw sensitivity bar
void drawSensitivityBar(float sensitivity)
{
  sensitivity *= 10;
  for (int i = 0; i < 20; i++)
  {
    if (i < sensitivity)
    {
      display.fillRect(SNSV_BAR_POS_X + (SNSV_BAR_WIDTH + SNSV_BAR_SPACING) * i, SNSV_BAR_POS_Y - ((SNSV_BAR_WIDTH - 2) + i), SNSV_BAR_WIDTH, (SNSV_BAR_WIDTH - 2) + i, SSD1306_WHITE);
    }
    else
    {
      display.drawRect(SNSV_BAR_POS_X + (SNSV_BAR_WIDTH + SNSV_BAR_SPACING) * i, SNSV_BAR_POS_Y - ((SNSV_BAR_WIDTH - 2) + i), SNSV_BAR_WIDTH, (SNSV_BAR_WIDTH - 2) + i, SSD1306_WHITE);
    }
  }
}

// Draw battery icon
void drawBattery(int x, int y, int bars)
{
  display.drawRect(x, y, BATTERY_WIDTH, BATTERY_HEIGHT, SSD1306_WHITE);
  for (int i = 0; i < bars; i++)
  {
    display.fillRect(x + BATTERY_PADDING + i * (BATTERY_BAR_WIDTH + 1), y + BATTERY_PADDING, BATTERY_BAR_WIDTH, BATTERY_BAR_HEIGHT, SSD1306_WHITE);
  }
  display.fillRect(x + BATTERY_WIDTH, y + (BATTERY_HEIGHT / 2) - (BATTERY_NOTCH_H / 2), BATTERY_NOTCH_W, BATTERY_NOTCH_H, SSD1306_WHITE);
}

// Draw signal strength
void drawSignalStrength(int numBars)
{
  for (int i = 0; i < RSSI_NUM_BARS; i++)
  {
    uint8_t x = RSSI_BAR_POS_X + (RSSI_BAR_WIDTH + RSSI_BAR_SPACING) * i;
    uint8_t y = RSSI_BAR_POS_Y - ((RSSI_BAR_WIDTH - 2) * 2 + i * 2);
    uint8_t w = RSSI_BAR_WIDTH;
    uint8_t h = (RSSI_BAR_WIDTH - 2) * 2 + i * 2;
    if (i < numBars)
    {
      display.fillRect(x, y, w, h, SSD1306_WHITE);
    }
    else
    {
      display.drawRect(x, y, w, h, SSD1306_WHITE);
    }
  }
}

// Display HUD
void displayHUD(uint8_t bars, uint8_t rssi)
{
  display.fillRect(TOP_BAR_LINE_X, TOP_BAR_LINE_Y, TOP_BAR_LINE_W, TOP_BAR_LINE_H, SSD1306_WHITE);
  drawBattery(BATTERY_POS_X, BATTERY_POS_Y, bars);
  clientIsConnected ? drawSignalStrength(rssi) : drawSignalStrength(0);
}

// Setup OLED display
void setupOLED()
{
  // Initialize the OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_SCREEN_ADDRESS))
  {
    debugPrint(false, 0, 0, 0, "OLED SSD1306 initialization failed");
    for (;;)
      ;
  }
  display.clearDisplay();
  drawText("Initializing...", 1, -1, -1);
  display.display();
}

// Navigate up and down in the menu
void navigateUp(int &selectedItem, int menuItems)
{
  selectedItem = (selectedItem - 1 + menuItems) % menuItems;
}
void navigateDown(int &selectedItem, int menuItems)
{
  selectedItem = (selectedItem + 1) % menuItems;
}