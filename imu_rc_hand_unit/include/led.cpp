// Setup LED
void setupLED() {
#ifdef DEV_BOARD
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(10, 0, 0));
  pixels.show();
#else
  // Set LED pins as outputs
  pinMode(PIN_RGB_RED, OUTPUT);
  pinMode(PIN_RGB_GREEN, OUTPUT);
  pinMode(PIN_RGB_BLUE, OUTPUT);
#endif
}

// Helper function to set the LED color
void setColor(const char *color) {
#ifdef DEV_BOARD
  pixels.clear();

  if      (strcmp(color, "RED")    == 0) { pixels.setPixelColor(0, pixels.Color(33, 0, 0)); }
  else if (strcmp(color, "GREEN")  == 0) { pixels.setPixelColor(0, pixels.Color(0, 33, 0)); }
  else if (strcmp(color, "BLUE")   == 0) { pixels.setPixelColor(0, pixels.Color(0, 0, 33)); }
  else if (strcmp(color, "YELLOW") == 0) { pixels.setPixelColor(0, pixels.Color(33, 33, 0)); }
  else if (strcmp(color, "PURPLE") == 0) { pixels.setPixelColor(0, pixels.Color(33, 0, 33)); }
  else if (strcmp(color, "WHITE")  == 0) { pixels.setPixelColor(0, pixels.Color(33, 33, 33)); }

  pixels.show();
#else
  bool red = LOW, green = LOW, blue = LOW;

  if      (strcmp(color, "RED")    == 0) { red = HIGH; }
  else if (strcmp(color, "GREEN")  == 0) { green = HIGH; }
  else if (strcmp(color, "BLUE")   == 0) { blue = HIGH; }
  else if (strcmp(color, "YELLOW") == 0) { red = HIGH; green = HIGH; } 
  else if (strcmp(color, "PURPLE") == 0) { red = HIGH; blue = HIGH; } 
  else if (strcmp(color, "WHITE")  == 0) { red = HIGH; green = HIGH; blue = HIGH; }

  // Set the pins accordingly
  digitalWrite(PIN_RGB_RED, red);
  digitalWrite(PIN_RGB_GREEN, green);
  digitalWrite(PIN_RGB_BLUE, blue);
#endif
}

// Helper function to set the LED status
void SetLEDStatus(const char *color, const char *mode) {
  static unsigned long previousMillis = 0;  // Tracks the last time the LED toggled
  static bool ledState = false;             // Current state of the LED

  unsigned long currentMillis = millis();    // Current time in milliseconds
  const unsigned long blinkInterval = 1000;  // Interval for blinking (in milliseconds)

  // Determine LED mode
  if (strcmp(mode, "BLINK") == 0) {
    // Handle blinking logic
    if (currentMillis - previousMillis >= blinkInterval) {
      previousMillis = currentMillis;
      ledState = !ledState;  // Toggle the LED state
    }

    // Set the LED based on the toggled state
    if (ledState) {
      setColor(color);
    } else {
// Turn off the LED
#ifdef DEV_BOARD
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
#else
      digitalWrite(PIN_RGB_RED, LOW);
      digitalWrite(PIN_RGB_GREEN, LOW);
      digitalWrite(PIN_RGB_BLUE, LOW);
#endif
    }
  } else if (strcmp(mode, "ON") == 0) {
    // Set the LED to the specified color in ON mode
    setColor(color);
  } else {
// Turn off the LED for unsupported modes
#ifdef DEV_BOARD
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
#else
    digitalWrite(PIN_RGB_RED, LOW);
    digitalWrite(PIN_RGB_GREEN, LOW);
    digitalWrite(PIN_RGB_BLUE, LOW);
#endif
  }
}