void IRAM_ATTR handleButtonPress(int buttonIndex, int pin) {
    buttonPress = true; // Set flag for button press
    detachInterrupt(pin);  // Disable interrupt for debounce
    buttonPressed[buttonIndex] = true; // Set flag to process in loop
}

void IRAM_ATTR handleUP() { handleButtonPress(0, PIN_BTN_U); }
void IRAM_ATTR handleDN() { handleButtonPress(1, PIN_BTN_D); }
void IRAM_ATTR handleSL() { handleButtonPress(2, PIN_BTN_S); }
void IRAM_ATTR handleBK() { handleButtonPress(3, PIN_BTN_B); }

void setupFNButtons() {
    pinMode(PIN_BTN_U, INPUT);
    pinMode(PIN_BTN_D, INPUT);
    pinMode(PIN_BTN_S, INPUT);
    pinMode(PIN_BTN_B, INPUT);

    attachInterrupt(digitalPinToInterrupt(PIN_BTN_U), handleUP, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_D), handleDN, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_S), handleSL, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_B), handleBK, FALLING);
}