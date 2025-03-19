void setupBattery() {
  // Set battery pin as input
  pinMode(PIN_BATTERY_READ, INPUT);
  pinMode(PIN_BATTERY_ENABLE_READ, OUTPUT);
  digitalWrite(PIN_BATTERY_ENABLE_READ, LOW);
}

uint16_t enableBattRead() {
  // Read battery voltage
  digitalWrite(PIN_BATTERY_ENABLE_READ, HIGH);
  uint16_t analogBits = analogRead(PIN_BATTERY_READ);
  digitalWrite(PIN_BATTERY_ENABLE_READ, LOW);

  return analogBits;
}

double batteryVoltage(uint16_t analogBits) { return (double)analogBits * 4.2 / 4095;}
uint8_t batteryPercentage(double voltage)  { return constrain(map(voltage, 3.7, 4.2, 0, 100), 0, 100);}
uint8_t batteryBars(uint16_t analogBits)   { return round(analogBits > 0 ? analogBits / 1024 : 0); }
bool checkCharging()                       { return digitalRead(PIN_CHARGER_STATUS);}
