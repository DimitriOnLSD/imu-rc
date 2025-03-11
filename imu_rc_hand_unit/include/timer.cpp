// Timer callbacks
void updateMenu()
{
    updateDisplay = true;
}

void updateIMU()
{
    updateData = true;
}

void updateBatteryStatus()
{
    updateBattery = true;
}

void sendBLEData()
{
    sendData = true;
}

void setupTimer()
{
    // Start the timers with the appropriate frequency (in seconds)
    menuTicker.attach(1.0 / UPDATE_RATE_MENU, updateMenu);
    imuTicker.attach(1.0 / UPDATE_RATE_IMU, updateIMU);
    battTicker.attach(1.0 / UPDATE_RATE_BATT, updateBatteryStatus);
    bleTicker.attach(1.0 / SEND_RATE_BLE, sendBLEData);
}