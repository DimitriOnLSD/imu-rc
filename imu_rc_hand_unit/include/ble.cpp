// Setup BLE device
void setupBLE() {
  // Initialize BLE device as a client
  BLEDevice::init("IMU-RC Hand Unit");
  pBLEScan = BLEDevice::getScan();  // Create BLE scan object
  pBLEScan->setActiveScan(true);    // Enable active scanning
}

// Callback function for BLE notifications
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  String notificationValue = "";
  for (size_t i = 0; i < length; i++) {
    notificationValue += (char)pData[i];
  }
  debugPrint(false, 0, 0, 0, "Notification received: " + notificationValue);
  serverBatteryPercentage = notificationValue.toInt();
}

// Function to connect to the server and return the connected BLE client
BLEClient *connectToServer(BLEAddress serverAddress) {
  // Create a new BLE client instance
  pClient = BLEDevice::createClient();

  // Attempt to connect to the server using its address
  debugPrint(false, 0, 0, 0, "Connecting to IMU-RC Car: ");
  debugPrint(false, 0, 0, 0, serverAddress.toString().c_str());

  if (pClient->connect(serverAddress)) {
    debugPrint(false, 0, 0, 0, "Connected to IMU-RC Car.");
    clientConnectionAttempt = true;
    clientIsConnected = true;
    clientIsScanning = false;
  } else {
    debugPrint(false, 0, 0, 0, "Failed to connect to IMU-RC Car.");
    return nullptr;
  }

  // Check if the service exists on the server
  BLERemoteService *pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    debugPrint(false, 0, 0, 0, "Failed to find IMU-RC Car service.");
    pClient->disconnect();
    return nullptr;
  }

  // Get the characteristic from the service
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
  if (pRemoteCharacteristic == nullptr) {
    debugPrint(false, 0, 0, 0, "Failed to find IMU-RC Car characteristic.");
    pClient->disconnect();
    return nullptr;
  }

  // Register for notifications
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    debugPrint(false, 0, 0, 0, "Registered for notifications.");
  }

  // Return the connected client
  return pClient;
}

// Scan for the server and connect to it
bool scanForServer() {
  debugPrint(false, 0, 0, 0, "Scanning for IMU-RC Car servers...");

  // Start BLE scan and look for devices
  BLEScanResults results = pBLEScan->start(SCAN_DURATION);
  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);

    // Check if the device advertises the service UUID we're looking for
    if (device.haveServiceUUID() && device.isAdvertisingService(SERVICE_UUID)) {

      display.clearDisplay();
      drawText("Connecting...", 1, -1, -1);
      display.display();
      SetLEDStatus("PURPLE", "ON");
      debugPrint(false, 0, 0, 0, "Found IMU-RC Car. Attempting to connect...");

      // Connect to the server
      BLEClient *pClient = connectToServer(device.getAddress());
      if (pClient != nullptr) {
        pBLEScan->clearResults();  // Clear scan results
        return true;               // Return true if connection is successful
      }
    }
  }

  pBLEScan->clearResults();  // Clear scan results
  return false;              // Return false if no connection was made
}