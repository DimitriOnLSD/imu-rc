#ifndef BLE_H
#define BLE_H

#define SERVICE_UUID BLEUUID("3e3f1b05-90a8-43af-aa6b-335b28282b57")
#define CHARACTERISTIC_UUID BLEUUID("3e2c8a88-90fd-4b6b-b947-997e80d03d5b")

#define SCAN_DURATION 5

BLEScan *pBLEScan;
BLEClient *pClient;
BLERemoteCharacteristic *pRemoteCharacteristic;

void setupBLE();
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify);
BLEClient *connectToServer(BLEAddress serverAddress);
bool scanForServer();

#include "ble.cpp"

#endif
