#include "ble.h"
#include "commands.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// clang-format off
constexpr const char *SERVICE_UUID = "df6a7398-901f-4824-a382-44cbc0e324ee";
constexpr const char *CHARACTERISTIC_UUID = "5ac98c9f-065b-4bf1-a950-be6972faf151";
// clang-format on

static BLEServer *server = nullptr;
static BLECharacteristic *characteristic = nullptr;
static bool is_connected = false;

bool ble_is_connected() { return is_connected; }

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    is_connected = true;
    Serial.println("[BLE] Device connected");
  }

  void onDisconnect(BLEServer *pServer) override {
    is_connected = false;
    Serial.println("[BLE] Device disconnected, restarting advertising...");
    pServer->getAdvertising()->start();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    uint8_t *data = pCharacteristic->getData();
    size_t data_length = pCharacteristic->getLength();
    if (data == nullptr || data_length == 0) {
      return;
    }

    char buffer[128];
    size_t copy_length =
        data_length < sizeof(buffer) - 1 ? data_length : sizeof(buffer) - 1;
    memcpy(buffer, data, copy_length);
    buffer[copy_length] = '\0';

    char *saveptr = nullptr;
    char *command = strtok_r(buffer, " \r\n\t", &saveptr);
    if (command == nullptr) {
      return;
    }

    char *args = saveptr;
    while (args && (*args == ' ' || *args == '\t')) {
      args++;
    }

    handle_command(command, args);
  }
};

void init_ble(const char *device_name) {
  BLEDevice::init(device_name);

  server = BLEDevice::createServer();
  server->setCallbacks(new MyServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  characteristic = service->createCharacteristic(
      CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_READ |
                               BLECharacteristic::PROPERTY_WRITE |
                               BLECharacteristic::PROPERTY_NOTIFY);

  characteristic->setCallbacks(new MyCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.printf("[BLE] Initialized and advertising as '%s'\n", device_name);
}

void notify(const char *fmt, ...) {
  if (!characteristic || !is_connected) {
    return;
  }

  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  characteristic->setValue(buffer);
  characteristic->notify();
}

void ble_send(const uint8_t *data, size_t length) {
  if (!characteristic || !is_connected || data == nullptr || length == 0) {
    return;
  }
  characteristic->setValue((uint8_t *)data, length);
  characteristic->notify();
}
