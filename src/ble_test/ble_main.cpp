// BLE peripheral example for ESP32 (Arduino) using NimBLE-Arduino 2.5.0
//
// Starts a GATT server exposing one notifiable int32 characteristic.
// The value is incremented periodically and notified to all subscribed clients.
// Supports multiple simultaneous central (client) connections.
//
// Build env in platformio.ini: [env:BLE_test]

#include <Arduino.h>
#include <NimBLEDevice.h>

// ---- UUIDs (feel free to change) -------------------------------------------
static constexpr const char* DEVICE_NAME          = "sup_ble_demo";
static constexpr const char* SERVICE_UUID         = "9f7f2000-2c6d-4f42-8d18-19f0f8b00001";
static constexpr const char* DUMMY_CHAR_UUID      = "9f7f2001-2c6d-4f42-8d18-19f0f8b00001";

// ---- Multi-connection configuration ----------------------------------------
// NimBLE must be built with enough connection slots. We also tell the stack
// to keep advertising after a client connects so additional centrals can join.
static constexpr uint8_t MAX_CONNECTIONS = 4;

// ---- Globals ---------------------------------------------------------------
static NimBLEServer*         g_server      = nullptr;
static NimBLECharacteristic* g_dummyChar   = nullptr;
static volatile int32_t      g_dummyValue  = 0;

// ---- Server callbacks ------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.printf("[BLE] Client connected, handle=%u, total=%u\n",
                      connInfo.getConnHandle(),
                      pServer->getConnectedCount());

        // Keep advertising so additional centrals can connect concurrently,
        // as long as we still have free connection slots.
        if (pServer->getConnectedCount() < MAX_CONNECTIONS) {
            NimBLEDevice::getAdvertising()->start();
        }
    }

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.printf("[BLE] Client disconnected, handle=%u, reason=0x%x, remaining=%u\n",
                      connInfo.getConnHandle(),
                      reason,
                      pServer->getConnectedCount());

        // Make sure we're advertising again after a disconnect.
        NimBLEDevice::getAdvertising()->start();
    }

    void onMTUChange(uint16_t MTU, NimBLEConnInfo& connInfo) override {
        Serial.printf("[BLE] MTU updated: %u (handle=%u)\n", MTU, connInfo.getConnHandle());
    }
};

// ---- Characteristic callbacks (optional, only for logging) -----------------
class DummyCharCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pCharacteristic,
                     NimBLEConnInfo& connInfo,
                     uint16_t subValue) override {
        const char* state = "unknown";
        switch (subValue) {
            case 0: state = "unsubscribed";          break;
            case 1: state = "notifications enabled"; break;
            case 2: state = "indications enabled";   break;
            case 3: state = "notify+indicate";       break;
        }
        Serial.printf("[BLE] Subscribe change from handle=%u: %s\n",
                      connInfo.getConnHandle(), state);
    }

    void onRead(NimBLECharacteristic* pCharacteristic,
                NimBLEConnInfo& connInfo) override {
        Serial.printf("[BLE] Read from handle=%u, value=%ld\n",
                      connInfo.getConnHandle(),
                      (long)g_dummyValue);
    }
};

// ---- Setup -----------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n[BLE] Starting NimBLE peripheral example");

    // Initialize NimBLE
    NimBLEDevice::init(DEVICE_NAME);

    // Allow up to MAX_CONNECTIONS simultaneous connections.
    // (Underlying limit is also governed by CONFIG_BT_NIMBLE_MAX_CONNECTIONS
    //  in the NimBLE config; the default in NimBLE-Arduino is high enough.)
    NimBLEDevice::setMTU(247);

    // Create server
    g_server = NimBLEDevice::createServer();
    g_server->setCallbacks(new ServerCallbacks());

    // Important: do NOT auto-stop advertising on connect,
    // so several centrals can connect one after another.
    g_server->advertiseOnDisconnect(true);

    // Create service & characteristic
    NimBLEService* service = g_server->createService(SERVICE_UUID);

    g_dummyChar = service->createCharacteristic(
        DUMMY_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    g_dummyChar->setCallbacks(new DummyCharCallbacks());

    // Initial value
    int32_t initial = 0;
    g_dummyChar->setValue(reinterpret_cast<uint8_t*>(&initial), sizeof(initial));

    // Configure advertising
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName(DEVICE_NAME);
    adv->addServiceUUID(service->getUUID());
    adv->enableScanResponse(true);
    adv->start();

    Serial.printf("[BLE] Advertising as \"%s\" (service %s)\n",
                  DEVICE_NAME, SERVICE_UUID);
}

// ---- Loop: periodically update and notify ----------------------------------
void loop() {
    static uint32_t lastUpdate = 0;
    const uint32_t now = millis();

    if (now - lastUpdate >= 1000) {
        lastUpdate = now;

        g_dummyValue++;

        // Update characteristic value (read clients will see this too)
        g_dummyChar->setValue(reinterpret_cast<uint8_t*>(const_cast<int32_t*>(&g_dummyValue)),
                              sizeof(g_dummyValue));

        // Notify ALL currently subscribed centrals.
        // NimBLE's notify() iterates over every connected & subscribed client.
        if (g_server->getConnectedCount() > 0) {
            g_dummyChar->notify();
            Serial.printf("[BLE] Notified value=%ld to %u client(s)\n",
                          (long)g_dummyValue,
                          g_server->getConnectedCount());
        }
    }

    delay(10);
}