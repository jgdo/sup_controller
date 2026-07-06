#include <atomic>
#include <thread>
#include <vector>
#include <optional>
#include <algorithm>

#include <NimBLEDevice.h>
#include <TFT_eSPI.h>
#include <esp_bt.h>

#include "bt_settings.h"
#include "adc_sampling.h"

static constexpr int STEERING_ADC_PIN = 33;
static constexpr int PIN_POWER_ADC = 32;

static constexpr int PIN_BUTTON_SIDE = 25;
static constexpr int PIN_BUTTON_RIGHT = 26;
static constexpr int PIN_BUTTON_LEFT = 27;

static constexpr unsigned long reconnectIntervalMs = 1000;
static constexpr unsigned long commandSendIntervalMs = 50;
static constexpr unsigned long displayUpdateIntervalMs = 200;

// Low-latency BLE connection parameters.
// Units for interval are 1.25 ms, so 6 = 7.5 ms and 12 = 15 ms.
// Timeout unit is 10 ms, so 200 = 2 seconds.
static constexpr uint16_t bleMinConnectionInterval = 6;
static constexpr uint16_t bleMaxConnectionInterval = 12;
static constexpr uint16_t bleSlaveLatency = 0;
static constexpr uint16_t bleSupervisionTimeout = 200;

static constexpr int AUTO_POWER_HYSTERESIS = 5;

Joystick steeringJoystick{STEERING_ADC_PIN, 2600, 960, 180};
Joystick powerJoystick{PIN_POWER_ADC, 1680, 950, 100};

static std::optional<PowerStatus> latestStatus;

TFT_eSPI tft;

int steeringOffset = 0;

static const NimBLEAdvertisedDevice *foundMotorDevice = nullptr;
static NimBLEClient *bleClient = nullptr;
static NimBLERemoteCharacteristic *commandCharacteristic = nullptr;
static NimBLERemoteCharacteristic *statusCharacteristic = nullptr;

static bool bleConnected = false;
static unsigned long lastCommandSendMs = 0;
static unsigned long lastReconnectAttemptMs = 0;
static unsigned long lastDisplayUpdateMs = 0;

struct RemoteStatus
{
  AnalogValue steering;
  AnalogValue power;
};

static RemoteStatus latestRemoteStatus;

static bool inAutoPower = false;
static float autoPowerSetValue = 0;
static bool inAutoPilot = false;

void resetConnection()
{
  latestStatus = std::nullopt;

  bleConnected = false;
  commandCharacteristic = nullptr;
  statusCharacteristic = nullptr;

  inAutoPilot = false;
  inAutoPower = false;
}

class ScanCallbacks final : public NimBLEScanCallbacks
{
public:
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override
  {
    Serial.printf("Advertised Device found: %s\n", advertisedDevice->toString().c_str());
    if (!advertisedDevice->isAdvertisingService(NimBLEUUID(BLE_SERVICE_UUID)))
    {
      return;
    }

    Serial.print("Found motor controller: ");
    Serial.println(advertisedDevice->getAddress().toString().c_str());

    foundMotorDevice = advertisedDevice;

    NimBLEDevice::getScan()->stop();
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override
  {
    Serial.print("BLE scan ended, devices: ");
    Serial.print(results.getCount());
    Serial.print(", reason: ");
    Serial.println(reason);
  }
};

class ClientCallbacks final : public NimBLEClientCallbacks
{
public:
  void onConnect(NimBLEClient *client) override
  {
  }

  void onDisconnect(NimBLEClient *client, int reason) override
  {
    Serial.print("Disconnected from motor controller, reason: ");
    Serial.println(reason);

    resetConnection();
  }

  bool onConnParamsUpdateRequest(
      NimBLEClient *client,
      const ble_gap_upd_params *params) override
  {
    /*
     * Accept the motor's low-latency parameter update request.
     */
    return true;
  }
};

static ScanCallbacks scanCallbacks;
static ClientCallbacks clientCallbacks;

void onStatusNotification(
    NimBLERemoteCharacteristic *characteristic,
    uint8_t *data,
    size_t length,
    bool isNotify)
{
  if (length != sizeof(PowerStatus))
  {
    Serial.println("Received invalid motor status packet");
    return;
  }

  PowerStatus status;
  memcpy(&status, data, sizeof(status));
  latestStatus = status;
}

void startMotorScan()
{
  foundMotorDevice = nullptr;

  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(100);
  scan->setWindow(100);

  Serial.println("Scanning for motor controller");
  scan->start(2000, false);
}

void connectToMotorController()
{
  if (foundMotorDevice == nullptr)
  {

    if (!NimBLEDevice::getScan()->isScanning())
    {
      startMotorScan();
    }
    else
    {
      Serial.println("Scan already in progress");
    }

    return;
  }

  Serial.print("Connecting to motor controller: ");
  Serial.println(foundMotorDevice->getAddress().toString().c_str());

  if (bleClient == nullptr)
  {
    bleClient = NimBLEDevice::createClient();
    bleClient->setClientCallbacks(&clientCallbacks, false);
  }
  else if (bleClient->isConnected())
  {
    bleClient->disconnect();
  }

  if (!bleClient->connect(foundMotorDevice))
  {
    Serial.println("BLE connection failed");

    NimBLEDevice::deleteClient(bleClient);
    bleClient = nullptr;

    foundMotorDevice = nullptr;

    return;
  }

  bleClient->updateConnParams(
      bleMinConnectionInterval,
      bleMaxConnectionInterval,
      bleSlaveLatency,
      bleSupervisionTimeout);

  NimBLERemoteService *service =
      bleClient->getService(BLE_SERVICE_UUID);

  if (service == nullptr)
  {
    Serial.println("Motor service not found");
    bleClient->disconnect();
    return;
  }

  commandCharacteristic =
      service->getCharacteristic(BLE_REMOTE_COMMAND_CHARACTERISTICS);

  statusCharacteristic =
      service->getCharacteristic(BLE_POWER_STATUS_CHARACTERISTICS);

  if (commandCharacteristic == nullptr || statusCharacteristic == nullptr)
  {
    Serial.println("Required BLE characteristics not found");
    bleClient->disconnect();
    return;
  }

  if (statusCharacteristic->canNotify())
  {
    if (!statusCharacteristic->subscribe(true, onStatusNotification))
    {
      Serial.println("Failed to subscribe to motor status notifications");
    }
  }

  bleConnected = true;
  Serial.println("Connected to motor controller");
}

void maintainBleConnection()
{
  if (bleConnected && bleClient != nullptr && bleClient->isConnected())
  {
    return;
  }

  resetConnection();

  const unsigned long nowMs = millis();

  if (nowMs - lastReconnectAttemptMs < reconnectIntervalMs)
  {
    return;
  }

  lastReconnectAttemptMs = nowMs;
  connectToMotorController();
}

void updateDisplay()
{
  const unsigned long nowMs = millis();
  if (nowMs - lastDisplayUpdateMs < displayUpdateIntervalMs)
  {
    return;
  }
  lastDisplayUpdateMs = nowMs;

  tft.setTextFont(2);
  tft.setTextSize(1);
  tft.setCursor(0, 0);

  tft.setTextColor(bleConnected ? TFT_GREEN : TFT_RED, TFT_BLACK);
  if (bleConnected)
  {
    tft.println("Connected      ");
    tft.setTextFont(2);
    tft.println(foundMotorDevice->getAddress());
  }
  else
  {
    tft.println("Disconnected");
    tft.setTextFont(2);
    tft.println("                   ");
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  tft.setTextFont(2);
  tft.setCursor(0, 50);
  tft.printf("Pr: %4d    \n", latestRemoteStatus.power.raw);
  tft.printf("Sr: %4d    \n", latestRemoteStatus.steering.raw);
  tft.printf("Pa: %4.0f    \n", latestRemoteStatus.power.value);
  tft.printf("Sa: %4.0f    \n", latestRemoteStatus.steering.value);
  tft.println();

  if (latestStatus)
  {
    tft.printf("SUP V: %2.2f    \n", latestStatus->batteryVoltage_mV / 1000.0);
    tft.printf("SUP A: %2.2f    \n", latestStatus->motorCurrent_mA / 1000.0);
    tft.printf("SUP Ah: %2.2f    \n", latestStatus->usedEnergy_mAh / 1000.0);
    const int watt = (int)latestStatus->batteryVoltage_mV * (int)latestStatus->motorCurrent_mA / 1000000;
    tft.printf("SUP Watt: %4d    \n", watt);
    tft.printf("SUP U adc: %4d\n", latestStatus->batteryVoltage_adc);
    tft.printf("SUP I adc: %4d\n", latestStatus->motorCurrent_adc);
  }
  else
  {
    tft.println("                 ");
    tft.println("                 ");
    tft.println("                 ");
    tft.println("                 ");
    tft.println("                 ");
    tft.println("                 ");
  }

  tft.setCursor(80, 50);
  if (inAutoPower)
  {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.print("P: ");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("%3.0f", autoPowerSetValue);
  }
  else
  {
    tft.print("        ");
  }

  tft.setTextFont(4);
  tft.setCursor(80, 85);
   tft.setTextColor(TFT_RED, TFT_BLACK);
  if (inAutoPilot)
  {
    tft.print("AP");
  }
  else
  {
    tft.print("       ");
  }
}

std::pair<bool, bool> readButtons()
{
  const bool left = !digitalRead(PIN_BUTTON_LEFT);
  const bool right = !digitalRead(PIN_BUTTON_RIGHT);

  return {left, right};
}

AnalogValue readSteering()
{
  static std::pair<bool, bool> lastButtons;

  const auto buttons = readButtons();
  if (buttons.first && !lastButtons.first)
  {
    steeringOffset += 1;
  }
  if (buttons.second && !lastButtons.second)
  {
    steeringOffset -= 1;
  }

  lastButtons = buttons;

  auto steering = steeringJoystick.read();
  float angle = (steering.value + steeringOffset - 90) / 90;
  constexpr float expo = 1.5;
  if (angle < 0)
  {
    angle = -std::pow(-angle, expo);
  }
  else
  {
    angle = std::pow(angle, expo);
  }
  steering.value = std::clamp<float>(angle * 90 + 90, 0, 180);
  return steering;
}

int sidePressCount(bool pressed, bool reset = false)
{
  static int count = 0;
  static long lastPress_ms = 0;
  static bool lastPressed = false;

  TODO debounce!

  static constexpr long MAX_INTERVAL_MS = 500;

  const auto now = millis();
  const auto interval_ms = now - lastPress_ms;
  const bool edge = pressed != lastPressed;

  if (reset)
  {
    count = 0;
  }
  else if (edge)
  {
    if (interval_ms <= MAX_INTERVAL_MS)
    {
      if (!pressed)
      {
        count++;
      }
    } else {
      count = 0;
    }
  }

  lastPress_ms = now;
  lastPressed = pressed;

  return count;
}

void updateRemoteStatus()
{
  latestRemoteStatus.steering = readSteering();
  latestRemoteStatus.power = powerJoystick.read();
  if (latestRemoteStatus.power.value <= 3)
  {
    latestRemoteStatus.power.value = 0;
  };
  const bool powerApplied = latestRemoteStatus.power.value > 0;

  const bool sidePressed = !digitalRead(PIN_BUTTON_SIDE);

  // Auto pilot handling
  const int pressCount = sidePressCount(sidePressed, powerApplied);
  if(pressCount == 3)
  {
    inAutoPilot = true;
    sidePressCount(false, true); // reset
  }

  // get original joystick value without offset
  const auto steeringPure = steeringJoystick.convertRaw(latestRemoteStatus.steering.raw) - 90.0f;
  if(abs(steeringPure) > 5.0f)
  {
    if(inAutoPilot) {
      inAutoPower = false; // disable auto power when disabling auto pilot
    }
    inAutoPilot = false;
  }

  // Auto power handling
  static float minPowerValue = 0;
  if (sidePressed && powerApplied)
  {
    inAutoPower = true;
    autoPowerSetValue = minPowerValue = latestRemoteStatus.power.value;
  }

  if (inAutoPower)
  {
    if (latestRemoteStatus.power.value < minPowerValue)
    {
      minPowerValue = latestRemoteStatus.power.value;
    }
    else if (latestRemoteStatus.power.value > minPowerValue + AUTO_POWER_HYSTERESIS)
    {
      inAutoPower = false;
    }
  }

  if (inAutoPower)
  {
    latestRemoteStatus.power.value = autoPowerSetValue;
  }
}

void sendRemoteCommand()
{
  if (!bleConnected || commandCharacteristic == nullptr)
  {
    return;
  }

  const unsigned long nowMs = millis();

  if (nowMs - lastCommandSendMs < commandSendIntervalMs)
  {
    return;
  }

  lastCommandSendMs = nowMs;

  RemoteCommand control;
  control.powerPercent = std::round(latestRemoteStatus.power.value);
  control.steeringAngleDeg = std::round(latestRemoteStatus.steering.value);
  control.autopilotEnabled = inAutoPilot ? AUTOPILOT_ENABLE_TAG : 0;

  const bool writeOk = commandCharacteristic->writeValue(
      reinterpret_cast<const uint8_t *>(&control),
      sizeof(control),
      false);

  if (!writeOk)
  {
    Serial.println("Failed to send remote command");
    resetConnection();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("setup() start");

  Joystick::startSampling(1);

  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_SIDE, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  // tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_WHITE, TFT_BLACK, true);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextPadding(100);

  NimBLEDevice::init(BT_NAME_REMOTE);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  Serial.println("Remote ready");
}

void loop()
{
  maintainBleConnection();
  updateRemoteStatus();
  sendRemoteCommand();
  updateDisplay();

  delay(5);
}
