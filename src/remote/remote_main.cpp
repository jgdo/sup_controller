#include <atomic>
#include <thread>
#include <vector>
#include <optional>
#include <algorithm>

#include <ArduinoBLE.h>
#include <TFT_eSPI.h>
#include <esp_bt.h>

#include "bt_settings.h"
#include "adc_sampling.h"

BLEService remoteService(BLE_UUID);
BLEUnsignedIntCharacteristic controlCharacteristic{BLE_CONTROL_CHARACTERISTICS, BLERead | BLENotify};
BLETypedCharacteristic<PowerStatus> powerStatusCharacteristic{BLE_POWER_STATUS_CHARACTERISTICS, BLEWrite};

static constexpr int STEERING_ADC_PIN = 33;
static constexpr int PIN_POWER_ADC = 32;

static constexpr int PIN_BUTTON_RIGHT = 26;
static constexpr int PIN_BUTTON_LEFT = 27;

Joystick steeringJoystick{STEERING_ADC_PIN, 2600, 960, 180};
Joystick powerJoystick{PIN_POWER_ADC, 1700, 950, 100};

TFT_eSPI tft;

int steeringOffset = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("setup() start");

  Joystick::startSampling(1);

  pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_GREEN);
  // tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_BLACK, TFT_GREEN, true);
  tft.setTextFont(4);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);
  tft.setTextPadding(100);

  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    tft.println("Starting BLE failed!");
    while (1)
      ;
  }

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

  BLE.setLocalName(BT_NAME_CONTOLLER);
  BLE.setAdvertisedService(remoteService);
  remoteService.addCharacteristic(controlCharacteristic);
  remoteService.addCharacteristic(powerStatusCharacteristic);
  BLE.addService(remoteService);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  Serial.println("setup() end");
}

void drawDebugScreen(const BLEDevice &central, const AnalogValue &power, const AnalogValue &steering, const std::optional<PowerStatus> &powerStatus)
{
  tft.setTextFont(2);
  tft.setCursor(0, 0);
  if (central)
  {
    tft.println("Connected      ");
    tft.setTextFont(2);
    tft.println(central.address());
  }
  else
  {
    tft.println("Disconnected");
  }

  tft.setTextFont(2);
  tft.setCursor(0, 50);
  tft.printf("Pr: %4d    \n", power.raw);
  tft.printf("Sr: %4d    \n", steering.raw);
  tft.printf("Pa: %4.0f    \n", power.value);
  tft.printf("Sa: %4.0f    \n", steering.value);
  tft.println();
  if (powerStatus)
  {
    tft.printf("SUP mV: %4d    \n", powerStatus->batteryVoltage_mV);
    tft.printf("SUP mA: %4d    \n", powerStatus->motorCurrent_mA);
    tft.printf("SUP mAh: %4d    \n", powerStatus->usedEnergy_mAh);
    const int watt = (int)powerStatus->batteryVoltage_mV * (int)powerStatus->motorCurrent_mA / 1000000;
    tft.printf("SUP Watt: %4d    \n", watt);
    tft.printf("SUP U adc: %4d\n", powerStatus->batteryVoltage_adc);
    tft.printf("SUP I adc: %4d\n", powerStatus->motorCurrent_adc);
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
  steering.value = std::clamp<float>(steering.value + steeringOffset, 0, 180);
  return steering;
}

void loop()
{
  BLEDevice central = BLE.central();

  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    std::optional<PowerStatus> status;
    while (central.connected())
    {
      const auto steering = readSteering();
      const auto power = powerJoystick.read();

      ControlUnion control;
      control.values.powerPercent = std::round(power.value);
      control.values.steeringAngleDeg = std::round(steering.value);
      controlCharacteristic.writeValue(control.bleValue);

      if (powerStatusCharacteristic.written())
      {
        status = powerStatusCharacteristic.value();
      }
      // const auto result = powerStatusCharacteristic.value((uint8_t *)&status, sizeof(status));
      // drawDebugScreen(central, power, steering, result ? &status : nullptr);
      drawDebugScreen(central, power, steering, status);

      delay(10);
    }
  }

  const auto steering = readSteering();
  const auto power = powerJoystick.read();
  drawDebugScreen(central, power, steering, std::nullopt);

  delay(100);
}
