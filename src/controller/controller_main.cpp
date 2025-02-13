#include <ArduinoBLE.h>
#include <TFT_eSPI.h>

BLEService controllerService("1101");
BLEUnsignedIntCharacteristic controlCharacteristic("2101", BLERead | BLENotify);

#include "bt_settings.h"

static constexpr int STEERING_ADC_PIN = 33;
static constexpr int PIN_POWER_ADC = 32;

class Joystick
{
public:
  Joystick(int adcPin, int minValue, int maxValue) : mAdcPin{adcPin}, mMinValue{minValue}, mMaxValue{maxValue} {}

  int readPercent()
  {
    const auto raw = analogRead(mAdcPin);

    if (raw <= mMinValue)
    {
      return 0;
    }
    else if (raw >= mMaxValue)
    {
      return 100;
    }

    return (raw - mMinValue) * 100 / (mMaxValue - mMinValue);
  }

private:
  const int mAdcPin;
  const int mMinValue;
  const int mMaxValue;
};

Joystick steeringJoystick{STEERING_ADC_PIN, 1200, 2260};
Joystick powerJoystick{PIN_POWER_ADC, 900, 1720};

TFT_eSPI tft;

void setup()
{
  Serial.begin(115200);
  Serial.println("setup() start");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_GREEN);
  // tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextColor(TFT_BLACK, TFT_GREEN, true);
  tft.setTextFont(4);
  tft.setTextPadding(100);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    tft.println("Starting BLE failed!");
    while (1)
      ;
  }

  BLE.setLocalName(BT_NAME_CONTOLLER);
  BLE.setAdvertisedService(controllerService);
  controllerService.addCharacteristic(controlCharacteristic);
  BLE.addService(controllerService);

  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");

  Serial.println("setup() end");
}

void loop()
{
  BLEDevice central = BLE.central();

  if (central)
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    tft.setCursor(0, 0);
    tft.printf("BLE connected %s", central.address().c_str());

    while (central.connected())
    {
      const auto steeringRaw = analogRead(STEERING_ADC_PIN);
      const auto powerRaw = analogRead(PIN_POWER_ADC);

      const auto angle = steeringJoystick.readPercent() * 180 / 100;
      const auto power = 100 - powerJoystick.readPercent();

      tft.setCursor(0, 120);
      tft.printf("Pr: %4d    \n", powerRaw);
      tft.printf("Sr: %4d    \n", steeringRaw);
      tft.printf("Pa: %4d    \n", power);
      tft.printf("Sa: %4d    \n", angle);

      ControlUnion control;
      control.values.powerPercent = power;
      control.values.steeringAngleDeg = angle;
      controlCharacteristic.writeValue(control.bleValue);

      delay(100);
    }
  }

  tft.setCursor(0, 0);
  tft.print("BLE disconnected                                      ");

  const auto steeringRaw = analogRead(STEERING_ADC_PIN);
  const auto powerRaw = analogRead(PIN_POWER_ADC);

  const auto angle = steeringJoystick.readPercent() * 180 / 100;
  const auto power = 100 - powerJoystick.readPercent();

  tft.setCursor(0, 120);
  tft.printf("Pr: %4d    \n", powerRaw);
  tft.printf("Sr: %4d    \n", steeringRaw);
  tft.printf("Pa: %4d    \n", power);
  tft.printf("Sa: %4d    \n", angle);
  delay(100);
}
