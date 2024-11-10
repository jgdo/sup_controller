#include <BluetoothSerial.h>
#include <TFT_eSPI.h>

#include "bt_settings.h"

static constexpr int STEERING_ADC_PIN = 25;
static constexpr int POWER_ADC_PIN = 33;

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
  int mAdcPin;
  int mMinValue;
  int mMaxValue;
};

Joystick steeringJoystick{STEERING_ADC_PIN, 1320, 2390};
Joystick powerJoystick{POWER_ADC_PIN, 1790, 2500};

BluetoothSerial SerialBT;
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
  tft.println("Connecting with SUP ...");

  // tft.drawString("--- ADC ---", tft.width() / 2, tft.height() / 2 - 16);

  // tft.setCursor(0, 100);
  // tft.printf("Connecting to %s ...\n", BT_NAME_MOTOR);

  SerialBT.begin(BT_NAME_CONTOLLER, true);
  const auto connected = SerialBT.connect(BT_NAME_MOTOR);

  const auto conStr = connected ? "Connected" : "Not connected";
  tft.println(conStr);

  Serial.println("setup() end");
}

void loop()
{
  const auto steeringRaw = analogRead(STEERING_ADC_PIN);
  const auto powerRaw = analogRead(POWER_ADC_PIN);

  const auto angle = steeringJoystick.readPercent() * 180 / 100;
  const auto power = powerJoystick.readPercent() * 180 / 100;
  SerialBT.printf("s %d\n", angle);
  SerialBT.printf("p %d\n", power);

  tft.setCursor(0, 120);
  tft.printf("Pr: %4d    \n", powerRaw);
  tft.printf("Sr: %4d    \n", steeringRaw);
  tft.printf("Pa: %4d    \n", power);
  tft.printf("Sa: %4d    \n", angle);

  delay(100);
}
