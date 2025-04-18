#include <atomic>
#include <thread>
#include <vector>
#include <optional>

#include <ArduinoBLE.h>
#include <TFT_eSPI.h>

#include "bt_settings.h"

BLEService remoteService(BLE_UUID);
BLEUnsignedIntCharacteristic controlCharacteristic{BLE_CONTROL_CHARACTERISTICS, BLERead | BLENotify};
BLETypedCharacteristic<PowerStatus> powerStatusCharacteristic{BLE_POWER_STATUS_CHARACTERISTICS, BLEWrite};

static constexpr int STEERING_ADC_PIN = 33;
static constexpr int PIN_POWER_ADC = 32;

struct AnalogValue
{
  int raw;
  int value;
  unsigned long lastMillis;
};

class Joystick
{
public:
  Joystick(int adcPin, int minValue, int maxValue) : mAdcPin{adcPin}, mMinValue{minValue}, mMaxValue{maxValue}
  {
    const uint32_t raw = analogRead(mAdcPin);
    std::fill(mAvgBuffer.begin(), mAvgBuffer.end(), raw);
    mAvgSum = raw * mAvgBuffer.size();

    sAllJoysticks.push_back(this);
  }

  ~Joystick()
  {
    sAllJoysticks.erase(std::remove(sAllJoysticks.begin(), sAllJoysticks.end(), this), sAllJoysticks.end());
  }

  AnalogValue read(int range)
  {
    const auto raw = mLastAvgValue.load();
    const auto lastMillis = mLastSamplingMillis.load();

    const auto minValue = std::min(mMinValue, mMaxValue);
    const auto maxValue = std::max(mMinValue, mMaxValue);

    auto value = (raw - minValue) * range / (maxValue - minValue);
    if (raw <= minValue)
    {
      value = 0;
    }
    else if (raw >= maxValue)
    {
      value = range;
    }

    if (mMaxValue < mMinValue)
    {
      value = range - value;
    }

    return {raw, value, lastMillis};
  }

  static void runSamplingLoop(unsigned long delay_ms)
  {
    while (true)
    {
      for (Joystick *j : sAllJoysticks)
      {
        j->sampleAdc();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
  }

private:
  static constexpr auto AVG_BUFFER_SIZE = 32;
  static std::vector<Joystick *> sAllJoysticks;

  const int mAdcPin;
  const int mMinValue;
  const int mMaxValue;

  uint32_t mAvgSum = 0;
  std::array<uint32_t, AVG_BUFFER_SIZE> mAvgBuffer;
  size_t mBufferIndex = 0;

  std::atomic<uint32_t> mLastAvgValue;
  std::atomic<unsigned long> mLastSamplingMillis; // such that main thread knows that sampling thread did not die

  void sampleAdc()
  {
    const uint32_t raw = analogRead(mAdcPin);
    mAvgSum -= mAvgBuffer.at(mBufferIndex);
    mAvgSum += raw;
    mAvgBuffer.at(mBufferIndex) = raw;

    mBufferIndex = (mBufferIndex + 1) % mAvgBuffer.size();

    mLastAvgValue = (mAvgSum + mAvgBuffer.size() / 2) / mAvgBuffer.size();
    mLastSamplingMillis = millis();
  }
};

std::vector<Joystick *> Joystick::sAllJoysticks;

Joystick steeringJoystick{STEERING_ADC_PIN, 1430, 2410};
Joystick powerJoystick{PIN_POWER_ADC, 1700, 950};

std::thread joystickThread{Joystick::runSamplingLoop, 1};

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
  tft.setCursor(0, 70);
  tft.printf("Pr: %4d    \n", power.raw);
  tft.printf("Sr: %4d    \n", steering.raw);
  tft.printf("Pa: %4d    \n", power.value);
  tft.printf("Sa: %4d    \n", steering.value);
  tft.println();
  if (powerStatus)
  {
    tft.printf("SUP mV: %4d    \n", powerStatus->batteryVoltage_mV);
    tft.printf("SUP mA: %4d    \n", powerStatus->motorCurrent_mA);
    tft.printf("SUP mAh: %4d    \n", powerStatus->usedEnergy_mAh);
  }
  else
  {
    tft.println("                 ");
    tft.println("                 ");
    tft.println("                 ");
  }
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
      const auto steering = steeringJoystick.read(180);
      const auto power = powerJoystick.read(100);

      ControlUnion control;
      control.values.powerPercent = power.value;
      control.values.steeringAngleDeg = steering.value;
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

  const auto steering = steeringJoystick.read(180);
  const auto power = powerJoystick.read(100);
  drawDebugScreen(central, power, steering, std::nullopt);

  delay(100);
}
