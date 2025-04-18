#include <sstream>
#include <optional>

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoBLE.h>

#include "bt_settings.h"

static constexpr int CURRENT_SENSING_ADC_PIN = 39;
static constexpr int RUDDER_SERVO_PIN = 14;
static constexpr int MOTOR_SERVO_PIN = 27;

static constexpr long POWER_RECEIVE_TIMEOUT_MS = 500;
static constexpr float CONTROL_FREQ = 50.0f;

Servo steeringServo, powerServo;

void setup()
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  steeringServo.attach(RUDDER_SERVO_PIN, 1000, 2000);
  powerServo.attach(MOTOR_SERVO_PIN, 1000, 2000);
  powerServo.write(0);

  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1)
      ;
  }

  Serial.println("Started BLE.");

  Serial.println("Canning for service ...");
  BLE.scanForUuid(BLE_UUID);

  Serial.println("setup() done.");
}

std::array<int, 16> valuesArray;
int valueCounter = 0;

std::optional<Control> readControl(BLECharacteristic &characteristic)
{
  ControlUnion control;
  if (characteristic.readValue(control.bleValue) == 0)
  {
    return std::nullopt;
  }

  return control.values;
}

void sendStatus(BLECharacteristic &characteristic, const PowerStatus &status)
{
  characteristic.writeValue((uint8_t *)&status, sizeof(status));
}

void loop()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  BLEDevice peripheral = BLE.available();

  if (peripheral)
  {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
    BLE.stopScan();

    if (peripheral.localName() != BT_NAME_CONTOLLER)
    {
      Serial.println("Remote name does not match, ignoring");
      sleep(500);
      return;
    }

    peripheral.connect();

    peripheral.discoverAttributes();
    BLEService service = peripheral.service(BLE_UUID);
    Serial.print("Service ");
    Serial.print(service.uuid());
    BLECharacteristic controlCharacteristic = service.characteristic(BLE_CONTROL_CHARACTERISTICS);
    BLECharacteristic statusCharacteristic = service.characteristic(BLE_POWER_STATUS_CHARACTERISTICS);

    Serial.print("\tCharacteristic ");
    Serial.println(controlCharacteristic.uuid());
    Serial.printf("Characteristics size: %d\n", controlCharacteristic.valueSize());
    Serial.printf("Characteristics can read: %d\n", controlCharacteristic.canRead());
    Serial.println(statusCharacteristic.uuid());
    Serial.printf("Characteristics size: %d\n", statusCharacteristic.valueSize());
    Serial.printf("Characteristics can read: %d\n", statusCharacteristic.canRead());

    while (peripheral)
    {
      digitalWrite(LED_BUILTIN, 1);

      const auto optInput = readControl(controlCharacteristic);
      if (!optInput)
      {
        break;
      }

      const auto input = *optInput;

      steeringServo.write(input.steeringAngleDeg);
      powerServo.write(input.powerPercent * 180 / 100);

      const auto currentAdc = analogRead(CURRENT_SENSING_ADC_PIN);
      valuesArray[valueCounter++] = currentAdc;
      if (valueCounter >= valuesArray.size())
      {
        valueCounter = 0;
      }

      int sum = 0;
      for (auto v : valuesArray)
      {
        sum += v;
      }

      sum = (sum + valuesArray.size() / 2) / valuesArray.size();

      Serial.printf("Current adc %4d, power: %4d, steering: %4d\n", sum, input.powerPercent, input.steeringAngleDeg);

      PowerStatus status{.motorCurrent_mA = sum, .usedEnergy_mAh = 7890, .batteryVoltage_mV = 1337};
      sendStatus(statusCharacteristic, status);

      delay(20);
    }

    powerServo.write(0);
    digitalWrite(LED_BUILTIN, 0);

    Serial.println("ble remote disconnected, stopping.");

    Serial.println("Canning for service ...");
    BLE.scanForUuid(BLE_UUID);
  }
  else
  {
    Serial.println("ble service not available");
  }

  powerServo.write(0);
  delay(500);
}
