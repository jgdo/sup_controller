#include <sstream>
#include <optional>
#include <mutex>

#include <Arduino.h>
#include <ESP32Servo.h>
#include <ArduinoBLE.h>
#include <esp_bt.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

#define USE_DSHOT 0

#if USE_DSHOT
#include <DShotRMT.h>
#else
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#endif

#include "adc_sampling.h"
#include "bt_settings.h"

#if defined(C3_SUPERMINI)
#define DSHOT_SERIAL Serial
constexpr auto PIN_BLUE_LED = 8;
#elif defined(WEMOS_D1)
#define DSHOT_SERIAL Serial2
#endif

#if defined(C3_SUPERMINI)
static constexpr int CURRENT_SENSING_ADC_PIN = 0;
static constexpr int VOLTAGE_SENSING_ADC_PIN = 1;
static constexpr int RUDDER_SERVO_PIN = 10;
static constexpr int MOTOR_SERVO_PIN = 21;

constexpr auto PIN_TO_ESC = 21;
constexpr auto PIN_FROM_ESC = 20;

#elif defined(WEMOS_D1)
static constexpr int CURRENT_SENSING_ADC_PIN = 39;
static constexpr int VOLTAGE_SENSING_ADC_PIN = 36;
static constexpr int RUDDER_SERVO_PIN = 14;
static constexpr int MOTOR_SERVO_PIN = 27;

constexpr auto PIN_TO_ESC = 17;
constexpr auto PIN_FROM_ESC = 16;
#endif

static constexpr long POWER_RECEIVE_TIMEOUT_MS = 500;
static constexpr float CONTROL_FREQ = 50.0f;

static constexpr float V_PER_STEP = 0.006091;

ThreeWire myWire(6, 5, 7); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

Servo steeringServo;

#if !USE_DSHOT
Servo motorServo;
#else
constexpr auto DSHOT_MODE = DSHOT600;
DShotRMT anESC(PIN_TO_ESC);
#endif

constexpr auto DSHOT_LOOP_TIMEOUT_MS = 300;

Joystick currentSensor_A{CURRENT_SENSING_ADC_PIN, 3642, 865, 70.0}; // WCS1700, 70A, 32mV/A
Joystick batteryVoltageSensor_V{VOLTAGE_SENSING_ADC_PIN, 0, 4095, 4095 * V_PER_STEP};

struct MotorValue
{
  int power;
  int servoSteeringAngle;
  unsigned long timestamp;
};

struct DshotFeedback
{
  uint32_t temp_C;
  uint32_t voltage_mV;
  uint32_t current_mA;
  uint32_t consumption_mAh;
  uint32_t rpm;
  uint16_t voltage_adc;
  uint16_t current_adc;
};

std::mutex motorMutex;
volatile MotorValue motorSetpointValue = {0, 90, 0};
volatile DshotFeedback lastMotorFeedback;

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for (i = 0; i < 8; i++)
    crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
  return crc_u;
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen)
{
  uint8_t crc = 0, i;
  for (i = 0; i < BufLen; i++)
    crc = update_crc8(Buf[i], crc);
  return crc;
}

std::tuple<int, int> getDshotMotorValueAndSteering()
{
  motorMutex.lock();
  const MotorValue lastVal = {motorSetpointValue.power, motorSetpointValue.servoSteeringAngle, motorSetpointValue.timestamp};
  motorMutex.unlock();

  const auto ms = millis();

  if ((lastVal.timestamp > ms) || (ms - lastVal.timestamp > DSHOT_LOOP_TIMEOUT_MS) || lastVal.power < 0 || lastVal.power > 2000)
  {
    Serial.println("Invalid power or timeout from main thread!");
    return {DSHOT_THROTTLE_MIN, lastVal.servoSteeringAngle};
  }

  return {std::min<int>(DSHOT_THROTTLE_MIN + lastVal.power, DSHOT_THROTTLE_MAX), lastVal.servoSteeringAngle};
}

void dshotLoop()
{
  uint8_t buffer[10] = {42};
  int na_count = 0;

  while (true)
  {

    const auto [power, steering] = getDshotMotorValueAndSteering();
#if USE_DSHOT
    anESC.send_dshot_value(power == DSHOT_THROTTLE_MIN ? 0 : power, ENABLE_TELEMETRIC);
    // Serial.println(power);

    if (Serial2.available() == 0)
    {
      na_count++;
    }
    else
    {
      na_count = 0;
    }

    while (Serial2.available() >= 10)
    {
      Serial2.readBytes(buffer, 10);

      DshotFeedback feedback{buffer[0],
                             buffer[1] * 256 + buffer[2],
                             buffer[3] * 256 + buffer[4],
                             buffer[5] * 256 + buffer[6],
                             buffer[7] * 256 + buffer[8]};

      const auto crc = get_crc8(buffer, 9);
      if (buffer[9] != crc)
      {
        Serial.println("DSHOT wrong crc, skipping");
        feedback = {0, 0, 0, 0, 0};
      }
      else if (buffer[0] == 0)
      {
        Serial.println("DSHOT received 0, skipping");
        continue;
      }

      motorMutex.lock();
      lastMotorFeedback.temp_C = feedback.temp_C;
      lastMotorFeedback.voltage_mV = feedback.voltage_mV * 10;
      lastMotorFeedback.current_mA = feedback.current_mA / 100;
      lastMotorFeedback.consumption_mAh = feedback.consumption_mAh / 1000;
      lastMotorFeedback.rpm = feedback.rpm;
      motorMutex.unlock();

      // Serial.println();
      // Serial.printf("temp: %d\n", feedback.temp_C);
      // Serial.printf("voltage: %d\n", feedback.voltage_mV);
      // Serial.printf("current: %d\n", feedback.current_mA);
      // Serial.printf("Consumption: %d\n", feedback.consumption_mAh);
      // Serial.printf("RPM: %d\n", feedback.rpm);
      // // Serial.printf("CRC: %d\n", buffer[9]);
      // // Serial.printf("expected CRC: %d\n", crc);
      // Serial.println();
    }

    if (na_count > 2000)
    {
      Serial.println("No DSHOT data received");
      na_count = 0;
    }

    delay(2);
#else
    static int64_t consumption_mAus = 0; //  Millampere microseconds
    static int64_t lastTime_us = -1;

    motorServo.writeMicroseconds(1000 + (power - DSHOT_THROTTLE_MIN) / 2);
    steeringServo.write(steering);

    const auto current = currentSensor_A.read();
    const auto voltage = batteryVoltageSensor_V.read();

    const int32_t current_mA = std::round(current.value * 1000.0f);
    const int32_t voltage_mV = std::round(voltage.value * 1000.0f);

    const int64_t us = esp_timer_get_time();
    if (lastTime_us > 0)
    {
      int64_t delta_us = us - lastTime_us;

      consumption_mAus += current_mA * delta_us;
    }

    lastTime_us = us;

    motorMutex.lock();
    lastMotorFeedback.temp_C = 0;
    lastMotorFeedback.voltage_mV = voltage_mV;
    lastMotorFeedback.current_mA = current_mA;
    lastMotorFeedback.consumption_mAh = consumption_mAus / (3600LL * 1'000'000LL);
    lastMotorFeedback.rpm = 0;
    lastMotorFeedback.voltage_adc = voltage.raw;
    lastMotorFeedback.current_adc = current.raw;
    motorMutex.unlock();

    delay(10);
#endif
  }
}

void setPower(int power, std::optional<int> steering)
{
  motorMutex.lock();
  motorSetpointValue.power = power;
  if (steering)
  {
    motorSetpointValue.servoSteeringAngle = steering.value();
  }
  motorSetpointValue.timestamp = millis();
  motorMutex.unlock();
}

std::thread dshotThread;

void printDateTime(const RtcDateTime &dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial.print(datestring);
}

void setup()
{
  // Serial.begin(115200);

  // Serial.print("compiled: ");
  // Serial.print(__DATE__);
  // Serial.println(__TIME__);

  // RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  // printDateTime(compiled);
  // Serial.println();

  // Rtc.Begin();

  // if (Rtc.GetIsWriteProtected())
  // {
  //   Serial.println("RTC was write protected, enabling writing now");
  //   Rtc.SetIsWriteProtected(false);
  // }

  // if (!Rtc.GetIsRunning())
  // {
  //   Serial.println("RTC was not actively running, starting now");
  //   Rtc.SetIsRunning(true);
  // }

  // RtcDateTime now = Rtc.GetDateTime();
  // if (now < compiled)
  // {
  //   Serial.println("RTC is older than compile time!  (Updating DateTime)");
  //   Rtc.SetDateTime(compiled);
  // }

  // while (true)
  // {
  //   RtcDateTime now = Rtc.GetDateTime();

  //   printDateTime(now);
  //   Serial.println();

  //   delay(1000

  //   );
  // }

  Joystick::startSampling(1);

  pinMode(PIN_BLUE_LED, OUTPUT);

  steeringServo.attach(RUDDER_SERVO_PIN, 750, 2250);

#if !USE_DSHOT
  motorServo.attach(MOTOR_SERVO_PIN, 1000, 2000);
#endif

#if USE_DSHOT
  Serial2.begin(115200, SERIAL_8N1, RX2, TX2);
  anESC.begin(DSHOT_MODE, NO_BIDIRECTION, 14);
#endif

  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1)
      ;
  }

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);

  Serial.println("Starting dshot loop");

  dshotThread = std::thread{dshotLoop};

  Serial.println("Started BLE.");

  Serial.println("Canning for service ...");
  BLE.scanForUuid(BLE_UUID);

  Serial.println("setup() done.");
}

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
  digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));

  BLEDevice peripheral = BLE.available();
  // BLEDevice::setPower(ESP_PWR_LVL_P9);

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
      // digitalWrite(digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));, 1);
      digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));

      const auto optInput = readControl(controlCharacteristic);
      if (!optInput)
      {
        break;
      }

      const auto input = *optInput;

      setPower(input.powerPercent * 20, input.steeringAngleDeg);
#if USE_DSHOT
      steeringServo.write(input.steeringAngleDeg);
#endif

      motorMutex.lock();
      const DshotFeedback feedback{
          lastMotorFeedback.temp_C,
          lastMotorFeedback.voltage_mV,
          lastMotorFeedback.current_mA,
          lastMotorFeedback.consumption_mAh,
          lastMotorFeedback.rpm,
          lastMotorFeedback.voltage_adc,
          lastMotorFeedback.current_adc,
      };
      motorMutex.unlock();

      Serial.printf("RPM: %5d, Current A %2.1f,  voltage mV = %2.4f, power: %4d, steering: %4d,\n",
                    feedback.rpm, feedback.current_mA / 1000.0f, feedback.voltage_mV / 1000.0f, input.powerPercent, input.steeringAngleDeg);

      PowerStatus status{
          .motorCurrent_mA = feedback.current_mA,
          .usedEnergy_mAh = feedback.consumption_mAh,
          .batteryVoltage_mV = feedback.voltage_mV,
          .batteryVoltage_adc = feedback.voltage_adc,
          .motorCurrent_adc = feedback.current_adc,
      };
      sendStatus(statusCharacteristic, status);

      delay(2);
    }

    setPower(0, std::nullopt);
    digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));

    Serial.println("ble remote disconnected, stopping.");

    Serial.println("Canning for service ...");
    BLE.scanForUuid(BLE_UUID);
  }
  else
  {
    Serial.println("ble service not available");
  }

  setPower(0, std::nullopt);
  delay(500);
}
