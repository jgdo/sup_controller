#include <sstream>
#include <optional>
#include <mutex>

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <ESP32Servo.h>
#include <esp_bt.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>


#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047

#include "adc_sampling.h"
#include "bt_settings.h"

constexpr auto PIN_BLUE_LED = 8;


static constexpr int CURRENT_SENSING_ADC_PIN = 0;
static constexpr int VOLTAGE_SENSING_ADC_PIN = 1;
static constexpr int RUDDER_SERVO_PIN = 10;
static constexpr int MOTOR_SERVO_PIN = 21;

constexpr auto PIN_TO_ESC = 21;
constexpr auto PIN_FROM_ESC = 20;

// Easy to modify LED blink configuration.
static constexpr float bluetoothWaitingLedFrequencyHz = 1.0f;
static constexpr float bluetoothConnectedLedFrequencyHz = 5.0f;

// Timing configuration.
static constexpr unsigned long statusSendIntervalMs = 100;
static constexpr unsigned long bluetoothCommandTimeoutMs = 300;

static constexpr long POWER_RECEIVE_TIMEOUT_MS = 500;
static constexpr float CONTROL_FREQ = 50.0f;

static constexpr float V_PER_STEP = 0.006545;

// Low-latency BLE connection parameters.
// Units for interval are 1.25 ms, so 6 = 7.5 ms and 12 = 15 ms.
// Timeout unit is 10 ms, so 200 = 2 seconds.
static constexpr uint16_t bleMinConnectionInterval = 6;
static constexpr uint16_t bleMaxConnectionInterval = 12;
static constexpr uint16_t bleSlaveLatency = 0;
static constexpr uint16_t bleSupervisionTimeout = 200;

ThreeWire myWire(6, 5, 7); // IO, SCLK, CE
RtcDS1302<ThreeWire> Rtc(myWire);

Servo steeringServo;

Servo motorServo;

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
    uint16_t voltage_mV;
    uint32_t current_mA;
    uint32_t consumption_mAh;
    uint32_t rpm;
    uint16_t voltage_adc;
    uint16_t current_adc;
};

std::mutex motorMutex;
volatile MotorValue motorSetpointValue = {0, 90, 0};
volatile DshotFeedback lastMotorFeedback;
std::atomic<uint8_t> maxMotorPowerPercentage = 100;

// ---- Multi-connection configuration ----------------------------------------
// NimBLE must be built with enough connection slots. We also tell the stack
// to keep advertising after a client connects so additional centrals can join.
static constexpr uint8_t MAX_CONNECTIONS = 4;

static NimBLEServer* bleServer = nullptr;
static NimBLECharacteristic* commandCharacteristic = nullptr;
static NimBLECharacteristic* statusCharacteristic = nullptr;
static NimBLECharacteristic* moxPowerCharacteristic = nullptr;

static unsigned long lastStatusSendMs = 0;
static unsigned long lastLedToggleMs = 0;

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
        crc_u = (crc_u & 0x80) ? 0x7 ^ (crc_u << 1) : (crc_u << 1);
    return crc_u;
}

uint8_t get_crc8(uint8_t* Buf, uint8_t BufLen)
{
    uint8_t crc = 0, i;
    for (i = 0; i < BufLen; i++)
        crc = update_crc8(Buf[i], crc);
    return crc;
}

std::tuple<int, int> getDshotMotorValueAndSteering()
{
    motorMutex.lock();
    const MotorValue lastVal = {
        motorSetpointValue.power, motorSetpointValue.servoSteeringAngle, motorSetpointValue.timestamp
    };
    motorMutex.unlock();

    const auto ms = millis();

    if ((lastVal.timestamp > ms) || (ms - lastVal.timestamp > DSHOT_LOOP_TIMEOUT_MS) || lastVal.power < 0 || lastVal.
        power > 2000)
    {
        Serial.println("Invalid power or timeout from main thread!");
        return {DSHOT_THROTTLE_MIN, lastVal.servoSteeringAngle};
    }

    return {std::min<int>(DSHOT_THROTTLE_MIN + lastVal.power, DSHOT_THROTTLE_MAX), lastVal.servoSteeringAngle};
}

void dshotLoop()
{
    while (true)
    {
        const auto [power, steering] = getDshotMotorValueAndSteering();

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
    }
}

void setMotorControls(int power, std::optional<int> steering)
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

void printDateTime(const RtcDateTime& dt)
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

class ServerCallbacks final : public NimBLEServerCallbacks
{
public:
    void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override
    {
        server->updateConnParams(
            connInfo.getConnHandle(),
            bleMinConnectionInterval,
            bleMaxConnectionInterval,
            bleSlaveLatency,
            bleSupervisionTimeout);

        Serial.print("Remote connected: ");
        Serial.println(connInfo.getAddress().toString().c_str());

        if (server->getConnectedCount() < MAX_CONNECTIONS) {
            NimBLEDevice::startAdvertising();
        }
    }

    void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override
    {
        if (!server->getConnectedCount())
        {
            setMotorControls(0, std::nullopt);
        }

        Serial.print("Remote disconnected: ");
        Serial.print(connInfo.getAddress().toString().c_str());
        Serial.print(", reason: ");
        Serial.println(reason);

        /*
         * Keep advertising after disconnects.
         * This allows reconnects and leaves the motor available for another
         * central if the ESP32/NimBLE configuration supports multiple clients.
         */
        NimBLEDevice::startAdvertising();
    }
} serverCallbacks;

class CommandCharacteristicCallbacks final : public NimBLECharacteristicCallbacks
{
public:
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override
    {
        const auto value = characteristic->getValue();

        if (value.size() != sizeof(Control))
        {
            Serial.print("Invalid remote command packet from ");
            Serial.println(connInfo.getAddress().toString().c_str());
            return;
        }

        Control control;
        memcpy(&control, value.data(), sizeof(Control));

        const float powerFactor = std::min<int>(maxMotorPowerPercentage.load(), 100) / 100.0F;

        const int powerNormalized = std::round(
                sqrt(control.powerPercent / 100.0F) * powerFactor * (DSHOT_THROTTLE_MAX - DSHOT_THROTTLE_MIN));

        setMotorControls(powerNormalized, control.steeringAngleDeg);
    }
} chrCallbacks;

class MaxPowerCharacteristicCallbacks final : public NimBLECharacteristicCallbacks
{
public:
    void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override
    {
        const auto value = characteristic->getValue<uint8_t>();
        maxMotorPowerPercentage.store(value);
    }
} maxPowerCallbacks;

void setupBle()
{
    NimBLEDevice::init(BT_NAME_MOTOR);
    NimBLEDevice::setMTU(247);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    bleServer = NimBLEDevice::createServer();
    bleServer->setCallbacks(&serverCallbacks, false);
    bleServer->advertiseOnDisconnect(true);

    NimBLEService* service = bleServer->createService(BLE_SERVICE_UUID);

    commandCharacteristic = service->createCharacteristic(
        BLE_CONTROL_CHARACTERISTICS,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    commandCharacteristic->setCallbacks(&chrCallbacks);

    moxPowerCharacteristic = service->createCharacteristic(
        BLE_MOTOR_POWER_CHARACTERISTICS,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY,
        1);
    moxPowerCharacteristic->setValue<uint8_t>(maxMotorPowerPercentage.load());
    moxPowerCharacteristic->setCallbacks(&maxPowerCallbacks);

    statusCharacteristic = service->createCharacteristic(
        BLE_POWER_STATUS_CHARACTERISTICS,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

    PowerStatus initialStatus = {
        .motorCurrent_mA = 0,
        .usedEnergy_mAh = 0,
        .batteryVoltage_mV = 0,
        .batteryVoltage_adc = 0,
        .motorCurrent_adc = 0,
    };
    statusCharacteristic->setValue(
        reinterpret_cast<const uint8_t*>(&initialStatus),
        sizeof(initialStatus));



    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setName(BT_NAME_MOTOR);
    advertising->enableScanResponse(true);
    advertising->start();

    Serial.println("BLE motor service advertising");
}

static void updateBluetoothStatusLed()
{
    const unsigned long nowMs = millis();
    const float blinkFrequencyHz = bleServer->getConnectedCount()
        ? bluetoothConnectedLedFrequencyHz
        : bluetoothWaitingLedFrequencyHz;

    const unsigned long toggleIntervalMs =
        static_cast<unsigned long>(1000.0f / (blinkFrequencyHz * 2.0f));

    if (nowMs - lastLedToggleMs < toggleIntervalMs)
    {
        return;
    }

    lastLedToggleMs = nowMs;
    digitalWrite(PIN_BLUE_LED, !digitalRead(PIN_BLUE_LED));
}

void sendStatusIfNeeded()
{
    if (!bleServer->getConnectedCount() || statusCharacteristic == nullptr)
    {
        return;
    }

    const unsigned long nowMs = millis();

    if (nowMs - lastStatusSendMs < statusSendIntervalMs)
    {
        return;
    }

    lastStatusSendMs = nowMs;

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
                  feedback.rpm, feedback.current_mA / 1000.0f, feedback.voltage_mV / 1000.0f);

    const PowerStatus status{
        .motorCurrent_mA = feedback.current_mA,
        .usedEnergy_mAh = feedback.consumption_mAh,
        .batteryVoltage_mV = feedback.voltage_mV,
        .batteryVoltage_adc = feedback.voltage_adc,
        .motorCurrent_adc = feedback.current_adc,
    };

    statusCharacteristic->setValue(
        reinterpret_cast<const uint8_t *>(&status),
        sizeof(status));

    statusCharacteristic->notify();
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
    motorServo.attach(MOTOR_SERVO_PIN, 1000, 2000);

    Serial.println("Starting dshot loop");

    dshotThread = std::thread{dshotLoop};

    setupBle();

    Serial.println("setup() done.");
}

void loop()
{
    updateBluetoothStatusLed();
    sendStatusIfNeeded();

    delay(5);
}
