#include <Arduino.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>

#include "bt_settings.h"
#include <sstream>

static constexpr int CURRENT_SENSING_ADC_PIN = 39;
static constexpr int RUDDER_SERVO_PIN = 14;
static constexpr int MOTOR_SERVO_PIN = 27;

static constexpr long POWER_RECEIVE_TIMEOUT_MS = 500;

Servo steeringServo, powerServo;
BluetoothSerial SerialBT;

void Bt_Status(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{

  if (event == ESP_SPP_SRV_OPEN_EVT)
  {
    Serial.println("Client Connected");
  }

  else if (event == ESP_SPP_CLOSE_EVT)
  {
    Serial.println("Client Disconnected");
  }

  else
  {
    Serial.println("Other BT event");
  }
}

void setup()
{
  Serial.begin(115200);
  SerialBT.register_callback(Bt_Status);
  SerialBT.begin(BT_NAME_MOTOR);

  steeringServo.attach(RUDDER_SERVO_PIN, 1000, 2000);
  powerServo.attach(MOTOR_SERVO_PIN, 1000, 2000);

  Serial.println("setup() done.");
}

std::array<int, 50> valuesArray;
int valueCounter = 0;

int steeringAngle = 90;
int powerAngle = 0;
long powerLastReceived = -1;

void processBtCommand(const std::string &cmd)
{
  // std::stringstream input{cmd};
  std::string segment;

  if (strncmp(cmd.c_str(), "s ", 2) == 0)
  {
    sscanf(cmd.c_str(), "s %d", &steeringAngle);
    Serial.printf("Setting steering angle from bluetooth: %d\n", steeringAngle);
  }
  else if (strncmp(cmd.c_str(), "p ", 2) == 0)
  {
    sscanf(cmd.c_str(), "p %d", &powerAngle);
    powerLastReceived = millis();
    Serial.printf("Setting power angle from bluetooth: %d\n", powerAngle);
  }

  // if (!std::getline(input, segment, ' '))
  // {
  //   return;
  // }

  // if (segment == "s")
  // {
  //   if (!std::getline(input, segment, ' '))
  //   {
  //     return;
  //   }

  //   steeringAngle = std::stoi(segment);
  //   Serial.printf("Setting steering angle from bluetooth: %d\n", steeringAngle);
  // }
}

void processBtCommands()
{
  static constexpr auto MAX_BT_BUF_LEN = 100;
  static std::string buffer;

  while (SerialBT.available())
  {
    const auto chr = SerialBT.read();
    if (chr < 0)
    {
      return; // should not happen
    }

    if (chr == '\n')
    {
      processBtCommand(buffer);
      buffer.clear();
    }
    else
    {
      buffer.push_back(chr);
    }
  }
}

void loop()
{
  processBtCommands();

  if(millis() > powerLastReceived + POWER_RECEIVE_TIMEOUT_MS) {
    powerAngle = 0;
  }

  steeringServo.write(steeringAngle);
  powerServo.write(powerAngle);

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

  Serial.printf("Current adc %4d, power: %4d, steering: %4d\n", sum, powerAngle, steeringAngle);
  delay(20);
}
