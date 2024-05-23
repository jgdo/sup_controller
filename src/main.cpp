#include <Arduino.h>
#include <ESP32Servo.h>

static constexpr int STEERING_ADC_PIN = 34;
static constexpr int STEERING_SERVO_PIN = 14;

Servo steeringServo;

void setup()
{
  Serial.begin(115200);
  steeringServo.attach(STEERING_SERVO_PIN);

  Serial.println("setup()");
}

void loop()
{
  const auto milliV = analogReadMilliVolts(STEERING_ADC_PIN);
  const auto adcRaw = analogRead(STEERING_ADC_PIN);

  const int angle = (adcRaw * 180 + 2048) / 4095;
  steeringServo.write(angle);

  Serial.println("-----------");
  Serial.printf("ADC raw: %4d\n", adcRaw);
  Serial.printf("mV: %.4f\n", milliV/1000.0F);
  Serial.printf("angle: %d\n", angle);

  delay(20);
}
