#include <TFT_eSPI.h>

TFT_eSPI tft{};

void setup()
{
  Serial.begin(115200);
  Serial.println("setup() start");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_GREEN);
  tft.setTextColor(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  tft.drawString("--- FooBar ---", tft.width() / 2, tft.height() / 2 - 16);

  Serial.println("setup() end");
}

void loop()
{
  // put your main code here, to run repeatedly:
}
