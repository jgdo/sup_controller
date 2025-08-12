#include <Arduino.h>

#include <DShotRMT.h>

constexpr auto PIN_TO_ESC = 17;
constexpr auto PIN_FROM_ESC = 16;
constexpr auto DSHOT_MODE = DSHOT600;

DShotRMT anESC(PIN_TO_ESC);

void setup()
{
    Serial.begin(115200);

    Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

    anESC.begin(DSHOT_MODE, NO_BIDIRECTION, 14);

    // pinMode(TO_ESC, OUTPUT);
    // pinMode(FROM_ESC, INPUT_PULLUP);

    Serial.println("Start");

    // for(int i = 0; i < 10; i++) {
    //     anESC.send_dshot_value(DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE);
    //     delay(1);
    // }

    // anESC.send_dshot_value(DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY, ENABLE_TELEMETRIC);
}

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

int na_count = 0;

void loop()
{
    anESC.send_dshot_value(DSHOT_THROTTLE_MIN, ENABLE_TELEMETRIC);

    // anESC.send_dshot_value(DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY, ENABLE_TELEMETRIC);

    // uint32_t rpm_1 = 0;
    // extended_telem_type_t telem = TELEM_TYPE_TEMPRATURE; // telemetry argument is optional
    // int error_a = anESC.get_dshot_packet(&rpm_1, &telem);

    // Serial.printf("%10d, %10d, %10.3f\n",
    //               error_a, rpm_1, anESC.get_telem_success_rate());

    // Serial.printf("available: %d\n", Serial2.available());

    uint8_t buffer[10] = {42};

    if(Serial2.available() == 0) {
        na_count++;
    } else {
        na_count = 0;
    }

    while (Serial2.available() >= 10)
    {
        Serial2.readBytes(buffer, 10);
        const auto crc = get_crc8(buffer, 9);
        if (buffer[9] != crc)
        {
            Serial.println("Wrong crc, skipping");
            continue;
        }
        else if (buffer[0] == 0)
        {
            Serial.println("Received 0, skipping");
            continue;
        }

        Serial.println();
        Serial.printf("temp: %d\n", buffer[0]);
        Serial.printf("voltage: %d\n", buffer[1] * 256 + buffer[2]);
        Serial.printf("current: %d\n", buffer[3] * 256 + buffer[4]);
        Serial.printf("Consumption: %d\n", buffer[5] * 256 + buffer[6]);
        Serial.printf("RPM: %d\n", buffer[7] * 256 + buffer[8]);
        // Serial.printf("CRC: %d\n", buffer[9]);
        // Serial.printf("expected CRC: %d\n", crc);
        Serial.println();
    }

    if(na_count > 1000) {
        Serial.println("No data received");
        na_count = 0;
    }

    delay(1);
}
