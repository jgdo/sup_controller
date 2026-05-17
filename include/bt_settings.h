#pragma once

static constexpr auto BT_NAME_REMOTE = "sup_remote";
static constexpr auto BT_NAME_MOTOR = "sup_board_motor";

static_assert(sizeof(unsigned int) == 4);

// Remote to motor
struct Control
{
    uint8_t powerPercent;
    uint8_t steeringAngleDeg;
    uint8_t pading[2];
};
static_assert(sizeof(Control) == 4);

union ControlUnion
{
    Control values;
    uint32_t bleValue;
};
static_assert(sizeof(ControlUnion) == 4);

struct PowerStatus {
    uint32_t motorCurrent_mA;
    uint32_t usedEnergy_mAh;
    uint16_t batteryVoltage_mV;
    uint16_t batteryVoltage_adc;
    uint16_t motorCurrent_adc;
    uint8_t pading[2];
};
static_assert(sizeof(PowerStatus) == 16);

static constexpr auto BLE_SERVICE_UUID = "9f7f1000-2c6d-4f42-8d18-19f0f8b00001";

static constexpr auto BLE_CONTROL_CHARACTERISTICS = "9f7f1001-2c6d-4f42-8d18-19f0f8b00001";
static constexpr auto BLE_POWER_STATUS_CHARACTERISTICS = "9f7f1002-2c6d-4f42-8d18-19f0f8b00001";
static constexpr auto BLE_MOTOR_POWER_CHARACTERISTICS = "9f7f1003-2c6d-4f42-8d18-19f0f8b00001";
