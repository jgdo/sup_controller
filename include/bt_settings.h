#pragma once

static constexpr auto BT_NAME_CONTOLLER = "sup_board_controller";
static constexpr auto BT_NAME_MOTOR = "sup_board_motor";

static_assert(sizeof(unsigned int) == 4);

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
    unsigned int bleValue;
};
static_assert(sizeof(ControlUnion) == 4);

static constexpr auto BLE_UUID = "1101";

static constexpr auto BLE_CONTROL_CHARACTERISTICS = "2101";
