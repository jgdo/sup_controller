#pragma once

static constexpr auto BT_NAME_CONTOLLER = "sup_board_controller";
static constexpr auto BT_NAME_MOTOR = "sup_board_motor";

static_assert(sizeof(unsigned int) == 4);
union ControlUnion
{
    struct {
        uint8_t powerPercent;
        uint8_t steeringAngleDeg;
        uint8_t pading[2];
    } values;

    unsigned int bleValue;
};
static_assert(sizeof(ControlUnion) == 4);
