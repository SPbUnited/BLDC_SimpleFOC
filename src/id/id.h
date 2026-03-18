#pragma once

#include <SimpleFOC.h>

namespace id
{
void init()
{
    pinMode(PC14, INPUT_PULLUP);
    pinMode(PC15, INPUT_PULLUP);
    pinMode(PB3, INPUT_PULLUP);
}

uint8_t get_id()
{
    uint8_t A0 = (~digitalRead(PC14) & 0x01) << 1;
    uint8_t A1 = (~digitalRead(PC15) & 0x01) << 0;
    uint8_t A2 = (~digitalRead(PB3) & 0x01) << 2;
    uint8_t motor_id = A0 + A1 + A2;

    // uint8_t lol_id = motor_id & 0xFF;
    return motor_id;
}
};  // namespace id