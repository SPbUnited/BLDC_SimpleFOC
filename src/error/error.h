#pragma once

namespace error
{

enum ERRORS
{
    NONE = 0,
    PCB_TEMP = 0b1,
    MOTOR_TEMP = 0b10,
    MOTOR_INIT_FAILED = 0b100,
    MOTOR_CALIB_FAILED = 0b1000,
    MOTOR_ERROR = 0b10000,
};

}  // namespace error
