#pragma once

#include <SimpleFOC.h>

namespace motor_tuning
{

/** Apply bounded MCUViewer settings when tuning mode is enabled. */
void apply(BLDCMotor& motor);

/** Copy controller state to debugger-visible telemetry symbols. */
void capture(const BLDCMotor& motor);

/** Return whether MCUViewer owns the motor target. */
bool enabled();

/** Return whether MCUViewer has selected the manual control path this boot. */
bool owns_control();

}  // namespace motor_tuning

extern "C"
{
    extern volatile uint8_t mcu_tune_enable;
    extern volatile float mcu_tune_target_rad_s;
    extern volatile float mcu_tune_current_limit_a;
    extern volatile float mcu_tune_velocity_limit_rad_s;
    extern volatile float mcu_tune_voltage_limit_v;
    extern volatile float mcu_tune_velocity_p;
    extern volatile float mcu_tune_velocity_i;
    extern volatile float mcu_tune_velocity_d;
    extern volatile float mcu_tune_velocity_filter_tf_s;

    extern volatile float mcu_tune_shaft_velocity_rad_s;
    extern volatile float mcu_tune_current_command_q_a;
    extern volatile float mcu_tune_voltage_bemf_v;
    extern volatile float mcu_tune_voltage_q_v;
}  // extern "C"
