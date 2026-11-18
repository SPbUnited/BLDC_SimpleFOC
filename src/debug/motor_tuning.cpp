#include "motor_tuning.h"

#include <cmath>

#include "motor_limits.h"

extern "C"
{
    volatile uint8_t mcu_tune_enable = 0;
    volatile float mcu_tune_target_rad_s = 0.0f;
    volatile float mcu_tune_current_limit_a = motor_limits::default_current_a;
    volatile float mcu_tune_velocity_limit_rad_s = motor_limits::velocity_rad_s;
    volatile float mcu_tune_voltage_limit_v = motor_limits::voltage_v;
    volatile float mcu_tune_velocity_p = 0.1f;
    volatile float mcu_tune_velocity_i = 5.0f;
    volatile float mcu_tune_velocity_d = 0.0f;
    volatile float mcu_tune_velocity_filter_tf_s = 0.004f;

    volatile float mcu_tune_shaft_velocity_rad_s = 0.0f;
    volatile float mcu_tune_current_command_q_a = 0.0f;
    volatile float mcu_tune_voltage_bemf_v = 0.0f;
    volatile float mcu_tune_voltage_q_v = 0.0f;
}

namespace
{

bool manual_control_selected = false;

float bounded_setting(float value, float current, float minimum, float maximum)
{
    if (!std::isfinite(value))
    {
        return current;
    }
    return constrain(value, minimum, maximum);
}

}  // namespace

namespace motor_tuning
{

bool enabled()
{
    return mcu_tune_enable != 0;
}

bool owns_control()
{
    return enabled() || manual_control_selected;
}

void apply(BLDCMotor& motor)
{
    if (!enabled())
    {
        if (manual_control_selected)
        {
            motor.target = 0.0f;
            mcu_tune_target_rad_s = 0.0f;
        }
        return;
    }

    manual_control_selected = true;
    motor.updateTorqueControlType(TorqueControlType::estimated_current);
    motor.controller = MotionControlType::velocity;

    const float current_limit = bounded_setting(mcu_tune_current_limit_a, motor.current_limit,
                                                0.0f, motor_limits::maximum_current_a);
    const float velocity_limit = bounded_setting(
        mcu_tune_velocity_limit_rad_s, motor.velocity_limit, 0.0f, motor_limits::velocity_rad_s);
    const float voltage_limit = bounded_setting(mcu_tune_voltage_limit_v, motor.voltage_limit,
                                                0.0f, motor_limits::voltage_v);
    const float velocity_p =
        bounded_setting(mcu_tune_velocity_p, motor.PID_velocity.P, 0.0f, 2.0f);
    const float velocity_i =
        bounded_setting(mcu_tune_velocity_i, motor.PID_velocity.I, 0.0f, 100.0f);
    const float velocity_d =
        bounded_setting(mcu_tune_velocity_d, motor.PID_velocity.D, 0.0f, 0.1f);
    const float velocity_filter_tf =
        bounded_setting(mcu_tune_velocity_filter_tf_s, motor.LPF_velocity.Tf, 0.0f, 0.1f);
    const float target =
        bounded_setting(mcu_tune_target_rad_s, motor.target, -velocity_limit, velocity_limit);

    mcu_tune_current_limit_a = current_limit;
    mcu_tune_velocity_limit_rad_s = velocity_limit;
    mcu_tune_voltage_limit_v = voltage_limit;
    mcu_tune_velocity_p = velocity_p;
    mcu_tune_velocity_i = velocity_i;
    mcu_tune_velocity_d = velocity_d;
    mcu_tune_velocity_filter_tf_s = velocity_filter_tf;
    mcu_tune_target_rad_s = target;

    motor.updateCurrentLimit(current_limit);
    motor.updateVelocityLimit(velocity_limit);
    motor.updateVoltageLimit(voltage_limit);
    motor.PID_velocity.P = velocity_p;
    motor.PID_velocity.I = velocity_i;
    motor.PID_velocity.D = velocity_d;
    motor.LPF_velocity.Tf = velocity_filter_tf;
    motor.PID_velocity.limit = current_limit;
    motor.target = target;
}

void capture(const BLDCMotor& motor)
{
    mcu_tune_shaft_velocity_rad_s = motor.shaft_velocity;
    mcu_tune_current_command_q_a = motor.current.q;
    mcu_tune_voltage_bemf_v = motor.voltage_bemf;
    mcu_tune_voltage_q_v = motor.voltage.q;
}

}  // namespace motor_tuning
