#pragma once

/**
 * Default and absolute model-based limits for the installed GL35 KV100.
 * Voltage limit is a phase voltage-vector bound, not a PWM duty setting.
 * Velocity bound retains voltage headroom at the default 1.3 A setting using
 * the 3.6 ohm phase-to-phase value and SimpleFOC's KV-based BEMF estimate.
 * Raising current toward 4 A can saturate the 12 V vector limit even at low speed.
 */
namespace motor_limits
{
constexpr float default_current_a = 1.3f;
constexpr float maximum_current_a = 4.0f;
constexpr float voltage_v = 12.0f;
constexpr float velocity_rad_s = 70.0f;
constexpr float warning_current_a = 1.0f;
}  // namespace motor_limits
