#pragma once

/**
 * CAN Fuoco - CAN bus communication protocol for BLDC motors
 * on omnidirectional SSL robot.
 *
 * example usage:
 *
 * CANFuocoMotorConfig motor_config = {
 *     .motor_id = 1,
 *     .get_supply_voltage = get_supply_voltage,
 *     .motor = motor,
 * };
 * CANFuoco can_fuoco(motor_config);
 *
 * void loop() {
 *     if (can.available()) {
 *         frame = can.read();
 *         can_fuoco.can_rx_callback(frame.id & 0xFF, frame.len, frame.buf);
 *         if (can_fuoco.can_tx_callback(frame.buf)) {
 *             can.write(frame);
 *         }
 *     }
 *     motor.loopFOC();
 * }
 */

#include <SimpleFOC.h>
#include <map>
#include "half_float.h"

enum CANFuocoRegisterMap
{
    EMERGENCY_STOP_W = 0,
    SUPPLY_VOLTAGE_R = 1,
    MONITOR_SETTINGS_RW = 4,

    // State variables
    TARGET_RW = 10,
    FEED_FORWARD_VELOCITY_R = 11,
    SHAFT_ANGLE_R = 12,
    ELECTRICAL_ANGLE_R = 13,
    SHAFT_VELOCITY_R = 14,
    CURRENT_SP_R = 15,
    SHAFT_VELOCITY_SP_R = 16,
    SHAFT_ANGLE_SP_R = 17,
    VOLTAGE_D_R = 18,
    VOLTAGE_Q_R = 19,
    CURRENT_D_R = 20,
    CURRENT_Q_R = 21,
    VOLTAGE_BEMF_R = 22,
    U_ALPHA_R = 23,
    U_BETA_R = 24,

    // Motor configuration parameters
    VOLTAGE_SENSOR_ALIGN_RW = 25,
    VELOCITY_INDEX_SEARCH_RW = 26,

    // Motor physical parameters
    PHASE_RESISTANCE_RW = 27,
    POLE_PAIRS_RW = 28,
    KV_RATING_RW = 29,
    PHASE_INDUCTANCE_RW = 30,

    // Limiting variables
    VOLTAGE_LIMIT_RW = 31,
    CURRENT_LIMIT_RW = 32,
    VELOCITY_LIMIT_RW = 33,

    // Motor status
    ENABLED_RW = 34,
    MOTOR_STATUS_R = 35,

    // PWM modulation settings
    FOC_MODULATION_RW = 36,
    MODULATION_CENTERED_RW = 37,

    // Control mode
    TORQUE_CONTROLLER_RW = 38,
    CONTROLLER_RW = 39,

    // Controller and low pass filter settings
    PID_CURRENT_D_P_RW = 40,
    PID_CURRENT_D_I_RW = 41,
    PID_CURRENT_D_D_RW = 42,
    LPF_CURRENT_D_RW = 43,
    PID_CURRENT_Q_P_RW = 45,
    PID_CURRENT_Q_I_RW = 46,
    PID_CURRENT_Q_D_RW = 47,
    LPF_CURRENT_Q_RW = 48,
    PID_VELOCITY_P_RW = 50,
    PID_VELOCITY_I_RW = 51,
    PID_VELOCITY_D_RW = 52,
    LPF_VELOCITY_RW = 53,
    P_ANGLE_P_RW = 55,
    LPF_ANGLE_RW = 56,
    MOTION_DOWNSAMPLE_RW = 58,
    MOTION_CNT_RW = 59,

    // Sensor settings
    SENSOR_OFFSET_RW = 60,
    ZERO_ELECTRIC_ANGLE_RW = 61,
    SENSOR_DIRECTION_RW = 62,
    PP_CHECK_RESULT_RW = 63,

    // Virtual registers
    MULTI_TARGET_W = 64,
    SHAFT_FEEDBACK_R = 96,
    SAVE_TO_EEPROM_W = 127,
};

struct CANFuocoMotorConfig
{
    int motor_id;
    float get_supply_voltage(void);
    BLDCMotor &motor;
    float speed = 0;
};

struct CANFuocoMonitorSettings
{
    CANFuocoRegisterMap register_to_monitor;
};

class CANFuoco : public CANFuocoMotorConfig
{
private:
    CANFuocoMonitorSettings monitor_settings;

    std::map<CANFuocoRegisterMap, uint8_t *> CANFuocoRegisterMapNames;

public:
    CANFuoco(CANFuocoMotorConfig &motor_config);

    void can_rx_callback(CANFuocoRegisterMap id, size_t len, uint8_t *buf);
    bool can_tx_callback(uint8_t *buf);
};

CANFuoco::CANFuoco(
    CANFuocoMotorConfig &motor_config) : CANFuocoMotorConfig(motor_config)
{
    CANFuocoRegisterMapNames = std::map<CANFuocoRegisterMap, uint8_t *>{
        {TARGET_RW, reinterpret_cast<uint8_t *>(&motor.target)},
        {FEED_FORWARD_VELOCITY_R, reinterpret_cast<uint8_t *>(&motor.feed_forward_velocity)},
        {SHAFT_ANGLE_R, reinterpret_cast<uint8_t *>(&motor.shaft_angle)},
        {ELECTRICAL_ANGLE_R, reinterpret_cast<uint8_t *>(&motor.electrical_angle)},
        {SHAFT_VELOCITY_R, reinterpret_cast<uint8_t *>(&motor.shaft_velocity)},
        {CURRENT_SP_R, reinterpret_cast<uint8_t *>(&motor.current.q)},
        {SHAFT_VELOCITY_SP_R, reinterpret_cast<uint8_t *>(&motor.shaft_velocity_sp)},
        {SHAFT_ANGLE_SP_R, reinterpret_cast<uint8_t *>(&motor.shaft_angle_sp)},
        {VOLTAGE_D_R, reinterpret_cast<uint8_t *>(&motor.voltage.d)},
        {VOLTAGE_Q_R, reinterpret_cast<uint8_t *>(&motor.voltage.q)},
        {CURRENT_D_R, reinterpret_cast<uint8_t *>(&motor.current.d)},
        {CURRENT_Q_R, reinterpret_cast<uint8_t *>(&motor.current.q)},
        {VOLTAGE_BEMF_R, reinterpret_cast<uint8_t *>(&motor.voltage_bemf)},
        {U_ALPHA_R, reinterpret_cast<uint8_t *>(&motor.Ualpha)},
        {U_BETA_R, reinterpret_cast<uint8_t *>(&motor.Ubeta)},
        {VOLTAGE_SENSOR_ALIGN_RW, reinterpret_cast<uint8_t *>(&motor.voltage_sensor_align)},
        {VELOCITY_INDEX_SEARCH_RW, reinterpret_cast<uint8_t *>(&motor.velocity_index_search)},
        {PHASE_RESISTANCE_RW, reinterpret_cast<uint8_t *>(&motor.phase_resistance)},
        {POLE_PAIRS_RW, reinterpret_cast<uint8_t *>(&motor.pole_pairs)},
        {KV_RATING_RW, reinterpret_cast<uint8_t *>(&motor.KV_rating)},
        {PHASE_INDUCTANCE_RW, reinterpret_cast<uint8_t *>(&motor.phase_inductance)},
        {VOLTAGE_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.voltage_limit)},
        {CURRENT_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.current_limit)},
        {VELOCITY_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.velocity_limit)},
        {ENABLED_RW, reinterpret_cast<uint8_t *>(&motor.enabled)},
        {MOTOR_STATUS_R, reinterpret_cast<uint8_t *>(&motor.motor_status)},
        {FOC_MODULATION_RW, reinterpret_cast<uint8_t *>(&motor.foc_modulation)},
        {MODULATION_CENTERED_RW, reinterpret_cast<uint8_t *>(&motor.modulation_centered)},
        {TORQUE_CONTROLLER_RW, reinterpret_cast<uint8_t *>(&motor.torque_controller)},
        {CONTROLLER_RW, reinterpret_cast<uint8_t *>(&motor.controller)},
        {PID_CURRENT_D_P_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_d.P)},
        {PID_CURRENT_D_I_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_d.I)},
        {PID_CURRENT_D_D_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_d.D)},
        {LPF_CURRENT_D_RW, reinterpret_cast<uint8_t *>(&motor.LPF_current_d.Tf)},
        {PID_CURRENT_Q_P_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.P)},
        {PID_CURRENT_Q_I_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.I)},
        {PID_CURRENT_Q_D_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.D)},
        {LPF_CURRENT_Q_RW, reinterpret_cast<uint8_t *>(&motor.LPF_current_q.Tf)},
        {PID_VELOCITY_P_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.P)},
        {PID_VELOCITY_I_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.I)},
        {PID_VELOCITY_D_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.D)},
        {LPF_VELOCITY_RW, reinterpret_cast<uint8_t *>(&motor.LPF_velocity.Tf)},
        {P_ANGLE_P_RW, reinterpret_cast<uint8_t *>(&motor.P_angle.P)},
        {LPF_ANGLE_RW, reinterpret_cast<uint8_t *>(&motor.LPF_angle.Tf)},
        {MOTION_DOWNSAMPLE_RW, reinterpret_cast<uint8_t *>(&motor.motion_downsample)},
        {MOTION_CNT_RW, reinterpret_cast<uint8_t *>(&motor.motion_cnt)},
        {SENSOR_OFFSET_RW, reinterpret_cast<uint8_t *>(&motor.sensor_offset)},
        {ZERO_ELECTRIC_ANGLE_RW, reinterpret_cast<uint8_t *>(&motor.zero_electric_angle)},
        {SENSOR_DIRECTION_RW, reinterpret_cast<uint8_t *>(&motor.sensor_direction)},
        {PP_CHECK_RESULT_RW, reinterpret_cast<uint8_t *>(&motor.pp_check_result)},
    };
}

void CANFuoco::can_rx_callback(CANFuocoRegisterMap id, size_t len, uint8_t *buf)
{
    switch (id)
    {
    default:
    case EMERGENCY_STOP_W:
        motor.disable();
        delay(15000);
        break;

    case MONITOR_SETTINGS_RW:
        memcpy(&monitor_settings, buf, len);
        break;

    case TARGET_RW:
    case VOLTAGE_SENSOR_ALIGN_RW:
    case VELOCITY_INDEX_SEARCH_RW:
    case PHASE_RESISTANCE_RW:
    case POLE_PAIRS_RW:
    case KV_RATING_RW:
    case PHASE_INDUCTANCE_RW:
    case VOLTAGE_LIMIT_RW:
    case CURRENT_LIMIT_RW:
    case VELOCITY_LIMIT_RW:
    case ENABLED_RW:
    case FOC_MODULATION_RW:
    case MODULATION_CENTERED_RW:
    case TORQUE_CONTROLLER_RW:
    case CONTROLLER_RW:
    case PID_CURRENT_D_P_RW:
    case PID_CURRENT_D_I_RW:
    case PID_CURRENT_D_D_RW:
    case LPF_CURRENT_D_RW:
    case PID_CURRENT_Q_P_RW:
    case PID_CURRENT_Q_I_RW:
    case PID_CURRENT_Q_D_RW:
    case LPF_CURRENT_Q_RW:
    case PID_VELOCITY_P_RW:
    case PID_VELOCITY_I_RW:
    case PID_VELOCITY_D_RW:
    case LPF_VELOCITY_RW:
    case P_ANGLE_P_RW:
    case LPF_ANGLE_RW:
    case MOTION_DOWNSAMPLE_RW:
    case MOTION_CNT_RW:
    case SENSOR_OFFSET_RW:
    case ZERO_ELECTRIC_ANGLE_RW:
    case SENSOR_DIRECTION_RW:
    case PP_CHECK_RESULT_RW:
        memcpy(CANFuocoRegisterMapNames.at(id), buf, len);
        break;

    case MULTI_TARGET_W:
        if (1 <= motor_id && motor_id < 5)
        {
            ushort *target_data = reinterpret_cast<ushort *>(buf);
            speed = half_to_float(target_data[motor_id - 1]);
            motor.target = speed;
            // motor.move(half_to_float(target_data[motor_id - 1]));
        }
        break;

    case SHAFT_FEEDBACK_R:
        break;

    case SAVE_TO_EEPROM_W:
        break;
    }
}

/**
 * Prepare CAN data for feedback transmission
 *
 * @return true if the feedback is OK and prepared
 */
bool CANFuoco::can_tx_callback(uint8_t *buf)
{
    size_t len = 8;

    switch (monitor_settings.register_to_monitor)
    {
    case SUPPLY_VOLTAGE_R:
    {
        float supply_voltage = 13; //get_supply_voltage();
        memcpy(buf, &supply_voltage, len);
        break;
    }

    case TARGET_RW:
    case FEED_FORWARD_VELOCITY_R:
    case SHAFT_ANGLE_R:
    case ELECTRICAL_ANGLE_R:
    case SHAFT_VELOCITY_R:
    case CURRENT_SP_R:
    case SHAFT_VELOCITY_SP_R:
    case SHAFT_ANGLE_SP_R:
    case VOLTAGE_D_R:
    case VOLTAGE_Q_R:
    case CURRENT_D_R:
    case CURRENT_Q_R:
    case VOLTAGE_BEMF_R:
    case U_ALPHA_R:
    case U_BETA_R:
    case VOLTAGE_SENSOR_ALIGN_RW:
    case VELOCITY_INDEX_SEARCH_RW:
    case PHASE_RESISTANCE_RW:
    case POLE_PAIRS_RW:
    case KV_RATING_RW:
    case PHASE_INDUCTANCE_RW:
    case VOLTAGE_LIMIT_RW:
    case CURRENT_LIMIT_RW:
    case VELOCITY_LIMIT_RW:
    case ENABLED_RW:
    case MOTOR_STATUS_R:
    case FOC_MODULATION_RW:
    case MODULATION_CENTERED_RW:
    case TORQUE_CONTROLLER_RW:
    case CONTROLLER_RW:
    case PID_CURRENT_D_P_RW:
    case PID_CURRENT_D_I_RW:
    case PID_CURRENT_D_D_RW:
    case LPF_CURRENT_D_RW:
    case PID_CURRENT_Q_P_RW:
    case PID_CURRENT_Q_I_RW:
    case PID_CURRENT_Q_D_RW:
    case LPF_CURRENT_Q_RW:
    case PID_VELOCITY_P_RW:
    case PID_VELOCITY_I_RW:
    case PID_VELOCITY_D_RW:
    case LPF_VELOCITY_RW:
    case P_ANGLE_P_RW:
    case LPF_ANGLE_RW:
    case MOTION_DOWNSAMPLE_RW:
    case MOTION_CNT_RW:
    case SENSOR_OFFSET_RW:
    case ZERO_ELECTRIC_ANGLE_RW:
    case SENSOR_DIRECTION_RW:
    case PP_CHECK_RESULT_RW:
        memcpy(buf, CANFuocoRegisterMapNames.at(monitor_settings.register_to_monitor), len);
        break;

    case SHAFT_FEEDBACK_R:
        memcpy(buf, &motor.shaft_velocity, sizeof(float));
        memcpy(buf + sizeof(float), &motor.shaft_angle, sizeof(float));
        break;

    default:
        return false;
    }

    return true;
}
