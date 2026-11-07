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
    PID_CURREND_D_OUTPUT_RAMP_RW = 43,
    PID_CURRENT_D_LIMIT_RW = 44,
    LPF_CURRENT_D_TF_RW = 45,
    PID_CURRENT_Q_P_RW = 46,
    PID_CURRENT_Q_I_RW = 47,
    PID_CURRENT_Q_D_RW = 48,
    PID_CURRENT_Q_OUTPUT_RAMP_RW = 49,
    PID_CURRENT_Q_LIMIT_RW = 50,
    LPF_CURRENT_Q_TF_RW = 51,
    PID_VELOCITY_P_RW = 52,
    PID_VELOCITY_I_RW = 53,
    PID_VELOCITY_D_RW = 54,
    PID_VELOCITY_OUTPUT_RAMP_RW = 55,
    PID_VELOCITY_LIMIT_RW = 56,
    LPF_VELOCITY_TF_RW = 57,
    P_ANGLE_P_RW = 58,
    P_ANGLE_LIMIT_RW = 59,
    LPF_ANGLE_TF_RW = 60,
    MOTION_DOWNSAMPLE_RW = 61,
    MOTION_CNT_RW = 62,

    // Sensor settings
    SENSOR_OFFSET_RW = 63,
    ZERO_ELECTRIC_ANGLE_RW = 64,
    SENSOR_DIRECTION_RW = 65,
    PP_CHECK_RESULT_RW = 66,

    // Virtual registers
    MULTI_TARGET_W = 96,
    SHAFT_FEEDBACK_R = 102,
    PCB_TEMPERATURE_R = 103,
    MOTOR_TEMPERATURE_R = 104,
    SAVE_TO_EEPROM_W = 127,
};

struct CANFuocoMotorConfig
{
    uint8_t &motor_id;
    float (*get_supply_voltage)(void) = nullptr;
    float (*get_pcb_temperature)(void) = nullptr;
    float (*get_motor_temperature)(void) = nullptr;
    BLDCMotor &motor;
    float speed = 0;
};

struct CANFuocoMonitorSettings
{
    uint8_t registers[4] = {SHAFT_FEEDBACK_R, 0, 0, 0};
};

class CANFuoco : public CANFuocoMotorConfig
{
  private:
    CANFuocoMonitorSettings monitor_settings;
    uint8_t monitor_index = 0;
    CANFuocoRegisterMap active_feedback_register = SHAFT_FEEDBACK_R;

    std::map<CANFuocoRegisterMap, uint8_t *> CANFuocoRegisterMapNames;

  public:
    CANFuoco(CANFuocoMotorConfig &motor_config);

    void can_rx_callback(uint32_t m_id, size_t len, uint8_t *buf);
    uint8_t can_tx_callback(uint8_t *buf);
    CANFuocoRegisterMap feedback_register() const;
};

CANFuoco::CANFuoco(CANFuocoMotorConfig &motor_config) : CANFuocoMotorConfig(motor_config)
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
        {PID_CURREND_D_OUTPUT_RAMP_RW,
         reinterpret_cast<uint8_t *>(&motor.PID_current_d.output_ramp)},
        {PID_CURRENT_D_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_d.limit)},
        {LPF_CURRENT_D_TF_RW, reinterpret_cast<uint8_t *>(&motor.LPF_current_d.Tf)},
        {PID_CURRENT_Q_P_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.P)},
        {PID_CURRENT_Q_I_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.I)},
        {PID_CURRENT_Q_D_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.D)},
        {PID_CURRENT_Q_OUTPUT_RAMP_RW,
         reinterpret_cast<uint8_t *>(&motor.PID_current_q.output_ramp)},
        {PID_CURRENT_Q_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.PID_current_q.limit)},
        {LPF_CURRENT_Q_TF_RW, reinterpret_cast<uint8_t *>(&motor.LPF_current_q.Tf)},
        {PID_VELOCITY_P_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.P)},
        {PID_VELOCITY_I_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.I)},
        {PID_VELOCITY_D_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.D)},
        {PID_VELOCITY_OUTPUT_RAMP_RW,
         reinterpret_cast<uint8_t *>(&motor.PID_velocity.output_ramp)},
        {PID_VELOCITY_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.PID_velocity.limit)},
        {LPF_VELOCITY_TF_RW, reinterpret_cast<uint8_t *>(&motor.LPF_velocity.Tf)},
        {P_ANGLE_P_RW, reinterpret_cast<uint8_t *>(&motor.P_angle.P)},
        {P_ANGLE_LIMIT_RW, reinterpret_cast<uint8_t *>(&motor.P_angle.limit)},
        {LPF_ANGLE_TF_RW, reinterpret_cast<uint8_t *>(&motor.LPF_angle.Tf)},
        {MOTION_DOWNSAMPLE_RW, reinterpret_cast<uint8_t *>(&motor.motion_downsample)},
        {MOTION_CNT_RW, reinterpret_cast<uint8_t *>(&motor.motion_cnt)},
        {SENSOR_OFFSET_RW, reinterpret_cast<uint8_t *>(&motor.sensor_offset)},
        {ZERO_ELECTRIC_ANGLE_RW, reinterpret_cast<uint8_t *>(&motor.zero_electric_angle)},
        {SENSOR_DIRECTION_RW, reinterpret_cast<uint8_t *>(&motor.sensor_direction)},
        {PP_CHECK_RESULT_RW, reinterpret_cast<uint8_t *>(&motor.pp_check_result)},
    };
}

void CANFuoco::can_rx_callback(uint32_t m_id, size_t len, uint8_t *buf)
{
    CANFuocoRegisterMap id = static_cast<CANFuocoRegisterMap>(m_id);
    switch (id)
    {
        default:
        case EMERGENCY_STOP_W:
            motor.disable();
            delay(15000);
            break;

        case MONITOR_SETTINGS_RW:
            if (len >= 1 && len <= sizeof(monitor_settings.registers))
            {
                memset(monitor_settings.registers, 0, sizeof(monitor_settings.registers));
                memcpy(monitor_settings.registers, buf, len);
                monitor_index = 0;
            }
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
        case PID_CURREND_D_OUTPUT_RAMP_RW:
        case PID_CURRENT_D_LIMIT_RW:
        case LPF_CURRENT_D_TF_RW:
        case PID_CURRENT_Q_P_RW:
        case PID_CURRENT_Q_I_RW:
        case PID_CURRENT_Q_D_RW:
        case PID_CURRENT_Q_OUTPUT_RAMP_RW:
        case PID_CURRENT_Q_LIMIT_RW:
        case LPF_CURRENT_Q_TF_RW:
        case PID_VELOCITY_P_RW:
        case PID_VELOCITY_I_RW:
        case PID_VELOCITY_D_RW:
        case PID_VELOCITY_OUTPUT_RAMP_RW:
        case PID_VELOCITY_LIMIT_RW:
        case LPF_VELOCITY_TF_RW:
        case P_ANGLE_P_RW:
        case P_ANGLE_LIMIT_RW:
        case LPF_ANGLE_TF_RW:
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
 * @return Number of prepared feedback bytes, or 0 if register is unsupported
 */
uint8_t CANFuoco::can_tx_callback(uint8_t *buf)
{
    for (uint8_t checked = 0; checked < 4; ++checked)
    {
        const auto reg =
            static_cast<CANFuocoRegisterMap>(monitor_settings.registers[monitor_index]);
        monitor_index = (monitor_index + 1) % 4;
        if (reg != EMERGENCY_STOP_W)
        {
            active_feedback_register = reg;
            break;
        }
        if (checked == 3)
        {
            return 0;
        }
    }

    switch (active_feedback_register)
    {
        case SUPPLY_VOLTAGE_R:
        {
            if (get_supply_voltage == nullptr)
            {
                return 0;
            }
            float supply_voltage = get_supply_voltage();
            memcpy(buf, &supply_voltage, sizeof(supply_voltage));
            return sizeof(supply_voltage);
        }

        case PCB_TEMPERATURE_R:
        case MOTOR_TEMPERATURE_R:
        {
            auto get_temperature = active_feedback_register == PCB_TEMPERATURE_R
                                       ? get_pcb_temperature
                                       : get_motor_temperature;
            if (get_temperature == nullptr)
            {
                return 0;
            }
            const float temperature = get_temperature();
            memcpy(buf, &temperature, sizeof(temperature));
            return sizeof(temperature);
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
        case PID_CURREND_D_OUTPUT_RAMP_RW:
        case PID_CURRENT_D_LIMIT_RW:
        case LPF_CURRENT_D_TF_RW:
        case PID_CURRENT_Q_P_RW:
        case PID_CURRENT_Q_I_RW:
        case PID_CURRENT_Q_D_RW:
        case PID_CURRENT_Q_OUTPUT_RAMP_RW:
        case PID_CURRENT_Q_LIMIT_RW:
        case LPF_CURRENT_Q_TF_RW:
        case PID_VELOCITY_P_RW:
        case PID_VELOCITY_I_RW:
        case PID_VELOCITY_D_RW:
        case PID_VELOCITY_OUTPUT_RAMP_RW:
        case PID_VELOCITY_LIMIT_RW:
        case LPF_VELOCITY_TF_RW:
        case P_ANGLE_P_RW:
        case P_ANGLE_LIMIT_RW:
        case LPF_ANGLE_TF_RW:
        case MOTION_DOWNSAMPLE_RW:
        case MOTION_CNT_RW:
        case SENSOR_OFFSET_RW:
        case ZERO_ELECTRIC_ANGLE_RW:
        case SENSOR_DIRECTION_RW:
        case PP_CHECK_RESULT_RW:
        {
            auto reg = active_feedback_register;
            size_t len =
                (reg == ENABLED_RW || reg == MODULATION_CENTERED_RW || reg == PP_CHECK_RESULT_RW)
                    ? sizeof(bool)
                : (reg == MOTOR_STATUS_R || reg == FOC_MODULATION_RW ||
                   reg == TORQUE_CONTROLLER_RW || reg == CONTROLLER_RW ||
                   reg == SENSOR_DIRECTION_RW)
                    ? sizeof(uint8_t)
                    : sizeof(float);
            memcpy(buf, CANFuocoRegisterMapNames.at(reg), len);
            return len;
        }

        case SHAFT_FEEDBACK_R:
            memcpy(buf, &motor.shaft_velocity, sizeof(float));
            memcpy(buf + sizeof(float), &motor.shaft_angle, sizeof(float));
            return 2 * sizeof(float);

        default:
            return 0;
    }
}

CANFuocoRegisterMap CANFuoco::feedback_register() const
{
    return active_feedback_register;
}
