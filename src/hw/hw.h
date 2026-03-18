#pragma once

#include <SimpleFOC.h>

#include "encoders/as5048a/MagneticSensorAS5048A.h"

extern int numMotorsUsed;

#define ADC_MOTOR_TEMP_PIN PA4
#define ADC_PCB_TEMP_PIN PB14
#define ADC_VBUS_PIN PA0

#define MAX_MOTOR_TEMP 75.0f
#define MAX_PCM_TEMP 75.0f
#define MIN_MOTOR_TEMP -5.0f
#define MIN_PCM_TEMP -5.0f

#define WARNING_MOTOR_TEMP 65.0f
#define WARNING_PCB_TEMP 65.0f

#define NORMAL_CURRENT_LIMIT 4.0
#define WARNING_CURRENT_LIMIT 1.0

const PinMap PinMap_SPI_MOSI[] = {
    {PB_5, SPI3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF6_SPI3)},
};

const PinMap PinMap_SPI_MISO[] = {
    {PC_11, SPI3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF6_SPI3)},
};

const PinMap PinMap_SPI_SCLK[] = {
    {PC_10, SPI3, STM_PIN_DATA(STM_MODE_AF_PP, GPIO_PULLUP, GPIO_AF6_SPI3)},
};

namespace hw
{

// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(7, 3.6, 82.5, 0.0021);
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);
SPIClass SPI_3(PB5, A_CAN_SHDN, A_BUTTON);
MagneticSensorAS5048A hall_encoder = MagneticSensorAS5048A(pinNametoDigitalPin(PA_15));
LowsideCurrentSense current_sense =
    LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);

void init(bool is_warning = false)
{
    // hall_encoder.init();
    // power supply voltage [V]
    driver.voltage_power_supply = 24.0;
    driver.voltage_limit = driver.voltage_power_supply;
    driver.pwm_frequency = is_warning ? 10000 : 20000;
    driver.init();

    hall_encoder.init(&SPI_3);

    motor.linkDriver(&driver);
    current_sense.skip_align = true;
    current_sense.linkDriver(&driver);
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.torque_controller = TorqueControlType::voltage;
    // motor.monitor_variables = _MON_CURR_D | _MON_CURR_Q;
    // motor.monitor_variables = _MON_VEL;
    // motor.monitor_downsample = 300;
    // motor.LPF_current_d.Tf = 0.01;
    // motor.LPF_current_q.Tf = 0.01;
    // motor.voltage_limit = 4;   // [V]

    motor.controller = MotionControlType::velocity;
    motor.linkSensor(&hall_encoder);
    motor.linkCurrentSense(&current_sense);
    motor.sensor_direction = Direction::CW;
    motor.voltage_sensor_align = 6.0;

    motor.init();
    current_sense.init();

    motor.initFOC();

    // motor.setPhaseVoltage(3, 0, _3PI_2);
    // _delay(700);
    // hall_encoder.update();
    // volatile float angle = motor.electricalAngle();
    // (void)angle;

    // https://community.simplefoc.com/t/stm32-analogread/4476
    // pinMode(pinNametoDigitalPin(PA_4), INPUT_ANALOG);
    // pinMode(pinNametoDigitalPin(PB_14), INPUT_ANALOG);

    float constexpr Km = 10.0;
    float constexpr Tf = 0.004;
    float constexpr Tu = 0.05;
    float constexpr Tm = 0.02;
    float constexpr B = 2;
    float constexpr Kp = 0.1;
    float constexpr Ki = Kp / Tm;

    motor.current_limit = 1.0;    // [A]
    motor.velocity_limit = 1000;  // [rad/s]
    motor.PID_velocity.P = Kp;
    motor.PID_velocity.I = Ki;
    motor.PID_velocity.D = 0.0;  // 0.0035
    motor.PID_velocity.limit = is_warning ? WARNING_CURRENT_LIMIT : NORMAL_CURRENT_LIMIT;  // [A]
    motor.LPF_velocity.Tf = Tf;

    // motor.PID_current_d.P = 3;
    // motor.PID_current_d.I = 1000;
    // motor.PID_current_d.D = 0;
    motor.PID_current_d.limit = driver.voltage_power_supply;
    // motor.LPF_current_d = 0.001f;

    // motor.PID_current_q.P = 3;
    // motor.PID_current_q.I = 1000;
    // motor.PID_current_q.D = 0;
    motor.PID_current_q.limit = driver.voltage_power_supply;
    // motor.LPF_current_q = 0.001f;
}

void reinit()
{
    // driver.disable();
    // motor.disable();

    // driver.pwm_frequency = 10000;
    // numMotorsUsed = 0;
    // driver.init();
    // motor.init();
    // motor.initFOC();
    motor.PID_velocity.limit = WARNING_CURRENT_LIMIT;
}

double get_motor_temp()
{
    static float old_temp = 0.0;
    double analog = _readADCVoltageLowSide(A_POTENTIOMETER, current_sense.params);
    // double analog = analogRead(ADC_MOTOR_TEMP_PIN) * 3.3 / 1024;
    // double analog = analogRead(PA4);
    analog = ((float)((1 << 12) - 1) / ((4096.0f / 3.3f) * analog) - 1.0f) / 2.12;
    analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);

    float temp = 1.0f / analog - 273.15f;

    float filtered_temp = 0.8f * temp + 0.2f * old_temp;
    old_temp = filtered_temp;
    return filtered_temp;
    // return analog;
}

float get_vbus(void)
{
    return _readADCVoltageLowSide(A_VBUS, current_sense.params) * 9.2;
    // return analogRead(ADC_VBUS_PIN) * 3.3 / 1024 * 9.2;
}

float get_pcb_temp()
{
    static float old_temp = 0.0;
    double analog = _readADCVoltageLowSide(A_TEMPERATURE, current_sense.params);
    // double analog = analogRead(ADC_PCB_TEMP_PIN) * 3.3 / 1024;
    analog = ((float)((1 << 12) - 1) / ((4096.0f / 3.3f) * analog) - 1.0f) / 2.12;
    analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);

    float temp = 1.0f / analog - 273.15f;

    float filtered_temp = 0.8f * temp + 0.2f * old_temp;
    old_temp = filtered_temp;
    return filtered_temp;
    // return analog;
}

void foc_loop()
{
    motor.loopFOC();
    motor.move();
}

void foc_abort()
{
    motor.target = 0;
    motor.disable();
}

bool is_pcb_temp_ok(float temp)
{
    return MIN_PCM_TEMP < temp && temp < MAX_PCM_TEMP;
}

bool is_motor_temp_ok(float temp)
{
    return MIN_MOTOR_TEMP < temp && temp < MAX_MOTOR_TEMP;
}

bool is_pcb_temp_warning(float temp)
{
    return temp > WARNING_PCB_TEMP;
}

bool is_motor_temp_warning(float temp)
{
    return temp > WARNING_MOTOR_TEMP;
}

};  // namespace hw