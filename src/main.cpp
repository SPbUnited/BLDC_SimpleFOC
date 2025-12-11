#include "SimpleFOCDrivers.h"
#include <SimpleFOC.h>
// #include "AS5600.h"
#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
// #include "MagneticSensorAS5600.h"
//#include <pinout.h>
float max_current = 2.0;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(1, 0.94, 770, 0.104 * 0.001); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAS5600 new_sensor;1 2 3 3 1 2 2 3 1
    //                             3 2 1 1 3 2 2 1 3
HallSensor new_sensor(PB6,PB7,PB8,1);
// MXLEMMINGObserverSensor new_sensor = MXLEMMINGObserverSensor(motor);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
double sen_angle = 0;
int8_t sen_dir = 0;
bool motor_callibrated = false;
bool motor_disabled = false;



bool test = true, use = false;

void setup() {
  // new_sensor.closeTransactions = true;
  // new_sensor.useHysteresis = false;
  new_sensor.pullup = Pullup::USE_INTERN;
  new_sensor.init();
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.voltage_limit = 12.0;
  driver.pwm_frequency = 14000;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
  // motor.sensor_direction = Direction::CW;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
 motor.monitor_variables = _MON_TARGET|_MON_ANGLE|_MON_VEL;
  // motor.monitor_variables = _MON_CURR_Q;
  motor.monitor_downsample = 300;
 // motor.LPF_current_d.Tf = 0.01;
  //motor.LPF_current_q.Tf = 0.01;
  // motor.voltage_limit = 12;   // [V]
  motor.current_limit = max_current;
  motor.velocity_limit = 10000; // [rad/s]
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 1.5;
motor.PID_velocity.I = 0.1;
motor.PID_velocity.D = 0;//0.001;//0.0035; //0.0035


motor.PID_current_d.P = 0.7;//2.75;
motor.PID_current_d.I = 1;
motor.PID_current_d.D = 0;
motor.LPF_current_d = 0.002f;

motor.PID_current_q.P = 0.7;//2.75;
motor.PID_current_q.I = 1;
motor.PID_current_q.D = 0;
motor.LPF_current_q = 0.002f;

  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.05;
  // motor.PID_velocity.output_ramp = 1000;
  motor.motion_downsample = 0;
  // init motor hardware
  
motor.linkSensor(&new_sensor);
//A - is the ID of drive

motor.linkCurrentSense(&current_sense);
// motor.sensor_direction= Direction::UNKNOWN;
new_sensor.electric_rotations = 0;
motor.init();
current_sense.init();


use = false;
test = true;
uint8_t motor_num = 1, robot = 1;
motor.initFOC();
pinMode(pinNametoDigitalPin(PC_6), OUTPUT);
// digitalPin
digitalWrite(pinNametoDigitalPin(PC_6), LOW);
}


uint32_t timer1 = 0, timer2 = 0;
uint16_t i = 0;
float tmp_rot = 0;
void loop() {
  motor.loopFOC();
   if(!motor_disabled)
   {
      static float recievedSpeed = -1500 * 0.10466;
      motor.move(-150);
  }
}