#include <SimpleFOC.h>
#include "AS5600.h"
#include "MagneticSensorAS5600.h"
//#include <pinout.h>

// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(14, 4.5, 70, 0.0018); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorAS5600 new_sensor;
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
Commander commander = Commander(Serial);
void onMotor(char* cmd){ 
  float new_speed = 0;
  commander.scalar(&new_speed, cmd);
  motor.target = float(new_speed / 10);
  last_receive_timer = millis();
}
void START_STOP(char* cmd){ 
  float motor_enable = 0;
  commander.scalar(&motor_enable, cmd);
  if(motor_enable == 1  && motor.target == 0)
  {
    motor.current_limit = 0.5;
    motor.sensor_direction = Direction::UNKNOWN;
    motor.initFOC();
  }
 }
void setup() {
  Wire.setClock(1000000);
  Serial.begin(115200);
  new_sensor.closeTransactions = false;
  new_sensor.useHysteresis = false;
  new_sensor.init();
  motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 16;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);
  motor.sensor_direction = Direction::UNKNOWN;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = 4;   // [V]
  motor.velocity_limit = 1200; // [rad/s]
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 0.2;
motor.PID_velocity.I = 20;
motor.PID_velocity.D = 0.0005; 
  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.03;
  motor.PID_velocity.output_ramp = 750;
  // init motor hardware
motor.linkSensor(&new_sensor);
//A - is the ID of driver
commander.add('A', onMotor, "motor");
commander.add('Z', START_STOP, "motor");
   current_sense.init();
   motor.linkCurrentSense(&current_sense);
   motor.init();
  //motor.initFOC();
}
uint32_t timer1 = 0;
void loop() {
  motor.loopFOC();
  if(millis() - last_receive_timer < 1000)
    motor.move();
  else
    motor.move(0);
  motor.monitor();
  commander.run();
}