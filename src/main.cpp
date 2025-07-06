#include <SimpleFOC.h>
#include "AS5600.h"
#include "MagneticSensorAS5600.h"
//#include <pinout.h>
float max_current = 1.35;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(14, 4.5, 63.5, 0.0008); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorAS5600 new_sensor;
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
Commander commander = Commander(Serial, 'e', false);
double sen_angle = 0;
int8_t sen_dir = 0;
bool motor_callibrated = false;
bool motor_disabled = false;
void onMotor(char* cmd){ 
  float new_speed = 0;
  commander.scalar(&new_speed, cmd);
  motor.target = float(new_speed);
  last_receive_timer = millis();
}
void START_STOP(char* cmd){ 
  float _start_stop_data = 0;
  commander.scalar(&_start_stop_data, cmd);
  int motor_enable = _start_stop_data;
  if(_start_stop_data == 1.0)
  {
    motor.current_limit = max_current;
    motor.sensor_direction = Direction::UNKNOWN;
    motor.initFOC();
    motor.controller = MotionControlType::velocity;
    motor_callibrated = true;
    if(motor.sensor_direction == Direction::CW)
      sen_dir = 1;
    else
      sen_dir = -1;
    sen_angle = motor.zero_electric_angle;
    
  }
  else if(_start_stop_data == 0.0)
  {
    if(motor_disabled == false)
    {
      motor.current_limit = 0;
      motor.controller = MotionControlType::torque;
      motor.torque_controller = TorqueControlType::foc_current;
      motor.LPF_current_d.Tf = 0.01;
      motor.LPF_current_q.Tf = 0.01;
        motor_disabled = true;
        motor.target = 0;
       // motor.spe
    }
  }
  else if(_start_stop_data = 2.0)
  {
      motor.current_limit = max_current;
      motor.controller = MotionControlType::velocity;
      motor.torque_controller = TorqueControlType::voltage;
      motor.LPF_current_d.Tf = 0.01;
      motor.LPF_current_q.Tf = 0.01;
        motor_disabled = true;
        motor.target = 0;
      motor_disabled = false;
  }
}
bool test = false, use = false;
 void setup() {
  Wire.setClock(1000000);
  Serial.begin(115200);
  new_sensor.closeTransactions = true;
  new_sensor.useHysteresis = false;
  new_sensor.init();
  motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 17.4;
  driver.pwm_frequency = 14000;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
  motor.sensor_direction = Direction::UNKNOWN;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
 // motor.monitor_variables = _MON_CURR_D|_MON_CURR_Q;
  motor.monitor_variables = _MON_VEL;
  motor.monitor_downsample = 300;
 // motor.LPF_current_d.Tf = 0.01;
  //motor.LPF_current_q.Tf = 0.01;
  motor.voltage_limit = 8;   // [V]
  motor.current_limit = max_current;
  motor.velocity_limit = 10000; // [rad/s]
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 0.7;
motor.PID_velocity.I = 7;
motor.PID_velocity.D = 0.0035; //0.0035


motor.PID_current_d.P = 2.75;
motor.PID_current_d.I = 350;
motor.PID_current_d.D = 0;
motor.LPF_current_d = 0.002f;

motor.PID_current_q.P = 2.75;
motor.PID_current_q.I = 350;
motor.PID_current_q.D = 0;
motor.LPF_current_q = 0.002f;

  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.05;
  motor.PID_velocity.output_ramp = 1000;
  motor.motion_downsample = 3;
  // init motor hardware
motor.linkSensor(&new_sensor);
//A - is the ID of drive
commander.add('Z', START_STOP, "motor");
current_sense.init();
motor.linkCurrentSense(&current_sense);
motor.init();
use = true;
test = false;
uint8_t motor_num = 3, robot = 2;
if(!test)
{
  if(robot == 1)
  {
    switch (motor_num)
    {
      case 1:
      motor.sensor_direction = Direction::CCW;
    motor.zero_electric_angle = 2.44;
    commander.add('A', onMotor, "motor");
    break;
      case 2:
      motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 2.14;    
    commander.add('B', onMotor, "motor");
    break;

    case 3:
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 1.36;
    commander.add('C', onMotor, "motor");
      break;
    case 4:
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 0.52;
    commander.add('D', onMotor, "motor");
      break;
    }
  }

  if(robot == 2)
  {
    switch (motor_num)
    {
      case 1:
      motor.sensor_direction = Direction::CCW;
    motor.zero_electric_angle = 5.88;
    commander.add('A', onMotor, "motor");
    break;
      case 2:
      motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 6.25;    
    commander.add('B', onMotor, "motor");
    break;

    case 3:
    motor.sensor_direction = Direction::CW;
    motor.zero_electric_angle = 1.35;
    commander.add('C', onMotor, "motor");
      break;
    case 4:
    motor.sensor_direction = Direction::CCW;
    motor.zero_electric_angle = 3.06;
    commander.add('D', onMotor, "motor");
      break;
    }
  }
}
else
{
  delay(7000);
}
motor.initFOC();
}
uint32_t timer1 = 0, timer2 = 0;
void loop() {
  motor.loopFOC();
  //motor.move(10);
  //timer2 += 1;
  // if(timer2 == 100)
  // {
  //   timer2 = 0;
  //   Serial.println(new_sensor.getAngle());
  // }
   if(!motor_disabled)
   {
   //motor.loopFOC();

    if(use == true)
    {
      if(millis() - last_receive_timer < 100)
      {
        motor.current_limit = max_current;
        motor.PID_velocity.P = 0.75;
        motor.PID_velocity.I = 0.001;
        motor.PID_velocity.D = 0.0035; 
        motor.velocity_limit = 10000;
        motor.PID_velocity.output_ramp = 1000;
        motor.move();
      }
      else
      {
      // motor.LPF_velocity = 0.06;
        motor.PID_velocity.P = 0.05;
        motor.PID_velocity.I = 0.0025;
        motor.PID_velocity.D = 0.0;
        motor.current_limit = 0.01;
        motor.velocity_limit = 0.05;
        motor.move(0);
      }
    }
   if(use == false)
   {
    if(millis() - last_receive_timer < 4000)
    {
      motor.current_limit = max_current;
      motor.PID_velocity.P = 0.75;
      motor.PID_velocity.I = 0.001;
      motor.PID_velocity.D = 0.0035; 
      motor.velocity_limit = 10000;
      motor.PID_velocity.output_ramp = 1000;
      motor.move(20);
      // motor.current_limit = max_current;
      // motor.PID_velocity.P = 0.75;
      // motor.PID_velocity.I = 0.1;
      // motor.PID_velocity.D = 0.0035; 
      // motor.velocity_limit = 10000;
      // motor.PID_velocity.output_ramp = 1000;
      //  motor.move(2);
    }
    else if(millis() - last_receive_timer < 8000)
    {
      motor.move(-6);
      }
      else if(millis() - last_receive_timer < 1000)
    {
    // motor.LPF_velocity = 0.06;
      motor.PID_velocity.P = 0.1;
      motor.PID_velocity.I = 0.005;
      motor.PID_velocity.D = 0.0;
      motor.current_limit = 0.01;
      motor.velocity_limit = 0.05;
      motor.move(0);
    }
    else if(millis() - last_receive_timer < 1000)
    {

      motor.PID_velocity.P = 0.1;
      motor.PID_velocity.I = 0.005;
      motor.PID_velocity.D = 0.0; 
      motor.velocity_limit = 0.05;
      motor.PID_velocity.output_ramp = 1000;
      motor.move(0);
      }
      else
        last_receive_timer = millis();
    motor.monitor();
   }
  }
  commander.run();
}