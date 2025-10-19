#include <SimpleFOC.h>
// #include "AS5600.h"
#include <SimpleFOCDrivers.h>
#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
// #include "MagneticSensorAS5600.h"
//#include <pinout.h>
float max_current = 2.0;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(1, 0.955, 1050, 0.0498 * 0.001); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAS5600 new_sensor;
HallSensor new_sensor(PB6,PB7,PB8,1);
// MXLEMMINGObserverSensor new_sensor = MXLEMMINGObserverSensor(motor);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
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

// void START_STOP(char* cmd){ 
//   float _start_stop_data = 0;
//   commander.scalar(&_start_stop_data, cmd);
//   int motor_enable = _start_stop_data;
//   if(_start_stop_data == 1.0)
//   {
//     motor.current_limit = max_current;
//     motor.sensor_direction = Direction::UNKNOWN;
//     motor.initFOC();
//     motor.controller = MotionControlType::velocity;
//     motor_callibrated = true;
//     if(motor.sensor_direction == Direction::CW)
//       sen_dir = 1;
//     else
//       sen_dir = -1;
//     sen_angle = motor.zero_electric_angle;
    
//   }
//   else if(_start_stop_data == 0.0)
//   {
//     if(motor_disabled == false)
//     {
//       motor.current_limit = 0;
//       motor.controller = MotionControlType::torque;
//       motor.torque_controller = TorqueControlType::foc_current;
//       motor.LPF_current_d.Tf = 0.01;
//       motor.LPF_current_q.Tf = 0.01;
//         motor_disabled = true;
//         motor.target = 0;
//        // motor.spe
//     }
//   }
//   else if(_start_stop_data = 2.0)
//   {
//       motor.current_limit = max_current;
//       motor.controller = MotionControlType::velocity;
//       motor.torque_controller = TorqueControlType::voltage;
//       motor.LPF_current_d.Tf = 0.01;
//       motor.LPF_current_q.Tf = 0.01;
//         motor_disabled = true;
//         motor.target = 0;
//       motor_disabled = false;
//   }
// }

bool test = true, use = false;

void setup() {
  Wire.setClock(1000000);
  Serial.begin(115200);
  // new_sensor.closeTransactions = true;
  // new_sensor.useHysteresis = false;
  new_sensor.init();
  motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.pwm_frequency = 10000;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
  motor.sensor_direction = Direction::CW;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::foc_current;
 motor.monitor_variables = _MON_TARGET|_MON_ANGLE|_MON_VEL;
  // motor.monitor_variables = _MON_CURR_Q;
  motor.monitor_downsample = 300;
 // motor.LPF_current_d.Tf = 0.01;
  //motor.LPF_current_q.Tf = 0.01;
  motor.voltage_limit = 12;   // [V]
  motor.current_limit = max_current;
  motor.velocity_limit = 10000; // [rad/s]
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 0.7;
motor.PID_velocity.I = 0.1;
motor.PID_velocity.D = 0.001;//0.0035; //0.0035


motor.PID_current_d.P = 1;//2.75;
motor.PID_current_d.I = 0;
motor.PID_current_d.D = 0;
motor.LPF_current_d = 0.002f;

motor.PID_current_q.P = 1;//2.75;
motor.PID_current_q.I = 0;
motor.PID_current_q.D = 0;
motor.LPF_current_q = 0.002f;

  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.5;
  // motor.PID_velocity.output_ramp = 1000;
  motor.motion_downsample = 3;
  // init motor hardware
motor.linkSensor(&new_sensor);
//A - is the ID of drive

motor.linkCurrentSense(&current_sense);
motor.init();
current_sense.init();




use = false;
test = true;
uint8_t motor_num = 1, robot = 1;
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
}
else
{
  delay(7000);
}
motor.initFOC();

Serial.println("Motor ready.");
Serial.println("Torque mode.");

}


uint32_t timer1 = 0, timer2 = 0;
uint16_t i = 0;
float tmp_rot = 0;
void loop() {
  motor.loopFOC();
  //motor.move(10);
  //timer2 += 1;
  // if(timer2 == 100)
  // {
  //   timer2 = 0;
    Serial.println("goida");
  // }
   if(!motor_disabled)
   {
      // motor.current_limit = max_current;
      // motor.PID_velocity.P = 0.05;
      // motor.PID_velocity.I = 0.01;
      // motor.PID_velocity.D = 0; 
      // motor.velocity_limit = 10000;
      // motor.PID_velocity.output_ramp = 1000;

      
      // if(Serial.available()){
      //   int newSpeed = Serial.parseInt();
      //   if(newSpeed > -2500 && newSpeed < 2500)
      //     recievedSpeed = newSpeed * 0.10466;
      //   // float newSpeed = Serial.parseFloat();
      //   // if(newSpeed < -2.0 && newSpeed > 2.0)
      //   // {newSpeed = 2.0 * (newSpeed/abs(newSpeed));}
      //   // recievedSpeed = newSpeed;
      //   Serial.println(recievedSpeed);
      // }
      // Serial.println
      tmp_rot = new_sensor.getFullRotations();
      // if (millis() - timer1 > 5000)
      // {
      //   i += 1;
      //   if (i >= 11)
      //   {
      //     i = 0;
      //   }
      //   timer1 = millis();
      // }
      static float recievedSpeed = -2000 * 0.10466;
      // recievedSpeed = 
      // float tmp_f = recievedSpeed * float(i);
      // if (tmp_f < -200) tmp_f = -200;
      // if (tmp_f > 200) tmp_f = 200;
      motor.move(recievedSpeed);
      // motor.current_limit = max_current;
      // motor.PID_velocity.P = 0.75;
      // motor.PID_velocity.I = 0.1;
      // motor.PID_velocity.D = 0.0035; 
      // motor.velocity_limit = 10000;
      // motor.PID_velocity.output_ramp = 1000;
      //  motor.move(2);
    /*}
    else if(millis() - last_receive_timer < 20000)
    {
      motor.move(-1*0.10466);
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
        */
    // motor.monitor();
  }
}