#include <SimpleFOC.h>
//#include <pinout.h>

// // NUMBER OF POLE PAIRS, NOT POLES
BLDCMotor motor = BLDCMotor(14, 4.5, 70, 0.0018); 
// // MUST USE 6PWM FOR B-G431 DRIVER
// BLDCDriver6PWM driver = BLDCDriver6PWM(PHASE_UH, PHASE_UL, PHASE_VH, PHASE_VL, PHASE_WH, PHASE_WL); 
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// LowsideCurrentSense current_sense = LowsideCurrentSense(0.003, 16.0, PA1, PA7, PB0);
//LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
Commander commander = Commander(Serial);
void onMotor(char* cmd){ commander.motor(&motor, cmd); }
void setup() {
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 16;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  motor.sensor_direction = Direction::CW;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.voltage_limit = 4;   // [V]
  motor.velocity_limit = 1200; // [rad/s]
  motor.current_limit = 0.5;
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 0.2;
motor.PID_velocity.I = 20;
motor.PID_velocity.D = 0.001; 
  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.03;
  motor.PID_velocity.output_ramp = 750;
  // init motor hardware
  motor.init();
sensor.init();
motor.linkSensor(&sensor);
commander.add('A', onMotor, "motor");
  // current_sense.init();
  // motor.linkCurrentSense(&current_sense);

  motor.initFOC();
}
uint32_t timer1 = 0;
void loop() {
  motor.loopFOC();

  motor.move();
  motor.monitor();
  commander.run();
 // motor.move(float(analogRead(A_POTENTIOMETER) / 20) - 25);
  // motor.move(100);
  // Serial.print(analogRead(A_POTENTIOMETER));

  // PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  // Serial.println(currents.a);

  // Serial.print("  ");
  // Serial.println(motor.shaft_velocity);
}