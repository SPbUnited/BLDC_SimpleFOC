#include "SimpleFOCDrivers.h"
#include <SimpleFOC.h>
#include <SPI.h>

#define SPI_SR_TXP

// #include "./STM32_CAN/STM32_CAN.h"
// #include "AS5600.h"
// #include "encoders/as5048a/AS5048A.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
// #include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
//#include <pinout.h>
// STM32_CAN Can1( PA11, PB9 );
// SPIClass *hspi = NULL;
// static CAN_message_t CAN_RX_msg;

// SPI Clock Speed - AS5048A support 10 MHz maximum
// static const uint32_t AS5048A_spiClk = 1000000UL;  // ?1 MHz

// Create sensor instances
// MagneticSensorAS5048A sensorA(pinNametoDigitalPin(PB_4), false, SPISettings(AS5048A_spiClk, MSBFIRST, SPI_MODE1));
// MagneticSensorAS5048A sensorB(CS_ENC_B, false, SPISettings(AS5048A_spiClk, MSBFIRST, SPI_MODE1));
// 
// void setup() {
	
// 	// Create hspi instance
// 	hspi = new SPIClass(HSPI); // Original ESP32 has HSPI and VSPI available for use
// 	hspi->begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI); // Assign the pins 0.0021

float max_current = 1.0;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(7, 3.6, 82.5, 0.0021); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
//MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
// MagneticSensorAS5600 new_sensor;
SPIClass SPI_3(PB5, A_CAN_SHDN, A_BUTTON);
MagneticSensorAS5048A new_sensor = MagneticSensorAS5048A(pinNametoDigitalPin(PA_15));
// MagneticSensorSPI new_sensor = MagneticSensorSPI(pinNametoDigitalPin(PB_4), 16, 0x3FFF);//MagneticSensorSPI(AS5048_SPI, pinNametoDigitalPin(PB_4));
// MXLEMMINGObserverSensor new_sensor = MXLEMMINGObserverSensor(motor);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
// Commander commander = Commander(Serial, 'e', false);
double sen_angle = 0;
int8_t sen_dir = 0;
bool motor_callibrated = false;
bool motor_disabled = false;

// void onMotor(char* cmd){ 
//   float new_speed = 0;
//   commander.scalar(&new_speed, cmd);
//   motor.target = float(new_speed);
//   last_receive_timer = millis();
// }

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

bool test_motor = true, use = true;

void setup() {
  // Wire.setClock(1000000);
  // Serial.begin(115200);
  // new_sensor.closeTransactions = true;
  // new_sensor.useHysteresis = false;
  // SPIClass mS = SPIClass(pinNametoDigitalPin(PB_5), pinNametoDigitalPin(PC_11), pinNametoDigitalPin(PC_10), pinNametoDigitalPin(PB_4));
  // Can1.begin();
  // Can1.setFrameFormat(STM32_CAN::FRAME_FORMAT::CLASSIC);
  // Can1.setBaudRate(250000, 250000);


  // new_sensor.init(&mS);
  // mS.begin();
  	// hspi = new SPIClass(); // Original ESP32 has HSPI and VSPI available for use
	// hspi->begin(pinNametoDigitalPin(PB_5), pinNametoDigitalPin(PC_11), pinNametoDigitalPin(PC_10)); // Assign the pins
  // SPI3
  // new_sensor.spi_mode = SPI_MODE1;
  // new_sensor.clock_speed = 500000;
  new_sensor.init(&SPI_3);
  // new_sensor.init();
  // motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.voltage_limit = 4;
  driver.pwm_frequency = 20000;
  driver.init();

  // link the motor and the driver
  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
  // motor.sensor_direction = Direction::UNKNOWN;
  // limiting motor movements
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
 motor.monitor_variables = _MON_CURR_D|_MON_CURR_Q;
  motor.monitor_variables = _MON_VEL;
  motor.monitor_downsample = 300;
 // motor.LPF_current_d.Tf = 0.01;
  //motor.LPF_current_q.Tf = 0.01;
  // motor.voltage_limit = 4;   // [V]
  motor.current_limit = max_current;
  motor.velocity_limit = 10000; // [rad/s]
  // default P=0.5 I = 10 D = 0
motor.PID_velocity.P = 5;
motor.PID_velocity.I = 2;
motor.PID_velocity.D = 0.35; //0.0035


motor.PID_current_d.P = 3;
motor.PID_current_d.I = 250;
motor.PID_current_d.D = 0;
motor.LPF_current_d = 0.002f;

motor.PID_current_q.P = 3;
motor.PID_current_q.I = 250;
motor.PID_current_q.D = 0;
motor.LPF_current_q = 0.002f;

  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.3;
  // motor.PID_velocity.output_ramp = 1000;
  // motor.motion_downsample = 3;
  // init motor hardware
motor.linkSensor(&new_sensor);
//A - is the ID of drive
// commander.add('Z', START_STOP, "motor");
motor.linkCurrentSense(&current_sense);
motor.sensor_direction= Direction::UNKNOWN;

motor.init();
current_sense.init();
// motor.zero_electric_angle = 0;


use = false;
test_motor = true;
uint8_t motor_num = 1, robot = 1;
motor.initFOC();
pinMode(pinNametoDigitalPin(PC_6), OUTPUT);
// digitalPin
digitalWrite(pinNametoDigitalPin(PC_6), HIGH);
}


uint32_t timer1 = 0, timer2 = 0;
bool flag = true;
void loop() {
  motor.loopFOC();
      motor.current_limit = max_current;
      // motor.PID_velocity.P = 1;
      // motor.PID_velocity.I = 10;
      // motor.PID_velocity.D = 0; 
      motor.velocity_limit = 10000;
      // motor.PID_velocity.output_ramp = 1000;
      // motor.move(10);
      // static float recievedSpeed = 0;
      // // motor.move(3.14);
      if(millis() - timer1 >= 5000)
      {
        flag = !flag;
        timer1 = millis();
      }
      if (flag)
      {
          motor.move(-7);
      }
      else
      {
        motor.move(7);
      }
      // motor.monitor();

    //   if (Can1.read(CAN_RX_msg) ) {
    //     Serial.print("Channel:");
    //     Serial.print(CAN_RX_msg.bus);
    //     if (CAN_RX_msg.flags.extended == false) {
    //       Serial.print(" Standard ID:");
    //     }
    //     else {
    //       Serial.print(" Extended ID:");
    //     }
    //     Serial.print(CAN_RX_msg.id, HEX);

    // #if defined(HAL_FDCAN_MODULE_ENABLED)
    //     if(CAN_RX_msg.flags.fd_rateswitch) {
    //       Serial.println(" [FD-BRS]");
    //     }
    //     else if(CAN_RX_msg.flags.fd_frame) {
    //       Serial.println(" [FD]");
    //     }
    // #endif

    //     Serial.print(" DLC: ");
    //     Serial.print(CAN_RX_msg.len);
    //     if (CAN_RX_msg.flags.remote == false) {
    //       Serial.print(" buf: ");
    //       for(int i=0; i<CAN_RX_msg.len; i++) {
    //         Serial.print("0x"); 
    //         Serial.print(CAN_RX_msg.buf[i], HEX); 
    //         if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
    //       }
    //       Serial.println();
    //     } else {
    //       Serial.println(" Data: REMOTE REQUEST FRAME");
    //     }
    // }
}