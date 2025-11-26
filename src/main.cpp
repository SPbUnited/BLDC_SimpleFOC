#include "SimpleFOCDrivers.h"
#include <SimpleFOC.h>
#include <SPI.h>
#include <SimpleCan.h>
#define SPI_SR_TXP
#include "../include/CAN_Fuoco.h"
// #include "./STM32_CAN/STM32_CAN.h"
// #include "AS5600.h"
// #include "encoders/as5048a/AS5048A.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
// #include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"
//#include <pinout.h>
// STM32_CAN Can1( PA11, PB9 );

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);
SimpleCan can1(/*terminateTransceiver:*/true);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
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

float get_vbus(void)
{
  return _readADCVoltageLowSide(A_VBUS, current_sense.params) * 9.2;
}

// void onMotor(char* cmd){ 
//   float new_speed = 0;
//   commander.scalar(&new_speed, cmd);

  // motor.target = float(new_speed);
  // last_receive_timer = millis();


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
CANFuocoMotorConfig motor_config = {.motor_id = 4,
    // .get_supply_voltage = get_vbus,
    .motor = motor,
};


bool test_motor = true, use = true;
uint8_t motor_id = 0;
void setup() {
  // Wire.setClock(1000000);
  // Serial.begin(115200);
  // new_sensor.closeTransactions = true;
  // new_sensor.useHysteresis = false;
  // SPIClass mS = SPIClass(pinNametoDigitalPin(PB_5), pinNametoDigitalPin(PC_11), pinNametoDigitalPin(PC_10), pinNametoDigitalPin(PB_4));
  


  // new_sensor.init(&mS);
  // mS.begin();
  	// hspi = new SPIClass(); // Original ESP32 has HSPI and VSPI available for use
	// hspi->begin(pinNametoDigitalPin(PB_5), pinNametoDigitalPin(PC_11), pinNametoDigitalPin(PC_10)); // Assign the pins
  // SPI3
  // new_sensor.spi_mode = SPI_MODE1;
  // new_sensor.clock_speed = 500000;
  
  // new_sensor.init();
  // motor.useMonitoring(Serial);
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.voltage_limit = 4;
  driver.pwm_frequency = 20000;
  driver.init();

  // Can1.begin();
  // Can1.setFrameFormat(STM32_CAN::FRAME_FORMAT::CLASSIC);
  // Can1.setBaudRate(250000, 250000);
  new_sensor.init(&SPI_3);
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
motor.PID_current_d.I = 3;
motor.PID_current_d.D = 0;
motor.LPF_current_d = 0.002f;

motor.PID_current_q.P = 3;
motor.PID_current_q.I = 3;
motor.PID_current_q.D = 0;
motor.LPF_current_q = 0.002f;

  // open loop control config
  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.3;
  // motor.PID_velocity.output_ramp = 1000;
  // motor.motion_downsample = 0;

  // init motor hardware
motor.linkSensor(&new_sensor);
//A - is the ID of drive
// commander.add('Z', START_STOP, "motor");
motor.linkCurrentSense(&current_sense);
motor.sensor_direction= Direction::CCW;

motor.init();
current_sense.init();
// motor.zero_electric_angle = 0;


use = false;
test_motor = true;
uint8_t motor_num = 1, robot = 1;
motor.initFOC();
pinMode(pinNametoDigitalPin(PC_6), OUTPUT);
pinMode(pinNametoDigitalPin(PB_13), OUTPUT);
pinMode(pinNametoDigitalPin(PB_11), OUTPUT);
// pinMode(PB14, INPUT_ANALOG);
// pinMode(PA4, INPUT_ANALOG);
pinMode(PC14, INPUT_PULLUP);
pinMode(PC15, INPUT_PULLUP);
pinMode(PB3, INPUT_PULLUP);
uint8_t A0 = (~digitalRead(PC14) & 0x01)<< 1;
uint8_t A1 = (~digitalRead(PC15) & 0x01) << 0;
uint8_t A2 = (~digitalRead(PB3) & 0x01) << 2;
motor_id = A0 + A1 + A2;

uint8_t lol_id =  motor_id & 0xFF;
// digitalPin
digitalWrite(pinNametoDigitalPin(PC_6), HIGH);
digitalWrite(pinNametoDigitalPin(PB_13), HIGH);
digitalWrite(pinNametoDigitalPin(PB_11), HIGH);
// delay()

motor_config.motor_id = motor_id;
motor.target = 0;
init_CAN();
}


CANFuoco can_fuoco(motor_config);

static void init_CAN()
{
	// Serial1.println(
  can1.init(CanSpeed::Kbit250);
    //  == HAL_OK
	// 	? "CAN: initialized."
	// 	: "CAN: error when initializing.");

	FDCAN_FilterTypeDef sFilterConfig;

	// Configure Rx filter
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	sFilterConfig.FilterID1 = 0x321;
	sFilterConfig.FilterID2 = 0x7FF;

	can1.configFilter(&sFilterConfig);
	can1.configGlobalFilter(FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	can1.activateNotification(&can1RxHandler);
	
	// Serial1.println(
  can1.start();
  //  == HAL_OK
	// 	? "CAN: started."
	// 	: "CAN: error when starting.");
}
uint32_t color = 0;
bool start = false;
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
	// if ((rxHeader.Identifier != 0x321) || (rxHeader.IdType != FDCAN_STANDARD_ID) || (rxHeader.DataLength != FDCAN_DLC_BYTES_2))
  if(rxHeader.Identifier == 0x040)
	{
    digitalToggle(LED_BUILTIN);    
    color += 1;
    // rxData[2] = 0;
    // rxData[3] = 0x3C;
    
    // uint8_t arr[8] = { 0, 0x3C, 0, 0x3C, 1, 1, 1, 1};
    // if (color<=300)
    // {
    //   arr[3] = 0xCD;
    // }
    // else if (color > 600)
    //   color = 0;
    // uint8_t* lolarr =  arr;
    can_fuoco.can_rx_callback(CANFuocoRegisterMap::MULTI_TARGET_W, 8, rxData);
    // motor.move(can_fuoco.speed);
    // if (!start){
    //   start = true;
    // }
	}
  
	// return
}

uint32_t timer1 = 0, timer2 = 0;
int8_t flag = 0;
uint32_t save_com = 0;
float test_float = 0;
double b14=0, a0=0, a4=0;

void loop() {
  motor.loopFOC();
      // motor.move(can_fuoco.speed);
  // motor.loopFOC();
  // b14 = analogRead(PB14);
  // b14 = _readADCVoltageLowSide(A_VBUS, current_sense.params) * 9.2;
  // double analog = _readADCVoltageLowSide(A_TEMPERATURE, current_sense.params);
  // analog = ((float)((1 << 12) - 1) / ((4096.0f/3.3f)*analog) - 1.0f) / 2.12;
  // analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);
  // a0 = 1.0f / analog - 273.15f;
  // analog = _readADCVoltageLowSide(PA4, current_sense.params);
  // analog = ((float)((1 << 12) - 1) / ((4096.0f/3.3f)*analog) - 1.0f) / 2.12;
  // analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);
  // a4 = 1.0f / analog - 273.15f;
  // // analog =  4700 * (4096 / ((4096/3.3) * analog) - 1.0);
  // // // analog = 0.47 / ((float)((1 << 12) - 1) / analog - 1.0f);
  // // analog = 1.0f/(298.15) + (1.0f/3380) * (log(analog/10000));// + 1.0f / (25 + 273.15f);
  // // a0 = 1.0f / analog - 273.15f;
  // // a0 = (1.0f / analog - 273.15f);
  // uint32_t tmp_an = a0 + b14 + a4;
  //   motor.current_limit = max_current;
  //   // motor.PID_velocity.P = 1;
  //   // motor.PID_velocity.I = 10;
  //   // motor.PID_velocity.D = 0; 
  //   motor.velocity_limit = 10000;
  //   // motor.PID_velocity.output_ramp = 1000;
  //   // motor.move(10);
  //   // static float recievedSpeed = 0;
  //   // // motor.move(3.14);
    // if(millis() - timer1 >= 3000)
    // {
    //   flag += 1;
    //   if (flag >= 3)
    //     flag = 0;
    //   timer1 = millis();
    // }
    // if (flag == 0)
    // {
    //     motor.move(50);
    // }
    // else if (flag == 1)
    // {
    //   motor.move(0);
    // }
    // uint8_t arr[8] = { 0, 0x3C, 0, 0x3C, 1, 1, 1, 1};
    // uint8_t* lolarr =  arr;
    // ushort *target_data = reinterpret_cast<ushort *>(lolarr);
    // test_float = half_to_float(target_data[motor_id - 1]);

    digitalToggle(PB11);    
    // int* ptr = arr;
    // can_fuoco.can_rx_callback(CANFuocoRegisterMap::MULTI_TARGET_W, 8, )
    // else
    // {
      motor.move();
    // }
    // // motor.monitor();
    // if (millis() - timer2 >= 1000)
    // {
    //   save_com = color;
    //   color = 0;
    //   timer2 = millis();
    // }
    // uint8_t arr[8] = { 0, 0x3C, 0, 0x3C, 1, 1, 1, 1};
    // uint8_t* lolarr =  arr;
    // can_fuoco.can_rx_callback(CANFuocoRegisterMap::MULTI_TARGET_W, 8, lolarr);
  //   if (Can1.read(CAN_RX_msg) ) {
  //     // Serial.print("Channel:");
  //     // Serial.print(CAN_RX_msg.bus);
  //     if (CAN_RX_msg.flags.extended == false) {
  //       // Serial.print(" Standard ID:");
  //     }
  //     else {
  //       // Serial.print(" Extended ID:");
  //     }
  //     // Serial.print(CAN_RX_msg.id, HEX);

  // #if defined(HAL_FDCAN_MODULE_ENABLED)
  //     if(CAN_RX_msg.flags.fd_rateswitch) {
  //       // Serial.println(" [FD-BRS]");
  //     }
  //     else if(CAN_RX_msg.flags.fd_frame) {
  //       // Serial.println(" [FD]");
  //     }
  // #endif

  //     // Serial.print(" DLC: ");
  //     // Serial.print(CAN_RX_msg.len);
  //     if (CAN_RX_msg.flags.remote == false) {
  //       // Serial.print(" buf: ");
  //       for(int i=0; i<CAN_RX_msg.len; i++) {
  //         // Serial.print("0x"); 
  //         // Serial.print(CAN_RX_msg.buf[i], HEX); 
  //         // if (i != (CAN_RX_msg.len-1))  Serial.print(" ");
  //       }
  //       // Serial.println();
  //     } else {
  //       // Serial.println(" Data: REMOTE REQUEST FRAME");
  //     }
  // }
  // delay(1);
}