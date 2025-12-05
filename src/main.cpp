#include "SimpleFOCDrivers.h"
#include <SimpleFOC.h>
#include <SPI.h>
#include <SimpleCan.h>
#define SPI_SR_TXP
#include "../include/CAN_Fuoco.h"
// #include "encoders/as5048a/AS5048A.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"
// #include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);
SimpleCan can1(/*terminateTransceiver:*/true);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];



float max_current = 1.0;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(7, 3.6, 82.5, 0.0021); 
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15); 
SPIClass SPI_3(PB5, A_CAN_SHDN, A_BUTTON);
MagneticSensorAS5048A new_sensor = MagneticSensorAS5048A(pinNametoDigitalPin(PA_15));
// MXLEMMINGObserverSensor new_sensor = MXLEMMINGObserverSensor(motor);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
// Commander commander = Commander(Serial, 'e', false);
double sen_angle = 0;
int8_t sen_dir = 0;
bool motor_callibrated = false;
bool motor_disabled = false;

    
CANFuocoMotorConfig motor_config = {.motor_id = 2,
    // .get_supply_voltage = get_vbus,
    .motor = motor,
};


bool test_motor = true, use = true;
uint8_t motor_id = 0;
void setup() {

  // Serial.begin(115200);
  // new_sensor.closeTransactions = true;
  // new_sensor.useHysteresis = false;
  
  // new_sensor.init();
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.voltage_limit = 8;
  driver.pwm_frequency = 20000;
  driver.init();
  new_sensor.init(&SPI_3);
  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
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

motor.controller = MotionControlType::velocity;
motor.LPF_velocity = 0.3;
motor.linkSensor(&new_sensor);
motor.linkCurrentSense(&current_sense);
motor.sensor_direction= Direction::CCW;

motor.init();
current_sense.init();


use = false;
test_motor = true;
uint8_t motor_num = 1, robot = 1;
motor.initFOC();
pinMode(pinNametoDigitalPin(PC_6), OUTPUT);
pinMode(pinNametoDigitalPin(PB_13), OUTPUT);
pinMode(pinNametoDigitalPin(PB_11), OUTPUT);
pinMode(PC14, INPUT_PULLUP);
pinMode(PC15, INPUT_PULLUP);
pinMode(PB3, INPUT_PULLUP);
uint8_t A0 = (~digitalRead(PC14) & 0x01)<< 1;
uint8_t A1 = (~digitalRead(PC15) & 0x01) << 0;
uint8_t A2 = (~digitalRead(PB3) & 0x01) << 2;
motor_id = A0 + A1 + A2;

uint8_t lol_id =  motor_id & 0xFF;
digitalWrite(pinNametoDigitalPin(PC_6), HIGH);
digitalWrite(pinNametoDigitalPin(PB_13), LOW);
digitalWrite(pinNametoDigitalPin(PB_11), HIGH);

motor_config.motor_id = motor_id;
motor.target = 0;
init_CAN();
}


CANFuoco can_fuoco(motor_config);

static void init_CAN()
{
  can1.init(CanSpeed::Kbit250);

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
	
  can1.start();
}
uint32_t color = 0;
bool start = false;
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
  
  if (rxHeader.Identifier == 0x40)
  {
    // digitalToggle(PB13);    
    color += 1;
    if (color % 100 == 0)
      digitalToggle(PB13);    
    can_fuoco.can_rx_callback(rxHeader.Identifier, 8, rxData);
  }
}

void send_message()
{
  TxHeader.Identifier = 0x321;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxData[0] = 0x13;
	TxData[1] = 0xAD;

	can1.addMessageToTxFifoQ(&TxHeader, TxData);
}

double get_motor_temp()
{
  double analog = _readADCVoltageLowSide(PA4, current_sense.params);
  analog = ((float)((1 << 12) - 1) / ((4096.0f/3.3f)*analog) - 1.0f) / 2.12;
  analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);
  return 1.0f / analog - 273.15f;
}

float get_vbus(void)
{
  return _readADCVoltageLowSide(A_VBUS, current_sense.params) * 9.2;
}

float get_pcm_temp()
{
  double analog = _readADCVoltageLowSide(A_TEMPERATURE, current_sense.params);
  analog = ((float)((1 << 12) - 1) / ((4096.0f/3.3f)*analog) - 1.0f) / 2.12;
  analog = (log(analog) / 3900.0f) + 1.0f / (25.0f + 273.15f);
  return 1.0f / analog - 273.15f;
}


// uint32_t timer2 = 0;
// int package_recived_per_second()
// {
//   if (millis() - timer2 >= 1000)
//     {
//       save_com = color;
//       color = 0;
//       timer2 = millis();
//     }
//     return save_com;
// }
// uint32_t timer2 = 0;
// int package_recived_per_second()
// {
//   if (millis() - timer2 >= 1000)
//     {
//       save_com = color;
//       color = 0;
//       timer2 = millis();
//     }
//     return save_com;
// }

uint32_t timer1 = 0;
int8_t flag = 0;
void test_motor_tuda_syda(float speed_forward, float speed_backward, uint32_t time_difference)
{
  if(millis() - timer1 >= time_difference)
  {
    flag += 1;
    if (flag >= 3)
      flag = 0;
    timer1 = millis();
  }
  if (flag == 0)
  {
      motor.move(speed_forward);
  }
  else if (flag == 1)
  {
    motor.move(speed_backward);
  }
}

uint32_t save_com = 0;
float test_float = 0;
double b14=0, a0=0, a4=0;
bool error = false;
void loop() {
  if (get_motor_temp() < 50.0)
  {
    motor.loopFOC();
    motor.move();
  }
  else
  {
    error = true;
    motor.target = 0;
    motor.move();  
    motor.disable();
  }
  // if (error)
  // {
  //   digitalWrite(PC6, HIGH);   
  //   digitalWrite(PB11, LOW);   
  // }
  // else
  // {
  //   digitalWrite(PC6, LOW);   
  //   // digitalWrite(PB13, LOW);
  //   digitalWrite(PB11, HIGH);   
  // }
  // motor.loopFOC();
  //   // motor.PID_velocity.output_ramp = 1000;
    
    // uint8_t arr[8] = { 0, 0x3C, 0, 0x3C, 1, 1, 1, 1};
    // uint8_t* lolarr =  arr;
    // ushort *target_data = reinterpret_cast<ushort *>(lolarr);
    // test_float = half_to_float(target_data[motor_id - 1]);   
}