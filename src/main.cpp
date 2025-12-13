#include "SimpleFOCDrivers.h"
#include <SimpleFOC.h>
#include "../include/CAN_Fuoco.h"
#include <SimpleCan.h>
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);
SimpleCan can1(/*terminateTransceiver:*/ true);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

#include "encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.h"

float max_current = 2.0;
// (Pole_pairs, resistance, kv, inductanse)
BLDCMotor motor = BLDCMotor(1, 0.94, 770, 0.104 * 0.001);
uint32_t last_receive_timer = 0;
BLDCDriver6PWM driver = BLDCDriver6PWM(PA8, PC13, PA9, PA12, PA10, PB15);

HallSensor new_sensor(PB6, PB7, PB8, 1);

// MXLEMMINGObserverSensor new_sensor = MXLEMMINGObserverSensor(motor);
LowsideCurrentSense current_sense = LowsideCurrentSense(0.003f, -9.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
double sen_angle = 0;
int8_t sen_dir = 0;
bool motor_callibrated = false;
bool motor_disabled = false;

uint8_t motor_id = 0;
CANFuocoMotorConfig motor_config = {
    .motor_id = motor_id,
    // .get_supply_voltage = get_vbus,
    .motor = motor,
};

bool test = true, use = false;

void setup()
{
  new_sensor.pullup = Pullup::USE_INTERN;
  new_sensor.init();
  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0;
  driver.voltage_limit = 12.0;
  driver.pwm_frequency = 14000;
  driver.init();

  motor.linkDriver(&driver);
  current_sense.skip_align = true;
  current_sense.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.monitor_variables = _MON_TARGET | _MON_ANGLE | _MON_VEL;
  motor.monitor_downsample = 300;
  // motor.LPF_current_d.Tf = 0.01;
  // motor.LPF_current_q.Tf = 0.01;
  // motor.voltage_limit = 12;   // [V]
  motor.current_limit = max_current;
  motor.velocity_limit = 10000; // [rad/s]
  // default P=0.5 I = 10 D = 0
  motor.PID_velocity.P = 1.5;
  motor.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0; // 0.001;//0.0035; //0.0035

  motor.PID_current_d.P = 0.7; // 2.75;
  motor.PID_current_d.I = 1;
  motor.PID_current_d.D = 0;
  motor.LPF_current_d = 0.002f;

  motor.PID_current_q.P = 0.7; // 2.75;
  motor.PID_current_q.I = 1;
  motor.PID_current_q.D = 0;
  motor.LPF_current_q = 0.002f;

  motor.controller = MotionControlType::velocity;
  motor.LPF_velocity = 0.05;
  // motor.PID_velocity.output_ramp = 1000;
  motor.motion_downsample = 0;
  // init motor hardware

  motor.linkSensor(&new_sensor);
  // A - is the ID of drive

  motor.linkCurrentSense(&current_sense);
  // motor.sensor_direction= Direction::UNKNOWN;
  new_sensor.electric_rotations = 0;
  motor.init();
  current_sense.init();

  use = false;
  uint8_t motor_num = 1, robot = 1;
  motor.initFOC();
  pinMode(pinNametoDigitalPin(PB_11), OUTPUT);
  pinMode(pinNametoDigitalPin(PB_13), OUTPUT);
  pinMode(pinNametoDigitalPin(PC_6), OUTPUT);
  pinMode(pinNametoDigitalPin(PB_13), OUTPUT);
  pinMode(pinNametoDigitalPin(PB_11), OUTPUT);
  pinMode(PC14, INPUT_PULLUP);
  pinMode(PC15, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP);
  uint8_t A0 = (~digitalRead(PC14) & 0x01) << 1;
  uint8_t A1 = (~digitalRead(PC15) & 0x01) << 0;
  uint8_t A2 = (~digitalRead(PB3) & 0x01) << 2;
  motor_id = A0 + A1 + A2;

  motor.initFOC();

  digitalWrite(pinNametoDigitalPin(PC_6), HIGH);
  digitalWrite(pinNametoDigitalPin(PB_13), LOW);
  digitalWrite(pinNametoDigitalPin(PB_11), HIGH);

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

  // if (rxHeader.Identifier == 0x40)
  // {
  // digitalToggle(PB13);
  if ((rxHeader.Identifier >> 8) == motor_id || (rxHeader.Identifier >> 8) == 0)
  {
    // digitalToggle(PB13);
    color += 1;
    if (color % 100 == 0)
      digitalToggle(PB13);
    can_fuoco.can_rx_callback(rxHeader.Identifier & 0xFF, rxHeader.DataLength, rxData);
  }
  // }
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
  analog = ((float)((1 << 12) - 1) / ((4096.0f / 3.3f) * analog) - 1.0f) / 2.12;
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
  analog = ((float)((1 << 12) - 1) / ((4096.0f / 3.3f) * analog) - 1.0f) / 2.12;
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

uint32_t timer1 = 0;
int8_t flag = 0;
uint8_t test_flag = 0;
void test_motor_tuda_syda(float speed_forward, float speed_backward, uint32_t time_difference)
{
  motor.loopFOC();
  if (millis() - timer1 >= time_difference)
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
double b14 = 0, a0 = 0, a4 = 0;
bool error = false;
void loop()
{
  if (motor_id != 5)
  {
    if (get_motor_temp() < 50.0 || get_pcm_temp() < 60.0)
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
    if (error)
    {
      digitalWrite(PC6, HIGH);
      digitalWrite(PB11, LOW);
    }
    else
    {
      digitalWrite(PC6, LOW);
      // digitalWrite(PB13, LOW);
      digitalWrite(PB11, HIGH);
    }
  }
  else
  {
    test_motor_tuda_syda(100, -200, 6000);
    color += 1;
    if (color % 100 == 0)
    {
      test_flag += 1;
    }
    if (test_flag == 0)
    {
      digitalWrite(PC6, LOW);
      digitalWrite(PB13, HIGH);
      digitalWrite(PB11, HIGH);
    }
    else if (test_flag == 1)
    {
      digitalWrite(PC6, HIGH);
      digitalWrite(PB13, LOW);
      digitalWrite(PB11, HIGH);
    }
    else
    {
      digitalWrite(PC6, HIGH);
      digitalWrite(PB13, HIGH);
      digitalWrite(PB11, LOW);
    }
    if (test_flag >= 3)
    {
      test_flag = 0;
    }
    
  }

  //   // motor.PID_velocity.output_ramp = 1000;

  // uint8_t arr[8] = { 0, 0x3C, 0, 0x3C, 1, 1, 1, 1};
  // uint8_t* lolarr =  arr;
  // ushort *target_data = reinterpret_cast<ushort *>(lolarr);
  // test_float = half_to_float(target_data[motor_id - 1]);
}