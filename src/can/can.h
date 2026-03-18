#pragma once

#include <SimpleCan.h>
#include <SimpleFOC.h>

#include "CAN_Fuoco.h"

namespace hw
{
extern BLDCMotor motor;
}

extern uint8_t motor_id;

namespace can
{

static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData);
static void init_CAN(void);
SimpleCan can1(/*terminateTransceiver:*/ true);
SimpleCan::RxHandler can1RxHandler(8, handleCanMessage);
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];

CANFuocoMotorConfig motor_config = {
    .motor_id = motor_id,
    // .get_supply_voltage = get_vbus,
    .motor = hw::motor,
};

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
    can1.configGlobalFilter(FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                            FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    can1.activateNotification(&can1RxHandler);

    can1.start();
}

// uint32_t color = 0;
static void handleCanMessage(FDCAN_RxHeaderTypeDef rxHeader, uint8_t *rxData)
{
    if ((rxHeader.Identifier >> 8) == motor_config.motor_id || (rxHeader.Identifier >> 8) == 0)
    {
        // digitalToggle(PB13);
        // color += 1;
        // if (color % 100 == 0)
        //     digitalToggle(PB13);
        can_fuoco.can_rx_callback(rxHeader.Identifier & 0xFF, rxHeader.DataLength, rxData);
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

void init()
{
    init_CAN();
}

};  // namespace can