#pragma once

#include <SimpleCan.h>
#include <SimpleFOC.h>

#include "CAN_Fuoco.h"

namespace hw
{
extern BLDCMotor motor;
float get_vbus(void);
}  // namespace hw

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
    .get_supply_voltage = hw::get_vbus,
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
    const uint32_t affected_id = rxHeader.Identifier >> 7;
    const uint32_t register_id = rxHeader.Identifier & 0x7F;
    if (affected_id == motor_config.motor_id || affected_id == 0)
    {
        can_fuoco.can_rx_callback(register_id, rxHeader.DataLength, rxData);

        uint8_t response_data[8] = {};
        const uint8_t response_length = can_fuoco.can_tx_callback(response_data);
        if (response_length == 0)
        {
            return;
        }

        FDCAN_TxHeaderTypeDef response_header = {};
        response_header.Identifier = (static_cast<uint32_t>(motor_config.motor_id) << 7) |
                                     static_cast<uint32_t>(can_fuoco.feedback_register());
        response_header.IdType = FDCAN_STANDARD_ID;
        response_header.TxFrameType = FDCAN_DATA_FRAME;
        response_header.DataLength = response_length == 1   ? FDCAN_DLC_BYTES_1
                                     : response_length == 2 ? FDCAN_DLC_BYTES_2
                                     : response_length == 3 ? FDCAN_DLC_BYTES_3
                                     : response_length == 4 ? FDCAN_DLC_BYTES_4
                                     : response_length == 5 ? FDCAN_DLC_BYTES_5
                                     : response_length == 6 ? FDCAN_DLC_BYTES_6
                                     : response_length == 7 ? FDCAN_DLC_BYTES_7
                                                            : FDCAN_DLC_BYTES_8;
        response_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        response_header.BitRateSwitch = FDCAN_BRS_OFF;
        response_header.FDFormat = FDCAN_CLASSIC_CAN;
        response_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        response_header.MessageMarker = 0;
        can1.addMessageToTxFifoQ(&response_header, response_data);
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
