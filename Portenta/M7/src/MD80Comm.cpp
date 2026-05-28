#include "MD80Comm.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

MD80Comm::MD80Comm() : motor_id_(0x64) {}

bool MD80Comm::openCAN() {
    __HAL_RCC_FDCAN_CLK_ENABLE(); 
    __HAL_RCC_GPIOB_CLK_ENABLE(); 
    __HAL_RCC_GPIOH_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8; 
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL; 
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1; 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_13; 
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    hfdcan1.Instance = FDCAN1; 
    hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL; 
    hfdcan1.Init.AutoRetransmission = DISABLE;
    hfdcan1.Init.TransmitPause = DISABLE; 
    hfdcan1.Init.ProtocolException = DISABLE;

    uint32_t fdcan_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    uint32_t nom_tq = fdcan_clk / 1000000; uint32_t nom_pre = 1;
    while(nom_tq > 120) { nom_pre++; nom_tq = fdcan_clk / (1000000 * nom_pre); }
    hfdcan1.Init.NominalPrescaler = nom_pre; 
    hfdcan1.Init.NominalTimeSeg1 = (nom_tq * 8) / 10 - 1; 
    hfdcan1.Init.NominalTimeSeg2 = nom_tq - 1 - hfdcan1.Init.NominalTimeSeg1; 
    hfdcan1.Init.NominalSyncJumpWidth = 1;

    uint32_t data_tq = fdcan_clk / 5000000; uint32_t data_pre = 1;
    while(data_tq > 24) { data_pre++; data_tq = fdcan_clk / (5000000 * data_pre); }
    hfdcan1.Init.DataPrescaler = data_pre; 
    hfdcan1.Init.DataTimeSeg1 = (data_tq * 8) / 10 - 1;
    hfdcan1.Init.DataTimeSeg2 = data_tq - 1 - hfdcan1.Init.DataTimeSeg1; 
    hfdcan1.Init.DataSyncJumpWidth = 1;

    hfdcan1.Init.MessageRAMOffset = 0; 
    hfdcan1.Init.StdFiltersNbr = 0; 
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 16; 
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0; 
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 16; 
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION; 
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) return false;
    
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, data_pre * (hfdcan1.Init.DataTimeSeg1 + 1), 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1); 
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return false;

    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME; 
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; 
    TxHeader.BitRateSwitch = FDCAN_BRS_ON; 
    TxHeader.FDFormat = FDCAN_FD_CAN; 
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; 
    TxHeader.MessageMarker = 0;

    return true;
}

// ────────────────────────────────────────────────
// Core transact: send 8 bytes, wait for matching reply
// ────────────────────────────────────────────────
bool MD80Comm::transact(uint8_t txBuf[8], uint8_t rxBuf[8], uint32_t timeout_us) {
    TxHeader.Identifier = motor_id_;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) {
        return false;
    }

    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                // Check if frame type and reg ID match
                if (RxData[0] == txBuf[0] && RxData[2] == txBuf[2] && RxData[3] == txBuf[3]) {
                    memcpy(rxBuf, RxData, 8);
                    return true;
                }
            }
        }
    }
    return false;
}

// ────────────────────────────────────────────────
// Register access (CAN2.0/FDCAN mode)
// ────────────────────────────────────────────────
bool MD80Comm::writeRegisterU8(uint16_t regID, uint8_t value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t)); 
    txBuf[4] = value;
    bool success = transact(txBuf, rxBuf, 2000);
    delayMicroseconds(150); // 황금 딜레이 복구
    return success;
}

bool MD80Comm::writeRegisterFloat(uint16_t regID, float value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_WRITE_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t)); 
    memcpy(&txBuf[4], &value, sizeof(float));   
    bool success = transact(txBuf, rxBuf, 2000);
    delayMicroseconds(150); // 황금 딜레이 복구
    return success;
}

bool MD80Comm::readRegisterFloat(uint16_t regID, float& value) {
    uint8_t txBuf[8] = {0};
    uint8_t rxBuf[8] = {0};
    txBuf[0] = MD80_FRAME_READ_REGISTER_FD;
    txBuf[1] = 0x00;
    memcpy(&txBuf[2], &regID, sizeof(uint16_t)); 
    
    if (!transact(txBuf, rxBuf, 2000)) {
        return false;
    }
    
    memcpy(&value, &rxBuf[4], sizeof(float));
    delayMicroseconds(150); // 황금 딜레이 복구
    return true;
}

// ────────────────────────────────────────────────
// High-level commands
// ────────────────────────────────────────────────
bool MD80Comm::enable() {
    return writeRegisterU8(MD80_REG_CONTROL_WORD, MD80_CONTROL_WORD_ENABLE);
}

bool MD80Comm::disable() {
    return writeRegisterU8(MD80_REG_CONTROL_WORD, MD80_CONTROL_WORD_DISABLE);
}

bool MD80Comm::zero() {
    return writeRegisterU8(MD80_REG_ZERO, 0x01);
}

bool MD80Comm::blink() {
    return writeRegisterU8(MD80_REG_BLINK, 0x01);
}

bool MD80Comm::clearErrors() {
    return writeRegisterU8(MD80_REG_CLEAR_ERRORS, 0x01);
}

bool MD80Comm::setMotionMode(MD80MotionMode mode) {
    return writeRegisterU8(MD80_REG_MOTION_MODE_COMMAND, static_cast<uint8_t>(mode));
}

bool MD80Comm::setTargetTorque(float torque_Nm) {
    return writeRegisterFloat(MD80_REG_TARGET_TORQUE, torque_Nm);
}

bool MD80Comm::setTargetPosition(float pos_rad) {
    return writeRegisterFloat(MD80_REG_TARGET_POS, pos_rad);
}

bool MD80Comm::setTargetVelocity(float vel_rads) {
    return writeRegisterFloat(MD80_REG_TARGET_VEL, vel_rads);
}

bool MD80Comm::setImpedanceParams(float kp, float kd) {
    if (!writeRegisterFloat(MD80_REG_IMPEDANCE_KP, kp)) return false;
    if (!writeRegisterFloat(MD80_REG_IMPEDANCE_KD, kd)) return false;
    return true;
}

bool MD80Comm::getPosition(float& pos_rad) {
    return readRegisterFloat(MD80_REG_MAIN_ENCODER_POS, pos_rad);
}

bool MD80Comm::getVelocity(float& vel_rads) {
    return readRegisterFloat(MD80_REG_MAIN_ENCODER_VEL, vel_rads);
}

bool MD80Comm::getTorque(float& torque_Nm) {
    return readRegisterFloat(MD80_REG_TORQUE, torque_Nm);
}

bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
    state.valid = false;
    if (!setTargetTorque(torque_Nm)) return false;
    if (!getPosition(state.position)) state.position = 0.0f;
    if (!getVelocity(state.velocity)) state.velocity = 0.0f;
    if (!getTorque(state.torque)) state.torque = 0.0f;
    state.valid = true;
    return true;
}
