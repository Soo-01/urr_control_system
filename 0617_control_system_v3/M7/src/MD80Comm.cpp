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
// bool MD80Comm::transact(uint8_t txBuf[8], uint8_t rxBuf[8], uint32_t timeout_us) {
//     TxHeader.Identifier = motor_id_;
//     if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) {
//         return false;
//     }

//     uint32_t start = micros();
//     while ((uint32_t)(micros() - start) < timeout_us) {
//         if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
//             if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
//                 // Check if frame type and reg ID match
//                 if (RxData[0] == txBuf[0] && RxData[2] == txBuf[2] && RxData[3] == txBuf[3]) {
//                     memcpy(rxBuf, RxData, 8);
//                     return true;
//                 }
//             }
//         }
//     }
//     return false;
// }


bool MD80Comm::transact(uint8_t txBuf[8], uint8_t rxBuf[8], uint32_t timeout_us) {
    // 1. [핵심 1: 버퍼 청소] 이전에 늦게 도착한 쓰레기 데이터가 남아있다면 싹 비웁니다 (발산 방지)
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
        HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);
    }

    // 2. 명령 전송
    TxHeader.Identifier = motor_id_;
    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, txBuf) != HAL_OK) {
        return false;
    }

    // 3. 응답 대기
    uint32_t start = micros();
    while ((uint32_t)(micros() - start) < timeout_us) {
        if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
            if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
                
                // 모터가 내 질문에 제대로 대답했는지 데이터 구조 확인
                if (RxData[0] == txBuf[0] && RxData[2] == txBuf[2] && RxData[3] == txBuf[3]) {
                    memcpy(rxBuf, RxData, 8);
                    
                    // 4. [핵심 2: 숨 고르기] 모터 칩셋이 다음 명령을 처리할 수 있도록 휴식 부여
                    // 이 60us가 타임아웃(1000us 허비)을 막아주어 전체 속도를 2배 이상 끌어올립니다!
                    delayMicroseconds(150); 
                    
                    return true;
                }
            }
        }
    }
    return false; // 타임아웃 발생 시
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
    // delayMicroseconds(150); // 황금 딜레이 복구
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
    // delayMicroseconds(150); // 황금 딜레이 복구
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
    // delayMicroseconds(150); // 황금 딜레이 복구
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

// bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
//     state.valid = false;
//     if (!setTargetTorque(torque_Nm)) return false;
//     if (!getPosition(state.position)) state.position = 0.0f;
//     if (!getVelocity(state.velocity)) state.velocity = 0.0f;
//     if (!getTorque(state.torque)) state.torque = 0.0f;
//     state.valid = true;
//     return true;
// }

// // MD80Comm.cpp 내부
// bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
//     uint8_t txBuf[16] = {0}; // CAN FD 확장 페이로드 사용 (MD80 프로토콜 규격에 맞게 크기 조절)
//     uint8_t rxBuf[16] = {0};
//     state.valid = false;

//     // 1. MAB 고속 제어 프레임 생성 (예: Frame Type 0x14 또는 특정 Command ID)
//     // CAN Header를 레지스터 모드(8바이트)가 아닌 FDCAN 모드(예: 16바이트 이상)로 세팅해야 합니다.
//     TxHeader.DataLength = FDCAN_DLC_BYTES_16; 
    
//     // 타겟 토크를 float에서 byte 배열로 변환하여 패킹
//     // (제어 파라미터 Kp, Kd 등도 이 배열에 함께 패킹해서 보냅니다)
//     txBuf[0] = 0x80; // 고속 제어 프레임 식별자 (MAB 매뉴얼 참조)
//     memcpy(&txBuf[1], &torque_Nm, sizeof(float)); 
//     // ... 나머지 제어 패킷 패킹 ...

//     // 2. 단 1번의 Transact로 송신 및 수신
//     if (!transact(txBuf, rxBuf, 1000)) { 
//         return false; 
//     }

//     // 3. 수신된 1개의 프레임에서 모든 상태값 언패킹 (Unpacking)
//     // 수신된 rxBuf 내에 위치, 속도, 토크가 모두 압축되어 들어옵니다.
//     memcpy(&state.position, &rxBuf[1], sizeof(float));
//     memcpy(&state.velocity, &rxBuf[5], sizeof(float));
//     memcpy(&state.torque,   &rxBuf[9], sizeof(float));
    
//     state.valid = true;
    
//     // 트랜잭션이 1회로 줄었으므로 딜레이는 최소화(예: 20us) 또는 제거 가능합니다.
//     delayMicroseconds(20); 

//     // 헤더를 원래대로 복구 (다른 레지스터 접근 함수들을 위해)
//     TxHeader.DataLength = FDCAN_DLC_BYTES_8; 
    
//     return true;
// }


// [수정할 부분: MD80Comm.cpp 맨 아래의 sendTorqueAndReadState 함수]
// bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
//     uint8_t txBuf[16] = {0}; // CAN FD 확장을 위한 16바이트 페이로드
//     uint8_t rxBuf[16] = {0};
//     state.valid = false;

//     // 1. 헤더 길이를 FDCAN 고속 제어 모드(16바이트)로 변경
//     TxHeader.DataLength = FDCAN_DLC_BYTES_16; 
    
//     // 2. MAB Robotics FDCAN 고속 제어 패킹 (명령 + 토크값)
//     // (※ 참고: 0x14는 MAB Candle 프로토콜의 일반적인 Fast Control Command 입니다. 펌웨어에 따라 0x80 등 다를 수 있음)
//     txBuf[0] = 0x14; 
//     memcpy(&txBuf[1], &torque_Nm, sizeof(float)); 
//     // 필요한 경우 Kp, Kd 등을 txBuf[5], txBuf[9] 등에 추가 패킹 가능

//     // 3. 단 한 번의 통신으로 전송 및 응답 대기 (timeout 1000us)
//     if (!transact(txBuf, rxBuf, 1000)) { 
//         TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 실패 시 안전을 위해 원래대로 복구
//         return false; 
//     }

//     // 4. 수신된 1개의 프레임에서 모든 상태값 언패킹 (Unpacking)
//     // 모터가 Pos, Vel, Tau를 하나의 rxBuf에 압축해서 보내옵니다.
//     memcpy(&state.position, &rxBuf[1], sizeof(float));
//     memcpy(&state.velocity, &rxBuf[5], sizeof(float));
//     memcpy(&state.torque,   &rxBuf[9], sizeof(float));
    
//     state.valid = true;
    
//     // 통신이 4번에서 1번으로 줄었으므로 딜레이 최소화
//     delayMicroseconds(20); 

//     // 다른 레지스터 제어(Enable 등)를 위해 헤더를 8바이트로 원복
//     TxHeader.DataLength = FDCAN_DLC_BYTES_8; 
    
//     return true;
// }
bool MD80Comm::sendTorqueAndReadState(float torque_Nm, MD80State& state) {
    state.valid = false;

    // 모터가 확실하게 알아듣는 기존 8바이트 레지스터 명령 방식으로 원복
    // (이전 최적화의 교훈을 살려, 황금 딜레이(150us)는 빼고 꼭 필요한 3가지만 통신합니다)
    bool ok_torque = setTargetTorque(torque_Nm);
    delayMicroseconds(80); // [추가] 모터 칩셋 숨 고르기
    bool ok_pos    = getPosition(state.position);
    delayMicroseconds(80); // [추가] 모터 칩셋 숨 고르기
    bool ok_vel    = getVelocity(state.velocity);
    
    // 통신 횟수를 줄여 속도를 높이기 위해 토크 센서값은 읽지 않음(0 처리)
    state.torque   = 0.0f; 

    // 세 가지 명령이 모두 정상적으로 응답을 받았다면
    if (ok_torque && ok_pos && ok_vel) {
        state.valid = true;
        return true;
    }
    
    // 하나라도 응답이 없거나 타임아웃 났다면 실패
    return false;
}