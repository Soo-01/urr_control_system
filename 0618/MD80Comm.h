#pragma once
/**
 * MD80Comm.h - MAB Robotics MD80 Motor Driver Communication
 * ============================================================================
 * CAN2.0 Register-based protocol (from Candleduino library)
 * - Frame Type 0x44 = Write Register
 * - Frame Type 0x43 = Read Register
 * - Standard CAN 8-byte frame
 * - Baudrate: 1Mbps (CAN2.0)
 * ============================================================================
 */

#include <Arduino.h>
#include <Arduino.h>
#include <cstring>

// ─── MD80 Frame Types (CAN2.0) ───
#define MD80_FRAME_WRITE_REGISTER       0x44
#define MD80_FRAME_READ_REGISTER        0x43
#define MD80_FRAME_WRITE_REGISTER_FD    0x42
#define MD80_FRAME_READ_REGISTER_FD     0x41

// ─── MD80 Register IDs ───
#define MD80_REG_CONTROL_WORD           0x0142
#define MD80_REG_MOTION_MODE_COMMAND    0x0140
#define MD80_REG_MOTION_MODE_STATUS     0x0141
#define MD80_REG_TARGET_POS             0x0150
#define MD80_REG_TARGET_VEL             0x0151
#define MD80_REG_TARGET_TORQUE          0x0152
#define MD80_REG_MAIN_ENCODER_POS       0x0063
#define MD80_REG_MAIN_ENCODER_VEL       0x0062
#define MD80_REG_OUTPUT_ENCODER_POS     0x0065
#define MD80_REG_OUTPUT_ENCODER_VEL     0x0064
#define MD80_REG_TORQUE                 0x0066
#define MD80_REG_ZERO                   0x008C
#define MD80_REG_BLINK                  0x008B
#define MD80_REG_RESET                  0x0088
#define MD80_REG_SAVE                   0x0080
#define MD80_REG_CLEAR_ERRORS           0x008A
#define MD80_REG_IMPEDANCE_KP           0x0200
#define MD80_REG_IMPEDANCE_KD           0x0201
#define MD80_REG_MOSFET_TEMPERATURE     0x0806

// ─── Control Word Values ───
#define MD80_CONTROL_WORD_ENABLE        0x27
#define MD80_CONTROL_WORD_DISABLE       0x40

// ─── Motion Modes ───
enum MD80MotionMode : uint8_t {
    MD80_MODE_IDLE      = 0,
    MD80_MODE_POSITION  = 1,
    MD80_MODE_VELOCITY  = 2,
    MD80_MODE_RAW_TORQUE = 3,
    MD80_MODE_IMPEDANCE = 4,
};

// ─── Motor State ───
struct MD80State {
    float position;     // rad
    float velocity;     // rad/s
    float torque;       // Nm
    bool  valid;
};

// ─── MD80 Communication Class ───
class MD80Comm {
public:
    MD80Comm();

    void setMotorID(uint32_t id) { motor_id_ = id; }
    uint32_t getMotorID() const { return motor_id_; }

    bool openCAN();

    // ─── Low-level register access ───
    bool writeRegisterU8(uint16_t regID, uint8_t value);
    bool writeRegisterFloat(uint16_t regID, float value);
    bool readRegisterFloat(uint16_t regID, float& value);

    // ─── High-level commands ───
    bool enable();
    bool disable();
    bool zero();
    bool blink();
    bool clearErrors();

    bool setMotionMode(MD80MotionMode mode);
    bool setTargetTorque(float torque_Nm);
    bool setTargetPosition(float pos_rad);
    bool setTargetVelocity(float vel_rads);
    bool setImpedanceParams(float kp, float kd);

    bool getPosition(float& pos_rad);
    bool getVelocity(float& vel_rads);
    bool getTorque(float& torque_Nm);

    // ─── Combined: send torque + read state in one cycle ───
    bool sendTorqueAndReadState(float torque_Nm, MD80State& state);

private:
    uint32_t motor_id_;

    // Core transact: send 8-byte frame, wait for reply, validate header
    // bool transact(uint8_t txBuf[8], uint8_t rxBuf[8], uint32_t timeout_us = 2000);
    bool transact(uint8_t* txBuf, uint8_t* rxBuf, uint32_t timeout_us = 2000);
};
