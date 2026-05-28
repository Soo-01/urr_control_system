/**
 * M7.ino - Portenta H7 M7 Core
 * ============================================================================
 * 재활 로봇 모터 제어 - 다중 모터 UDP 동시 제어 버전
 * 
 * - Jetson To M7 UDP 통신 도입 (Port 9999 수신)
 * - 4개 모터 동시 제어 배열 루프 (500Hz)
 * - 기존 모터 프로필(motor_profiles) 및 모드별 제어 알고리즘 완전 유지
 * - 제어 상태 변수(MotorState) 구조체화로 독립적 제어 보장
 * ============================================================================
 */

#include <Arduino.h>
#include <mbed.h>
#include <math.h>

#include <PortentaEthernet.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "src/MD80Comm.h"

// ============================================================================
// [UDP & Network Configuration]
// ============================================================================
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(172, 26, 178, 50); // RIGHT ARM (IP_RIGHT_ARM)
unsigned int localPort = 9999;
EthernetUDP Udp;

IPAddress jetson_ip;
uint16_t jetson_port = 0;
bool jetson_connected = false;

// ============================================================================
// [UDP Structures]
// ============================================================================
#pragma pack(push, 1)
enum ControlMode : uint8_t { 
    CMD_IDLE = 0, CMD_AROM = 1, CMD_PROM = 2, CMD_CPM = 3, CMD_ISOM = 4, CMD_ISOT = 5, CMD_FIX_POS = 6 
};

struct JointCmd {
    uint8_t mode;        
    float target_pos_min; 
    float target_pos_max; 
    float speed;         
    float resistance;    
};

struct JetsonToM7_UDP {
    uint8_t robot_state;
    uint32_t seq;
    JointCmd joints[4];
};

struct JointStateMsg {
    float pos;
    float vel;
    float tau_meas;
    float tau_cmd;
    uint8_t online;
};

struct M7ToJetson_UDP {
    uint8_t robot_state;
    uint32_t seq_echo;
    uint32_t m7_rx_to_tx_us;
    uint32_t fdcan_rtt_us;
    JointStateMsg joints[4];
};
#pragma pack(pop)

uint32_t last_seq_echo = 0;

// ============================================================================
// [Watchdog Shared Memory (SRAM4)]
// ============================================================================
struct WatchdogData {
    volatile uint32_t m7_heartbeat;
    volatile uint8_t motor_comm_failed; // 1 = 모터 통신 완전히 두절
    volatile uint8_t e_stop_triggered;
};
volatile WatchdogData* wd_data = (volatile WatchdogData*)0x38001000;

// ============================================================================
// [Motor Profiles Configuration]
// ============================================================================
struct MotorProfile {
    uint32_t id;
    String name;
    float dob_j;
    float dob_b;
    float cpm_kp;
    float cpm_kd;
    float gc_m;
    float gc_l;
    float max_torque;
};

// 모터 4개에 대한 프로필 정의
MotorProfile motor_profiles[4] = {
    // 1. AK80-64 (ID: 100) - 어깨 굴곡/신전
    { 100, "AK80-64", 0.01f, 0.05f, 100.0f, 1.5f, 0.0f, 0.0f, 30.0f },
    // 2. AK60-6 (ID: 110) - 어깨 내회전/외회전
    { 110, "AK60-6(1)", 0.002f, 0.01f, 10.0f, 0.5f, 0.0f, 0.0f, 10.0f },
    // 3. AK70-10 (ID: 120) - 팔꿈치 굴곡/신전
    { 120, "AK70-10", 0.004f, 0.011f, 15.0f, 1.0f, 0.0f, 0.0f, 15.0f },
    // 4. AK60-6 (ID: 130) - 손목 굴곡/신전
    { 130, "AK60-6(2)", 0.002f, 0.01f, 10.0f, 0.5f, 0.0f, 0.0f, 10.0f }
};

// ============================================================================
// [Global Configuration] 
// ============================================================================
// 4개 모터를 동시에 제어하기 위해 주기를 500Hz로 하향 (CAN2.0 대역폭 확보)
static const float LOOP_RATE_HZ = 500.0f; 
static const float DOB_Ts = 1.0f / LOOP_RATE_HZ;
static const float gc_G = 9.80665f;

static const float fc_vel = 50.0f;
static const float tau_vel = 1.0f / (2.0f * M_PI * fc_vel);
static const float alpha_vel = DOB_Ts / (tau_vel + DOB_Ts);

static const float fc_Q = 50.0f;
static const float tau_Q = 1.0f / (2.0f * M_PI * fc_Q);
static const float alpha_Q = DOB_Ts / (tau_Q + DOB_Ts);

// ============================================================================
// [Motor Communication & State]
// ============================================================================
MD80Comm motor;
bool motor_enabled = false;

enum class CPMState { BUILD, RUN, HOLD };
enum class AROMState { IDLE, RUNNING };
enum class IsometricState { IDLE, MOVING_TO_TARGET, HOLDING };

// 단일 변수로 존재하던 제어 상태를 배열(Struct)로 묶어 독립화
struct MotorState {
    uint8_t prev_mode = CMD_IDLE;

    float current_position = 0.0f;
    float current_velocity = 0.0f;
    float motor_output_torque = 0.0f;
    float real_torque = 0.0f;
    bool online = false;

    // DOB Variables
    float filtered_velocity = 0.0f;
    float prev_filtered_velocity = 0.0f;
    float acceleration = 0.0f;
    float tau_nominal_plant = 0.0f;
    float filtered_tau_nominal_plant = 0.0f;
    float estimated_disturbance = 0.0f;
    float target_torque = 0.0f;
    float prev_target_torque = 0.0f;
    float filtered_prev_target_torque = 0.0f;

    // Mode Flags
    bool g_cpmMode = false;
    bool g_promMode = false;
    bool g_aromMode = false;
    bool g_isometricMode = false;
    bool g_fixMode = false;

    // CPM Variables
    float P_MIN = 0.0f, P_MAX = 0.0f;
    float last_end = 0.0f;
    CPMState cpm_state = CPMState::BUILD;
    int cpm_dir = 1, cpm_idx = 0;
    uint32_t cpm_hold_ts = 0;
    bool cpm_inited = false;
    
    float* pos_trap_traj_buf;
    float* vel_trap_traj_buf;
    int trap_traj_buf_size = 0;
    float CPM_V_MAX = 0.3f;
    float CPM_T_ACC = 1000.0f; // ms

    // AROM Variables
    AROMState arom_state = AROMState::IDLE;
    float arom_max_position = 0.0f;
    float arom_min_position = 0.0f;
    float arom_init_position = 0.0f;

    // Isometric Variables
    IsometricState isom_state = IsometricState::IDLE;
    float isom_target_angle = 0.0f;
    uint32_t isom_hold_duration_ms = 0;
    uint32_t isom_hold_start_time = 0;

    // FIX_POS Variables
    float fix_target_pos = 0.0f;
};

MotorState m_states[4];
bool expected_online[4] = {false, false, false, false}; // 부팅(Enable) 시점에 연결이 확인된 모터들만 추적

// ============================================================================
// [Ticker for Loop]
// ============================================================================
mbed::Ticker t_control;
volatile bool tick_control = false;
void onControlTick() { tick_control = true; }

// ============================================================================
// [Helper Functions]
// ============================================================================
void resetDOB(int i) {
    MotorState& s = m_states[i];
    s.filtered_velocity = 0.0f;
    s.prev_filtered_velocity = 0.0f;
    s.acceleration = 0.0f;
    s.tau_nominal_plant = 0.0f;
    s.filtered_tau_nominal_plant = 0.0f;
    s.estimated_disturbance = 0.0f;
    s.prev_target_torque = 0.0f;
    s.filtered_prev_target_torque = 0.0f;
    s.target_torque = 0.0f;
    s.cpm_inited = false;
}

float clamp_torque(int i, float tau) {
    float max_t = motor_profiles[i].max_torque;
    if (tau > max_t) return max_t;
    if (tau < -max_t) return -max_t;
    return tau;
}

void readMotorState(int i) {
    MotorState& s = m_states[i];
    // 읽어온 값이 있다면 online 처리, 실패 시 이전 값 유지
    if (motor.getPosition(s.current_position) && 
        motor.getVelocity(s.current_velocity) && 
        motor.getTorque(s.real_torque)) {
        s.online = true;
    } else {
        s.online = false;
    }
}

// ============================================================================
// [Control: CPM]
// ============================================================================
void cpm_trapezoidal_profile(int i, float pos_min, float pos_max, float vel_max, float t_acc_ms) {
    MotorState& s = m_states[i];
    float sgn = (pos_max >= pos_min) ? 1.0f : -1.0f;
    vel_max = sgn * vel_max;
    float t_acc_s = t_acc_ms * 0.001f;
    float T_ms = t_acc_ms + (pos_max - pos_min) / vel_max * 1000.0f;
    float T_s = T_ms * 0.001f;
    float t_dcc_ms = T_ms - t_acc_ms;
    float t_dcc_s = t_dcc_ms * 0.001f;

    if (t_dcc_ms < t_acc_ms) t_dcc_ms = t_acc_ms;
    
    // LOOP_RATE_HZ(500Hz) 기반으로 버퍼 크기 계산
    s.trap_traj_buf_size = (int)(T_ms * (LOOP_RATE_HZ / 1000.0f)) + 1;
    if (s.trap_traj_buf_size > 10000) s.trap_traj_buf_size = 10000; // 버퍼 오버플로우 방지 (최대 20초 궤적)

    for (int t = 0; t < s.trap_traj_buf_size; t++) {
        float t_s = t * DOB_Ts;
        if (t_s <= t_acc_s) {
            s.pos_trap_traj_buf[t] = pos_min + 0.5f * vel_max / t_acc_s * t_s * t_s;
            s.vel_trap_traj_buf[t] = vel_max / t_acc_s * t_s;
        } else if (t_s <= t_dcc_s) {
            s.pos_trap_traj_buf[t] = (pos_min + 0.5f * vel_max * t_acc_s) + vel_max * (t_s - t_acc_s);
            s.vel_trap_traj_buf[t] = vel_max;
        } else {
            s.pos_trap_traj_buf[t] = ((pos_min + 0.5f * vel_max * t_acc_s) + vel_max * (t_dcc_s - t_acc_s)) 
                                    + vel_max / (t_dcc_s - T_s) * (0.5f * (t_s * t_s - t_dcc_s * t_dcc_s) - T_s * (t_s - t_dcc_s));
            s.vel_trap_traj_buf[t] = vel_max / (t_dcc_s - T_s) * (t_s - T_s);
        }
    }
}

float run_cpm(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    if (!s.cpm_inited) {
        s.filtered_velocity = 0.0f;
        s.prev_filtered_velocity = 0.0f;
        s.cpm_inited = true;
    } else {
        s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);
    }
    s.acceleration = (s.filtered_velocity - s.prev_filtered_velocity) / DOB_Ts;
    s.prev_filtered_velocity = s.filtered_velocity;

    float tau_g = p.gc_m * p.gc_l * gc_G * sinf(s.current_position);

    if (s.cpm_state == CPMState::BUILD) {
        float start = s.last_end;
        float end = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
        s.cpm_idx = 0;
        cpm_trapezoidal_profile(i, start, end, s.CPM_V_MAX, s.CPM_T_ACC);
        s.cpm_state = CPMState::RUN;
        return 0.0f;
    }
    else if (s.cpm_state == CPMState::RUN) {
        if (s.cpm_idx < s.trap_traj_buf_size) {
            float pos_ref = s.pos_trap_traj_buf[s.cpm_idx];
            float vel_ref = s.vel_trap_traj_buf[s.cpm_idx];
            float tau_pd = p.cpm_kp * (pos_ref - s.current_position) + p.cpm_kd * (vel_ref - s.filtered_velocity);

            // DOB
            s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
            s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
            s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
            s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

            s.target_torque = clamp_torque(i, tau_pd + tau_g - s.estimated_disturbance);
            s.prev_target_torque = s.target_torque - tau_g;
            s.cpm_idx++;
            return s.target_torque;
        } else {
            s.last_end = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
            s.cpm_hold_ts = millis();
            s.cpm_state = CPMState::HOLD;
            s.cpm_inited = false;
        }
    }
    else if (s.cpm_state == CPMState::HOLD) {
        float hold_pos = (s.cpm_dir < 0) ? s.P_MIN : s.P_MAX;
        float tau_pd_hold = p.cpm_kp * (hold_pos - s.current_position) - p.cpm_kd * s.filtered_velocity;

        // DOB
        s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
        s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
        s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
        s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

        s.target_torque = clamp_torque(i, tau_pd_hold + tau_g - s.estimated_disturbance);
        s.prev_target_torque = s.target_torque - tau_g;

        if (millis() - s.cpm_hold_ts >= 1000) {
            s.cpm_dir = -s.cpm_dir;
            s.cpm_state = CPMState::BUILD;
        }
        return s.target_torque;
    }
    return 0.0f;
}

// ============================================================================
// [Control: PROM / AROM Transparent Mode]
// ============================================================================
float run_transparent(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);
    s.acceleration = (s.filtered_velocity - s.prev_filtered_velocity) / DOB_Ts;

    // DOB
    s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
    s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
    s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
    s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

    float tau_g = p.gc_m * p.gc_l * gc_G * sinf(s.current_position);
    s.target_torque = clamp_torque(i, tau_g - s.estimated_disturbance);

    s.prev_target_torque = s.target_torque - tau_g;
    s.prev_filtered_velocity = s.filtered_velocity;
    return s.target_torque;
}

// ============================================================================
// [Control: Isometric]
// ============================================================================
float run_isometric(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);
    s.acceleration = (s.filtered_velocity - s.prev_filtered_velocity) / DOB_Ts;

    float Kp = p.cpm_kp * 1.5f; 
    float Kd = p.cpm_kd * 1.2f;

    // DOB
    s.tau_nominal_plant = p.dob_j * s.acceleration + p.dob_b * s.filtered_velocity;
    s.filtered_tau_nominal_plant += alpha_Q * (s.tau_nominal_plant - s.filtered_tau_nominal_plant);
    s.filtered_prev_target_torque += alpha_Q * (s.prev_target_torque - s.filtered_prev_target_torque);
    s.estimated_disturbance = s.filtered_tau_nominal_plant - s.filtered_prev_target_torque;

    float tau_g = p.gc_m * p.gc_l * gc_G * sinf(s.current_position);
    float tau_pd = 0.0f;

    if (s.isom_state == IsometricState::MOVING_TO_TARGET) {
        float pos_err = s.isom_target_angle - s.current_position;
        tau_pd = Kp * pos_err - Kd * s.filtered_velocity;

        if (fabs(pos_err) < (2.0f * M_PI / 180.0f) && fabs(s.filtered_velocity) < 0.05f) {
            s.isom_hold_start_time = millis();
            s.isom_state = IsometricState::HOLDING;
        }
    } else if (s.isom_state == IsometricState::HOLDING) {
        tau_pd = (Kp * 1.5f) * (s.isom_target_angle - s.current_position) - (Kd * 1.5f) * s.filtered_velocity;
        if (millis() - s.isom_hold_start_time >= s.isom_hold_duration_ms) {
            s.g_isometricMode = false;
            s.isom_state = IsometricState::IDLE;
        }
    }

    s.target_torque = clamp_torque(i, tau_pd + tau_g - s.estimated_disturbance);
    s.prev_target_torque = s.target_torque - tau_g;
    s.prev_filtered_velocity = s.filtered_velocity;
    return s.target_torque;
}

// ============================================================================
// [AROM Mode Runner]
// ============================================================================
float run_arom(int i) {
    MotorState& s = m_states[i];
    float tau = run_transparent(i);

    switch (s.arom_state) {
        case AROMState::IDLE:
            s.arom_init_position = s.current_position;
            s.arom_max_position = s.current_position;
            s.arom_min_position = s.current_position;
            s.arom_state = AROMState::RUNNING;
            break;

        case AROMState::RUNNING:
            if (s.current_position > s.arom_max_position) s.arom_max_position = s.current_position;
            if (s.current_position < s.arom_min_position) s.arom_min_position = s.current_position;
            break;
    }
    return tau;
}

// ============================================================================
// [Control: FIX_POS]
// ============================================================================
float run_fix_pos(int i) {
    MotorState& s = m_states[i];
    MotorProfile& p = motor_profiles[i];

    s.filtered_velocity += alpha_vel * (s.current_velocity - s.filtered_velocity);
    
    // 단순 PD 제어 (Jetson에서 지정한 목표 위치로 유지)
    float pos_err = s.fix_target_pos - s.current_position; 
    float tau_pd = p.cpm_kp * pos_err - p.cpm_kd * s.filtered_velocity;
    
    float tau_g = p.gc_m * p.gc_l * gc_G * sinf(s.current_position);
    
    s.target_torque = clamp_torque(i, tau_pd + tau_g);
    return s.target_torque;
}

// ============================================================================
// [UDP Command Parser]
// ============================================================================
void processUDPCommand(const JetsonToM7_UDP& cmd) {
    // 1. 상태 전환 (전체 Enable / Disable)
    if (cmd.robot_state == 1 && !motor_enabled) {
        for (int i=0; i<4; i++) {
            motor.setMotorID(motor_profiles[i].id);
            motor.setMotionMode(MD80_MODE_RAW_TORQUE);
            delay(10);
            motor.enable();
            delay(10);
            
            // Enable 직후 해당 모터가 실제로 물리적으로 살아있는지 확인하여 "감시 대상"으로 등록
            readMotorState(i);
            expected_online[i] = m_states[i].online;
        }
        motor_enabled = true;
    } else if (cmd.robot_state == 0 && motor_enabled) {
        for (int i=0; i<4; i++) {
            motor.setMotorID(motor_profiles[i].id);
            motor.setTargetTorque(0.0f);
            delay(2);
            motor.disable();
            delay(2);
        }
        motor_enabled = false;
    }

    last_seq_echo = cmd.seq;

    if (!motor_enabled) return;

    // 2. 각 관절별 모드 파싱 및 독립 변수 세팅
    for (int i = 0; i < 4; i++) {
        uint8_t mode = cmd.joints[i].mode;
        MotorState& s = m_states[i];
        
        // 새로운 모드로 변경되었을 때 초기화 수행
        if (s.prev_mode != mode) {
            resetDOB(i);
            s.prev_mode = mode;
            
            s.g_cpmMode = (mode == CMD_CPM);
            s.g_promMode = (mode == CMD_PROM);
            s.g_aromMode = (mode == CMD_AROM);
            s.g_isometricMode = (mode == CMD_ISOM);
            s.g_fixMode = (mode == CMD_FIX_POS);

            if (s.g_cpmMode) {
                s.P_MIN = cmd.joints[i].target_pos_min;
                s.P_MAX = cmd.joints[i].target_pos_max;
                if (cmd.joints[i].speed > 0.01f) {
                    s.CPM_V_MAX = cmd.joints[i].speed;
                }
                s.last_end = s.current_position;
                s.cpm_state = CPMState::BUILD;
                s.cpm_dir = 1;
            }
            else if (s.g_isometricMode) {
                s.isom_target_angle = cmd.joints[i].target_pos_min;
                // Jetson의 resistance 변수는 Hold Duration(초)을 의미
                s.isom_hold_duration_ms = (uint32_t)(cmd.joints[i].resistance * 1000.0f);
                s.isom_state = IsometricState::MOVING_TO_TARGET;
            }
            else if (s.g_aromMode) {
                s.arom_state = AROMState::IDLE;
            }
            else if (s.g_fixMode) {
                s.fix_target_pos = cmd.joints[i].target_pos_min; // 주로 HOME_POS (0.0)
            }
            
            const char* modeStr = "IDLE";
            if (s.g_cpmMode) modeStr = "CPM";
            else if (s.g_promMode) modeStr = "PROM";
            else if (s.g_aromMode) modeStr = "AROM";
            else if (s.g_isometricMode) modeStr = "ISOM";
            else if (s.g_fixMode) modeStr = "FIX_POS";
            
            Serial.print("[EVENT] Motor ");
            Serial.print(motor_profiles[i].id);
            Serial.print(" mode changed to: ");
            Serial.println(modeStr);
        }
    }
}

// ============================================================================
// [Setup]
// ============================================================================
void setup() {
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LOW);  // Red ON during init
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000));

    Serial.println("=== Portenta H7 Rehab Controller (UDP Multi-Motor) ===");

    // Watchdog Shared Memory 초기화
    wd_data->m7_heartbeat = 0;
    wd_data->motor_comm_failed = 0;
    wd_data->e_stop_triggered = 0;

    // 동적 배열 할당 (SRAM 힙 영역 사용) - 모터당 10000개 배열 (20초 궤적 분량)
    for (int i=0; i<4; i++) {
        m_states[i].pos_trap_traj_buf = new float[10000];
        m_states[i].vel_trap_traj_buf = new float[10000];
    }

    bootM4();
    delay(100);
    Serial.println("[INIT] M4 booted");

    // CAN 통신 초기화 (기본적으로 첫 번째 모터 ID 바인딩)
    motor.setMotorID(motor_profiles[0].id);
    if (motor.openCAN()) {
        Serial.println("[INIT] CAN opened (FDCAN 1M/5M BRS)");
    } else {
        Serial.println("[ERROR] CAN init failed!");
        while(1) { delay(100); }
    }
    delay(100);
    
    // 이더넷 & UDP 초기화
    Serial.println("[INIT] Starting Ethernet...");
    Ethernet.begin(mac, ip);
    
    // Portenta Ethernet 연결 대기
    while (Ethernet.linkStatus() == LinkOFF) { 
        delay(500); 
        Serial.print("."); 
    }
    
    Udp.begin(localPort);
    Serial.print("\n[INIT] UDP Listening on IP: ");
    Serial.print(Ethernet.localIP());
    Serial.print(" Port: ");
    Serial.println(localPort);

    // 500Hz 루프 실행
    t_control.attach(&onControlTick, 1.0f / LOOP_RATE_HZ);

    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, LOW);   // Blue ON = ready
    Serial.println("[INIT] Setup Complete. Waiting for Jetson UDP Commands...");
}

// ============================================================================
// [Main Loop]
// ============================================================================
void loop() {
    // 1. UDP 수신 (비동기 처리)
    int packetSize = Udp.parsePacket();
    if (packetSize > 0) {
        if (packetSize == sizeof(JetsonToM7_UDP)) {
            JetsonToM7_UDP cmd;
            Udp.read((char*)&cmd, sizeof(JetsonToM7_UDP));
            
            jetson_ip = Udp.remoteIP();
            jetson_port = Udp.remotePort();
            jetson_connected = true;

            processUDPCommand(cmd);
        } else {
            static uint32_t last_err_print = 0;
            if (millis() - last_err_print > 1000) { // 1초 단위로만 경고 출력
                Serial.print("[UDP ERR] Packet size mismatch! Expected: ");
                Serial.print(sizeof(JetsonToM7_UDP));
                Serial.print(" bytes, but got: ");
                Serial.print(packetSize);
                Serial.println(" bytes.");
                last_err_print = millis();
            }
        }
    }

    // 2. 500Hz 다중 제어 루프
    if (!tick_control) return;
    tick_control = false;

    // Watchdog 데이터 갱신
    wd_data->m7_heartbeat++;

    // E-STOP 상태 체크 (M4에서 발동)
    if (wd_data->e_stop_triggered == 1) {
        if (motor_enabled) {
            for (int i=0; i<4; i++) {
                motor.setMotorID(motor_profiles[i].id);
                motor.setTargetTorque(0.0f);
                delay(2);
                motor.disable();
                delay(2);
            }
            motor_enabled = false;
            Serial.println("[CRITICAL] E-STOP TRIGGERED BY M4 WATCHDOG! Motors Disabled.");
        }
        return; // 모터 제어 루프 진입 차단
    }

    uint32_t t_start = micros();
    uint32_t fdcan_rtt_total = 0;

    if (motor_enabled) {
        bool comm_failure = false;
        
        // 4개 모터 순회하며 개별 제어 루프 수행
        for (int i = 0; i < 4; i++) {
            uint32_t m_start = micros();
            
            motor.setMotorID(motor_profiles[i].id); // 제어 대상 ID 스위칭
            
            readMotorState(i);
            
            // [지능형 안전 로직] 처음에 연결되어 있던 모터가 갑자기 끊어질 때만 E-STOP 발동
            if (expected_online[i] && !m_states[i].online) {
                comm_failure = true;
            }

            float cmd_tau = 0.0f;
            MotorState& s = m_states[i];

            if (s.g_cpmMode) {
                cmd_tau = run_cpm(i);
            } else if (s.g_promMode) {
                cmd_tau = run_transparent(i);
            } else if (s.g_aromMode) {
                cmd_tau = run_arom(i);
            } else if (s.g_isometricMode) {
                cmd_tau = run_isometric(i);
            } else if (s.g_fixMode) {
                cmd_tau = run_fix_pos(i);
            }

            cmd_tau = clamp_torque(i, cmd_tau);
            motor.setTargetTorque(cmd_tau);
            s.motor_output_torque = cmd_tau;
            
            fdcan_rtt_total += (micros() - m_start);
        }
        
        // 등록된 모터 중 하나라도 통신이 끊겼다면 M4에게 E-Stop 요청
        wd_data->motor_comm_failed = comm_failure ? 1 : 0;
    } else {
        wd_data->motor_comm_failed = 0;
    }

    uint32_t t_end = micros();
    uint32_t m7_process_us = t_end - t_start;

    // 3. UDP 상태 응답 (루프 완료 후 Jetson으로 바로 전송)
    if (jetson_connected) {
        M7ToJetson_UDP reply;
        reply.robot_state = motor_enabled ? 1 : 0;
        reply.seq_echo = last_seq_echo;
        reply.m7_rx_to_tx_us = m7_process_us; // 제어 처리 시간 기록
        reply.fdcan_rtt_us = fdcan_rtt_total;

        for (int i = 0; i < 4; i++) {
            reply.joints[i].pos = m_states[i].current_position;
            reply.joints[i].vel = m_states[i].current_velocity;
            reply.joints[i].tau_meas = m_states[i].real_torque;
            reply.joints[i].tau_cmd = m_states[i].motor_output_torque;
            reply.joints[i].online = m_states[i].online ? 1 : 0;
        }

        Udp.beginPacket(jetson_ip, jetson_port);
        Udp.write((uint8_t*)&reply, sizeof(M7ToJetson_UDP));
        Udp.endPacket();
    }
}