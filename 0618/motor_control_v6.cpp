/*
 * 작성일자: 2026-05-14
 * 작성자: 상지재활로봇 제어 전문가
 * 
 * [ 상세 설명 ]
 * 이 노드는 상지재활로봇의 상위 제어기(Jetson) 역할을 수행하는 ROS 2 기반의 C++ 노드입니다.
 * 태블릿 어플리케이션(Dart)에서 전송하는 블루투스 명령(문자열)을 ROS 2 토픽('/tablet_cmd')을 통해 수신하여, 
 * 이를 좌/우 양팔의 Portenta H7(M7 코어)에 UDP 패킷으로 변환하여 전달합니다.
 * 
 * [ 주요 기능 ]
 * 1. 선택적 관절 구동: 'PART:xxx' 명령을 수신하여 활성화할 팔(Left/Right)과 관절 인덱스를 식별합니다.
 * 2. 특정 포지션 고정(FIX_POS): 선택되지 않은 나머지 3개의 관절은 0도(또는 지정된 초기 자세)로 부드럽게 이동시킨 후 강력하게 고정(FIX_POS)합니다.
 * 3. 5가지 제어 모드 매핑: AROM, PROM, CPM, ISOMETRIC 명령을 파싱하여 UDP 구조체에 담아 Portenta로 전송합니다.
 * 4. 양팔 실시간 데이터 수신: Portenta로부터 50Hz 주기로 들어오는 센서 데이터(각도, 속도, 토크, 온라인 여부)를 수신하고, 
 *    AI(강직도 추정) 및 UI에 전달할 수 있도록 '/robot/joint_states' 토픽으로 퍼블리시합니다.
 * 
 * [ 주의 사항 ]
 * - 태블릿 앱(Dart) 코드에서 `// await BluetoothService.instance.send(payload);` 로 
 *   주석 처리되어 있는 'PART:xxx' 전송 부분을 반드시 주석 해제해야 Jetson이 어떤 관절을 선택했는지 알 수 있습니다!
 */

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// UDP 설정
#define JETSON_PORT 8888
#define PORTENTA_PORT 9999
const char* IP_RIGHT_ARM = "172.26.178.50";
const char* IP_LEFT_ARM = "172.26.178.51";

#pragma pack(push, 1)
enum ControlMode : uint8_t { 
    IDLE = 0, AROM = 1, PROM = 2, CPM = 3, ISOM = 4, ISOT = 5, FIX_POS = 6 
};

struct JointCmd {
    uint8_t mode;        
    float target_pos_min; 
    float target_pos_max; 
    float speed;         
    float resistance;    
};

struct JetsonToM7_UDP {
    uint8_t robot_state; // 0: Off, 1: On, 2: Zero, 3: E-Stop
    uint32_t seq;
    JointCmd joints[4];
};

struct JointState {
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
    JointState joints[4];
};
#pragma pack(pop)

enum ArmSide { RIGHT_ARM = 0, LEFT_ARM = 1, NONE = -1 };

const float DEFAULT_SPEED = 0.05f;

class UpperLimbManagerNode : public rclcpp::Node {
public:
    UpperLimbManagerNode() : Node("upper_limb_manager"), is_running_(true) {
        // 태블릿 앱으로부터의 블루투스 명령 수신 (ROS 브릿지를 통해 들어온다고 가정)
        cmd_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "tablet_cmd", 10, std::bind(&UpperLimbManagerNode::cmdCallback, this, std::placeholders::_1));

        // AI 모델 및 UI 모니터링용 상태 퍼블리셔
        state_publisher_ = this->create_publisher<std_msgs::msg::String>("robot/joint_states", 10);

        // 기본 초기화
        memset(&cmd_right_, 0, sizeof(JetsonToM7_UDP));
        memset(&cmd_left_, 0, sizeof(JetsonToM7_UDP));
        cmd_right_.robot_state = 1; // On
        cmd_left_.robot_state = 1;  // On

        // 소켓 초기화 및 스레드 시작
        initUDPSockets();
        udp_rx_thread_ = std::thread(&UpperLimbManagerNode::udpReceiveLoop, this);
        control_tx_thread_ = std::thread(&UpperLimbManagerNode::controlTxLoop, this);

        RCLCPP_INFO(this->get_logger(), "Upper Limb Manager Node Started.");
    }

    // ~UpperLimbManagerNode() {
    //     is_running_ = false;
    //     if (udp_rx_thread_.joinable()) udp_rx_thread_.join();
    //     if (control_tx_thread_.joinable()) control_tx_thread_.join();
    //     close(sockfd_rx_);
    // }
    ~UpperLimbManagerNode() {
        is_running_ = false;
        
        // -------------------------------------------------------------
        // [안전 로직] Ctrl+C 종료 시 모터 Disable 명령 (robot_state = 0) 전송
        RCLCPP_INFO(this->get_logger(), "Shutting down... Sending Disable Command to Motors.");
        
        cmd_right_.robot_state = 0; // 0: Disable
        cmd_left_.robot_state = 0;
        for (int i = 0; i < 4; i++) {
            cmd_right_.joints[i].mode = IDLE;
            cmd_left_.joints[i].mode = IDLE;
        }

        // 마지막 패킷 전송 (확실한 전달을 위해 2~3회 반복 전송)
        for(int i=0; i<3; i++) {
            sendto(sockfd_rx_, &cmd_right_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&right_arm_addr_, sizeof(right_arm_addr_));
            sendto(sockfd_rx_, &cmd_left_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&left_arm_addr_, sizeof(left_arm_addr_));
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        // -------------------------------------------------------------

        if (udp_rx_thread_.joinable()) udp_rx_thread_.join();
        if (control_tx_thread_.joinable()) control_tx_thread_.join();
        close(sockfd_rx_);
    }

private:
    std::thread udp_rx_thread_, control_tx_thread_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscriber_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;

    std::atomic<bool> is_running_;
    int sockfd_rx_;
    struct sockaddr_in servaddr_, right_arm_addr_, left_arm_addr_;

    std::mutex cmd_mtx_;
    JetsonToM7_UDP cmd_right_, cmd_left_;

    // 통신 속도 측정용
    uint32_t seq_counter_ = 0;
    uint64_t tx_timestamps_[1024] = {0};

    // 현재 선택된 부위 상태 (기본값: 오른쪽 어깨 굴곡/신전)
    ArmSide selected_arm_ = RIGHT_ARM;
    int selected_joint_idx_ = 0; // 0:ShoulderEF, 1:ShoulderRo, 2:Elbow, 3:Wrist
    
    // PROM 진행을 위한 내부 토글 상태
    bool prom_active_ = false;
    bool arom_active_ = false;

    // 현재 관절 각도 실시간 저장 (stop 시 FIX_POS 반영용)
    float current_pos_right_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float current_pos_left_[4]  = {0.0f, 0.0f, 0.0f, 0.0f};

    uint64_t get_micros() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                   std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    void initUDPSockets() {
        sockfd_rx_ = socket(AF_INET, SOCK_DGRAM, 0);
        
        // 포트 재사용 설정
        int opt = 1;
        setsockopt(sockfd_rx_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // [핵심 추가] 소켓 수신 타임아웃 설정 (0.1초) -> 이걸 넣어야 Ctrl+C 종료가 잘 먹힙니다.
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        setsockopt(sockfd_rx_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        memset(&servaddr_, 0, sizeof(servaddr_));
        servaddr_.sin_family = AF_INET;
        servaddr_.sin_addr.s_addr = INADDR_ANY;
        servaddr_.sin_port = htons(JETSON_PORT);
        bind(sockfd_rx_, (const struct sockaddr *)&servaddr_, sizeof(servaddr_));

        memset(&right_arm_addr_, 0, sizeof(right_arm_addr_));
        right_arm_addr_.sin_family = AF_INET;
        right_arm_addr_.sin_port = htons(PORTENTA_PORT);
        inet_pton(AF_INET, IP_RIGHT_ARM, &right_arm_addr_.sin_addr);

        memset(&left_arm_addr_, 0, sizeof(left_arm_addr_));
        left_arm_addr_.sin_family = AF_INET;
        left_arm_addr_.sin_port = htons(PORTENTA_PORT);
        inet_pton(AF_INET, IP_LEFT_ARM, &left_arm_addr_.sin_addr);
    }

    std::vector<float> getHomePos(int selected_joint_idx) {
        // 관절에 따른 시작 자세 설정 (단위: Radian)
        // [어깨 굴곡/신전, 어깨 내회전/외회전, 팔꿈치 굴곡/신전, 손목 굴곡/신전]
        if (selected_joint_idx == 1) {
            // 어깨 내회전/외회전: [90, 0, 90, 0]
            return {static_cast<float>(M_PI / 2.0), 0.0f, static_cast<float>(M_PI / 2.0), 0.0f};
        } else if (selected_joint_idx == 3) {
            // 손목 굴곡/신전: [0, 0, 90, 0]
            return {0.0f, 0.0f, static_cast<float>(M_PI / 2.0), 0.0f};
        } else {
            // 어깨 굴곡/신전 (0), 팔꿈치 굴곡/신전 (2)
            return {0.0f, 0.0f, 0.0f, 0.0f};
        }
    }

    void applyHoldToUnselectedJoints() {
        // 선택된 팔과 관절을 제외한 모든 관절을 FIX_POS로 설정하여 타겟 자세로 이동 후 고정
        std::vector<float> target_home_pos = getHomePos(selected_joint_idx_);
        
        for (int i = 0; i < 4; i++) {
            if (selected_arm_ != RIGHT_ARM || selected_joint_idx_ != i) {
                cmd_right_.joints[i].mode = FIX_POS;
                cmd_right_.joints[i].target_pos_min = target_home_pos[i];
                cmd_right_.joints[i].speed = DEFAULT_SPEED;
            }
            if (selected_arm_ != LEFT_ARM || selected_joint_idx_ != i) {
                cmd_left_.joints[i].mode = FIX_POS;
                cmd_left_.joints[i].target_pos_min = target_home_pos[i];
                cmd_left_.joints[i].speed = DEFAULT_SPEED;
            }
        }
    }

    void applyStopFix() {
        // 정지 명령 발생 시, 현재 실제 관절 각도 기반으로 즉시 FIX_POS 반영
        for (int i = 0; i < 4; i++) {
            cmd_right_.joints[i].mode = FIX_POS;
            cmd_right_.joints[i].target_pos_min = current_pos_right_[i];
            
            cmd_left_.joints[i].mode = FIX_POS;
            cmd_left_.joints[i].target_pos_min = current_pos_left_[i];
        }
        
        // 3초 뒤 IDLE로 전환하여 완전히 힘을 뺌
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(3));
            std::lock_guard<std::mutex> lock(cmd_mtx_);
            resetAllToIdle();
            RCLCPP_INFO(this->get_logger(), "Transitioned to IDLE after 3 seconds of FIX_POS.");
        }).detach();
    }

    void resetAllToIdle() {
        for (int i = 0; i < 4; i++) {
            cmd_right_.joints[i].mode = IDLE;
            cmd_left_.joints[i].mode = IDLE;
        }
    }

    void cmdCallback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        // 개행문자 제거
        cmd.erase(std::remove(cmd.begin(), cmd.end(), '\n'), cmd.end());
        cmd.erase(std::remove(cmd.begin(), cmd.end(), '\r'), cmd.end());

        RCLCPP_INFO(this->get_logger(), "Bluetooth Command Rx: '%s'", cmd.c_str());

        std::lock_guard<std::mutex> lock(cmd_mtx_);

        // 1. 관절 선택 (PART:xxxx)
        if (cmd.find("PART:") == 0) {
            std::string part = cmd.substr(5);
            if (part == "lShoulderEF") { selected_arm_ = LEFT_ARM; selected_joint_idx_ = 0; }
            else if (part == "lShoulderRo") { selected_arm_ = LEFT_ARM; selected_joint_idx_ = 1; }
            else if (part == "lElbow") { selected_arm_ = LEFT_ARM; selected_joint_idx_ = 2; }
            else if (part == "lWrist") { selected_arm_ = LEFT_ARM; selected_joint_idx_ = 3; }
            else if (part == "rShoulderEF") { selected_arm_ = RIGHT_ARM; selected_joint_idx_ = 0; }
            else if (part == "rShoulderRo") { selected_arm_ = RIGHT_ARM; selected_joint_idx_ = 1; }
            else if (part == "rElbow") { selected_arm_ = RIGHT_ARM; selected_joint_idx_ = 2; }
            else if (part == "rWrist") { selected_arm_ = RIGHT_ARM; selected_joint_idx_ = 3; }
            else {
                RCLCPP_WARN(this->get_logger(), "Unknown PART: %s", part.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "Selected Arm: %d, Joint: %d", selected_arm_, selected_joint_idx_);
            return;
        }

        // 동작 명령은 반드시 부위가 먼저 선택되어 있어야 함
        if (selected_arm_ == NONE || selected_joint_idx_ == -1) {
            if (cmd == "stop" || cmd == "x") resetAllToIdle(); // 예외적으로 정지 명령은 허용
            else {
                RCLCPP_WARN(this->get_logger(), "Command ignored: No PART selected yet.");
                return;
            }
        }

        // 대상 구조체 포인터 획득
        JetsonToM7_UDP* target_cmd = (selected_arm_ == RIGHT_ARM) ? &cmd_right_ : &cmd_left_;
        JointCmd* target_joint = &(target_cmd->joints[selected_joint_idx_]);

        // 모든 비선택 관절을 고정 모드로 세팅
        applyHoldToUnselectedJoints();

        // 2. 모드 파싱 및 매핑
        if (cmd == "stop" || cmd == "x" || cmd == "isom_stop") {
            applyStopFix();
            prom_active_ = false;
            arom_active_ = false;
            RCLCPP_INFO(this->get_logger(), "All Modes Stopped and Fixed.");
        }
        else if (cmd == "arom") {
            arom_active_ = true;
            target_joint->mode = AROM;
        }
        else if (cmd.find("prom") == 0) {
            prom_active_ = true;
            target_joint->mode = PROM;
            target_joint->speed = DEFAULT_SPEED; // 기본 속도 적용
            
            std::stringstream ss(cmd);
            std::string item;
            std::vector<std::string> tokens;
            while(std::getline(ss, item, ',')) { tokens.push_back(item); }
            if(tokens.size() >= 2) {
                target_joint->speed = std::stof(tokens[1]);
            }
        }
        else if (cmd == "dir") {
            // 방향 전환 처리는 실제 M7 내에서 상태 변경을 요구하므로,
            // Jetson에서는 PROM 모드 플래그만 유지. (실제 M7 코드의 dir 처리를 위한 커스텀 패킷 정의 필요. 
            // 현재 구조체에서는 PROM 모드 재전송 시 방향이 바뀌도록 M7 코드를 향후 조정하거나, speed 부호를 반대로 보냄)
            target_joint->speed = -(target_joint->speed); 
        }
        else if (cmd.find("cpm,") == 0) {
            std::stringstream ss(cmd.substr(4));
            std::string item1, item2, item3;
            if (std::getline(ss, item1, ',') && std::getline(ss, item2, ',')) {
                target_joint->mode = CPM;
                target_joint->target_pos_min = std::stof(item1) * M_PI / 180.0f; // Degree to Radian
                target_joint->target_pos_max = std::stof(item2) * M_PI / 180.0f;
                target_joint->speed = DEFAULT_SPEED; 
                if (std::getline(ss, item3, ',')) {
                    target_joint->speed = std::stof(item3);
                }
            }
        }
        else if (cmd.find("isometric,") == 0) {
            std::stringstream ss(cmd.substr(10));
            std::string item1, item2;
            if (std::getline(ss, item1, ',') && std::getline(ss, item2, ',')) {
                target_joint->mode = ISOM;
                target_joint->target_pos_min = std::stof(item1) * M_PI / 180.0f;
                target_joint->resistance = std::stof(item2); // Hold duration
                target_joint->speed = DEFAULT_SPEED;
            }
        }
        else if (cmd.find("isotonic,") == 0) {
            std::stringstream ss(cmd.substr(9));
            std::string item1, item2;
            if (std::getline(ss, item1, ',') && std::getline(ss, item2, ',')) {
                target_joint->mode = ISOT; // 5번 모드
                // mode는 target_pos_min에, resistance는 resistance에 임시 저장 (M7 측 구현 예정)
                target_joint->target_pos_min = std::stof(item1);
                target_joint->resistance = std::stof(item2);
                target_joint->speed = DEFAULT_SPEED;
            }
        }
    }

    void controlTxLoop() {
        while (is_running_) {
            {
                std::lock_guard<std::mutex> lock(cmd_mtx_);
                cmd_right_.seq = seq_counter_;
                cmd_left_.seq = seq_counter_;
                tx_timestamps_[seq_counter_ % 1024] = get_micros();
                seq_counter_++;

                sendto(sockfd_rx_, &cmd_right_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&right_arm_addr_, sizeof(right_arm_addr_));
                sendto(sockfd_rx_, &cmd_left_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&left_arm_addr_, sizeof(left_arm_addr_));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 100Hz 상위 루프 명령 전송
        }
    }

    void udpReceiveLoop() {
        M7ToJetson_UDP rx_data;
        struct sockaddr_in cliaddr;
        socklen_t len = sizeof(cliaddr);

        while (is_running_) {
            ssize_t n = recvfrom(sockfd_rx_, &rx_data, sizeof(rx_data), MSG_DONTWAIT, (struct sockaddr *)&cliaddr, &len);
            uint64_t now_us = get_micros();
            
            if (n == sizeof(M7ToJetson_UDP)) {
                char source_ip[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &(cliaddr.sin_addr), source_ip, INET_ADDRSTRLEN);
                std::string arm = (strcmp(source_ip, IP_RIGHT_ARM) == 0) ? "RIGHT" : "LEFT";

                // 실제 수신 통신량(Throughput) 카운트를 위한 정적 변수
                static uint32_t rx_packet_count = 0;
                static uint64_t last_print_time = get_micros();
                
                rx_packet_count++; // 패킷을 받을 때마다 카운트 1 증가

                // 정확히 1초(1,000,000 us)마다 통계 출력
                if (now_us - last_print_time >= 1000000) {
                    double actual_udp_hz = rx_packet_count / ((now_us - last_print_time) / 1000000.0);
                    
                    // FDCAN 통신 소요 시간을 모터 4개로 나누어 1개당 속도 산출 (1kHz 확인용)
                    double single_motor_hz = rx_data.fdcan_rtt_us > 0 ? 1000000.0 / (rx_data.fdcan_rtt_us / 4.0) : 0.0;
                    double m7_loop_hz = rx_data.m7_rx_to_tx_us > 0 ? 1000000.0 / rx_data.m7_rx_to_tx_us : 0.0;

                    RCLCPP_INFO(this->get_logger(), 
                        "[%s] Jetson <-> M7 통신 속도: %.1f Hz (목표: 100Hz)\n"
                        "        M7 <-> 모터 1개 통신 속도: %.1f Hz (목표: 1000Hz) | M7 전체 제어루프: %.1f Hz\n"
                        "      [0] Pos: %5.2f | Vel: %5.2f | TauCmd: %6.2f | TauMeas: %6.2f\n"
                        "      [1] Pos: %5.2f | Vel: %5.2f | TauCmd: %6.2f | TauMeas: %6.2f\n"
                        "      [2] Pos: %5.2f | Vel: %5.2f | TauCmd: %6.2f | TauMeas: %6.2f\n"
                        "      [3] Pos: %5.2f | Vel: %5.2f | TauCmd: %6.2f | TauMeas: %6.2f", 
                        arm.c_str(), 
                        actual_udp_hz,   // 100Hz 나와야 함
                        single_motor_hz, // 1000Hz 나와야 함
                        m7_loop_hz,      // 약 250Hz 나와야 함
                        rx_data.joints[0].pos, rx_data.joints[0].vel, rx_data.joints[0].tau_cmd, rx_data.joints[0].tau_meas,
                        rx_data.joints[1].pos, rx_data.joints[1].vel, rx_data.joints[1].tau_cmd, rx_data.joints[1].tau_meas,
                        rx_data.joints[2].pos, rx_data.joints[2].vel, rx_data.joints[2].tau_cmd, rx_data.joints[2].tau_meas,
                        rx_data.joints[3].pos, rx_data.joints[3].vel, rx_data.joints[3].tau_cmd, rx_data.joints[3].tau_meas);

                    // 카운터 및 시간 초기화
                    rx_packet_count = 0;
                    last_print_time = now_us;
                }

                // 수신된 현재 관절 각도 업데이트
                for (int i = 0; i < 4; i++) {
                    if (arm == "RIGHT") {
                        current_pos_right_[i] = rx_data.joints[i].pos;
                    } else {
                        current_pos_left_[i] = rx_data.joints[i].pos;
                    }
                }

                // 수신된 상태를 JSON 형태의 문자열로 만들어 퍼블리시 (UI 및 AI 모델용)
                std::stringstream json_ss;
                json_ss << "{\"arm\":\"" << arm << "\",\"state\":[";
                for (int i=0; i<4; i++) {
                    json_ss << "{\"id\":" << i 
                            << ",\"online\":" << (int)rx_data.joints[i].online
                            << ",\"pos\":" << rx_data.joints[i].pos 
                            << ",\"vel\":" << rx_data.joints[i].vel 
                            << ",\"tau_cmd\":" << rx_data.joints[i].tau_cmd
                            << ",\"tau_meas\":" << rx_data.joints[i].tau_meas << "}";
                    if (i < 3) json_ss << ",";
                }
                json_ss << "]}";

                std_msgs::msg::String msg;
                msg.data = json_ss.str();
                state_publisher_->publish(msg);


                // // -------------------------------------------------------------
                // // [핵심 수정 2] M7으로부터 상태를 받았으므로, 그 즉시 제어 명령 반사(Reply 송신)!
                // // 양팔을 구분하여 패킷을 보낸 쪽의 M7 코어에게만 즉시 응답을 돌려줍니다.
                // {
                //     std::lock_guard<std::mutex> lock(cmd_mtx_);
                //     if (arm == "RIGHT") {
                //         cmd_right_.seq = seq_counter_;
                //         tx_timestamps_[seq_counter_ % 1024] = now_us; // 송신 시간 기록 (RTT 계산용)
                //         sendto(sockfd_rx_, &cmd_right_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&right_arm_addr_, sizeof(right_arm_addr_));
                //     } else {
                //         cmd_left_.seq = seq_counter_;
                //         tx_timestamps_[seq_counter_ % 1024] = now_us;
                //         sendto(sockfd_rx_, &cmd_left_, sizeof(JetsonToM7_UDP), 0, (const struct sockaddr *)&left_arm_addr_, sizeof(left_arm_addr_));
                //     }
                //     seq_counter_++;
                // }
                // // -------------------------------------------------------------


            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        }
    }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UpperLimbManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
