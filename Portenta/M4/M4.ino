/**
 * M4.ino - Portenta H7 M4 Core
 * ============================================================================
 * [Safety Watchdog & System Monitor]
 * M7 코어와 공유 메모리(SRAM4: 0x38001000)를 통해 통신하며, 
 * 모터 통신이 아예 안될 때(완전 단절)만 E-Stop을 발동하도록 수정되었습니다.
 * ============================================================================
 */

#include <Arduino.h>

struct WatchdogData {
    volatile uint32_t m7_heartbeat;
    volatile uint8_t motor_comm_failed;
    volatile uint8_t e_stop_triggered;
};

// SRAM4 영역 (M7과 M4가 데이터를 빠르게 공유할 수 있는 하드웨어 메모리 주소)
volatile WatchdogData* wd_data = (volatile WatchdogData*)0x38001000;

void setup() {
    // Portenta H7 내장 RGB LED 핀 설정
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    
    // LED 끄기 (Portenta LED는 Active LOW 방식이므로 HIGH가 꺼짐)
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, HIGH);
}

void loop() {
    // 모터 통신이 아예 끊겼을 때(완전 단절) 비상 정지 발동
    if (wd_data->motor_comm_failed == 1) {
        wd_data->e_stop_triggered = 1; // M7에게 비상 정지 명령 하달
        
        // 빨간색 LED 점멸 (경고 - Active Low)
        digitalWrite(LEDR, LOW);
        digitalWrite(LEDG, HIGH);
        digitalWrite(LEDB, HIGH);
        
        /* 
         * [추후 확장]
         * 여기에 모터 메인 전원 릴레이(Relay)를 물리적으로 끊어버리는 
         * GPIO 제어 코드를 추가하면 100% 하드웨어 레벨의 안전한 E-Stop이 완성됩니다.
         * 예: digitalWrite(RELAY_PIN, LOW);
         */
    } else {
        wd_data->e_stop_triggered = 0;
        
        // 초록색 LED 점등 (정상 작동 중 - Active Low)
        digitalWrite(LEDR, HIGH);
        digitalWrite(LEDG, LOW);
        digitalWrite(LEDB, HIGH);
    }
    
    delay(100); // 10Hz 주기로 가볍게 감시
}