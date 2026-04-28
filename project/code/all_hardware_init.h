#ifndef ALL_HARDWARE_INIT_H_
#define ALL_HARDWARE_INIT_H_

#include "zf_common_headfile.h"
#include "imu_fusion.h"


// ========================= PIT =========================
extern uint8    pit_state;
extern uint16_t TimerCount;
extern volatile uint8_t g_imu_update_req;
#define PIT_NUM                 (PIT_CH0)   
#define PIT_NUM1                (PIT_CH1)


// ========================= GPIO: LED/KEY/SW =========================
#define LED1                    (P19_0)

#define KEY1                    (P20_0)
#define KEY2                    (P20_1)
#define KEY3                    (P20_2)
#define KEY4                    (P20_3)

#define SWITCH1                 (P21_5)
#define SWITCH2                 (P21_6)

// ========================= Buzzer =========================
#define BUZZER_PIN              (P19_4)

// ========================= Encoder =========================
#define ENCODER_QUADDEC1        (TC_CH58_ENCODER)
#define ENCODER_QUADDEC_A1      (TC_CH58_ENCODER_CH1_P17_3)
#define ENCODER_QUADDEC_B1      (TC_CH58_ENCODER_CH2_P17_4)

#define ENCODER_QUADDEC2        (TC_CH27_ENCODER)
#define ENCODER_QUADDEC_A2      (TC_CH27_ENCODER_CH1_P19_2)
#define ENCODER_QUADDEC_B2      (TC_CH27_ENCODER_CH2_P19_3)

// ===== 固定方向：来自最后一次 AUTO-CAL =====
#define ENCODER1_SIGN           (-1)   // E1 数值取反才是“正转为正”
#define ENCODER2_SIGN           (+1)   // E2 数值本身就是“正转为正”

// ===== 明确轮子和编码器的配对（方便读代码）=====
#define LEFT_ENCODER_QD         ENCODER_QUADDEC1   // 左轮 -> E1
#define RIGHT_ENCODER_QD        ENCODER_QUADDEC2   // 右轮 -> E2
#define LEFT_ENCODER_SIGN       ENCODER1_SIGN
#define RIGHT_ENCODER_SIGN      ENCODER2_SIGN
// 如果别处需要打印速度，在此保留 extern（定义放在 motor.c）
extern volatile int   Motor1Speed;
extern volatile int   Motor2Speed;

// ========================= Motor IO/PWM =========================
#define MOTOR1_DIR_FORWARD_LEVEL   GPIO_LOW
#define MOTOR1_DIR_REVERSE_LEVEL   GPIO_HIGH
#define MOTOR2_DIR_FORWARD_LEVEL   GPIO_LOW
#define MOTOR2_DIR_REVERSE_LEVEL   GPIO_HIGH

#define MAX_DUTY                 (8)     // 最大占空“点数”，对应 0~8（建议与 pwm_set_duty 的映射保持一致）
#define MIN_DUTY                 (5)     // 起步占空（克服静摩擦）
#define CLAMP(x, lo, hi)  ((x)<(lo)?(lo):((x)>(hi)?(hi):(x)))

#define DIR_L1                   (P09_1)
#define PWM_L1                   (TCPWM_CH24_P09_0)
#define DIR_R1                   (P10_3)
#define PWM_R1                   (TCPWM_CH30_P10_2)

// ========================= Servo =========================
#define STEER_SERVO_PWM            (TCPWM_CH13_P00_3)   
#define STEER_SERVO_FREQ           (50U)                
#define STEER_SERVO_LEFT_DEG       (60.0f)              
#define STEER_SERVO_RIGHT_DEG      (120.0f)             
#define STEER_SERVO_CENTER_DEG     (90.0f)              
extern float g_servo_center_trim_deg;
#if (STEER_SERVO_FREQ < 50U) || (STEER_SERVO_FREQ > 300U)
#error "STEER_SERVO_FREQ out of range. See servo manual (50~300Hz)."
#endif

// 打印所需（定义放在 motor.c）
extern int      g_motor1_set_param;
extern int      g_motor2_set_param;
extern uint32_t g_motor1_duty;
extern uint32_t g_motor2_duty;
extern float    g_pid1_output;
extern float    g_pid2_output;

// 目标速度（定义放在 motor.c）
extern volatile float g_target_speed_m1;
extern volatile float g_target_speed_m2;

// Ackermann 舵机变量定义位于 all_hardware_init.c
extern float g_servo_angle_deg;
void ackermann_servo_init(void);
void ackermann_servo_set_angle(float angle_deg);
uint16_t steer_servo_deg_to_duty(float angle_deg);



// ========================= Wireless UART =========================
extern uint8 data_buffer[32];
extern uint8 data_len;
void wprintf(const char *fmt, ...);


// ========================= API =========================
void all_hardware_init(void);

#endif // ALL_HARDWARE_INIT_H_



