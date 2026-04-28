#include "all_hardware_init.h"

// ------------------------- 仅保留“硬件初始化”需要的全局 -------------------------
uint8    pit_state   = 0;          // PIT 中断标志
volatile uint8_t g_imu_update_req = 0;
uint16_t TimerCount  = 0;          // PIT 计数，2ms 基准
uint16_t data[128];                // LCD 测试数据（若无用可删）
int16_t  data_index  = 0;          // LCD 测试数据索引（若无用可删）
uint8    data_buffer[32];          // 无线串口 Rx 缓冲
uint8    data_len    = 0;          // 无线串口接收长度
float g_servo_center_trim_deg = 0.0f;   // 当前用于中心点复验，先不加 trim
// ------------------------- 硬件初始化 -------------------------
float g_servo_angle_deg = STEER_SERVO_CENTER_DEG;   // current steering servo command (deg)
static uint8_t g_wireless_uart_ready = 0;
static uint16_t g_wireless_retry_gate = 0;

#define SERVO_DUTY_SCALE        (10000U)
#define STEER_SERVO_CNT_INDEX   (13U)      // TCPWM line index for STEER_SERVO_PWM (CH13)

uint16_t steer_servo_deg_to_duty(float angle_deg)
{
    // Use independent high-resolution scale for servo duty, decoupled from global PWM_DUTY_MAX.
    float duty = ((float)SERVO_DUTY_SCALE / (1000.0f / (float)STEER_SERVO_FREQ)) * (0.5f + angle_deg / 90.0f);
    duty = CLAMP(duty, 0.0f, (float)SERVO_DUTY_SCALE);
    return (uint16_t)(duty + 0.5f);
}

static void steer_servo_apply_duty(uint16_t duty_scaled)
{
    uint16_t period = (uint16_t)TCPWM0->GRP[0].CNT[STEER_SERVO_CNT_INDEX].unPERIOD.u32Register;
    uint16_t compare = (uint16_t)(((uint32_t)(period + 1U) * duty_scaled) / SERVO_DUTY_SCALE);

    TCPWM0->GRP[0].CNT[STEER_SERVO_CNT_INDEX].unCC0_BUFF.u32Register = compare;
    TCPWM0->GRP[0].CNT[STEER_SERVO_CNT_INDEX].unTR_CMD.u32Register |= 1U;
}// ========= 封装：无线串口 printf =========
void wprintf(const char *fmt, ...)
{
    char buf[128];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    // 无线串口初始化失败后，允许在后续日志中自动重试恢复
    if (!g_wireless_uart_ready)
    {
        if (g_wireless_retry_gate == 0)
        {
            if (!wireless_uart_init())
            {
                g_wireless_uart_ready = 1;
                wireless_uart_send_string("[INFO] wireless uart reconnected.\r\n");
            }
            g_wireless_retry_gate = 200;
        }
        else
        {
            g_wireless_retry_gate--;
        }
    }

    if (g_wireless_uart_ready)
    {
        wireless_uart_send_string(buf);
    }
}
void all_hardware_init(void)
{
    // PIT: 2ms 基准（pit0_ch0_isr 建议每 10ms 执行一次逻辑）
    pit_ms_init(PIT_NUM, 2);
    pit_ms_init(PIT_NUM1, 2);


    // LED
    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // KEY
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(KEY4, GPI, GPIO_HIGH, GPI_PULL_UP);

    // SWITCH
    gpio_init(SWITCH1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(SWITCH2, GPI, GPIO_HIGH, GPI_PULL_UP);

    // Buzzer
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // Encoder
    encoder_quad_init(ENCODER_QUADDEC1, ENCODER_QUADDEC_A1, ENCODER_QUADDEC_B1);
    encoder_quad_init(ENCODER_QUADDEC2, ENCODER_QUADDEC_A2, ENCODER_QUADDEC_B2);

    // Motor IO & PWM（上电默认方向电平设为“正转电平”，便于自校准）
    gpio_init(DIR_L1, GPO, MOTOR1_DIR_FORWARD_LEVEL, GPO_PUSH_PULL);
    pwm_init (PWM_L1, 17000, 0);   // 17kHz, duty=0

    gpio_init(DIR_R1, GPO, MOTOR2_DIR_FORWARD_LEVEL, GPO_PUSH_PULL);
    pwm_init (PWM_R1, 17000, 0);   // 17kHz, duty=0

    // Ackermann steering servo: hold dynamic center before motion
    ackermann_servo_init();

    // IMU：有限重试 + 明显告警（避免无限死循环卡死）
    {
        const int kMaxRetry = 5;
        int ok = 0;
        for (int i = 0; i < kMaxRetry; ++i) {
            if (!imu963ra_init()) { ok = 1; break; }
            printf("\r\nimu963ra init error. retry=%d/%d", i+1, kMaxRetry);
            gpio_toggle_level(LED1);
            system_delay_ms(200);
        }
        if (!ok) {
            // 仍失败：快速闪灯提示，但不阻塞整个系统（可按需 return/死循环）
            for (int i = 0; i < 10; ++i) {
                gpio_toggle_level(LED1);
                system_delay_ms(100);
            }
            printf("\r\nimu963ra init failed after %d retries.\r\n", kMaxRetry);
        }
    }
    imu_fusion_init(500.0f);
    imu_fusion_zero_calibrate(2000);  // 上电静置 2s 标定零偏（期间不要动机器）




    // Wireless UART
    if (wireless_uart_init()) {
        g_wireless_uart_ready = 0;
        // 初始化失败：闪灯提示 + 不死锁（可根据项目需要决定是否阻塞）
        for (int i = 0; i < 10; ++i) {
            gpio_toggle_level(LED1);
            system_delay_ms(100);
        }
        printf("\r\nWireless UART init failed.\r\n");
    } else {
        g_wireless_uart_ready = 1;
        wireless_uart_send_byte('\r');
        wireless_uart_send_byte('\n');
        wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");
        wireless_uart_send_string("[BOOT_TAG] CYT4BB_BUILD_20260310_A\r\n");
    }
    // 摄像头初始化已在 main_cm7_0.c 中完成，这里不再重复初始化，避免阻塞

}
void ackermann_servo_set_angle(float angle_deg)
{
    // 逻辑角度 + 中值修正量
    float cmd = angle_deg + g_servo_center_trim_deg;

    // 限幅在左右极限内
    float clipped = CLAMP(cmd, STEER_SERVO_LEFT_DEG, STEER_SERVO_RIGHT_DEG);

    g_servo_angle_deg = clipped;
    steer_servo_apply_duty(steer_servo_deg_to_duty(clipped));
}


void ackermann_servo_init(void)
{
    float init_cmd = STEER_SERVO_CENTER_DEG + g_servo_center_trim_deg;

    pwm_init(STEER_SERVO_PWM, STEER_SERVO_FREQ, 0);
    steer_servo_apply_duty(steer_servo_deg_to_duty(init_cmd));

    g_servo_angle_deg = init_cmd;
}



















