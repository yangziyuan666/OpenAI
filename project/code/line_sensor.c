#include "line_sensor.h"
#include "all_hardware_init.h"
static line_sensor_cfg_t s_cfg = {
    .active_low = false,
    .msb_first  = true
};

// 微秒级延时，BSP 一般有 system_delay_us；若没有可换成少量 NOP
static inline void _us_delay(uint32_t us) { system_delay_us(us); }

static inline void _ck_low(void)  { gpio_set_level(LINE_CK_PIN, GPIO_LOW);  }
static inline void _ck_high(void) { gpio_set_level(LINE_CK_PIN, GPIO_HIGH); }
static inline void _sh_low(void)  { gpio_set_level(LINE_SH_PIN, GPIO_LOW);  }
static inline void _sh_high(void) { gpio_set_level(LINE_SH_PIN, GPIO_HIGH); }

void line_sensor_init(const line_sensor_cfg_t *cfg)
{
    if (cfg) s_cfg = *cfg;

    // D1/D2 输入，上拉（避免悬空毛刺）
    gpio_init(LINE_D1_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(LINE_D2_PIN, GPI, GPIO_HIGH, GPI_PULL_UP);

    // CK、SH 输出，推挽
    gpio_init(LINE_CK_PIN, GPO, GPIO_LOW,  GPO_PUSH_PULL);
    gpio_init(LINE_SH_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL); // SH 高：移位态

    // 复位到“安全空闲”：CK=0，SH=1
    _ck_low();
    _sh_high();
    _us_delay(2);
}

static inline uint8_t _sample_two_8bits(uint8_t *byte_d1, uint8_t *byte_d2)
{
    uint8_t b1 = 0, b2 = 0;

    if (s_cfg.msb_first) {
        // D7..D0 依次输出到 Q7
        for (int i = 7; i >= 0; --i) {
            _ck_high();
            _us_delay(1); // 给 165 输出稳定时间
            uint8_t bit1 = gpio_get_level(LINE_D1_PIN) ? 1 : 0;
            uint8_t bit2 = gpio_get_level(LINE_D2_PIN) ? 1 : 0;
            _ck_low();

            if (s_cfg.active_low) { bit1 ^= 1; bit2 ^= 1; }
            b1 |= (bit1 << i);
            b2 |= (bit2 << i);
        }
    } else {
        // LSB first（极少用）
        for (int i = 0; i < 8; ++i) {
            _ck_high();
            _us_delay(1);
            uint8_t bit1 = gpio_get_level(LINE_D1_PIN) ? 1 : 0;
            uint8_t bit2 = gpio_get_level(LINE_D2_PIN) ? 1 : 0;
            _ck_low();

            if (s_cfg.active_low) { bit1 ^= 1; bit2 ^= 1; }
            b1 |= (bit1 << i);
            b2 |= (bit2 << i);
        }
    }

    *byte_d1 = b1;
    *byte_d2 = b2;
    return 0;
}

uint16_t line_sensor_read16(void)
{
    // 并行加载：SH 置低加载并行输入，再回到高电平进入移位态
    _sh_low();
    _us_delay(1);     // /PL 宽度
    _sh_high();
    _us_delay(1);

    uint8_t d1 = 0, d2 = 0;
    _sample_two_8bits(&d1, &d2);

    // 合并：高 8 位来自 D2，低 8 位来自 D1
    return ( ((uint16_t)d2) << 8 ) | d1;
}

void line_sensor_to_array(uint16_t raw16, uint8_t dst[16])
{
    // 把 16 位展开为数组，便于调试显示
    for (int i = 0; i < 16; ++i) {
        // 默认把 bit0 放在 dst[0]，bit15 放在 dst[15]
        dst[i] = (raw16 >> i) & 0x1;
    }
}

float line_sensor_position(uint16_t raw16, bool *valid)
{
    // 以 16 个探头，权重 -7..+8（左负右正），忽略全 0/全 1 情况
    static const int8_t weight[16] =
        {-7,-6,-5,-4,-3,-2,-1, 0, 1, 2, 3, 4, 5, 6, 7, 8};

    int sum_w = 0;
    int cnt   = 0;
    for (int i = 0; i < 16; ++i) {
        if ((raw16 >> i) & 0x1) {
            sum_w += weight[i];
            cnt++;
        }
    }
    if (cnt == 0 || cnt == 16) {
        if (valid) *valid = false;
        return 0.0f;
    }
    if (valid) *valid = true;
    return (float)sum_w / (float)cnt;  // -7.5 ~ +7.5
}
