#ifndef _LINE_SENSOR_H_
#define _LINE_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "zf_common_headfile.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool active_low;      // 传感器“黑线=0、白色=1”则设 false；若相反设 true
    bool msb_first;       // HC165 默认输出 D7→D0（MSB first），一般设 true
} line_sensor_cfg_t;

/** 初始化 CK(输出)、SH(输出)、D1/D2(输入上拉) 并设定默认电平 */
void line_sensor_init(const line_sensor_cfg_t *cfg);

/** 并行读取两片 74HC165：
 *  返回值高 8 位来自 D2，低 8 位来自 D1。
 *  比特顺序遵循 cfg->msb_first（true：bit7 先出）
 */
uint16_t line_sensor_read16(void);

/** 将 16bit 原始值转为 0/1 数组（0~15），dst[0] 对应 D1 的 bit0（或 bit7，取决于 msb_first） */
void line_sensor_to_array(uint16_t raw16, uint8_t dst[16]);

/** 简单“求位置”示例（加权平均），返回 -7.5 ~ +7.5（单位：格）
 *  例如把左 8 个点权重设为 -7..0，右 8 个点设为 1..8，可据此做横向误差
 *  如果整排全黑/全白，返回 0，并在 *valid = false
 */
float line_sensor_position(uint16_t raw16, bool *valid);

#ifdef __cplusplus
}
#endif
#endif
