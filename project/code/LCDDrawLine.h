#ifndef _LCD_DRAW_LINE_H_
#define _LCD_DRAW_LINE_H_

#include "zf_common_headfile.h"

/*
 * 当前仅保留“LCD叠线显示 + 边界结果容器”所需定义。
 * 不再保留旧 tv_process/vcm_process 算法接口。
 */

#define VCM_IMG_W               (MT9V03X_W)
#define VCM_IMG_H               (MT9V03X_H)

#define VCM_ROI_Y_START         30
#define VCM_ROI_Y_END           (VCM_IMG_H - 1)
#define VCM_ANCHOR_ROW          100
#define VCM_ERR_NEIGHBOR_RADIUS 5

#define VCM_CRED_INVALID        0
#define VCM_CRED_WEAK           1
#define VCM_CRED_ACCEPTABLE     2
#define VCM_CRED_STRONG         3

typedef struct
{
    uint16_t left_edge [VCM_IMG_H];
    uint16_t right_edge[VCM_IMG_H];
    uint16_t midline   [VCM_IMG_H];
    uint8_t  row_valid [VCM_IMG_H];

    uint16_t valid_rows;
    uint16_t both_edge_rows;
    uint16_t single_edge_rows;
    uint8_t  line_lost;

    uint8_t  row_edge_type[VCM_IMG_H];

    /* 与当前主流程兼容保留：阶段2会写入该字段 */
    uint16_t lwc_col;
} vcm_result_t;

#endif
