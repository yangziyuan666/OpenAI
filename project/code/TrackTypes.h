#ifndef _TRACK_TYPES_H_
#define _TRACK_TYPES_H_

#include "zf_common_headfile.h"

/*
 * 阶段1观测结果
 * ------------------------------------------------------------
 * 用途：阶段1输出给阶段2/3/5使用的基础观测量。
 */
typedef struct
{
    uint8_t threshold;       /* 当前帧二值化阈值 */
    uint8_t contrast;        /* 当前ROI对比度（max-min） */
    uint8_t threshold_valid; /* 阈值是否可信（1可信/0不可信） */
    uint8_t lwc_len;         /* 最长白带长度（采样点计数） */
    uint8_t lwc_col;         /* 最长白带中心列（seed） */
    uint8_t quality_ok;      /* 质量分级（0~3，数值越大越好） */
} stage1_observe_t;

/*
 * 阶段2找边与中线结果
 * ------------------------------------------------------------
 * 用途：阶段2输出几何信息与十字状态，供阶段3误差与阶段5控制使用。
 */
typedef struct
{
    uint8_t left_x;          /* 左边界列坐标 */
    uint8_t right_x;         /* 右边界列坐标 */
    uint8_t mid_x;           /* 中线列坐标 */
    uint8_t valid_rows;      /* 有效找边行数（VR） */
    uint8_t robust_rows;     /* 稳健行数（宽度合法，RR） */
    uint8_t width;           /* 当前宽度（right-left） */
    int8_t  lr_ratio;        /* 左右灰度比近似指标（调试量） */
    uint8_t width_ref;       /* 宽度基线参考值 */
    uint8_t edge_valid;      /* 边界结果是否可用 */
    uint8_t cross_active;    /* 十字锁存是否激活 */
    uint8_t cross_state;     /* 十字状态：NONE/PRE/IN */
    uint8_t cross_entry_mid; /* 十字入口中线参考 */
    uint8_t cross_wide_rows; /* 十字检测中的宽行计数 */
    uint8_t cross_jump_left; /* 左边跳变命中数 */
    uint8_t cross_jump_right;/* 右边跳变命中数 */
} stage2_edge_t;

/*
 * 阶段3误差结果
 * ------------------------------------------------------------
 * raw : 原始误差（mid_x - REF_X）
 * filt: 低通后的控制误差（用于舵机PD）
 */
typedef struct
{
    int16_t raw;             /* 原始误差 ER */
    int16_t filt;            /* 滤波误差 EF */
} stage3_error_t;

#endif
