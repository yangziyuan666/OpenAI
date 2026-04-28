#ifndef _WHITE_SEED_H_
#define _WHITE_SEED_H_

#include "TrackTypes.h"

void white_seed_find(const uint8_t image[MT9V03X_H][MT9V03X_W],
                     uint8_t threshold,
                     uint8_t y_start,
                     uint8_t y_end,
                     uint8_t y_step,
                     uint8_t x_start,
                     uint8_t x_end,
                     uint8_t x_step,
                     uint8_t *best_len,
                     uint8_t *best_col);

/*
 * 阶段1观测更新：
 * 1) ROI 抽样得到动态阈值与对比度
 * 2) 搜索最长白带，输出 seed 列位置
 * 3) 生成阶段1质量标记 quality_ok
 */
void white_seed_observe_update(const uint8_t image[MT9V03X_H][MT9V03X_W],
                               stage1_observe_t *obs);

#endif
