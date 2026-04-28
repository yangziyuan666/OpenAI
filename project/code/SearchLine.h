#ifndef _SEARCH_LINE_H_
#define _SEARCH_LINE_H_

#include "TrackTypes.h"
#include "LCDDrawLine.h"

/* 阶段2：逐行找边、十字处理、聚合中线 */
void search_line_edge_update(const uint8_t image[MT9V03X_H][MT9V03X_W],
                             stage1_observe_t *obs,
                             stage2_edge_t *edge);

/* 阶段2：将边线结果写入 VCM 叠线显示结构 */
void search_line_fill_vcm_for_overlay(const uint8_t image[MT9V03X_H][MT9V03X_W],
                                      const stage1_observe_t *obs,
                                      const stage2_edge_t *edge,
                                      vcm_result_t *vcm);

#endif
