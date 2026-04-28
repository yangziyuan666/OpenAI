#ifndef _MIDDLINE_ERROR_H_
#define _MIDDLINE_ERROR_H_

#include "TrackTypes.h"

/*
 * 阶段3误差更新：
 * raw  = mid_x - REF_X
 * filt = raw 的一阶低通（低质量时缓慢回零）
 */
void middline_error_update(const stage1_observe_t *obs,
                           const stage2_edge_t *edge,
                           stage3_error_t *err);

#endif
