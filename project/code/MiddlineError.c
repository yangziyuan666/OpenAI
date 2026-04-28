#include "MiddlineError.h"

/* 阶段3误差基准：图像参考中心列与误差限幅 */
#define STAGE3_ERR_REF_X       104
#define STAGE3_ERR_CLAMP        80

void middline_error_update(const stage1_observe_t *obs,
                           const stage2_edge_t *edge,
                           stage3_error_t *err)
{
    /* raw：当前中线相对参考中心的瞬时偏差 */
    int16_t raw = (int16_t)edge->mid_x - (int16_t)STAGE3_ERR_REF_X;
    if (raw > STAGE3_ERR_CLAMP) raw = STAGE3_ERR_CLAMP;
    if (raw < -STAGE3_ERR_CLAMP) raw = -STAGE3_ERR_CLAMP;
    err->raw = raw;

    if (obs->quality_ok == 0U)
    {
        /* 低质量帧：滤波误差缓慢回零，避免坏帧造成误差突跳 */
        int32_t t = (int32_t)err->filt * 7;
        if (t >= 0) t += 4; else t -= 4;
        err->filt = (int16_t)(t / 8);
    }
    else
    {
        /* 正常帧：一阶低通 filt = 0.75 * filt + 0.25 * raw */
        int32_t t = (int32_t)err->filt * 3 + (int32_t)raw;
        if (t >= 0) t += 2; else t -= 2;
        err->filt = (int16_t)(t / 4);
    }
}
