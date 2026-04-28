#include "WhiteSeed.h"

/* 阶段1阈值观测扫描参数 */
#define STAGE1_TH_Y_START      118
#define STAGE1_TH_Y_END         60
#define STAGE1_TH_Y_STEP         2
#define STAGE1_TH_X_START       12
#define STAGE1_TH_X_END        174
#define STAGE1_TH_X_STEP         4

/* 阶段1最长白带扫描参数 */
#define STAGE1_LWC_Y_START     118
#define STAGE1_LWC_Y_END        50
#define STAGE1_LWC_Y_STEP        2
#define STAGE1_LWC_X_START      12
#define STAGE1_LWC_X_END       174
#define STAGE1_LWC_X_STEP        2

#define STAGE1_CONTRAST_MIN     20
#define STAGE1_LWC_MIN_LEN       6

void white_seed_find(const uint8_t image[MT9V03X_H][MT9V03X_W],
                     uint8_t threshold,
                     uint8_t y_start,
                     uint8_t y_end,
                     uint8_t y_step,
                     uint8_t x_start,
                     uint8_t x_end,
                     uint8_t x_step,
                     uint8_t *best_len,
                     uint8_t *best_col)
{
    uint8_t local_best_len = 0U;
    uint8_t local_best_col = 0U;

    if ((best_len == 0) || (best_col == 0) || (y_step == 0U) || (x_step == 0U))
    {
        return;
    }

    for (int y = y_start; y >= y_end; y -= y_step)
    {
        uint8_t run_len = 0U;
        int run_start = x_start;

        for (int x = x_start; x <= x_end; x += x_step)
        {
            uint8_t is_white = (image[y][x] >= threshold) ? 1U : 0U;
            if (is_white)
            {
                if (run_len == 0U)
                {
                    run_start = x;
                }

                run_len++;
                if (run_len > local_best_len)
                {
                    local_best_len = run_len;
                    local_best_col = (uint8_t)((run_start + x) >> 1);
                }
            }
            else
            {
                run_len = 0U;
            }
        }
    }

    *best_len = local_best_len;
    *best_col = local_best_col;
}

void white_seed_observe_update(const uint8_t image[MT9V03X_H][MT9V03X_W],
                               stage1_observe_t *obs)
{
    uint8_t min_v = 255U;
    uint8_t max_v = 0U;

    for (int y = STAGE1_TH_Y_START; y >= STAGE1_TH_Y_END; y -= STAGE1_TH_Y_STEP)
    {
        for (int x = STAGE1_TH_X_START; x <= STAGE1_TH_X_END; x += STAGE1_TH_X_STEP)
        {
            uint8_t v = image[y][x];
            if (v < min_v) min_v = v;
            if (v > max_v) max_v = v;
        }
    }

    obs->contrast = (uint8_t)(max_v - min_v);
    if (obs->contrast >= STAGE1_CONTRAST_MIN)
    {
        obs->threshold = (uint8_t)(((uint16_t)min_v + (uint16_t)max_v) >> 1);
        obs->threshold_valid = 1U;
    }
    else
    {
        obs->threshold_valid = 0U;
    }

    white_seed_find(image,
                    obs->threshold,
                    STAGE1_LWC_Y_START, STAGE1_LWC_Y_END, STAGE1_LWC_Y_STEP,
                    STAGE1_LWC_X_START, STAGE1_LWC_X_END, STAGE1_LWC_X_STEP,
                    &obs->lwc_len, &obs->lwc_col);

    obs->quality_ok = (uint8_t)((obs->threshold_valid && (obs->lwc_len >= STAGE1_LWC_MIN_LEN)) ? 1U : 0U);
}
