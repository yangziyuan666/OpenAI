#include "SearchLine.h"
#include <string.h>

/* 阶段1到阶段2的衔接门限：白带长度不足时会降级质量 */
#define STAGE1_LWC_MIN_LEN       6

/* 扫描窗口参数（图像ROI） */
#define STAGE2_EDGE_Y_START    118
#define STAGE2_EDGE_Y_END       50
#define STAGE2_EDGE_Y_STEP       2
#define STAGE2_EDGE_X_START     12
#define STAGE2_EDGE_X_END      174
#define STAGE2_EDGE_X_STEP       2
/* 单行白带最小长度 */
#define STAGE2_RUN_MIN_LEN       4
/* 引导扫描开关（当前关闭，使用全行主扫） */
#define STAGE2_GUIDED_ENABLE      0
#define STAGE2_GUIDED_FALLBACK_FULL 1
#define STAGE2_GUIDED_HALF_EST_MIN 24
#define STAGE2_GUIDED_HALF_EST_MAX 80
#define STAGE2_GUIDED_SEARCH_MARGIN 24
#define STAGE2_GUIDED_MIN_SCORE   16
/* 左右灰度比例约束（当前关闭） */
#define STAGE2_LR_RATIO_ENABLE    0
#define STAGE2_LR_RATIO_ABS_MAX  55
/* 阈值放宽重试参数 */
#define STAGE2_RELAXED_TH_DELTA  8
#define STAGE2_RELAXED_RUN_MIN_LEN 3
/* 差比和评分约束（当前关闭） */
#define STAGE2_DRS_ENABLE           0
#define STAGE2_DRS_MIN_SCORE       50
#define STAGE2_DRS_DIST_MARGIN      3
/* 局部自适应阈值（当前关闭） */
#define STAGE2_LOCAL_ADAPT_ENABLE   0
#define STAGE2_LOCAL_X_HALF_WIN    20
#define STAGE2_LOCAL_CONTRAST_MIN  10
#define STAGE2_LOCAL_TH_BIAS        2
/* 宽度基线递推与质量分级参数 */
#define STAGE2_WIDTH_BASE_INIT    110
#define STAGE2_WIDTH_BASE_NUM       7
#define STAGE2_WIDTH_BASE_DEN       8
#define STAGE2_ANCHOR_ROW      100
#define STAGE2_ANCHOR_WIN        4
#define STAGE2_Q_ROWS_L1         3
#define STAGE2_Q_ROWS_L2        10
#define STAGE2_Q_ROWS_L3        18
#define STAGE2_WIDTH_MIN        40
#define STAGE2_WIDTH_MAX       180
/* 十字检测与状态机参数 */
#define STAGE2_CROSS_ENABLE      1
#define STAGE2_CROSS_Y_HIGH    100
#define STAGE2_CROSS_Y_LOW      70
#define STAGE2_CROSS_WIDE_MARGIN 32
#define STAGE2_CROSS_WIDE_RATIO_X100 60
#define STAGE2_CROSS_WIDE_ROWS_MIN   6
#define STAGE2_CROSS_JUMP_TH        16
#define STAGE2_CROSS_JUMP_MIN        2
#define STAGE2_CROSS_ENTER_FRAMES    3
#define STAGE2_CROSS_EXIT_FRAMES     3
#define STAGE2_CROSS_IN_HOLD_FRAMES 10
#define STAGE2_CROSS_STATE_NONE      0
#define STAGE2_CROSS_STATE_PRE       1
#define STAGE2_CROSS_STATE_IN        2

static uint8_t stage2_clip_x(int x)
{
    if (x < STAGE2_EDGE_X_START) return STAGE2_EDGE_X_START;
    if (x > STAGE2_EDGE_X_END)   return STAGE2_EDGE_X_END;
    return (uint8_t)x;
}

/* 模块内部历史状态：中线低通与宽度基线 */
static uint8_t g_stage2_mid_filt = (uint8_t)((STAGE2_EDGE_X_START + STAGE2_EDGE_X_END) >> 1);
static uint8_t g_stage2_width_base = STAGE2_WIDTH_BASE_INIT;

static uint8_t stage2_drs_u8(uint8_t a, uint8_t b)
{
    uint16_t diff = (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
    uint16_t sum1 = (uint16_t)a + (uint16_t)b + 1U;
    return (uint8_t)((diff * 255U + (sum1 >> 1)) / sum1);
}

static int16_t stage2_lr_ratio_s100(uint8_t left_v, uint8_t right_v)
{
    int16_t num = (int16_t)left_v - (int16_t)right_v;
    int16_t den = (int16_t)left_v + (int16_t)right_v + 1;
    return (int16_t)((num * 100) / den);
}

static int stage2_abs_i(int v)
{
    return (v >= 0) ? v : -v;
}

static uint8_t stage2_count_jump_hits(const uint8_t x_arr[],
                                      const uint8_t valid[],
                                      int n,
                                      int jump_th)
{
    uint8_t hits = 0U;
    for (int i = 1; i < n; i++)
    {
        if (!valid[i - 1] || !valid[i]) continue;
        if (stage2_abs_i((int)x_arr[i] - (int)x_arr[i - 1]) >= jump_th)
        {
            if (hits < 255U) hits++;
        }
    }
    return hits;
}

static uint8_t stage2_find_jump_near(const uint8_t x_arr[],
                                     const uint8_t valid[],
                                     int n,
                                     int jump_th,
                                     int *i0,
                                     int *i1)
{
    for (int i = 1; i < n; i++)
    {
        if (!valid[i - 1] || !valid[i]) continue;
        if (stage2_abs_i((int)x_arr[i] - (int)x_arr[i - 1]) >= jump_th)
        {
            *i0 = i - 1;
            *i1 = i;
            return 1U;
        }
    }
    return 0U;
}

static uint8_t stage2_find_jump_far(const uint8_t x_arr[],
                                    const uint8_t valid[],
                                    int n,
                                    int jump_th,
                                    int *i0,
                                    int *i1)
{
    for (int i = n - 1; i >= 1; i--)
    {
        if (!valid[i - 1] || !valid[i]) continue;
        if (stage2_abs_i((int)x_arr[i] - (int)x_arr[i - 1]) >= jump_th)
        {
            *i0 = i - 1;
            *i1 = i;
            return 1U;
        }
    }
    return 0U;
}

static uint8_t stage2_patch_between_jumps(uint8_t x_arr[],
                                          uint8_t valid[],
                                          int n0,
                                          int n1,
                                          int f0,
                                          int f1,
                                          uint8_t patch_mark[])
{
    int s = (n0 < n1) ? n0 : n1;
    int e = (f0 > f1) ? f0 : f1;
    if (e <= s + 1) return 0U;

    int x0 = (int)x_arr[s];
    int x1 = (int)x_arr[e];
    int den = e - s;
    uint8_t patched = 0U;

    for (int i = s + 1; i < e; i++)
    {
        int num = (x1 - x0) * (i - s);
        int xi = x0 + ((num >= 0) ? (num + den / 2) / den : (num - den / 2) / den);
        if (xi < STAGE2_EDGE_X_START) xi = STAGE2_EDGE_X_START;
        if (xi > STAGE2_EDGE_X_END) xi = STAGE2_EDGE_X_END;
        if (!valid[i])
        {
            patched = 1U;
            if (patch_mark) patch_mark[i] = 1U;
        }
        x_arr[i] = (uint8_t)xi;
        valid[i] = 1U;
    }
    return patched;
}

static uint8_t sat_inc_u8(uint8_t v)
{
    return (v < 255U) ? (uint8_t)(v + 1U) : 255U;
}

static void stage2_find_single_edges(const uint8_t row[MT9V03X_W],
                                     uint8_t threshold,
                                     uint8_t *left_found,
                                     uint8_t *right_found)
{
    /* 阶段1观测：独立估计左右边是否可见，用于统计单侧丢线计数 */
    uint8_t lf = 0U, rf = 0U;

    for (int x = STAGE2_EDGE_X_START + STAGE2_EDGE_X_STEP; x <= STAGE2_EDGE_X_END; x += STAGE2_EDGE_X_STEP)
    {
        uint8_t prv_w = (row[x - STAGE2_EDGE_X_STEP] >= threshold) ? 1U : 0U;
        uint8_t cur_w = (row[x] >= threshold) ? 1U : 0U;
        if (cur_w && !prv_w)
        {
            lf = 1U;
            break;
        }
    }

    for (int x = STAGE2_EDGE_X_END - STAGE2_EDGE_X_STEP; x >= STAGE2_EDGE_X_START; x -= STAGE2_EDGE_X_STEP)
    {
        uint8_t cur_w = (row[x] >= threshold) ? 1U : 0U;
        uint8_t nxt_w = (row[x + STAGE2_EDGE_X_STEP] >= threshold) ? 1U : 0U;
        if (cur_w && !nxt_w)
        {
            rf = 1U;
            break;
        }
    }

    *left_found = lf;
    *right_found = rf;
}

static uint8_t stage2_find_row_edge_pair(const uint8_t row[MT9V03X_W],
                                         uint8_t threshold,
                                         uint8_t seed_x,
                                         uint8_t run_min_len,
                                         uint8_t *left_x,
                                         uint8_t *right_x)
{
    /* 单行候选选择：优先距离seed近，其次边缘评分高 */
    uint8_t best_found = 0U;
    uint8_t best_len = 0U;
    uint8_t best_left = 0U;
    uint8_t best_right = 0U;
    uint16_t best_dist = 0xFFFFU;
    uint16_t best_edge_score = 0U;
    int16_t best_lr_abs = 32767;

    uint8_t in_run = 0U;
    uint8_t run_start = 0U;

    for (int x = STAGE2_EDGE_X_START; x <= STAGE2_EDGE_X_END; x += STAGE2_EDGE_X_STEP)
    {
        uint8_t white = (row[x] >= threshold) ? 1U : 0U;

        if (white && !in_run)
        {
            in_run = 1U;
            run_start = (uint8_t)x;
        }

        uint8_t is_last_pixel = (x + STAGE2_EDGE_X_STEP > STAGE2_EDGE_X_END) ? 1U : 0U;
        if ((!white || is_last_pixel) && in_run)
        {
            uint8_t run_end = (white && is_last_pixel) ? (uint8_t)x : (uint8_t)(x - STAGE2_EDGE_X_STEP);
            uint8_t run_len = (uint8_t)(((uint16_t)run_end - (uint16_t)run_start) / STAGE2_EDGE_X_STEP + 1U);
            uint8_t run_mid = (uint8_t)(((uint16_t)run_start + (uint16_t)run_end) >> 1);
            uint16_t dist = (run_mid >= seed_x) ? (uint16_t)(run_mid - seed_x) : (uint16_t)(seed_x - run_mid);
            int left_out = run_start - STAGE2_EDGE_X_STEP;
            int right_out = run_end + STAGE2_EDGE_X_STEP;
            if (left_out < STAGE2_EDGE_X_START) left_out = run_start;
            if (right_out > STAGE2_EDGE_X_END) right_out = run_end;
            uint16_t edge_score = (uint16_t)stage2_drs_u8(row[run_start], row[left_out]) +
                                  (uint16_t)stage2_drs_u8(row[run_end], row[right_out]);
            int16_t lr_ratio = stage2_lr_ratio_s100(row[run_start], row[run_end]);
            int16_t lr_abs = (lr_ratio >= 0) ? lr_ratio : (int16_t)(-lr_ratio);

            if (run_len >= run_min_len)
            {
#if STAGE2_DRS_ENABLE
                if (edge_score < STAGE2_DRS_MIN_SCORE)
                {
                    in_run = 0U;
                    continue;
                }
#endif
#if STAGE2_LR_RATIO_ENABLE
                if (lr_abs > STAGE2_LR_RATIO_ABS_MAX)
                {
                    in_run = 0U;
                    continue;
                }
#endif
                if (!best_found ||
                    (dist + STAGE2_DRS_DIST_MARGIN < best_dist) ||
                    ((dist <= best_dist + STAGE2_DRS_DIST_MARGIN) && (edge_score > best_edge_score)) ||
                    ((dist == best_dist) && (edge_score == best_edge_score) && (lr_abs < best_lr_abs)) ||
                    ((dist == best_dist) && (edge_score == best_edge_score) && (lr_abs == best_lr_abs) && (run_len > best_len)))
                {
                    best_found = 1U;
                    best_len = run_len;
                    best_dist = dist;
                    best_edge_score = edge_score;
                    best_lr_abs = lr_abs;
                    best_left = run_start;
                    best_right = run_end;
                }
            }
            in_run = 0U;
        }
    }

    if (best_found)
    {
        *left_x = best_left;
        *right_x = best_right;
        return 1U;
    }
    return 0U;
}

#if STAGE2_GUIDED_ENABLE
static uint8_t stage2_find_row_edge_pair_guided(const uint8_t row[MT9V03X_W],
                                                uint8_t threshold,
                                                uint8_t seed_x,
                                                uint8_t run_min_len,
                                                uint8_t *left_x,
                                                uint8_t *right_x)
{
    /* 基于宽度基线估计左右边位置，在小窗口内找边缘 */
    int x_min = STAGE2_EDGE_X_START;
    int x_max = STAGE2_EDGE_X_END;
    int half_est = (int)g_stage2_width_base / 2;
    if (half_est < STAGE2_GUIDED_HALF_EST_MIN) half_est = STAGE2_GUIDED_HALF_EST_MIN;
    if (half_est > STAGE2_GUIDED_HALF_EST_MAX) half_est = STAGE2_GUIDED_HALF_EST_MAX;

    int x_seed = (int)seed_x;
    if (x_seed < x_min) x_seed = x_min;
    if (x_seed > x_max) x_seed = x_max;

    int x_l_exp = x_seed - half_est;
    int x_r_exp = x_seed + half_est;
    if (x_l_exp < x_min + STAGE2_EDGE_X_STEP) x_l_exp = x_min + STAGE2_EDGE_X_STEP;
    if (x_r_exp > x_max - STAGE2_EDGE_X_STEP) x_r_exp = x_max - STAGE2_EDGE_X_STEP;

    int l0 = x_l_exp - STAGE2_GUIDED_SEARCH_MARGIN;
    int l1 = x_l_exp + STAGE2_GUIDED_SEARCH_MARGIN;
    int r0 = x_r_exp - STAGE2_GUIDED_SEARCH_MARGIN;
    int r1 = x_r_exp + STAGE2_GUIDED_SEARCH_MARGIN;
    if (l0 < x_min + STAGE2_EDGE_X_STEP) l0 = x_min + STAGE2_EDGE_X_STEP;
    if (l1 > x_max - STAGE2_EDGE_X_STEP) l1 = x_max - STAGE2_EDGE_X_STEP;
    if (r0 < x_min + STAGE2_EDGE_X_STEP) r0 = x_min + STAGE2_EDGE_X_STEP;
    if (r1 > x_max - STAGE2_EDGE_X_STEP) r1 = x_max - STAGE2_EDGE_X_STEP;

    uint8_t left_found = 0U;
    uint8_t right_found = 0U;
    uint8_t left_best = 0U;
    uint8_t right_best = 0U;
    uint8_t left_score = 0U;
    uint8_t right_score = 0U;

    for (int x = l0; x <= l1; x += STAGE2_EDGE_X_STEP)
    {
        uint8_t cur_w = (row[x] >= threshold) ? 1U : 0U;
        uint8_t prv_w = (row[x - STAGE2_EDGE_X_STEP] >= threshold) ? 1U : 0U;
        if (cur_w && !prv_w)
        {
            uint8_t score = stage2_drs_u8(row[x], row[x - STAGE2_EDGE_X_STEP]);
            if (!left_found || score > left_score)
            {
                left_found = 1U;
                left_best = (uint8_t)x;
                left_score = score;
            }
        }
    }

    for (int x = r0; x <= r1; x += STAGE2_EDGE_X_STEP)
    {
        uint8_t cur_w = (row[x] >= threshold) ? 1U : 0U;
        uint8_t nxt_w = (row[x + STAGE2_EDGE_X_STEP] >= threshold) ? 1U : 0U;
        if (cur_w && !nxt_w)
        {
            uint8_t score = stage2_drs_u8(row[x], row[x + STAGE2_EDGE_X_STEP]);
            if (!right_found || score > right_score)
            {
                right_found = 1U;
                right_best = (uint8_t)x;
                right_score = score;
            }
        }
    }

    if (!left_found || !right_found || right_best <= left_best)
    {
        return 0U;
    }

    uint8_t run_len = (uint8_t)(((uint16_t)right_best - (uint16_t)left_best) / STAGE2_EDGE_X_STEP + 1U);
    if (run_len < run_min_len)
    {
        return 0U;
    }

#if STAGE2_LR_RATIO_ENABLE
    {
        int16_t lr_ratio = stage2_lr_ratio_s100(row[left_best], row[right_best]);
        int16_t lr_abs = (lr_ratio >= 0) ? lr_ratio : (int16_t)(-lr_ratio);
        if (lr_abs > STAGE2_LR_RATIO_ABS_MAX)
        {
            return 0U;
        }
    }
#endif

#if STAGE2_DRS_ENABLE
    if ((uint8_t)(left_score + right_score) < STAGE2_GUIDED_MIN_SCORE)
    {
        return 0U;
    }
#endif

    *left_x = left_best;
    *right_x = right_best;
    return 1U;
}
#endif

#if STAGE2_LOCAL_ADAPT_ENABLE
static uint8_t stage2_row_local_threshold(const uint8_t row[MT9V03X_W],
                                          uint8_t seed_x,
                                          uint8_t global_th,
                                          uint8_t *local_th)
{
    /* 失败行局部重估阈值：仅在seed附近小窗统计min/max */
    int x0 = (int)seed_x - STAGE2_LOCAL_X_HALF_WIN;
    int x1 = (int)seed_x + STAGE2_LOCAL_X_HALF_WIN;
    if (x0 < STAGE2_EDGE_X_START) x0 = STAGE2_EDGE_X_START;
    if (x1 > STAGE2_EDGE_X_END) x1 = STAGE2_EDGE_X_END;

    uint8_t min_v = 255U;
    uint8_t max_v = 0U;
    for (int x = x0; x <= x1; x += STAGE2_EDGE_X_STEP)
    {
        uint8_t v = row[x];
        if (v < min_v) min_v = v;
        if (v > max_v) max_v = v;
    }

    uint8_t contrast = (uint8_t)(max_v - min_v);
    if (contrast < STAGE2_LOCAL_CONTRAST_MIN)
    {
        *local_th = global_th;
        return 0U;
    }

    uint8_t th = (uint8_t)(((uint16_t)min_v + (uint16_t)max_v) >> 1);
    if (th > STAGE2_LOCAL_TH_BIAS) th = (uint8_t)(th - STAGE2_LOCAL_TH_BIAS);
    *local_th = th;
    return 1U;
}
#endif

static uint8_t stage2_find_row_edge_pair_adaptive(const uint8_t row[MT9V03X_W],
                                                  uint8_t global_th,
                                                  uint8_t seed_x,
                                                  uint8_t *left_x,
                                                  uint8_t *right_x)
{
    /* 自适应顺序：主阈值 -> 放宽阈值 -> 局部阈值 -> 回退全行 */
    uint8_t found = 0U;

#if STAGE2_GUIDED_ENABLE
    found = stage2_find_row_edge_pair_guided(row, global_th, seed_x,
                                             STAGE2_RUN_MIN_LEN, left_x, right_x);
#else
    found = stage2_find_row_edge_pair(row, global_th, seed_x,
                                      STAGE2_RUN_MIN_LEN, left_x, right_x);
#endif
    if (!found && global_th > STAGE2_RELAXED_TH_DELTA)
    {
        uint8_t th_relaxed = (uint8_t)(global_th - STAGE2_RELAXED_TH_DELTA);
#if STAGE2_GUIDED_ENABLE
        found = stage2_find_row_edge_pair_guided(row, th_relaxed, seed_x,
                                                 STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
#else
        found = stage2_find_row_edge_pair(row, th_relaxed, seed_x,
                                          STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
#endif
    }

#if STAGE2_LOCAL_ADAPT_ENABLE
    if (!found)
    {
        uint8_t th_local = global_th;
        if (stage2_row_local_threshold(row, seed_x, global_th, &th_local))
        {
#if STAGE2_GUIDED_ENABLE
            found = stage2_find_row_edge_pair_guided(row, th_local, seed_x,
                                                     STAGE2_RUN_MIN_LEN, left_x, right_x);
#else
            found = stage2_find_row_edge_pair(row, th_local, seed_x,
                                              STAGE2_RUN_MIN_LEN, left_x, right_x);
#endif
            if (!found && th_local > STAGE2_RELAXED_TH_DELTA)
            {
                uint8_t th_local_relaxed = (uint8_t)(th_local - STAGE2_RELAXED_TH_DELTA);
#if STAGE2_GUIDED_ENABLE
                found = stage2_find_row_edge_pair_guided(row, th_local_relaxed, seed_x,
                                                         STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
#else
                found = stage2_find_row_edge_pair(row, th_local_relaxed, seed_x,
                                                  STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
#endif
            }
        }
    }
#endif

#if STAGE2_GUIDED_ENABLE && STAGE2_GUIDED_FALLBACK_FULL
    if (!found)
    {
        found = stage2_find_row_edge_pair(row, global_th, seed_x,
                                          STAGE2_RUN_MIN_LEN, left_x, right_x);
        if (!found && global_th > STAGE2_RELAXED_TH_DELTA)
        {
            uint8_t th_relaxed = (uint8_t)(global_th - STAGE2_RELAXED_TH_DELTA);
            found = stage2_find_row_edge_pair(row, th_relaxed, seed_x,
                                              STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
        }
#if STAGE2_LOCAL_ADAPT_ENABLE
        if (!found)
        {
            uint8_t th_local = global_th;
            if (stage2_row_local_threshold(row, seed_x, global_th, &th_local))
            {
                found = stage2_find_row_edge_pair(row, th_local, seed_x,
                                                  STAGE2_RUN_MIN_LEN, left_x, right_x);
                if (!found && th_local > STAGE2_RELAXED_TH_DELTA)
                {
                    uint8_t th_local_relaxed = (uint8_t)(th_local - STAGE2_RELAXED_TH_DELTA);
                    found = stage2_find_row_edge_pair(row, th_local_relaxed, seed_x,
                                                      STAGE2_RELAXED_RUN_MIN_LEN, left_x, right_x);
                }
            }
        }
#endif
    }
#endif

    return found;
}

void search_line_edge_update(const uint8_t image[MT9V03X_H][MT9V03X_W],
                             stage1_observe_t *obs,
                             stage2_edge_t *edge)
{
    /* seed_x：本帧找边起点，来源于阶段1白带中心 */
    uint8_t seed_x = obs->lwc_col;
    if (seed_x < STAGE2_EDGE_X_START || seed_x > STAGE2_EDGE_X_END)
    {
        seed_x = (uint8_t)((STAGE2_EDGE_X_START + STAGE2_EDGE_X_END) >> 1);
    }

    uint16_t left_sum = 0U;
    uint16_t right_sum = 0U;
    /* valid_rows: 找到边的行数；robust_rows: 宽度合法行数 */
    uint8_t valid_rows = 0U;
    uint8_t robust_rows = 0U;
    int16_t lr_ratio_sum = 0;
    uint8_t lr_ratio_cnt = 0U;
    uint8_t anchor_found = 0U;
    uint8_t anchor_left = 0U;
    uint8_t anchor_right = 0U;
    uint8_t l_loss_rows = 0U;
    uint8_t r_loss_rows = 0U;

    /* 扫描缓存：rl/rr=左右边，rf=是否有效，ry=行号 */
#define SCAN_MAX 36
    uint8_t rl[SCAN_MAX], rr[SCAN_MAX], rf[SCAN_MAX];
    int ry[SCAN_MAX];
    int sc = 0;

    {
        uint8_t sx = seed_x;
        for (int y = STAGE2_EDGE_Y_START; y >= STAGE2_EDGE_Y_END; y -= STAGE2_EDGE_Y_STEP)
        {
            if (sc >= SCAN_MAX) break;
            uint8_t lx = 0U, rx = 0U;
            uint8_t l_found = 0U, r_found = 0U;

            stage2_find_single_edges(image[y], obs->threshold, &l_found, &r_found);
            if (l_found && !r_found) r_loss_rows = sat_inc_u8(r_loss_rows);
            if (r_found && !l_found) l_loss_rows = sat_inc_u8(l_loss_rows);

            uint8_t f = stage2_find_row_edge_pair_adaptive(image[y], obs->threshold, sx, &lx, &rx);
            rl[sc] = lx; rr[sc] = rx; rf[sc] = f; ry[sc] = y;
            if (f) sx = (uint8_t)(((uint16_t)lx + (uint16_t)rx) >> 1);
            sc++;
        }
    }

#if STAGE2_CROSS_ENABLE
    {
        static uint8_t cross_latched = 0U;
        static uint8_t cross_enter_cnt = 0U;
        static uint8_t cross_exit_cnt = 0U;
        static uint8_t cross_prev_active = 0U;
        static uint8_t cross_in_hold_cnt = 0U;
        static uint8_t cross_entry_mid = (uint8_t)((STAGE2_EDGE_X_START + STAGE2_EDGE_X_END) >> 1);
        uint8_t valid_rows_cross = 0U;
        uint8_t wide_rows = 0U;
        /* 十字宽门限：在宽度基线基础上加余量 */
        int width_gate = (int)g_stage2_width_base + STAGE2_CROSS_WIDE_MARGIN;
        if (width_gate < STAGE2_WIDTH_MIN) width_gate = STAGE2_WIDTH_MIN;
        if (width_gate > STAGE2_WIDTH_MAX) width_gate = STAGE2_WIDTH_MAX;

        for (int i = 0; i < sc; i++)
        {
            if (!rf[i]) continue;
            int y = ry[i];
            if (y > STAGE2_CROSS_Y_HIGH || y < STAGE2_CROSS_Y_LOW) continue;
            valid_rows_cross++;
            if ((int)rr[i] - (int)rl[i] >= width_gate) wide_rows++;
        }

        uint8_t jump_l = stage2_count_jump_hits(rl, rf, sc, STAGE2_CROSS_JUMP_TH);
        uint8_t jump_r = stage2_count_jump_hits(rr, rf, sc, STAGE2_CROSS_JUMP_TH);
        uint8_t cross_cand = 0U;
        if ((valid_rows_cross >= STAGE2_CROSS_WIDE_ROWS_MIN) &&
            ((uint16_t)wide_rows * 100U >= (uint16_t)valid_rows_cross * STAGE2_CROSS_WIDE_RATIO_X100) &&
            (jump_l >= STAGE2_CROSS_JUMP_MIN) &&
            (jump_r >= STAGE2_CROSS_JUMP_MIN))
        {
            cross_cand = 1U;
        }

        if (cross_cand)
        {
            cross_exit_cnt = 0U;
            cross_enter_cnt = sat_inc_u8(cross_enter_cnt);
            if (cross_enter_cnt >= STAGE2_CROSS_ENTER_FRAMES) cross_latched = 1U;
        }
        else
        {
            cross_enter_cnt = 0U;
        }

        edge->cross_wide_rows = wide_rows;
        edge->cross_jump_left = jump_l;
        edge->cross_jump_right = jump_r;
        edge->cross_entry_mid = cross_entry_mid;
        edge->cross_state = STAGE2_CROSS_STATE_NONE;

        if (cross_latched)
        {
            if (!cross_prev_active)
            {
                cross_entry_mid = g_stage2_mid_filt;
                cross_in_hold_cnt = STAGE2_CROSS_IN_HOLD_FRAMES;
            }

            if (jump_l >= STAGE2_CROSS_JUMP_MIN && jump_r >= STAGE2_CROSS_JUMP_MIN)
            {
                edge->cross_state = STAGE2_CROSS_STATE_PRE;
                cross_entry_mid = g_stage2_mid_filt;
                edge->cross_entry_mid = cross_entry_mid;
                cross_in_hold_cnt = STAGE2_CROSS_IN_HOLD_FRAMES;
            }
            else
            {
                edge->cross_state = STAGE2_CROSS_STATE_IN;
                if (cross_in_hold_cnt > 0U) cross_in_hold_cnt--;
            }

            if (!cross_cand)
            {
                edge->cross_state = STAGE2_CROSS_STATE_IN;
                if (cross_in_hold_cnt > 0U)
                {
                    cross_exit_cnt = 0U;
                }
                else
                {
                    cross_exit_cnt = sat_inc_u8(cross_exit_cnt);
                    if (cross_exit_cnt >= STAGE2_CROSS_EXIT_FRAMES)
                    {
                        cross_latched = 0U;
                        edge->cross_state = STAGE2_CROSS_STATE_NONE;
                        cross_exit_cnt = 0U;
                    }
                }
            }

            int ln0 = 0, ln1 = 0, lf0 = 0, lf1 = 0;
            int rn0 = 0, rn1 = 0, rf0_i = 0, rf1_i = 0;
            uint8_t l_near = stage2_find_jump_near(rl, rf, sc, STAGE2_CROSS_JUMP_TH, &ln0, &ln1);
            uint8_t l_far  = stage2_find_jump_far (rl, rf, sc, STAGE2_CROSS_JUMP_TH, &lf0, &lf1);
            uint8_t r_near = stage2_find_jump_near(rr, rf, sc, STAGE2_CROSS_JUMP_TH, &rn0, &rn1);
            uint8_t r_far  = stage2_find_jump_far (rr, rf, sc, STAGE2_CROSS_JUMP_TH, &rf0_i, &rf1_i);
            if (edge->cross_state == STAGE2_CROSS_STATE_PRE)
            {
                if (l_near && l_far) stage2_patch_between_jumps(rl, rf, ln0, ln1, lf0, lf1, 0);
                if (r_near && r_far) stage2_patch_between_jumps(rr, rf, rn0, rn1, rf0_i, rf1_i, 0);
            }
        }
        else
        {
            edge->cross_state = STAGE2_CROSS_STATE_NONE;
            cross_exit_cnt = 0U;
        }
        edge->cross_active = cross_latched;
        cross_prev_active = cross_latched;
    }
#else
    edge->cross_active = 0U;
    edge->cross_state = STAGE2_CROSS_STATE_NONE;
    edge->cross_entry_mid = (uint8_t)((STAGE2_EDGE_X_START + STAGE2_EDGE_X_END) >> 1);
    edge->cross_wide_rows = 0U;
    edge->cross_jump_left = 0U;
    edge->cross_jump_right = 0U;
#endif

    for (int i = 0; i < sc; i++)
    {
        if (!rf[i]) continue;
        uint8_t left_x = rl[i], right_x = rr[i];

        seed_x = (uint8_t)(((uint16_t)left_x + (uint16_t)right_x) >> 1);
        valid_rows++;

        uint8_t span = (right_x > left_x) ? (uint8_t)(right_x - left_x) : 0U;
        if (span >= STAGE2_WIDTH_MIN && span <= STAGE2_WIDTH_MAX)
        {
            robust_rows++;
            left_sum += left_x;
            right_sum += right_x;
            lr_ratio_sum += stage2_lr_ratio_s100(image[ry[i]][left_x], image[ry[i]][right_x]);
            if (lr_ratio_cnt < 255U) lr_ratio_cnt++;

            if (!anchor_found)
            {
                int dy = ry[i] - STAGE2_ANCHOR_ROW;
                if (dy < 0) dy = -dy;
                if (dy <= STAGE2_ANCHOR_WIN)
                {
                    anchor_found = 1U;
                    anchor_left = left_x;
                    anchor_right = right_x;
                }
            }
        }
    }
#undef SCAN_MAX

    edge->valid_rows = valid_rows;
    edge->robust_rows = robust_rows;
    edge->edge_valid = (robust_rows >= STAGE2_Q_ROWS_L1) ? 1U : 0U;
    edge->lr_ratio = (lr_ratio_cnt > 0U) ? (int8_t)(lr_ratio_sum / (int16_t)lr_ratio_cnt) : 0;

    if (anchor_found)
    {
        edge->left_x = anchor_left;
        edge->right_x = anchor_right;
    }
    else if (robust_rows > 0U)
    {
        edge->left_x = (uint8_t)(left_sum / robust_rows);
        edge->right_x = (uint8_t)(right_sum / robust_rows);
    }
    else
    {
        edge->left_x = 0U;
        edge->right_x = 0U;
    }

    if (edge->edge_valid && edge->right_x > edge->left_x)
    {
        /* 中线低通，减少帧间抖动 */
        uint8_t raw_mid = (uint8_t)(((uint16_t)edge->left_x + (uint16_t)edge->right_x) >> 1);
        g_stage2_mid_filt = (uint8_t)(((uint16_t)g_stage2_mid_filt * 3U + (uint16_t)raw_mid + 2U) >> 2);
        edge->mid_x = g_stage2_mid_filt;
        edge->width = (uint8_t)(edge->right_x - edge->left_x);
    }
    else
    {
        /* 边失效时向阶段1中心缓慢回退，避免中线突跳 */
        g_stage2_mid_filt = (uint8_t)(((uint16_t)g_stage2_mid_filt * 3U + (uint16_t)obs->lwc_col + 2U) >> 2);
        edge->mid_x = g_stage2_mid_filt;
        edge->width = 0U;
    }

    if ((robust_rows >= STAGE2_Q_ROWS_L2) &&
        (edge->width >= STAGE2_WIDTH_MIN) &&
        (edge->width <= STAGE2_WIDTH_MAX))
    {
        /* 仅在中高质量帧更新宽度基线 */
        g_stage2_width_base = (uint8_t)((STAGE2_WIDTH_BASE_NUM * (uint16_t)g_stage2_width_base +
                                         (uint16_t)edge->width + 1U) / STAGE2_WIDTH_BASE_DEN);
    }
    edge->width_ref = g_stage2_width_base;

    if (edge->cross_state == STAGE2_CROSS_STATE_IN)
    {
        int mid = (int)edge->cross_entry_mid;
        int half = (int)edge->width_ref / 2;
        if (half < 18) half = 18;
        int lx = mid - half;
        int rx = mid + half;
        if (lx < STAGE2_EDGE_X_START) lx = STAGE2_EDGE_X_START;
        if (rx > STAGE2_EDGE_X_END) rx = STAGE2_EDGE_X_END;
        if (rx <= lx) rx = lx + 1;
        edge->left_x = (uint8_t)lx;
        edge->right_x = (uint8_t)rx;
        edge->mid_x = (uint8_t)(((uint16_t)edge->left_x + (uint16_t)edge->right_x) >> 1);
        edge->width = (uint8_t)(edge->right_x - edge->left_x);
        edge->edge_valid = 1U;
    }

    if (!obs->threshold_valid || obs->lwc_len < STAGE1_LWC_MIN_LEN)
    {
        obs->quality_ok = 0U;
    }
    else if (robust_rows >= STAGE2_Q_ROWS_L3)
    {
        obs->quality_ok = 3U;
    }
    else if (robust_rows >= STAGE2_Q_ROWS_L2)
    {
        obs->quality_ok = 2U;
    }
    else if (robust_rows >= STAGE2_Q_ROWS_L1)
    {
        obs->quality_ok = 1U;
    }
    else
    {
        obs->quality_ok = 0U;
    }

    if (edge->cross_state == STAGE2_CROSS_STATE_IN && obs->quality_ok == 0U)
    {
        obs->quality_ok = 1U;
}
}
void search_line_fill_vcm_for_overlay(const uint8_t image[MT9V03X_H][MT9V03X_W],
                                      const stage1_observe_t *obs,
                                      const stage2_edge_t *edge,
                                      vcm_result_t *vcm)
{
    /* 叠线路径复用阶段2找边，保证显示与控制一致 */
    uint16_t valid_rows = 0U;
    uint8_t seed_x = edge->mid_x;
    if (seed_x < STAGE2_EDGE_X_START || seed_x > STAGE2_EDGE_X_END)
    {
        seed_x = (uint8_t)((STAGE2_EDGE_X_START + STAGE2_EDGE_X_END) >> 1);
    }

    memset(vcm->left_edge, 0, sizeof(vcm->left_edge));
    memset(vcm->right_edge, 0, sizeof(vcm->right_edge));
    memset(vcm->midline, 0, sizeof(vcm->midline));
    memset(vcm->row_valid, 0, sizeof(vcm->row_valid));
    memset(vcm->row_edge_type, 0, sizeof(vcm->row_edge_type));

#define SCAN_MAX_O 36
    uint8_t rl_o[SCAN_MAX_O], rr_o[SCAN_MAX_O], rf_o[SCAN_MAX_O], rp_o[SCAN_MAX_O];
    int ry_o[SCAN_MAX_O];
    int sc_o = 0;

    {
        uint8_t sx = seed_x;
        for (int y = STAGE2_EDGE_Y_START; y >= STAGE2_EDGE_Y_END; y -= STAGE2_EDGE_Y_STEP)
        {
            if (sc_o >= SCAN_MAX_O) break;
            uint8_t lx = 0U, rx = 0U;
            uint8_t f = stage2_find_row_edge_pair_adaptive(image[y], obs->threshold, sx, &lx, &rx);
            rl_o[sc_o] = lx; rr_o[sc_o] = rx; rf_o[sc_o] = f; rp_o[sc_o] = 0U; ry_o[sc_o] = y;
            if (f) sx = (uint8_t)(((uint16_t)lx + (uint16_t)rx) >> 1);
            sc_o++;
        }
    }

    if (edge->cross_state == STAGE2_CROSS_STATE_PRE)
    {
        int ln0 = 0, ln1 = 0, lf0 = 0, lf1 = 0;
        int rn0 = 0, rn1 = 0, rf0_i = 0, rf1_i = 0;
        uint8_t l_near = stage2_find_jump_near(rl_o, rf_o, sc_o, STAGE2_CROSS_JUMP_TH, &ln0, &ln1);
        uint8_t l_far  = stage2_find_jump_far (rl_o, rf_o, sc_o, STAGE2_CROSS_JUMP_TH, &lf0, &lf1);
        uint8_t r_near = stage2_find_jump_near(rr_o, rf_o, sc_o, STAGE2_CROSS_JUMP_TH, &rn0, &rn1);
        uint8_t r_far  = stage2_find_jump_far (rr_o, rf_o, sc_o, STAGE2_CROSS_JUMP_TH, &rf0_i, &rf1_i);
        if (l_near && l_far) stage2_patch_between_jumps(rl_o, rf_o, ln0, ln1, lf0, lf1, rp_o);
        if (r_near && r_far) stage2_patch_between_jumps(rr_o, rf_o, rn0, rn1, rf0_i, rf1_i, rp_o);
    }
    else if (edge->cross_state == STAGE2_CROSS_STATE_IN)
    {
        int mid = (int)edge->cross_entry_mid;
        int half = (int)edge->width_ref / 2;
        if (half < 18) half = 18;
        int lx = mid - half;
        int rx = mid + half;
        if (lx < STAGE2_EDGE_X_START) lx = STAGE2_EDGE_X_START;
        if (rx > STAGE2_EDGE_X_END) rx = STAGE2_EDGE_X_END;
        if (rx <= lx) rx = lx + 1;
        for (int i = 0; i < sc_o; i++)
        {
            rl_o[i] = (uint8_t)lx;
            rr_o[i] = (uint8_t)rx;
            rf_o[i] = 1U;
            rp_o[i] = 1U;
        }
    }

    for (int i = 0; i < sc_o; i++)
    {
        if (!rf_o[i]) continue;
        uint8_t left_x = rl_o[i], right_x = rr_o[i];
        int y = ry_o[i];

        seed_x = (uint8_t)(((uint16_t)left_x + (uint16_t)right_x) >> 1);
        uint8_t span = (right_x > left_x) ? (uint8_t)(right_x - left_x) : 0U;
        if (span >= STAGE2_WIDTH_MIN && span <= STAGE2_WIDTH_MAX)
        {
            vcm->left_edge[y] = left_x;
            vcm->right_edge[y] = right_x;
            vcm->midline[y] = (uint16_t)(((uint16_t)left_x + (uint16_t)right_x) >> 1);
            vcm->row_valid[y] = 1U;
            vcm->row_edge_type[y] = rp_o[i] ? 2U : 1U;
            valid_rows++;
        }
    }
#undef SCAN_MAX_O

    vcm->valid_rows = valid_rows;
    vcm->both_edge_rows = valid_rows;
    vcm->single_edge_rows = 0U;
    vcm->line_lost = (valid_rows < STAGE2_Q_ROWS_L1) ? 1U : 0U;
}
