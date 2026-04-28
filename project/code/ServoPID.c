#include "ServoPID.h"
#include "all_hardware_init.h"
#include "motor.h"

static uint8_t sat_inc_u8(uint8_t v)
{
    return (v < 255U) ? (uint8_t)(v + 1U) : 255U;
}

static float servo_pid_slew_limit(float prev, float target, float step_max)
{
    float d = target - prev;
    if (d > step_max) d = step_max;
    if (d < -step_max) d = -step_max;
    return prev + d;
}

static float servo_pid_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float servo_pid_compute_confidence(const stage1_observe_t *obs,
                                          const stage2_edge_t *edge,
                                          const stage3_error_t *err)
{
    float q = servo_pid_clampf((float)obs->quality_ok / 3.0f, 0.0f, 1.0f);
    float rr = servo_pid_clampf((float)edge->robust_rows / STAGE5_CONF_RR_REF, 0.0f, 1.0f);
    float vr = servo_pid_clampf((float)edge->valid_rows / STAGE5_CONF_VR_REF, 0.0f, 1.0f);

    int width_ref = edge->width_ref;
    if (width_ref < 40 || width_ref > 180)
    {
        width_ref = 110;
    }
    int wdiff = (int)edge->width - width_ref;
    if (wdiff < 0) wdiff = -wdiff;
    float w = servo_pid_clampf(1.0f - ((float)wdiff / STAGE5_CONF_WIDTH_TOL), 0.0f, 1.0f);
    if (edge->width == 0U) w = 0.0f;

    int ej = (int)err->raw - (int)err->filt;
    if (ej < 0) ej = -ej;
    float j = servo_pid_clampf(1.0f - ((float)ej / STAGE5_CONF_EJ_REF), 0.0f, 1.0f);

    float conf = STAGE5_CONF_W_Q * q +
                 STAGE5_CONF_W_RR * rr +
                 STAGE5_CONF_W_VR * vr +
                 STAGE5_CONF_W_W * w +
                 STAGE5_CONF_W_J * j;
    if (obs->quality_ok == 0U && edge->robust_rows == 0U) conf *= 0.6f;
    return servo_pid_clampf(conf, 0.0f, 1.0f);
}

void servo_pid_control_update(const stage1_observe_t *obs,
                              const stage2_edge_t *edge,
                              const stage3_error_t *err,
                              servo_pid_ctrl_t *ctrl)
{
    static int16_t last_ef = 0;
    static uint16_t last_ctrl_tick = 0;
    static uint8_t ctrl_tick_inited = 0;
    static uint8_t ef_inited = 0;
    static uint8_t pd_curve_mode = 0;
    static uint8_t pd_enter_cnt = 0;
    static uint8_t pd_exit_cnt = 0;
    static uint8_t pd_hold_cnt = 0;
    float conf = servo_pid_compute_confidence(obs, edge, err);
    ctrl->conf_x100 = (uint8_t)(conf * 100.0f + 0.5f);

    float speed_target = STAGE5_SPEED_MIN + (STAGE5_SPEED_MAX - STAGE5_SPEED_MIN) * conf;
    int16_t ef_abs = err->filt;
    if (ef_abs < 0) ef_abs = (int16_t)(-ef_abs);

    if (ef_abs >= STAGE5_CURVE_EF_TH && speed_target > STAGE5_CURVE_SPEED_CAP)
    {
        speed_target = STAGE5_CURVE_SPEED_CAP;
    }
    if (edge->cross_active && speed_target > STAGE5_CROSS_SPEED_CAP)
    {
        speed_target = STAGE5_CROSS_SPEED_CAP;
    }
    if (conf < STAGE5_LOW_CONF_TH)
    {
        speed_target *= STAGE5_LOW_CONF_SPEED_SCALE;
    }
    uint16_t now_tick = TimerCount;
    float dt_ms = STAGE5_DERR_REF_MS;
    if (ctrl_tick_inited)
    {
        uint16_t dt_tick = (uint16_t)(now_tick - last_ctrl_tick);
        dt_ms = (float)dt_tick * STAGE5_CTRL_TICK_MS;
    }
    last_ctrl_tick = now_tick;
    ctrl_tick_inited = 1U;

    float d_err = 0.0f;
    if (ef_inited)
    {
        float dt_ms_clamped = servo_pid_clampf(dt_ms, STAGE5_DERR_DT_MIN_MS, STAGE5_DERR_DT_MAX_MS);
        float d_frame = (float)(err->filt - last_ef);
        d_err = d_frame * (STAGE5_DERR_REF_MS / dt_ms_clamped);
    }
    else
    {
        ef_inited = 1U;
    }

    {
        float de_abs = (d_err >= 0.0f) ? d_err : -d_err;
        uint8_t curve_req = (uint8_t)((ef_abs >= STAGE5_PD_ENTER_EF_TH) ||
                                      (de_abs >= STAGE5_PD_ENTER_DE_TH) ||
                                      (edge->cross_active != 0U));
        uint8_t straight_req = (uint8_t)((ef_abs <= STAGE5_PD_EXIT_EF_TH) &&
                                         (de_abs <= STAGE5_PD_EXIT_DE_TH) &&
                                         (edge->cross_active == 0U));

        if (pd_curve_mode)
        {
            if (curve_req)
            {
                pd_hold_cnt = STAGE5_PD_MIN_HOLD_FRAMES;
                pd_exit_cnt = 0U;
            }
            else
            {
                if (pd_hold_cnt > 0U) pd_hold_cnt--;
                if (pd_hold_cnt == 0U && straight_req)
                {
                    pd_exit_cnt = sat_inc_u8(pd_exit_cnt);
                    if (pd_exit_cnt >= STAGE5_PD_EXIT_FRAMES)
                    {
                        pd_curve_mode = 0U;
                        pd_exit_cnt = 0U;
                    }
                }
                else
                {
                    pd_exit_cnt = 0U;
                }
            }
            pd_enter_cnt = 0U;
        }
        else
        {
            if (curve_req)
            {
                pd_enter_cnt = sat_inc_u8(pd_enter_cnt);
                if (pd_enter_cnt >= STAGE5_PD_ENTER_FRAMES)
                {
                    pd_curve_mode = 1U;
                    pd_hold_cnt = STAGE5_PD_MIN_HOLD_FRAMES;
                    pd_enter_cnt = 0U;
                }
            }
            else
            {
                pd_enter_cnt = 0U;
            }
            pd_exit_cnt = 0U;
        }
    }

    float steer_k = pd_curve_mode ? STAGE5_STEER_KP_CURVE : STAGE5_STEER_KP_STRAIGHT;
    float steer_kd = pd_curve_mode ? STAGE5_STEER_KD_CURVE : STAGE5_STEER_KD_STRAIGHT;
    float delta_min = pd_curve_mode ? STAGE5_STEER_DELTA_MIN_CURVE : STAGE5_STEER_DELTA_MIN_STRAIGHT;
    float delta_max = pd_curve_mode ? STAGE5_STEER_DELTA_MAX_CURVE : STAGE5_STEER_DELTA_MAX_STRAIGHT;
    float steer_rate_limit = pd_curve_mode ? STAGE5_STEER_RATE_LIMIT_CURVE : STAGE5_STEER_RATE_LIMIT_STRAIGHT;
    float delta_lim = delta_min + (delta_max - delta_min) * conf;

    float delta_target = (float)err->filt * steer_k + d_err * steer_kd;
    ctrl->d_err = d_err;
    ctrl->d_t_ms = (uint16_t)(dt_ms + 0.5f);
    ctrl->delta_target_raw = delta_target;
    ctrl->curve_mode = pd_curve_mode;
    delta_target = CLAMP(delta_target, -delta_lim, delta_lim);
    float steer_target = STEER_SERVO_CENTER_DEG + delta_target;
    steer_target = CLAMP(steer_target, STEER_SERVO_LEFT_DEG, STEER_SERVO_RIGHT_DEG);

    uint8_t hard_lost = (obs->quality_ok == 0U) &&
                        (edge->valid_rows <= STAGE5_EMERGENCY_VR_MAX) &&
                        (edge->robust_rows <= STAGE5_EMERGENCY_RR_MAX);
    if (hard_lost)
    {
        ctrl->emergency_cnt = sat_inc_u8(ctrl->emergency_cnt);
    }
    else if (ctrl->emergency_cnt > 0U)
    {
        ctrl->emergency_cnt--;
    }
    ctrl->emergency_active = (ctrl->emergency_cnt >= STAGE5_EMERGENCY_HOLD_N) ? 1U : 0U;

    if (ctrl->emergency_active)
    {
        ctrl->steer_cmd_deg = servo_pid_slew_limit(ctrl->steer_cmd_deg, STEER_SERVO_CENTER_DEG, STAGE5_EMERGENCY_STEER_RATE);
        ctrl->speed_cmd = 0.0f;
    }
    else
    {
        ctrl->steer_cmd_deg = servo_pid_slew_limit(ctrl->steer_cmd_deg, steer_target, steer_rate_limit);
        ctrl->speed_cmd = servo_pid_slew_limit(ctrl->speed_cmd, speed_target, STAGE5_SPEED_RATE_LIMIT);
    }

#if ENABLE_STAGE5_GATE_CONTROL
#if ENABLE_STAGE5_SERVO_OUTPUT
    ackermann_servo_set_angle(ctrl->steer_cmd_deg);
#else
    ackermann_servo_set_angle(STEER_SERVO_CENTER_DEG);
#endif
#if ENABLE_STAGE5_MOTOR_OUTPUT
    g_target_speed_m1 = ctrl->speed_cmd;
    g_target_speed_m2 = ctrl->speed_cmd;
#else
    g_target_speed_m1 = 0.0f;
    g_target_speed_m2 = 0.0f;
#endif
#else
    ackermann_servo_set_angle(STEER_SERVO_CENTER_DEG);
    g_target_speed_m1 = 0.0f;
    g_target_speed_m2 = 0.0f;
#endif

    last_ef = err->filt;
}
