#ifndef _SERVO_PID_H_
#define _SERVO_PID_H_

#include "TrackTypes.h"

#define ENABLE_STAGE5_GATE_CONTROL    1
#define ENABLE_STAGE5_SERVO_OUTPUT    1
#define ENABLE_STAGE5_MOTOR_OUTPUT    1
#define STAGE5_SPEED_MIN            2.0f
#define STAGE5_SPEED_MAX            6.0f
#define STAGE5_CURVE_EF_TH           14
#define STAGE5_CURVE_SPEED_CAP      4.5f
#define STAGE5_LOW_CONF_TH          0.35f
#define STAGE5_LOW_CONF_SPEED_SCALE 0.70f
#define STAGE5_STEER_K_LOW          0.32f
#define STAGE5_STEER_K_HIGH         0.62f
#define STAGE5_STEER_K_SWITCH_EF        8
#define STAGE5_STEER_K_CURVE_BOOST  0.10f
#define STAGE5_STEER_K_MAX_HARD     1.35f
#define STAGE5_STEER_KD             0.20f
#define STAGE5_STEER_KP_STRAIGHT    0.32f
#define STAGE5_STEER_KD_STRAIGHT    0.16f
#define STAGE5_STEER_KP_CURVE       0.75f
#define STAGE5_STEER_KD_CURVE       0.28f
#define STAGE5_CTRL_TICK_MS         2.0f
#define STAGE5_DERR_DT_MIN_MS      20.0f
#define STAGE5_DERR_DT_MAX_MS     200.0f
#define STAGE5_DERR_REF_MS        100.0f
#define STAGE5_CTRL_PERIOD_TICKS     10
#define STAGE5_STEER_DELTA_MIN_STRAIGHT  8.0f
#define STAGE5_STEER_DELTA_MAX_STRAIGHT 14.0f
#define STAGE5_STEER_DELTA_MIN_CURVE    12.0f
#define STAGE5_STEER_DELTA_MAX_CURVE    23.0f
#define STAGE5_STEER_RATE_LIMIT_STRAIGHT 0.55f
#define STAGE5_STEER_RATE_LIMIT_CURVE    0.95f
#define STAGE5_PD_ENTER_EF_TH       10
#define STAGE5_PD_ENTER_DE_TH        6.0f
#define STAGE5_PD_EXIT_EF_TH         4
#define STAGE5_PD_EXIT_DE_TH         2.0f
#define STAGE5_PD_ENTER_FRAMES       3
#define STAGE5_PD_EXIT_FRAMES        8
#define STAGE5_PD_MIN_HOLD_FRAMES   14
#define STAGE5_EMERGENCY_VR_MAX      1
#define STAGE5_EMERGENCY_RR_MAX      0
#define STAGE5_EMERGENCY_HOLD_N      8
#define STAGE5_EMERGENCY_STEER_RATE  0.8f
#define STAGE5_SPEED_RATE_LIMIT     1.0f
#define STAGE5_CROSS_SPEED_CAP      4.0f
#define STAGE5_CONF_RR_REF         22.0f
#define STAGE5_CONF_VR_REF         24.0f
#define STAGE5_CONF_WIDTH_TOL      28.0f
#define STAGE5_CONF_EJ_REF         24.0f
#define STAGE5_CONF_W_Q            0.30f
#define STAGE5_CONF_W_RR           0.25f
#define STAGE5_CONF_W_VR           0.20f
#define STAGE5_CONF_W_W            0.15f
#define STAGE5_CONF_W_J            0.10f

typedef struct
{
    float steer_cmd_deg;
    float speed_cmd;
    float d_err;
    uint16_t d_t_ms;
    float delta_target_raw;
    uint8_t curve_mode;
    uint8_t emergency_cnt;
    uint8_t emergency_active;
    uint8_t conf_x100;
} servo_pid_ctrl_t;

void servo_pid_control_update(const stage1_observe_t *obs,
                              const stage2_edge_t *edge,
                              const stage3_error_t *err,
                              servo_pid_ctrl_t *ctrl);

#endif
