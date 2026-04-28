/*********************************************************************************************************************
* 文件: main_cm7_0.c
* 说明: 主循环框架（采图 + 阶段1/2/3视觉 + 阶段4控制 + LCD/WIFI可视化）。
* 注释编码: UTF-8 with BOM（避免中文乱码）。
*********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "all_hardware_init.h"
#include "motor.h"
#include "MotorPID.h"
#include "imu_fusion.h"
#include "LCDDrawLine.h"
#include "WhiteSeed.h"
#include "SearchLine.h"
#include "MiddlineError.h"
#include "ServoPID.h"
#include "zf_device_ips200pro.h"
#include <stdio.h>

#define ENABLE_WIFI_VISION      0

#if ENABLE_WIFI_VISION
#define INCLUDE_BOUNDARY_TYPE   1
#define WIFI_SSID_TEST          "tianxuan4"
#define WIFI_PASSWORD_TEST      "17324705812"
#define TCP_TARGET_IP           "192.168.137.1"
#define TCP_TARGET_PORT         "8080"
#define WIFI_LOCAL_PORT         "6666"
#define WIFI_SEND_FRAME_DIV     2
#endif

//==================================================================

//==================================================================
#if ENABLE_WIFI_VISION
static uint8  image_copy[MT9V03X_H][MT9V03X_W];

static uint8  x1_boundary[MT9V03X_H];
static uint8  x2_boundary[MT9V03X_H];
static uint8  x3_boundary[MT9V03X_H];

static uint8  g_wifi_ready       = 0;
static uint8  g_wifi_send_div    = 0;
#endif

static uint16 g_pro_image_id   = 0;
static uint16 g_pro_lbl_fps    = 0;
static uint16 g_pro_lbl_err    = 0;
static uint16 g_pro_lbl_cross  = 0;
static uint16 g_pro_lbl_init   = 0;

static ips200pro_image_line_uint16_struct s_left_overlay[VCM_IMG_H];
static ips200pro_image_line_uint16_struct s_right_overlay[VCM_IMG_H];
static ips200pro_image_line_uint16_struct s_mid_overlay[VCM_IMG_H];
static ips200pro_image_line_uint16_struct s_mid_single_overlay[VCM_IMG_H];

//====================================

//====================================
#define LOG_HEARTBEAT           0

//====================================

//====================================
#define ENABLE_LCD_FULL_VIS     1
#define LCD_TEXT_FRAME_DIV      8
#define ENABLE_LCD_OVERLAY      1

#define STAGE1_UART_PRINT_ENABLE 1
#define STAGE1_UART_FRAME_DIV   10

//====================================
#define ENABLE_H0_SERVO_TEST    0
#define H0_STEP_MS              1000
#define H0_STEP_TICKS           (H0_STEP_MS / 2)

//====================================

//====================================
#define ENABLE_J0_FORCED_ANGLE  0
#define J0_ANGLE_A              90.0f
#define J0_ANGLE_B              90.0f
#define J0_TOGGLE_MS            1000
#define J0_TOGGLE_TICKS         (J0_TOGGLE_MS / 2)

//====================================

vcm_result_t     g_vcm_result;

static stage1_observe_t g_stage1_obs = {128U, 0U, 0U, 0U, 0U, 0U};
static stage2_edge_t    g_stage2_edge = {0};
static stage3_error_t   g_stage3_err = {0, 0};
static servo_pid_ctrl_t g_stage5 = {STEER_SERVO_CENTER_DEG, 0.0f, 0.0f, 0U, 0.0f, 0U, 0U, 0U, 0U};


#define COLOR_RED      0xF800
#define COLOR_GREEN    0x07E0
#define COLOR_BLUE     0x001F
#define COLOR_YELLOW   0xFFE0

#if ENABLE_WIFI_VISION
static void build_boundary_from_vcm_edge(const uint16_t edge[VCM_IMG_H],
                                         uint8 x_boundary[MT9V03X_H])
{
    memset(x_boundary, 0, MT9V03X_H);
    for (uint16_t y = 0; y < MT9V03X_H; y++)
    {
        uint16_t x = edge[y];
        if (x == 0) continue;
        if (x >= MT9V03X_W) x = MT9V03X_W - 1;
        x_boundary[y] = (uint8)x;
    }
}

static void build_boundary_from_vcm_midline(const uint16_t mid[VCM_IMG_H],
                                            const uint8_t  valid[VCM_IMG_H],
                                            uint8 x_boundary[MT9V03X_H])
{
    memset(x_boundary, 0, MT9V03X_H);
    for (uint16_t y = 0; y < MT9V03X_H; y++)
    {
        if (!valid[y]) continue;
        uint16_t x = mid[y];
        if (x >= MT9V03X_W) x = MT9V03X_W - 1;
        x_boundary[y] = (uint8)x;
    }
}

//==================================================================

//==================================================================
static void wifi_vision_init(void)
{

    {
        const int wifi_retry_max = 30;
        int ok = 0;
        for (int i = 0; i < wifi_retry_max; i++)
        {
            if (!wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
            {
                ok = 1;
                break;
            }
            printf("\r\n connect wifi failed. retry=%d/%d", i + 1, wifi_retry_max);
            system_delay_ms(100);
        }
        if (!ok)
        {
            g_wifi_ready = 0;
            wprintf("[WARN] WIFI init failed, continue without WIFI vision.\r\n");
            return;
        }
    }

    printf("\r\n module version:%s", wifi_spi_version);
    printf("\r\n module mac    :%s", wifi_spi_mac_addr);
    printf("\r\n module ip     :%s", wifi_spi_ip_addr_port);

#if (WIFI_SPI_AUTO_CONNECT != 1)
    {
        const int tcp_retry_max = 30;
        int ok = 0;
        for (int i = 0; i < tcp_retry_max; i++)
        {
            if (!wifi_spi_socket_connect("TCP",
                                         TCP_TARGET_IP,
                                         TCP_TARGET_PORT,
                                         WIFI_LOCAL_PORT))
            {
                ok = 1;
                break;
            }
            printf("\r\n Connect TCP Servers error, retry=%d/%d", i + 1, tcp_retry_max);
            system_delay_ms(100);
        }
        if (!ok)
        {
            g_wifi_ready = 0;
            wprintf("[WARN] TCP connect failed, continue without WIFI vision.\r\n");
            return;
        }
    }
#endif

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
#if (INCLUDE_BOUNDARY_TYPE == 0)

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        MT9V03X_W,
        MT9V03X_H);

#elif (INCLUDE_BOUNDARY_TYPE == 1)

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        image_copy[0],
        MT9V03X_W,
        MT9V03X_H);

    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        MT9V03X_H,
        x1_boundary,
        x2_boundary,
        x3_boundary,
        NULL, NULL, NULL);

#elif (INCLUDE_BOUNDARY_TYPE == 4)

    seekfree_assistant_camera_information_config(
        SEEKFREE_ASSISTANT_MT9V03X,
        NULL,
        MT9V03X_W,
        MT9V03X_H);

    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY,
        MT9V03X_H,
        x1_boundary,
        x2_boundary,
        x3_boundary,
        NULL, NULL, NULL);
#endif

    g_wifi_ready    = 1;
    g_wifi_send_div = 0;
}
//==================================================================

//==================================================================
static void wifi_vision_send_frame(const uint8 src_img[MT9V03X_H][MT9V03X_W],
                                   const vcm_result_t *vcm)
{
    if (!g_wifi_ready)
        return;

    g_wifi_send_div++;
    if (g_wifi_send_div < WIFI_SEND_FRAME_DIV)
        return;
    g_wifi_send_div = 0;

    memcpy(image_copy[0], src_img[0], MT9V03X_IMAGE_SIZE);

#if (INCLUDE_BOUNDARY_TYPE == 1 || INCLUDE_BOUNDARY_TYPE == 4)
    build_boundary_from_vcm_edge(vcm->left_edge,  x1_boundary);
    build_boundary_from_vcm_edge(vcm->right_edge, x2_boundary);
    build_boundary_from_vcm_midline(vcm->midline, vcm->row_valid, x3_boundary);
#endif

    seekfree_assistant_camera_send();
}
// ==================================================================
#endif

// ==================================================================
static void visualize_track_on_screen_pro(const uint8 image[MT9V03X_H][MT9V03X_W],
                                          const vcm_result_t *vcm)
{
    uint16_t left_cnt = 0, right_cnt = 0, mid_cnt = 0, mid_s_cnt = 0;

    for (uint16_t y = VCM_ROI_Y_START; y <= VCM_ROI_Y_END; y++)
    {
        uint16_t lx = vcm->left_edge[y];
        uint16_t rx = vcm->right_edge[y];

        if (lx > 0 && lx < MT9V03X_W)
        {
            s_left_overlay[left_cnt].x = lx;
            s_left_overlay[left_cnt].y = y;
            left_cnt++;
        }
        if (rx > 0 && rx < MT9V03X_W)
        {
            s_right_overlay[right_cnt].x = rx;
            s_right_overlay[right_cnt].y = y;
            right_cnt++;
        }
        if (vcm->row_valid[y])
        {
            uint16_t mx = vcm->midline[y];
            if (mx < MT9V03X_W)
            {
                if (vcm->row_edge_type[y] == 2U)
                {
                    s_mid_single_overlay[mid_s_cnt].x = mx;
                    s_mid_single_overlay[mid_s_cnt].y = y;
                    mid_s_cnt++;
                }
                else
                {
                    s_mid_overlay[mid_cnt].x = mx;
                    s_mid_overlay[mid_cnt].y = y;
                    mid_cnt++;
                }
            }
        }
    }

    if (left_cnt > 0)
        ips200pro_image_draw_line(g_pro_image_id, 1, s_left_overlay,
                                  left_cnt, IMAGE_LINE_TYPE_UINT16, COLOR_RED);
    if (right_cnt > 0)
        ips200pro_image_draw_line(g_pro_image_id, 2, s_right_overlay,
                                  right_cnt, IMAGE_LINE_TYPE_UINT16, COLOR_BLUE);
    if (mid_cnt > 0)
        ips200pro_image_draw_line(g_pro_image_id, 3, s_mid_overlay,
                                  mid_cnt, IMAGE_LINE_TYPE_UINT16, COLOR_GREEN);
    if (mid_s_cnt > 0)
        ips200pro_image_draw_line(g_pro_image_id, 4, s_mid_single_overlay,
                                  mid_s_cnt, IMAGE_LINE_TYPE_UINT16, COLOR_YELLOW);

    ips200pro_image_display(g_pro_image_id, image[0],
                            MT9V03X_W, MT9V03X_H,
                            IMAGE_GRAYSCALE, 0);
}

int main(void)
{

    clock_init(SYSTEM_CLOCK_250M);
    debug_init();

    ips200pro_init("", IPS200PRO_TITLE_TOP, 0);
    g_pro_image_id = ips200pro_image_create(0, 0, MT9V03X_W, MT9V03X_H);
    g_pro_lbl_fps  = ips200pro_label_create(0, MT9V03X_H + 2, 160, 20);
    g_pro_lbl_err  = ips200pro_label_create(0, MT9V03X_H + 24, 200, 20);
    g_pro_lbl_cross = ips200pro_label_create(0, MT9V03X_H + 46, 200, 20);
    g_pro_lbl_init = ips200pro_label_create(0, MT9V03X_H + 68, 200, 20);

    ips200pro_label_printf(g_pro_lbl_init, "mt9v03x init...");
    while(1)
    {
        if(mt9v03x_init())
        {
            ips200pro_label_printf(g_pro_lbl_init, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
        system_delay_ms(500);
    }
    ips200pro_label_printf(g_pro_lbl_init, "init success.");

    all_hardware_init();   
    PID_init();
    ackermann_servo_set_angle(STEER_SERVO_CENTER_DEG);

    g_target_speed_m1 = 0.0f;
    g_target_speed_m2 = 0.0f;

#if ENABLE_WIFI_VISION
    wifi_vision_init();
#endif
    uint8_t lcd_frame_div = 0;
    uint16_t last_frame_tick = TimerCount;
    uint16_t last_warn_tick  = TimerCount;
    uint32_t loop_alive_tick = 0;
    uint16_t fps_window_start_tick = TimerCount;
    uint16_t fps_frame_count = 0;
    float fps_now = 0.0f;
    uint8_t obs_uart_div = 0;
    uint16_t stage5_last_tick = TimerCount;
    uint8_t stage5_has_frame = 0U;

#if ENABLE_J0_FORCED_ANGLE
    uint8_t  j0_state = 0;
    uint16_t j0_last_tick = TimerCount;
    float    j0_cmd = J0_ANGLE_A;
    ackermann_servo_set_angle(j0_cmd);
    wprintf("[J0_DIAG] START forced-angle mode: A=%.1f(duty=%u) B=%.1f(duty=%u) period=%dms\r\n",
            J0_ANGLE_A, steer_servo_deg_to_duty(J0_ANGLE_A),
            J0_ANGLE_B, steer_servo_deg_to_duty(J0_ANGLE_B),
            J0_TOGGLE_MS);
    wprintf("[J0_DIAG] PWM_DUTY_MAX=%d FREQ=%u LEFT=%.1f(duty=%u) CENTER=%.1f(duty=%u) RIGHT=%.1f(duty=%u) trim=%.1f\r\n",
            PWM_DUTY_MAX, STEER_SERVO_FREQ,
            STEER_SERVO_LEFT_DEG,   steer_servo_deg_to_duty(STEER_SERVO_LEFT_DEG),
            STEER_SERVO_CENTER_DEG, steer_servo_deg_to_duty(STEER_SERVO_CENTER_DEG),
            STEER_SERVO_RIGHT_DEG,  steer_servo_deg_to_duty(STEER_SERVO_RIGHT_DEG),
            g_servo_center_trim_deg);
#endif

#if ENABLE_H0_SERVO_TEST
    uint8_t  h0_stage = 1;
    uint16_t h0_last_tick = TimerCount;
    static const float h0_angles[4] = {
        STEER_SERVO_LEFT_DEG,
        STEER_SERVO_CENTER_DEG,
        STEER_SERVO_RIGHT_DEG,
        STEER_SERVO_CENTER_DEG
    };
    static const char * const h0_names[4] = {"LEFT", "CENTER", "RIGHT", "CENTER"};
    wprintf("[SERVO_H0] init stage=1(CENTER) cmd=%.1f\r\n",
            STEER_SERVO_CENTER_DEG);
#endif

    while (true)
    {

        if (g_imu_update_req)
        {
            g_imu_update_req = 0;
            imu_fusion_update();
        }

        imu_euler_t e = imu_fusion_get_euler();
        float yaw_deg = e.yaw * 57.29578f;
        (void)yaw_deg;

        if (++loop_alive_tick >= 200000)
        {
            loop_alive_tick = 0;
#if LOG_HEARTBEAT
            wprintf("[HB] loop alive\r\n");
#endif
        }

#if ENABLE_J0_FORCED_ANGLE

        {
            uint16_t j0_dt = (uint16_t)(TimerCount - j0_last_tick);
            if (j0_dt >= J0_TOGGLE_TICKS)
            {
                j0_last_tick = TimerCount;
                j0_state = !j0_state;
                j0_cmd = j0_state ? J0_ANGLE_B : J0_ANGLE_A;
                ackermann_servo_set_angle(j0_cmd);
                wprintf("[J0_DIAG] cmd=%.1f duty=%u state=%s\r\n",
                        j0_cmd, steer_servo_deg_to_duty(j0_cmd),
                        j0_state ? "B" : "A");
            }
        }
#endif

#if ENABLE_H0_SERVO_TEST

        {
            uint16_t h0_dt = (uint16_t)(TimerCount - h0_last_tick);
            if (h0_dt >= H0_STEP_TICKS)
            {
                h0_last_tick = TimerCount;
                h0_stage = (h0_stage + 1) % 4;
                ackermann_servo_set_angle(h0_angles[h0_stage]);
                wprintf("[SERVO_H0] cmd=%.1f stage=%d(%s)\r\n",
                        h0_angles[h0_stage], h0_stage, h0_names[h0_stage]);
            }
        }

        g_target_speed_m1 = 0.0f;
        g_target_speed_m2 = 0.0f;
#endif

#if !ENABLE_STAGE5_GATE_CONTROL
        g_target_speed_m1 = 0.0f;
        g_target_speed_m2 = 0.0f;
#endif

#if !ENABLE_J0_FORCED_ANGLE && !ENABLE_H0_SERVO_TEST
        {
            uint16_t now_tick = TimerCount;
            uint16_t stage5_dt = (uint16_t)(now_tick - stage5_last_tick);
            if (stage5_dt >= STAGE5_CTRL_PERIOD_TICKS)
            {
                stage5_last_tick = now_tick;
                if (stage5_has_frame)
                {
                    uint16_t frame_age = (uint16_t)(now_tick - last_frame_tick);
                    if (frame_age <= 250)
                    {
                        servo_pid_control_update(&g_stage1_obs, &g_stage2_edge, &g_stage3_err, &g_stage5);
                    }
                    else
                    {
#if ENABLE_STAGE5_GATE_CONTROL
                        g_target_speed_m1 = 0.0f;
                        g_target_speed_m2 = 0.0f;
                        ackermann_servo_set_angle(STEER_SERVO_CENTER_DEG);
                        g_stage5.speed_cmd = 0.0f;
                        g_stage5.steer_cmd_deg = STEER_SERVO_CENTER_DEG;
#endif
                    }
                }
            }
        }
#endif

        if (mt9v03x_finish_flag)
        {
            last_frame_tick = TimerCount;
            mt9v03x_finish_flag = 0;

            memset(&g_vcm_result, 0, sizeof(g_vcm_result));
            white_seed_observe_update(mt9v03x_image, &g_stage1_obs);
            search_line_edge_update(mt9v03x_image, &g_stage1_obs, &g_stage2_edge);
            search_line_fill_vcm_for_overlay(mt9v03x_image, &g_stage1_obs, &g_stage2_edge, &g_vcm_result);
            middline_error_update(&g_stage1_obs, &g_stage2_edge, &g_stage3_err);
            stage5_has_frame = 1U;
#if STAGE1_UART_PRINT_ENABLE
            if (++obs_uart_div >= STAGE1_UART_FRAME_DIV)
            {
                obs_uart_div = 0;
                wprintf("[OBS] Q=%u VR=%2u RR=%2u CS=%u EF=%4d\r\n",
                        (unsigned)g_stage1_obs.quality_ok,
                        (unsigned)g_stage2_edge.valid_rows,
                        (unsigned)g_stage2_edge.robust_rows,
                        (unsigned)g_stage2_edge.cross_state,
                        (int)g_stage3_err.filt);
            }
#endif

            fps_frame_count++;
            {
                uint16_t dt_tick = (uint16_t)(TimerCount - fps_window_start_tick);
                if (dt_tick >= 500)
                {
                    fps_now = ((float)fps_frame_count * 500.0f) / (float)dt_tick;
                    fps_frame_count = 0;
                    fps_window_start_tick = TimerCount;
                }
            }

            lcd_frame_div++;
            if (lcd_frame_div >= LCD_TEXT_FRAME_DIV)
            {
                lcd_frame_div = 0;

#if ENABLE_LCD_FULL_VIS
#if ENABLE_LCD_OVERLAY
                visualize_track_on_screen_pro(mt9v03x_image, &g_vcm_result);
#else
                ips200pro_image_display(g_pro_image_id, mt9v03x_image[0],
                                        MT9V03X_W, MT9V03X_H,
                                        IMAGE_GRAYSCALE, 0);
#endif
#else

                ips200pro_image_display(g_pro_image_id, mt9v03x_image[0],
                                        MT9V03X_W, MT9V03X_H,
                                        IMAGE_GRAYSCALE, 0);
#endif

                ips200pro_label_printf(g_pro_lbl_fps, "FPS:%4.1f CM:%u", fps_now, (unsigned)g_stage5.curve_mode);
                ips200pro_label_printf(g_pro_lbl_err, "ERR:%4d FILT:%4d",
                                       (int)g_stage3_err.raw,
                                       (int)g_stage3_err.filt);
                ips200pro_label_printf(g_pro_lbl_cross, "CROSS:%s S:%u",
                                       g_stage2_edge.cross_active ? "ON" : "OFF",
                                       (unsigned)g_stage2_edge.cross_state);
            }

#if ENABLE_WIFI_VISION
            wifi_vision_send_frame(mt9v03x_image, &g_vcm_result);
#endif

        }
        else
        {

            uint16_t now_tick = TimerCount;
            uint16_t dt_frame = (uint16_t)(now_tick - last_frame_tick);
            uint16_t dt_warn  = (uint16_t)(now_tick - last_warn_tick);
            if (dt_frame >= 250 && dt_warn >= 500)
            {
                last_warn_tick = now_tick;
                wprintf("[WARN] No camera frame (%u ms)\r\n", (unsigned)(dt_frame * 2));
#if ENABLE_STAGE5_GATE_CONTROL
                g_target_speed_m1 = 0.0f;
                g_target_speed_m2 = 0.0f;
                ackermann_servo_set_angle(STEER_SERVO_CENTER_DEG);
                g_stage5.speed_cmd = 0.0f;
                g_stage5.steer_cmd_deg = STEER_SERVO_CENTER_DEG;
#endif
            }
        }
    }
}
