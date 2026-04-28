#include "imu_fusion.h"
#include "zf_common_headfile.h"
#include "zf_device_imu963ra.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f  // 单精度浮点数版本（适配你的代码）
#endif

// ================== 调参区 ==================
static float s_dt = 0.002f;            // 采样周期，默认 2ms => 500Hz
static const float LPF_A_GYRO = 0.2f;  // 一阶低通系数（陀螺）
static const float LPF_A_ACC  = 0.2f;  // 一阶低通系数（加计）
static const float COMP_A_RP  = 0.98f; // roll/pitch 互补：陀螺主导
static const float COMP_A_YAW = 0.98f; // yaw 互补：陀螺主导、磁航向慢校

// =============== 内部状态量 ===============
static float gx_bias_dps=0, gy_bias_dps=0, gz_bias_dps=0; // 陀螺零偏（deg/s）
static float ax_bias_g=0,  ay_bias_g=0,  az_bias_g=0;     // 加计零偏（g），az 以 1g 为基准

static float gx_f=0, gy_f=0, gz_f=0;    // 滤波后的陀螺（rad/s）
static float ax_f=0, ay_f=0, az_f=1.0f; // 滤波后的加计（g）

static float roll_=0, pitch_=0, yaw_=0; // 欧拉角（rad）
static float yaw_zero=0;                 // 航向零点

static inline float deg2rad(float d){ return d * (float)M_PI / 180.0f; }
static inline float inv_sqrtf_(float x){ return 1.0f/sqrtf(x); }

void imu_fusion_init(float sample_hz)
{
    s_dt = 1.0f / (sample_hz > 1.0f ? sample_hz : 500.0f);
    gx_bias_dps = gy_bias_dps = gz_bias_dps = 0.0f;
    ax_bias_g = ay_bias_g = 0.0f; az_bias_g = 0.0f;
    gx_f = gy_f = gz_f = 0.0f;
    ax_f = ay_f = 0.0f; az_f = 1.0f;
    roll_ = pitch_ = yaw_ = 0.0f;
    yaw_zero = 0.0f;
}

// 上电静置 N ms 做零偏
void imu_fusion_zero_calibrate(uint16_t ms)
{
    uint16_t cnt = (uint16_t)((ms + 1) / 2); // 2ms 一次
    if (cnt < 200) cnt = 200;                // 至少 400ms

    double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;

    for(uint16_t i=0;i<cnt;i++){
        imu963ra_get_acc();
        imu963ra_get_gyro();
        // 使用官方转换宏（单位：g / deg/s）
        sax += imu963ra_acc_transition(imu963ra_acc_x);
        say += imu963ra_acc_transition(imu963ra_acc_y);
        saz += imu963ra_acc_transition(imu963ra_acc_z);
        sgx += imu963ra_gyro_transition(imu963ra_gyro_x);
        sgy += imu963ra_gyro_transition(imu963ra_gyro_y);
        sgz += imu963ra_gyro_transition(imu963ra_gyro_z);
        system_delay_ms(2);
    }

    ax_bias_g = (float)(sax/cnt);
    ay_bias_g = (float)(say/cnt);
    // 让静止时 az_g 接近 +1g：后续 az_g - az_bias_g ≈ 1.0
    az_bias_g = (float)(saz/cnt - 1.0f);

    gx_bias_dps = (float)(sgx/cnt);
    gy_bias_dps = (float)(sgy/cnt);
    gz_bias_dps = (float)(sgz/cnt);

    // 初始化滤波状态
    ax_f = ay_f = 0.0f; az_f = 1.0f;
    gx_f = gy_f = gz_f = 0.0f;
    roll_ = pitch_ = yaw_ = 0.0f;
    yaw_zero = 0.0f;
}

void imu_fusion_set_yaw_zero(void)
{
    yaw_zero = yaw_;
}

// 每个采样周期（2ms）调用
void imu_fusion_update(void)
{
    // 1) 采样
    imu963ra_get_acc();
    imu963ra_get_gyro();
    imu963ra_get_mag();

    // 2) 转物理量并去偏
    // acc: g
    float ax_g = imu963ra_acc_transition(imu963ra_acc_x) - ax_bias_g;
    float ay_g = imu963ra_acc_transition(imu963ra_acc_y) - ay_bias_g;
    float az_g = imu963ra_acc_transition(imu963ra_acc_z) - az_bias_g; // 期望 ≈ 1.0

    // gyro: deg/s -> rad/s，并去偏
    float gx_rs = deg2rad(imu963ra_gyro_transition(imu963ra_gyro_x) - gx_bias_dps);
    float gy_rs = deg2rad(imu963ra_gyro_transition(imu963ra_gyro_y) - gy_bias_dps);
    float gz_rs = deg2rad(imu963ra_gyro_transition(imu963ra_gyro_z) - gz_bias_dps);

    // 3) 一阶低通
    ax_f += LPF_A_ACC  * (ax_g - ax_f);
    ay_f += LPF_A_ACC  * (ay_g - ay_f);
    az_f += LPF_A_ACC  * (az_g - az_f);
    gx_f += LPF_A_GYRO * (gx_rs - gx_f);
    gy_f += LPF_A_GYRO * (gy_rs - gy_f);
    gz_f += LPF_A_GYRO * (gz_rs - gz_f);

    // 4) 陀螺积分（预测）
    float roll_g  = roll_  + gx_f * s_dt;
    float pitch_g = pitch_ + gy_f * s_dt;
    float yaw_g   = yaw_   + gz_f * s_dt;

    // 5) 加计解算 roll/pitch（基于重力方向）
    // 归一化
    float ax = ax_f, ay = ay_f, az = az_f;  // az ≈ 1g
    float norm = inv_sqrtf_(ax*ax + ay*ay + az*az);
    ax *= norm; ay *= norm; az *= norm;

    float roll_acc  = atan2f(ay, az);
    float pitch_acc = -asinf(ax);

    roll_  = COMP_A_RP * roll_g  + (1.0f - COMP_A_RP) * roll_acc;
    pitch_ = COMP_A_RP * pitch_g + (1.0f - COMP_A_RP) * pitch_acc;

    // 6) 磁力计倾斜补偿的航向
    float mx = (float)imu963ra_mag_x;
    float my = (float)imu963ra_mag_y;
    float mz = (float)imu963ra_mag_z;

    float cr = cosf(roll_),  sr = sinf(roll_);
    float cp = cosf(pitch_), sp = sinf(pitch_);

    // 参考：mxh = mx*cp + mz*sp;  myh = mx*sr*sp + my*cr - mz*sr*cp;
    float mxh = mx*cp + mz*sp;
    float myh = mx*sr*sp + my*cr - mz*sr*cp;

    float yaw_mag = atan2f(-myh, mxh);  // [-pi,pi)

    // 互补修正 yaw（抑制零漂）
    yaw_ = COMP_A_YAW * yaw_g + (1.0f - COMP_A_YAW) * yaw_mag;

    // wrap 到 [-pi, pi)
    if (yaw_ >  M_PI) yaw_ -= 2.0f * M_PI;
    if (yaw_ < -M_PI) yaw_ += 2.0f * M_PI;
}

imu_euler_t imu_fusion_get_euler(void)
{
    imu_euler_t e = { roll_, pitch_, yaw_ - yaw_zero };
    if (e.yaw >  M_PI) e.yaw -= 2.0f * M_PI;
    if (e.yaw < -M_PI) e.yaw += 2.0f * M_PI;
    return e;
}


float imu_fusion_get_yaw_rate(void) { return gz_f; }
