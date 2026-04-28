#pragma once
#include <stdint.h>

typedef struct {
    float roll;   // rad
    float pitch;  // rad
    float yaw;    // rad
} imu_euler_t;

// 初始化（sample_hz = 采样频率，和你调用 update 的频率一致）
void imu_fusion_init(float sample_hz);

// 上电静置零偏标定（ms：静止时间，建议 >=1500 ms）
void imu_fusion_zero_calibrate(uint16_t ms);

// 每个采样周期调用（放到定时器中断里）
void imu_fusion_update(void);

// 获取当前欧拉角（rad）（yaw 已扣除了 set_yaw_zero 的零点）
imu_euler_t imu_fusion_get_euler(void);

// 将当前航向设为 0（例如按键调用）
void imu_fusion_set_yaw_zero(void);

float imu_fusion_get_yaw_rate(void);   // rad/s
