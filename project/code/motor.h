#pragma once
#include <stdint.h>
#include <stdbool.h>




// 目标速度（脉冲/换算单位）
extern volatile float g_target_speed_m1;
extern volatile float g_target_speed_m2;

// 速度测量值
extern volatile int Motor1Speed;
extern volatile int Motor2Speed;

// 电机设置 & 速度闭环
void Motor_Set(int m1, int m2);
void motorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed);



