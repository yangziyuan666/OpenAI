#include "MotorPID.h"
#include "all_hardware_init.h"
//定义一个结构体类型变量
tPid pidMotor1Speed;
tPid pidMotor2Speed;
//给结构体类型变量赋初值
void PID_init()
{
    // 电机1
    pidMotor1Speed.actual_val = 0.0f;
    pidMotor1Speed.target_val = 0.0f;  
    pidMotor1Speed.err = 0.0f;
    pidMotor1Speed.err_last = 0.0f;
    pidMotor1Speed.err_sum = 0.0f;
    pidMotor1Speed.Kp = 1.0f;        
    pidMotor1Speed.Ki = 0.1f;          
    pidMotor1Speed.Kd = 0.0f;           
    pidMotor1Speed.output_val = 0.0f;

    // 电机2
    pidMotor2Speed.actual_val = 0.0f;
    pidMotor2Speed.target_val = 0.0f;  
    pidMotor2Speed.err = 0.0f;
    pidMotor2Speed.err_last = 0.0f;
    pidMotor2Speed.err_sum = 0.0f;
    pidMotor2Speed.Kp = 1.0f;           
    pidMotor2Speed.Ki = 0.1f;           
    pidMotor2Speed.Kd = 0.0f;           
    pidMotor2Speed.output_val = 0.0f;
}

/*******************
* PID计算函数（带输出限幅）
*******************/
float PID_realize(tPid *pid, float actual_val)
{
    // 1. 误差计算（不变）
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;
    if (pid->err_sum > 20.0f) pid->err_sum = 20.0f;
    if (pid->err_sum < -20.0f) pid->err_sum = -20.0f;

    // 2. 原始输出（不变）
    float raw_output = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);

    // 3. 输出滤波（必须在这！用pid->output_val存历史值）
    static bool first_run = true;  // 新增：第一次运行不滤波，避免初始值干扰
    if (first_run) {
        pid->output_val = raw_output;  // 第一次直接用原始输出
        first_run = false;
    } else {
        pid->output_val = 0.7f * raw_output + 0.3f * pid->output_val;  // 滤波
    }

    // 4. 输出限幅（不变）
    if (pid->output_val > MAX_DUTY) pid->output_val = MAX_DUTY;
    if (pid->output_val < -MAX_DUTY) pid->output_val = -MAX_DUTY;

    // 5. 更新误差（不变）
    pid->err_last = pid->err;
    return pid->output_val;
}