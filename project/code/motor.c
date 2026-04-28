#include "motor.h"
#include "all_hardware_init.h"
#include "MotorPID.h"
#include <math.h>


volatile float g_target_speed_m1 = 0.0f;
volatile float g_target_speed_m2 = 0.0f;

volatile int   Motor1Speed = 0;
volatile int   Motor2Speed = 0;

int g_motor1_set_param = 0;
int g_motor2_set_param = 0;
uint32_t g_motor1_duty  = 0;
uint32_t g_motor2_duty  = 0;
float g_pid1_output     = 0.0f;
float g_pid2_output     = 0.0f;

void Motor_Set(int motor1, int motor2)
{
    g_motor1_set_param = motor1;
    g_motor2_set_param = motor2;

    // L1
    if (motor1 >= 0) {
        gpio_set_level(DIR_L1, MOTOR1_DIR_FORWARD_LEVEL);
        if (motor1 > MAX_DUTY) motor1 = MAX_DUTY;
        g_motor1_duty = (uint32_t)motor1;
    } else {
        gpio_set_level(DIR_L1, MOTOR1_DIR_REVERSE_LEVEL);
        if (motor1 < -MAX_DUTY) motor1 = -MAX_DUTY;
        g_motor1_duty = (uint32_t)(-motor1);
    }
    pwm_set_duty(PWM_L1, g_motor1_duty);

    // R1
    if (motor2 >= 0) {
        gpio_set_level(DIR_R1, MOTOR2_DIR_FORWARD_LEVEL);
        if (motor2 > MAX_DUTY) motor2 = MAX_DUTY;
        g_motor2_duty = (uint32_t)motor2;
    } else {
        gpio_set_level(DIR_R1, MOTOR2_DIR_REVERSE_LEVEL);
        if (motor2 < -MAX_DUTY) motor2 = -MAX_DUTY;
        g_motor2_duty = (uint32_t)(-motor2);
    }
    pwm_set_duty(PWM_R1, g_motor2_duty);
}

void motorPidSetSpeed(float Motor1SetSpeed, float Motor2SetSpeed)
{
    // 方向与幅值分离
    int   dir1   = (Motor1SetSpeed >= 0.0f) ? +1 : -1;
    int   dir2   = (Motor2SetSpeed >= 0.0f) ? +1 : -1;
    float sp1_set = fabsf(Motor1SetSpeed);
    float sp2_set = fabsf(Motor2SetSpeed);
    float sp1_mea = fabsf((float)Motor1Speed);
    float sp2_mea = fabsf((float)Motor2Speed);

    pidMotor1Speed.target_val = sp1_set;
    pidMotor2Speed.target_val = sp2_set;

    float duty1 = PID_realize(&pidMotor1Speed, sp1_mea);
    float duty2 = PID_realize(&pidMotor2Speed, sp2_mea);

    g_pid1_output = duty1;
    g_pid2_output = duty2;

    // 近零直接停
    if (sp1_set <= 1.0f) duty1 = 0;
    if (sp2_set <= 1.0f) duty2 = 0;

    // 限幅 + 起步占空
    duty1 = CLAMP(duty1, 0, MAX_DUTY);
    duty2 = CLAMP(duty2, 0, MAX_DUTY);
    if (duty1 > 0 && duty1 < MIN_DUTY) duty1 = MIN_DUTY;
    if (duty2 > 0 && duty2 < MIN_DUTY) duty2 = MIN_DUTY;

    Motor_Set(dir1 * (int)duty1, dir2 * (int)duty2);
}

