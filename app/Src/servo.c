#include "servo.h"
//这里的是从度数辐射到ccr

/* ===== 舵机参数（你以后只需要改这里） ===== */
#define SERVO_MIN_US     500.0f    // 0°
#define SERVO_MAX_US     2500.0f   // 180°
#define SERVO_MIN_ANGLE  0.0f
#define SERVO_MAX_ANGLE  180.0f

/* ===== 死区设置（防抖，可先不动） ===== */
#define SERVO_DEADBAND_US  0.0f    // 小于 2us 的变化不更新
//这里修改死区防抖，因为外部可以加装原来的参数是2.0
/* ===== 内部变量 ===== */
static TIM_HandleTypeDef *servo_tim = NULL;
static uint32_t last_ccr_pitch = 0;
static uint32_t last_ccr_yaw  = 0;

/* ===== 初始化 ===== */
void Servo_Init(TIM_HandleTypeDef *htim)
{
    servo_tim = htim;

    HAL_TIM_PWM_Start(servo_tim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(servo_tim, TIM_CHANNEL_2);
}

/* ===== 核心函数：设置舵机角度 ===== */
void Servo_SetAngle(Servo_ID_t id, float angle)
{
    if (servo_tim == NULL)
        return;

    /* 1. 角度限幅（物理保护） */
    if (angle > SERVO_MAX_ANGLE) angle = SERVO_MAX_ANGLE;
    if (angle < SERVO_MIN_ANGLE) angle = SERVO_MIN_ANGLE;

    /* 2. 角度 → 脉宽（µs） */
    float pulse_us = SERVO_MIN_US +
        (angle - SERVO_MIN_ANGLE) *
        (SERVO_MAX_US - SERVO_MIN_US) /
        (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE);

    uint32_t ccr = (uint32_t)(pulse_us + 0.5f);  // 四舍五入

    /* 3. 输出到对应通道（带死区） */
    if (id == SERVO_PITCH)
    {
        if ((ccr > last_ccr_pitch + SERVO_DEADBAND_US) ||
            (ccr + SERVO_DEADBAND_US < last_ccr_pitch))
        {
            __HAL_TIM_SET_COMPARE(servo_tim, TIM_CHANNEL_1, ccr);
            last_ccr_pitch = ccr;
        }
    }
    else if (id == SERVO_YAW)
    {
        if ((ccr > last_ccr_yaw + SERVO_DEADBAND_US) ||
            (ccr + SERVO_DEADBAND_US < last_ccr_yaw))
        {
            __HAL_TIM_SET_COMPARE(servo_tim, TIM_CHANNEL_2, ccr);
            last_ccr_yaw = ccr;
        }
    }
}
//
// Created by toolpride on 2025/12/14.
//