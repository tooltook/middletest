#include "pid.h"

/**
 * @brief 初始化 PID 参数
 */
void PID_Init(PID_t *pid,
              float kp, float ki, float kd,
              float integral_limit,
              float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->last_error = 0.0f;

    pid->integral_limit = integral_limit;
    pid->output_limit = output_limit;
}

/**
 * @brief 重置 PID 内部状态
 */
void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
}

/**
 * @brief PID 更新
 */
float PID_Update(PID_t *pid,
                 float target,
                 float measure,
                 float dt)
{
    // dt 保护（非常重要）
    if (dt <= 0.0001f)
        return 0.0f;

    float error = target - measure;

    /* -------- 积分项 -------- */
    pid->integral += error * dt;

    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;

    /* -------- 微分项 -------- */
    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    /* -------- PID 输出 -------- */
    float output =
        pid->kp * error +
        pid->ki * pid->integral +
        pid->kd * derivative;

    /* -------- 输出限幅 -------- */
    if (output > pid->output_limit)
        output = pid->output_limit;
    else if (output < -pid->output_limit)
        output = -pid->output_limit;

    return output;
}

//
// Created by toolpride on 2025/12/14.
//