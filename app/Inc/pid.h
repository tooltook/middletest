#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct
    {
        float kp;
        float ki;
        float kd;

        float integral;
        float last_error;

        float integral_limit;   // 积分限幅
        float output_limit;     // 输出限幅（给执行器用）

    } PID_t;

    /**
     * @brief  初始化 PID
     */
    void PID_Init(PID_t *pid,
                  float kp, float ki, float kd,
                  float integral_limit,
                  float output_limit);

    /**
     * @brief  重置 PID 状态（很重要）
     */
    void PID_Reset(PID_t *pid);

    /**
     * @brief  PID 更新函数（位置式）
     */
    float PID_Update(PID_t *pid,
                     float target,
                     float measure,
                     float dt);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H *///
// Created by toolpride on 2025/12/14.
//

#ifndef MIDDLETESTMARK3_PID_H
#define MIDDLETESTMARK3_PID_H

#endif //MIDDLETESTMARK3_PID_H