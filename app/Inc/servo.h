#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f1xx_hal.h"

typedef enum {
    SERVO_PITCH = 0,
    SERVO_YAW = 1,
} Servo_ID_t;

void Servo_Init(TIM_HandleTypeDef *htim);
void Servo_SetAngle(Servo_ID_t id, float angle);

#endif
//
// Created by toolpride on 2025/12/14.
//

#ifndef MIDDLETESTMARK3_SERVO_H
#define MIDDLETESTMARK3_SERVO_H

#endif //MIDDLETESTMARK3_SERVO_H