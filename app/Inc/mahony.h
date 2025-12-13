#ifndef __MAHONY_H
#define __MAHONY_H

#include <math.h>

extern float pitch, roll, yaw;

void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt);

#endif
//
// Created by toolpride on 2025/12/11.
//

#ifndef MIDDLETESTMARK3_MAHONY_H
#define MIDDLETESTMARK3_MAHONY_H

#endif //MIDDLETESTMARK3_MAHONY_H