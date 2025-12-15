#ifndef __MAHONY_H
#define __MAHONY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

    /* 姿态角，单位：度 */
    extern float pitch;
    extern float roll;
    extern float yaw;

    /* Mahony 滤波更新函数 */
    void Mahony_Update(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float dt);

#ifdef __cplusplus
}
#endif

#endif /* __MAHONY_H */
