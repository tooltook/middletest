#include "mahony.h"

float twoKp = 2.0f * 0.5f;   // proportional gain
float twoKi = 2.0f * 0.0f;   // integral gain

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;     // quaternion
static float integralFBx = 0, integralFBy = 0, integralFBz = 0;

float pitch, roll, yaw;

void Mahony_Update(float gx, float gy, float gz,
                   float ax, float ay, float az,
                   float dt)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;

    gx *= (M_PI / 180.0f);
    gy *= (M_PI / 180.0f);
    gz *= (M_PI / 180.0f);

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        recipNorm = 1.0f / sqrtf(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;

        halfex = (ay*halfvz - az*halfvy);
        halfey = (az*halfvx - ax*halfvz);
        halfez = (ax*halfvy - ay*halfvx);

        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * dt;
            integralFBy += twoKi * halfey * dt;
            integralFBz += twoKi * halfez * dt;

            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    pitch = asinf(-2.0f * (q1*q3 - q0*q2)) * 57.3f;
    roll  = atan2f(2.0f * (q0*q1 + q2*q3),
                  1.0f - 2.0f * (q1*q1 + q2*q2)) * 57.3f;
    yaw   = atan2f(2.0f * (q0*q3 + q1*q2),
                  1.0f - 2.0f * (q2*q2 + q3*q3)) * 57.3f;
}
//
// Created by toolpride on 2025/12/11.
//