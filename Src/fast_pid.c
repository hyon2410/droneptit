/*
 * fast_pid.c
 *
 *  Created on: Dec 31, 2025
 *      Author: Admin
 */


#include "fast_pid.h"

static inline float constrain(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void FastPID_Init(FastPID_t *pid,
                  float kp, float ki, float kd,
                  float hz,
                  float out_min, float out_max)
{
    float Ts = 1.0f / hz;

    pid->kp = kp;
    pid->ki = ki * Ts;
    pid->kd = kd / Ts;

    pid->out_min = out_min;
    pid->out_max = out_max;

    FastPID_Reset(pid);
}

void FastPID_Reset(FastPID_t *pid)
{
    pid->error = 0;
    pid->error_sum = 0;
    pid->last_error = 0;
}

float FastPID_Step(FastPID_t *pid, float setpoint, float feedback)
{
    pid->error = setpoint - feedback;

    // Anti-windup (clamp integrator)
    if (pid->error_sum * pid->ki <= pid->out_max &&
        pid->error_sum * pid->ki >= pid->out_min)
    {
        pid->error_sum += pid->error;
    }

    float delta_error = pid->error - pid->last_error;
    pid->last_error = pid->error;

    pid->error_sum = constrain(pid->error_sum,
                               pid->out_min / pid->ki,
                               pid->out_max / pid->ki);

    float output =
        pid->kp * pid->error +
        pid->ki * pid->error_sum +
        pid->kd * delta_error;

    return constrain(output, pid->out_min, pid->out_max);
}
