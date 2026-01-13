#ifndef FAST_PID_H
#define FAST_PID_H

typedef struct {
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;

    float error;
    float error_sum;
    float last_error;
} FastPID_t;

void FastPID_Init(FastPID_t *pid,
                  float kp, float ki, float kd,
                  float hz,
                  float out_min, float out_max);

void FastPID_Reset(FastPID_t *pid);

float FastPID_Step(FastPID_t *pid, float setpoint, float feedback);

#endif
