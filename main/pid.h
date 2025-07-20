#ifndef PID_H
#define PID_H

typedef struct {
    float proportional_gain;
    float integral_gain;
    float derivative_gain;

    float set_point;
    float previous_error;
    float previous_time;
    float previous_integral;
} pid_t;

void initialize_pid(pid_t*, float, float, float, float);

float calculate_pid_response(pid_t*, float);

#endif
