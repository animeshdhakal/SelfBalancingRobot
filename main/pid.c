#include "pid.h"
#include <freertos/FreeRTOS.h>


void initialize_pid(pid_t *pid, float p, float i, float d, float set_point)
{
    pid->proportional_gain = p;
    pid->integral_gain = i;
    pid->derivative_gain = d;

    pid->set_point = set_point;
    pid->previous_error = 0;
    pid->previous_time = 0;
    pid->previous_integral = 0;
}

float calculate_pid_response(pid_t *pid, float current_value)
{
    float error = pid->set_point - current_value;
    float current_time = pdTICKS_TO_MS(xTaskGetTickCount()) / 1000.0f;


    float dt = current_time - pid->previous_time;
    float de = error - pid->previous_error;

    pid->previous_integral += error * dt;
    pid->previous_time = current_time;
    pid->previous_error = error;

    return error * pid->proportional_gain + (de / dt) * pid->derivative_gain + pid->previous_integral;
}
