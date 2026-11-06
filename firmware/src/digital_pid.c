/**
 * @file digital_pid.c
 * @brief Discrete PID Speed Regulator Implementation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "digital_pid.h"
#include <math.h>

void digital_pid_init(digital_pid_t *pid, float kp, float ki, float kd, float n_filter, float out_min, float out_max) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->n_filter = n_filter;
    pid->out_min = out_min;
    pid->out_max = out_max;
    digital_pid_reset(pid);
}

void digital_pid_reset(digital_pid_t *pid) {
    pid->target_speed_rpm = 0.0f;
    pid->actual_speed_rpm = 0.0f;
    pid->prev_error_rpm = 0.0f;
    pid->integral_acc = 0.0f;
    pid->prev_derivative = 0.0f;
    pid->output_duty = 0.0f;
    pid->saturated = false;
}

float digital_pid_update(digital_pid_t *pid, float target_rpm, float actual_rpm, float dt_seconds) {
    pid->target_speed_rpm = target_rpm;
    pid->actual_speed_rpm = actual_rpm;

    float error = target_rpm - actual_rpm;

    // Proportional Term
    float p_term = pid->kp * error;

    // Integral Term with Anti-Windup Clamping
    // Only accumulate integral if output is NOT saturated, or error opposes saturation
    if (!pid->saturated || (error * pid->output_duty < 0.0f)) {
        pid->integral_acc += pid->ki * error * dt_seconds;
    }

    // Filtered Derivative Term: Low-pass filter N / (1 + N * s)
    // dD/dt = N * (Kd * de/dt - D)
    float de = error - pid->prev_error_rpm;
    float alpha_d = (pid->n_filter * dt_seconds) / (1.0f + pid->n_filter * dt_seconds);
    float raw_d = (dt_seconds > 0.0f) ? (pid->kd * de / dt_seconds) : 0.0f;
    float d_term = (1.0f - alpha_d) * pid->prev_derivative + alpha_d * raw_d;
    pid->prev_derivative = d_term;
    pid->prev_error_rpm = error;

    // Sum Terms
    float unsat_output = p_term + pid->integral_acc + d_term;

    // Saturation Clamping
    if (unsat_output > pid->out_max) {
        pid->output_duty = pid->out_max;
        pid->saturated = true;
    } else if (unsat_output < pid->out_min) {
        pid->output_duty = pid->out_min;
        pid->saturated = true;
    } else {
        pid->output_duty = unsat_output;
        pid->saturated = false;
    }

    return pid->output_duty;
}
