/**
 * @file digital_pid.h
 * @brief Discrete PID Speed Regulator with Anti-Windup & Derivative Filter
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef DIGITAL_PID_H
#define DIGITAL_PID_H

#include "bldc_types.h"

void digital_pid_init(digital_pid_t *pid, float kp, float ki, float kd, float n_filter, float out_min, float out_max);
void digital_pid_reset(digital_pid_t *pid);
float digital_pid_update(digital_pid_t *pid, float target_rpm, float actual_rpm, float dt_seconds);

#endif // DIGITAL_PID_H
