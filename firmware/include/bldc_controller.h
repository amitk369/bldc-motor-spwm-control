/**
 * @file bldc_controller.h
 * @brief Master BLDC Controller Interface
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BLDC_CONTROLLER_H
#define BLDC_CONTROLLER_H

#include "bldc_types.h"
#include "hall_commutation.h"
#include "digital_pid.h"

void bldc_controller_init(bldc_controller_t *ctrl);
void bldc_controller_set_target_speed(bldc_controller_t *ctrl, float speed_rpm);
void bldc_controller_step(bldc_controller_t *ctrl, uint8_t hall_sensor_bits, float actual_speed_rpm, float dt_seconds);

#endif // BLDC_CONTROLLER_H
