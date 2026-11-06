/**
 * @file hall_commutation.h
 * @brief 6-Step Trapezoidal Commutation Logic Header
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef HALL_COMMUTATION_H
#define HALL_COMMUTATION_H

#include "bldc_types.h"

// Decode 3-bit Hall sensor signals into 6-step sector
bldc_sector_t hall_decode_sector(uint8_t hall_bits);

// Apply 6-step commutation switches according to decoded sector and PWM duty
void hall_update_inverter_gates(bldc_sector_t sector, float duty_cycle, inverter_gate_signals_t *gates);

#endif // HALL_COMMUTATION_H
