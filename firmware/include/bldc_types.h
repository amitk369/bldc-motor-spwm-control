/**
 * @file bldc_types.h
 * @brief Commutation Sectors, Inverter States & Regulator Structures
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BLDC_TYPES_H
#define BLDC_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "bldc_config.h"

// 6 Commutation Sectors derived from Hall Sensors (H_a, H_b, H_c)
typedef enum {
    COMMUTATION_INVALID = 0,
    COMMUTATION_SECTOR_1 = 1, // Ha=1, Hb=0, Hc=1 (5)
    COMMUTATION_SECTOR_2 = 2, // Ha=1, Hb=0, Hc=0 (4)
    COMMUTATION_SECTOR_3 = 3, // Ha=1, Hb=1, Hc=0 (6)
    COMMUTATION_SECTOR_4 = 4, // Ha=0, Hb=1, Hc=0 (2)
    COMMUTATION_SECTOR_5 = 5, // Ha=0, Hb=1, Hc=1 (3)
    COMMUTATION_SECTOR_6 = 6  // Ha=0, Hb=0, Hc=1 (1)
} bldc_sector_t;

// 3-Phase Inverter Half-Bridge Switching States
typedef enum {
    SWITCH_FLOATING = 0,
    SWITCH_HIGH_ON,
    SWITCH_LOW_ON,
    SWITCH_PWM_HIGH // High side modulated with SPWM, low side complimentary
} phase_switch_state_t;

typedef struct {
    phase_switch_state_t phase_u;
    phase_switch_state_t phase_v;
    phase_switch_state_t phase_w;
    float duty_cycle;
} inverter_gate_signals_t;

// Digital PID Speed Regulator State with Anti-Windup
typedef struct {
    float kp;
    float ki;
    float kd;
    float n_filter;          // Derivative filter coefficient
    float out_min;
    float out_max;
    
    float target_speed_rpm;
    float actual_speed_rpm;
    float prev_error_rpm;
    float integral_acc;
    float prev_derivative;
    float output_duty;       // 0.0 to 1.0
    bool saturated;
} digital_pid_t;

// Master Controller State
typedef struct {
    uint8_t hall_raw;        // 3-bit Hall value (Ha<<2 | Hb<<1 | Hc)
    bldc_sector_t current_sector;
    float target_speed_rpm;
    float measured_speed_rpm;
    float rotor_angle_rad;
    float phase_current_u_a;
    float phase_current_v_a;
    float phase_current_w_a;
    float electromagnetic_torque_nm;
    float load_torque_nm;
    
    digital_pid_t speed_pid;
    inverter_gate_signals_t inverter_gates;
    
    bool enabled;
    uint32_t fault_code;
} bldc_controller_t;

#endif // BLDC_TYPES_H
