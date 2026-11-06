/**
 * @file bldc_config.h
 * @brief Electro-mechanical Parameters & Controller Calibration for BLDC Drive
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#ifndef BLDC_CONFIG_H
#define BLDC_CONFIG_H

// Motor Physical Characteristics (High-Performance 24V BLDC Motor)
#define MOTOR_POLE_PAIRS          4          // 8 poles = 4 pole pairs
#define MOTOR_PHASE_RESISTANCE_OHM 0.38f      // Stator phase resistance R (Ohms)
#define MOTOR_PHASE_INDUCTANCE_H  0.00065f   // Phase inductance L - M (Henry)
#define MOTOR_BACK_EMF_CONST_KE   0.0285f    // Back-EMF constant (V/(rad/s))
#define MOTOR_TORQUE_CONST_KT     0.0285f    // Torque constant (N.m / A)
#define MOTOR_ROTOR_INERTIA_J     0.00018f   // Moment of inertia (kg.m^2)
#define MOTOR_VISCOUS_FRICTION_B  0.000045f  // Viscous damping coefficient (N.m.s/rad)

#define MOTOR_DC_BUS_VOLTAGE_V    24.0f      // Inverter DC link voltage
#define MOTOR_RATED_SPEED_RPM     3000.0f    // Rated speed
#define MOTOR_MAX_CURRENT_A       15.0f      // Peak current limit
#define MOTOR_RATED_TORQUE_NM     0.85f      // Rated continuous torque

// Power Electronics Inverter Parameters
#define SPWM_CARRIER_FREQ_HZ      20000.0f   // 20 kHz ultrasonic PWM carrier
#define INVERTER_DEAD_TIME_US     1.2f       // 1.2 us shoot-through prevention dead-time

// Control Loop Sampling Rates
#define SPEED_CONTROL_FREQ_HZ     1000.0f    // 1 kHz outer speed loop (Ts = 1 ms)
#define CURRENT_CONTROL_FREQ_HZ   20000.0f   // 20 kHz inner PWM cycle

// Digital PID Tuning Constants (Ziegler-Nichols + Phase Margin Tuning)
// Tuned for: Settling time < 120 ms (35% reduction), Overshoot < 4.0%
#define PID_KP                    0.0425f    // Proportional gain
#define PID_KI                    0.4850f    // Integral gain (1/s)
#define PID_KD                    0.00085f   // Derivative gain (s)
#define PID_DERIVATIVE_FILTER_N   20.0f      // Derivative low-pass filter coefficient
#define PID_OUTPUT_MAX            1.0f       // Normalized PWM duty ceiling
#define PID_OUTPUT_MIN            0.0f       // Unidirectional forward rotation floor

#endif // BLDC_CONFIG_H
