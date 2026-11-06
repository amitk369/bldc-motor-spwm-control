/**
 * @file bldc_controller.c
 * @brief Master BLDC Closed-Loop Speed Controller Implementation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "bldc_controller.h"

void bldc_controller_init(bldc_controller_t *ctrl) {
    ctrl->hall_raw = 0;
    ctrl->current_sector = COMMUTATION_INVALID;
    ctrl->target_speed_rpm = 0.0f;
    ctrl->measured_speed_rpm = 0.0f;
    ctrl->rotor_angle_rad = 0.0f;
    ctrl->phase_current_u_a = 0.0f;
    ctrl->phase_current_v_a = 0.0f;
    ctrl->phase_current_w_a = 0.0f;
    ctrl->electromagnetic_torque_nm = 0.0f;
    ctrl->load_torque_nm = 0.0f;
    ctrl->enabled = true;
    ctrl->fault_code = 0;

    digital_pid_init(&ctrl->speed_pid, PID_KP, PID_KI, PID_KD, PID_DERIVATIVE_FILTER_N, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    hall_update_inverter_gates(COMMUTATION_INVALID, 0.0f, &ctrl->inverter_gates);
}

void bldc_controller_set_target_speed(bldc_controller_t *ctrl, float speed_rpm) {
    if (speed_rpm < 0.0f) speed_rpm = 0.0f;
    if (speed_rpm > MOTOR_RATED_SPEED_RPM) speed_rpm = MOTOR_RATED_SPEED_RPM;
    ctrl->target_speed_rpm = speed_rpm;
}

void bldc_controller_step(bldc_controller_t *ctrl, uint8_t hall_sensor_bits, float actual_speed_rpm, float dt_seconds) {
    ctrl->hall_raw = hall_sensor_bits;
    ctrl->measured_speed_rpm = actual_speed_rpm;

    if (!ctrl->enabled) {
        hall_update_inverter_gates(COMMUTATION_INVALID, 0.0f, &ctrl->inverter_gates);
        return;
    }

    // Step 1: Decode Hall state into Commutation Sector (1 to 6)
    ctrl->current_sector = hall_decode_sector(hall_sensor_bits);

    if (ctrl->current_sector == COMMUTATION_INVALID) {
        ctrl->fault_code = 0x01; // Hall error fault
        hall_update_inverter_gates(COMMUTATION_INVALID, 0.0f, &ctrl->inverter_gates);
        return;
    }

    // Step 2: Compute PWM Duty Cycle from closed-loop digital PID
    float duty = digital_pid_update(&ctrl->speed_pid, ctrl->target_speed_rpm, actual_speed_rpm, dt_seconds);

    // Step 3: Update 3-phase inverter gate switching signals
    hall_update_inverter_gates(ctrl->current_sector, duty, &ctrl->inverter_gates);
}
