/**
 * @file test_bldc.c
 * @brief Unit Tests for BLDC Commutation & Closed-Loop Digital PID
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../firmware/include/bldc_controller.h"

void test_hall_decoding() {
    printf("[TEST] Testing 120-degree Hall sensor decoding...\n");
    // Standard 6 valid states: 5, 4, 6, 2, 3, 1
    assert(hall_decode_sector(5) == COMMUTATION_SECTOR_1);
    assert(hall_decode_sector(4) == COMMUTATION_SECTOR_2);
    assert(hall_decode_sector(6) == COMMUTATION_SECTOR_3);
    assert(hall_decode_sector(2) == COMMUTATION_SECTOR_4);
    assert(hall_decode_sector(3) == COMMUTATION_SECTOR_5);
    assert(hall_decode_sector(1) == COMMUTATION_SECTOR_6);

    // Invalid states (all 0s or all 1s are impossible with 120-deg separation)
    assert(hall_decode_sector(0) == COMMUTATION_INVALID);
    assert(hall_decode_sector(7) == COMMUTATION_INVALID);
    printf("  -> Hall decoding PASSED.\n");
}

void test_inverter_gate_commutation() {
    printf("[TEST] Testing 3-Phase Inverter Gate Switching Signals...\n");
    inverter_gate_signals_t gates;

    // Sector 1: Phase U PWM high, Phase W low, Phase V floating
    hall_update_inverter_gates(COMMUTATION_SECTOR_1, 0.75f, &gates);
    assert(gates.phase_u == SWITCH_PWM_HIGH);
    assert(gates.phase_v == SWITCH_FLOATING);
    assert(gates.phase_w == SWITCH_LOW_ON);
    assert(fabsf(gates.duty_cycle - 0.75f) < 0.001f);

    // Sector 4: Phase W PWM high, Phase U low, Phase V floating
    hall_update_inverter_gates(COMMUTATION_SECTOR_4, 0.50f, &gates);
    assert(gates.phase_u == SWITCH_LOW_ON);
    assert(gates.phase_v == SWITCH_FLOATING);
    assert(gates.phase_w == SWITCH_PWM_HIGH);

    // Invalid Sector: All floating for shoot-through safety
    hall_update_inverter_gates(COMMUTATION_INVALID, 0.50f, &gates);
    assert(gates.phase_u == SWITCH_FLOATING);
    assert(gates.phase_v == SWITCH_FLOATING);
    assert(gates.phase_w == SWITCH_FLOATING);
    assert(gates.duty_cycle == 0.0f);
    printf("  -> 6-step trapezoidal gate actuation PASSED.\n");
}

void test_digital_pid_anti_windup() {
    printf("[TEST] Testing Digital PID Speed Regulator Anti-Windup...\n");
    digital_pid_t pid;
    digital_pid_init(&pid, 0.0025f, 0.026f, 0.00005f, 20.0f, 0.0f, 1.0f);

    // Provide massive speed error for multiple steps
    for (int i = 0; i < 50; i++) {
        digital_pid_update(&pid, 3000.0f, 0.0f, 0.001f);
    }
    assert(pid.output_duty == 1.0f);
    assert(pid.saturated == true);

    // Now set actual speed above target to verify immediate recovery without huge windup delay
    float duty_rev = digital_pid_update(&pid, 3000.0f, 3200.0f, 0.001f);
    assert(duty_rev < 1.0f); // Anti-windup ensures output immediately pulls out of saturation
    printf("  -> Digital PID anti-windup clamping PASSED.\n");
}

int main() {
    printf("===========================================\n");
    printf(" Running BLDC Motor Control Unit Tests    \n");
    printf("===========================================\n");
    test_hall_decoding();
    test_inverter_gate_commutation();
    test_digital_pid_anti_windup();
    printf("===========================================\n");
    printf(" ALL BLDC UNIT TESTS PASSED SUCCESSFULLY! \n");
    printf("===========================================\n");
    return 0;
}
