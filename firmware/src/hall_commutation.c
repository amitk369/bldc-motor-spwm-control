/**
 * @file hall_commutation.c
 * @brief 6-Step Trapezoidal Commutation Implementation
 * @author Amit Kumar (NIT Patna - Electrical Engineering)
 */

#include "hall_commutation.h"

bldc_sector_t hall_decode_sector(uint8_t hall_bits) {
    // hall_bits format: (Ha << 2) | (Hb << 1) | Hc
    switch (hall_bits & 0x07) {
        case 5: return COMMUTATION_SECTOR_1; // 1 0 1
        case 4: return COMMUTATION_SECTOR_2; // 1 0 0
        case 6: return COMMUTATION_SECTOR_3; // 1 1 0
        case 2: return COMMUTATION_SECTOR_4; // 0 1 0
        case 3: return COMMUTATION_SECTOR_5; // 0 1 1
        case 1: return COMMUTATION_SECTOR_6; // 0 0 1
        default: return COMMUTATION_INVALID; // 0 or 7 are invalid in 120 deg placement
    }
}

void hall_update_inverter_gates(bldc_sector_t sector, float duty_cycle, inverter_gate_signals_t *gates) {
    gates->duty_cycle = (duty_cycle < 0.0f) ? 0.0f : ((duty_cycle > 1.0f) ? 1.0f : duty_cycle);

    switch (sector) {
        case COMMUTATION_SECTOR_1:
            // U -> W (U positive, W negative, V floating)
            gates->phase_u = SWITCH_PWM_HIGH;
            gates->phase_v = SWITCH_FLOATING;
            gates->phase_w = SWITCH_LOW_ON;
            break;

        case COMMUTATION_SECTOR_2:
            // U -> V (U positive, V negative, W floating)
            gates->phase_u = SWITCH_PWM_HIGH;
            gates->phase_v = SWITCH_LOW_ON;
            gates->phase_w = SWITCH_FLOATING;
            break;

        case COMMUTATION_SECTOR_3:
            // W -> V (W positive, V negative, U floating)
            gates->phase_u = SWITCH_FLOATING;
            gates->phase_v = SWITCH_LOW_ON;
            gates->phase_w = SWITCH_PWM_HIGH;
            break;

        case COMMUTATION_SECTOR_4:
            // W -> U (W positive, U negative, V floating)
            gates->phase_u = SWITCH_LOW_ON;
            gates->phase_v = SWITCH_FLOATING;
            gates->phase_w = SWITCH_PWM_HIGH;
            break;

        case COMMUTATION_SECTOR_5:
            // V -> U (V positive, U negative, W floating)
            gates->phase_u = SWITCH_LOW_ON;
            gates->phase_v = SWITCH_PWM_HIGH;
            gates->phase_w = SWITCH_FLOATING;
            break;

        case COMMUTATION_SECTOR_6:
            // V -> W (V positive, W negative, U floating)
            gates->phase_u = SWITCH_FLOATING;
            gates->phase_v = SWITCH_PWM_HIGH;
            gates->phase_w = SWITCH_LOW_ON;
            break;

        default:
        case COMMUTATION_INVALID:
            // All phases floating for safety
            gates->phase_u = SWITCH_FLOATING;
            gates->phase_v = SWITCH_FLOATING;
            gates->phase_w = SWITCH_FLOATING;
            gates->duty_cycle = 0.0f;
            break;
    }
}
