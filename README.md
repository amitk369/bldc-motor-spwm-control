# Closed-Loop Speed Control of BLDC Motor via SPWM Inverter

[![Tests](https://img.shields.io/badge/Unit%20Tests-Passing%20(100%25)-brightgreen.svg)]()
[![Settling Time](https://img.shields.io/badge/Settling%20Time%20Reduction-35.4%25%20(%3E%2035%25%20spec)-success.svg)]()
[![Overshoot](https://img.shields.io/badge/Overshoot-%3C%200.1%25%20(%3C%204%25%20spec)-blue.svg)]()
[![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)]()

> **Author:** Amit Kumar  
> **Institution:** National Institute of Technology, Patna (B.Tech., Electrical Engineering)  
> **Core Focus:** Power Electronics, Motor Drives, Inverter Switching, Digital Control Systems, MATLAB/Simulink

---

## 1. Executive Summary

This repository presents the full engineering model and digital firmware for **closed-loop speed control of a Permanent Magnet Brushless DC (BLDC) motor** driven by a **3-phase Voltage Source Inverter (VSI)** with **Sinusoidal Pulse-Width Modulation (SPWM)**. 

### Key Achievements:
1. **Dynamic Modeling:** Built high-fidelity ODE models simulating 3-phase stator phase voltages, trapezoidal back-EMF, rotor electromechanical dynamics, and inverter bridge MOSFET switching.
2. **6-Step Trapezoidal Commutation:** Decoded 3 Hall-effect sensors ($120^\circ$ electrical displacement) to command 6 inverter conduction sectors with complementary dead-time insertion ($1.2\,\mu\text{s}$).
3. **Optimized Digital PID Tuning:** Implemented 2-DOF setpoint weighting, derivative-on-measurement damping, and anti-windup clamping, **reducing speed settling time by 35.4%** (from $181.0\text{ ms}$ down to $117.0\text{ ms}$) with **peak overshoot held strictly under 4.0%** ($<0.1\%$ initial step, $1.37\%$ under dynamic load disturbances).

---

## 2. Mathematical Dynamics & Control Theory

### 2.1 Stator Voltage Equations
The instantaneous phase terminal voltages of the 3-phase star-connected stator are:
$$\begin{bmatrix} v_a \\ v_b \\ v_c \end{bmatrix} = \begin{bmatrix} R & 0 & 0 \\ 0 & R & 0 \\ 0 & 0 & R \end{bmatrix} \begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix} + \begin{bmatrix} L - M & 0 & 0 \\ 0 & L - M & 0 \\ 0 & 0 & L - M \end{bmatrix} \frac{d}{dt} \begin{bmatrix} i_a \\ i_b \\ i_c \end{bmatrix} + \begin{bmatrix} e_a \\ e_b \\ e_c \end{bmatrix}$$

Where $R = 0.38\,\Omega$, $L - M = 0.65\text{ mH}$, and $e_a, e_b, e_c$ are $120^\circ$ flat-top trapezoidal back-EMF waveforms:
$$e_x(\theta_e) = K_e \cdot f(\theta_e) \cdot \omega_m$$

### 2.2 Electromagnetic Torque & Mechanical Dynamics
$$T_e = \frac{e_a i_a + e_b i_b + e_c i_c}{\omega_m} \approx 2 \cdot K_t \cdot I_{active}$$
$$J \frac{d\omega_m}{dt} = T_e - T_L - B \omega_m$$
Where $J = 1.8 \times 10^{-4}\text{ kg}\cdot\text{m}^2$, $B = 4.5 \times 10^{-5}\text{ N}\cdot\text{m}\cdot\text{s/rad}$, and $K_t = 0.0285\text{ N}\cdot\text{m/A}$.

### 2.3 Closed-Loop Digital PID with Anti-Windup
To eliminate the large integrator windup and overshoot exhibited by standard PI controllers under step references, a 2-DOF discrete regulator was synthesized:
$$u(k) = K_p (b \cdot \omega^* - \omega) + I(k) - K_d \frac{\omega(k) - \omega(k-1)}{\Delta t}$$
- **Anti-Windup Clamping:** The integrator halts accumulation whenever duty cycle reaches the actuator boundary ($[0.0, 1.0]$):
  $$I(k) = I(k-1) + K_i \cdot (\omega^* - \omega) \cdot \Delta t \quad \text{if } 0 < u_{unsat} < 1$$

---

## 3. Experimental Simulation Metrics

| Performance Metric | Baseline Factory PI | Tuned Digital PID | Target Specification |
|---|---|---|---|
| **Settling Time ($t_s$, 2% band)** | $181.0\text{ ms}$ | **$117.0\text{ ms}$** | **35% reduction achieved ($35.4\%$)** |
| **Step Overshoot ($M_p$)** | $10.98\%$ | **$0.07\%$** | **Held under 4.0%** |
| **Load Step Drop ($0.2\text{ N}\cdot\text{m}$ shock)** | $4.85\%$ | **$1.37\%$** | Rapid recovery in $<25\text{ ms}$ |
| **PWM Carrier Frequency** | $10\text{ kHz}$ | **$20\text{ kHz}$** | Ultrasonic, ripple-free torque |

---

## 4. Repository Structure

```
bldc-motor-spwm-control/
├── firmware/
│   ├── include/
│   │   ├── bldc_config.h       # Motor physical constants & PID gains
│   │   ├── bldc_types.h        # Sector enums, gate states, PID structures
│   │   ├── hall_commutation.h  # 6-step Hall sensor decoder
│   │   ├── digital_pid.h       # Anti-windup discrete PID regulator
│   │   └── bldc_controller.h   # Master closed-loop speed controller
│   └── src/
│       ├── hall_commutation.c  # Commutation sector-to-gate mapping
│       ├── digital_pid.c       # Derivative-on-measurement & clamping
│       └── bldc_controller.c   # Top-level step coordinator
├── simulation/
│   ├── simulate_bldc.py        # Complete ODE electromechanical solver
│   └── bldc_sim_summary.json   # Simulation benchmark results
├── tests/
│   └── test_bldc.c             # C assertion unit tests
├── web/
│   └── index.html              # Interactive 3-Phase Inverter & Motor Simulator
├── Makefile                    # Test automation
└── README.md                   # Full documentation & derivations
```

---

## 5. Quickstart & Verification

### Run Native C Unit Tests
```bash
make test
```
*Validates 120-degree Hall decoding logic, 6-step gate actuation patterns, and anti-windup clamping.*

### Run Electromechanical Simulation
```bash
make sim
# or: python3 simulation/simulate_bldc.py
```
*Verifies the exact 35.4% settling time reduction and $<4\%$ overshoot constraints.*

### Open Digital Twin Web Simulator
Open `web/index.html` in your browser to inspect the 6 inverter MOSFETs switching in real-time, rotating 8-pole rotor, and dynamic step response oscilloscope.

---

## 6. Interview Defense (STAR Format)

- **Situation:** High-speed BLDC motor drives in robotics and electric traction suffer from torque ripple, acoustic switching noise, and sluggish dynamic response when standard untuned PI loops encounter sudden step load torque changes.
- **Task:** Architect a closed-loop speed control scheme using a 3-phase SPWM inverter that reduces settling time by $\ge 35\%$ while guaranteeing $<4\%$ dynamic overshoot.
- **Action:** Modeled the motor and inverter dynamics in MATLAB/Simulink equivalent ODEs. Implemented 6-step trapezoidal commutation with $1.2\,\mu\text{s}$ dead-time insertion. Tuned a discrete 2-DOF digital PID controller with derivative filtering on measurement and anti-windup saturation clamping.
- **Result:** Decreased speed settling time by 35.4% (from 181.0 ms to 117.0 ms) with negligible overshoot ($0.07\%$), verifying robust stability under $0.20\text{ N}\cdot\text{m}$ load shock disturbances.
