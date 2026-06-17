# PSIM Power Electronics & Motor Control

![Language](https://img.shields.io/badge/LANGUAGE-C-00599C?style=for-the-badge&labelColor=444444)
![Editor](https://img.shields.io/badge/EDITOR-VISUAL%20STUDIO-5C2D91?style=for-the-badge&labelColor=444444)
![Simulation](https://img.shields.io/badge/SIMULATION-PSIM-0078D4?style=for-the-badge&labelColor=444444)

## Overview

A PSIM-based power electronics and motor control study archive built with C controller code and simulated in PSIM.

## Project Areas

| Area | Topics |
|---|---|
| Power Electronics | Buck converter, 3-phase PWM inverter |
| PWM Modulation | SVPWM, offset SVPWM, THIPWM, DPWM, Bolognani method |
| Reference Frame | abc transformation, stationary frame, synchronous dq frame |
| Motor Control | DC motor control, induction motor control, PMSM vector and high-speed control |

## Project List

| Area | Original Folder | Focus |
|---|---|---|
| Power Electronics | `buck_1` | Buck converter control practice |
| Power Electronics | `pwm_inverter` | 3-phase PWM inverter implementation |
| PWM Modulation | `space_vector` | Space vector PWM sector and duty calculation |
| PWM Modulation | `offset_svpwm` | Offset voltage injection based SVPWM |
| PWM Modulation | `offset_thipwm` | Third-harmonic injection PWM offset method |
| PWM Modulation | `30degree_dpwm` | 30-degree discontinuous PWM modulation |
| PWM Modulation | `60degree_dpwm` | 60-degree discontinuous PWM modulation |
| PWM Modulation | `60degree_dpwm_2` | 60-degree DPWM variant experiment |
| PWM Modulation | `120degree_dpwm` | 120-degree discontinuous PWM modulation |
| PWM Modulation | `bolognani` | Bolognani method based voltage vector modulation |
| Reference Frame | `d-q_trans` | abc, stationary, and synchronous dq transformation |
| Motor Control | `motor_control_dc` | DC motor current, field, and speed control |
| Motor Control | `v-f_control` | Induction motor V/f control |
| Motor Control | `IM_vector` | Induction motor vector control |
| Motor Control | `IM_highspeed` | Induction motor high-speed control |
| Motor Control | `PM_vector` | PMSM vector control |
| Motor Control | `PM_highspeed` | PMSM high-speed control |

## Structure

```text
.
|-- projects/
|   |-- power-electronics/
|   |-- pwm-modulation/
|   |-- reference-frame/
|   `-- motor-control/
|-- .gitignore
`-- README.md
```

Each project should keep a simple review-oriented layout:

```text
project-name/
|-- psim/
|   `-- main.psimsch
`-- controller.c
```

## How to Use

1. Choose a project from the project list.
2. Open `psim/main.psimsch` in PSIM to inspect the simulation circuit.
3. Review `controller.c` to understand the C-based controller implementation written in Visual Studio.
