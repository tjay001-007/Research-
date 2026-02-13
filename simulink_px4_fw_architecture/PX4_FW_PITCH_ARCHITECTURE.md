# PX4 Fixed-Wing Pitch Attitude Control Architecture — Simulink Blocks

## System Overview

```
 ┌─────────────────────────────────────────────────────────────────────┐
 │                    NAVIGATION / MISSION                            │
 │                  (altitude_sp, airspeed_sp)                        │
 └──────────────────────────┬──────────────────────────────────────────┘
                            │
 ┌──────────────────────────▼──────────────────────────────────────────┐
 │                                                                     │
 │   ┌─────────────────────────────────────────────────────────────┐   │
 │   │               TECS (Orange Block)                           │   │
 │   │         Total Energy Control System                         │   │
 │   │                                                             │   │
 │   │  alt_sp ──┐    ┌──────────┐    ┌──────────┐                │   │
 │   │           ├──► │ Alt Error │──► │ 1/(g*τ)  │──► pitch_sp   │   │
 │   │  alt_meas ┘    └──────────┘    └──────────┘                │   │
 │   │                                                             │   │
 │   │  aspd_sp ─┐    ┌──────────┐    ┌──────────┐                │   │
 │   │           ├──► │ Spd Error│──► │ Spd Gain │──► throttle    │   │
 │   │  aspd_meas┘    └──────────┘    └──────────┘                │   │
 │   │                                                             │   │
 │   │  Energy balance:  SPE - SKE → pitch                        │   │
 │   │  Total energy:    SPE + SKE → throttle                     │   │
 │   └────────────────────────┬────────────────────────────────────┘   │
 │                            │ pitch_sp                               │
 │   ┌────────────────────────▼────────────────────────────────────┐   │
 │   │          PITCH ATTITUDE CONTROLLER (Blue Block)             │   │
 │   │                  Outer Loop                                 │   │
 │   │                                                             │   │
 │   │  pitch_sp ──┐   ┌────────────┐   ┌──────────┐              │   │
 │   │             ├─► │ Pitch Error│─► │  Kp=12   │──┐           │   │
 │   │  pitch_meas ┘   └────────────┘   └──────────┘  │           │   │
 │   │                                                 + ──► q_sp  │   │
 │   │  roll_meas ──► ┌──────────────────────┐         │           │   │
 │   │               │ Turn Compensation     │─────────┘           │   │
 │   │  airspeed ──► │ q = g/V*(1/cosφ - 1)  │                    │   │
 │   │               └──────────────────────┘                      │   │
 │   │                                                             │   │
 │   │  Output: rate_sp (q_cmd) clamped to ±120 deg/s             │   │
 │   └────────────────────────┬────────────────────────────────────┘   │
 │                            │ rate_sp (q_cmd)                        │
 │   ┌────────────────────────▼────────────────────────────────────┐   │
 │   │          PITCH RATE CONTROLLER (Green Block)                │   │
 │   │                  Inner Loop                                 │   │
 │   │                                                             │   │
 │   │  rate_sp ──┐    ┌────────────┐                              │   │
 │   │            ├──► │ Rate Error │──┬──► [Kp=0.4] ──┐          │   │
 │   │  rate_meas ┘    └────────────┘  │                │          │   │
 │   │                                 ├──► [Ki=0.3]─►∫─┤ ──► Σ ──┤   │
 │   │                                 │    (clamp±0.4) │    ↓     │   │
 │   │  rate_meas ──► [d/dt] ──► [-Kd=0.015] ──────────┘  elev   │   │
 │   │                (on meas)                          [-1,+1]   │   │
 │   │                                                             │   │
 │   │  Features:                                                  │   │
 │   │   • Derivative on measurement (no setpoint kick)            │   │
 │   │   • Anti-windup integrator clamping                         │   │
 │   │   • Conditional integration (freeze when saturated)         │   │
 │   └────────────────────────┬────────────────────────────────────┘   │
 │                            │ elevator [-1, +1]                      │
 └────────────────────────────┼────────────────────────────────────────┘
                              │
 ┌────────────────────────────▼────────────────────────────────────────┐
 │               AIRCRAFT PLANT (Red Block)                            │
 │                                                                     │
 │   elevator ──► [M_eff] ──┐                                         │
 │                          + ──► [1/Iyy] ──► ∫ q ──► ∫ θ            │
 │   θ ──► [Cmα_eff] ──────┤    (q_dot)        │       │             │
 │   q ──► [Cmq_eff] ──────┘                   │       ├──► pitch    │
 │                                              │       │             │
 │   PITCH UNSTABLE: Cmα = +0.3 (destabilizing)│       │             │
 │   Pitch damping:  Cmq = -3.0 (stabilizing)  │       │             │
 │                                              ▼       │             │
 │   Altitude: h_dot = V * sin(θ) ──► ∫ h ──► altitude │             │
 │                                                      ▼             │
 │   Mass = 2.0 kg, Iyy = 0.1 kg·m²                 pitch_rate       │
 └──────────────┬──────────────┬──────────────┬────────────────────────┘
                │              │              │
 ┌──────────────▼──────────────▼──────────────▼────────────────────────┐
 │               SENSORS (Gray Block)                                  │
 │                                                                     │
 │   pitch_true ──► [AHRS LPF τ=20ms]  ──► pitch_meas  ──► feedback  │
 │   rate_true  ──► [Gyro LPF τ=10ms]  ──► rate_meas   ──► feedback  │
 │   alt_true   ──► [Baro LPF τ=50ms]  ──► alt_meas    ──► feedback  │
 └─────────────────────────────────────────────────────────────────────┘
```

## Simulink Block Color Legend

| Color  | Subsystem                | PX4 Module             |
|--------|--------------------------|------------------------|
| Orange | TECS                     | `fw_pos_control_l1`    |
| Blue   | Pitch Attitude (Outer)   | `fw_att_control`       |
| Green  | Pitch Rate (Inner)       | `fw_att_control`       |
| Red    | Aircraft Plant           | (real aircraft / sim)  |
| Gray   | Sensors                  | EKF2 / sensor drivers  |
| Yellow | Input Setpoints          | Navigator / RC input   |

## PX4 Parameter Mapping

### Pitch Rate Controller (Inner Loop)

| PX4 Parameter | Simulink Block     | Value  | Description                     |
|---------------|--------------------|--------|---------------------------------|
| `FW_PR_P`     | `Kp_Rate`          | 0.4    | Pitch rate proportional gain    |
| `FW_PR_I`     | `Ki_Rate`          | 0.3    | Pitch rate integral gain        |
| `FW_PR_D`     | `Kd_Rate`          | 0.015  | Pitch rate derivative gain      |
| `FW_PR_IMAX`  | `Integrator` limit | 0.4    | Integrator anti-windup limit    |
| `FW_P_RMAX`   | `Rate_Sat`         | 120°/s | Maximum commanded pitch rate    |

### Pitch Attitude Controller (Outer Loop)

| PX4 Parameter | Simulink Block     | Value  | Description                     |
|---------------|--------------------|--------|---------------------------------|
| `FW_P_TC`     | `Kp_Pitch` (1/TC)  | 12.0   | Pitch attitude P gain           |
| `FW_P_LIM_MAX`| Pitch_Sat upper    | +45°   | Maximum pitch angle             |
| `FW_P_LIM_MIN`| Pitch_Sat lower    | -45°   | Minimum pitch angle             |

### TECS (Total Energy Control System)

| PX4 Parameter    | Simulink Block       | Value  | Description                  |
|------------------|----------------------|--------|------------------------------|
| `FW_T_CLMB_MAX`  | `ClimbRate_Sat`      | 5.0    | Max climb rate [m/s]         |
| `FW_T_SINK_MAX`  | `ClimbRate_Sat`      | 3.0    | Max sink rate [m/s]          |
| `FW_T_TAU`       | `Tau_Inv` (1/tau)    | 5.0    | Energy time constant [s]     |
| `FW_T_PTCH_DAMP` | `Energy_to_Pitch`    | 0.7    | Pitch damping ratio          |
| `FW_T_SPDWEIGHT` | Energy balance       | 1.0    | Speed vs altitude priority   |

## How to Use

```matlab
% Step 1: Load parameters
px4_fw_pitch_parameters;

% Step 2: Build the Simulink model
build_px4_fw_pitch_architecture;

% Step 3: Run simulation
run_pitch_simulation;

% Or build + run in one step:
run_pitch_simulation;
```

## Subsystem Details

### Opening Individual Blocks
Double-click any colored subsystem in the Simulink model to see its
internal block diagram:

- **TECS**: Shows altitude error → climb rate → energy balance → pitch setpoint
- **Pitch Attitude Ctrl**: Shows P-controller + coordinated turn compensation
- **Pitch Rate Ctrl**: Shows full PID with derivative-on-measurement
- **Aircraft Plant**: Shows pitch dynamics with unstable Cmα feedback
- **Sensors**: Shows AHRS/gyro/baro low-pass filters

### Signal Flow
```
Setpoints → TECS → Attitude Ctrl → Rate Ctrl → Plant → Sensors → (feedback)
                                                   ↑                   │
                                                   └───────────────────┘
```
