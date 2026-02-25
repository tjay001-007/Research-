# Incremental Nonlinear Dynamic Inversion (INDI) — Theory and Implementation

This document explains the INDI control law implemented in `indi_controller.m` and its Simulink wrapper `indi_controller_sfunc.m`, including the mathematical formulation, architecture, advantages over classical NDI, and step-by-step integration guide.

---

## Table of Contents

1. [What is INDI?](#what-is-indi)
2. [Why INDI for Unstable Aircraft?](#why-indi-for-unstable-aircraft)
3. [Mathematical Formulation](#mathematical-formulation)
4. [Controller Architecture](#controller-architecture)
5. [The B Matrix (Control Effectiveness)](#the-b-matrix-control-effectiveness)
6. [INDI vs Classical NDI — Comparison](#indi-vs-classical-ndi--comparison)
7. [Implementation Details](#implementation-details)
8. [Simulink Integration](#simulink-integration)
9. [Gain Tuning Guide](#gain-tuning-guide)
10. [Files Reference](#files-reference)

---

## What is INDI?

**Incremental Nonlinear Dynamic Inversion (INDI)** is a sensor-based flight control technique that achieves robust nonlinear control without requiring an accurate aerodynamic model.

The key idea is simple: instead of computing the total control command from a full model of the aircraft (as in classical NDI), INDI computes only the **incremental change** in the control command by comparing the **desired angular acceleration** with the **measured angular acceleration**.

```
Classical NDI:
  delta = B_inv * (omega_dot_desired - f(x))        ← needs full model f(x)

INDI:
  delta = delta_prev + B_inv * (omega_dot_desired - omega_dot_measured)
                                                      ← needs only B and sensor
```

The measured angular acceleration (`omega_dot_measured`) implicitly contains all aerodynamic effects — including nonlinearities, wind gusts, model errors, and even structural damage. INDI only needs the **control effectiveness matrix B** (how much angular acceleration each surface produces per radian of deflection) to be reasonably accurate.

---

## Why INDI for Unstable Aircraft?

This UCAV has **positive Cm_alpha** (pitching moment increases with angle of attack), making it statically unstable. Without active control, any pitch perturbation grows exponentially with a time-to-double of approximately 0.5 seconds.

INDI is particularly well-suited for unstable aircraft because:

1. **Robustness to model errors**: The unstable Cm_alpha creates large aerodynamic moments that are difficult to model precisely. Classical NDI must cancel these moments using the model — any error leads to residual instability. INDI uses the measured angular acceleration, so model errors in Cm_alpha don't affect performance.

2. **Automatic disturbance rejection**: Wind gusts, CG shifts, and aerodynamic nonlinearities all appear in the measured omega_dot. INDI rejects them automatically without explicit modeling.

3. **Reduced gain sensitivity**: Classical NDI requires high gains to compensate for model uncertainty in the unstable pitch axis. INDI achieves the same performance with lower gains because the cancellation is more accurate.

4. **Graceful degradation**: If the aircraft is damaged (e.g., control surface partially lost), the measured omega_dot reflects the reduced effectiveness, and INDI adapts implicitly.

---

## Mathematical Formulation

### Rotational Equations of Motion

The angular dynamics of a rigid aircraft are:

```
J * omega_dot = M_aero(x) + G * delta - omega x (J * omega)
```

where:
- `J` = inertia tensor (3x3, including Ixz)
- `omega` = [p; q; r] = body angular rates
- `omega_dot` = [pdot; qdot; rdot] = angular accelerations
- `M_aero(x)` = aerodynamic moments (function of full state — the complex part)
- `G * delta` = control moments (G = moment derivatives, delta = surface deflections)
- `omega x (J * omega)` = gyroscopic/inertia coupling

### Classical NDI

Classical NDI solves for delta that produces the desired omega_dot:

```
omega_dot_desired = K * (omega_cmd - omega) + Ki * integral(omega_cmd - omega)

delta = B_inv * (omega_dot_desired - J_inv * (M_aero(x) - omega x (J * omega)))
```

This requires knowing `M_aero(x)` — the total aerodynamic moment as a function of alpha, beta, Mach, rates, etc. Any error in this model directly degrades performance.

### INDI Derivation

At the current time step, the measured angular acceleration is:

```
omega_dot_meas = J_inv * (M_aero_current + G * delta_current - omega x (J * omega))
```

At the next time step, with a new control input `delta_new`:

```
omega_dot_new = J_inv * (M_aero_new + G * delta_new - omega x (J * omega))
```

Taking the difference (and assuming `M_aero` doesn't change much in one time step):

```
omega_dot_new - omega_dot_meas ≈ J_inv * G * (delta_new - delta_current)
                                = B * (delta_new - delta_current)
```

where `B = J_inv * G` is the **control effectiveness matrix**.

Solving for the control increment:

```
delta_new = delta_current + B_inv * (omega_dot_desired - omega_dot_meas)
```

**This is the INDI law.** Notice that `M_aero(x)` has completely disappeared — it is replaced by the measurement `omega_dot_meas`.

### Assumption

The key assumption is that `M_aero` doesn't change significantly between consecutive time steps. At 250 Hz (dt = 4 ms), this is very well satisfied — aerodynamic moments change on timescales of ~50–200 ms (10–50x slower than the control rate).

---

## Controller Architecture

```
                         OUTER LOOP                    INNER LOOP (INDI)
                    ┌─────────────────┐          ┌──────────────────────────┐
                    │                 │          │                          │
 phi_cmd ──┐       │  Kinematic      │ p_cmd    │   omega_dot_des =        │  da_cmd
 theta_cmd ┼──────>│  Inversion      ├─q_cmd──>│   K*(omega_cmd - omega)  ├──de_cmd──>
            │       │  (Euler to      │ r_cmd    │   + Ki * integral        │  dr_cmd
            │       │   body rates)   │          │                          │
            │       └─────────────────┘          │   delta_u = B_inv *      │
            │                                     │   (omega_dot_des -       │
   phi ─────┘                                     │    omega_dot_meas)       │
   theta ──── (from plant)                        │                          │
   beta ───── (computed)                          │   delta = delta_prev     │
                                                   │         + delta_u        │
   omega_dot_meas ─── (from plant) ──────────────>│                          │
   delta_prev ──────── (from actuators) ─────────>│                          │
                                                   └──────────────────────────┘
```

### Outer Loop: Attitude to Rate Commands

The outer loop converts attitude errors into body rate commands using kinematic inversion:

```matlab
p_cmd = K_phi * (phi_cmd - phi)                    % Roll rate from roll error
q_cmd = K_theta * (theta_cmd - theta)              % Pitch rate from pitch error
r_cmd = g*sin(phi)*cos(theta)/V - K_beta * beta    % Coordinated turn + sideslip
```

The yaw rate command includes a coordinated turn term (`g*sin(phi)*cos(theta)/V`) that produces the correct yaw rate for a banked turn, plus a sideslip regulation term.

### Inner Loop: INDI

The inner loop computes the desired angular acceleration and inverts it through the B matrix:

```matlab
% Desired angular acceleration
pdot_des = K_p * (p_cmd - p) + int_p
qdot_des = K_q * (q_cmd - q) + int_q
rdot_des = K_r * (r_cmd - r) + int_r

% INDI control law
delta_u = B_inv * ([pdot_des; qdot_des; rdot_des] - omega_dot_meas)
delta_new = delta_prev + delta_u
```

---

## The B Matrix (Control Effectiveness)

The B matrix maps control surface deflection increments to angular acceleration increments:

```
B = J_inv * G
```

where:

```
G = qbar * S * [ b*Cl_da,    0,         b*Cl_dr  ]
                [ 0,          c*Cm_de,   0         ]
                [ b*Cn_da,    0,         b*Cn_dr   ]
```

and `J_inv` includes the Ixz coupling:

```
J_inv = [ Izz/Gamma,  0,       Ixz/Gamma ]
        [ 0,          1/Iyy,   0          ]
        [ Ixz/Gamma,  0,       Ixx/Gamma  ]

Gamma = Ixx * Izz - Ixz^2
```

The B matrix is 3x3 (square) because we have 3 control surfaces (aileron, elevator, rudder) and 3 rotational axes (roll, pitch, yaw). It is always invertible as long as each surface has primary authority over its respective axis.

**Note**: B depends on dynamic pressure (`qbar = 0.5 * rho * V^2`). At low speed, B elements become small (less control authority). The controller accounts for this by computing B at the current flight condition.

---

## INDI vs Classical NDI — Comparison

| Aspect | Classical NDI | INDI |
|--------|---------------|------|
| **Model dependency** | Requires full aero model M(x) | Only needs B matrix |
| **Model accuracy needed** | High (errors degrade stability) | Low (only B, ~20% error OK) |
| **Disturbance rejection** | Requires disturbance model | Automatic via measurement |
| **Wind gust handling** | Explicit gust model needed | Implicit cancellation |
| **Damage tolerance** | Fails if model doesn't match | Degrades gracefully |
| **Computational cost** | Higher (full model evaluation) | Lower (B matrix + 3x3 inverse) |
| **Sensor requirement** | Standard IMU | IMU + angular accel (or gyro differentiation) |
| **Gain sensitivity** | High on unstable axis | Lower (better cancellation) |
| **Tuning difficulty** | Harder (model + gains) | Easier (mostly B + rate gains) |

### When to use INDI over NDI

- Aircraft with significant model uncertainty
- Statically unstable configurations
- Operations in turbulent conditions
- When graceful degradation is required (military/UAV)
- When a high-fidelity aero model is not available

### When classical NDI might be preferred

- Very well-characterized aircraft with accurate models
- When angular acceleration sensors are not available
- Low-rate controllers where the INDI assumption breaks down

---

## Implementation Details

### Angular Acceleration Measurement

INDI requires measured angular acceleration (`omega_dot`). There are two approaches:

**1. Direct sensor (ideal)**: Some modern IMUs provide angular acceleration output. In the Simulink implementation, the plant S-function outputs `omega_dot` on port 6, which is fed directly to the INDI controller.

**2. Gyro differentiation**: In real flight hardware, differentiate gyro measurements:
```matlab
omega_dot_meas = (omega_current - omega_previous) / dt
```
This requires a low-pass filter to avoid noise amplification (typically 2nd-order Butterworth at 30–50 Hz for a 250 Hz controller).

### Anti-Windup

All integrators use clamping anti-windup: the integrator output is hard-limited to prevent wind-up when the actuators saturate. The limits are set in `fcs.int_lim_p/q/r`.

### Output Rate Limiting

The controller limits the rate of change of surface commands to prevent exciting structural modes and to stay within actuator rate capability:

```matlab
da_cmd = rate_limit(da_raw, da_prev, fcs.da_rate_max, dt)
```

### Position Saturation

Final surface commands are clamped to the physical actuator limits defined in `aircraft.act`.

---

## Simulink Integration

### Quick Start (Programmatic)

```matlab
cd ucav_ndi
setup_ucav                      % Load all parameters
build_ucav_simulink_model       % Creates ucav_indi_sim.slx
sim('ucav_indi_sim')            % Run 800s mission
```

### Manual Integration

1. **Add S-Function blocks** (from Simulink Library > User-Defined Functions > Level-2 MATLAB S-Function):

   | Block | S-function name | Purpose |
   |-------|----------------|---------|
   | Guidance | `ucav_guidance_sfunc` | L1 lateral + PI altitude/speed |
   | INDI Controller | `indi_controller_sfunc` | Incremental dynamic inversion |
   | Plant | `ucav_plant_sfunc` | 6-DOF aircraft dynamics |

2. **Add Actuator subsystem** between INDI and Plant:
   - 3 channels: aileron, elevator, rudder
   - Each: Transfer Fcn `1/(0.025s+1)` → Rate Limiter → Saturation

3. **Wire feedback**:
   - Plant outputs → Guidance inputs (position, velocity, euler, airdata)
   - Plant outputs → INDI inputs (velocity, omega, euler, omega_dot)
   - Actuator positions → INDI input (act_pos)
   - Guidance att_cmd → INDI att_cmd
   - INDI surf_cmd → Actuators
   - Guidance throttle → Plant throttle

4. **Configure solver**: Fixed-step ODE4, dt = 0.004 s

5. **Run `setup_ucav`** before starting the model.

### Port Reference

**Guidance S-Function (`ucav_guidance_sfunc`)**

| Port | Direction | Signal | Dim |
|------|-----------|--------|-----|
| In 1 | Input | position [N;E;D] | 3 |
| In 2 | Input | velocity [u;v;w] | 3 |
| In 3 | Input | euler [phi;theta;psi] | 3 |
| In 4 | Input | airdata [alpha;beta;Mach;V;qbar;alt] | 6 |
| Out 1 | Output | att_cmd [phi_cmd;theta_cmd] | 2 |
| Out 2 | Output | throttle | 1 |
| Out 3 | Output | debug [8 signals] | 8 |

**INDI Controller S-Function (`indi_controller_sfunc`)**

| Port | Direction | Signal | Dim |
|------|-----------|--------|-----|
| In 1 | Input | att_cmd [phi_cmd;theta_cmd] | 2 |
| In 2 | Input | velocity [u;v;w] | 3 |
| In 3 | Input | omega [p;q;r] | 3 |
| In 4 | Input | euler [phi;theta;psi] | 3 |
| In 5 | Input | omega_dot [pdot;qdot;rdot] | 3 |
| In 6 | Input | act_pos [da;de;dr] | 3 |
| Out 1 | Output | surf_cmd [da;de;dr] | 3 |
| Out 2 | Output | debug [8 signals] | 8 |

**Plant S-Function (`ucav_plant_sfunc`)**

| Port | Direction | Signal | Dim |
|------|-----------|--------|-----|
| In 1 | Input | da (aileron) | 1 |
| In 2 | Input | de (elevator) | 1 |
| In 3 | Input | dr (rudder) | 1 |
| In 4 | Input | throttle | 1 |
| Out 1 | Output | position [N;E;D] | 3 |
| Out 2 | Output | velocity [u;v;w] | 3 |
| Out 3 | Output | omega [p;q;r] | 3 |
| Out 4 | Output | euler [phi;theta;psi] | 3 |
| Out 5 | Output | accel [ax;ay;az] | 3 |
| Out 6 | Output | omega_dot [pdot;qdot;rdot] | 3 |
| Out 7 | Output | airdata [alpha;beta;Mach;V;qbar;alt] | 6 |

---

## Gain Tuning Guide

### Outer Loop (Attitude Tracking)

| Gain | Default | Effect |
|------|---------|--------|
| `fcs.K_phi` | 2.5 | Roll bandwidth. Increase for faster roll response. |
| `fcs.K_theta` | 3.0 | Pitch bandwidth. Higher than roll because pitch is unstable. |
| `fcs.K_beta` | 1.5 | Sideslip suppression. Higher = tighter coordination. |

**Rule of thumb**: Outer loop bandwidth should be 3–5x slower than inner loop.

### Inner Loop (Rate Tracking / INDI)

| Gain | Default | Effect |
|------|---------|--------|
| `fcs.K_p` | 8.0 | Roll rate proportional. Sets pdot per unit p error. |
| `fcs.K_q` | 10.0 | Pitch rate proportional. Highest because pitch is unstable. |
| `fcs.K_r` | 6.0 | Yaw rate proportional. |
| `fcs.Ki_p` | 1.5 | Roll rate integral. Rejects B-matrix errors. |
| `fcs.Ki_q` | 3.0 | Pitch rate integral. Higher for the unstable axis. |
| `fcs.Ki_r` | 1.0 | Yaw rate integral. |

**Tuning procedure**:
1. Start with integral gains at zero
2. Increase proportional gains until rate tracking is fast without oscillation
3. Add integral gain slowly until steady-state errors disappear
4. If oscillation occurs, reduce proportional gain first

### Guidance Gains

| Gain | Default | Effect |
|------|---------|--------|
| `guidance.L1_ratio` | 12 | L1 distance = V * ratio. Higher = smoother but slower turns. |
| `guidance.Kh` | 0.06 | Altitude P gain. Higher = tighter altitude hold. |
| `guidance.Ki_h` | 0.008 | Altitude I gain. Removes steady-state altitude error. |
| `guidance.Kv` | 0.08 | Speed P gain. Higher = tighter speed hold. |
| `guidance.Ki_v` | 0.02 | Speed I gain. |

---

## Files Reference

| File | Description |
|------|-------------|
| `setup_ucav.m` | Master configuration (aircraft, INDI gains, guidance, mission, trim) |
| `ucav_aerodynamics.m` | Aero forces/moments (3 surfaces: aileron, elevator, rudder) |
| `indi_controller.m` | Core INDI algorithm (standalone MATLAB function) |
| `ucav_guidance_law.m` | L1 lateral + PI altitude/speed guidance |
| `run_ucav_mission.m` | Pure-MATLAB simulation (800s, RK4, no Simulink needed) |
| `ucav_plant_sfunc.m` | Simulink S-Function: 6-DOF plant (12 continuous states) |
| `indi_controller_sfunc.m` | Simulink S-Function: INDI controller (1 DWork vector) |
| `ucav_guidance_sfunc.m` | Simulink S-Function: L1+PI guidance (1 DWork vector) |
| `build_ucav_simulink_model.m` | Programmatic Simulink model builder |

---

## References

1. Sieberling, S., Chu, Q. P., & Mulder, J. A. (2010). "Robust Flight Control Using Incremental Nonlinear Dynamic Inversion and Angular Acceleration Prediction." *AIAA Journal of Guidance, Control, and Dynamics*, 33(6), 1732–1742.

2. Smeur, E. J. J., Chu, Q. P., & de Croon, G. C. H. E. (2016). "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles." *AIAA Journal of Guidance, Control, and Dynamics*, 39(3), 450–461.

3. Grondman, F., Looye, G., Kuchar, R. O., Chu, Q. P., & Van Kampen, E. J. (2018). "Design and Flight Testing of Incremental Nonlinear Dynamic Inversion-based Control Laws for a Passenger Aircraft." *AIAA 2018-0385*.

4. Park, S., Deyst, J., & How, J. P. (2007). "Performance and Lyapunov Stability of a Nonlinear Path Following Guidance Method." *AIAA Journal of Guidance, Control, and Dynamics*, 30(6), 1718–1728.

5. Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2016). *Aircraft Control and Simulation*, 3rd Edition. Wiley.
