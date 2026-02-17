# PX4 Fixed-Wing Controller Architecture — Detailed Explanation

## Table of Contents

1. [Architecture Overview](#1-architecture-overview)
2. [Module 1: fw_att_control — Attitude Controller](#2-fw_att_control--fixed-wing-attitude-controller)
3. [Module 2: fw_rate_control — Rate Controller](#3-fw_rate_control--fixed-wing-rate-controller)
4. [Module 3: fw_autotune_attitude_control — Autotuner](#4-fw_autotune_attitude_control--attitude-autotuner)
5. [Module 4: fw_lateral_longitudinal_control — TECS/NPFG Path Controller](#5-fw_lateral_longitudinal_control--tecsnpfg-path-controller)
6. [Module 5: fw_mode_manager — Mode Manager & Navigation](#6-fw_mode_manager--mode-manager--navigation)
7. [Control Signal Flow](#7-control-signal-flow--complete-path)
8. [Folder Structure Reference](#8-folder-structure)

---

## 1. Architecture Overview

The PX4 fixed-wing control stack uses a **cascaded controller architecture** with clear separation of concerns. The signal flows from high-level navigation down to actuator outputs:

```
┌─────────────────────────────────────────────────────────────────────┐
│                     fw_mode_manager                                 │
│   (Navigation: waypoints, loiter, takeoff, landing, figure-8)      │
│   Outputs: lateral_setpoint, longitudinal_setpoint, position_sp    │
└─────────────────────────┬───────────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────────┐
│               fw_lateral_longitudinal_control                       │
│   (TECS: pitch/throttle from altitude+airspeed error)               │
│   (NPFG: roll from course/lateral-acceleration error)               │
│   Outputs: vehicle_attitude_setpoint (roll, pitch, yaw, thrust)     │
└─────────────────────────┬───────────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────────┐
│                      fw_att_control                                  │
│   (Attitude → Rate conversion using P-controllers + Jacobian)       │
│   Outputs: vehicle_rates_setpoint (roll_rate, pitch_rate, yaw_rate) │
└─────────────────────────┬───────────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────────┐
│                     fw_rate_control                                   │
│   (PID rate controller → torque/thrust setpoints)                    │
│   Outputs: vehicle_torque_setpoint, vehicle_thrust_setpoint          │
└─────────────────────────┬───────────────────────────────────────────┘
                          │
┌─────────────────────────▼───────────────────────────────────────────┐
│                    Control Allocator (mixer)                          │
│   (Maps torque/thrust to individual servo/motor PWM outputs)         │
└─────────────────────────────────────────────────────────────────────┘
```

A separate **fw_autotune_attitude_control** module can inject system identification signals to auto-tune the rate PID gains.

All modules communicate via **uORB** — PX4's publish/subscribe middleware inspired by DDS. Each module subscribes to upstream outputs and publishes its own results.

---

## 2. fw_att_control — Fixed-Wing Attitude Controller

### Purpose

Converts **attitude setpoints** (desired roll, pitch, yaw as quaternion) into **body rate setpoints** (desired roll_rate, pitch_rate, yaw_rate in rad/s). This is the "outer loop" in the attitude/rate cascade.

### Files

| File | Role |
|------|------|
| `FixedwingAttitudeControl.hpp` | Class declaration, uORB subscriptions/publications, parameters |
| `FixedwingAttitudeControl.cpp` | Main Run() loop, manual mode handling, VTOL tailsitter transforms |
| `fw_roll_controller.h/cpp` | Roll attitude → roll body rate (P-controller + Jacobian) |
| `fw_pitch_controller.h/cpp` | Pitch attitude → pitch body rate (P-controller + Jacobian) |
| `fw_yaw_controller.h/cpp` | Coordinated turn yaw rate calculation |
| `fw_wheel_controller.h/cpp` | Ground steering wheel PID controller |
| `fw_att_control_params.c` | Parameter definitions (time constants, rate limits) |

### How It Works — Step by Step

#### 2.1 Initialization & Scheduling

```cpp
FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint)
                          : ORB_ID(vehicle_attitude_setpoint))
```

The module runs as a **ScheduledWorkItem** triggered by new `vehicle_attitude` messages (from the EKF/estimator). If no attitude update arrives within 20ms, it runs anyway as a backup. For VTOL aircraft, it publishes to virtual topics that the VTOL transition manager consumes.

#### 2.2 The Main Run() Loop

Every cycle (~250Hz, matching IMU rate):

1. **Read parameters** if changed (`_parameter_update_sub.updated()`)
2. **Get current attitude** from EKF as quaternion → convert to rotation matrix `_R` and Euler angles
3. **Tailsitter handling**: For VTOL tailsitters, the rotation matrix is transformed by swapping roll/yaw columns and inverting the new x-axis, because the tailsitter's "hover" frame is 90° rotated from its "forward flight" frame
4. **Manual input processing** (`vehicle_manual_poll`): In STABILIZED mode, converts stick inputs to attitude setpoints
5. **Read attitude setpoint** from upstream controller
6. **State checks**: Determine if we're in fixed-wing mode, in transition, or landed
7. **Run attitude sub-controllers** (roll, pitch, yaw)
8. **Publish rate setpoints**
9. **Run wheel controller** if on ground with runway steering enabled

#### 2.3 Roll Controller (`fw_roll_controller.cpp`)

A simple **proportional controller with Euler-to-body-rate Jacobian transformation**:

```cpp
float RollController::control_roll(float roll_setpoint, float euler_yaw_rate_setpoint,
                                    float roll, float pitch) {
    const float roll_error = roll_setpoint - roll;
    _euler_rate_setpoint = roll_error / _tc;  // P-gain = 1/time_constant

    // Jacobian: euler rates → body rates
    // roll_body = roll_euler - sin(pitch) * yaw_euler
    const float roll_body_rate_setpoint_raw = _euler_rate_setpoint
        - sinf(pitch) * euler_yaw_rate_setpoint;
    _body_rate_setpoint = math::constrain(roll_body_rate_setpoint_raw, -_max_rate, _max_rate);
}
```

**Key concept**: The Euler-to-body-rate Jacobian accounts for the kinematic coupling between axes. When pitched up, a pure yaw Euler rate creates a component of roll body rate — the `sin(pitch)` term corrects for this.

The gain is expressed as `1/time_constant` (parameter `FW_R_TC`). A smaller time constant = more aggressive response.

#### 2.4 Pitch Controller (`fw_pitch_controller.cpp`)

Same P-controller structure with a different Jacobian row:

```cpp
float PitchController::control_pitch(float pitch_setpoint, float euler_yaw_rate_setpoint,
                                      float roll, float pitch) {
    const float pitch_error = pitch_setpoint - pitch;
    _euler_rate_setpoint = pitch_error / _tc;

    // Jacobian: pitch_body = cos(roll) * pitch_euler + cos(pitch) * sin(roll) * yaw_euler
    const float pitch_body_rate_setpoint_raw = cosf(roll) * _euler_rate_setpoint +
        cosf(pitch) * sinf(roll) * euler_yaw_rate_setpoint;
    _body_rate_setpoint = math::constrain(pitch_body_rate_setpoint_raw,
                                           -_max_rate_neg, _max_rate_pos);
}
```

Note: pitch has **asymmetric rate limits** (`_max_rate_pos` and `_max_rate_neg`) since aircraft typically have different authority for nose-up vs nose-down.

#### 2.5 Yaw Controller — Coordinated Turn (`fw_yaw_controller.cpp`)

Unlike roll and pitch, the yaw controller does NOT track a yaw setpoint. Instead it computes the yaw rate required for a **coordinated turn** (zero sideslip):

```cpp
// Coordinated turn: yaw_rate = tan(roll) * cos(pitch) * g / airspeed
_euler_rate_setpoint = tanf(constrained_roll) * cosf(pitch) * CONSTANTS_ONE_G / airspeed;

// Jacobian: yaw_body = -sin(roll) * pitch_euler_rate + cos(roll) * cos(pitch) * yaw_euler_rate
const float yaw_body_rate_setpoint_raw = -sinf(roll) * euler_pitch_rate_setpoint +
    cosf(roll) * cosf(pitch) * _euler_rate_setpoint;
```

This comes from the flight dynamics constraint: in a banked turn at constant altitude, the horizontal component of lift provides centripetal acceleration. The formula `ψ̇ = g·tan(φ)/V` is the classic coordinated turn equation.

The controller handles **inverted flight** (|roll| > 90°) by clamping the roll to avoid the tan(90°) singularity.

#### 2.6 Wheel Controller (`fw_wheel_controller.cpp`)

A full **PID controller** for ground steering during runway operations:

```cpp
// Attitude (outer): P-controller on yaw error
_body_rate_setpoint = wrap_pi(yaw_setpoint - yaw) / _tc;

// Rate (inner): PI + FF controller
_last_output = _body_rate_setpoint * _k_ff * groundspeed_scaler +
    groundspeed_scaler * groundspeed_scaler * (rate_error * _k_p) + _integrator;
```

The controller uses **groundspeed scaling** — at higher speeds, less steering authority is needed (similar to how car steering effort changes with speed). The integrator only runs above 1 m/s to prevent windup while stationary.

#### 2.7 Autotune Integration

The attitude controller checks for signals from the autotuner:
```cpp
if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
    if (pid_autotune.state == ..._STATE_ROLL || ...) {
        bodyrate_autotune_ff = matrix::Vector3f(pid_autotune.rate_sp);
        body_rates_setpoint += bodyrate_autotune_ff;  // inject excitation signal
    }
}
```

---

## 3. fw_rate_control — Fixed-Wing Rate Controller

### Purpose

Converts **body rate setpoints** (desired angular velocities) into **torque and thrust setpoints** that the control allocator maps to servo deflections and motor commands. This is the "inner loop."

### Files

| File | Role |
|------|------|
| `FixedwingRateControl.hpp` | Class declaration, PID rate controller, airspeed scaling |
| `FixedwingRateControl.cpp` | Main Run() loop, trim scheduling, manual mode, flaps/spoilers |
| `fw_rate_control_params.c` | PID gains (P, I, D, FF for each axis), trim values |

### How It Works

#### 3.1 The Rate Control Loop

The core uses PX4's `RateControl` library class (a PID controller with anti-windup):

```cpp
// Configure PID gains
const Vector3f rate_p = Vector3f(_param_fw_rr_p.get(), _param_fw_pr_p.get(), _param_fw_yr_p.get());
const Vector3f rate_i = Vector3f(_param_fw_rr_i.get(), _param_fw_pr_i.get(), _param_fw_yr_i.get());
const Vector3f rate_d = Vector3f(_param_fw_rr_d.get(), _param_fw_pr_d.get(), _param_fw_yr_d.get());
_rate_control.setPidGains(rate_p, rate_i, rate_d);
```

Each cycle:
1. Read angular velocity from gyroscope (via `vehicle_angular_velocity`)
2. Read rate setpoints from attitude controller
3. Compute PID output: `angular_acceleration_setpoint = PID(rates, body_rates_setpoint, dt)`
4. Apply airspeed scaling: `control_u = angular_acceleration_setpoint * airspeed_scaling²`
5. Add trim offsets
6. Publish torque and thrust setpoints

#### 3.2 Airspeed Scaling

A critical concept for fixed-wing control — aerodynamic control surface effectiveness scales with **dynamic pressure** (proportional to V²):

```cpp
if (_param_fw_arsp_scale_en.get()) {
    const float airspeed_constrained = math::max(airspeed, min_airspeed);
    _airspeed_scaling = _param_fw_airspd_trim.get() / airspeed_constrained;
}
```

The scaling factor is `V_trim / V_actual`. At higher airspeed, smaller deflections produce the same force, so the controller output is scaled down. At lower speeds, larger deflections are needed.

The torque output is then: `control_u = gain_compression * angular_accel_sp * airspeed_scaling²`

The feed-forward gain is inversely scaled: `scaled_gain_ff = gain_ff / airspeed_scaling`

#### 3.3 Trim Scheduling

Actuator trims are interpolated based on airspeed to handle the fact that aircraft balance changes with speed:

```cpp
Vector3f trim(_param_trim_roll.get(), _param_trim_pitch.get(), _param_trim_yaw.get());
trim *= _airspeed_scaling * _airspeed_scaling;

if (airspeed < _param_fw_airspd_trim.get()) {
    // Interpolate between min-airspeed trim and nominal trim
    trim(0) += interpolate(airspeed, _param_fw_airspd_min.get(),
                           _param_fw_airspd_trim.get(), _param_fw_dtrim_r_vmin.get(), 0.0f);
} else {
    // Interpolate between nominal trim and max-airspeed trim
    trim(0) += interpolate(airspeed, _param_fw_airspd_trim.get(),
                           _param_fw_airspd_max.get(), 0.0f, _param_fw_dtrim_r_vmax.get());
}
```

#### 3.4 Anti-Windup via Control Allocation Feedback

The rate controller receives saturation information back from the control allocator to implement proper anti-windup:

```cpp
if (_control_allocator_status_subs[0].update(&control_allocator_status)) {
    for (size_t i = 0; i < 3; i++) {
        _rate_control.setPositiveSaturationFlag(i,
            control_allocator_status.unallocated_torque[i] > FLT_EPSILON);
        _rate_control.setNegativeSaturationFlag(i,
            control_allocator_status.unallocated_torque[i] < -FLT_EPSILON);
    }
}
```

When the allocator cannot achieve the demanded torque (servos at limits), the integrator stops winding up in that direction.

#### 3.5 Roll-to-Yaw Feed-Forward

Counters adverse yaw during roll maneuvers:
```cpp
_vehicle_torque_setpoint.xyz[2] = math::constrain(
    _vehicle_torque_setpoint.xyz[2] + _param_fw_rll_to_yaw_ff.get() * _vehicle_torque_setpoint.xyz[0],
    -1.f, 1.f);
```

#### 3.6 Manual Modes

- **ACRO mode**: Stick inputs map directly to rate setpoints (`stick * max_acro_rate`)
- **MANUAL mode**: Stick inputs map directly to torque/thrust with scaling factors
- **Flaps/Spoilers**: Published from manual RC input channels when in manual mode

#### 3.7 Battery Scaling

Optionally scales thrust to compensate for battery voltage drop:
```cpp
if (_param_fw_bat_scale_en.get() && _vehicle_thrust_setpoint.xyz[0] > 0.1f) {
    _vehicle_thrust_setpoint.xyz[0] *= _battery_scale;
}
```

---

## 4. fw_autotune_attitude_control — Attitude Autotuner

### Purpose

Automatically tunes the PID rate gains by performing **system identification** during flight. It injects excitation signals (steps or sine sweeps), measures the aircraft response, and computes optimal PID gains.

### Files

| File | Role |
|------|------|
| `fw_autotune_attitude_control.hpp` | State machine, system ID object, gain storage |
| `fw_autotune_attitude_control.cpp` | State machine logic, signal generation, gain computation |
| `fw_autotune_attitude_control_params.c` | Autotune configuration parameters |

### How It Works

#### 4.1 State Machine

The autotuner follows a precise sequence:

```
idle → init → roll_amp_detection → roll → roll_pause →
               pitch_amp_detection → pitch → pitch_pause →
               yaw_amp_detection → yaw → yaw_pause →
               apply → test → verification → complete
```

- **Amplitude Detection States**: Before exciting each axis, the autotuner gradually increases the excitation amplitude until the aircraft shows a measurable response. This ensures the signal-to-noise ratio is sufficient for system identification.
- **Identification States** (roll/pitch/yaw): The excitation signal is injected and the system response is recorded.
- **Pause States**: Brief pauses between axes to let transients settle.
- **Apply/Test/Verify**: The computed gains are applied and tested to verify stability.

#### 4.2 Excitation Signals

Three signal types are supported (configurable via `FW_AT_SYSID_TYPE`):
- **Step signals** (`kStep`): Simple step inputs — good for quick identification
- **Linear sine sweep** (`kLinearSineSweep`): Frequency sweeps from f0 to f1 — better frequency response characterization
- **Logarithmic sine sweep** (`kLogSineSweep`): Log-spaced sweeps — better low-frequency resolution

The signal is injected additively into the rate setpoints through the `autotune_attitude_control_status` topic.

#### 4.3 System Identification

Uses PX4's `SystemIdentification` library which implements a recursive least-squares algorithm to fit a transfer function model to the input-output data. From the identified model, optimal PID gains are computed using the `pid_design` library.

#### 4.4 Gain Application

Gains can be applied automatically or require manual confirmation (parameter `FW_AT_APPLY`):
```cpp
void FwAutotuneAttitudeControl::saveGainsToParams() {
    // Write computed gains to FW_RR_P, FW_RR_I, FW_RR_FF, etc.
    // Also updates attitude time constants FW_R_TC, FW_P_TC
}
```

The original gains are backed up so they can be reverted if the new gains cause instability.

---

## 5. fw_lateral_longitudinal_control — TECS/NPFG Path Controller

### Purpose

This is the **path-following and energy management** controller. It converts lateral/longitudinal setpoints (course, altitude, airspeed) into attitude and thrust setpoints.

It uses two key algorithms:
- **TECS** (Total Energy Control System): Manages the trade-off between kinetic energy (airspeed) and potential energy (altitude) using pitch and throttle
- **NPFG/Airspeed Direction Controller**: Computes the roll angle needed to follow a desired ground track or course

### Files

| File | Role |
|------|------|
| `FwLateralLongitudinalControl.hpp` | Class with TECS, NPFG, PerformanceModel |
| `FwLateralLongitudinalControl.cpp` | Main loop: runs TECS and NPFG, publishes attitude setpoint |
| `fw_lat_long_params.c` | TECS tuning params, roll limits, airspeed params |

### How It Works

#### 5.1 Subscriptions (Inputs)

The controller subscribes to:
- `fixed_wing_lateral_setpoint`: Desired course/heading, lateral acceleration, or airspeed direction
- `fixed_wing_longitudinal_setpoint`: Desired altitude, height rate, equivalent airspeed, or direct pitch/throttle
- `vehicle_local_position`: Current position, velocity, altitude
- `airspeed_validated`: Current calibrated airspeed
- `wind`: Wind estimate for ground-track calculations

#### 5.2 Lateral Control (NPFG → Roll Setpoint)

The lateral controller converts lateral guidance commands into a roll angle:

```
Lateral acceleration setpoint → Roll angle via:
    roll = atan(lateral_accel / g)

or equivalently:
    roll = atan(V² / (g * R))  where R is turn radius
```

The `CourseToAirspeedRefMapper` converts a desired course angle into an airspeed direction reference, and the `AirspeedDirectionController` tracks this reference to produce lateral acceleration commands.

Key features:
- **Roll slew rate limiting** (`FW_PN_R_SLEW_MAX`): Prevents abrupt roll changes
- **Roll angle limits** (`FW_R_LIM`): Maximum bank angle
- **Wind compensation**: Accounts for wind when converting between course and heading

#### 5.3 Longitudinal Control (TECS → Pitch + Throttle)

TECS manages the total energy (kinetic + potential) of the aircraft:

```
Total Energy = ½mV² + mgh
Specific Total Energy Rate = V̇ + g·ḣ  (normalized by mg)

Throttle controls total energy rate (adding/removing energy)
Pitch controls energy distribution (trading speed for height)
```

The controller:
1. Computes altitude error and airspeed error
2. Converts to total energy rate error and energy distribution error
3. Uses PI controllers to compute throttle and pitch commands
4. Respects airframe limits (min/max throttle, pitch limits, climb/sink rates)

Key parameters:
- `FW_T_ALT_TC`: Altitude tracking time constant
- `FW_T_HRATE_FF`: Height rate feed-forward
- `FW_T_PTCH_DAMP`: Pitch damping
- `FW_T_THR_DAMPING`: Throttle damping
- `FW_T_SPDWEIGHT`: Speed vs height priority weighting (0=height priority, 2=speed priority, 1=balanced)

#### 5.4 Output

The controller publishes a `vehicle_attitude_setpoint` containing:
- Roll angle (from lateral controller)
- Pitch angle (from TECS)
- Yaw (typically current heading — yaw is handled by coordinated turn in attitude controller)
- Thrust (from TECS)
- Flaps/spoilers setpoints

---

## 6. fw_mode_manager — Mode Manager & Navigation

### Purpose

The largest and most complex module. It implements **all fixed-wing flight modes** and translates high-level navigation commands into the lateral/longitudinal setpoints consumed by `fw_lateral_longitudinal_control`.

### Files

| File | Role |
|------|------|
| `FixedWingModeManager.hpp/cpp` | Main class: mode logic for missions, loiter, RTL, takeoff, landing |
| `ControllerConfigurationHandler.hpp/cpp` | Manages lateral/longitudinal controller configurations |
| `launchdetection/LaunchDetector.h/cpp` | Catapult/hand-launch detection via accelerometer |
| `runway_takeoff/RunwayTakeoff.h/cpp` | Runway takeoff state machine |
| `figure_eight/FigureEight.hpp/cpp` | Figure-of-eight pattern generation |
| `fw_mode_manager_params.c` | Mode-specific parameters |

### Flight Modes Implemented

#### 6.1 AUTO MISSION
- Follows waypoint sequences from the navigator
- Computes course to next waypoint
- Handles altitude setpoints between waypoints
- Manages transitions between mission items

#### 6.2 AUTO LOITER
- Flies circular orbits around a point
- Computes tangent course for circle following
- Configurable radius and direction

#### 6.3 AUTO TAKEOFF
Two sub-modes:
- **Catapult/Hand Launch**: Uses `LaunchDetector` which monitors accelerometer for a high-g spike indicating launch. Control surfaces are held neutral until launch is detected, then climbs to clearance altitude.
- **Runway Takeoff**: `RunwayTakeoff` state machine manages ground roll → rotation → climb phases with wheel steering on the ground.

#### 6.4 AUTO LAND
Multi-phase landing:
1. **Approach**: Flies to landing approach point
2. **Glide slope**: Follows terrain-referenced glide path
3. **Flare**: Reduces sink rate near ground
4. **Touchdown**: Ground roll with wheel braking
- Supports terrain following via distance sensor
- Touchdown position nudging via stick inputs

#### 6.5 AUTO RTL (Return To Launch)
- Flies to home position
- Loiters at safe altitude
- Can transition to landing

#### 6.6 POSITION HOLD (Manual)
- Altitude hold via TECS
- Heading hold with deadband
- Stick inputs override heading/altitude

#### 6.7 ALTITUDE HOLD (Manual)
- Throttle controls altitude via TECS
- Stick roll/pitch for lateral control
- No heading hold

#### 6.8 Figure-of-Eight
- Generates figure-8 patterns for survey/mapping
- Computes dual-loiter geometry

### Controller Configuration Handler

The `ControllerConfigurationHandler` manages runtime reconfiguration of the lateral and longitudinal controllers based on the current flight phase. For example, during landing flare, it may tighten the altitude tracking or change pitch limits:

```cpp
// Publishes configurations like:
lateral_control_configuration_s
longitudinal_control_configuration_s
```

---

## 7. Control Signal Flow — Complete Path

Here is the complete data flow for a typical AUTO MISSION flight:

```
Navigator publishes position_setpoint_triplet (waypoints)
         │
         ▼
fw_mode_manager reads waypoints, computes:
  → fixed_wing_lateral_setpoint   (course to waypoint = 045°)
  → fixed_wing_longitudinal_setpoint (altitude=100m, airspeed=15m/s)
         │
         ▼
fw_lateral_longitudinal_control reads setpoints:
  NPFG: course_error=5° → lateral_accel=1.2m/s² → roll_sp=7°
  TECS: alt_error=+3m, aspd_error=-0.5m/s → pitch_sp=2°, thrust=0.6
  → publishes vehicle_attitude_setpoint (q_d from roll=7°, pitch=2°)
         │
         ▼
fw_att_control reads attitude setpoint:
  roll_ctrl:  roll_error=7°-5°=2° → roll_rate_sp = 2°/0.4s = 5°/s
  pitch_ctrl: pitch_error=2°-1°=1° → pitch_rate_sp = 1°/0.5s = 2°/s
  yaw_ctrl:   coordinated turn → yaw_rate_sp = tan(7°)·cos(2°)·9.81/15 = 0.08 rad/s
  → publishes vehicle_rates_setpoint (p=5°/s, q=2°/s, r=4.6°/s)
         │
         ▼
fw_rate_control reads rate setpoint:
  PID on each axis + airspeed_scaling + trim
  roll:  (5°/s - 4.8°/s) * Kp + ∫ * Ki + FF = 0.05 + trim_roll
  pitch: (2°/s - 1.9°/s) * Kp + ∫ * Ki + FF = 0.03 + trim_pitch
  yaw:   (4.6°/s - 4.5°/s) * Kp + ∫ * Ki + FF = 0.01 + trim_yaw
  → publishes vehicle_torque_setpoint  (xyz = [0.05, 0.03, 0.01])
  → publishes vehicle_thrust_setpoint  (xyz = [0.6, 0, 0])
         │
         ▼
Control Allocator maps to servos:
  aileron_left  = +0.05
  aileron_right = -0.05
  elevator      = +0.03
  rudder        = +0.01
  motor_throttle = 0.6
```

### VTOL Tailsitter Transformations

Throughout the stack, special handling exists for VTOL tailsitters. The key insight is that the control interfaces (uORB messages) are always in the **hover (multicopter) body frame**, but the fixed-wing controllers operate in the **fixed-wing frame**. Transformations are applied at the boundaries:

**Attitude controller** (entering FW frame):
```cpp
// Swap roll and yaw columns in rotation matrix, negate new x-axis
R_adapted(0, 2) = _R(0, 0); // etc.
```

**Rate controller** (entering FW frame):
```cpp
rates = Vector3f(-angular_velocity.xyz[2], angular_velocity.xyz[1], angular_velocity.xyz[0]);
```

**Rate controller** (leaving back to hover frame):
```cpp
_vehicle_torque_setpoint.xyz[0] = _vehicle_torque_setpoint.xyz[2];
_vehicle_torque_setpoint.xyz[2] = -helper;
```

---

## 8. Folder Structure

```
px4_fw_controllers/
├── PX4_FIXED_WING_CONTROLLER_DETAILED_EXPLANATION.md  (this file)
│
├── fw_att_control/                    # Attitude Controller (outer loop)
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── FixedwingAttitudeControl.hpp   # Main class declaration
│   ├── FixedwingAttitudeControl.cpp   # Main Run() loop
│   ├── fw_roll_controller.h           # Roll P-controller interface
│   ├── fw_roll_controller.cpp         # Roll P-controller + Jacobian
│   ├── fw_pitch_controller.h          # Pitch P-controller interface
│   ├── fw_pitch_controller.cpp        # Pitch P-controller + Jacobian
│   ├── fw_yaw_controller.h            # Coordinated turn controller interface
│   ├── fw_yaw_controller.cpp          # Coordinated turn yaw rate calc
│   ├── fw_wheel_controller.h          # Ground steering PID interface
│   ├── fw_wheel_controller.cpp        # Ground steering PID
│   └── fw_att_control_params.c        # Parameter definitions
│
├── fw_rate_control/                   # Rate Controller (inner loop)
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── FixedwingRateControl.hpp       # Class with RateControl, airspeed scaling
│   ├── FixedwingRateControl.cpp       # PID rate control, trim, flaps/spoilers
│   └── fw_rate_control_params.c       # PID gains, trims, scaling params
│
├── fw_autotune_attitude_control/      # Automatic PID Tuner
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── fw_autotune_attitude_control.hpp  # State machine, sys-id objects
│   ├── fw_autotune_attitude_control.cpp  # Autotune logic, signal gen, gain calc
│   └── fw_autotune_attitude_control_params.c  # Autotune configuration
│
├── fw_lateral_longitudinal_control/   # TECS + NPFG Path Controller
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── FwLateralLongitudinalControl.hpp  # TECS, NPFG, PerformanceModel
│   ├── FwLateralLongitudinalControl.cpp  # Energy mgmt + lateral guidance
│   └── fw_lat_long_params.c              # TECS tuning, roll/pitch limits
│
└── fw_mode_manager/                   # Mode Manager & Navigation
    ├── CMakeLists.txt
    ├── Kconfig
    ├── FixedWingModeManager.hpp        # All mode implementations
    ├── FixedWingModeManager.cpp        # Mode logic (~2800 lines)
    ├── ControllerConfigurationHandler.hpp  # Runtime controller config
    ├── ControllerConfigurationHandler.cpp
    ├── fw_mode_manager_params.c
    ├── launchdetection/
    │   ├── CMakeLists.txt
    │   ├── LaunchDetector.h            # Catapult/hand launch detection
    │   ├── LaunchDetector.cpp
    │   └── launchdetection_params.c
    ├── runway_takeoff/
    │   ├── CMakeLists.txt
    │   ├── RunwayTakeoff.h             # Runway takeoff state machine
    │   ├── RunwayTakeoff.cpp
    │   └── runway_takeoff_params.c
    └── figure_eight/
        ├── CMakeLists.txt
        ├── FigureEight.hpp             # Figure-8 pattern generation
        └── FigureEight.cpp
```

---

## Key Design Patterns Used Throughout

1. **uORB Publish/Subscribe**: All inter-module communication uses typed messages. Modules are decoupled — they can be developed and tested independently.

2. **ScheduledWorkItem / WorkItem**: Modules run on PX4's work queue system, triggered by new data (usually IMU or position updates). This provides deterministic timing without dedicated threads.

3. **ModuleBase + ModuleParams**: Standard PX4 module interface with parameter management. Parameters are declared with `DEFINE_PARAMETERS()` macro and auto-synced from the parameter store.

4. **Cascaded Control**: Each level runs at an appropriate rate — rate control at ~250Hz (gyro rate), attitude control at ~250Hz, position/path control at ~50Hz, navigation at ~10Hz.

5. **Graceful Degradation**: Airspeed fallback to trim speed, wind estimate timeout handling, control surface locking during launch detection, integrator resets on landing.
