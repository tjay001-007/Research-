# NDI/INDI Flight Control System — Simulink Integration Guide

This repository contains three progressive implementations of Nonlinear Dynamic Inversion (NDI/INDI) flight control for statically unstable aircraft, each with Level-2 MATLAB S-Function wrappers for Simulink integration.

---

## Repository Structure

```
Research-/
│
├── ndi_flight_controller.m        # Core NDI algorithm (standalone function)
├── ndi_controller_sfunc.m         # Level-2 S-Function wrapper for basic NDI
├── setup_ndi_controller.m         # Parameter setup + stability analysis
├── run_ndi_closed_loop.m          # Pure-MATLAB simulation (no Simulink needed)
│
├── fighter_ndi/                   # Production-grade fighter NDI
│   ├── fighter_aero_model.m       # Tabular aerodynamics (alpha-Mach lookup)
│   ├── ndi_production_controller.m # 6-stage FCS
│   ├── setup_fighter.m            # Fighter parameters + full aero database
│   ├── run_fighter_sim.m          # Pure-MATLAB combat maneuver simulation
│   ├── fighter_plant_sfunc.m      # S-Function: 6-DOF plant
│   ├── ndi_fcs_sfunc.m            # S-Function: NDI/INDI controller
│   ├── build_ndi_simulink_model.m # Programmatic Simulink model builder
│   └── plot_results.m             # Post-simulation analysis plots
│
└── ucav_ndi/                      # Autonomous UCAV with INDI + trajectory following
    ├── setup_ucav.m               # Master config (aircraft, INDI, guidance, mission)
    ├── ucav_aerodynamics.m        # Aero model (aileron+elevator+rudder, NO canard)
    ├── indi_controller.m          # Core INDI algorithm
    ├── ucav_guidance_law.m        # L1 lateral + PI altitude/speed guidance
    ├── run_ucav_mission.m         # Pure-MATLAB 800s mission simulation
    ├── ucav_plant_sfunc.m         # S-Function: 6-DOF UCAV plant
    ├── indi_controller_sfunc.m    # S-Function: INDI controller
    ├── ucav_guidance_sfunc.m      # S-Function: L1+PI guidance
    ├── build_ucav_simulink_model.m # Programmatic Simulink model builder
    └── INDI_README.md             # Detailed INDI theory and integration guide
```

---

## Quick Start

### Basic NDI (no Simulink)

```matlab
run_ndi_closed_loop    % 30s pitch/roll step response
```

### Fighter NDI in Simulink

```matlab
cd fighter_ndi
setup_fighter
build_ndi_simulink_model
sim('fighter_ndi_sim')
plot_results
```

### UCAV INDI Mission (no Simulink)

```matlab
cd ucav_ndi
setup_ucav
run_ucav_mission       % 800s autonomous patrol mission
```

### UCAV INDI in Simulink

```matlab
cd ucav_ndi
setup_ucav
build_ucav_simulink_model
sim('ucav_indi_sim')
```

---

## Key Implementations

### 1. Basic NDI Demo (root level)

Textbook cascaded NDI for pitch-unstable aircraft. Good for understanding the fundamentals.

### 2. Production Fighter NDI (`fighter_ndi/`)

6-stage FCS with tabular aero, carefree handling, INDI option, structural filters. Delta-canard configuration with 4 control surfaces.

### 3. Autonomous UCAV INDI (`ucav_ndi/`)

**This is the primary implementation.** Features:
- **Conventional tail configuration** (aileron + elevator + rudder, NO canard)
- **INDI controller** — sensor-based, robust to model errors
- **L1 lateral guidance** — same algorithm as PX4/ArduPilot
- **PI altitude/speed control** with anti-windup
- **Racetrack patrol mission** defined in WGS-84 (lat/lon/alt)
- **Full 6-DOF dynamics** with Ixz inertia coupling and ISA atmosphere
- **Actuator dynamics** (first-order lag + rate limit + saturation)

See [`ucav_ndi/INDI_README.md`](ucav_ndi/INDI_README.md) for detailed INDI theory and Simulink integration instructions.

---

## How Level-2 S-Functions Work

All S-functions follow this lifecycle:

```
Start()             → Load parameters from workspace (evalin('base',...))
InitializeConditions() → Zero integrators, set initial DWork
[Simulation Loop]:
  Outputs()         → Read inputs, call algorithm, write outputs
  Update()          → Update DWork (integrators, filters, previous commands)
```

Key patterns:
- Parameters loaded via `evalin('base',...)` and stored with `setappdata()`
- Persistent states stored in DWork vectors
- Discrete sample time: `block.SampleTimes = [0.004, 0]` (250 Hz)

---

## Manual Simulink Integration

1. Run the setup script (`setup_ucav` or `setup_fighter`)
2. Add **Level-2 MATLAB S-Function** blocks from the Simulink library
3. Set the S-function name (e.g., `indi_controller_sfunc`)
4. Add actuator dynamics between controller and plant
5. Wire feedback from plant outputs to controller/guidance inputs
6. Set solver to fixed-step ODE4 at 0.004 s
7. Run simulation

See the INDI README for detailed port reference tables.

---

## Prerequisites

- MATLAB R2016b or later
- Simulink (for Simulink integration; pure-MATLAB scripts work without it)
- No additional toolboxes required

---

## Troubleshooting

| Issue | Cause | Fix |
|-------|-------|-----|
| "aircraft struct not found" | Setup not run | Run `setup_ucav` or `setup_fighter` |
| S-function not found | Not on path | `addpath('ucav_ndi')` or `addpath('fighter_ndi')` |
| Simulation diverges | No controller in loop | Ensure INDI/NDI feedback is connected |
| Oscillation | Gains too high | Reduce K_q and K_p first |
| Poor trajectory tracking | Guidance gains mismatch | Adjust L1_ratio and Kh for your airspeed |
