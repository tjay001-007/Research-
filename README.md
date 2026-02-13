# Research-

Pitch-unstable aircraft simulation and PX4 fixed-wing attitude control research.

## Contents

- `aircraft_simulink_project.tar-2.gz` — MATLAB/Simulink 6-DOF aircraft simulation with UDP bridge to Gazebo
- `px4_fw_architecture.docx` — PX4 fixed-wing autopilot architecture reference
- `px4_att_control/` — C++17 header-only attitude control library (see below)

## px4\_att\_control Library

A standalone C++17 implementation of PX4's fixed-wing attitude control pipeline:

| Layer | File | Description |
|-------|------|-------------|
| Math | `math_types.hpp` | Vec3, Mat33, Quatf with quaternion error computation |
| Params | `params.hpp` | All PX4 FW attitude/rate parameters with defaults |
| Attitude | `attitude_controller.hpp` | Quaternion error → body-rate setpoints (outer loop) |
| Rate | `rate_controller.hpp` | PID+FF with airspeed scaling, D-term filter, anti-windup (inner loop) |
| Allocator | `control_allocator.hpp` | Effectiveness matrix inversion with saturation feedback |
| Pipeline | `attitude_control_pipeline.hpp` | Full cascaded controller in one call |

### Building

```bash
cd px4_att_control
mkdir build && cd build
cmake ..
make
```

### Running Examples

```bash
# Pitch-unstable aircraft disturbance rejection (outputs CSV)
./pitch_unstable_sim > pitch_unstable.csv

# Step response comparison: stable vs unstable gains
./step_response > step_response.csv
```

### Usage

```cpp
#include "px4_att_control/px4_att_control.hpp"
using namespace px4_att_control;

AttitudeControlPipeline pipeline;
pipeline.configure_pitch_unstable();

VehicleState state;
state.attitude   = current_quaternion;
state.body_rates = current_gyro;
state.airspeed   = current_airspeed;

AttitudeSetpoint sp;
sp.q_sp       = desired_quaternion;
sp.thrust_sp  = 0.5f;

ControlOutput ctrl = pipeline.update(state, sp, dt);
// ctrl.aileron, ctrl.elevator, ctrl.rudder, ctrl.throttle
```