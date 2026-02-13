// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Full Pipeline
//
// Combines all three stages of PX4's fixed-wing attitude control:
//   1. Attitude Controller  (outer loop): attitude error -> rate setpoints
//   2. Rate Controller      (inner loop): rate error -> torque commands
//   3. Control Allocator:                 torque -> surface deflections
//
// This is the main interface for running the complete cascaded controller.

#pragma once

#include "attitude_controller.hpp"
#include "rate_controller.hpp"
#include "control_allocator.hpp"

namespace px4_att_control {

// Measured vehicle state required by the pipeline
struct VehicleState {
    Quatf attitude;                // Current attitude quaternion (NED -> body)
    Vec3  body_rates;              // Current angular rates in body frame (rad/s)
    float airspeed = 15.f;         // Indicated airspeed (m/s)
};

class AttitudeControlPipeline {
public:
    AttitudeControlPipeline() = default;

    // Configure with standard defaults
    void configure_defaults() {
        att_ctrl_.set_params(AttitudeControlParams{});
        rate_ctrl_.set_params(RateControlParams::defaults());
        alloc_.set_params(ControlAllocatorParams::fw_defaults());
    }

    // Configure for pitch-unstable aircraft (uses aggressive gains)
    void configure_pitch_unstable() {
        att_ctrl_.set_params(AttitudeControlParams::pitch_unstable_defaults());
        rate_ctrl_.set_params(RateControlParams::pitch_unstable_defaults());
        alloc_.set_params(ControlAllocatorParams::fw_defaults());
    }

    // Access individual controllers for fine-grained configuration
    AttitudeController& attitude_controller() { return att_ctrl_; }
    RateController&     rate_controller()     { return rate_ctrl_; }
    ControlAllocator&   control_allocator()   { return alloc_; }

    // Run the full cascade: attitude -> rate -> allocation
    //   state   - current vehicle state
    //   sp      - attitude setpoint from guidance
    //   dt      - timestep (seconds)
    ControlOutput update(const VehicleState& state,
                         const AttitudeSetpoint& sp,
                         float dt) {
        // Stage 1: Attitude -> Rate setpoints
        RateSetpoint rate_sp = att_ctrl_.update(state.attitude, sp, state.airspeed);

        // Stage 2: Rate -> Torque commands
        TorqueSetpoint torque = rate_ctrl_.update(
            rate_sp.rate_sp, state.body_rates,
            state.airspeed, rate_sp.thrust_sp,
            dt, last_saturation_);

        // Stage 3: Torque -> Control surface deflections
        ControlOutput out = alloc_.allocate(torque.torque, torque.thrust);

        // Store saturation for next iteration's anti-windup
        last_saturation_ = out.saturation;

        return out;
    }

    // Reset all controller states (call on mode transitions)
    void reset() {
        rate_ctrl_.reset();
        last_saturation_ = Vec3::zero();
    }

private:
    AttitudeController att_ctrl_;
    RateController     rate_ctrl_;
    ControlAllocator   alloc_;
    Vec3               last_saturation_ = Vec3::zero();
};

} // namespace px4_att_control
