// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Control Allocator
//
// Converts torque commands (roll/pitch/yaw) to control surface deflections
// using the effectiveness matrix approach from PX4's ControlAllocator module.
//
// deflection = effectiveness^{-1} * torque
//
// Handles saturation with feedback to indicate which axes are saturated,
// allowing the rate controller to stop integrating on saturated axes.

#pragma once

#include "math_types.hpp"
#include "params.hpp"

namespace px4_att_control {

struct ControlOutput {
    float aileron  = 0.f;   // [-1, 1]
    float elevator = 0.f;   // [-1, 1]
    float rudder   = 0.f;   // [-1, 1]
    float throttle = 0.f;   // [0, 1]

    // Per-axis saturation flags: nonzero if the surface is at its limit
    // Sign indicates direction (+1 = positive limit, -1 = negative limit)
    Vec3 saturation = Vec3::zero();
};

class ControlAllocator {
public:
    void set_params(const ControlAllocatorParams& p) {
        params_ = p;
        eff_inv_ = p.effectiveness.inversed();
    }

    const ControlAllocatorParams& params() const { return params_; }

    // Allocate torque commands to control surfaces
    //   torque    - desired [roll, pitch, yaw] torques (normalized)
    //   thrust    - desired thrust [0, 1]
    ControlOutput allocate(const Vec3& torque, float thrust) {
        // Invert effectiveness matrix to get deflections
        Vec3 deflection = eff_inv_ * torque;

        ControlOutput out;
        out.saturation = Vec3::zero();

        // Clamp and record saturation
        out.aileron  = clamp_with_sat(deflection.x,
                                       params_.deflection_min.x,
                                       params_.deflection_max.x,
                                       out.saturation.x);
        out.elevator = clamp_with_sat(deflection.y,
                                       params_.deflection_min.y,
                                       params_.deflection_max.y,
                                       out.saturation.y);
        out.rudder   = clamp_with_sat(deflection.z,
                                       params_.deflection_min.z,
                                       params_.deflection_max.z,
                                       out.saturation.z);

        // Throttle is independent of the 3-axis allocation
        out.throttle = constrain(thrust, params_.throttle_min, params_.throttle_max);

        return out;
    }

private:
    ControlAllocatorParams params_;
    Mat33 eff_inv_ = Mat33::identity();

    // Clamp value and set saturation indicator
    static float clamp_with_sat(float val, float lo, float hi, float& sat) {
        if (val > hi) {
            sat = 1.f;
            return hi;
        } else if (val < lo) {
            sat = -1.f;
            return lo;
        }
        sat = 0.f;
        return val;
    }
};

} // namespace px4_att_control
