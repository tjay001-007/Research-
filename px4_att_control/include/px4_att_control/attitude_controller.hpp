// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Attitude Controller
//
// Outer-loop controller that converts attitude error (quaternion) to
// body-rate setpoints. Mirrors PX4 FixedwingAttitudeControl module.
//
// Architecture:
//   attitude error = q_current^{-1} * q_setpoint
//   rate_sp = 2 * error.xyz / time_constant  (proportional on reduced error)
//   + coordinated-turn yaw rate
//   + feed-forward from guidance rate commands

#pragma once

#include "math_types.hpp"
#include "params.hpp"

namespace px4_att_control {

struct AttitudeSetpoint {
    Quatf q_sp;                       // Desired attitude quaternion
    Vec3  rate_ff = Vec3::zero();     // Feed-forward body rates from guidance (rad/s)
    float thrust_sp = 0.f;            // Desired thrust [0, 1]
};

struct RateSetpoint {
    Vec3  rate_sp = Vec3::zero();     // Commanded body rates (rad/s)
    float thrust_sp = 0.f;            // Pass-through thrust
};

class AttitudeController {
public:
    void set_params(const AttitudeControlParams& p) { params_ = p; }
    const AttitudeControlParams& params() const { return params_; }

    // Main update: compute rate setpoints from current and desired attitude
    //   q         - current attitude quaternion (NED body)
    //   sp        - attitude setpoint (desired quaternion + ff rates + thrust)
    //   airspeed  - current indicated airspeed (m/s), used for coordinated turn
    RateSetpoint update(const Quatf& q, const AttitudeSetpoint& sp, float airspeed) {
        // --- Attitude error (body frame) ---
        Vec3 att_error = Quatf::reduced_error(q, sp.q_sp);

        // --- Proportional control: error / time_constant ---
        Vec3 rate_sp;
        rate_sp.x = att_error.x / params_.roll_time_constant;
        rate_sp.y = att_error.y / params_.pitch_time_constant;
        rate_sp.z = 0.f; // Yaw handled separately

        // --- Add feed-forward from guidance ---
        rate_sp.x += params_.roll_ff  * sp.rate_ff.x;
        rate_sp.y += params_.pitch_ff * sp.rate_ff.y;
        rate_sp.z += params_.yaw_ff   * sp.rate_ff.z;

        // --- Coordinated turn: yaw rate = g * tan(roll) / V ---
        if (params_.coordinated_turn && airspeed > 1.f) {
            Vec3 euler = q.to_euler();
            float roll = euler.x;
            constexpr float g = 9.80665f;
            float yaw_rate_coord = g * std::tan(roll) / std::max(airspeed, 5.f);
            rate_sp.z += yaw_rate_coord;
        }

        // --- Yaw attitude error contribution ---
        rate_sp.z += att_error.z / params_.roll_time_constant; // Yaw uses roll TC as proxy

        // --- Rate limiting ---
        rate_sp.x = constrain(rate_sp.x, -params_.roll_rate_max,  params_.roll_rate_max);
        rate_sp.y = constrain(rate_sp.y,  params_.pitch_rate_min, params_.pitch_rate_max);
        rate_sp.z = constrain(rate_sp.z, -params_.yaw_rate_max,   params_.yaw_rate_max);

        return {rate_sp, sp.thrust_sp};
    }

private:
    AttitudeControlParams params_;
};

} // namespace px4_att_control
