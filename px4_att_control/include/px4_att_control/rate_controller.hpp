// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Rate Controller
//
// Inner-loop PID+FF controller that converts body-rate error to
// torque commands. Mirrors PX4 FixedwingRateControl module.
//
// Features:
//   - Per-axis PID with integrator anti-windup
//   - D-term low-pass filter (first-order)
//   - Airspeed scaling: gains * (trim_airspeed / indicated_airspeed)
//   - Integrator conditional damping on saturation

#pragma once

#include "math_types.hpp"
#include "params.hpp"

namespace px4_att_control {

struct TorqueSetpoint {
    Vec3  torque = Vec3::zero();      // Commanded torques [roll, pitch, yaw] (normalized)
    float thrust = 0.f;              // Commanded thrust [0, 1]
};

class RateController {
public:
    void set_params(const RateControlParams& p) { params_ = p; }
    const RateControlParams& params() const { return params_; }

    // Reset integrators and filter state
    void reset() {
        integral_    = Vec3::zero();
        rate_prev_   = Vec3::zero();
        rate_d_filt_ = Vec3::zero();
        initialized_ = false;
    }

    // Main update: compute torque commands from rate error
    //   rate_sp       - commanded body rates (rad/s)
    //   rate          - measured body rates (rad/s)
    //   airspeed      - indicated airspeed (m/s), for gain scaling
    //   thrust_sp     - desired thrust [0, 1]
    //   dt            - timestep (seconds)
    //   saturated     - output from control allocator, true per axis if saturated
    TorqueSetpoint update(const Vec3& rate_sp, const Vec3& rate,
                          float airspeed, float thrust_sp,
                          float dt, const Vec3& saturated = Vec3::zero()) {
        if (dt < 1e-6f) return {Vec3::zero(), thrust_sp};

        // --- Airspeed scaling ---
        float as_scale = 1.f;
        if (params_.airspeed_scaling_en) {
            float as_clamped = constrain(airspeed, params_.min_airspeed, params_.max_airspeed);
            as_scale = params_.trim_airspeed / std::max(as_clamped, 1.f);
        }

        // --- Rate error ---
        Vec3 rate_error = rate_sp - rate;

        // --- D-term with low-pass filter ---
        Vec3 rate_d;
        if (!initialized_) {
            rate_d = Vec3::zero();
            rate_prev_ = rate;
            rate_d_filt_ = Vec3::zero();
            initialized_ = true;
        } else {
            // Derivative of measured rate (derivative-on-measurement to avoid setpoint kick)
            rate_d = (rate - rate_prev_) / dt;
            rate_prev_ = rate;

            // Per-axis first-order low-pass on D-term
            float alpha_p = lp_alpha(params_.pitch.d_cutoff_freq, dt);
            float alpha_r = lp_alpha(params_.roll.d_cutoff_freq,  dt);
            float alpha_y = lp_alpha(params_.yaw.d_cutoff_freq,   dt);

            rate_d_filt_.x = rate_d_filt_.x + alpha_r * (rate_d.x - rate_d_filt_.x);
            rate_d_filt_.y = rate_d_filt_.y + alpha_p * (rate_d.y - rate_d_filt_.y);
            rate_d_filt_.z = rate_d_filt_.z + alpha_y * (rate_d.z - rate_d_filt_.z);
        }

        // --- Integrator update with anti-windup ---
        // Only integrate if not saturated in the same direction as the error
        Vec3 i_increment = rate_error * dt;
        if (saturated.x * rate_error.x <= 0.f) integral_.x += i_increment.x;
        if (saturated.y * rate_error.y <= 0.f) integral_.y += i_increment.y;
        if (saturated.z * rate_error.z <= 0.f) integral_.z += i_increment.z;

        // Clamp integrals
        integral_.x = constrain(integral_.x, -params_.roll.i_max,  params_.roll.i_max);
        integral_.y = constrain(integral_.y, -params_.pitch.i_max, params_.pitch.i_max);
        integral_.z = constrain(integral_.z, -params_.yaw.i_max,   params_.yaw.i_max);

        // --- PID + FF output ---
        Vec3 torque;

        // Roll
        torque.x = as_scale * (
            params_.roll.kp  * rate_error.x
          + params_.roll.ki  * integral_.x
          - params_.roll.kd  * rate_d_filt_.x   // negative: derivative-on-measurement
          + params_.roll.kff * rate_sp.x
        );

        // Pitch
        torque.y = as_scale * (
            params_.pitch.kp  * rate_error.y
          + params_.pitch.ki  * integral_.y
          - params_.pitch.kd  * rate_d_filt_.y
          + params_.pitch.kff * rate_sp.y
        );

        // Yaw
        torque.z = as_scale * (
            params_.yaw.kp  * rate_error.z
          + params_.yaw.ki  * integral_.z
          - params_.yaw.kd  * rate_d_filt_.z
          + params_.yaw.kff * rate_sp.z
        );

        return {torque, thrust_sp};
    }

    // Access integrator state (useful for logging/debugging)
    Vec3 integral() const { return integral_; }

private:
    RateControlParams params_;
    Vec3  integral_    = Vec3::zero();
    Vec3  rate_prev_   = Vec3::zero();
    Vec3  rate_d_filt_ = Vec3::zero();
    bool  initialized_ = false;

    // First-order low-pass filter coefficient
    static float lp_alpha(float cutoff_hz, float dt) {
        if (cutoff_hz <= 0.f || dt <= 0.f) return 1.f;
        float rc = 1.f / (2.f * 3.14159265f * cutoff_hz);
        return dt / (rc + dt);
    }
};

} // namespace px4_att_control
