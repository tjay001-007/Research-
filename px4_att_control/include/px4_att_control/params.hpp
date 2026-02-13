// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Parameters
// Mirrors PX4 FW attitude/rate controller parameters

#pragma once

#include "math_types.hpp"

namespace px4_att_control {

// ---------------------------------------------------------------------------
// AttitudeControlParams: attitude (outer) loop parameters
// Maps to PX4 FW_P_TC, FW_R_TC, etc.
// ---------------------------------------------------------------------------
struct AttitudeControlParams {
    // Attitude time constants (seconds). Lower = more aggressive.
    // The attitude controller produces rate setpoints proportional to
    // attitude error divided by these time constants.
    float pitch_time_constant   = 0.4f;   // FW_P_TC
    float roll_time_constant    = 0.4f;   // FW_R_TC

    // Maximum body rates the attitude loop will command (rad/s)
    float pitch_rate_max        = 60.f * DEG_TO_RAD;  // FW_P_RMAX_POS
    float pitch_rate_min        = -60.f * DEG_TO_RAD;  // FW_P_RMAX_NEG (negative)
    float roll_rate_max         = 70.f * DEG_TO_RAD;  // FW_R_RMAX
    float yaw_rate_max          = 50.f * DEG_TO_RAD;  // FW_Y_RMAX

    // Coordinated turn: yaw rate = g * tan(roll) / V
    bool  coordinated_turn      = true;

    // Feed-forward: pass through rate setpoints from guidance
    float pitch_ff              = 0.5f;
    float roll_ff               = 0.5f;
    float yaw_ff                = 0.3f;

    // Pitch-unstable aircraft: use higher gains (per project recommendations)
    static AttitudeControlParams pitch_unstable_defaults() {
        AttitudeControlParams p;
        p.pitch_time_constant = 0.2f;   // Faster response for unstable pitch
        p.pitch_rate_max      = 120.f * DEG_TO_RAD;
        p.pitch_rate_min      = -120.f * DEG_TO_RAD;
        p.roll_rate_max       = 90.f * DEG_TO_RAD;
        return p;
    }
};

// ---------------------------------------------------------------------------
// RateControlParams: rate (inner) loop PID parameters per axis
// ---------------------------------------------------------------------------
struct AxisRatePID {
    float kp  = 0.0f;   // Proportional gain
    float ki  = 0.0f;   // Integral gain
    float kd  = 0.0f;   // Derivative gain
    float kff = 0.0f;   // Feed-forward gain

    float i_max = 0.4f; // Integrator saturation (absolute)
    float d_cutoff_freq = 30.f; // D-term low-pass filter cutoff (Hz)
};

struct RateControlParams {
    AxisRatePID pitch;
    AxisRatePID roll;
    AxisRatePID yaw;

    // Airspeed scaling reference (m/s). Rate gains are scaled by
    // (trim_airspeed / indicated_airspeed) to maintain consistent
    // handling across the speed envelope.
    float trim_airspeed     = 15.f;   // FW_AIRSPD_TRIM
    float min_airspeed      = 10.f;   // FW_AIRSPD_MIN
    float max_airspeed      = 25.f;   // FW_AIRSPD_MAX

    // Enable airspeed scaling
    bool airspeed_scaling_en = true;

    // PX4 default gains (typical fixed-wing)
    static RateControlParams defaults() {
        RateControlParams p;
        // Pitch
        p.pitch.kp  = 0.08f;   // FW_PR_P
        p.pitch.ki  = 0.04f;   // FW_PR_I
        p.pitch.kd  = 0.0f;    // FW_PR_D
        p.pitch.kff = 0.5f;    // FW_PR_FF
        p.pitch.i_max = 0.4f;
        // Roll
        p.roll.kp   = 0.05f;   // FW_RR_P
        p.roll.ki   = 0.02f;   // FW_RR_I
        p.roll.kd   = 0.0f;    // FW_RR_D
        p.roll.kff  = 0.5f;    // FW_RR_FF
        p.roll.i_max = 0.4f;
        // Yaw
        p.yaw.kp    = 0.05f;   // FW_YR_P
        p.yaw.ki    = 0.02f;   // FW_YR_I
        p.yaw.kd    = 0.0f;    // FW_YR_D
        p.yaw.kff   = 0.3f;    // FW_YR_FF
        p.yaw.i_max = 0.2f;
        return p;
    }

    // Gains tuned for pitch-unstable aircraft (per project recommendations)
    static RateControlParams pitch_unstable_defaults() {
        RateControlParams p = defaults();
        p.pitch.kp  = 0.40f;   // ~5x normal (compensate for positive Cmalpha)
        p.pitch.ki  = 0.30f;   // ~7.5x normal
        p.pitch.kd  = 0.015f;  // Non-zero D for damping
        p.pitch.kff = 0.5f;
        p.pitch.d_cutoff_freq = 40.f;
        return p;
    }
};

// ---------------------------------------------------------------------------
// ControlAllocatorParams: control surface effectiveness and limits
// ---------------------------------------------------------------------------
struct ControlAllocatorParams {
    // Effectiveness matrix: maps [roll_torque, pitch_torque, yaw_torque]
    // to control surface deflections [aileron, elevator, rudder].
    // Each row is the torque produced per unit deflection of that surface.
    // The allocator inverts this to get deflections from desired torques.
    Mat33 effectiveness = Mat33::identity();

    // Control surface deflection limits (normalized [-1, 1])
    Vec3 deflection_min = {-1.f, -1.f, -1.f};
    Vec3 deflection_max = { 1.f,  1.f,  1.f};

    // Flap/throttle are not part of the 3-axis allocation
    float throttle_min = 0.f;
    float throttle_max = 1.f;

    // Standard fixed-wing effectiveness (aileron, elevator, rudder)
    static ControlAllocatorParams fw_defaults() {
        ControlAllocatorParams p;
        // Row 0: aileron produces roll torque
        // Row 1: elevator produces pitch torque
        // Row 2: rudder produces yaw torque
        p.effectiveness(0,0) = 1.0f; p.effectiveness(0,1) = 0.0f; p.effectiveness(0,2) = 0.0f;
        p.effectiveness(1,0) = 0.0f; p.effectiveness(1,1) = 1.0f; p.effectiveness(1,2) = 0.0f;
        p.effectiveness(2,0) = 0.0f; p.effectiveness(2,1) = 0.0f; p.effectiveness(2,2) = 1.0f;
        return p;
    }

    // Flying wing (elevons): aileron and elevator are mixed
    static ControlAllocatorParams flying_wing_defaults() {
        ControlAllocatorParams p;
        // elevon_left  = 0.5*(roll + pitch)
        // elevon_right = 0.5*(-roll + pitch)
        // rudder = yaw (if present)
        p.effectiveness(0,0) = 0.5f;  p.effectiveness(0,1) = 0.5f;  p.effectiveness(0,2) = 0.0f;
        p.effectiveness(1,0) = -0.5f; p.effectiveness(1,1) = 0.5f;  p.effectiveness(1,2) = 0.0f;
        p.effectiveness(2,0) = 0.0f;  p.effectiveness(2,1) = 0.0f;  p.effectiveness(2,2) = 1.0f;
        return p;
    }
};

} // namespace px4_att_control
