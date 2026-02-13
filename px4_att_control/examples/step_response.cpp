// PX4 Attitude Control Library - Step Response Test
//
// Tests the attitude controller's response to step inputs on each axis.
// Useful for tuning gain parameters and verifying stability margins.
//
// Outputs CSV data for plotting rise time, overshoot, and settling time.
//
// Build:
//   mkdir build && cd build && cmake .. && make
//   ./step_response > step_response.csv

#include <cstdio>
#include <cmath>
#include "px4_att_control/px4_att_control.hpp"

using namespace px4_att_control;

// Simplified single-axis rotational plant: I * omega_dot = torque * effectiveness
struct SingleAxisPlant {
    float inertia;
    float effectiveness;
    float damping;         // Aerodynamic damping (negative = stabilizing)
    float angle    = 0.f;
    float rate     = 0.f;

    void step(float control_input, float dt) {
        float torque = effectiveness * control_input + damping * rate;
        float accel = torque / inertia;
        rate  += accel * dt;
        angle += rate * dt;
    }
};

void run_step_test(const char* label,
                   SingleAxisPlant& plant,
                   const RateControlParams& rate_params,
                   float step_angle_deg,
                   float sim_time,
                   float dt) {
    RateController rate_ctrl;
    rate_ctrl.set_params(rate_params);

    AttitudeControlParams att_params;
    att_params.pitch_time_constant = 0.2f;
    att_params.roll_time_constant  = 0.4f;

    float step_angle = step_angle_deg * DEG_TO_RAD;
    float airspeed = 20.f;
    int steps = static_cast<int>(sim_time / dt);

    std::printf("# %s step response: %.1f deg\n", label, step_angle_deg);
    std::printf("time,%s_angle_deg,%s_rate_dps,%s_control\n", label, label, label);

    for (int i = 0; i < steps; i++) {
        float t = i * dt;

        // Attitude error -> rate setpoint (simplified single-axis)
        float error = step_angle - plant.angle;
        float rate_sp = error / att_params.pitch_time_constant;

        // Rate limit
        rate_sp = constrain(rate_sp, -120.f * DEG_TO_RAD, 120.f * DEG_TO_RAD);

        // Rate controller
        Vec3 rate_sp_vec = Vec3::zero();
        Vec3 rate_vec    = Vec3::zero();
        rate_sp_vec.y = rate_sp;     // pitch axis
        rate_vec.y    = plant.rate;

        TorqueSetpoint torque = rate_ctrl.update(rate_sp_vec, rate_vec,
                                                  airspeed, 0.5f, dt);

        // Apply to plant
        plant.step(torque.torque.y, dt);

        // Output at 50 Hz
        if (i % 5 == 0) {
            std::printf("%.4f,%.3f,%.3f,%.5f\n",
                        t,
                        plant.angle * RAD_TO_DEG,
                        plant.rate * RAD_TO_DEG,
                        torque.torque.y);
        }
    }
}

int main() {
    float dt = 0.004f;

    // --- Test 1: Standard aircraft pitch step ---
    {
        SingleAxisPlant pitch_stable;
        pitch_stable.inertia       = 0.10f;
        pitch_stable.effectiveness = 0.5f;
        pitch_stable.damping       = -0.3f;  // Stable (Cmalpha < 0)

        RateControlParams params = RateControlParams::defaults();
        run_step_test("pitch_stable", pitch_stable, params, 10.f, 3.f, dt);
    }

    std::printf("\n\n");

    // --- Test 2: Pitch-unstable aircraft pitch step ---
    {
        SingleAxisPlant pitch_unstable;
        pitch_unstable.inertia       = 0.10f;
        pitch_unstable.effectiveness = 0.5f;
        pitch_unstable.damping       = 0.3f;   // UNSTABLE (Cmalpha > 0)

        RateControlParams params = RateControlParams::pitch_unstable_defaults();
        run_step_test("pitch_unstable", pitch_unstable, params, 10.f, 3.f, dt);
    }

    std::printf("\n\n");

    // --- Test 3: Roll step (standard gains) ---
    {
        SingleAxisPlant roll;
        roll.inertia       = 0.05f;
        roll.effectiveness = 0.3f;
        roll.damping       = -0.5f;

        RateControlParams params = RateControlParams::defaults();
        run_step_test("roll", roll, params, 30.f, 3.f, dt);
    }

    return 0;
}
