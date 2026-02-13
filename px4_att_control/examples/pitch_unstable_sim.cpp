// PX4 Attitude Control Library - Pitch-Unstable Aircraft Simulation
//
// Demonstrates the full attitude control pipeline on a simplified
// pitch-unstable aircraft model. This matches the research project's
// focus: testing PX4 control algorithms on aircraft with Cmalpha > 0.
//
// The simulation:
//   1. Initializes a pitch-unstable aircraft in trimmed flight
//   2. Introduces a 5-degree pitch-up disturbance
//   3. Runs the PX4 attitude controller to stabilize
//   4. Outputs time-series data (CSV) for analysis
//
// Build:
//   mkdir build && cd build && cmake .. && make
//   ./pitch_unstable_sim > output.csv

#include <cstdio>
#include <cmath>
#include "px4_att_control/px4_att_control.hpp"

using namespace px4_att_control;

// Simplified 3-axis rotational dynamics for a pitch-unstable aircraft
// State: angular rates (p, q, r) and attitude quaternion
struct AircraftDynamics {
    // Inertia tensor (diagonal, from project parameters)
    float Ixx = 0.05f;   // kg*m^2
    float Iyy = 0.10f;
    float Izz = 0.12f;

    // Aerodynamic stability derivatives (body-axis moments per rad)
    float Clp    = -0.5f;   // Roll damping
    float Cmalpha =  0.3f;  // Pitch stiffness (POSITIVE = unstable)
    float Cmq    = -3.0f;   // Pitch rate damping
    float Cnr    = -0.1f;   // Yaw damping

    // Control effectiveness (moment per unit deflection)
    float Clda = 0.3f;    // Roll from aileron
    float Cmde = 0.5f;    // Pitch from elevator
    float Cndr = -0.1f;   // Yaw from rudder

    float qbar = 0.5f * 1.225f * 20.f * 20.f;  // Dynamic pressure at 20 m/s
    float S    = 0.3f;   // Wing area
    float c    = 0.2f;   // Mean chord
    float b    = 1.5f;   // Wingspan

    Quatf attitude = Quatf{};
    Vec3  omega    = Vec3::zero();  // Body rates (rad/s)

    // Compute angular accelerations from current state and control deflections
    Vec3 compute_moments(float aileron, float elevator, float rudder) const {
        float qS = qbar * S;

        // Aerodynamic moments (stability + damping + control)
        Vec3 euler = attitude.to_euler();
        float alpha = euler.y;  // Approximate: pitch angle ≈ angle of attack in small perturbations

        float L_aero = qS * b * (Clp * omega.x * b / (2.f * 20.f) + Clda * aileron);
        float M_aero = qS * c * (Cmalpha * alpha + Cmq * omega.y * c / (2.f * 20.f) + Cmde * elevator);
        float N_aero = qS * b * (Cnr * omega.z * b / (2.f * 20.f) + Cndr * rudder);

        // Angular acceleration = moment / inertia (simplified, ignoring cross-coupling)
        return {L_aero / Ixx, M_aero / Iyy, N_aero / Izz};
    }

    // Integrate one timestep (RK4)
    void step(float aileron, float elevator, float rudder, float dt) {
        // RK4 for angular rates
        Vec3 k1 = compute_moments(aileron, elevator, rudder);

        Vec3 omega_tmp = omega + k1 * (dt * 0.5f);
        // Recompute at midpoint (simplified: same control inputs)
        Vec3 k2 = compute_moments(aileron, elevator, rudder);

        omega_tmp = omega + k2 * (dt * 0.5f);
        Vec3 k3 = compute_moments(aileron, elevator, rudder);

        omega_tmp = omega + k3 * dt;
        Vec3 k4 = compute_moments(aileron, elevator, rudder);

        omega += (k1 + k2 * 2.f + k3 * 2.f + k4) * (dt / 6.f);

        // Quaternion integration (first-order)
        Quatf omega_quat{0.f, omega.x, omega.y, omega.z};
        Quatf q_dot = attitude * omega_quat * 0.5f;
        attitude.w += q_dot.w * dt;
        attitude.x += q_dot.x * dt;
        attitude.y += q_dot.y * dt;
        attitude.z += q_dot.z * dt;
        attitude = attitude.normalized();
    }
};

int main() {
    // --- Setup controller pipeline ---
    AttitudeControlPipeline pipeline;
    pipeline.configure_pitch_unstable();

    // --- Setup aircraft dynamics ---
    AircraftDynamics aircraft;

    // Initial condition: trimmed level flight, then apply disturbance
    aircraft.attitude = Quatf::from_euler(0.f, 5.f * DEG_TO_RAD, 0.f); // 5 deg pitch-up
    aircraft.omega = Vec3::zero();

    // Setpoint: level flight (identity quaternion)
    AttitudeSetpoint setpoint;
    setpoint.q_sp = Quatf{};  // Wings-level, zero pitch
    setpoint.thrust_sp = 0.5f;

    // --- Simulation parameters ---
    const float dt = 0.004f;          // 250 Hz (matches PX4)
    const float sim_time = 5.0f;      // 5 seconds
    const int steps = static_cast<int>(sim_time / dt);

    // CSV header
    std::printf("time,roll_deg,pitch_deg,yaw_deg,p_dps,q_dps,r_dps,"
                "aileron,elevator,rudder,throttle\n");

    for (int i = 0; i < steps; i++) {
        float t = i * dt;

        // Current vehicle state
        VehicleState state;
        state.attitude = aircraft.attitude;
        state.body_rates = aircraft.omega;
        state.airspeed = 20.f;  // Constant airspeed assumption

        // Run controller
        ControlOutput ctrl = pipeline.update(state, setpoint, dt);

        // Apply to plant
        aircraft.step(ctrl.aileron, ctrl.elevator, ctrl.rudder, dt);

        // Output every 10 steps (25 Hz output rate)
        if (i % 10 == 0) {
            Vec3 euler = aircraft.attitude.to_euler();
            std::printf("%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.4f,%.4f,%.4f\n",
                        t,
                        euler.x * RAD_TO_DEG,
                        euler.y * RAD_TO_DEG,
                        euler.z * RAD_TO_DEG,
                        aircraft.omega.x * RAD_TO_DEG,
                        aircraft.omega.y * RAD_TO_DEG,
                        aircraft.omega.z * RAD_TO_DEG,
                        ctrl.aileron,
                        ctrl.elevator,
                        ctrl.rudder,
                        ctrl.throttle);
        }
    }

    return 0;
}
