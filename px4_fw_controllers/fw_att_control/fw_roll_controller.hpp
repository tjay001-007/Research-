/**
 * @file fw_roll_controller.hpp
 *
 * Fixed-wing roll attitude controller -- OUTER LOOP.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This controller sits between the path-following guidance (which outputs a
 * desired roll angle) and the body-rate controller (which outputs aileron
 * deflection). Its single job is:
 *
 *     Given:  desired roll angle (phi_sp) and current attitude
 *     Output: desired roll BODY rate (p_body) [rad/s]
 *
 * It is a simple proportional (P) controller followed by an Euler-to-body
 * Jacobian transformation.
 *
 * ============================================================================
 * CONTROL LAW DERIVATION
 * ============================================================================
 *
 * Step 1: Compute the desired Euler roll rate.
 * ------------------------------------------------
 *   We want to close the error at a rate governed by a time constant:
 *
 *       phi_dot_desired = (phi_sp - phi) / tau_roll
 *
 *   where tau_roll is the first-order time constant (FW_R_TC). A smaller
 *   time constant gives a faster response but may excite structural modes
 *   or cause actuator saturation. Typical values are 0.4 to 0.6 seconds.
 *
 *   This is equivalent to a P-gain of Kp = 1/tau_roll, which PX4 prefers
 *   to parameterize as a time constant because it is more intuitive for
 *   tuning: "How fast should the aircraft null a roll error?"
 *
 * Step 2: Transform Euler rates to body rates (Jacobian).
 * --------------------------------------------------------
 *   The relationship between Euler angle rates [phi_dot, theta_dot, psi_dot]
 *   and body angular rates [p, q, r] is given by the kinematic equations:
 *
 *       p = phi_dot - sin(theta) * psi_dot
 *       q = cos(phi) * theta_dot + cos(theta) * sin(phi) * psi_dot
 *       r = -sin(phi) * theta_dot + cos(phi) * cos(theta) * psi_dot
 *
 *   These equations come from decomposing the angular velocity vector into
 *   successive frame rotations (ZYX Euler angle convention).
 *
 *   For the ROLL axis, we need the first equation:
 *
 *       p = phi_dot - sin(theta) * psi_dot
 *
 *   The -sin(theta)*psi_dot term is a kinematic coupling: when the aircraft
 *   has a non-zero pitch angle and is yawing, part of that yaw motion
 *   projects onto the body x-axis and appears as a roll rate. We must
 *   subtract it to get the correct body-frame roll rate command.
 *
 *   For the yaw Euler rate (psi_dot), we use the coordinated-turn yaw rate
 *   that the yaw controller will independently compute. This ensures
 *   consistency across the three axes.
 *
 * Step 3: Rate limiting.
 * ----------------------
 *   The output body rate is clamped to [-max_rate, +max_rate] to prevent
 *   commanding rates that the airframe cannot physically achieve. The
 *   max_rate parameter (FW_R_RMAX) is determined by flight testing.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_att_control/ecl_roll_controller.cpp
 *   - Stevens & Lewis, "Aircraft Control and Simulation", Chapter 1.3
 *   - Beard & McLain, "Small Unmanned Aircraft", Appendix B
 */

#pragma once

namespace fw_att_control {

class RollController {
public:
    RollController() = default;
    ~RollController() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the roll time constant (FW_R_TC).
     *
     * This is the first-order time constant for roll attitude tracking.
     * The effective P-gain is 1/tc. A value of 0.4s means the controller
     * will attempt to null 63% of the roll error in 0.4 seconds.
     *
     * @param tc  Time constant [s], must be > 0
     */
    void set_time_constant(float tc);

    /**
     * Set the maximum commanded roll body rate (FW_R_RMAX).
     *
     * @param max_rate  Maximum rate [rad/s], must be >= 0
     */
    void set_max_rate(float max_rate);

    /* ---- Runtime interface ---- */

    /**
     * Compute the desired roll body rate.
     *
     * This is the main function called once per attitude control cycle (~250Hz).
     *
     * @param roll          Current roll angle (phi) [rad]
     * @param roll_sp       Desired roll angle [rad]
     * @param pitch         Current pitch angle (theta) [rad]  -- needed for Jacobian
     * @param yaw_rate_euler  The Euler yaw rate (psi_dot) [rad/s] from coordinated turn
     * @return              Desired roll body rate (p) [rad/s]
     */
    float control_attitude(float roll, float roll_sp, float pitch, float yaw_rate_euler);

    /* ---- Accessors (for logging / telemetry) ---- */

    float get_euler_rate_setpoint() const { return _euler_rate_setpoint; }

private:
    /* ---- Parameters ---- */

    /** First-order time constant for roll error closure [s] */
    float _tc = 0.4f;

    /** Maximum commanded body roll rate [rad/s] */
    float _max_rate = 1.05f;  // ~60 deg/s default

    /* ---- Internal state (for logging) ---- */

    /** Last computed Euler rate setpoint (before Jacobian) */
    float _euler_rate_setpoint = 0.0f;
};

} // namespace fw_att_control
