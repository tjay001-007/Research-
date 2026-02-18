/**
 * @file fw_pitch_controller.hpp
 *
 * Fixed-wing pitch attitude controller -- OUTER LOOP.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This controller sits between the path-following guidance (which outputs a
 * desired pitch angle via TECS) and the body-rate controller (which outputs
 * elevator deflection). Its single job is:
 *
 *     Given:  desired pitch angle (theta_sp) and current attitude
 *     Output: desired pitch BODY rate (q_body) [rad/s]
 *
 * It is a simple proportional (P) controller followed by an Euler-to-body
 * Jacobian transformation, with ASYMMETRIC rate limiting.
 *
 * ============================================================================
 * CONTROL LAW DERIVATION
 * ============================================================================
 *
 * Step 1: Compute the desired Euler pitch rate.
 * ------------------------------------------------
 *   We want to close the error at a rate governed by a time constant:
 *
 *       theta_dot_desired = (theta_sp - theta) / tau_pitch
 *
 *   where tau_pitch is the first-order time constant (FW_P_TC). A smaller
 *   time constant gives a faster response but may excite the phugoid or
 *   short-period modes, or cause elevator saturation. Typical values are
 *   0.3 to 0.5 seconds.
 *
 *   This is equivalent to a P-gain of Kp = 1/tau_pitch, which PX4 prefers
 *   to parameterize as a time constant because it is more intuitive for
 *   tuning: "How fast should the aircraft null a pitch error?"
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
 *   For the PITCH axis, we need the second equation:
 *
 *       q = cos(phi) * theta_dot + cos(theta) * sin(phi) * psi_dot
 *
 *   Physical interpretation of the two terms:
 *
 *     cos(phi) * theta_dot:
 *       When the aircraft is banked, only a fraction (cos(phi)) of the
 *       Euler pitch rate projects onto the body y-axis. At 60 degrees of
 *       bank, only half of the commanded Euler pitch rate appears as a
 *       body pitch rate -- the other half appears as a body yaw rate.
 *
 *     cos(theta) * sin(phi) * psi_dot:
 *       When the aircraft is banked and yawing (coordinated turn), part
 *       of the yaw rotation projects onto the body y-axis. In a banked
 *       turn, what we perceive as "yaw" in the Euler sense partially
 *       becomes body pitch. This coupling is zero at wings-level, and
 *       increases with bank angle.
 *
 *   The yaw Euler rate (psi_dot) is provided by the yaw controller's
 *   coordinated turn calculation, ensuring consistency across axes.
 *
 * Step 3: Asymmetric rate limiting.
 * -----------------------------------
 *   The output body rate is clamped to [-max_rate_neg, +max_rate_pos].
 *   Unlike roll, the pitch axis has ASYMMETRIC limits because:
 *
 *     - Nose-up authority: The elevator can produce a certain maximum
 *       nose-up pitch rate. This is often limited by stall margin --
 *       commanding too much nose-up pitch rate at low speed can stall
 *       the aircraft.
 *
 *     - Nose-down authority: At high speed, the elevator can produce
 *       very high nose-down pitch rates, which can lead to excessive
 *       negative g-loads or structural damage.
 *
 *   Separate positive and negative rate limits (FW_P_RMAX_POS and
 *   FW_P_RMAX_NEG) allow independent tuning for these two regimes.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_att_control/ecl_pitch_controller.cpp
 *   - Stevens & Lewis, "Aircraft Control and Simulation", Chapter 1.3
 *   - Beard & McLain, "Small Unmanned Aircraft", Appendix B
 */

#pragma once

namespace fw_att_control {

class PitchController {
public:
    PitchController() = default;
    ~PitchController() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the pitch time constant (FW_P_TC).
     *
     * This is the first-order time constant for pitch attitude tracking.
     * The effective P-gain is 1/tc. A value of 0.4s means the controller
     * will attempt to null 63% of the pitch error in 0.4 seconds.
     *
     * @param tc  Time constant [s], must be > 0
     */
    void set_time_constant(float tc);

    /**
     * Set the maximum commanded pitch-UP body rate (FW_P_RMAX_POS).
     *
     * This limits how fast the controller can command the nose upward.
     * Nose-up rate is typically limited to prevent stall at low speeds.
     *
     * @param max_rate_pos  Maximum positive (nose-up) rate [rad/s], must be >= 0
     */
    void set_max_rate_pos(float max_rate_pos);

    /**
     * Set the maximum commanded pitch-DOWN body rate (FW_P_RMAX_NEG).
     *
     * This limits how fast the controller can command the nose downward.
     * Nose-down rate is typically limited to prevent excessive negative g.
     *
     * @param max_rate_neg  Maximum negative (nose-down) rate [rad/s], must be >= 0
     *                      (stored as a positive value; applied as the lower clamp)
     */
    void set_max_rate_neg(float max_rate_neg);

    /* ---- Runtime interface ---- */

    /**
     * Compute the desired pitch body rate.
     *
     * This is the main function called once per attitude control cycle (~250Hz).
     *
     * @param pitch          Current pitch angle (theta) [rad]
     * @param pitch_sp       Desired pitch angle [rad]
     * @param roll           Current roll angle (phi) [rad]  -- needed for Jacobian
     * @param yaw_rate_euler The Euler yaw rate (psi_dot) [rad/s] from coordinated turn
     * @return               Desired pitch body rate (q) [rad/s]
     */
    float control_attitude(float pitch, float pitch_sp, float roll, float yaw_rate_euler);

    /* ---- Accessors (for logging / telemetry) ---- */

    float get_euler_rate_setpoint() const { return _euler_rate_setpoint; }

private:
    /* ---- Parameters ---- */

    /** First-order time constant for pitch error closure [s] */
    float _tc = 0.4f;

    /** Maximum commanded body pitch rate in positive (nose-up) direction [rad/s] */
    float _max_rate_pos = 1.05f;  // ~60 deg/s default

    /** Maximum commanded body pitch rate in negative (nose-down) direction [rad/s] */
    float _max_rate_neg = 1.05f;  // ~60 deg/s default

    /* ---- Internal state (for logging) ---- */

    /** Last computed Euler rate setpoint (before Jacobian) */
    float _euler_rate_setpoint = 0.0f;
};

} // namespace fw_att_control
