/**
 * @file fw_wheel_controller.hpp
 *
 * Fixed-wing wheel (ground steering) controller.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This controller handles yaw control during ground operations -- taxiing,
 * takeoff roll, and landing rollout. It commands the nose wheel (or
 * differential braking) to track a desired heading.
 *
 * Unlike the airborne yaw controller (which computes a coordinated turn
 * rate), this controller actively tracks a yaw setpoint because on the
 * ground, yaw IS a directly controlled state via the steering mechanism.
 *
 *     Given:  desired yaw angle (psi_sp), current yaw, and body yaw rate
 *     Output: wheel steering command (normalized torque) [-1, +1]
 *
 * It uses a cascaded control structure:
 *   - Outer loop: P-controller on yaw error -> desired yaw body rate
 *   - Inner loop: PI + feedforward on yaw rate error -> steering command
 *
 * ============================================================================
 * CONTROL LAW DERIVATION
 * ============================================================================
 *
 * Step 1 (Outer loop): Compute desired body yaw rate from yaw error.
 * --------------------------------------------------------------------
 *   On the ground, we can directly control yaw through steering, so
 *   we track a heading setpoint:
 *
 *       yaw_error = wrap_pi(psi_sp - psi)
 *
 *   The wrapping is essential for yaw because yaw angles span the full
 *   circle [-pi, pi]. Without wrapping, crossing the +/- 180 degree
 *   boundary would cause a full-circle command.
 *
 *   The P-controller with time constant:
 *
 *       rate_setpoint = yaw_error / tau
 *
 *   where tau is the first-order time constant (FW_WR_TC). On the ground,
 *   the time constant is typically larger (slower response) than in the
 *   air because:
 *     - Steering dynamics are slower than aerodynamic surfaces
 *     - Aggressive steering at high groundspeed can cause loss of control
 *     - Tire friction limits the achievable yaw rate
 *
 * Step 2: Rate limiting.
 * -----------------------
 *   The rate setpoint is clamped to [-max_rate, +max_rate] to prevent:
 *     - Tire scrubbing at high steering rates
 *     - Loss of lateral traction
 *     - Mechanical steering system limits
 *
 * Step 3 (Inner loop): PI + Feedforward on rate error.
 * ------------------------------------------------------
 *   The steering command is computed as:
 *
 *       output = rate_sp * k_ff * scaler
 *              + scaler^2 * rate_error * k_p
 *              + integrator
 *
 *   Where:
 *     scaler = groundspeed-based scaling factor
 *     rate_error = rate_sp - current_yaw_rate
 *
 *   Feedforward (k_ff * rate_sp * scaler):
 *     Provides an immediate steering command proportional to the desired
 *     yaw rate. This gives responsive tracking without waiting for the
 *     error to build up. The feedforward gain is typically the dominant
 *     term during transient maneuvers.
 *
 *   Proportional (k_p * rate_error * scaler^2):
 *     Corrects for deviations between commanded and actual yaw rate.
 *     The squared scaler amplifies the correction at low groundspeeds,
 *     where more steering authority is available.
 *
 *   Integral (k_i * integral(rate_error)):
 *     Eliminates steady-state rate errors caused by:
 *       - Crosswind on the ground
 *       - Asymmetric friction (e.g., one flat tire)
 *       - Steering mechanism bias
 *
 * Step 4: Groundspeed scaling.
 * -----------------------------
 *   At higher groundspeeds, the tire generates more lateral force for
 *   a given steering angle, so less steering input is needed. The
 *   groundspeed scaler adjusts the controller gains to maintain
 *   consistent response across the speed range.
 *
 *   This is analogous to airspeed scaling for aerodynamic surfaces.
 *
 * Step 5: Integrator management.
 * --------------------------------
 *   The integrator is only active above a minimum groundspeed (1 m/s)
 *   to prevent windup while the aircraft is stationary. When stationary:
 *     - The steering has no effect on yaw rate (static friction)
 *     - The integrator would grow without bound, causing a large
 *       steering jump when the aircraft starts moving
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_att_control/ecl_wheel_controller.cpp
 *   - Rajamani, "Vehicle Dynamics and Control", Chapter 2
 *   - Pacejka, "Tire and Vehicle Dynamics", Chapter 1
 */

#pragma once

namespace fw_att_control {

class WheelController {
public:
    WheelController() = default;
    ~WheelController() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the yaw time constant for ground steering (FW_WR_TC).
     *
     * This is the first-order time constant for yaw heading tracking
     * on the ground. Typical values are 0.5 to 2.0 seconds.
     *
     * @param tc  Time constant [s], must be > 0
     */
    void set_time_constant(float tc);

    /**
     * Set the feedforward gain (FW_WR_FF).
     *
     * The feedforward gain provides immediate steering response
     * proportional to the desired yaw rate. Higher values give faster
     * response but may cause oscillation on slippery surfaces.
     *
     * @param k_ff  Feedforward gain, dimensionless
     */
    void set_k_ff(float k_ff);

    /**
     * Set the proportional gain on rate error (FW_WR_P).
     *
     * @param k_p  Proportional gain [1/s]
     */
    void set_k_p(float k_p);

    /**
     * Set the integral gain on rate error (FW_WR_I).
     *
     * The integrator compensates for steady-state disturbances
     * (crosswind, steering bias).
     *
     * @param k_i  Integral gain [1/s^2]
     */
    void set_k_i(float k_i);

    /**
     * Set the maximum commanded yaw body rate on the ground (FW_WR_RMAX).
     *
     * @param max_rate  Maximum rate [rad/s], must be >= 0
     */
    void set_max_rate(float max_rate);

    /**
     * Set the maximum integrator contribution (FW_WR_IMAX).
     *
     * Prevents integrator windup by clamping the accumulated integral
     * to [-integrator_max, +integrator_max].
     *
     * @param integrator_max  Maximum integrator magnitude (normalized), must be >= 0
     */
    void set_integrator_max(float integrator_max);

    /* ---- Runtime interface ---- */

    /**
     * Compute the wheel steering command.
     *
     * This is the main function called once per attitude control cycle (~250Hz).
     *
     * @param yaw              Current yaw angle (psi) [rad]
     * @param yaw_sp           Desired yaw angle [rad]
     * @param yaw_rate         Current body yaw rate [rad/s] (from gyroscope)
     * @param groundspeed      Current groundspeed [m/s]
     * @param groundspeed_scaler  Groundspeed-based gain scaler (typically V_trim / V_ground)
     * @param dt               Time step [s]
     * @return                 Steering command, normalized [-1, +1]
     */
    float control_attitude(float yaw, float yaw_sp, float yaw_rate,
                           float groundspeed, float groundspeed_scaler, float dt);

    /**
     * Reset the integrator.
     *
     * Should be called when transitioning from airborne to ground mode,
     * or when the controller is re-engaged after being disabled.
     */
    void reset_integrator();

    /* ---- Accessors (for logging / telemetry) ---- */

    float get_rate_setpoint() const { return _rate_setpoint; }
    float get_integrator() const { return _integrator; }

private:
    /* ---- Parameters ---- */

    /** First-order time constant for yaw heading tracking on ground [s] */
    float _tc = 1.0f;

    /** Feedforward gain on rate setpoint (dimensionless) */
    float _k_ff = 0.3f;

    /** Proportional gain on rate error [1/s] */
    float _k_p = 1.0f;

    /** Integral gain on rate error [1/s^2] */
    float _k_i = 0.1f;

    /** Maximum commanded yaw body rate on ground [rad/s] */
    float _max_rate = 0.5236f;  // ~30 deg/s default

    /** Maximum integrator contribution (normalized) */
    float _integrator_max = 0.3f;

    /* ---- Internal state ---- */

    /** Rate setpoint from outer loop [rad/s] */
    float _rate_setpoint = 0.0f;

    /** Accumulated integral of rate error (normalized) */
    float _integrator = 0.0f;
};

} // namespace fw_att_control
