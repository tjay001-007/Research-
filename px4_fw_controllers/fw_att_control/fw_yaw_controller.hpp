/**
 * @file fw_yaw_controller.hpp
 *
 * Fixed-wing yaw attitude controller -- OUTER LOOP.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This controller sits between the path-following guidance and the body-rate
 * controller (which outputs rudder deflection). Its job is fundamentally
 * DIFFERENT from the roll and pitch controllers:
 *
 *     It does NOT track a yaw setpoint.
 *
 * Instead, it computes the yaw body rate required for a COORDINATED TURN --
 * that is, a turn with zero sideslip (the ball centered).
 *
 *     Given:  current attitude and airspeed
 *     Output: desired yaw BODY rate (r_body) [rad/s] for coordinated flight
 *
 * The reason we do not track a yaw angle is that for fixed-wing aircraft,
 * yaw is not a directly controlled state in the same way as roll and pitch.
 * The aircraft turns by banking (rolling), and the rudder's primary role
 * is to coordinate the turn (eliminate adverse yaw and sideslip), not to
 * command a yaw angle.
 *
 * ============================================================================
 * CONTROL LAW DERIVATION
 * ============================================================================
 *
 * Step 1: Coordinated turn yaw rate (Euler frame).
 * --------------------------------------------------
 *   In a steady, coordinated, level turn, the centripetal acceleration
 *   is provided by the horizontal component of lift. The balance of
 *   forces yields the coordinated turn yaw rate:
 *
 *       psi_dot = tan(phi) * cos(theta) * g / V
 *
 *   Where:
 *     phi   = roll (bank) angle
 *     theta = pitch angle
 *     g     = gravitational acceleration (9.81 m/s^2)
 *     V     = true airspeed [m/s]
 *
 *   Derivation:
 *     In a banked turn at constant altitude:
 *       - Lift (L) balances weight vertically: L * cos(phi) = m * g
 *       - Lift provides centripetal force horizontally: L * sin(phi) = m * V * psi_dot
 *       - Dividing: tan(phi) = V * psi_dot / g
 *       - Rearranging: psi_dot = g * tan(phi) / V
 *       - Including the pitch correction: psi_dot = g * tan(phi) * cos(theta) / V
 *
 *   The cos(theta) factor accounts for the fact that when pitched up,
 *   less of the lift vector is available for turning. In level flight
 *   (theta = 0), cos(theta) = 1 and the factor disappears.
 *
 * Step 2: Singularity protection for tan(phi).
 * -----------------------------------------------
 *   The tangent function has a singularity at phi = +/- 90 degrees
 *   (vertical bank). In inverted flight (|phi| > 90 degrees), the
 *   sign of tan(phi) reverses, which would command the wrong yaw
 *   direction. To handle this safely:
 *
 *     - The roll angle input to the coordinated turn formula is clamped
 *       to a safe range. PX4 uses a maximum of approximately
 *       +/- 80 degrees (~1.396 rad) to prevent the tangent from
 *       blowing up.
 *
 *     - For inverted flight, the yaw rate is set to zero (no
 *       coordinated turn computation) since the formula's assumptions
 *       no longer hold.
 *
 * Step 3: Transform Euler rates to body rates (Jacobian).
 * --------------------------------------------------------
 *   The kinematic equation for yaw body rate is:
 *
 *       r = -sin(phi) * theta_dot + cos(phi) * cos(theta) * psi_dot
 *
 *   Where:
 *     r         = yaw body rate (what we command to the rate controller)
 *     phi       = current roll angle
 *     theta_dot = Euler pitch rate (from the pitch controller)
 *     theta     = current pitch angle
 *     psi_dot   = Euler yaw rate (what we just computed)
 *
 *   Physical interpretation:
 *
 *     -sin(phi) * theta_dot:
 *       When the aircraft is banked and pitching, part of the pitch
 *       rotation projects onto the body z-axis and appears as a yaw
 *       rate. This is the kinematic coupling from pitch to yaw through
 *       bank angle.
 *
 *     cos(phi) * cos(theta) * psi_dot:
 *       The Euler yaw rate projected onto the body z-axis. At wings-level,
 *       cos(phi) = 1 and all of the yaw rate is body yaw. As the bank
 *       angle increases, less of the Euler yaw appears as body yaw and
 *       more appears as body pitch.
 *
 * Step 4: Rate limiting.
 * -----------------------
 *   The output body rate is clamped to [-max_rate, +max_rate] to prevent
 *   commanding rates that exceed the rudder's authority.
 *
 * ============================================================================
 * COORDINATED TURN METHOD
 * ============================================================================
 *
 *   Method 0 (open-loop): Uses only the kinematic coordinated turn formula.
 *     Good for aircraft with minimal adverse yaw. Simple and stable.
 *
 *   Method 1 (with sideslip compensation): Adds a feedback term on sideslip
 *     angle to actively drive sideslip to zero. Useful for aircraft with
 *     significant adverse yaw (e.g., large dihedral, swept wings).
 *     (Not fully implemented here; the feedback would come from an
 *     external sideslip estimator.)
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_att_control/ecl_yaw_controller.cpp
 *   - Stevens & Lewis, "Aircraft Control and Simulation", Chapter 3.4
 *   - Beard & McLain, "Small Unmanned Aircraft", Chapter 5
 *   - McCormick, "Aerodynamics, Aeronautics, and Flight Mechanics", Ch. 8
 */

#pragma once

namespace fw_att_control {

class YawController {
public:
    YawController() = default;
    ~YawController() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the maximum commanded yaw body rate (FW_Y_RMAX).
     *
     * @param max_rate  Maximum rate [rad/s], must be >= 0
     */
    void set_max_rate(float max_rate);

    /**
     * Set the coordinated turn method.
     *
     * @param method  0 = open-loop (formula only)
     *                1 = with sideslip compensation
     */
    void set_coordinated_method(int method);

    /**
     * Set the minimum airspeed for the coordinated turn computation.
     *
     * Below this airspeed the coordinated turn formula produces excessively
     * large yaw rates (because of the 1/V term). The controller uses this
     * as a floor on the airspeed value in the formula.
     *
     * @param airspeed_min  Minimum airspeed [m/s]
     */
    void set_airspeed_min(float airspeed_min);

    /* ---- Runtime interface ---- */

    /**
     * Compute the desired yaw body rate for coordinated flight.
     *
     * This is the main function called once per attitude control cycle (~250Hz).
     *
     * @param roll             Current roll angle (phi) [rad]
     * @param pitch            Current pitch angle (theta) [rad]
     * @param pitch_rate_euler The Euler pitch rate (theta_dot) [rad/s] from pitch controller
     * @param airspeed         True airspeed [m/s]
     * @return                 Desired yaw body rate (r) [rad/s]
     */
    float control_attitude(float roll, float pitch, float pitch_rate_euler, float airspeed);

    /* ---- Accessors (for logging / telemetry) ---- */

    float get_euler_rate_setpoint() const { return _euler_rate_setpoint; }

private:
    /* ---- Parameters ---- */

    /** Maximum commanded body yaw rate [rad/s] */
    float _max_rate = 0.7854f;  // ~45 deg/s default

    /** Coordinated turn method: 0=open-loop, 1=with sideslip compensation */
    int _coordinated_method = 0;

    /** Minimum airspeed used in the coordinated turn formula [m/s] */
    float _airspeed_min = 10.0f;

    /* ---- Internal state (for logging) ---- */

    /** Last computed Euler yaw rate setpoint (before Jacobian) */
    float _euler_rate_setpoint = 0.0f;
};

} // namespace fw_att_control
