/**
 * @file fw_roll_controller.cpp
 *
 * Implementation of the fixed-wing roll attitude controller.
 *
 * See fw_roll_controller.hpp for the full algorithm description and
 * derivation of the control law.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_att_control/ecl_roll_controller.cpp
 */

#include "fw_roll_controller.hpp"
#include "../common/math_utils.hpp"

#include <cmath>
#include <algorithm>

namespace fw_att_control {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void RollController::set_time_constant(float tc)
{
    /*
     * Guard against zero or negative time constants.
     * A minimum of 0.01s prevents division-by-zero and absurdly high gains.
     * In practice, values below ~0.2s are rarely used because they demand
     * extremely fast roll rates.
     */
    _tc = std::max(tc, 0.01f);
}

void RollController::set_max_rate(float max_rate)
{
    /*
     * Max rate must be non-negative. A value of 0 means "no rate limit"
     * is not meaningful, so we enforce a small positive floor.
     */
    _max_rate = std::max(max_rate, 0.01f);
}

/* ======================================================================
 * Main control law
 * ====================================================================== */

float RollController::control_attitude(float roll, float roll_sp, float pitch,
                                       float yaw_rate_euler)
{
    /*
     * STEP 1: Compute the roll attitude error.
     * ------------------------------------------
     * Both roll and roll_sp are in radians and should already be in the
     * range [-pi, pi]. We subtract to get the error. No wrap_pi is needed
     * for roll because roll angles are typically bounded to [-pi, pi] by
     * the estimator, and the desired roll from guidance never exceeds
     * the configured maximum bank angle (e.g., 45 degrees).
     *
     * The sign convention is:
     *   positive error -> aircraft needs to roll right -> positive rate command
     */
    const float roll_error = roll_sp - roll;

    /*
     * STEP 2: P-controller -- compute desired Euler roll rate.
     * ---------------------------------------------------------
     * The time-constant formulation:
     *
     *   phi_dot_desired = roll_error / tau
     *
     * This drives the error exponentially to zero with time constant tau.
     * The differential equation is:
     *
     *   d(phi_error)/dt = -phi_error / tau
     *
     * Solution: phi_error(t) = phi_error(0) * exp(-t/tau)
     *
     * So after one time constant, ~63% of the initial error is removed.
     *
     * Note: there is no integral or derivative term here. The inner
     * rate controller (downstream) provides those. This keeps the
     * attitude loop simple and avoids double-integration dynamics.
     */
    _euler_rate_setpoint = roll_error / _tc;

    /*
     * STEP 3: Euler-to-body Jacobian transformation.
     * -----------------------------------------------
     * The kinematic equation for roll body rate is:
     *
     *   p = phi_dot - sin(theta) * psi_dot
     *
     * Where:
     *   p         = roll body rate (what we command to the rate controller)
     *   phi_dot   = Euler roll rate (what we just computed)
     *   theta     = current pitch angle
     *   psi_dot   = Euler yaw rate (from coordinated turn calculation)
     *
     * Physical interpretation of the coupling term -sin(theta)*psi_dot:
     *   When the aircraft is pitched up (theta > 0) and yawing (psi_dot != 0),
     *   the yaw rotation has a component along the body x-axis. For example,
     *   in a 30-degree climb with a yaw rate of 10 deg/s, the coupling is:
     *     -sin(30deg) * 10 = -5 deg/s
     *   We must add this to the commanded roll Euler rate to get the correct
     *   body-frame roll rate that achieves the desired attitude change.
     *
     * Without this correction, the aircraft would exhibit unexpected rolling
     * during climbing/descending turns. This is a common source of
     * "mysterious" roll oscillations in poorly-implemented autopilots.
     */
    float body_rate = _euler_rate_setpoint - std::sin(pitch) * yaw_rate_euler;

    /*
     * STEP 4: Rate limiting.
     * -----------------------
     * Clamp the output to the symmetric rate limit. This prevents the
     * attitude controller from demanding rates that would:
     *   a) Exceed structural limits
     *   b) Cause the rate controller integrator to wind up
     *   c) Lead to aggressive maneuvers that could stall the wing
     *
     * The rate limit is symmetric for roll because most aircraft have
     * symmetric aileron authority (unlike pitch, which has asymmetric
     * limits for nose-up vs. nose-down).
     */
    body_rate = math::constrain(body_rate, -_max_rate, _max_rate);

    return body_rate;
}

} // namespace fw_att_control
