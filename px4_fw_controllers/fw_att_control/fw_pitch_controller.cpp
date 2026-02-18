/**
 * @file fw_pitch_controller.cpp
 *
 * Implementation of the fixed-wing pitch attitude controller.
 *
 * See fw_pitch_controller.hpp for the full algorithm description and
 * derivation of the control law.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_att_control/ecl_pitch_controller.cpp
 */

#include "fw_pitch_controller.hpp"
#include "../common/math_utils.hpp"

#include <cmath>
#include <algorithm>

namespace fw_att_control {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void PitchController::set_time_constant(float tc)
{
    /*
     * Guard against zero or negative time constants.
     * A minimum of 0.01s prevents division-by-zero and absurdly high gains.
     * In practice, values below ~0.2s are rarely used because they demand
     * extremely fast pitch rates that can cause structural loading issues
     * or trigger the short-period oscillation mode.
     */
    _tc = std::max(tc, 0.01f);
}

void PitchController::set_max_rate_pos(float max_rate_pos)
{
    /*
     * Maximum nose-up rate. Must be positive. A small floor is enforced
     * to prevent the constraint from collapsing to zero, which would
     * prevent any nose-up pitch commands.
     */
    _max_rate_pos = std::max(max_rate_pos, 0.01f);
}

void PitchController::set_max_rate_neg(float max_rate_neg)
{
    /*
     * Maximum nose-down rate. Stored as a positive value internally;
     * it is applied as the negative clamp: body_rate >= -_max_rate_neg.
     * This convention matches PX4, where FW_P_RMAX_NEG is specified
     * as a positive number representing the magnitude of the limit.
     */
    _max_rate_neg = std::max(max_rate_neg, 0.01f);
}

/* ======================================================================
 * Main control law
 * ====================================================================== */

float PitchController::control_attitude(float pitch, float pitch_sp, float roll,
                                        float yaw_rate_euler)
{
    /*
     * STEP 1: Compute the pitch attitude error.
     * -------------------------------------------
     * Both pitch and pitch_sp are in radians. Pitch angles are bounded
     * to [-pi/2, pi/2] by the attitude estimator, so no wrapping is
     * needed. Typical commanded pitch angles range from about -15 degrees
     * (descent) to +20 degrees (climb).
     *
     * The sign convention is:
     *   positive error -> aircraft needs to pitch up -> positive rate command
     */
    const float pitch_error = pitch_sp - pitch;

    /*
     * STEP 2: P-controller -- compute desired Euler pitch rate.
     * ----------------------------------------------------------
     * The time-constant formulation:
     *
     *   theta_dot_desired = pitch_error / tau
     *
     * This drives the error exponentially to zero with time constant tau.
     * The differential equation is:
     *
     *   d(theta_error)/dt = -theta_error / tau
     *
     * Solution: theta_error(t) = theta_error(0) * exp(-t/tau)
     *
     * So after one time constant, ~63% of the initial error is removed.
     *
     * Note: there is no integral or derivative term here. The inner
     * rate controller (downstream) provides those. This keeps the
     * attitude loop simple and avoids double-integration dynamics.
     */
    _euler_rate_setpoint = pitch_error / _tc;

    /*
     * STEP 3: Euler-to-body Jacobian transformation.
     * ------------------------------------------------
     * The kinematic equation for pitch body rate is:
     *
     *   q = cos(phi) * theta_dot + cos(theta) * sin(phi) * psi_dot
     *
     * Where:
     *   q           = pitch body rate (what we command to the rate controller)
     *   phi         = current roll angle
     *   theta_dot   = Euler pitch rate (what we just computed)
     *   theta       = current pitch angle
     *   psi_dot     = Euler yaw rate (from coordinated turn calculation)
     *
     * Physical interpretation of the two terms:
     *
     *   cos(phi) * theta_dot:
     *     When the aircraft is banked (phi != 0), the Euler pitch rate does
     *     not fully align with the body y-axis. At 60 degrees of bank,
     *     cos(60) = 0.5, so only half of the commanded Euler pitch rate
     *     becomes body pitch rate. The controller must account for this
     *     reduction. In a steep bank, the controller naturally commands
     *     more Euler pitch rate to achieve the desired body rate.
     *
     *   cos(theta) * sin(phi) * psi_dot:
     *     During a coordinated turn at non-zero bank angle, part of the
     *     yaw rotation (psi_dot) projects onto the body y-axis. This term
     *     accounts for that kinematic coupling. For a typical bank angle
     *     of 30 degrees and a yaw rate of 15 deg/s, this contributes:
     *       cos(0) * sin(30) * 15 = 0.5 * 15 = 7.5 deg/s
     *     to the body pitch rate. Neglecting this term causes the aircraft
     *     to "nose down" in coordinated turns -- the classic loss of
     *     altitude in turns that many pilots experience.
     */
    float body_rate = std::cos(roll) * _euler_rate_setpoint
                    + std::cos(pitch) * std::sin(roll) * yaw_rate_euler;

    /*
     * STEP 4: Asymmetric rate limiting.
     * -----------------------------------
     * Clamp the output to [-max_rate_neg, +max_rate_pos]. Unlike roll,
     * pitch has asymmetric limits because:
     *
     *   Nose-up (positive q):
     *     Limited by stall margin. At low speed, commanding too much
     *     nose-up rate increases angle of attack and can trigger a stall.
     *     FW_P_RMAX_POS is typically set conservatively based on flight
     *     testing at minimum operational airspeed.
     *
     *   Nose-down (negative q):
     *     Limited by structural considerations. At high speed, a large
     *     nose-down pitch rate can cause negative g-loads that exceed
     *     the structural design limits. FW_P_RMAX_NEG protects against
     *     this.
     *
     * Additionally, asymmetric limits prevent the attitude controller
     * from demanding rates that would cause the rate controller
     * integrator to wind up differently in each direction.
     */
    body_rate = math::constrain(body_rate, -_max_rate_neg, _max_rate_pos);

    return body_rate;
}

} // namespace fw_att_control
