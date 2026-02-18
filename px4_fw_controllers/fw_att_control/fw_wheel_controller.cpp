/**
 * @file fw_wheel_controller.cpp
 *
 * Implementation of the fixed-wing wheel (ground steering) controller.
 *
 * See fw_wheel_controller.hpp for the full algorithm description and
 * derivation of the control law.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_att_control/ecl_wheel_controller.cpp
 */

#include "fw_wheel_controller.hpp"
#include "../common/math_utils.hpp"

#include <cmath>
#include <algorithm>

namespace fw_att_control {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void WheelController::set_time_constant(float tc)
{
    /*
     * Guard against zero or negative time constants.
     * A minimum of 0.01s prevents division-by-zero. On the ground, time
     * constants below ~0.3s are usually impractical because the steering
     * mechanism (nose wheel servo, differential braking) has significant
     * physical lag and backlash.
     */
    _tc = std::max(tc, 0.01f);
}

void WheelController::set_k_ff(float k_ff)
{
    /*
     * Feedforward gain. A floor of zero is enforced (no negative
     * feedforward). Typical values are 0.1 to 0.5. Higher values provide
     * more responsive tracking but can cause oscillation on surfaces
     * with non-linear friction characteristics (e.g., wet grass).
     */
    _k_ff = std::max(k_ff, 0.0f);
}

void WheelController::set_k_p(float k_p)
{
    /*
     * Proportional gain on rate error. Must be non-negative.
     * This gain compensates for the difference between the desired
     * and actual yaw rate. Higher values give tighter rate tracking
     * but can excite the yaw oscillation mode (shimmy) on nose-wheel
     * aircraft.
     */
    _k_p = std::max(k_p, 0.0f);
}

void WheelController::set_k_i(float k_i)
{
    /*
     * Integral gain. Must be non-negative. The integrator handles
     * steady-state disturbances like crosswind forces and steering
     * mechanism bias. Typical values are small (0.01 to 0.2) because
     * the integrator accumulates over time and aggressive integration
     * can cause sustained oscillations.
     */
    _k_i = std::max(k_i, 0.0f);
}

void WheelController::set_max_rate(float max_rate)
{
    /*
     * Maximum yaw rate on the ground. A small positive floor is enforced.
     * The default of ~30 deg/s is appropriate for most fixed-wing UAS.
     * Larger aircraft may need lower limits due to tire dynamics.
     */
    _max_rate = std::max(max_rate, 0.01f);
}

void WheelController::set_integrator_max(float integrator_max)
{
    /*
     * Maximum integrator contribution. This prevents windup, where the
     * integrator accumulates a large value during sustained error (e.g.,
     * strong crosswind while taxiing) and then causes overshoot when the
     * error reverses.
     *
     * The value is in normalized units [-1, +1], matching the output
     * range. A typical value of 0.3 means the integrator alone cannot
     * command more than 30% of full steering deflection.
     */
    _integrator_max = std::max(integrator_max, 0.0f);
}

/* ======================================================================
 * Integrator management
 * ====================================================================== */

void WheelController::reset_integrator()
{
    /*
     * Called on mode transitions (e.g., landing -> taxi, or controller
     * re-engagement). The integrator must be zeroed to prevent accumulated
     * state from a previous maneuver from affecting the current one.
     *
     * Example scenario: during landing rollout on a crosswind runway,
     * the integrator builds up to counteract the weathervane tendency.
     * If the aircraft then transitions to taxi mode on a different heading,
     * the stale integrator would cause an immediate steering transient.
     */
    _integrator = 0.0f;
}

/* ======================================================================
 * Main control law
 * ====================================================================== */

float WheelController::control_attitude(float yaw, float yaw_sp, float yaw_rate,
                                        float groundspeed, float groundspeed_scaler,
                                        float dt)
{
    /*
     * STEP 1 (OUTER LOOP): Compute the yaw heading error.
     * ------------------------------------------------------
     * Yaw angles span the full circle, so we MUST use wrap_pi() to
     * ensure the error is always in [-pi, pi]. Without wrapping, if
     * the aircraft is at yaw = +170 degrees and the setpoint is
     * -170 degrees, the naive error would be -340 degrees, causing
     * the aircraft to turn the long way around instead of the short
     * 20-degree turn.
     *
     * The sign convention is:
     *   positive error -> aircraft needs to yaw right -> positive rate command
     */
    const float yaw_error = math::wrap_pi(yaw_sp - yaw);

    /*
     * STEP 2 (OUTER LOOP): P-controller -- compute desired body yaw rate.
     * ---------------------------------------------------------------------
     * The time-constant formulation:
     *
     *   rate_setpoint = yaw_error / tau
     *
     * On the ground, we work directly with body rates (no Euler-to-body
     * Jacobian needed) because:
     *   - Roll is approximately zero on the ground (wings level)
     *   - Pitch is approximately constant (level ground or slight slope)
     *   - With roll ≈ 0, the Jacobian simplifies to r ≈ psi_dot
     *
     * Therefore, the Euler yaw rate IS approximately the body yaw rate,
     * and no transformation is needed.
     */
    _rate_setpoint = yaw_error / _tc;

    /*
     * STEP 3: Rate limiting.
     * -----------------------
     * Clamp the rate setpoint to prevent commanding rates that would:
     *   - Exceed the steering mechanism's physical limits
     *   - Cause tire scrubbing (lateral tire slip)
     *   - Lead to loss of directional control at high groundspeed
     *
     * The rate limit is symmetric because ground steering authority
     * is typically symmetric (equal left and right).
     */
    _rate_setpoint = math::constrain(_rate_setpoint, -_max_rate, _max_rate);

    /*
     * STEP 4 (INNER LOOP): Compute the rate tracking error.
     * -------------------------------------------------------
     * The rate error is the difference between the desired and actual
     * body yaw rate. The actual rate comes from the gyroscope (z-axis).
     */
    const float rate_error = _rate_setpoint - yaw_rate;

    /*
     * STEP 5 (INNER LOOP): Feedforward term.
     * ----------------------------------------
     * The feedforward provides an immediate steering command proportional
     * to the desired rate, scaled by groundspeed:
     *
     *   ff_term = rate_setpoint * k_ff * groundspeed_scaler
     *
     * The feedforward bypasses the feedback loop entirely and provides
     * the "expected" steering command for the desired yaw rate. This
     * gives much faster response than waiting for the feedback terms
     * (P and I) to build up.
     *
     * The groundspeed_scaler (typically V_trim / V_ground) reduces the
     * feedforward at high speed because less steering deflection is needed
     * when the tires generate more lateral force.
     */
    const float ff_term = _rate_setpoint * _k_ff * groundspeed_scaler;

    /*
     * STEP 6 (INNER LOOP): Proportional term on rate error.
     * -------------------------------------------------------
     * The proportional term corrects for the difference between desired
     * and actual yaw rate:
     *
     *   p_term = rate_error * k_p * groundspeed_scaler^2
     *
     * The squared scaler amplifies the correction at low groundspeeds,
     * compensating for the reduced tire lateral force at low speeds.
     * At high groundspeeds, the scaler^2 becomes small, preventing
     * aggressive steering corrections that could destabilize the aircraft.
     *
     * Physical reasoning for scaler^2:
     *   - Tire lateral force ≈ proportional to V (at low slip angles)
     *   - Yaw moment from steering ≈ proportional to V * force ≈ V^2
     *   - So the steering gain needs to scale as 1/V^2 to maintain
     *     constant closed-loop bandwidth
     *   - Since scaler ≈ V_trim/V, scaler^2 ≈ V_trim^2/V^2, which
     *     provides the desired 1/V^2 scaling
     */
    const float p_term = rate_error * _k_p * groundspeed_scaler * groundspeed_scaler;

    /*
     * STEP 7 (INNER LOOP): Integral term with conditional integration.
     * ------------------------------------------------------------------
     * The integrator accumulates the rate error over time:
     *
     *   integrator += rate_error * k_i * dt
     *
     * CRITICAL: The integrator is only updated when groundspeed > 1 m/s.
     * When the aircraft is stationary:
     *   - Steering commands have no effect on yaw rate (static friction
     *     holds the aircraft)
     *   - The rate error is non-zero (desired rate != 0, but actual = 0)
     *   - Without this guard, the integrator would wind up to its maximum
     *     value while the aircraft sits still
     *   - When the aircraft starts moving, the accumulated integrator
     *     would cause a violent steering jerk
     *
     * The 1 m/s threshold is chosen because:
     *   - Above 1 m/s, the tires are rolling and steering is effective
     *   - Below 1 m/s, the aircraft may be stationary or just starting
     *     to move, and steering response is unreliable
     *
     * The integrator is clamped to [-integrator_max, +integrator_max]
     * to prevent windup even during prolonged maneuvers.
     */
    static constexpr float min_groundspeed_for_integration = 1.0f;

    if (groundspeed > min_groundspeed_for_integration) {
        _integrator += rate_error * _k_i * dt;
        _integrator = math::constrain(_integrator, -_integrator_max, _integrator_max);
    }

    /*
     * STEP 8: Sum all terms to produce the steering command.
     * --------------------------------------------------------
     * The output is the sum of feedforward, proportional, and integral:
     *
     *   output = ff_term + p_term + integrator
     *
     * The output is a normalized steering command in [-1, +1], where:
     *   +1 = full right steering
     *   -1 = full left steering
     *
     * This normalized command is sent to the control allocator, which
     * maps it to the appropriate actuator (nose wheel servo angle,
     * differential braking percentage, etc.).
     */
    float output = ff_term + p_term + _integrator;

    /*
     * STEP 9: Output clamping.
     * --------------------------
     * Clamp the output to the valid actuator range. Values outside
     * [-1, +1] are physically meaningless and could cause issues
     * in the control allocator or actuator drivers.
     */
    output = math::constrain(output, -1.0f, 1.0f);

    return output;
}

} // namespace fw_att_control
