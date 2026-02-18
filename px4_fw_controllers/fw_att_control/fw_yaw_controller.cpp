/**
 * @file fw_yaw_controller.cpp
 *
 * Implementation of the fixed-wing yaw attitude controller.
 *
 * See fw_yaw_controller.hpp for the full algorithm description and
 * derivation of the control law.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_att_control/ecl_yaw_controller.cpp
 */

#include "fw_yaw_controller.hpp"
#include "../common/math_utils.hpp"

#include <cmath>
#include <algorithm>

namespace fw_att_control {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void YawController::set_max_rate(float max_rate)
{
    /*
     * Max rate must be positive. A small floor is enforced to prevent the
     * constraint from collapsing to zero, which would eliminate all yaw
     * rate commands and leave the aircraft uncoordinated.
     */
    _max_rate = std::max(max_rate, 0.01f);
}

void YawController::set_coordinated_method(int method)
{
    /*
     * Clamp to valid range [0, 1].
     *   0 = Open-loop coordinated turn (formula only).
     *   1 = With sideslip feedback compensation.
     *
     * Method 0 is sufficient for most small UAS with low adverse yaw.
     * Method 1 adds active sideslip rejection and is useful for aircraft
     * with significant Dutch roll tendencies or adverse yaw from
     * differential drag during aileron deflection.
     */
    _coordinated_method = math::constrain(method, 0, 1);
}

void YawController::set_airspeed_min(float airspeed_min)
{
    /*
     * The coordinated turn formula divides by airspeed. Below this minimum,
     * the computed yaw rate becomes unreasonably large. A floor of 1 m/s
     * prevents near-division-by-zero. In practice, if the airspeed is this
     * low, the aircraft is either on the ground (where the wheel controller
     * handles yaw) or in a deep stall (where no controller is effective).
     */
    _airspeed_min = std::max(airspeed_min, 1.0f);
}

/* ======================================================================
 * Main control law
 * ====================================================================== */

float YawController::control_attitude(float roll, float pitch,
                                      float pitch_rate_euler, float airspeed)
{
    /*
     * STEP 1: Protect against low airspeed.
     * ----------------------------------------
     * The coordinated turn formula has a 1/V term. At very low airspeeds
     * (taxiing, deep stall, wind shear), the computed yaw rate would
     * become unreasonably large. We enforce a minimum airspeed for the
     * computation.
     *
     * The floor is applied only to the airspeed value used in the formula,
     * not to the actual airspeed measurement, so it does not affect other
     * systems.
     */
    const float airspeed_safe = std::max(airspeed, _airspeed_min);

    /*
     * STEP 2: Handle inverted flight and singularity protection.
     * ------------------------------------------------------------
     * The coordinated turn formula uses tan(phi), which has a singularity
     * at phi = +/- 90 degrees. For inverted flight (|phi| > 90 degrees),
     * the formula's assumptions break down entirely:
     *
     *   - In inverted flight, lift points "away" from the turn center,
     *     so the standard force balance no longer applies.
     *   - The sign of tan(phi) reverses past 90 degrees, which would
     *     command the wrong yaw direction.
     *
     * Our approach:
     *   - If |roll| > ~80 degrees, clamp the roll to +/- 80 degrees for
     *     the coordinated turn computation.
     *   - If |roll| > 90 degrees (inverted), set the Euler yaw rate to
     *     zero. The rate controller will still provide yaw damping, but
     *     we do not attempt coordinated turn logic in inverted flight.
     *
     * The 80 degree limit (approximately 1.396 radians) ensures:
     *   tan(80 deg) ≈ 5.67, which gives a maximum yaw rate of about
     *   5.67 * 9.81 / 15 ≈ 3.7 rad/s at 15 m/s. This is already beyond
     *   what most aircraft can achieve, so the rate limiter in Step 5
     *   will further constrain this.
     */
    static constexpr float roll_max = 1.396f;  // ~80 degrees

    const bool inverted = std::fabs(roll) > math::PI * 0.5f;

    /*
     * STEP 3: Compute the coordinated turn Euler yaw rate.
     * ------------------------------------------------------
     * The steady-state coordinated turn formula:
     *
     *   psi_dot = tan(phi) * cos(theta) * g / V
     *
     * This is derived from the balance of forces in a banked turn:
     *   - The vertical component of lift equals weight: L*cos(phi) = m*g
     *   - The horizontal component provides centripetal force: L*sin(phi) = m*V*psi_dot
     *   - Dividing: psi_dot = g * tan(phi) / V
     *   - The cos(theta) term adjusts for the fact that in a climbing or
     *     descending turn, the effective gravity component in the turn
     *     plane is reduced by cos(theta).
     *
     * In level flight (theta=0, cos(theta)=1):
     *   At 30 degrees bank and 20 m/s: psi_dot = tan(30)*9.81/20 = 0.283 rad/s ≈ 16 deg/s
     *   At 45 degrees bank and 20 m/s: psi_dot = tan(45)*9.81/20 = 0.491 rad/s ≈ 28 deg/s
     *   At 60 degrees bank and 15 m/s: psi_dot = tan(60)*9.81/15 = 1.133 rad/s ≈ 65 deg/s
     */
    if (inverted) {
        /*
         * In inverted flight, set the Euler yaw rate to zero.
         * The rate controller's yaw damping term will still provide
         * stability, but we do not attempt to compute a coordinated
         * turn rate because the force balance assumptions are invalid.
         */
        _euler_rate_setpoint = 0.0f;
    } else {
        /*
         * Normal (upright) flight. Clamp roll to avoid the tan() singularity.
         * The sign of roll is preserved so that the yaw rate direction
         * is correct: positive roll -> positive yaw rate (turn right).
         */
        const float roll_clamped = math::constrain(roll, -roll_max, roll_max);

        _euler_rate_setpoint = std::tan(roll_clamped) * std::cos(pitch)
                             * math::GRAVITY / airspeed_safe;
    }

    /*
     * STEP 4: Euler-to-body Jacobian transformation.
     * ------------------------------------------------
     * The kinematic equation for yaw body rate is:
     *
     *   r = -sin(phi) * theta_dot + cos(phi) * cos(theta) * psi_dot
     *
     * Where:
     *   r         = yaw body rate (what we command to the rate controller)
     *   phi       = current roll angle
     *   theta_dot = Euler pitch rate (from the pitch controller output)
     *   theta     = current pitch angle
     *   psi_dot   = Euler yaw rate (what we computed above)
     *
     * Physical interpretation of the coupling term -sin(phi) * theta_dot:
     *   When the aircraft is banked and pitching, part of the pitch
     *   rotation appears as yaw in the body frame. For example, at 30
     *   degrees bank with a pitch rate of 10 deg/s:
     *     -sin(30) * 10 = -5 deg/s of body yaw rate
     *   This coupling ensures that the body-frame rate command correctly
     *   accounts for the interaction between pitch and yaw through the
     *   roll angle.
     *
     * Physical interpretation of cos(phi) * cos(theta) * psi_dot:
     *   The Euler yaw rate projected onto the body z-axis. At wings-level
     *   (phi = 0), all of psi_dot appears as body yaw rate. As bank angle
     *   increases, more of the Euler yaw rate projects onto the body
     *   y-axis (pitch) rather than z-axis (yaw). At 90 degrees bank,
     *   cos(phi) = 0 and the Euler yaw becomes entirely body pitch --
     *   which is physically correct: in a knife-edge turn, "yawing" the
     *   aircraft's nose around is actually a body pitch rotation.
     */
    float body_rate = -std::sin(roll) * pitch_rate_euler
                    + std::cos(roll) * std::cos(pitch) * _euler_rate_setpoint;

    /*
     * STEP 5: Rate limiting.
     * -----------------------
     * Clamp the output to the symmetric rate limit. The yaw rate limit
     * is typically set based on the rudder's maximum authority. Unlike
     * pitch, the yaw limits are symmetric because rudder authority is
     * generally symmetric (equal deflection left and right).
     *
     * The default of ~45 deg/s is generous for most small UAS. In
     * practice, the rudder authority and Dutch roll mode dynamics
     * determine the practical limit.
     */
    body_rate = math::constrain(body_rate, -_max_rate, _max_rate);

    return body_rate;
}

} // namespace fw_att_control
