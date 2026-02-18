/**
 * @file RunwayTakeoff.cpp
 *
 * Implementation of the runway takeoff state machine.
 *
 * See RunwayTakeoff.hpp for the full state machine description, parameter
 * guidelines, and references.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_pos_control/runway_takeoff/RunwayTakeoff.cpp
 */

#include "RunwayTakeoff.hpp"
#include "../../common/math_utils.hpp"

#include <algorithm>
#include <cmath>

namespace fw_mode_manager {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void RunwayTakeoff::set_heading_hold_tc(float tc)
{
    /*
     * The heading hold time constant must be positive. A very small value
     * will produce aggressive yaw corrections that could cause directional
     * instability on the ground (especially with tricycle gear). A minimum
     * of 0.1s prevents impractically fast yaw response.
     */
    _heading_hold_tc = std::max(tc, 0.1f);
}

void RunwayTakeoff::set_initial_pitch(float pitch)
{
    /*
     * The initial pitch during ground roll. Clamped to a reasonable range:
     *   - Minimum 0 rad: no negative pitch during takeoff roll
     *   - Maximum ~20 deg: excessive pitch on the ground may cause tail strike
     */
    _initial_pitch = math::constrain(pitch, 0.0f, 0.35f);
}

void RunwayTakeoff::set_climbout_pitch(float pitch)
{
    /*
     * The climbout pitch angle. Clamped to a reasonable range:
     *   - Minimum ~5 deg: must produce positive climb rate
     *   - Maximum ~25 deg: excessive pitch risks stall at low speed
     */
    _climbout_pitch = math::constrain(pitch, 0.087f, 0.44f);
}

void RunwayTakeoff::set_min_airspeed_scaling(float scaling)
{
    /*
     * The scaling factor must be >= 1.0 (at minimum, rotate at stall speed).
     * Values below 1.0 would mean rotating below stall speed, which is
     * physically nonsensical.
     */
    _min_airspeed_scaling = std::max(scaling, 1.0f);
}

void RunwayTakeoff::set_rotation_airspeed(float airspeed)
{
    /*
     * A value of 0 means "auto" (computed from min airspeed).
     * Negative values are not meaningful.
     */
    _rotation_airspeed = std::max(airspeed, 0.0f);
}

void RunwayTakeoff::set_rotation_speed_threshold(float threshold)
{
    /*
     * The threshold fraction must be in [0.5, 1.0]. Values below 0.5
     * would start rotation far too early (at half the desired speed).
     * Values above 1.0 would delay rotation past the target speed.
     */
    _rotation_speed_threshold = math::constrain(threshold, 0.5f, 1.0f);
}

void RunwayTakeoff::set_min_airspeed(float min_airspeed)
{
    _min_airspeed = std::max(min_airspeed, 1.0f);
}

void RunwayTakeoff::set_clearance_altitude(float alt)
{
    _clearance_altitude = std::max(alt, 1.0f);
}

/* ======================================================================
 * Internal helpers
 * ====================================================================== */

float RunwayTakeoff::computeRotationAirspeed() const
{
    /*
     * Two modes for determining the rotation airspeed:
     *
     * 1. Explicit (rotation_airspeed > 0):
     *    Use the operator-specified value directly. This is preferred when
     *    the aircraft's takeoff performance has been characterized in flight
     *    testing and a known-good rotation speed is available.
     *
     * 2. Auto (rotation_airspeed = 0):
     *    Compute from the minimum flyable airspeed (approximately the stall
     *    speed) scaled by min_airspeed_scaling. The standard practice in
     *    aviation is:
     *
     *      V_rotate = V_stall * scaling_factor
     *
     *    With scaling = 1.3, this gives 30% margin above stall. This is
     *    slightly conservative for small UAVs (manned aircraft typically
     *    use 1.1-1.2), but provides extra margin for wind gusts and
     *    sensor noise.
     */
    if (_rotation_airspeed > math::FLT_EPSILON) {
        return _rotation_airspeed;
    }

    return _min_airspeed * _min_airspeed_scaling;
}

/* ======================================================================
 * State machine update
 * ====================================================================== */

void RunwayTakeoff::update(float airspeed, float altitude_agl, float dt)
{
    (void)dt;  // dt is available for future extensions (timing, filtering)

    /*
     * The state machine transitions based on airspeed and altitude:
     *
     *   IDLE -> CLIMBOUT_INIT:
     *     When airspeed exceeds the rotation threshold, the aircraft has
     *     enough dynamic pressure for the wings to generate lift and the
     *     control surfaces to be effective.
     *
     *   CLIMBOUT_INIT -> CLIMBOUT:
     *     Immediate transition. CLIMBOUT_INIT exists as a one-shot state
     *     to mark the rotation event. In a more complex implementation,
     *     this state could wait for the aircraft to actually achieve the
     *     climbout pitch angle before proceeding.
     *
     *   CLIMBOUT -> FLY:
     *     When the aircraft has climbed above the clearance altitude AGL.
     *     At this point, obstacle clearance is achieved and normal flight
     *     navigation can begin.
     */

    switch (_state) {

    case RunwayTakeoffState::IDLE: {
        /*
         * IDLE: Ground roll phase.
         *
         * The aircraft is accelerating down the runway. We monitor airspeed
         * and wait for it to reach the rotation speed.
         *
         * The effective trigger speed is:
         *   V_trigger = rotation_speed_threshold * V_rotate
         *
         * With defaults (threshold = 0.9, scaling = 1.3, min_airspeed = 10):
         *   V_rotate  = 10 * 1.3 = 13 m/s
         *   V_trigger = 0.9 * 13 = 11.7 m/s
         *
         * This means rotation begins slightly before the full rotation speed,
         * giving the aircraft time to pitch up as it continues accelerating.
         * By the time the pitch angle reaches climbout_pitch, the airspeed
         * will be at or above V_rotate.
         */
        const float v_rotate = computeRotationAirspeed();
        const float v_trigger = _rotation_speed_threshold * v_rotate;

        if (airspeed >= v_trigger) {
            _state = RunwayTakeoffState::CLIMBOUT_INIT;
        }
        break;
    }

    case RunwayTakeoffState::CLIMBOUT_INIT:
        /*
         * CLIMBOUT_INIT: Rotation transition.
         *
         * This is a transient state that immediately transitions to CLIMBOUT.
         * The pitch setpoint switches from initial_pitch to climbout_pitch
         * in this transition. The getPitchSetpoint() method returns the
         * appropriate value based on the current state.
         *
         * In a real aircraft, the rotation takes 1-3 seconds as the pitch
         * angle increases from ground-roll attitude to climb attitude.
         * The attitude controller handles the actual rate of pitch change.
         */
        _state = RunwayTakeoffState::CLIMBOUT;
        break;

    case RunwayTakeoffState::CLIMBOUT:
        /*
         * CLIMBOUT: Climbing to clearance altitude.
         *
         * The aircraft is climbing at climbout_pitch with full throttle.
         * We monitor altitude AGL and wait for clearance.
         *
         * The clearance altitude is chosen to ensure:
         *   - The aircraft is above any nearby obstacles
         *   - There is enough altitude to safely execute a turn if needed
         *   - The airspeed has stabilized at or above the minimum
         *
         * Typical clearance altitudes are 10-30 meters AGL. The FAA
         * recommends clearing all obstacles by at least 50 feet (15m)
         * before initiating any turns after takeoff.
         */
        if (altitude_agl >= _clearance_altitude) {
            _state = RunwayTakeoffState::FLY;
        }
        break;

    case RunwayTakeoffState::FLY:
        /*
         * FLY: Normal flight.
         *
         * The takeoff sequence is complete. The state remains here until
         * the state machine is explicitly reset. No further transitions
         * occur in this state.
         */
        break;
    }
}

/* ======================================================================
 * Setpoint generation
 * ====================================================================== */

float RunwayTakeoff::getPitchSetpoint(float pitch_sp_from_tecs) const
{
    /*
     * Pitch setpoint depends on the takeoff phase:
     *
     * IDLE:
     *   Use initial_pitch. This is a modest nose-up attitude (e.g., 10 deg)
     *   appropriate for ground roll. For tricycle gear aircraft, this keeps
     *   the nosewheel light. For taildragger, this is the three-point
     *   attitude. The slight positive pitch provides angle of attack for
     *   early lift generation.
     *
     * CLIMBOUT_INIT / CLIMBOUT:
     *   Use climbout_pitch. This is a steeper climb attitude (e.g., 15 deg)
     *   designed to maximize initial climb rate while maintaining a safe
     *   margin above stall. The pitch is fixed (not modulated by TECS)
     *   because at low altitude we prioritize consistent climb performance
     *   over precise airspeed tracking.
     *
     * FLY:
     *   Pass through the TECS pitch setpoint. At this point TECS has full
     *   authority to balance altitude and airspeed tracking optimally.
     */
    switch (_state) {
    case RunwayTakeoffState::IDLE:
        return _initial_pitch;

    case RunwayTakeoffState::CLIMBOUT_INIT:
    case RunwayTakeoffState::CLIMBOUT:
        return _climbout_pitch;

    case RunwayTakeoffState::FLY:
    default:
        return pitch_sp_from_tecs;
    }
}

float RunwayTakeoff::getYawSetpoint(float heading) const
{
    /*
     * Heading hold during ground roll and climb-out:
     *
     * During IDLE and CLIMBOUT, the aircraft must maintain the runway
     * heading. This is critical for safety:
     *   - Ground roll: prevents veering off the runway
     *   - Climb-out: prevents turning at low altitude where a stall/spin
     *     would be unrecoverable
     *
     * During FLY, we return the navigation heading unchanged, allowing
     * the mission or loiter controller to command turns.
     *
     * Note: The actual heading hold PID is implemented in the mode manager
     * or attitude controller. Here we simply return the appropriate
     * heading reference: either the captured initial heading (for ground
     * phases) or the navigation heading (for FLY).
     */
    switch (_state) {
    case RunwayTakeoffState::IDLE:
    case RunwayTakeoffState::CLIMBOUT_INIT:
    case RunwayTakeoffState::CLIMBOUT:
        /*
         * Return the captured initial heading if available. If no heading
         * was captured (shouldn't happen in normal operation), fall through
         * to the navigation heading.
         */
        if (_heading_captured) {
            return _initial_heading;
        }
        return heading;

    case RunwayTakeoffState::FLY:
    default:
        return heading;
    }
}

float RunwayTakeoff::getMinAirspeedScaling() const
{
    /*
     * During ground roll (IDLE), the minimum airspeed protection is
     * relaxed because the aircraft is expected to be below flying speed
     * while accelerating. We return 0 to disable the minimum airspeed
     * check -- TECS should not try to push the nose down to gain speed
     * while the aircraft is still on the ground.
     *
     * During CLIMBOUT_INIT and CLIMBOUT, we use the configured scaling
     * factor. The aircraft should maintain at least this multiple of
     * stall speed during the initial climb.
     *
     * During FLY, normal minimum airspeed scaling (1.0) applies, handled
     * by the outer controller. We return 1.0 to indicate no additional
     * scaling.
     */
    switch (_state) {
    case RunwayTakeoffState::IDLE:
        return 0.0f;

    case RunwayTakeoffState::CLIMBOUT_INIT:
    case RunwayTakeoffState::CLIMBOUT:
        return _min_airspeed_scaling;

    case RunwayTakeoffState::FLY:
    default:
        return 1.0f;
    }
}

/* ======================================================================
 * Reset
 * ====================================================================== */

void RunwayTakeoff::reset()
{
    /*
     * Full reset to initial conditions. Called when:
     *   - The vehicle first enters runway takeoff mode
     *   - A takeoff is aborted and needs to be restarted
     *   - The vehicle lands and is preparing for another takeoff
     */
    _state = RunwayTakeoffState::IDLE;
    _heading_captured = false;
    _initial_heading = 0.0f;
}

} // namespace fw_mode_manager
