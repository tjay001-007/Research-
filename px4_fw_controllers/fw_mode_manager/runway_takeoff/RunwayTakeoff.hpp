/**
 * @file RunwayTakeoff.hpp
 *
 * Runway takeoff state machine for fixed-wing aircraft.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * For aircraft that take off from a runway (as opposed to hand/catapult launch),
 * the takeoff sequence involves a ground roll, rotation, and climb-out. This
 * module manages the state machine that controls the aircraft through these
 * phases, providing appropriate pitch, yaw, and airspeed setpoints at each
 * stage.
 *
 * The takeoff sequence is:
 *
 *   1. IDLE:          Waiting on the runway, not yet rolling
 *   2. CLIMBOUT_INIT: Ground roll complete, initial rotation to takeoff pitch
 *   3. CLIMBOUT:      Climbing at a fixed pitch angle to clear obstacles
 *   4. FLY:           Normal flight, handoff to the mission controller
 *
 * ============================================================================
 * STATE MACHINE DESCRIPTION
 * ============================================================================
 *
 *   IDLE
 *   ----
 *   The aircraft is on the runway with the motor running (or about to start).
 *   During this phase:
 *     - Yaw is held to the runway heading using a heading-hold controller
 *     - Pitch is set to `initial_pitch` (typically 5-10 degrees nose-up to
 *       keep the nosewheel/tailwheel on the ground while providing slight
 *       angle of attack for lift generation)
 *     - The throttle is at full takeoff power
 *
 *   The aircraft accelerates down the runway. When the airspeed reaches
 *   `rotation_speed_threshold * rotation_airspeed`, the aircraft has enough
 *   speed to become airborne, and we transition to CLIMBOUT_INIT.
 *
 *   CLIMBOUT_INIT
 *   -------------
 *   The rotation phase. The aircraft pitches up to `climbout_pitch` (typically
 *   12-15 degrees). This is a transient state that immediately transitions
 *   to CLIMBOUT once the pitch setpoint is applied.
 *
 *   The purpose of this separate state is to provide a clean transition
 *   point where the pitch target changes from ground-roll pitch to
 *   climb-out pitch. Some implementations use this state to perform
 *   additional checks (gear retraction, flap adjustment, etc.).
 *
 *   CLIMBOUT
 *   --------
 *   The aircraft is climbing at `climbout_pitch` with full throttle. The
 *   heading is still held to the runway heading (no turns until safe altitude).
 *
 *   The aircraft climbs until it reaches the clearance altitude (provided
 *   by the caller as the altitude AGL). Once above clearance, we transition
 *   to FLY.
 *
 *   FLY
 *   ---
 *   Normal flight. The takeoff sequence is complete. The mode manager takes
 *   over with mission navigation setpoints. Pitch is no longer clamped to
 *   the climbout angle; instead, TECS (Total Energy Control System) manages
 *   the pitch/throttle balance for altitude and airspeed tracking.
 *
 * ============================================================================
 * AIRSPEED-DEPENDENT ROTATION
 * ============================================================================
 *
 * The rotation airspeed is the speed at which the aircraft is commanded to
 * rotate (pitch up) for takeoff. Two modes are supported:
 *
 *   1. Explicit rotation_airspeed > 0:
 *      The aircraft rotates at this specific airspeed. Useful when the
 *      takeoff speed is well-characterized (e.g., from flight test data).
 *
 *   2. rotation_airspeed = 0 (auto):
 *      The rotation airspeed is computed from the minimum airspeed scaled
 *      by `min_airspeed_scaling`. A typical scaling of 1.3 means rotation
 *      occurs at 130% of stall speed, providing a 30% margin above stall.
 *      This follows the standard aviation practice of V_rotate ~ 1.1-1.3 * V_stall.
 *
 * The `rotation_speed_threshold` (default 0.9) provides hysteresis: the
 * aircraft begins rotation at 90% of the computed rotation speed. This
 * accounts for airspeed sensor lag and provides earlier rotation initiation
 * so the aircraft reaches full rotation pitch near the desired speed.
 *
 * ============================================================================
 * HEADING HOLD
 * ============================================================================
 *
 * During ground roll and initial climb-out, the aircraft must maintain the
 * runway heading. This is critical because:
 *   - Crosswind on the ground can push the aircraft off the runway
 *   - Asymmetric thrust (single-engine prop torque) causes yaw tendency
 *   - Turning at low altitude during climb-out is dangerous
 *
 * The heading hold uses a simple proportional controller with a time
 * constant `heading_hold_tc`. A slower time constant (larger value) gives
 * gentler corrections, while a faster one tracks the heading more tightly.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_pos_control/runway_takeoff/RunwayTakeoff.cpp
 *   - FAA Airplane Flying Handbook, Chapter 5: Takeoffs and Departure Climbs
 *   - Anderson, "Introduction to Flight", Chapter 6
 */

#pragma once

#include <cstdint>

namespace fw_mode_manager {

/**
 * Runway takeoff state machine states.
 *
 * These represent the progression from idle on the runway through
 * ground roll, rotation, climb-out, and transition to normal flight.
 */
enum class RunwayTakeoffState : uint8_t {
    IDLE           = 0,   ///< On the runway, waiting or beginning ground roll
    CLIMBOUT_INIT  = 1,   ///< Rotation initiated, pitching up to climb angle
    CLIMBOUT       = 2,   ///< Climbing at constant pitch to clearance altitude
    FLY            = 3    ///< Normal flight, takeoff sequence complete
};

class RunwayTakeoff {
public:
    RunwayTakeoff() = default;
    ~RunwayTakeoff() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the heading hold time constant (RWTO_HDG_TC).
     *
     * Controls how aggressively the yaw controller tracks the runway heading
     * during ground roll and initial climb-out. Smaller values give tighter
     * heading tracking but may cause oscillation with nosewheel steering.
     *
     * @param tc  Time constant [s], must be > 0
     */
    void set_heading_hold_tc(float tc);

    /**
     * Set the initial pitch angle during ground roll (RWTO_PSP).
     *
     * This is the pitch setpoint during the ground roll phase (IDLE state).
     * A slight nose-up attitude (5-10 degrees) keeps the tail down and
     * provides angle of attack for lift generation as speed increases.
     *
     * @param pitch  Initial pitch angle [rad]
     */
    void set_initial_pitch(float pitch);

    /**
     * Set the climbout pitch angle (RWTO_CLMB_PSP).
     *
     * After rotation, the aircraft pitches up to this angle for the
     * climb-out phase. Typically 12-18 degrees, chosen to balance climb
     * rate against stall margin.
     *
     * @param pitch  Climbout pitch angle [rad]
     */
    void set_climbout_pitch(float pitch);

    /**
     * Set the minimum airspeed scaling factor (RWTO_AIRSPD_SCL).
     *
     * When rotation_airspeed is 0 (auto mode), the rotation speed is
     * computed as min_airspeed * min_airspeed_scaling. A value of 1.3
     * means rotation at 130% of stall speed.
     *
     * @param scaling  Scaling factor, must be >= 1.0
     */
    void set_min_airspeed_scaling(float scaling);

    /**
     * Set the explicit rotation airspeed (RWTO_ROT_AIRSPD).
     *
     * If > 0, this overrides the auto-computed rotation speed.
     * If = 0, the rotation speed is auto-computed from min airspeed.
     *
     * @param airspeed  Rotation airspeed [m/s], 0 = auto
     */
    void set_rotation_airspeed(float airspeed);

    /**
     * Set the rotation speed threshold fraction (RWTO_ROT_THR).
     *
     * The aircraft begins rotation when airspeed reaches this fraction
     * of the rotation airspeed. A value of 0.9 means rotation starts at
     * 90% of the full rotation speed.
     *
     * @param threshold  Fraction [0, 1], typically 0.8-1.0
     */
    void set_rotation_speed_threshold(float threshold);

    /**
     * Set the minimum airspeed parameter.
     *
     * Used for auto-computing rotation airspeed when rotation_airspeed = 0.
     *
     * @param min_airspeed  Minimum flyable airspeed [m/s]
     */
    void set_min_airspeed(float min_airspeed);

    /**
     * Set the clearance altitude AGL for transitioning to FLY.
     *
     * @param alt  Clearance altitude above ground level [m]
     */
    void set_clearance_altitude(float alt);

    /* ---- Runtime interface ---- */

    /**
     * Update the runway takeoff state machine.
     *
     * This is the main function called once per control cycle. It advances
     * the state machine based on current airspeed and altitude conditions.
     *
     * @param airspeed      Current calibrated airspeed [m/s]
     * @param altitude_agl  Current altitude above ground level [m]
     * @param dt            Time step since last call [s]
     */
    void update(float airspeed, float altitude_agl, float dt);

    /**
     * Get the current state of the takeoff state machine.
     *
     * @return  Current RunwayTakeoffState enum value
     */
    RunwayTakeoffState getState() const { return _state; }

    /**
     * Get the pitch setpoint for the current takeoff phase.
     *
     * During IDLE and CLIMBOUT_INIT: returns the initial or climbout pitch.
     * During CLIMBOUT: returns the climbout pitch.
     * During FLY: returns the TECS pitch setpoint (passed through).
     *
     * @param pitch_sp_from_tecs  The pitch setpoint from TECS (used in FLY state)
     * @return  Pitch setpoint [rad]
     */
    float getPitchSetpoint(float pitch_sp_from_tecs) const;

    /**
     * Get the yaw setpoint for heading hold during ground roll and climb-out.
     *
     * During IDLE and CLIMBOUT: returns the clamped heading error correction.
     * During FLY: returns the input heading unchanged (no heading hold).
     *
     * @param heading  Current desired heading from navigation [rad]
     * @return  Yaw heading setpoint [rad]
     */
    float getYawSetpoint(float heading) const;

    /**
     * Get the minimum airspeed scaling factor for the current phase.
     *
     * During ground roll and initial climb, the minimum airspeed scaling
     * may be relaxed to allow the aircraft to fly at lower speeds while
     * still on the runway. During FLY, the normal scaling applies.
     *
     * @return  Minimum airspeed scaling factor
     */
    float getMinAirspeedScaling() const;

    /**
     * Check if the aircraft is still on the ground (IDLE state).
     *
     * @return  true if in IDLE state (ground roll)
     */
    bool isOnGround() const { return _state == RunwayTakeoffState::IDLE; }

    /**
     * Reset the state machine to IDLE.
     *
     * Called when re-entering takeoff mode or when the takeoff is aborted.
     */
    void reset();

private:
    /* ---- Parameters ---- */

    /** Heading hold time constant [s] */
    float _heading_hold_tc = 1.0f;

    /** Pitch angle during ground roll [rad] (~10 deg) */
    float _initial_pitch = 0.1745f;

    /** Pitch angle during climb-out [rad] (~15 deg) */
    float _climbout_pitch = 0.2618f;

    /** Scaling factor for auto rotation speed computation */
    float _min_airspeed_scaling = 1.3f;

    /** Explicit rotation airspeed [m/s], 0 = auto from min airspeed */
    float _rotation_airspeed = 0.0f;

    /** Fraction of rotation airspeed at which rotation begins */
    float _rotation_speed_threshold = 0.9f;

    /** Minimum flyable airspeed [m/s] (for auto rotation speed) */
    float _min_airspeed = 10.0f;

    /** Clearance altitude AGL for transition to FLY [m] */
    float _clearance_altitude = 10.0f;

    /* ---- Internal state ---- */

    /** Current state machine state */
    RunwayTakeoffState _state = RunwayTakeoffState::IDLE;

    /** Runway heading captured at the start of the takeoff roll [rad] */
    float _initial_heading = 0.0f;

    /** Whether the initial heading has been captured */
    bool _heading_captured = false;

    /* ---- Internal helpers ---- */

    /**
     * Compute the effective rotation airspeed.
     *
     * If an explicit rotation airspeed is set, use it directly.
     * Otherwise, compute from minimum airspeed and scaling factor.
     *
     * @return  Rotation airspeed [m/s]
     */
    float computeRotationAirspeed() const;
};

} // namespace fw_mode_manager
