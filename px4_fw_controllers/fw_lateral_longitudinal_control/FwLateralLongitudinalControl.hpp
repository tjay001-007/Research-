/**
 * @file FwLateralLongitudinalControl.hpp
 *
 * Fixed-wing lateral (path-following) and longitudinal (energy management)
 * controller -- the OUTER-MOST guidance loop.
 *
 * ============================================================================
 * ARCHITECTURE
 * ============================================================================
 *
 * This module sits at the top of the fixed-wing control cascade:
 *
 *   Mode manager  -->  **Lateral/Longitudinal control**  -->  Attitude control  -->  Rate control
 *                          (this module)                        (fw_att_control)      (fw_rate_control)
 *
 * It receives high-level guidance commands (desired altitude, airspeed,
 * lateral acceleration) from the mode manager / navigator, and produces
 * attitude and thrust setpoints that feed the downstream attitude controller.
 *
 * The module contains two semi-independent control algorithms:
 *
 *   1. TECS -- Total Energy Control System (longitudinal axis)
 *      Manages pitch and throttle to track altitude and airspeed simultaneously.
 *
 *   2. Lateral controller (simplified NPFG)
 *      Converts a lateral acceleration command into a roll angle setpoint.
 *
 * ============================================================================
 * TECS -- TOTAL ENERGY CONTROL SYSTEM
 * ============================================================================
 *
 * Concept
 * -------
 * A fixed-wing aircraft in flight has two forms of mechanical energy:
 *
 *   Kinetic energy:    E_k = (1/2) * m * V^2
 *   Potential energy:  E_p = m * g * h
 *
 *   Total energy:      E   = E_k + E_p = (1/2) * m * V^2 + m * g * h
 *
 * The key insight of TECS is that the aircraft's two primary longitudinal
 * controls -- throttle and pitch (elevator) -- have fundamentally different
 * effects on the energy state:
 *
 *   THROTTLE controls the TOTAL energy rate.
 *     - Opening the throttle adds energy to the system (engine does work).
 *     - Closing the throttle removes energy (drag dissipates it).
 *     - Throttle cannot change the energy DISTRIBUTION between speed and height.
 *
 *   PITCH controls the ENERGY DISTRIBUTION.
 *     - Pitching up converts kinetic energy to potential energy (aircraft
 *       trades speed for height -- it climbs but slows down).
 *     - Pitching down converts potential energy to kinetic energy (aircraft
 *       descends but accelerates).
 *     - In steady state, pitch does not change total energy.
 *
 * This decoupling is the fundamental reason why TECS works: instead of having
 * one loop for altitude and another for airspeed that fight each other through
 * the aircraft dynamics, TECS decomposes the problem into two non-competing
 * loops in the energy domain.
 *
 * Mathematical Derivation
 * -----------------------
 * Start from total energy:
 *
 *   E = (1/2) * m * V^2 + m * g * h
 *
 * Take the time derivative:
 *
 *   E_dot = m * V * V_dot + m * g * h_dot
 *
 * Normalise by (m * g * V) to get the "specific energy rate" (dimensionless
 * rate, which makes the two energy forms comparable regardless of mass or speed):
 *
 *   STE_rate = E_dot / (m * g * V) = V_dot / g + h_dot / V
 *
 * For small perturbations around trim, we can approximate:
 *
 *   Specific Total Energy Rate:        STE_rate  = V_dot / g + h_dot / V
 *   Specific Energy Balance Rate:      SEB_rate  = h_dot / V - V_dot / g
 *
 * Equivalently, using V_dot (airspeed rate) and h_dot (climb rate):
 *
 *   STE_rate = V_dot + g * h_dot / V   (total energy rate, throttle controls this)
 *   SEB_rate = g * h_dot / V - V_dot   (energy balance, pitch controls this)
 *
 * Note: STE_rate = (kinetic_rate + potential_rate), SEB_rate = (potential_rate - kinetic_rate).
 * Adding them gives 2 * potential_rate; subtracting gives 2 * kinetic_rate.
 *
 * The controller then computes:
 *   - throttle_sp from PI control on (STE_rate_sp - STE_rate)
 *   - pitch_sp    from PI control on (SEB_rate_sp - SEB_rate)
 *
 * The "speed weight" parameter adjusts the relative importance of speed vs
 * height tracking:
 *   - spdweight = 1.0: balanced (default)
 *   - spdweight = 0.0: prioritise height (ignore speed error in SEB)
 *   - spdweight = 2.0: prioritise speed (ignore height error in SEB)
 *
 * This weighting is applied by scaling the airspeed contribution in the
 * energy balance error.
 *
 * ============================================================================
 * LATERAL CONTROLLER (SIMPLIFIED NPFG)
 * ============================================================================
 *
 * Concept
 * -------
 * The upstream path planner (mode manager) computes a lateral acceleration
 * command [m/s^2] that, if achieved, will steer the aircraft onto the
 * desired path. This lateral acceleration can come from a full NPFG
 * (Nonlinear Path Following Guidance) algorithm, an L1 controller, or
 * any other guidance law.
 *
 * This module converts that lateral acceleration into a roll angle using
 * the coordinated-turn relationship:
 *
 *   In a coordinated (no-sideslip) turn, the horizontal component of lift
 *   provides the centripetal acceleration:
 *
 *     L * sin(phi) = m * a_lat
 *     L * cos(phi) = m * g        (vertical equilibrium)
 *
 *   Dividing:
 *
 *     tan(phi) = a_lat / g
 *
 *   Therefore:
 *
 *     phi_sp = atan(a_lat / g)
 *
 *   For small angles, phi_sp is approximately a_lat / g, but we use the
 *   full atan() for accuracy at higher bank angles.
 *
 * The roll setpoint is then constrained to the configured roll limit and
 * slew-rate limited to avoid abrupt roll commands that could destabilise
 * the aircraft or cause passenger discomfort.
 *
 * Geometry
 * --------
 *           desired path
 *           ----------->
 *                  ^
 *                 / a_lat (centripetal)
 *                /
 *       aircraft *-----> V_ground
 *
 * The lateral acceleration points from the aircraft toward the desired path.
 * A positive a_lat means "turn right" (positive roll), and negative means
 * "turn left" (negative roll), consistent with the NED sign convention.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - Lambregts, A.A., "Vertical Flight Path and Speed Control Autopilot
 *     Design Using Total Energy Principles", AIAA-83-2239, 1983.
 *   - Faleiro, L. and Lambregts, A.A., "Analysis and Tuning of a Total
 *     Energy Control System", AIAA Journal of Guidance, 1999.
 *   - PX4 source: src/modules/fw_pos_control/fw_path_control.cpp
 *   - PX4 source: src/lib/tecs/TECS.hpp / TECS.cpp
 *   - Thomas, S. et al., "NPFG: A Nonlinear Path Following Guidance Law
 *     for Fixed-Wing UAVs", 2021.
 *   - Beard & McLain, "Small Unmanned Aircraft", Chapter 10 (Path Following)
 */

#pragma once

/* ------------------------------------------------------------------ */
/* Includes                                                            */
/* ------------------------------------------------------------------ */

#include "../common/math_utils.hpp"
#include "../common/uorb_msgs.hpp"

#include <cstdint>
#include <cmath>

/* ================================================================== */
/* TECS -- Total Energy Control System                                 */
/* ================================================================== */

/**
 * TECS parameter set.
 *
 * In PX4 each of these maps to a PARAM_DEFINE_FLOAT and can be changed
 * at runtime through QGC or the parameter file. We store them as plain
 * struct members with flight-tested defaults.
 *
 * Naming follows PX4 convention: FW_T_* for TECS parameters.
 */
struct TECSParams {

    /* ---- Altitude tracking ---- */

    /**
     * FW_T_ALT_TC -- Altitude time constant [s].
     *
     * Governs how aggressively the controller closes the altitude error.
     * The desired climb rate is computed as:
     *
     *   h_dot_sp = (alt_sp - alt) / alt_tc + climb_rate_ff
     *
     * A smaller value means faster altitude convergence but may exceed
     * the aircraft's climb performance, leading to airspeed droop.
     * Typical range: 3.0 to 10.0 seconds.
     */
    float alt_tc{5.0f};

    /**
     * FW_T_HRATE_FF -- Height rate feed-forward gain.
     *
     * Fraction of the commanded height rate that is directly added as a
     * feed-forward term. Improves transient response when the mode manager
     * commands a specific climb/descent rate.
     *
     * 0.0 = no feed-forward; 1.0 = full feed-forward.
     */
    float hrate_ff{0.3f};

    /* ---- Damping ---- */

    /**
     * FW_T_PTCH_DAMP -- Pitch loop damping ratio.
     *
     * Provides damping in the energy distribution (pitch) loop by feeding
     * back the measured specific energy balance rate. Higher values reduce
     * overshoot in altitude/speed trades but slow the response.
     *
     * Typical range: 0.0 to 0.5.
     */
    float ptch_damp{0.1f};

    /**
     * FW_T_THR_DAMP -- Throttle loop damping ratio.
     *
     * Provides damping in the total energy (throttle) loop by feeding
     * back the measured specific total energy rate. Higher values reduce
     * throttle oscillations but slow energy acquisition.
     *
     * Typical range: 0.0 to 0.5.
     */
    float thr_damp{0.1f};

    /* ---- Speed vs height priority ---- */

    /**
     * FW_T_SPDWEIGHT -- Speed / height priority weighting.
     *
     * Controls how the controller distributes energy between airspeed
     * and altitude tracking when the aircraft cannot achieve both
     * simultaneously (e.g., climbing at full throttle):
     *
     *   0.0 = height priority  (sacrifice speed to hold altitude)
     *   1.0 = balanced         (equal priority, default)
     *   2.0 = speed priority   (sacrifice altitude to hold speed)
     *
     * In practice:
     *   - Use 0.0 for terrain-following or obstacle avoidance (altitude is safety-critical).
     *   - Use 2.0 for stall prevention on approach (speed is safety-critical).
     *   - Use 1.0 for normal cruise.
     */
    float spdweight{1.0f};

    /* ---- Throttle constraints ---- */

    /**
     * FW_THR_MIN -- Minimum throttle [0, 1].
     *
     * Lower bound on the TECS throttle output. Set to 0 for gliders.
     * For powered aircraft, a small positive value (e.g., 0.05) can prevent
     * engine flame-out at idle.
     */
    float thr_min{0.0f};

    /**
     * FW_THR_MAX -- Maximum throttle [0, 1].
     *
     * Upper bound on the TECS throttle output. Limits engine stress and
     * temperature. Set to 1.0 for full-authority control.
     */
    float thr_max{1.0f};

    /* ---- Pitch constraints ---- */

    /**
     * FW_P_LIM_MIN -- Minimum pitch angle [rad].
     *
     * Maximum nose-down pitch TECS is allowed to command.
     * -0.3491 rad corresponds to approximately -20 degrees.
     */
    float ptch_min{-0.3491f};

    /**
     * FW_P_LIM_MAX -- Maximum pitch angle [rad].
     *
     * Maximum nose-up pitch TECS is allowed to command.
     * 0.3491 rad corresponds to approximately +20 degrees.
     */
    float ptch_max{0.3491f};

    /* ---- Climb / sink rate constraints ---- */

    /**
     * FW_T_CLMB_MAX -- Maximum climb rate [m/s].
     *
     * The height rate setpoint is clamped to this value. Should match
     * the aircraft's maximum sustainable climb rate at full throttle
     * and trim airspeed. Exceeding this will cause TECS to saturate.
     */
    float climbrate_max{5.0f};

    /**
     * FW_T_SINK_MAX -- Maximum sink rate [m/s].
     *
     * The height rate setpoint is clamped to (negative of) this value.
     * Limits descent rate to prevent excessive speed build-up or
     * structural loads in a dive.
     */
    float sinkrate_max{5.0f};

    /* ---- Airspeed parameters ---- */

    /**
     * FW_AIRSPD_TRIM -- Trim (cruise) airspeed [m/s].
     *
     * The airspeed at which the aircraft flies in level cruise with
     * nominal throttle. Used as the linearisation point for energy rate
     * calculations and as the default setpoint when no specific airspeed
     * is commanded.
     */
    float airspeed_trim{15.0f};

    /**
     * FW_AIRSPD_MIN -- Minimum airspeed [m/s].
     *
     * TECS will not command airspeeds below this value. Should be above
     * stall speed with margin. Also used to clamp the airspeed in
     * energy rate calculations to avoid division by near-zero.
     */
    float airspeed_min{10.0f};

    /**
     * FW_AIRSPD_MAX -- Maximum airspeed [m/s].
     *
     * TECS will not command airspeeds above this value. Protects against
     * structural over-speed (Vne) and excessive dynamic pressure.
     */
    float airspeed_max{20.0f};

    /* ---- Throttle time constant ---- */

    /**
     * FW_T_THR_TC -- Throttle integrator time constant [s].
     *
     * Controls how fast the throttle integrator accumulates. Larger values
     * give a more conservative (slower) throttle response, which improves
     * stability but degrades total-energy tracking bandwidth.
     *
     * The integrator gain is effectively: Ki_throttle = 1 / time_const_throt.
     */
    float time_const_throt{5.0f};

    /* ---- Vertical acceleration limit ---- */

    /**
     * FW_T_VERT_ACC -- Maximum vertical acceleration [m/s^2].
     *
     * Limits the rate of change of climb/sink rate setpoint. This prevents
     * TECS from commanding step changes in height rate that the aircraft
     * cannot follow, which would cause large transient energy errors.
     *
     * The height rate setpoint is rate-limited by this value:
     *   |d(h_dot_sp)/dt| <= vert_accel_limit
     */
    float vert_accel_limit{7.0f};
};


/**
 * TECS -- Total Energy Control System.
 *
 * This class implements the core TECS algorithm. It receives altitude and
 * airspeed setpoints along with the current state, and produces pitch and
 * throttle setpoints that are sent to the downstream attitude controller
 * and control allocator respectively.
 *
 * Internal structure:
 *   - Two PI controllers: one for total energy rate (throttle), one for
 *     energy distribution rate (pitch).
 *   - First-order filtering on measured energy rates for noise rejection.
 *   - Rate limiting on height rate setpoint for smooth transitions.
 *
 * Usage:
 *   1. Configure parameters via the public `params` member.
 *   2. Call update() each control cycle with current state and setpoints.
 *   3. Read back pitch_sp and throttle_sp from the returned pair.
 *   4. Call reset() on mode transitions or when landed.
 */
class TECS {
public:
    TECS() = default;
    ~TECS() = default;

    /** Tunable parameters (public for direct access, like PX4 convention). */
    TECSParams params;

    /**
     * Output structure returned by update().
     *
     * Contains the two primary longitudinal control outputs.
     */
    struct Output {
        float pitch_sp{0.0f};      ///< Commanded pitch angle [rad]
        float throttle_sp{0.0f};   ///< Commanded throttle [0, 1]
    };

    /**
     * Run one TECS iteration.
     *
     * This is the main entry point, called once per control cycle (~50 Hz
     * in PX4, though it can run faster).
     *
     * @param alt            Current altitude [m] (positive up, NED: -z)
     * @param alt_sp         Desired altitude [m]
     * @param airspeed       Current calibrated airspeed [m/s]
     * @param airspeed_sp    Desired calibrated airspeed [m/s]
     * @param climb_rate     Current vertical speed [m/s] (positive up)
     * @param climb_rate_sp  Desired vertical speed [m/s] (feed-forward term)
     * @param dt             Time step [s]
     * @return               Output struct with pitch_sp and throttle_sp
     */
    Output update(float alt, float alt_sp,
                  float airspeed, float airspeed_sp,
                  float climb_rate, float climb_rate_sp,
                  float dt);

    /**
     * Reset all internal state.
     *
     * Should be called:
     *   - On transition from manual to auto mode
     *   - On landing detection
     *   - When TECS was inactive for an extended period
     *
     * This zeroes the integrators and clears the differentiation state,
     * preventing stale state from causing transient spikes on the next
     * activation.
     */
    void reset();

    /* ---- Telemetry accessors ---- */

    float get_throttle_integ()  const { return _throttle_integ; }
    float get_pitch_integ()     const { return _pitch_integ; }
    float get_ste_rate()        const { return _ste_rate; }
    float get_seb_rate()        const { return _seb_rate; }
    float get_ste_rate_sp()     const { return _ste_rate_sp; }
    float get_seb_rate_sp()     const { return _seb_rate_sp; }
    float get_height_rate_sp()  const { return _height_rate_sp; }

private:

    /* ---- PI integrator state ---- */

    /**
     * Throttle integrator [dimensionless].
     *
     * Accumulates the integral of total energy rate error over time.
     * The integrator ensures zero steady-state error in total energy
     * tracking despite constant disturbances (e.g., headwind, weight change).
     */
    float _throttle_integ{0.0f};

    /**
     * Pitch integrator [rad].
     *
     * Accumulates the integral of energy balance rate error over time.
     * Ensures zero steady-state error in the speed/height distribution
     * despite constant offsets (e.g., CG shift, incorrect trim).
     */
    float _pitch_integ{0.0f};

    /* ---- Filtered energy rates (for damping) ---- */

    /**
     * Filtered specific total energy rate.
     *
     * This is the measured (actual) total energy rate, low-pass filtered
     * for use in the damping term. Without filtering, sensor noise on
     * airspeed and climb rate would cause throttle jitter.
     */
    float _ste_rate{0.0f};

    /**
     * Filtered specific energy balance rate.
     *
     * Same as above but for the pitch (energy distribution) loop.
     */
    float _seb_rate{0.0f};

    /* ---- Setpoints (stored for telemetry) ---- */

    float _ste_rate_sp{0.0f};     ///< Specific total energy rate setpoint
    float _seb_rate_sp{0.0f};     ///< Specific energy balance rate setpoint
    float _height_rate_sp{0.0f};  ///< Height rate setpoint after limiting [m/s]

    /* ---- Differentiation state ---- */

    /**
     * Previous altitude measurement [m].
     *
     * Used to numerically differentiate altitude when a direct climb rate
     * measurement is not available. Even when climb rate is available,
     * this is used for rate-of-change limiting.
     */
    float _last_alt{0.0f};

    /**
     * Previous airspeed measurement [m/s].
     *
     * Used to compute the airspeed rate (V_dot) by finite differencing.
     * The PX4 airspeed sensor does not provide a rate output, so
     * differentiation is necessary.
     */
    float _last_airspeed{0.0f};

    /**
     * Previous height rate setpoint [m/s].
     *
     * Used to apply vertical acceleration limiting: the change in
     * height_rate_sp between consecutive cycles is bounded by
     * vert_accel_limit * dt.
     */
    float _last_height_rate_sp{0.0f};

    /**
     * Flag indicating whether the TECS has been initialised with
     * at least one sample. On the first call, differentiation and
     * filtering cannot be performed (no "previous" value exists).
     */
    bool _initialized{false};
};


/* ================================================================== */
/* Lateral controller parameters                                       */
/* ================================================================== */

/**
 * Parameters for the lateral (roll from lateral acceleration) controller.
 */
struct LateralParams {

    /**
     * FW_R_LIM -- Maximum roll angle [rad].
     *
     * The roll setpoint is constrained to [-roll_limit, +roll_limit].
     * 0.7854 rad corresponds to approximately 45 degrees, which is a
     * common limit for small fixed-wing UAVs. Larger values allow
     * tighter turns but increase stall risk (the stall speed increases
     * as 1/sqrt(cos(phi))).
     */
    float roll_limit{0.7854f};

    /**
     * FW_R_SLEW -- Maximum roll rate for setpoint slew limiting [rad/s].
     *
     * Limits how fast the roll setpoint can change between consecutive
     * control cycles. This is NOT a rate limit on the aircraft's actual
     * roll rate (that is handled by the attitude and rate controllers
     * downstream), but rather a limit on how quickly this module changes
     * its COMMAND.
     *
     * Slew rate limiting prevents step changes in the roll setpoint that
     * could excite structural modes, cause passenger discomfort, or
     * lead to actuator saturation in the downstream loops.
     *
     * 1.5708 rad/s corresponds to approximately 90 deg/s.
     */
    float roll_slew_rate{1.5708f};
};


/* ================================================================== */
/* Main class: FwLateralLongitudinalControl                            */
/* ================================================================== */

/**
 * FwLateralLongitudinalControl -- the outermost guidance controller.
 *
 * Combines TECS (longitudinal) and lateral-acceleration-to-roll (lateral)
 * control into a single module that produces attitude and thrust setpoints.
 *
 * Lifecycle (mirrors a PX4 WorkItem):
 *   1.  Constructor: allocate, set defaults
 *   2.  init():      subscribe to uORB topics (simulated here)
 *   3.  Run():       called each cycle (~50 Hz), does the real work
 *
 * Inputs (via uORB subscriptions, simulated as struct members):
 *   - fixed_wing_lateral_setpoint_s      (lateral acceleration / course command)
 *   - fixed_wing_longitudinal_setpoint_s (altitude / airspeed / climb rate)
 *   - vehicle_local_position_s           (current position, velocity, heading)
 *   - airspeed_validated_s               (calibrated and true airspeed)
 *   - wind_s                             (wind estimate from EKF)
 *   - vehicle_attitude_s                 (current attitude for yaw extraction)
 *
 * Outputs (via uORB publication):
 *   - vehicle_attitude_setpoint_s        (desired roll, pitch, yaw, thrust)
 */
class FwLateralLongitudinalControl {
public:
    FwLateralLongitudinalControl() = default;
    ~FwLateralLongitudinalControl() = default;

    /**
     * Initialise the module.
     *
     * Sets up internal state and validates parameters. In PX4 this
     * would also register uORB subscriptions.
     *
     * @return true on success.
     */
    bool init();

    /**
     * Execute one control cycle.
     *
     * This is the main function called at ~50 Hz. It reads the latest
     * setpoints and vehicle state, runs the lateral and TECS controllers,
     * and publishes the resulting attitude/thrust setpoint.
     *
     * @param timestamp_now  Current monotonic time [us].
     */
    void Run(uint64_t timestamp_now);

    /* ---- Input setters (simulate uORB subscriptions) ---- */

    void setLateralSetpoint(const uorb::fixed_wing_lateral_setpoint_s& msg) {
        _lateral_setpoint = msg;
    }
    void setLongitudinalSetpoint(const uorb::fixed_wing_longitudinal_setpoint_s& msg) {
        _longitudinal_setpoint = msg;
    }
    void setLocalPosition(const uorb::vehicle_local_position_s& msg) {
        _local_position = msg;
    }
    void setAirspeed(const uorb::airspeed_validated_s& msg) {
        _airspeed_validated = msg;
    }
    void setWind(const uorb::wind_s& msg) {
        _wind = msg;
    }
    void setAttitude(const uorb::vehicle_attitude_s& msg) {
        _attitude = msg;
    }

    /* ---- Output getters (simulate uORB publication) ---- */

    const uorb::vehicle_attitude_setpoint_s& getAttitudeSetpoint() const {
        return _att_sp_pub;
    }

    /* ---- Sub-controller access (for testing / telemetry) ---- */

    TECS&       tecs()       { return _tecs; }
    const TECS& tecs() const { return _tecs; }

    /** Lateral controller parameters (public for direct tuning). */
    LateralParams lateral_params;

    /* ---- Telemetry accessors ---- */

    float get_roll_setpoint()  const { return _roll_sp; }
    float get_pitch_setpoint() const { return _pitch_sp; }
    float get_throttle()       const { return _throttle_sp; }

private:

    /* ---- Internal methods ---- */

    /**
     * Compute the roll setpoint from lateral acceleration.
     *
     * Uses the coordinated-turn equation:
     *   phi_sp = atan(a_lat / g)
     *
     * The result is constrained to the roll limit and slew-rate limited
     * to prevent abrupt roll commands.
     *
     * @param lateral_accel  Desired lateral acceleration [m/s^2]
     * @param dt             Time step [s]
     * @return               Roll setpoint [rad]
     */
    float computeRollSetpoint(float lateral_accel, float dt);

    /**
     * Determine the effective calibrated airspeed to use.
     *
     * Prefers the validated airspeed measurement. Falls back to
     * ground speed (from local position) as a rough estimate when
     * no airspeed sensor is available. Final fallback is trim airspeed.
     *
     * @return Effective CAS [m/s], clamped to [airspeed_min, airspeed_max].
     */
    float getEffectiveAirspeed() const;

    /**
     * Determine the current climb rate.
     *
     * Uses the vertical velocity from the local position estimate.
     * In NED frame, z is down, so climb rate = -vz.
     *
     * @return Climb rate [m/s] (positive up).
     */
    float getClimbRate() const;

    /**
     * Get the current altitude.
     *
     * In NED frame, altitude (positive up) = -z.
     *
     * @return Altitude [m] (positive up).
     */
    float getAltitude() const;

    /* ---- Sub-controllers ---- */

    TECS _tecs;   ///< Total Energy Control System instance

    /* ---- uORB input mirrors ---- */

    uorb::fixed_wing_lateral_setpoint_s      _lateral_setpoint{};
    uorb::fixed_wing_longitudinal_setpoint_s _longitudinal_setpoint{};
    uorb::vehicle_local_position_s           _local_position{};
    uorb::airspeed_validated_s               _airspeed_validated{};
    uorb::wind_s                             _wind{};
    uorb::vehicle_attitude_s                 _attitude{};

    /* ---- uORB output storage ---- */

    uorb::vehicle_attitude_setpoint_s _att_sp_pub{};

    /* ---- Internal state ---- */

    uint64_t _last_run_timestamp{0};   ///< Timestamp of previous Run() [us]

    float _roll_sp{0.0f};             ///< Current roll setpoint [rad]
    float _pitch_sp{0.0f};            ///< Current pitch setpoint [rad] (from TECS)
    float _throttle_sp{0.0f};         ///< Current throttle setpoint [0,1] (from TECS)
    float _last_roll_sp{0.0f};        ///< Previous roll setpoint for slew limiting [rad]

    /**
     * Flag to know if this is the very first Run() call.
     * Used to avoid a huge dt on the first iteration.
     */
    bool _first_run{true};

    /* ---- Constants ---- */

    /**
     * Maximum allowed dt [s].
     *
     * If the scheduler is delayed, we clamp dt to avoid integrator
     * wind-up from a single large step. 5x the nominal 20 ms period.
     */
    static constexpr float MAX_DT = 0.10f;  // 100 ms

    /**
     * Minimum dt [s] to consider a valid control cycle.
     *
     * Prevents division-by-zero if two callbacks fire in quick succession.
     */
    static constexpr float MIN_DT = 0.001f;  // 1 ms
};
