/**
 * @file FixedwingRateControl.hpp
 *
 * Fixed-wing rate (inner-loop) controller -- class declaration.
 *
 * =========================================================================
 * ARCHITECTURE OVERVIEW
 * =========================================================================
 *
 * This is the INNERMOST loop of the fixed-wing control cascade:
 *
 *   Path planner  -->  Attitude controller  -->  **Rate controller**  -->  Control allocator
 *                       (outer loop)              (this module)             (actuators)
 *
 * The rate controller receives angular-rate setpoints in body frame
 * (roll rate p, pitch rate q, yaw rate r) from the attitude controller
 * and computes normalised torque commands that are sent to the control
 * allocator, which then maps them to individual surface deflections.
 *
 * The core computation each cycle:
 *
 *   1.  angular_accel_sp = PID(rates_measured, rates_setpoint, dt)
 *   2.  torque = angular_accel_sp  *  airspeed_scaling^2  *  gain_compression
 *   3.  torque += trim  (airspeed-scheduled)
 *   4.  Publish torque & thrust setpoints
 *
 * In addition to the stabilised flight-controller modes, this module also
 * handles:
 *   - ACRO mode   (stick -> rate setpoint)
 *   - MANUAL mode (stick -> direct actuator, bypassing PID entirely)
 *   - VTOL tailsitter frame transforms
 *   - Battery voltage compensation on thrust
 *   - Roll-to-yaw adverse-yaw feed-forward
 *   - Anti-windup via control-allocator saturation feedback
 *
 * =========================================================================
 * AIRSPEED SCALING
 * =========================================================================
 *
 * Aerodynamic control surfaces produce moments proportional to dynamic
 * pressure q_bar = 0.5 * rho * V^2.  When the autopilot commands a
 * normalised deflection, the resulting angular acceleration depends on
 * V^2.  To keep the *closed-loop bandwidth* constant across the flight
 * envelope we must cancel this V^2 dependency:
 *
 *   airspeed_scaling = V_trim / V_actual          (clamped >= V_min)
 *
 *   torque_cmd = angular_accel_sp * airspeed_scaling^2
 *
 * At trim speed the scaling is 1.0; faster => smaller commands, slower
 * => larger commands.  Feed-forward gains are inversely scaled because
 * they relate rate_sp to deflection, not to angular acceleration.
 *
 * =========================================================================
 * TRIM SCHEDULING
 * =========================================================================
 *
 * Even in steady flight, non-zero surface deflection is needed to
 * compensate for CG offset, propeller torque, etc.  The nominal trim
 * values (FW_MAN_R_TRIM, FW_MAN_P_TRIM, FW_MAN_Y_TRIM) correspond
 * to the trim airspeed.  At different airspeeds the required trim
 * changes, so we interpolate:
 *
 *   V < V_trim : trim = nominal + lerp(dtrim_vmin, 0, (V-V_min)/(V_trim-V_min))
 *   V > V_trim : trim = nominal + lerp(0, dtrim_vmax, (V-V_trim)/(V_max-V_trim))
 *
 * The interpolation deltas (FW_DTRIM_R_VMIN etc.) are typically found
 * through flight testing.
 *
 * =========================================================================
 * REFERENCES
 * =========================================================================
 *
 *   PX4 source: src/modules/fw_rate_control/FixedwingRateControl.hpp/cpp
 *   PX4 parameter reference: https://docs.px4.io/main/en/advanced_config/parameter_reference.html
 */

#pragma once

/* ------------------------------------------------------------------ */
/* Includes                                                            */
/* ------------------------------------------------------------------ */

#include "../common/math_utils.hpp"
#include "../common/uorb_msgs.hpp"
#include "../common/rate_control.hpp"

#include <cstdint>
#include <cmath>

/* ================================================================== */
/* Parameter structures                                                */
/* ================================================================== */

/**
 * All tunable parameters for the fixed-wing rate controller.
 *
 * In PX4, each of these is a `PARAM_DEFINE_FLOAT / INT32` that can be
 * changed at runtime via QGC or the parameter file.  We store them as
 * plain struct members with sensible defaults so the module works
 * out-of-the-box in simulation.
 *
 * Naming follows PX4 conventions:
 *   FW_RR_*  = roll rate
 *   FW_PR_*  = pitch rate
 *   FW_YR_*  = yaw rate
 *   FW_AIRSPD_* = airspeed
 */
struct FwRateControlParams {

    /* ---- Roll-rate PID ---- */

    /**
     * FW_RR_P -- Roll-rate proportional gain.
     *
     * Higher values make the aircraft respond more aggressively to
     * roll-rate errors, but too high causes oscillation on the roll axis.
     * Units: rad/s^2 per rad/s of error (dimensionless in practice
     *        because the output is normalised torque).
     */
    float fw_rr_p{0.05f};

    /**
     * FW_RR_I -- Roll-rate integral gain.
     *
     * Eliminates steady-state roll-rate error (e.g., from CG offset or
     * asymmetric drag).  Must be small to avoid slow oscillations.
     */
    float fw_rr_i{0.1f};

    /**
     * FW_RR_D -- Roll-rate derivative gain.
     *
     * Provides damping.  The RateControl library uses derivative-on-
     * measurement to avoid setpoint kicks.
     */
    float fw_rr_d{0.003f};

    /** FW_RR_FF -- Roll-rate feed-forward gain. */
    float fw_rr_ff{0.5f};

    /** FW_RR_IMAX -- Roll-rate integrator limit (absolute value). */
    float fw_rr_imax{0.4f};

    /* ---- Pitch-rate PID ---- */
    float fw_pr_p{0.08f};
    float fw_pr_i{0.1f};
    float fw_pr_d{0.003f};
    float fw_pr_ff{0.5f};
    float fw_pr_imax{0.4f};

    /* ---- Yaw-rate PID ---- */
    float fw_yr_p{0.05f};
    float fw_yr_i{0.1f};
    float fw_yr_d{0.0f};     ///< Yaw D is often zero to avoid rudder flutter
    float fw_yr_ff{0.3f};
    float fw_yr_imax{0.2f};

    /* ---- Airspeed parameters ---- */

    /**
     * FW_AIRSPD_TRIM -- Trim (cruise) airspeed [m/s].
     *
     * The airspeed at which the nominal PID gains produce the desired
     * control authority.  Airspeed scaling is unity at this speed.
     */
    float fw_airspd_trim{15.0f};

    /**
     * FW_AIRSPD_MIN -- Minimum flyable airspeed [m/s].
     *
     * Below this, airspeed scaling is clamped to avoid extremely large
     * control outputs.  Should be above stall speed with margin.
     */
    float fw_airspd_min{10.0f};

    /**
     * FW_AIRSPD_MAX -- Maximum operational airspeed [m/s].
     *
     * Used for scheduling and as upper clamp for scaling.
     */
    float fw_airspd_max{20.0f};

    /**
     * FW_ARSP_SCALE_EN -- Enable airspeed scaling (0 or 1).
     *
     * When disabled (0), airspeed_scaling is held at 1.0.  Useful when
     * flying without an airspeed sensor (but not recommended).
     */
    int32_t fw_arsp_scale_en{1};

    /* ---- Trim offsets ---- */

    /**
     * FW_MAN_R_TRIM / P_TRIM / Y_TRIM -- Nominal manual-mode trim.
     *
     * The normalised actuator deflection required for straight-and-level
     * flight at trim airspeed.  Typically found by trimming the aircraft
     * manually and reading the stick position.
     */
    float fw_man_r_trim{0.0f};
    float fw_man_p_trim{0.0f};
    float fw_man_y_trim{0.0f};

    /* ---- Airspeed-dependent trim deltas ---- */

    /**
     * FW_DTRIM_R_VMIN / VMAX -- Roll trim delta at min / max airspeed.
     *
     * Added to the nominal trim when airspeed departs from trim speed.
     * For example, a positive FW_DTRIM_R_VMIN means the aircraft needs
     * more right-aileron trim at low speed (perhaps due to torque effect
     * from the propeller at high angle of attack).
     */
    float fw_dtrim_r_vmin{0.0f};
    float fw_dtrim_r_vmax{0.0f};
    float fw_dtrim_p_vmin{0.0f};
    float fw_dtrim_p_vmax{0.0f};
    float fw_dtrim_y_vmin{0.0f};
    float fw_dtrim_y_vmax{0.0f};

    /* ---- Roll-to-yaw feed-forward ---- */

    /**
     * FW_RLL_TO_YAW_FF -- Roll-to-yaw coupling feed-forward.
     *
     * When the aircraft rolls, differential lift on the wings creates
     * adverse yaw (yawing away from the turn direction).  This gain
     * pre-emptively applies rudder proportional to the roll torque
     * command to counteract that adverse yaw before the yaw-rate PID
     * has to react.
     *
     * Typical range: 0.0 (disabled) to 0.3.
     */
    float fw_rll_to_yaw_ff{0.0f};

    /* ---- ACRO mode rates ---- */

    /**
     * Maximum body rate in ACRO mode [rad/s].
     * Full stick deflection maps to these rates.
     */
    float fw_acro_x_max{1.5708f};  ///< ~90 deg/s roll
    float fw_acro_y_max{1.5708f};  ///< ~90 deg/s pitch
    float fw_acro_z_max{0.7854f};  ///< ~45 deg/s yaw

    /* ---- Flaps / spoilers ---- */
    float fw_flaps_scl{1.0f};      ///< Flap deflection scale [0,1]
    float fw_spoilers_man{0.0f};   ///< Manual spoiler deployment scale [0,1]

    /* ---- Battery compensation ---- */

    /**
     * FW_BAT_SCALE_EN -- Enable battery voltage thrust compensation.
     *
     * When the battery voltage drops, the available thrust decreases.
     * Enabling this multiplies the thrust command by the battery scale
     * factor to maintain consistent performance across the battery
     * discharge curve.
     */
    int32_t fw_bat_scale_en{0};

    /* ---- VTOL ---- */

    /**
     * When true, the vehicle is a VTOL tailsitter, meaning the body
     * axes in hover are rotated 90 degrees relative to fixed-wing flight.
     * The rate controller must transform angular velocities and torque
     * commands when entering / leaving the fixed-wing control frame.
     */
    bool is_vtol_tailsitter{false};
};


/* ================================================================== */
/* Main class                                                          */
/* ================================================================== */

/**
 * FixedwingRateControl -- the inner-loop rate controller.
 *
 * Lifecycle (mirrors a PX4 WorkItem):
 *   1.  Constructor: allocate, set defaults
 *   2.  init():      subscribe to uORB topics (simulated here)
 *   3.  Run():       called each cycle (~250 Hz), does the real work
 *
 * Inputs (via uORB subscriptions simulated as struct members):
 *   - vehicle_angular_velocity_s   (gyro)
 *   - vehicle_rates_setpoint_s     (from attitude controller)
 *   - vehicle_attitude_s           (current attitude, used for trim)
 *   - airspeed_validated_s         (airspeed)
 *   - vehicle_status_s             (flight mode, VTOL flags)
 *   - manual_control_setpoint_s    (sticks for ACRO / MANUAL)
 *   - battery_status_s             (voltage compensation)
 *   - control_allocator_status_s   (saturation feedback)
 *   - vehicle_local_position_s     (ground speed for no-airspeed fallback)
 *
 * Outputs (via uORB publications):
 *   - vehicle_torque_setpoint_s
 *   - vehicle_thrust_setpoint_s
 *   - vehicle_rates_setpoint_s  (published back in ACRO mode so other
 *                                 modules can see the generated rate_sp)
 */
class FixedwingRateControl {
public:
    FixedwingRateControl();
    ~FixedwingRateControl() = default;

    /**
     * Initialise subscriptions and internal state.
     * @return true on success.
     */
    bool init();

    /**
     * Execute one control cycle.
     *
     * This is the function that runs at ~250 Hz (driven by the gyro
     * topic in PX4).  It reads all inputs, computes the PID, applies
     * scaling and trim, and publishes the results.
     *
     * @param timestamp_now  Current monotonic time [us].
     */
    void Run(uint64_t timestamp_now);

    /* ---- Input setters (simulate uORB subscriptions) ---- */

    void setAngularVelocity(const uorb::vehicle_angular_velocity_s& msg) { _angular_velocity = msg; }
    void setRatesSetpoint(const uorb::vehicle_rates_setpoint_s& msg)     { _rates_setpoint = msg; }
    void setAttitude(const uorb::vehicle_attitude_s& msg)                 { _attitude = msg; }
    void setAirspeed(const uorb::airspeed_validated_s& msg)               { _airspeed_validated = msg; }
    void setVehicleStatus(const uorb::vehicle_status_s& msg)              { _vehicle_status = msg; }
    void setManualControl(const uorb::manual_control_setpoint_s& msg)     { _manual_control = msg; }
    void setBatteryStatus(const uorb::battery_status_s& msg)              { _battery_status = msg; }
    void setAllocatorStatus(const uorb::control_allocator_status_s& msg)  { _control_allocator_status = msg; }
    void setLocalPosition(const uorb::vehicle_local_position_s& msg)      { _local_position = msg; }
    void setAutotuneStatus(const uorb::autotune_attitude_control_status_s& msg) { _autotune_status = msg; }

    /* ---- Output getters (simulate uORB publications) ---- */

    const uorb::vehicle_torque_setpoint_s& getTorqueSetpoint() const { return _torque_sp_pub; }
    const uorb::vehicle_thrust_setpoint_s& getThrustSetpoint() const { return _thrust_sp_pub; }
    const uorb::vehicle_rates_setpoint_s&  getPublishedRatesSetpoint() const { return _rates_setpoint_pub; }

    /* ---- Parameters (public so tests can tweak) ---- */
    FwRateControlParams params;

    /* ---- Telemetry accessors ---- */

    /** Current airspeed scaling factor (for logging / debugging). */
    float getAirspeedScaling() const { return _airspeed_scaling; }

    /** Get the PID integrator state for telemetry. */
    math::Vector3f getIntegrator() const { return _rate_control.getIntegrator(); }

private:

    /* ---- Internal methods ---- */

    /**
     * Apply the PID gains from the parameter struct to the RateControl
     * library instance.  Called once at init and whenever parameters change.
     */
    void updatePidGains();

    /**
     * Compute the airspeed scaling factor.
     *
     * @param airspeed_cas  Calibrated airspeed [m/s] (or estimate).
     * @return  scaling factor (V_trim / V_actual), clamped.
     */
    float computeAirspeedScaling(float airspeed_cas) const;

    /**
     * Compute airspeed-scheduled trim values.
     *
     * @param airspeed  Current calibrated airspeed [m/s].
     * @return  3-axis trim vector (normalised actuator units).
     */
    math::Vector3f computeAirspeedTrim(float airspeed) const;

    /**
     * Determine the effective calibrated airspeed to use.
     *
     * If a validated airspeed measurement is available and recent, use it.
     * Otherwise, fall back to ground speed from the EKF (which ignores
     * wind but is better than nothing), or finally to the trim airspeed
     * as a last resort.
     *
     * @return  Effective CAS [m/s].
     */
    float getEffectiveAirspeed() const;

    /**
     * Update saturation flags on the PID integrator from allocator feedback.
     *
     * The control allocator reports `unallocated_torque` -- the difference
     * between what was demanded and what could be achieved.  If the
     * unallocated torque has the same sign as the demanded torque on a
     * given axis, that axis is saturated in that direction.
     */
    void updateAntiWindup();

    /* ---- RateControl library ---- */

    control::RateControl _rate_control;   ///< The 3-axis PID engine

    /* ---- uORB input mirrors ---- */

    uorb::vehicle_angular_velocity_s        _angular_velocity{};
    uorb::vehicle_rates_setpoint_s          _rates_setpoint{};
    uorb::vehicle_attitude_s                _attitude{};
    uorb::airspeed_validated_s              _airspeed_validated{};
    uorb::vehicle_status_s                  _vehicle_status{};
    uorb::manual_control_setpoint_s         _manual_control{};
    uorb::battery_status_s                  _battery_status{};
    uorb::control_allocator_status_s        _control_allocator_status{};
    uorb::vehicle_local_position_s          _local_position{};
    uorb::autotune_attitude_control_status_s _autotune_status{};

    /* ---- uORB output storage ---- */

    uorb::vehicle_torque_setpoint_s  _torque_sp_pub{};
    uorb::vehicle_thrust_setpoint_s  _thrust_sp_pub{};
    uorb::vehicle_rates_setpoint_s   _rates_setpoint_pub{};

    /* ---- Internal state ---- */

    uint64_t _last_run_timestamp{0};   ///< Timestamp of previous Run() [us]
    float    _airspeed_scaling{1.0f};  ///< Current V_trim / V_actual factor

    /**
     * Flap and spoiler setpoint values, captured from manual control
     * and scaled by parameters.  These are published alongside the
     * main torque commands so the allocator can drive flap / spoiler
     * servos.
     */
    float _flaps_setpoint{0.0f};
    float _spoiler_setpoint{0.0f};

    /**
     * Flag to know if this is the very first Run() call.
     * Used to avoid a huge dt on the first iteration.
     */
    bool _first_run{true};

    /* ---- Constants ---- */

    /**
     * Maximum allowed dt [s].
     *
     * If the scheduler is delayed (CPU overload, etc.), we clamp dt
     * to avoid the integrator accumulating a huge step.  5x the
     * nominal 4 ms period is generous enough for normal jitter.
     */
    static constexpr float MAX_DT = 0.02f;  // 20 ms

    /**
     * Minimum dt [s] to consider a valid control cycle.
     *
     * Prevents division-by-zero and nonsensical gains if two callbacks
     * fire in quick succession.
     */
    static constexpr float MIN_DT = 0.0002f;  // 0.2 ms
};
