/**
 * @file FixedwingRateControl.cpp
 *
 * Implementation of the fixed-wing rate (inner-loop) controller.
 *
 * See FixedwingRateControl.hpp for the full architecture overview, airspeed
 * scaling derivation, and trim scheduling description.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_rate_control/FixedwingRateControl.cpp
 *
 * =========================================================================
 * CONTROL FLOW SUMMARY (per Run() call)
 * =========================================================================
 *
 *   1.  Compute dt from timestamps, clamp to [MIN_DT, MAX_DT]
 *   2.  Determine effective airspeed (sensor -> groundspeed -> trim fallback)
 *   3.  Compute airspeed scaling factor: V_trim / V_actual
 *   4.  [VTOL tailsitter] Transform input body rates to fixed-wing frame
 *   5.  Update anti-windup flags from control allocator feedback
 *   6.  Update PID gains (in case params changed at runtime)
 *   7.  Run 3-axis PID: angular_accel_sp = PID(rates, rate_sp, dt)
 *   8.  Apply airspeed scaling: torque = angular_accel_sp * scaling^2
 *   9.  Scale feed-forward by 1/airspeed_scaling (done before PID step)
 *  10.  Add airspeed-scheduled trim offsets
 *  11.  Constrain torque commands to [-1, +1]
 *  12.  Apply roll-to-yaw adverse-yaw feed-forward
 *  13.  Set thrust from rate setpoint; apply battery compensation
 *  14.  [VTOL tailsitter] Reverse-transform output torques
 *  15.  Publish torque and thrust setpoints
 *
 * MANUAL and ACRO modes override parts of this pipeline:
 *   - MANUAL: bypass PID entirely; sticks map directly to torque/thrust
 *   - ACRO:   sticks map to rate setpoint, then PID runs normally
 *
 * =========================================================================
 * NAV STATE CONVENTIONS
 * =========================================================================
 *
 * PX4 defines nav_state values in vehicle_status_s.  We use the standard
 * values for the modes this controller needs to distinguish:
 *
 *   0  = MANUAL (direct stick-to-surface)
 *   5  = ACRO   (stick-to-rate, PID-stabilised)
 *
 * All other nav_state values are treated as "stabilised" modes, where the
 * rate setpoint comes from the upstream attitude controller.
 */

#include "FixedwingRateControl.hpp"

#include <cmath>
#include <algorithm>

/* ------------------------------------------------------------------ */
/* PX4 nav_state constants (subset needed by rate controller)          */
/* ------------------------------------------------------------------ */

static constexpr uint8_t NAVIGATION_STATE_MANUAL = 0;
static constexpr uint8_t NAVIGATION_STATE_ACRO   = 5;

/* ------------------------------------------------------------------ */
/* Airspeed staleness threshold                                        */
/* ------------------------------------------------------------------ */

/**
 * If the airspeed measurement is older than this threshold, we consider
 * it stale and fall back to the ground-speed estimate or trim airspeed.
 * 1 second is generous -- at 250 Hz we would have missed ~250 updates.
 */
static constexpr uint64_t AIRSPEED_TIMEOUT_US = 1'000'000;  // 1 second


/* ======================================================================
 * Constructor
 * ====================================================================== */

FixedwingRateControl::FixedwingRateControl()
{
    /*
     * Zero-initialise all message structs.
     *
     * The default member initialisers in the header already set most
     * fields to zero / identity.  We explicitly mark _first_run = true
     * so that the very first call to Run() knows to skip the PID
     * (there is no valid dt until we have two timestamps).
     */
    _first_run          = true;
    _last_run_timestamp = 0;
    _airspeed_scaling   = 1.0f;
    _flaps_setpoint     = 0.0f;
    _spoiler_setpoint   = 0.0f;
}


/* ======================================================================
 * init()
 * ====================================================================== */

bool FixedwingRateControl::init()
{
    /*
     * Apply the default PID gains from the parameter struct to the
     * RateControl library instance.  In PX4 this would also set up
     * uORB subscriptions and schedule the WorkItem callback.  Here
     * we just need the gains.
     */
    updatePidGains();
    return true;
}


/* ======================================================================
 * Run()  --  main control cycle, called at ~250 Hz
 * ====================================================================== */

void FixedwingRateControl::Run(uint64_t timestamp_now)
{
    /* ----------------------------------------------------------------
     * STEP 0: Compute the time step dt [seconds].
     * ----------------------------------------------------------------
     * PX4 timestamps are in microseconds.  We convert the delta to
     * seconds and clamp it to a safe range:
     *
     *   MIN_DT (0.2 ms) -- prevents near-zero dt which would cause
     *     extremely large derivative terms and integrator steps.
     *
     *   MAX_DT (20 ms)  -- prevents huge integrator accumulation if
     *     the scheduler is delayed (CPU overload, sensor dropout, etc.).
     *     20 ms is 5x the nominal 4 ms period, generous enough for
     *     normal scheduling jitter.
     */
    const float dt_us = static_cast<float>(timestamp_now - _last_run_timestamp);
    float dt = dt_us * 1e-6f;  // microseconds -> seconds
    dt = math::constrain(dt, MIN_DT, MAX_DT);

    /* ----------------------------------------------------------------
     * STEP 0b: First-run guard.
     * ----------------------------------------------------------------
     * On the very first call we have no previous timestamp, so dt is
     * meaningless (could be billions of microseconds since boot).
     * We record the timestamp and return without running the PID.
     * This ensures the derivative term and integrator start cleanly.
     */
    if (_first_run) {
        _last_run_timestamp = timestamp_now;
        _first_run = false;
        return;
    }

    /* ----------------------------------------------------------------
     * STEP 1: Determine effective airspeed and compute scaling.
     * ----------------------------------------------------------------
     * Aerodynamic surfaces generate moments proportional to dynamic
     * pressure (q_bar = 0.5 * rho * V^2).  To keep the closed-loop
     * bandwidth constant across the flight envelope, we scale the
     * torque commands by (V_trim / V_actual)^2.
     *
     * See header for full derivation.
     */
    const float effective_airspeed = getEffectiveAirspeed();
    _airspeed_scaling = computeAirspeedScaling(effective_airspeed);

    /* ----------------------------------------------------------------
     * STEP 2: Read angular velocity (body rates from gyro).
     * ----------------------------------------------------------------
     * In the standard fixed-wing body frame:
     *   xyz[0] = p  (roll rate)
     *   xyz[1] = q  (pitch rate)
     *   xyz[2] = r  (yaw rate)
     */
    const uorb::vehicle_angular_velocity_s& angular_velocity = _angular_velocity;

    /* ----------------------------------------------------------------
     * STEP 2b: VTOL tailsitter frame transform on INPUT rates.
     * ----------------------------------------------------------------
     * A tailsitter hovers nose-up, so when transitioning to fixed-wing
     * mode, the body axes are rotated 90 degrees about the pitch axis
     * relative to a conventional fixed-wing:
     *
     *   Tailsitter hover:     x = up,   y = right, z = forward
     *   Fixed-wing convention: x = fwd, y = right, z = down
     *
     * The transform for angular rates from the tailsitter's physical
     * body frame to the "virtual" fixed-wing frame is:
     *
     *   p_fw  = -r_body    (FW roll rate = negative body yaw rate)
     *   q_fw  =  q_body    (pitch rate unchanged)
     *   r_fw  =  p_body    (FW yaw rate = body roll rate)
     *
     * This must be done BEFORE the PID, which is tuned in the
     * fixed-wing frame, and reversed on the output torques.
     */
    math::Vector3f rates;

    if (params.is_vtol_tailsitter) {
        rates = math::Vector3f(
            -angular_velocity.xyz[2],   // p_fw = -r_body
             angular_velocity.xyz[1],   // q_fw =  q_body
             angular_velocity.xyz[0]    // r_fw =  p_body
        );
    } else {
        rates = math::Vector3f(
            angular_velocity.xyz[0],
            angular_velocity.xyz[1],
            angular_velocity.xyz[2]
        );
    }

    /* ----------------------------------------------------------------
     * STEP 3: Build the rate setpoint vector.
     * ----------------------------------------------------------------
     * In stabilised modes (attitude controller upstream), the rate
     * setpoint comes from _rates_setpoint, which was published by
     * the attitude controller.
     *
     * In ACRO mode, we override this below with stick * max_rate.
     * In MANUAL mode, we skip the PID entirely.
     */
    math::Vector3f rate_sp(
        _rates_setpoint.roll,
        _rates_setpoint.pitch,
        _rates_setpoint.yaw
    );

    /* ----------------------------------------------------------------
     * STEP 4: Mode-specific handling.
     * ---------------------------------------------------------------- */
    const uint8_t nav_state = _vehicle_status.nav_state;

    if (nav_state == NAVIGATION_STATE_MANUAL) {
        /* =============================================================
         * MANUAL MODE -- bypass PID entirely.
         * =============================================================
         *
         * In manual mode, the pilot's stick positions map directly to
         * normalised torque commands.  No PID, no rate setpoint, no
         * airspeed scaling (the pilot is "the controller").
         *
         * The trim offsets are ADDED so that the aircraft flies straight
         * when the sticks are centred.  This is the primary mechanism
         * for hands-off trim in manual flight.
         *
         * Flaps and spoilers are also set from manual control inputs.
         */

        /* Compute airspeed-scheduled trim for the current speed */
        const math::Vector3f trim = computeAirspeedTrim(effective_airspeed);

        /* Map sticks directly to torque, adding trim offset */
        _torque_sp_pub.xyz[0] = math::constrain(
            _manual_control.roll + trim.x(), -1.0f, 1.0f);
        _torque_sp_pub.xyz[1] = math::constrain(
            -_manual_control.pitch + trim.y(), -1.0f, 1.0f);
        _torque_sp_pub.xyz[2] = math::constrain(
            _manual_control.yaw + trim.z(), -1.0f, 1.0f);

        _torque_sp_pub.timestamp = timestamp_now;

        /*
         * Thrust in manual mode comes directly from the throttle stick.
         * PX4 convention: manual_control.throttle is in [-1, 1] but for
         * fixed-wing forward thrust we typically use [0, 1].
         * We pass it through as-is; the allocator handles clamping.
         */
        _thrust_sp_pub.xyz[0] = _manual_control.throttle;
        _thrust_sp_pub.xyz[1] = 0.0f;
        _thrust_sp_pub.xyz[2] = 0.0f;
        _thrust_sp_pub.timestamp = timestamp_now;

        /* Flaps: manual control flaps input scaled by parameter */
        _flaps_setpoint  = _manual_control.flaps * params.fw_flaps_scl;

        /* Spoilers: manual aux1 channel scaled by parameter */
        _spoiler_setpoint = _manual_control.aux1 * params.fw_spoilers_man;

        _last_run_timestamp = timestamp_now;
        return;
    }

    /* ---- ACRO mode: compute rate setpoint from sticks ---- */
    if (nav_state == NAVIGATION_STATE_ACRO) {
        /*
         * ACRO MODE -- sticks command angular rates.
         * -------------------------------------------
         * Full stick deflection (+/-1) maps to the maximum rate configured
         * by the fw_acro_{x,y,z}_max parameters.  This gives the pilot
         * direct rate control, useful for aerobatic flight and tuning.
         *
         * The computed rate setpoint is then passed to the PID below,
         * which stabilises the rate (unlike MANUAL, where PID is bypassed).
         */
        rate_sp = math::Vector3f(
            _manual_control.roll  * params.fw_acro_x_max,
            -_manual_control.pitch * params.fw_acro_y_max,
            _manual_control.yaw   * params.fw_acro_z_max
        );

        /*
         * Publish the generated rate setpoint so that other modules
         * (e.g., logging, ground station) can see what rate was
         * commanded in ACRO mode.
         */
        _rates_setpoint_pub.timestamp = timestamp_now;
        _rates_setpoint_pub.roll      = rate_sp.x();
        _rates_setpoint_pub.pitch     = rate_sp.y();
        _rates_setpoint_pub.yaw       = rate_sp.z();
        _rates_setpoint_pub.thrust_body[0] = _manual_control.throttle;
        _rates_setpoint_pub.thrust_body[1] = 0.0f;
        _rates_setpoint_pub.thrust_body[2] = 0.0f;
    }

    /* ----------------------------------------------------------------
     * STEP 5: Update anti-windup flags from control allocator feedback.
     * ----------------------------------------------------------------
     * The control allocator reports how much torque it could NOT achieve
     * ("unallocated" torque).  If the unallocated torque is in the same
     * direction as the demanded torque, the actuator(s) on that axis
     * are saturated.  We tell the PID integrator to stop accumulating
     * in that direction, preventing integrator windup.
     *
     * This is a feedback-based anti-windup strategy, as opposed to
     * a fixed clamp on the integrator.  It adapts to the actual
     * actuator limits, including cross-axis coupling in the allocator.
     */
    updateAntiWindup();

    /* ----------------------------------------------------------------
     * STEP 6: Update PID gains.
     * ----------------------------------------------------------------
     * In PX4, parameters can change at any time (via QGC or MAVLink).
     * We re-apply them every cycle.  The cost is trivial compared to
     * the PID computation itself.
     *
     * Feed-forward gains receive special treatment: they must be
     * divided by the airspeed scaling factor.  The reason is that
     * feed-forward maps a desired rate directly to a surface deflection
     * (torque), bypassing the "angular acceleration" stage.  Since
     * torque is later multiplied by airspeed_scaling^2, and feed-forward
     * should only scale linearly with dynamic pressure (not quadratically),
     * we pre-divide by airspeed_scaling so the net effect on FF is
     * multiplied by airspeed_scaling^1 (the correct V-dependency for
     * a direct rate-to-deflection mapping).
     *
     *   Derivation:
     *     FF_output = (FF_gain / scaling) * rate_sp        [from PID]
     *     torque    = FF_output * scaling^2                 [post-PID]
     *     net       = FF_gain * rate_sp * scaling^1         [correct]
     *
     * For P, I, D the scaling^2 is correct because those terms produce
     * angular acceleration, which maps to torque via moment of inertia
     * and dynamic pressure (~V^2).
     */
    updatePidGains();

    /*
     * Apply the feed-forward gain scaling.
     *
     * We override the FF gain in the RateControl with the parameter
     * value divided by the current airspeed scaling.  This ensures
     * the feed-forward path has the correct V-dependency as derived
     * above.  We do this AFTER updatePidGains() so it overrides the
     * base FF values.
     */
    {
        const float inv_scaling = (_airspeed_scaling > math::FLT_EPSILON)
                                  ? (1.0f / _airspeed_scaling) : 1.0f;
        const math::Vector3f ff_scaled(
            params.fw_rr_ff * inv_scaling,
            params.fw_pr_ff * inv_scaling,
            params.fw_yr_ff * inv_scaling
        );
        _rate_control.setFeedForwardGain(ff_scaled);
    }

    /* ----------------------------------------------------------------
     * STEP 7: Run the 3-axis PID.
     * ----------------------------------------------------------------
     * The RateControl::update() function computes:
     *
     *   angular_accel_sp[i] = P * error + I * integral(error) + D * d(rate)/dt + FF * rate_sp
     *
     * The output is an angular acceleration setpoint [rad/s^2].
     * We then convert this to a normalised torque command using the
     * airspeed scaling.
     */
    const math::Vector3f angular_accel_sp = _rate_control.update(rates, rate_sp, dt);

    /* ----------------------------------------------------------------
     * STEP 8: Apply airspeed scaling to convert angular acceleration
     *         setpoint into normalised torque commands.
     * ----------------------------------------------------------------
     * The relationship between surface deflection (delta), dynamic
     * pressure (q_bar), and angular acceleration (alpha_dot) is:
     *
     *   alpha_dot = (q_bar * S * b * C_l_delta / I_xx) * delta
     *
     * where S = wing area, b = span, C_l_delta = control derivative,
     * I_xx = moment of inertia.  Since q_bar ~ V^2:
     *
     *   delta ~ alpha_dot / V^2
     *
     * We normalise by trim speed, so:
     *
     *   torque (normalised) = angular_accel_sp * (V_trim / V_actual)^2
     *                       = angular_accel_sp * airspeed_scaling^2
     *
     * This cancels the V^2 dependency: at high speed we deflect less,
     * at low speed we deflect more, maintaining constant bandwidth.
     */
    const float scaling_sq = _airspeed_scaling * _airspeed_scaling;

    math::Vector3f torque = angular_accel_sp * scaling_sq;

    /* ----------------------------------------------------------------
     * STEP 9: Add airspeed-scheduled trim offsets.
     * ----------------------------------------------------------------
     * Even in perfectly trimmed steady flight, non-zero surface
     * deflections are needed to compensate for CG offset, propeller
     * torque, asymmetric drag, etc.
     *
     * The trim values are airspeed-scheduled: the nominal trim
     * (fw_man_{r,p,y}_trim) is for the trim airspeed, and deltas
     * are interpolated for speeds above or below trim.
     *
     * The trim is scaled by airspeed_scaling^2 for the same reason
     * as the main torque command: the physical deflection needed
     * changes with dynamic pressure.  (computeAirspeedTrim handles
     * this internally.)
     */
    const math::Vector3f trim = computeAirspeedTrim(effective_airspeed);
    torque += trim;

    /* ----------------------------------------------------------------
     * STEP 10: Constrain torque commands to [-1, +1].
     * ----------------------------------------------------------------
     * The control allocator expects normalised commands.  Clamping
     * here is a safety measure; the allocator also clamps internally.
     */
    torque = math::Vector3f(
        math::constrain(torque.x(), -1.0f, 1.0f),
        math::constrain(torque.y(), -1.0f, 1.0f),
        math::constrain(torque.z(), -1.0f, 1.0f)
    );

    /* ----------------------------------------------------------------
     * STEP 11: Roll-to-yaw adverse-yaw feed-forward.
     * ----------------------------------------------------------------
     * When the aircraft rolls, the ailerons create differential drag:
     * the down-going aileron produces more drag than the up-going one.
     * This creates a yawing moment OPPOSITE to the roll direction
     * (adverse yaw).  Without compensation, the aircraft "skids" in
     * turns, which is uncomfortable, increases drag, and can be
     * dangerous near stall.
     *
     * The feed-forward term applies proactive rudder proportional to
     * the roll torque command.  This is faster than waiting for the
     * yaw-rate PID to detect and correct the yaw error.
     *
     *   torque_yaw += fw_rll_to_yaw_ff * torque_roll
     *
     * The sign is positive because:
     *   - Right roll command (positive roll torque) creates adverse yaw
     *     to the LEFT (negative yaw moment).
     *   - We want to ADD right rudder (positive yaw torque) to
     *     compensate.
     *   - So positive roll_torque * positive gain -> positive yaw_torque.
     *
     * After adding, we re-constrain the yaw torque.
     */
    torque = math::Vector3f(
        torque.x(),
        torque.y(),
        math::constrain(
            torque.z() + params.fw_rll_to_yaw_ff * torque.x(),
            -1.0f, 1.0f)
    );

    /* ----------------------------------------------------------------
     * STEP 12: Set thrust from the rate setpoint.
     * ----------------------------------------------------------------
     * The thrust command is passed through from the upstream controller
     * (TECS for auto modes, manual stick for stabilised modes).
     * It is NOT part of the PID -- the rate controller just forwards it
     * to the allocator.
     *
     * For ACRO mode, thrust comes from the manual throttle stick
     * (stored in _rates_setpoint_pub above).
     */
    float thrust = (nav_state == NAVIGATION_STATE_ACRO)
                   ? _manual_control.throttle
                   : _rates_setpoint.thrust_body[0];

    /* ----------------------------------------------------------------
     * STEP 13: Battery voltage compensation on thrust.
     * ----------------------------------------------------------------
     * As the battery discharges, its voltage drops.  For a fixed PWM
     * command, a lower voltage produces less thrust.  To maintain
     * consistent thrust performance across the discharge curve, we
     * multiply the thrust command by a voltage-dependent scale factor:
     *
     *   thrust_compensated = thrust * (V_nominal / V_actual)
     *
     * The battery_status.scale field provides this ratio.
     *
     * We only apply compensation when:
     *   (a) The feature is enabled (fw_bat_scale_en == 1)
     *   (b) The commanded thrust is above a minimum threshold (0.1)
     *       to avoid amplifying noise at low thrust
     *
     * The final thrust is clamped to [0, 1] for forward flight.
     */
    if (params.fw_bat_scale_en && thrust > 0.1f) {
        thrust *= _battery_status.scale;
    }

    /* ----------------------------------------------------------------
     * STEP 14: VTOL tailsitter -- reverse transform on OUTPUT torques.
     * ----------------------------------------------------------------
     * We applied the forward transform on the input rates (Step 2b).
     * Now we must reverse it on the output torques so the commands
     * are in the tailsitter's physical body frame (the frame in which
     * the actuators are mounted).
     *
     * Forward transform (input, Step 2b):
     *   p_fw = -r_body   =>   r_body = -p_fw
     *   q_fw =  q_body   =>   q_body =  q_fw
     *   r_fw =  p_body   =>   p_body =  r_fw
     *
     * Inverse on torques (output):
     *   torque_p_body =  torque_r_fw    (roll motor  <- FW yaw torque)
     *   torque_q_body =  torque_q_fw    (pitch motor <- FW pitch torque)
     *   torque_r_body = -torque_p_fw    (yaw motor   <- neg FW roll torque)
     */
    if (params.is_vtol_tailsitter) {
        const float t_roll_fw  = torque.x();
        const float t_pitch_fw = torque.y();
        const float t_yaw_fw   = torque.z();

        torque = math::Vector3f(
             t_yaw_fw,       // body roll  = FW yaw
             t_pitch_fw,     // body pitch = FW pitch (unchanged)
            -t_roll_fw       // body yaw   = -FW roll
        );
    }

    /* ----------------------------------------------------------------
     * STEP 15: Store output torque and thrust setpoints.
     * ----------------------------------------------------------------
     * These are the final commands sent to the control allocator,
     * which maps them to individual servo deflections and motor
     * PWM values.
     */
    _torque_sp_pub.timestamp = timestamp_now;
    _torque_sp_pub.xyz[0]    = torque.x();
    _torque_sp_pub.xyz[1]    = torque.y();
    _torque_sp_pub.xyz[2]    = torque.z();

    _thrust_sp_pub.timestamp = timestamp_now;
    _thrust_sp_pub.xyz[0]    = thrust;
    _thrust_sp_pub.xyz[1]    = 0.0f;
    _thrust_sp_pub.xyz[2]    = 0.0f;

    /* ----------------------------------------------------------------
     * STEP 16: Publish flaps and spoilers from manual control.
     * ----------------------------------------------------------------
     * Flaps and spoilers are not part of the PID loop.  They are set
     * by the pilot (or an auto mode) and scaled by parameters.
     * The allocator handles the actual servo mapping.
     */
    _flaps_setpoint   = _manual_control.flaps * params.fw_flaps_scl;
    _spoiler_setpoint = _manual_control.aux1 * params.fw_spoilers_man;

    /* ---- Update timestamp for next cycle ---- */
    _last_run_timestamp = timestamp_now;
}


/* ======================================================================
 * updatePidGains()
 * ======================================================================
 *
 * Transfer the tunable parameters from the FwRateControlParams struct
 * into the RateControl library.  In PX4 this is called whenever the
 * parameter-update event fires; here we call it every cycle for
 * simplicity (the overhead is negligible).
 *
 * The three-axis gain vectors are:
 *   index 0 = roll (x)
 *   index 1 = pitch (y)
 *   index 2 = yaw (z)
 */
void FixedwingRateControl::updatePidGains()
{
    /* Proportional gains */
    const math::Vector3f p(
        params.fw_rr_p,
        params.fw_pr_p,
        params.fw_yr_p
    );

    /* Integral gains */
    const math::Vector3f i(
        params.fw_rr_i,
        params.fw_pr_i,
        params.fw_yr_i
    );

    /* Derivative gains */
    const math::Vector3f d(
        params.fw_rr_d,
        params.fw_pr_d,
        params.fw_yr_d
    );

    _rate_control.setPidGains(p, i, d);

    /* Feed-forward gains.
     *
     * Note: these are the BASE values.  In Run(), the FF gains are
     * re-set with an airspeed-scaling correction (divided by
     * _airspeed_scaling) so that the feed-forward path has the
     * correct V-dependency.  Setting them here ensures the correct
     * values are present if Run() is not called (e.g., during init).
     */
    const math::Vector3f ff(
        params.fw_rr_ff,
        params.fw_pr_ff,
        params.fw_yr_ff
    );
    _rate_control.setFeedForwardGain(ff);

    /* Integrator limits (absolute value).
     *
     * These prevent the integrator from accumulating unbounded torque
     * even when the anti-windup from the allocator is not yet active
     * (e.g., on the first few cycles before the allocator publishes
     * feedback).
     */
    const math::Vector3f imax(
        params.fw_rr_imax,
        params.fw_pr_imax,
        params.fw_yr_imax
    );
    _rate_control.setIntegratorLimit(imax);
}


/* ======================================================================
 * computeAirspeedScaling()
 * ======================================================================
 *
 * Compute the airspeed-dependent scaling factor used to maintain
 * constant closed-loop bandwidth across the flight envelope.
 *
 * Physics:
 *   Aerodynamic moments are proportional to dynamic pressure:
 *     M = q_bar * S * b * C_m = 0.5 * rho * V^2 * S * b * C_m
 *
 *   For a given normalised control deflection, the resulting angular
 *   acceleration scales with V^2.  To cancel this:
 *
 *     scaling = V_trim / V_actual
 *     torque  = angular_accel_sp * scaling^2
 *             = angular_accel_sp * (V_trim / V_actual)^2
 *             ~ angular_accel_sp / (V_actual^2 / V_trim^2)
 *
 *   At trim speed, scaling = 1.0: the PID gains are tuned for this
 *   condition.  Faster => scaling < 1 => smaller deflections.
 *   Slower => scaling > 1 => larger deflections.
 *
 *   The lower clamp at V_min prevents excessively large commands near
 *   stall, where the aerodynamic model breaks down anyway.
 *
 * @param airspeed_cas  Calibrated airspeed [m/s].
 * @return              Scaling factor (dimensionless).
 */
float FixedwingRateControl::computeAirspeedScaling(float airspeed_cas) const
{
    if (params.fw_arsp_scale_en) {
        /*
         * Clamp the airspeed to the minimum to avoid:
         *   (a) Division by zero if airspeed is reported as 0
         *   (b) Extremely large scaling at very low airspeed, which
         *       would saturate actuators and make the aircraft
         *       uncontrollable
         */
        const float airspeed_clamped = std::max(airspeed_cas, params.fw_airspd_min);
        return params.fw_airspd_trim / airspeed_clamped;
    }

    /*
     * Scaling disabled -- return unity.  This is used when flying
     * without an airspeed sensor and relying on the EKF's airspeed
     * estimate.  Not recommended for gusty conditions because the
     * controller cannot adapt to true airspeed changes.
     */
    return 1.0f;
}


/* ======================================================================
 * computeAirspeedTrim()
 * ======================================================================
 *
 * Compute the 3-axis trim vector, scheduled by airspeed.
 *
 * Even in steady, level flight the control surfaces need non-zero
 * deflection to compensate for:
 *   - CG offset from the aerodynamic centre
 *   - Propeller torque reaction
 *   - Asymmetric drag (e.g., antenna mast, camera pod)
 *   - Aerodynamic asymmetries from manufacturing
 *
 * The nominal trim values (fw_man_{r,p,y}_trim) are determined at
 * the trim airspeed (fw_airspd_trim) by manual flight trimming.
 * At other airspeeds the required trim changes because:
 *   - Propeller torque effect varies with throttle (correlated with V)
 *   - The angle of attack changes with V (affecting CG-AC offset moment)
 *   - Control surface effectiveness changes with V^2
 *
 * We interpolate linearly between three anchor points:
 *
 *   V_min:   trim_nominal + dtrim_vmin   (e.g., more right aileron at low speed)
 *   V_trim:  trim_nominal                (zero delta at trim speed)
 *   V_max:   trim_nominal + dtrim_vmax   (e.g., less right aileron at high speed)
 *
 * The entire trim vector is then scaled by airspeed_scaling^2 because
 * the trim deflection must also be adjusted for dynamic pressure
 * (same reasoning as the main torque scaling).
 *
 * @param airspeed  Current calibrated airspeed [m/s].
 * @return          3-axis trim vector in normalised actuator units.
 */
math::Vector3f FixedwingRateControl::computeAirspeedTrim(float airspeed) const
{
    /*
     * Start with the base (nominal) trim, which corresponds to the
     * deflections needed at the trim airspeed.
     */
    float roll_trim  = params.fw_man_r_trim;
    float pitch_trim = params.fw_man_p_trim;
    float yaw_trim   = params.fw_man_y_trim;

    /*
     * Interpolate the airspeed-dependent trim deltas.
     *
     * Below trim speed: interpolate from (V_min, dtrim_vmin) to (V_trim, 0).
     *   At V_min, the full delta is added.
     *   At V_trim, the delta is zero.
     *
     * Above trim speed: interpolate from (V_trim, 0) to (V_max, dtrim_vmax).
     *   At V_trim, the delta is zero.
     *   At V_max, the full delta is added.
     */
    if (airspeed < params.fw_airspd_trim) {
        /*
         * Below trim speed -- interpolate the "vmin" deltas.
         * The delta is largest at V_min and tapers to zero at V_trim.
         */
        roll_trim  += math::interpolate(airspeed,
                                         params.fw_airspd_min, params.fw_airspd_trim,
                                         params.fw_dtrim_r_vmin, 0.0f);
        pitch_trim += math::interpolate(airspeed,
                                         params.fw_airspd_min, params.fw_airspd_trim,
                                         params.fw_dtrim_p_vmin, 0.0f);
        yaw_trim   += math::interpolate(airspeed,
                                         params.fw_airspd_min, params.fw_airspd_trim,
                                         params.fw_dtrim_y_vmin, 0.0f);
    } else {
        /*
         * Above trim speed -- interpolate the "vmax" deltas.
         * The delta is zero at V_trim and grows to dtrim_vmax at V_max.
         */
        roll_trim  += math::interpolate(airspeed,
                                         params.fw_airspd_trim, params.fw_airspd_max,
                                         0.0f, params.fw_dtrim_r_vmax);
        pitch_trim += math::interpolate(airspeed,
                                         params.fw_airspd_trim, params.fw_airspd_max,
                                         0.0f, params.fw_dtrim_p_vmax);
        yaw_trim   += math::interpolate(airspeed,
                                         params.fw_airspd_trim, params.fw_airspd_max,
                                         0.0f, params.fw_dtrim_y_vmax);
    }

    /*
     * Scale the entire trim vector by airspeed_scaling^2.
     *
     * The trim represents a surface deflection, and the moment produced
     * by that deflection depends on dynamic pressure (~V^2).  We need
     * to adjust the deflection so that the MOMENT stays constant as
     * airspeed changes.  Since the main torque path already uses
     * scaling^2, we apply the same to trim for consistency.
     */
    const float scaling_sq = _airspeed_scaling * _airspeed_scaling;

    return math::Vector3f(
        roll_trim  * scaling_sq,
        pitch_trim * scaling_sq,
        yaw_trim   * scaling_sq
    );
}


/* ======================================================================
 * getEffectiveAirspeed()
 * ======================================================================
 *
 * Determine the best available airspeed estimate for control purposes.
 *
 * Priority (highest to lowest):
 *
 *   1. Validated airspeed sensor (pitot tube fused with wind estimate).
 *      This is the most accurate source.  We check the timestamp to
 *      ensure the measurement is recent (< AIRSPEED_TIMEOUT_US).
 *
 *   2. Ground speed from the EKF (sqrt(vx^2 + vy^2)).
 *      In zero-wind conditions this equals airspeed.  In windy
 *      conditions it can be significantly off, but it is still
 *      better than a fixed guess because it tracks the aircraft's
 *      energy state.  We only use it if the EKF reports valid velocity.
 *
 *   3. Trim airspeed (last resort).
 *      If neither airspeed nor ground speed is available, we assume
 *      the aircraft is flying at its trim speed.  This gives correct
 *      scaling at least for nominal flight conditions.
 *
 * @return  Effective calibrated airspeed [m/s].
 */
float FixedwingRateControl::getEffectiveAirspeed() const
{
    /*
     * Option 1: Validated airspeed from the sensor.
     *
     * Check that the timestamp is non-zero (sensor has published at
     * least once) and that the measurement is recent.  A stale
     * measurement could be arbitrarily wrong (e.g., pitot tube iced
     * over or disconnected).
     */
    if (_airspeed_validated.timestamp > 0) {
        /*
         * Compute the age of the measurement.  We use the last Run()
         * timestamp as the current time reference (it was set at the
         * end of the previous cycle, or at the start of this cycle
         * for the first-run case).
         *
         * Note: we use _last_run_timestamp rather than a live clock
         * because this module does not have direct hrt_absolute_time()
         * access in the standalone build.  In PX4, hrt_elapsed_time()
         * would be used.
         */
        const uint64_t age_us = (_last_run_timestamp > _airspeed_validated.timestamp)
                                ? (_last_run_timestamp - _airspeed_validated.timestamp)
                                : 0;

        if (age_us < AIRSPEED_TIMEOUT_US) {
            return _airspeed_validated.calibrated_airspeed_m_s;
        }
    }

    /*
     * Option 2: Ground speed as a proxy for airspeed.
     *
     * In the local NED frame, the horizontal velocity components are
     * vx (north) and vy (east).  The horizontal ground speed is:
     *
     *   V_ground = sqrt(vx^2 + vy^2)
     *
     * This ignores wind, so in a headwind the true airspeed is higher
     * than ground speed, and in a tailwind it is lower.  Nevertheless,
     * using ground speed gives reasonable airspeed scaling behaviour
     * for most conditions.
     *
     * We require v_xy_valid from the EKF to ensure the estimate is
     * trustworthy (e.g., GPS lock is present).
     */
    if (_local_position.v_xy_valid) {
        const float groundspeed = std::sqrt(
            _local_position.vx * _local_position.vx +
            _local_position.vy * _local_position.vy
        );
        return groundspeed;
    }

    /*
     * Option 3: Fall back to trim airspeed.
     *
     * This is the safest guess when no speed information is available.
     * The airspeed scaling will be 1.0 (since V_trim / V_trim = 1),
     * giving the nominal PID response.
     */
    return params.fw_airspd_trim;
}


/* ======================================================================
 * updateAntiWindup()
 * ======================================================================
 *
 * Read the control allocator's saturation feedback and set the
 * corresponding flags on the PID integrator.
 *
 * The control allocator attempts to achieve the demanded torque using
 * the available actuators (ailerons, elevator, rudder).  When an
 * actuator reaches its physical limit, the allocator reports the
 * "unallocated" torque -- the part of the demand it could not fulfil.
 *
 * Anti-windup logic:
 *   If unallocated_torque[axis] > 0, it means we demanded MORE positive
 *   torque than the actuators can provide.  The positive direction is
 *   saturated, so we should NOT let the integrator increase further
 *   in the positive direction on that axis (it would only make the
 *   windup worse).
 *
 *   Conversely, if unallocated_torque[axis] < 0, the negative direction
 *   is saturated.
 *
 * This is a more sophisticated anti-windup than a simple integrator
 * clamp because it accounts for the actual actuator limits and
 * cross-axis coupling in the allocator.  For example, if the elevator
 * is at max deflection due to a combined pitch+yaw demand, only the
 * saturated directions are frozen while the opposite direction remains
 * free to respond.
 */
void FixedwingRateControl::updateAntiWindup()
{
    for (int axis = 0; axis < 3; axis++) {
        const float unalloc = _control_allocator_status.unallocated_torque[axis];

        /*
         * Positive saturation: unallocated torque is positive, meaning
         * the demand exceeded what could be achieved in the positive
         * direction.  Freeze the integrator from increasing further.
         */
        _rate_control.setPositiveSaturationFlag(axis, (unalloc > math::FLT_EPSILON));

        /*
         * Negative saturation: unallocated torque is negative, meaning
         * the demand exceeded what could be achieved in the negative
         * direction.  Freeze the integrator from decreasing further.
         */
        _rate_control.setNegativeSaturationFlag(axis, (unalloc < -math::FLT_EPSILON));
    }
}
