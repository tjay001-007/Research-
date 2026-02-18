/**
 * @file FixedwingAttitudeControl.cpp
 *
 * Implementation of the main fixed-wing attitude controller.
 *
 * See FixedwingAttitudeControl.hpp for the full architecture description,
 * control flow documentation, and parameter definitions.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_att_control/FixedwingAttitudeControl.cpp
 *
 * ============================================================================
 * CONTROL LOOP OVERVIEW
 * ============================================================================
 *
 * The Run() method executes the following pipeline at ~250 Hz:
 *
 *   +-----------+     +--------+     +--------+     +--------+
 *   | Attitude  | --> |  Yaw   | --> |  Roll  | --> | Pitch  |
 *   | Estimation|     |  Ctrl  |     |  Ctrl  |     |  Ctrl  |
 *   +-----------+     +--------+     +--------+     +--------+
 *         |                |              |              |
 *         v                v              v              v
 *     Euler angles    psi_dot_sp      p_sp (body)    q_sp (body)
 *                   (coordinated     (roll body     (pitch body
 *                     turn rate)       rate)          rate)
 *                                         |
 *                                         v
 *                              +-----------------------+
 *                              | vehicle_rates_setpoint|
 *                              +-----------------------+
 *
 * The yaw controller runs FIRST because its output -- the Euler yaw rate
 * from the coordinated turn equation -- is needed by both the roll and
 * pitch controllers for their Euler-to-body Jacobian transformations.
 *
 * Optionally, the wheel controller runs after the main axes to produce
 * a yaw rate override for ground steering.
 */

#include "FixedwingAttitudeControl.hpp"

#include <cmath>
#include <algorithm>

namespace fw_att_control {

/* ======================================================================
 * Subscription setters
 * ======================================================================
 *
 * These methods mimic the uORB subscription mechanism. In the real PX4,
 * the subscriber automatically receives the latest published message.
 * Here, external code (test harness, simulation bridge) must push data
 * before each call to Run().
 */

void FixedwingAttitudeControl::set_vehicle_attitude(const uorb::vehicle_attitude_s& msg)
{
    _vehicle_attitude = msg;
}

void FixedwingAttitudeControl::set_vehicle_attitude_setpoint(const uorb::vehicle_attitude_setpoint_s& msg)
{
    _att_sp = msg;
}

void FixedwingAttitudeControl::set_vehicle_angular_velocity(const uorb::vehicle_angular_velocity_s& msg)
{
    _angular_velocity = msg;
}

void FixedwingAttitudeControl::set_airspeed_validated(const uorb::airspeed_validated_s& msg)
{
    _airspeed_validated = msg;

    /*
     * Mark airspeed as valid if we have a positive calibrated value.
     * In the real PX4, validity is determined by the airspeed validator
     * module which checks sensor health, consistency, and staleness.
     * Here we use a simple positive-value check.
     */
    _airspeed_valid = (msg.calibrated_airspeed_m_s > 0.0f);
}

void FixedwingAttitudeControl::set_vehicle_status(const uorb::vehicle_status_s& msg)
{
    _vehicle_status = msg;
}

void FixedwingAttitudeControl::set_manual_control_setpoint(const uorb::manual_control_setpoint_s& msg)
{
    _manual_control = msg;
}

void FixedwingAttitudeControl::set_autotune_attitude_control_status(
    const uorb::autotune_attitude_control_status_s& msg)
{
    _autotune_status = msg;
}

void FixedwingAttitudeControl::set_vehicle_local_position(const uorb::vehicle_local_position_s& msg)
{
    _local_position = msg;
}

/* ======================================================================
 * Parameter application
 * ======================================================================
 *
 * Push the current parameter values into each sub-controller. In the
 * real PX4 this is guarded by a "parameters_updated" flag to avoid
 * unnecessary work. Here we always apply since the cost is trivial.
 */

void FixedwingAttitudeControl::apply_parameters()
{
    /*
     * Roll controller:
     *   - Time constant determines how aggressively roll errors are closed.
     *   - Max rate prevents demanding more than the airframe can deliver.
     */
    _roll_ctrl.set_time_constant(_params.fw_r_tc);
    _roll_ctrl.set_max_rate(_params.fw_r_rmax);

    /*
     * Pitch controller:
     *   - Same time-constant / max-rate pattern as roll.
     *   - Pitch has asymmetric rate limits (pos = nose-up, neg = nose-down)
     *     because elevator authority is typically asymmetric.
     */
    _pitch_ctrl.set_time_constant(_params.fw_p_tc);
    _pitch_ctrl.set_max_rate_pos(_params.fw_p_rmax_pos);
    _pitch_ctrl.set_max_rate_neg(_params.fw_p_rmax_neg);

    /*
     * Yaw controller:
     *   - Max rate limits the coordinated-turn yaw rate.
     *   - This is less commonly tuned; the default is appropriate for
     *     most airframes.
     */
    _yaw_ctrl.set_max_rate(_params.fw_y_rmax);

    /*
     * Wheel controller:
     *   - Time constant and max rate for nose-wheel ground steering.
     *   - Only relevant when fw_w_en is set and the aircraft is on ground.
     */
    _wheel_ctrl.set_time_constant(_params.fw_w_tc);
    _wheel_ctrl.set_max_rate(_params.fw_w_rmax);
}

/* ======================================================================
 * VTOL tailsitter transform
 * ======================================================================
 *
 * A tailsitter sits on its tail for hover, with the body x-axis pointing
 * UP. When it transitions to fixed-wing flight, the body x-axis points
 * FORWARD. However, the attitude estimator always reports attitude relative
 * to the physical body frame, so we need to rotate the DCM before
 * extracting Euler angles.
 *
 * The transformation swaps the body-x and body-z columns of the rotation
 * matrix. Geometrically, this is a 90-degree rotation about the body
 * y-axis (pitch axis):
 *
 *   R_fw = R_body * Ry(90deg)
 *
 * In matrix form, Ry(90) swaps columns 0 and 2 with a sign change:
 *   new_col0 =  old_col2   (the "forward" axis in FW mode was "down" in body)
 *   new_col1 =  old_col1   (lateral axis unchanged)
 *   new_col2 = -old_col0   (the "down" axis in FW mode was "forward" in body)
 *
 * After this swap, extracting Euler angles from the modified DCM gives
 * roll/pitch/yaw in the conventional fixed-wing sense.
 */

void FixedwingAttitudeControl::apply_tailsitter_transform(math::Matrix3f& R)
{
    /*
     * Save the original first column before overwriting.
     * We operate row-by-row since the matrix is stored as m[row][col].
     */
    for (int row = 0; row < 3; row++) {
        const float col0 = R(row, 0);
        R(row, 0) =  R(row, 2);
        R(row, 2) = -col0;
    }
}

/* ======================================================================
 * STABILIZED mode: stick-to-attitude conversion
 * ======================================================================
 *
 * In STABILIZED mode, the pilot directly commands attitude angles with
 * the sticks. There is no path controller providing setpoints; instead,
 * this function converts raw stick inputs [-1, +1] into angle setpoints.
 *
 * Mapping:
 *   roll_sp  =  manual.roll  * fw_man_r_max * fw_man_r_sc
 *   pitch_sp = -manual.pitch * fw_man_p_max * fw_man_p_sc
 *   thrust   =  manual.throttle
 *
 * The pitch sign is inverted because:
 *   - Stick convention: pulling back (pilot action) gives positive pitch input
 *   - Aircraft convention: nose-up is positive pitch angle
 *   - But PX4 manual_control_setpoint uses: stick-back = positive value
 *   - And we want: stick-back = pitch-up = positive pitch setpoint
 *   - However, the PX4 convention for pitch stick is inverted from the
 *     intuitive direction, so we negate to get the correct mapping.
 *
 * Yaw is not converted to an angle setpoint. Instead, the yaw stick
 * directly commands a body yaw rate (scaled by fw_man_y_sc), which is
 * added to the coordinated-turn yaw rate from the yaw controller.
 */

void FixedwingAttitudeControl::generate_stabilized_setpoint()
{
    /*
     * Roll setpoint: full stick = maximum bank angle.
     *
     * The scale factor allows adjusting sensitivity without changing
     * the maximum angle. For example, fw_man_r_sc = 0.5 means full
     * stick only commands half the maximum bank angle.
     */
    _att_sp.roll_body = _manual_control.roll * _params.fw_man_r_max * _params.fw_man_r_sc;

    /*
     * Pitch setpoint: full stick-back = maximum pitch-up angle.
     *
     * The negative sign handles the stick convention inversion.
     * manual_control.pitch > 0 when stick is pushed forward (nose-down desired),
     * but we want positive pitch_body to mean nose-up. So we negate.
     */
    _att_sp.pitch_body = -_manual_control.pitch * _params.fw_man_p_max * _params.fw_man_p_sc;

    /*
     * Yaw: no angle setpoint in STABILIZED mode. The yaw stick value
     * is used later as a direct body rate command (see Run()).
     * We store zero here; the actual yaw handling is in the main loop.
     */
    _att_sp.yaw_body = 0.0f;

    /*
     * Thrust: direct passthrough from throttle stick.
     *
     * For fixed-wing, thrust is along the body x-axis (forward).
     * The throttle stick range is typically [0, 1] after dead-zone
     * processing, but we pass it through as-is.
     */
    _att_sp.thrust_body[0] = _manual_control.throttle;
    _att_sp.thrust_body[1] = 0.0f;
    _att_sp.thrust_body[2] = 0.0f;
}

/* ======================================================================
 * Main control loop
 * ====================================================================== */

void FixedwingAttitudeControl::Run()
{
    /* ==================================================================
     * STEP 1: Apply parameters to sub-controllers.
     * ==================================================================
     * In the real PX4, parameters are only re-applied when the parameter
     * server signals a change (via orb_check on the parameter_update topic).
     * Here we always apply since the overhead is negligible and it keeps
     * the code simpler for educational purposes.
     */
    apply_parameters();

    /* ==================================================================
     * STEP 2: Convert attitude quaternion to rotation matrix and Euler angles.
     * ==================================================================
     *
     * The attitude estimator (EKF2) publishes the vehicle attitude as a
     * quaternion. We convert to:
     *   - DCM (rotation matrix): needed for the Jacobian transformations
     *     inside the sub-controllers, and for the tailsitter column swap.
     *   - Euler angles (roll, pitch, yaw): the natural error space for
     *     the attitude controllers.
     *
     * The quaternion representation is singularity-free and used internally
     * by the EKF, but Euler angles are more intuitive for control law
     * formulation (small-angle linearization, time-constant tuning, etc.).
     */
    const math::Quatf q(
        _vehicle_attitude.q[0],
        _vehicle_attitude.q[1],
        _vehicle_attitude.q[2],
        _vehicle_attitude.q[3]
    );

    math::Matrix3f R = q.to_dcm();

    /* ==================================================================
     * STEP 3: VTOL tailsitter handling.
     * ==================================================================
     *
     * If this is a tailsitter in fixed-wing mode, we transform the DCM
     * so that Euler angle extraction produces conventional fixed-wing
     * roll/pitch/yaw. Without this, the controllers would "see" the
     * hover-frame orientation and produce nonsensical commands.
     *
     * This must happen BEFORE Euler angle extraction and BEFORE the
     * sub-controllers run, because they all operate in the conventional
     * fixed-wing frame.
     */
    if (_params.is_vtol_tailsitter) {
        apply_tailsitter_transform(R);
    }

    float roll  = 0.0f;
    float pitch = 0.0f;
    float yaw   = 0.0f;
    R.to_euler(roll, pitch, yaw);

    /* ==================================================================
     * STEP 4: Generate attitude setpoints in STABILIZED mode.
     * ==================================================================
     *
     * In STABILIZED mode there is no guidance/path controller providing
     * attitude setpoints. The pilot flies the aircraft directly through
     * stick-to-angle mappings.
     *
     * In AUTO modes, the attitude setpoint has already been set by the
     * path controller via set_vehicle_attitude_setpoint(), so we skip
     * this step.
     */
    if (_vehicle_status.nav_state == NAVIGATION_STATE_STABILIZED) {
        generate_stabilized_setpoint();
    }

    /* ==================================================================
     * STEP 5: Get validated airspeed (or fallback to trim).
     * ==================================================================
     *
     * Airspeed is critical for the yaw controller's coordinated turn
     * calculation:
     *
     *   psi_dot = g * tan(phi) / V
     *
     * If no valid airspeed is available (sensor failure, not installed),
     * we fall back to the trim airspeed. This is a safe default because:
     *   - At trim speed, the coordinated turn rate is moderate
     *   - If actual speed is higher, the computed rate is slightly too high
     *     (minor yaw oscillation, quickly damped by the rate controller)
     *   - If actual speed is lower, the computed rate is slightly too low
     *     (minor slip, acceptable for safety)
     *
     * We also enforce a floor at fw_airspd_min to prevent division by
     * very small numbers in the coordinated turn equation.
     */
    float airspeed = _params.fw_airspd_trim;

    if (_airspeed_valid) {
        airspeed = _airspeed_validated.calibrated_airspeed_m_s;
    }

    /*
     * Clamp airspeed to the minimum. Below this threshold, the
     * coordinated turn yaw rate would become unrealistically large.
     * This also handles the case where the airspeed sensor reads
     * near-zero during pre-flight or on the ground.
     */
    airspeed = std::max(airspeed, _params.fw_airspd_min);

    /* ==================================================================
     * STEP 6: Run yaw controller FIRST.
     * ==================================================================
     *
     * The yaw controller computes the Euler yaw rate (psi_dot) needed
     * to maintain coordinated flight in a banked turn. The coordinated
     * turn equation is:
     *
     *   psi_dot = g * tan(phi) / V
     *
     * This Euler yaw rate is then used by both the roll and pitch
     * controllers in their Euler-to-body Jacobian transformations:
     *
     *   p = phi_dot   - sin(theta) * psi_dot        (roll)
     *   q = cos(phi) * theta_dot + cos(theta) * sin(phi) * psi_dot  (pitch)
     *
     * Therefore, the yaw controller MUST run before roll and pitch.
     *
     * The yaw controller receives:
     *   - Current roll angle (for tan(phi) in the coordinated turn eq.)
     *   - Current airspeed (for V in the coordinated turn eq.)
     *   - Whether active yaw control is requested (e.g., by the path controller
     *     for crosswind landing or specific maneuvers)
     *   - The desired yaw body rate from the attitude setpoint (if yaw control
     *     is active, this overrides the coordinated turn calculation)
     */
    const float yaw_rate_euler = _yaw_ctrl.control_attitude(
        roll,
        airspeed
    );

    /* ==================================================================
     * STEP 7: Run roll controller.
     * ==================================================================
     *
     * The roll controller converts the roll angle error into a body roll
     * rate command. It uses the Euler yaw rate from the yaw controller
     * for the kinematic coupling term:
     *
     *   p = phi_dot - sin(theta) * psi_dot
     *
     * See fw_roll_controller.hpp for the full derivation.
     */
    const float roll_rate_sp = _roll_ctrl.control_attitude(
        roll,
        _att_sp.roll_body,
        pitch,
        yaw_rate_euler
    );

    /* ==================================================================
     * STEP 8: Run pitch controller.
     * ==================================================================
     *
     * The pitch controller converts the pitch angle error into a body
     * pitch rate command. Like roll, it uses the Euler yaw rate for the
     * kinematic coupling term:
     *
     *   q = cos(phi) * theta_dot + cos(theta) * sin(phi) * psi_dot
     *
     * See fw_pitch_controller.hpp for the full derivation.
     */
    const float pitch_rate_sp = _pitch_ctrl.control_attitude(
        pitch,
        _att_sp.pitch_body,
        roll,
        yaw_rate_euler
    );

    /* ==================================================================
     * STEP 9: Assemble the body rate setpoint vector.
     * ==================================================================
     *
     * At this point we have:
     *   - roll_rate_sp  (p): from the roll controller
     *   - pitch_rate_sp (q): from the pitch controller
     *   - yaw_rate_sp   (r): from the yaw controller's Euler rate,
     *     but we need to convert it to a body rate as well.
     *
     * For the yaw axis in the body frame, the kinematic relationship is:
     *
     *   r = -sin(phi) * theta_dot + cos(phi) * cos(theta) * psi_dot
     *
     * However, in PX4 the yaw body rate setpoint is typically set to the
     * Euler yaw rate directly, with the cross-coupling handled implicitly
     * by the rate controller's feedforward and the roll/pitch controllers'
     * Jacobian terms. For simplicity and consistency with PX4, we set the
     * yaw rate setpoint to the coordinated-turn Euler yaw rate.
     *
     * In STABILIZED mode, the yaw stick adds a direct yaw rate command
     * on top of the coordinated turn rate, allowing the pilot to
     * intentionally slip or skid the aircraft.
     */
    float yaw_rate_sp = yaw_rate_euler;

    if (_vehicle_status.nav_state == NAVIGATION_STATE_STABILIZED) {
        /*
         * Add the pilot's yaw stick command as a direct body yaw rate.
         * The scale factor fw_man_y_sc adjusts sensitivity.
         * The maximum yaw rate parameter fw_y_rmax limits the total.
         */
        yaw_rate_sp += _manual_control.yaw * _params.fw_y_rmax * _params.fw_man_y_sc;
    }

    _rates_setpoint.roll  = roll_rate_sp;
    _rates_setpoint.pitch = pitch_rate_sp;
    _rates_setpoint.yaw   = yaw_rate_sp;

    /*
     * Pass through the thrust command from the attitude setpoint.
     * For fixed-wing, only the x-component (forward thrust) is used.
     * The attitude controller does not modify thrust; it simply relays
     * what the path controller (or STABILIZED mode) commanded.
     */
    _rates_setpoint.thrust_body[0] = _att_sp.thrust_body[0];
    _rates_setpoint.thrust_body[1] = _att_sp.thrust_body[1];
    _rates_setpoint.thrust_body[2] = _att_sp.thrust_body[2];

    /* ==================================================================
     * STEP 10: Autotune excitation signal injection.
     * ==================================================================
     *
     * The autotune module (if active) publishes a small excitation signal
     * that must be ADDED to the body rate setpoints. This signal is
     * typically a multi-sine or chirp waveform designed to excite the
     * aircraft's dynamic modes so that the system identification algorithm
     * can estimate the plant transfer function.
     *
     * The excitation is added AFTER the controllers have computed their
     * nominal rates, so it appears as a perturbation on top of the
     * commanded trajectory. The resulting response (measured by the gyros)
     * is recorded by the autotune module to fit the model.
     *
     * The injection is only active when the autotune state machine is in
     * the INJECT state. In IDLE state, the rate_sp array is zero and
     * adding it is a no-op.
     *
     * Safety: the excitation amplitudes are small (typically < 5 deg/s)
     * and the autotune module monitors for excessive attitude deviations,
     * aborting if the aircraft moves too far from the setpoint.
     */
    if (_autotune_status.state == AUTOTUNE_STATE_INJECT) {
        _rates_setpoint.roll  += _autotune_status.rate_sp[0];
        _rates_setpoint.pitch += _autotune_status.rate_sp[1];
        _rates_setpoint.yaw   += _autotune_status.rate_sp[2];
    }

    /* ==================================================================
     * STEP 11: Set timestamp on the published message.
     * ==================================================================
     *
     * In the real PX4, this would be hrt_absolute_time(). Here we copy
     * the attitude timestamp to maintain temporal traceability. The rate
     * controller uses this timestamp for dt calculation and staleness
     * detection.
     */
    _rates_setpoint.timestamp = _vehicle_attitude.timestamp;

    /* ==================================================================
     * STEP 12: Wheel controller (ground steering).
     * ==================================================================
     *
     * The wheel controller provides nose-wheel steering for ground
     * operations (taxiing, takeoff roll, landing rollout). It is only
     * active when:
     *   a) The wheel controller is enabled (FW_W_EN = 1)
     *   b) The aircraft is on the ground (condition_landed = true)
     *
     * When active, the wheel controller's output REPLACES the yaw body
     * rate setpoint. This makes sense because:
     *   - On the ground, the rudder is ineffective at low speeds
     *   - The nose-wheel provides much better directional control
     *   - The coordinated-turn yaw rate is meaningless on the ground
     *
     * The wheel controller works like the other axis controllers: it
     * takes a heading error and produces a yaw rate command, but tuned
     * for the much slower dynamics of ground steering (larger time
     * constant, lower max rate).
     *
     * When the aircraft is airborne, this section is skipped entirely
     * and the yaw rate from the coordinated turn calculation is used.
     *
     * The fw_control_yaw_wheel field in the attitude setpoint provides
     * the desired ground heading or steering command from the path
     * controller or pilot.
     */
    if (_params.fw_w_en && _vehicle_status.condition_landed) {
        /*
         * Run the wheel controller with the current yaw angle and the
         * desired heading from the attitude setpoint.
         *
         * The wheel controller internally wraps the heading error to
         * [-pi, pi] and applies the same time-constant / rate-limit
         * pattern as the other axis controllers.
         */
        const float wheel_rate_sp = _wheel_ctrl.control_attitude(
            yaw,
            _att_sp.fw_control_yaw_wheel
        );

        /*
         * Override the yaw rate setpoint with the wheel controller output.
         * The roll and pitch rate setpoints remain unchanged -- on the
         * ground, they serve to keep the wings level and pitch stable.
         */
        _rates_setpoint.yaw = wheel_rate_sp;
    }
}

} // namespace fw_att_control
