/**
 * @file FixedwingAttitudeControl.hpp
 *
 * Main fixed-wing attitude controller -- orchestration layer.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This class is the top-level module that ties together the four individual
 * axis controllers (roll, pitch, yaw, wheel) and interfaces with the rest
 * of the PX4 autopilot through uORB subscriptions and publications.
 *
 * Its responsibilities are:
 *
 *   1. Subscribe to vehicle state (attitude, angular velocity, airspeed,
 *      vehicle status, manual control, autotune status, local position).
 *
 *   2. Convert the attitude quaternion into a rotation matrix and Euler
 *      angles, applying VTOL tailsitter transformations when needed.
 *
 *   3. In STABILIZED manual mode, map pilot stick inputs to attitude
 *      setpoints (roll angle, pitch angle, direct yaw rate, thrust).
 *
 *   4. Run the sub-controllers in the correct order:
 *        a) Yaw controller first (produces Euler yaw rate for coordinated turn)
 *        b) Roll controller (needs the Euler yaw rate for Jacobian coupling)
 *        c) Pitch controller (needs the Euler yaw rate for Jacobian coupling)
 *        d) Wheel controller (if enabled and on ground)
 *
 *   5. Inject autotune excitation signals on top of the computed body rates.
 *
 *   6. Publish the final vehicle_rates_setpoint message for the downstream
 *      rate controller.
 *
 * ============================================================================
 * ARCHITECTURE CONTEXT
 * ============================================================================
 *
 * The PX4 fixed-wing control stack is structured as a cascade:
 *
 *   Path planner  -->  Attitude controller (THIS MODULE)  -->  Rate controller
 *       ^                      |                                     |
 *   position_sp       vehicle_rates_setpoint_s              vehicle_torque_setpoint_s
 *
 * The attitude controller receives desired roll/pitch/yaw angles and outputs
 * desired body angular rates. This decoupling keeps each layer simple and
 * independently tunable.
 *
 * For VTOL tailsitters, the body-frame axes are rotated 90 degrees relative
 * to a conventional fixed-wing. When in fixed-wing mode, the rotation matrix
 * columns are swapped so that the attitude controllers still "see" a standard
 * fixed-wing orientation. This is handled transparently in this module.
 *
 * ============================================================================
 * CONTROL FLOW IN Run()
 * ============================================================================
 *
 *   1. Read parameters (if changed)
 *   2. Convert quaternion --> DCM --> Euler angles
 *   3. [VTOL tailsitter] Swap roll/yaw columns in the DCM
 *   4. [STABILIZED mode] Convert stick inputs to attitude setpoints
 *   5. Get validated airspeed (fallback to trim if unavailable)
 *   6. Run yaw controller --> Euler yaw rate (psi_dot)
 *   7. Run roll controller (with psi_dot) --> body roll rate (p)
 *   8. Run pitch controller (with psi_dot) --> body pitch rate (q)
 *   9. [Autotune] Add excitation signal to body rate setpoints
 *  10. Publish vehicle_rates_setpoint
 *  11. [Wheel] Run wheel controller (if enabled and on ground)
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_att_control/FixedwingAttitudeControl.cpp
 *   - Stevens & Lewis, "Aircraft Control and Simulation", Chapter 1.3
 *   - Beard & McLain, "Small Unmanned Aircraft", Chapters 5-6
 */

#pragma once

#include "fw_roll_controller.hpp"
#include "fw_pitch_controller.hpp"
#include "fw_yaw_controller.hpp"
#include "fw_wheel_controller.hpp"

#include "../common/math_utils.hpp"
#include "../common/uorb_msgs.hpp"

namespace fw_att_control {

/* ======================================================================
 * Navigation state constants
 * ======================================================================
 * In the real PX4 these come from vehicle_status.h. We define the ones
 * relevant to the attitude controller here for self-contained compilation.
 */
static constexpr uint8_t NAVIGATION_STATE_MANUAL    = 0;
static constexpr uint8_t NAVIGATION_STATE_STABILIZED = 1;
static constexpr uint8_t NAVIGATION_STATE_ACRO      = 2;

/* ======================================================================
 * Autotune state constants
 * ====================================================================== */
static constexpr uint8_t AUTOTUNE_STATE_IDLE   = 0;
static constexpr uint8_t AUTOTUNE_STATE_INJECT = 1;

/* ======================================================================
 * Parameters
 * ======================================================================
 *
 * All tunable parameters are collected in a single struct for clarity.
 * In the real PX4 firmware these are declared via DEFINE_PARAMETERS()
 * and updated through the parameter server. Here we store them directly
 * with sensible defaults.
 *
 * Naming convention follows PX4 parameter names (FW_R_TC, FW_P_TC, etc.)
 * mapped to lower_snake_case.
 */
struct AttitudeControlParams {

    /* ---- Roll axis ---- */

    /**
     * Roll time constant [s].
     *
     * First-order time constant for roll attitude error closure.
     * Effective P-gain = 1/fw_r_tc. Smaller values give faster response
     * but risk exciting structural modes or saturating ailerons.
     *
     * PX4 param: FW_R_TC
     */
    float fw_r_tc = 0.4f;

    /**
     * Maximum roll body rate [rad/s].
     *
     * Limits the roll rate command from the attitude controller to prevent
     * demanding rates the airframe cannot achieve. Determined by flight test.
     *
     * PX4 param: FW_R_RMAX (converted from deg/s to rad/s)
     * Default: ~60 deg/s = 1.05 rad/s
     */
    float fw_r_rmax = 1.05f;

    /* ---- Pitch axis ---- */

    /**
     * Pitch time constant [s].
     *
     * First-order time constant for pitch attitude error closure.
     * Effective P-gain = 1/fw_p_tc.
     *
     * PX4 param: FW_P_TC
     */
    float fw_p_tc = 0.4f;

    /**
     * Maximum positive (nose-up) pitch body rate [rad/s].
     *
     * Pitch has asymmetric limits because most aircraft have more elevator
     * authority for nose-up than nose-down (the CG is ahead of the AC).
     *
     * PX4 param: FW_P_RMAX_POS
     * Default: ~60 deg/s = 1.05 rad/s
     */
    float fw_p_rmax_pos = 1.05f;

    /**
     * Maximum negative (nose-down) pitch body rate [rad/s].
     *
     * PX4 param: FW_P_RMAX_NEG
     * Default: ~60 deg/s = 1.05 rad/s
     */
    float fw_p_rmax_neg = 1.05f;

    /* ---- Yaw axis ---- */

    /**
     * Maximum yaw body rate [rad/s].
     *
     * PX4 param: FW_Y_RMAX
     * Default: ~45 deg/s = 0.7854 rad/s
     */
    float fw_y_rmax = 0.7854f;

    /* ---- Wheel (ground steering) ---- */

    /**
     * Wheel (ground steering) time constant [s].
     *
     * Controls how quickly the nose-wheel steering closes heading errors
     * while taxiing. Larger values give smoother but slower corrections.
     *
     * PX4 param: FW_W_TC
     */
    float fw_w_tc = 1.0f;

    /**
     * Maximum wheel steering rate [rad/s].
     *
     * PX4 param: FW_W_RMAX
     * Default: ~28.6 deg/s = 0.5 rad/s
     */
    float fw_w_rmax = 0.5f;

    /**
     * Enable wheel controller for ground steering.
     *
     * When enabled, the wheel controller replaces the yaw-axis body rate
     * command while the aircraft is on the ground (condition_landed == true).
     *
     * PX4 param: FW_W_EN
     * 0 = disabled, 1 = enabled
     */
    int fw_w_en = 0;

    /* ---- Manual (STABILIZED) mode parameters ---- */

    /**
     * Maximum manual roll angle [rad].
     *
     * Full stick deflection maps to this bank angle in STABILIZED mode.
     *
     * PX4 param: FW_MAN_R_MAX
     * Default: 45 deg = 0.7854 rad
     */
    float fw_man_r_max = 0.7854f;

    /**
     * Maximum manual pitch angle [rad].
     *
     * Full stick deflection maps to this pitch angle in STABILIZED mode.
     * Note: this is typically smaller than roll because large pitch angles
     * can easily stall the aircraft.
     *
     * PX4 param: FW_MAN_P_MAX
     * Default: 20 deg = 0.3491 rad
     */
    float fw_man_p_max = 0.3491f;

    /**
     * Manual roll scale factor.
     *
     * Multiplied with the stick-to-angle conversion. Values > 1 make
     * the roll response more sensitive, < 1 less sensitive.
     *
     * PX4 param: FW_MAN_R_SC
     */
    float fw_man_r_sc = 1.0f;

    /**
     * Manual pitch scale factor.
     *
     * PX4 param: FW_MAN_P_SC
     */
    float fw_man_p_sc = 1.0f;

    /**
     * Manual yaw scale factor.
     *
     * In STABILIZED mode this scales the yaw stick input which is passed
     * through as a direct body yaw rate command.
     *
     * PX4 param: FW_MAN_Y_SC
     */
    float fw_man_y_sc = 1.0f;

    /* ---- VTOL ---- */

    /**
     * Whether this vehicle is a VTOL tailsitter.
     *
     * When true, the attitude controller applies a 90-degree rotation
     * to the DCM so that the roll/pitch/yaw axes map correctly in
     * fixed-wing flight mode. Specifically, the first and third columns
     * of the rotation matrix are swapped (with a sign change) because
     * the tailsitter's body x-axis points up in hover but forward in
     * fixed-wing mode.
     *
     * PX4 param: derived from vehicle_status.is_vtol_tailsitter
     */
    bool is_vtol_tailsitter = false;

    /* ---- Airspeed ---- */

    /**
     * Trim (cruise) airspeed [m/s].
     *
     * Used as the airspeed reference for the yaw controller's coordinated
     * turn calculation, and as a fallback when no validated airspeed is
     * available.
     *
     * PX4 param: FW_AIRSPD_TRIM
     */
    float fw_airspd_trim = 15.0f;

    /**
     * Minimum airspeed [m/s].
     *
     * Below this speed the controller uses minimum airspeed for
     * coordinated turn calculations to avoid division by very small
     * numbers and unrealistically large rate commands.
     *
     * PX4 param: FW_AIRSPD_MIN
     */
    float fw_airspd_min = 10.0f;
};

/* ======================================================================
 * Main attitude controller class
 * ====================================================================== */

class FixedwingAttitudeControl {
public:
    FixedwingAttitudeControl() = default;
    ~FixedwingAttitudeControl() = default;

    /* ---- Subscription interface ----
     *
     * In the real PX4 firmware these would be uORB::Subscription objects
     * that automatically receive the latest published message. Here we
     * provide setter methods so that a test harness or simulation can
     * inject data before calling Run().
     */

    /** Provide current vehicle attitude (quaternion from EKF). */
    void set_vehicle_attitude(const uorb::vehicle_attitude_s& msg);

    /** Provide desired attitude from the path controller or mode logic. */
    void set_vehicle_attitude_setpoint(const uorb::vehicle_attitude_setpoint_s& msg);

    /** Provide current body angular rates from the gyroscope. */
    void set_vehicle_angular_velocity(const uorb::vehicle_angular_velocity_s& msg);

    /** Provide validated airspeed measurement. */
    void set_airspeed_validated(const uorb::airspeed_validated_s& msg);

    /** Provide vehicle status (nav state, VTOL flags, landed). */
    void set_vehicle_status(const uorb::vehicle_status_s& msg);

    /** Provide manual control stick inputs (for STABILIZED mode). */
    void set_manual_control_setpoint(const uorb::manual_control_setpoint_s& msg);

    /** Provide autotune status (excitation signal injection). */
    void set_autotune_attitude_control_status(const uorb::autotune_attitude_control_status_s& msg);

    /** Provide local position (used for ground speed, heading, etc.). */
    void set_vehicle_local_position(const uorb::vehicle_local_position_s& msg);

    /* ---- Parameter access ---- */

    /**
     * Get a mutable reference to the parameter struct.
     *
     * Callers can modify parameters directly. In the real PX4 firmware,
     * parameter updates are detected via a poll and applied in the Run()
     * loop. Here, parameters take effect on the next Run() call.
     */
    AttitudeControlParams& params() { return _params; }
    const AttitudeControlParams& params() const { return _params; }

    /* ---- Main execution ---- */

    /**
     * Execute one iteration of the attitude control loop.
     *
     * This is called at ~250 Hz (matching the attitude estimator output rate).
     * It reads the latest subscribed data, runs all sub-controllers, and
     * produces a vehicle_rates_setpoint.
     *
     * In the real PX4 firmware this is triggered by new attitude data via
     * a poll/callback mechanism. Here it must be called explicitly.
     */
    void Run();

    /* ---- Publication interface ---- */

    /**
     * Get the most recently computed rates setpoint.
     *
     * In the real PX4 firmware this would be published via uORB. Here the
     * caller retrieves it after Run() completes.
     */
    const uorb::vehicle_rates_setpoint_s& get_rates_setpoint() const { return _rates_setpoint; }

    /**
     * Get the (possibly modified) attitude setpoint.
     *
     * In STABILIZED mode, Run() overwrites the attitude setpoint from
     * stick inputs. This accessor exposes the result for logging/testing.
     */
    const uorb::vehicle_attitude_setpoint_s& get_attitude_setpoint() const { return _att_sp; }

private:

    /* ---- Sub-controllers ---- */

    RollController  _roll_ctrl;
    PitchController _pitch_ctrl;
    YawController   _yaw_ctrl;
    WheelController _wheel_ctrl;

    /* ---- Parameters ---- */

    AttitudeControlParams _params;

    /* ---- Subscribed data (latest messages) ---- */

    uorb::vehicle_attitude_s                  _vehicle_attitude{};
    uorb::vehicle_attitude_setpoint_s         _att_sp{};
    uorb::vehicle_angular_velocity_s          _angular_velocity{};
    uorb::airspeed_validated_s                _airspeed_validated{};
    uorb::vehicle_status_s                    _vehicle_status{};
    uorb::manual_control_setpoint_s           _manual_control{};
    uorb::autotune_attitude_control_status_s  _autotune_status{};
    uorb::vehicle_local_position_s            _local_position{};

    /* ---- Published data ---- */

    uorb::vehicle_rates_setpoint_s _rates_setpoint{};

    /* ---- Internal state ---- */

    /**
     * Flag indicating whether valid airspeed data has been received.
     *
     * When false, the controller falls back to using the trim airspeed
     * for all airspeed-dependent calculations (coordinated turn, etc.).
     */
    bool _airspeed_valid = false;

    /* ---- Private helper methods ---- */

    /**
     * Apply parameters to the sub-controllers.
     *
     * Called at the beginning of each Run() to ensure sub-controllers
     * are configured with the latest parameter values. In the real PX4
     * this only runs when the parameter server signals a change.
     */
    void apply_parameters();

    /**
     * Apply the VTOL tailsitter rotation to a DCM.
     *
     * For a tailsitter in fixed-wing mode, the body frame is rotated
     * 90 degrees about the pitch axis relative to a conventional aircraft.
     * This function swaps the first and third columns of the DCM (with a
     * sign flip on the original first column) so that Euler angle extraction
     * yields roll/pitch/yaw as if the aircraft were a conventional layout.
     *
     * Specifically, the transformation is:
     *   new_col0 = old_col2    (body-x gets what was body-z)
     *   new_col2 = -old_col0   (body-z gets negative of what was body-x)
     *
     * @param R  The rotation matrix to modify in-place.
     */
    void apply_tailsitter_transform(math::Matrix3f& R);

    /**
     * Convert manual stick inputs to attitude setpoints (STABILIZED mode).
     *
     * Mapping:
     *   - Roll stick  [-1,+1]  -->  roll angle  [-fw_man_r_max, +fw_man_r_max]
     *   - Pitch stick [-1,+1]  -->  pitch angle [-fw_man_p_max, +fw_man_p_max]
     *     (with sign inversion: stick-back = positive stick = pitch-UP = negative pitch)
     *   - Throttle stick        -->  forward thrust
     *   - Yaw stick is passed through as a direct rate command (scaled)
     *
     * The resulting setpoint is stored in _att_sp.
     */
    void generate_stabilized_setpoint();
};

} // namespace fw_att_control
