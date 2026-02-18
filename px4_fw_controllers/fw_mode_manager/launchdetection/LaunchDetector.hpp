/**
 * @file LaunchDetector.hpp
 *
 * Catapult / hand-launch detection via accelerometer monitoring.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * Many small fixed-wing UAVs are launched by hand or catapult rather than
 * using a runway. During the pre-launch phase the aircraft sits idle with the
 * motor off. The autopilot must detect the moment of launch so it can:
 *
 *   1. Arm the motor (possibly after a configurable delay)
 *   2. Begin climbing to a safe altitude
 *   3. Transition from the takeoff state machine to normal flight
 *
 * The detection method is simple and robust: monitor the body-x axis
 * accelerometer for a high-g spike that exceeds a threshold for a minimum
 * duration. This reliably discriminates between a genuine launch event and
 * incidental handling (bumps, tilting, etc.).
 *
 * ============================================================================
 * DETECTION ALGORITHM
 * ============================================================================
 *
 * The detector runs a three-state machine:
 *
 *   WAITING  --[accel_x > threshold]--> DETECTED_ACCEL
 *                                           |
 *                                [acceleration sustained for trigger_duration]
 *                                           |
 *                                           v
 *                                       LAUNCHED
 *
 * State transitions:
 *
 *   WAITING -> DETECTED_ACCEL:
 *     Triggered when the body-x acceleration exceeds `acceleration_threshold`.
 *     Body-x is used because the launch impulse is predominantly along the
 *     aircraft's longitudinal axis (forward throw or catapult rail).
 *     A running timer begins accumulating the duration of the exceedance.
 *
 *   DETECTED_ACCEL -> WAITING (fallback):
 *     If the acceleration drops below threshold before `trigger_duration`
 *     has elapsed, the detector resets to WAITING. This prevents false
 *     triggers from transient bumps.
 *
 *   DETECTED_ACCEL -> LAUNCHED:
 *     Once the acceleration has been sustained above threshold for at least
 *     `trigger_duration`, the launch is confirmed. This state is latched --
 *     the detector stays in LAUNCHED until explicitly reset.
 *
 * The `motor_delay` parameter specifies an additional time after detection
 * before the motor should be started. This allows the aircraft to clear the
 * operator's hand or catapult mechanism before the propeller spins up,
 * preventing injury or mechanical interference.
 *
 * ============================================================================
 * PARAMETER SELECTION GUIDELINES
 * ============================================================================
 *
 *   acceleration_threshold (default 30.0 m/s^2 ~ 3g):
 *     A typical hand launch produces 3-8g along the body x-axis. Catapults
 *     can produce 10-20g. The threshold must be set high enough to avoid
 *     triggering from normal handling (~1-2g max) but low enough to
 *     reliably detect the launch. 30 m/s^2 (~3g) is a good default for
 *     hand launches.
 *
 *   trigger_duration (default 0.05s = 50ms):
 *     The sustained duration requirement filters out transient spikes
 *     (e.g., the aircraft being set down on a table). A hand launch
 *     typically sustains high-g for 100-200ms, so 50ms is conservative
 *     and ensures fast detection.
 *
 *   motor_delay (default 0.0s):
 *     Set to 0.3-0.5s for hand launches to clear the operator's hand.
 *     For catapult launches, 0.0s is typical since the aircraft is
 *     already clear of the operator.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *   - PX4 source: src/modules/fw_pos_control/launchdetection/LaunchDetector.cpp
 *   - PX4 parameter: FW_LAUN_AC_THLD, FW_LAUN_AC_T, FW_LAUN_MOT_DEL
 */

#pragma once

#include <cstdint>

namespace fw_mode_manager {

/**
 * Launch detection state machine states.
 *
 * These represent the progression from waiting for launch through
 * detection to confirmed launch.
 */
enum class LaunchState : uint8_t {
    WAITING         = 0,   ///< Idle, monitoring accelerometer
    DETECTED_ACCEL  = 1,   ///< Acceleration spike detected, timing duration
    LAUNCHED        = 2    ///< Launch confirmed (latched until reset)
};

class LaunchDetector {
public:
    LaunchDetector() = default;
    ~LaunchDetector() = default;

    /* ---- Configuration setters ---- */

    /**
     * Set the acceleration threshold for launch detection (FW_LAUN_AC_THLD).
     *
     * The body-x acceleration must exceed this value to begin the detection
     * timer. Units are m/s^2. Typical hand-launch values are 25-40 m/s^2.
     *
     * @param threshold  Acceleration threshold [m/s^2], must be > 0
     */
    void set_acceleration_threshold(float threshold);

    /**
     * Set the minimum duration the acceleration must be sustained (FW_LAUN_AC_T).
     *
     * The acceleration must remain above threshold for this duration before
     * the launch is confirmed. This filters out transient bumps.
     *
     * @param duration  Trigger duration [s], must be >= 0
     */
    void set_trigger_duration(float duration);

    /**
     * Set the motor start delay after launch detection (FW_LAUN_MOT_DEL).
     *
     * After the launch is confirmed, this additional delay must elapse
     * before the motor is started. Allows the aircraft to clear the
     * operator's hand.
     *
     * @param delay  Motor delay [s], must be >= 0
     */
    void set_motor_delay(float delay);

    /* ---- Runtime interface ---- */

    /**
     * Update the launch detector with a new accelerometer sample.
     *
     * This is the main function called once per control cycle. It advances
     * the state machine based on the current body-x acceleration reading.
     *
     * @param accel_x  Body-x axis acceleration [m/s^2] (positive = forward)
     * @param dt       Time step since last call [s]
     */
    void update(float accel_x, float dt);

    /**
     * Check whether the launch has been confirmed.
     *
     * Returns true once the state machine has reached LAUNCHED and the
     * motor delay has elapsed.
     *
     * @return  true if launch is confirmed and motor may be started
     */
    bool isLaunched() const;

    /**
     * Reset the detector to the WAITING state.
     *
     * Called when re-entering the takeoff mode or when the operator
     * re-arms the vehicle for another launch attempt.
     */
    void reset();

    /**
     * Get the current state of the launch detection state machine.
     *
     * @return  Current LaunchState enum value
     */
    LaunchState getLaunchState() const { return _state; }

private:
    /* ---- Parameters ---- */

    /** Acceleration threshold for launch detection [m/s^2] */
    float _acceleration_threshold = 30.0f;

    /** Minimum duration acceleration must be sustained [s] */
    float _trigger_duration = 0.05f;

    /** Delay after detection before motor start [s] */
    float _motor_delay = 0.0f;

    /* ---- Internal state ---- */

    /** Current state machine state */
    LaunchState _state = LaunchState::WAITING;

    /** Accumulated time that acceleration has exceeded threshold [s] */
    float _accel_duration = 0.0f;

    /** Accumulated time since launch was confirmed (for motor delay) [s] */
    float _motor_delay_elapsed = 0.0f;
};

} // namespace fw_mode_manager
