/**
 * @file LaunchDetector.cpp
 *
 * Implementation of catapult / hand-launch detection.
 *
 * See LaunchDetector.hpp for the full algorithm description, state machine
 * diagram, and parameter selection guidelines.
 *
 * The real PX4 equivalent is:
 *   src/modules/fw_pos_control/launchdetection/LaunchDetector.cpp
 */

#include "LaunchDetector.hpp"

#include <algorithm>
#include <cmath>

namespace fw_mode_manager {

/* ======================================================================
 * Configuration
 * ====================================================================== */

void LaunchDetector::set_acceleration_threshold(float threshold)
{
    /*
     * The threshold must be positive and large enough to avoid false
     * triggers from normal handling. Values below ~5 m/s^2 (0.5g) would
     * trigger from simply tilting the aircraft. We enforce a minimum of
     * 1.0 m/s^2 to prevent accidental launches.
     */
    _acceleration_threshold = std::max(threshold, 1.0f);
}

void LaunchDetector::set_trigger_duration(float duration)
{
    /*
     * Duration must be non-negative. A value of zero means the launch
     * triggers on the first sample exceeding the threshold (no filtering).
     * This may be acceptable for catapult launches with very clean signals
     * but is not recommended for hand launches.
     */
    _trigger_duration = std::max(duration, 0.0f);
}

void LaunchDetector::set_motor_delay(float delay)
{
    /*
     * Motor delay must be non-negative. A value of zero means the motor
     * starts immediately upon launch confirmation. For hand launches,
     * a delay of 0.3-0.5s is recommended for safety.
     */
    _motor_delay = std::max(delay, 0.0f);
}

/* ======================================================================
 * State machine update
 * ====================================================================== */

void LaunchDetector::update(float accel_x, float dt)
{
    /*
     * The state machine progresses through three stages:
     *
     *   WAITING -> DETECTED_ACCEL -> LAUNCHED
     *
     * Each transition is described in detail below.
     */

    switch (_state) {

    case LaunchState::WAITING:
        /*
         * WAITING state: Monitoring for the initial acceleration spike.
         *
         * The accelerometer reads body-x acceleration. In level resting
         * conditions, body-x sees only the component of gravity projected
         * onto the x-axis, which is typically near zero (gravity is along
         * body-z when level). During a hand launch, the operator throws
         * the aircraft forward, producing a large positive body-x
         * acceleration impulse.
         *
         * We compare against the threshold. If exceeded, we transition to
         * DETECTED_ACCEL and begin timing. If not, we remain in WAITING.
         */
        if (accel_x > _acceleration_threshold) {
            _state = LaunchState::DETECTED_ACCEL;
            _accel_duration = dt;
        }
        break;

    case LaunchState::DETECTED_ACCEL:
        /*
         * DETECTED_ACCEL state: Verifying the acceleration is sustained.
         *
         * A transient spike (e.g., the aircraft being bumped or set down)
         * will typically last only one or two samples (< 10ms at 250 Hz).
         * A genuine launch sustains high acceleration for 100-200ms.
         *
         * We accumulate the time that acceleration remains above threshold.
         * If the duration exceeds `trigger_duration`, the launch is
         * confirmed and we transition to LAUNCHED.
         *
         * If the acceleration drops below threshold before the duration
         * requirement is met, we fall back to WAITING. This prevents
         * false positives from short transient events.
         */
        if (accel_x > _acceleration_threshold) {
            _accel_duration += dt;

            if (_accel_duration >= _trigger_duration) {
                _state = LaunchState::LAUNCHED;
                _motor_delay_elapsed = 0.0f;
            }
        } else {
            /*
             * Acceleration dropped below threshold before the required
             * duration. Reset to WAITING. We clear the accumulated
             * duration so the next detection attempt starts fresh.
             */
            _state = LaunchState::WAITING;
            _accel_duration = 0.0f;
        }
        break;

    case LaunchState::LAUNCHED:
        /*
         * LAUNCHED state: Launch confirmed, waiting for motor delay.
         *
         * The launch has been confirmed but we may still need to wait
         * for the motor delay to elapse before allowing the motor to
         * spin up. This delay provides safety margin for the operator's
         * hand to clear the propeller arc.
         *
         * We accumulate time until the motor delay is satisfied. The
         * isLaunched() method checks both the state and the delay.
         *
         * This state is latched -- once launched, the detector stays
         * in LAUNCHED until explicitly reset by the mode manager.
         */
        _motor_delay_elapsed += dt;
        break;
    }
}

/* ======================================================================
 * Query methods
 * ====================================================================== */

bool LaunchDetector::isLaunched() const
{
    /*
     * Launch is fully confirmed only when:
     *   1. The state machine has reached LAUNCHED (acceleration spike
     *      was detected and sustained for the required duration)
     *   2. The motor delay has elapsed (safety clearance for operator)
     *
     * Both conditions must be met before the mode manager enables the
     * motor and transitions to climb-out.
     */
    return (_state == LaunchState::LAUNCHED) &&
           (_motor_delay_elapsed >= _motor_delay);
}

void LaunchDetector::reset()
{
    /*
     * Full reset of all internal state. Called when:
     *   - The vehicle is re-armed for a new launch attempt
     *   - The operator switches out of and back into takeoff mode
     *   - An aborted takeoff sequence is restarted
     */
    _state = LaunchState::WAITING;
    _accel_duration = 0.0f;
    _motor_delay_elapsed = 0.0f;
}

} // namespace fw_mode_manager
