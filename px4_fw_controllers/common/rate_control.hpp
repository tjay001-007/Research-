/**
 * @file rate_control.hpp
 *
 * Generic 3-axis PID rate controller (shared library).
 *
 * In PX4 this lives in src/lib/rate_control/ and is used by both
 * multirotor and fixed-wing rate controllers. It implements a standard
 * PID controller with:
 *
 *   - Per-axis P, I, D, and feed-forward gains
 *   - Anti-windup via saturation flags from the control allocator
 *   - Derivative-on-measurement (avoids setpoint "kicks")
 *   - Configurable integral limits
 *
 * The output represents an angular *acceleration* setpoint [rad/s^2],
 * which the caller then scales by airspeed² and maps to actuator commands.
 */

#pragma once

#include "math_utils.hpp"

namespace control {

class RateControl {
public:
    RateControl() = default;

    /* ---- Configuration ---- */

    void setPidGains(const math::Vector3f& p,
                     const math::Vector3f& i,
                     const math::Vector3f& d) {
        _gain_p = p;
        _gain_i = i;
        _gain_d = d;
    }

    void setFeedForwardGain(const math::Vector3f& ff) { _gain_ff = ff; }

    void setIntegratorLimit(const math::Vector3f& lim) { _lim_int = lim; }

    /**
     * Set saturation flags from the control allocator.
     *
     * When an axis is positively saturated (actuator at max), the
     * integrator on that axis stops accumulating in the positive
     * direction, and vice versa. This is PX4's primary anti-windup
     * mechanism for the rate controller.
     */
    void setPositiveSaturationFlag(int axis, bool saturated) {
        _saturated_positive[axis] = saturated;
    }
    void setNegativeSaturationFlag(int axis, bool saturated) {
        _saturated_negative[axis] = saturated;
    }

    /**
     * Run one PID iteration on all three axes.
     *
     * @param rate        Measured body rates [rad/s] (from gyroscope)
     * @param rate_sp     Desired body rates [rad/s] (from attitude controller)
     * @param dt          Time step [s]
     * @return            Angular acceleration setpoint [rad/s^2]
     */
    math::Vector3f update(const math::Vector3f& rate,
                          const math::Vector3f& rate_sp,
                          float dt) {
        math::Vector3f output;

        for (int i = 0; i < 3; i++) {
            const float error = rate_sp(i) - rate(i);

            /* ---- Proportional ---- */
            const float p_term = _gain_p(i) * error;

            /* ---- Integral (with anti-windup) ----
             *
             * Anti-windup logic: if the allocator reports positive
             * saturation AND the error would push the integrator
             * further positive, we freeze it. Same for negative.
             */
            if (!(_saturated_positive[i] && error > 0.0f) &&
                !(_saturated_negative[i] && error < 0.0f)) {
                _integrator(i) += _gain_i(i) * error * dt;
            }
            _integrator(i) = math::constrain(_integrator(i),
                                              -_lim_int(i), _lim_int(i));

            /* ---- Derivative (on measurement, not setpoint) ----
             *
             * Using derivative-on-measurement avoids the "derivative kick"
             * that occurs when the setpoint changes abruptly:
             *   d_term = -Kd * d(rate_measured)/dt
             *
             * The negative sign is because we want to oppose the rate
             * of change of the measured quantity (damping effect).
             */
            float d_term = 0.0f;
            if (_initialized && dt > 1e-6f) {
                d_term = -_gain_d(i) * (rate(i) - _prev_rate(i)) / dt;
            }

            /* ---- Feed-forward ----
             *
             * Feed-forward acts on the desired rate directly. It provides
             * immediate response to commanded changes without waiting for
             * error to build up. Particularly important for fixed-wing
             * control where the plant has significant inertia.
             */
            const float ff_term = _gain_ff(i) * rate_sp(i);

            output(i) = p_term + _integrator(i) + d_term + ff_term;
        }

        _prev_rate = rate;
        _initialized = true;

        return output;
    }

    /** Reset all integrators (e.g., on landing or mode transition) */
    void resetIntegrator() { _integrator.zero(); }

    /** Get current integrator values (for telemetry) */
    math::Vector3f getIntegrator() const { return _integrator; }

private:
    /* Gains */
    math::Vector3f _gain_p;
    math::Vector3f _gain_i;
    math::Vector3f _gain_d;
    math::Vector3f _gain_ff;
    math::Vector3f _lim_int{0.4f, 0.4f, 0.4f};

    /* State */
    math::Vector3f _integrator;
    math::Vector3f _prev_rate;
    bool _initialized{false};

    /* Saturation flags from control allocator */
    bool _saturated_positive[3]{false, false, false};
    bool _saturated_negative[3]{false, false, false};
};

} // namespace control
