/**
 * @file fw_pitch_rate_ctrl.c
 *
 * Fixed-wing pitch RATE controller (inner loop) implementation.
 *
 * PID with:
 *  - Integral anti-windup (clamping + back-calculation awareness)
 *  - Derivative-on-measurement to avoid setpoint kick
 *  - Feed-forward on commanded rate
 *  - Hard output limits
 */

#include "fw_pitch_rate_ctrl.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void fw_pitch_rate_ctrl_init(fw_pitch_rate_state_t *state)
{
    state->integrator  = 0.0f;
    state->prev_rate   = 0.0f;
    state->initialized = false;
}

void fw_pitch_rate_ctrl_reset_integrator(fw_pitch_rate_state_t *state)
{
    state->integrator = 0.0f;
}

float fw_pitch_rate_ctrl_update(
    fw_pitch_rate_state_t       *state,
    const fw_pitch_rate_params_t *params,
    float                        rate_sp,
    float                        rate_meas,
    float                        dt)
{
    /* --- Guard against pathological dt --- */
    if (dt < 1e-6f) {
        return 0.0f;
    }

    /* Clamp the commanded rate */
    rate_sp = clampf(rate_sp, -params->rate_max, params->rate_max);

    /* Rate error */
    const float rate_error = rate_sp - rate_meas;

    /* --- Proportional term --- */
    const float p_term = params->kp * rate_error;

    /* --- Integral term (trapezoidal, with anti-windup clamp) --- */
    state->integrator += params->ki * rate_error * dt;
    state->integrator  = clampf(state->integrator,
                                -params->integrator_max,
                                 params->integrator_max);
    const float i_term = state->integrator;

    /* --- Derivative term (on measurement, not setpoint) --- */
    float d_term = 0.0f;
    if (state->initialized) {
        /*
         * d/dt(measurement) approximated by backward difference.
         * Negative sign: we want to oppose rate-of-change of the
         * measured rate (damping).
         */
        d_term = -params->kd * (rate_meas - state->prev_rate) / dt;
    }
    state->prev_rate   = rate_meas;
    state->initialized = true;

    /* --- Feed-forward --- */
    const float ff_term = params->ff * rate_sp;

    /* --- Total output --- */
    float output = p_term + i_term + d_term + ff_term;
    output = clampf(output, params->output_min, params->output_max);

    /*
     * Conditional integration: freeze integrator when output is
     * saturated AND the integrator would push it further.
     */
    if ((output >= params->output_max && rate_error > 0.0f) ||
        (output <= params->output_min && rate_error < 0.0f)) {
        /* Undo the integration step we just did */
        state->integrator -= params->ki * rate_error * dt;
    }

    return output;
}
