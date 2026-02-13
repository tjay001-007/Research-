/**
 * @file fw_pitch_att_ctrl.c
 *
 * Fixed-wing pitch ATTITUDE controller (outer loop) implementation.
 *
 * Architecture
 * ============
 *
 *  pitch_sp ──┐
 *             ├─ error ─► P(+I) ─► q_cmd_base
 *  pitch_meas ┘                       │
 *                                     + ── q_coord ──► q_cmd (rate setpoint)
 *                                     │
 *                         turn-compensation
 *                       (gravity component)
 *
 * Turn compensation
 * -----------------
 * In a coordinated turn the aircraft must pitch up to compensate
 * for the load-factor increase: n = 1/cos(phi).  The additional
 * pitch rate required is:
 *
 *     q_coord = g / V * (1/cos(phi) - 1)         [rad/s]
 *
 * This feed-forward reduces altitude loss in banked turns.
 */

#include "fw_pitch_att_ctrl.h"
#include <math.h>

#define GRAVITY_MSS  9.80665f
#define MIN_AIRSPEED 3.0f       /* [m/s] avoid division by tiny V */

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static inline float wrap_pi(float angle)
{
    /* Wrap angle to [-pi, pi) */
    while (angle >  (float)M_PI) angle -= 2.0f * (float)M_PI;
    while (angle < -(float)M_PI) angle += 2.0f * (float)M_PI;
    return angle;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void fw_pitch_att_ctrl_init(fw_pitch_att_state_t *state)
{
    state->integrator      = 0.0f;
    state->coord_rate_filt = 0.0f;
    state->initialized     = false;
}

void fw_pitch_att_ctrl_reset_integrator(fw_pitch_att_state_t *state)
{
    state->integrator = 0.0f;
}

float fw_pitch_att_ctrl_update(
    fw_pitch_att_state_t          *state,
    const fw_pitch_att_params_t   *att_params,
    const fw_pitch_coord_params_t *coord_params,
    float                          pitch_sp,
    float                          pitch_meas,
    float                          roll_meas,
    float                          airspeed,
    float                          dt)
{
    if (dt < 1e-6f) {
        return 0.0f;
    }

    /* --- Clamp the pitch setpoint --- */
    pitch_sp = clampf(pitch_sp, att_params->pitch_min, att_params->pitch_max);

    /* --- Pitch error (shortest path) --- */
    const float pitch_error = wrap_pi(pitch_sp - pitch_meas);

    /* --- Proportional term --- */
    float q_cmd = att_params->kp * pitch_error;

    /* --- Optional integral term --- */
    if (att_params->ki > 0.0f) {
        state->integrator += att_params->ki * pitch_error * dt;
        state->integrator  = clampf(state->integrator,
                                    -att_params->integrator_max,
                                     att_params->integrator_max);
        q_cmd += state->integrator;
    }

    /* --- Coordinated-turn gravity compensation --- */
    if (coord_params->enabled) {
        float v = airspeed;
        if (v < MIN_AIRSPEED) {
            v = MIN_AIRSPEED;
        }

        float cos_roll = cosf(roll_meas);
        /* Protect against extreme bank angles */
        if (fabsf(cos_roll) < 0.2f) {
            cos_roll = (cos_roll >= 0.0f) ? 0.2f : -0.2f;
        }

        /*
         * Required pitch rate to sustain altitude in a banked turn:
         *   q_coord = (g / V) * (1/cos(phi) - 1)
         */
        float q_coord_raw = (GRAVITY_MSS / v) * (1.0f / cos_roll - 1.0f);

        /* First-order low-pass filter on the coordination term */
        float alpha = dt / (coord_params->time_const + dt);
        state->coord_rate_filt += alpha * (q_coord_raw - state->coord_rate_filt);

        q_cmd += state->coord_rate_filt;
    }

    /* --- Clamp output rate --- */
    q_cmd = clampf(q_cmd, -att_params->rate_max, att_params->rate_max);

    state->initialized = true;

    return q_cmd;
}
