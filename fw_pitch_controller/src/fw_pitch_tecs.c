/**
 * @file fw_pitch_tecs.c
 *
 * Simplified TECS pitch-axis implementation.
 *
 * Total Energy Control System overview
 * =====================================
 *
 * Total specific energy:   E_T = V^2 / (2g) + h
 * Energy balance:          E_B = V^2 / (2g) - h
 *
 * - Total energy error  -> throttle command
 * - Energy balance error -> pitch command
 *
 * The pitch setpoint is derived from the energy balance error
 * so that pitch trades altitude for airspeed (or vice-versa)
 * depending on the weight_airspeed parameter.
 *
 * This is a simplified version suitable for testing; a production
 * implementation would also handle throttle, stall protection, and
 * underspeed logic.
 */

#include "fw_pitch_tecs.h"
#include <math.h>

#define GRAVITY_MSS     9.80665f
#define MIN_AIRSPEED    3.0f

static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void fw_tecs_init(fw_tecs_state_t *state)
{
    state->energy_error_integ  = 0.0f;
    state->balance_error_integ = 0.0f;
    state->alt_rate_filt       = 0.0f;
    state->prev_altitude       = 0.0f;
    state->initialized         = false;
}

float fw_tecs_update(
    fw_tecs_state_t        *state,
    const fw_tecs_params_t *params,
    float                   alt_sp,
    float                   alt_meas,
    float                   airspeed_sp,
    float                   airspeed,
    float                   dt)
{
    if (dt < 1e-6f) {
        return 0.0f;
    }

    /* Guard airspeed */
    if (airspeed < MIN_AIRSPEED) {
        airspeed = MIN_AIRSPEED;
    }
    if (airspeed_sp < MIN_AIRSPEED) {
        airspeed_sp = MIN_AIRSPEED;
    }

    /* --- Altitude rate (filtered) --- */
    float alt_rate = 0.0f;
    if (state->initialized) {
        alt_rate = (alt_meas - state->prev_altitude) / dt;
    }
    state->prev_altitude = alt_meas;

    /* Low-pass filter on altitude rate */
    float alpha = dt / (params->time_const * 0.1f + dt);
    state->alt_rate_filt += alpha * (alt_rate - state->alt_rate_filt);

    /* Clamp climb/sink rate */
    float alt_rate_sp = (alt_sp - alt_meas) / params->time_const;
    alt_rate_sp = clampf(alt_rate_sp, -params->sink_rate_max,
                                        params->climb_rate_max);

    /* --- Specific energy rates --- */
    /*  Potential energy rate: hdot / 1  (specific, per unit weight) */
    float spe_rate     = state->alt_rate_filt;
    float spe_rate_sp  = alt_rate_sp;

    /*  Kinetic energy rate: V * Vdot / g  (approximated from setpoint) */
    float ske_rate_sp  = 0.0f;  /* simplified: assume constant V target */
    float ske_rate     = 0.0f;

    /* Weight parameter: 0 = pure altitude, 2 = pure airspeed */
    float w = clampf(params->weight_airspeed, 0.0f, 2.0f);

    /* --- Energy balance error (drives pitch) --- */
    /*
     * balance = SPE - SKE  (when w=1 they are equally weighted)
     * Positive balance error -> pitch up (convert KE to PE)
     * Negative balance error -> pitch down (convert PE to KE)
     */
    float seb_rate_sp = (2.0f - w) * spe_rate_sp - w * ske_rate_sp;
    float seb_rate    = (2.0f - w) * spe_rate     - w * ske_rate;
    float seb_error   = seb_rate_sp - seb_rate;

    /* Integrate the balance error */
    state->balance_error_integ += seb_error * dt;
    state->balance_error_integ  = clampf(state->balance_error_integ,
                                         -params->integrator_max,
                                          params->integrator_max);

    /* --- Pitch setpoint from energy balance --- */
    /*
     * pitch_cmd = K * (seb_error + Ki * integral(seb_error))
     *
     * K is derived from the time constant and damping:
     *   K = 1 / (g * tau)   (dimensional analysis: [m/s^2 * s] = [m/s])
     */
    float gain = 1.0f / (GRAVITY_MSS * params->time_const);
    float pitch_cmd = gain * (seb_error * 2.0f * params->pitch_damping
                              + state->balance_error_integ);

    /* Clamp to pitch limits */
    pitch_cmd = clampf(pitch_cmd, params->pitch_min, params->pitch_max);

    state->initialized = true;

    return pitch_cmd;
}
