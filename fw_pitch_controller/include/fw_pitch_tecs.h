/**
 * @file fw_pitch_tecs.h
 *
 * Simplified TECS (Total Energy Control System) pitch-axis interface.
 *
 * TECS manages the energy balance between kinetic (airspeed) and
 * potential (altitude) energy by commanding pitch and throttle.
 * This module provides the pitch setpoint output that TECS would
 * produce, simplified for integration testing.
 *
 * In a full PX4 stack the TECS module lives separately; this file
 * provides a lightweight stand-alone version so the pitch controller
 * can be tested end-to-end with altitude/airspeed targets.
 */

#ifndef FW_PITCH_TECS_H
#define FW_PITCH_TECS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * TECS tuning parameters (pitch axis only).
 */
typedef struct {
    float weight_airspeed;   /**< 0..2 : 0=full alt priority, 2=full spd  */
    float time_const;        /**< Energy-balance time constant [s]         */
    float pitch_damping;     /**< Damping ratio on energy error            */
    float integrator_max;    /**< Energy integrator clamp                  */
    float pitch_max;         /**< Upper pitch limit [rad]                  */
    float pitch_min;         /**< Lower pitch limit [rad] (negative=dive)  */
    float sink_rate_max;     /**< Maximum commanded sink rate [m/s]        */
    float climb_rate_max;    /**< Maximum commanded climb rate [m/s]       */
} fw_tecs_params_t;

/**
 * TECS controller state.
 */
typedef struct {
    float energy_error_integ;     /**< Integrated total energy error       */
    float balance_error_integ;    /**< Integrated energy balance error     */
    float alt_rate_filt;          /**< Filtered altitude rate [m/s]        */
    float prev_altitude;          /**< Previous altitude for rate calc     */
    bool  initialized;
} fw_tecs_state_t;

/**
 * Return sensible defaults for TECS pitch parameters.
 */
static inline fw_tecs_params_t fw_tecs_params_defaults(void)
{
    fw_tecs_params_t p;
    p.weight_airspeed = 1.0f;
    p.time_const      = 5.0f;
    p.pitch_damping   = 0.7f;
    p.integrator_max  = 0.3f;
    p.pitch_max       = 0.4363f;   /* 25 deg */
    p.pitch_min       = -0.3491f;  /* -20 deg */
    p.sink_rate_max   = 3.0f;
    p.climb_rate_max  = 5.0f;
    return p;
}

/**
 * Initialise / reset TECS state.
 */
void fw_tecs_init(fw_tecs_state_t *state);

/**
 * Run one TECS pitch-axis iteration.
 *
 * @param state       TECS state (updated in place).
 * @param params      TECS tuning parameters.
 * @param alt_sp      Desired altitude [m] (NED, so negative = up).
 * @param alt_meas    Measured altitude [m].
 * @param airspeed_sp Desired true airspeed [m/s].
 * @param airspeed    Measured true airspeed [m/s].
 * @param dt          Time step [s].
 * @return            Commanded pitch angle [rad] for the attitude loop.
 */
float fw_tecs_update(
    fw_tecs_state_t      *state,
    const fw_tecs_params_t *params,
    float                  alt_sp,
    float                  alt_meas,
    float                  airspeed_sp,
    float                  airspeed,
    float                  dt);

#ifdef __cplusplus
}
#endif

#endif /* FW_PITCH_TECS_H */
