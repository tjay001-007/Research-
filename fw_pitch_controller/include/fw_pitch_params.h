/**
 * @file fw_pitch_params.h
 *
 * Fixed-wing pitch attitude controller parameters.
 *
 * Defines tunable gains and limits for the cascaded pitch controller
 * (outer attitude loop + inner rate loop) compatible with PX4 autopilot.
 *
 * Reference frame: NED (North-East-Down), body-frame pitch axis (Y-body).
 */

#ifndef FW_PITCH_PARAMS_H
#define FW_PITCH_PARAMS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Pitch rate (inner loop) PID gains.
 *
 * The rate controller drives elevator deflection to track a commanded
 * pitch rate (q_cmd). Output is normalized elevator: [-1, +1].
 */
typedef struct {
    float kp;            /**< Proportional gain [rad/s -> normalized]      */
    float ki;            /**< Integral gain     [rad   -> normalized]      */
    float kd;            /**< Derivative gain   [rad/s^2 -> normalized]    */
    float integrator_max; /**< Anti-windup integrator clamp (absolute)     */
    float rate_max;      /**< Maximum commanded pitch rate  [rad/s]        */
    float output_min;    /**< Minimum elevator output (normalized, typ -1) */
    float output_max;    /**< Maximum elevator output (normalized, typ +1) */
    float ff;            /**< Feed-forward gain on pitch rate command      */
} fw_pitch_rate_params_t;

/**
 * Pitch attitude (outer loop) P/PI gains.
 *
 * The attitude controller converts a pitch angle error into a
 * pitch rate setpoint fed to the inner loop.
 */
typedef struct {
    float kp;            /**< Proportional gain  [rad -> rad/s]            */
    float ki;            /**< Integral gain      [rad*s -> rad/s]          */
    float integrator_max; /**< Anti-windup integrator clamp [rad/s]        */
    float pitch_max;     /**< Maximum pitch setpoint       [rad]           */
    float pitch_min;     /**< Minimum pitch setpoint       [rad]           */
    float rate_max;      /**< Max rate output from attitude loop [rad/s]   */
} fw_pitch_att_params_t;

/**
 * Coordinated turn compensation parameters.
 */
typedef struct {
    bool  enabled;       /**< Enable gravity compensation in turns         */
    float time_const;    /**< Coordination filter time constant [s]        */
} fw_pitch_coord_params_t;

/**
 * Top-level pitch controller configuration.
 */
typedef struct {
    fw_pitch_rate_params_t  rate;   /**< Inner-loop (rate) gains           */
    fw_pitch_att_params_t   att;    /**< Outer-loop (attitude) gains       */
    fw_pitch_coord_params_t coord;  /**< Coordinated turn compensation     */
    float dt_min;                   /**< Minimum dt to accept [s]          */
    float dt_max;                   /**< Maximum dt to accept [s]          */
} fw_pitch_params_t;

/**
 * Return a default parameter set tuned for a small fixed-wing UAV
 * with the pitch-unstable aerodynamics in the companion Simulink model:
 *   Cmalpha = +0.3  (destabilizing)
 *   Cmq     = -3.0  (damping)
 *   mass    = 2.0 kg
 *   Iyy     = 0.1 kg*m^2
 */
static inline fw_pitch_params_t fw_pitch_params_defaults(void)
{
    fw_pitch_params_t p;

    /* --- Inner loop: pitch rate --- */
    p.rate.kp            = 0.4f;
    p.rate.ki            = 0.3f;
    p.rate.kd            = 0.015f;
    p.rate.integrator_max = 0.4f;
    p.rate.rate_max      = 2.094f;     /* 120 deg/s */
    p.rate.output_min    = -1.0f;
    p.rate.output_max    =  1.0f;
    p.rate.ff            = 0.0f;

    /* --- Outer loop: pitch attitude --- */
    p.att.kp             = 12.0f;
    p.att.ki             = 0.0f;
    p.att.integrator_max = 0.5f;
    p.att.pitch_max      = 0.7854f;    /* +45 deg */
    p.att.pitch_min      = -0.7854f;   /* -45 deg */
    p.att.rate_max       = 2.094f;     /* 120 deg/s */

    /* --- Coordinated turn compensation --- */
    p.coord.enabled      = true;
    p.coord.time_const   = 0.1f;

    /* --- Timing guard-rails --- */
    p.dt_min = 0.001f;   /* 1 ms  */
    p.dt_max = 0.050f;   /* 50 ms */

    return p;
}

#ifdef __cplusplus
}
#endif

#endif /* FW_PITCH_PARAMS_H */
