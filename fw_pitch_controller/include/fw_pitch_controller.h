/**
 * @file fw_pitch_controller.h
 *
 * Top-level fixed-wing pitch attitude controller.
 *
 * Combines the cascaded control loops into a single callable
 * interface:
 *
 *   ┌──────────────────────────────────────────────────────┐
 *   │  TECS (optional)                                     │
 *   │  alt_sp / aspd_sp ──► pitch_sp                       │
 *   └─────────────┬────────────────────────────────────────┘
 *                 │ pitch_sp
 *   ┌─────────────▼────────────────────────────────────────┐
 *   │  Attitude (outer) loop                               │
 *   │  pitch_sp, pitch_meas ──► rate_sp (q_cmd)            │
 *   │  + coordinated-turn compensation                     │
 *   └─────────────┬────────────────────────────────────────┘
 *                 │ rate_sp
 *   ┌─────────────▼────────────────────────────────────────┐
 *   │  Rate (inner) loop                                   │
 *   │  rate_sp, rate_meas ──► elevator_cmd [-1,+1]         │
 *   └─────────────┬────────────────────────────────────────┘
 *                 │
 *                 ▼  elevator command
 *
 * Operating modes
 * ===============
 *  MANUAL      - controller inactive, pass-through
 *  STABILIZE   - attitude + rate loops active, pilot sets pitch_sp
 *  ALTITUDE    - TECS drives pitch_sp from altitude / airspeed
 *  RATE_ONLY   - only the inner rate loop is active (acro)
 */

#ifndef FW_PITCH_CONTROLLER_H
#define FW_PITCH_CONTROLLER_H

#include "fw_pitch_params.h"
#include "fw_pitch_rate_ctrl.h"
#include "fw_pitch_att_ctrl.h"
#include "fw_pitch_tecs.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Controller operating mode.
 */
typedef enum {
    FW_PITCH_MODE_MANUAL    = 0,  /**< No control, pass-through          */
    FW_PITCH_MODE_RATE_ONLY = 1,  /**< Inner rate loop only (acro)       */
    FW_PITCH_MODE_STABILIZE = 2,  /**< Attitude + rate loops             */
    FW_PITCH_MODE_ALTITUDE  = 3,  /**< TECS + attitude + rate loops      */
} fw_pitch_mode_t;

/**
 * Sensor / state inputs required each iteration.
 */
typedef struct {
    float pitch;       /**< Euler pitch angle (theta) [rad]               */
    float roll;        /**< Euler roll  angle (phi)   [rad]               */
    float pitch_rate;  /**< Body pitch rate (q)        [rad/s]            */
    float airspeed;    /**< True airspeed              [m/s]              */
    float altitude;    /**< Altitude (positive up, or NED -z) [m]        */
} fw_pitch_inputs_t;

/**
 * Setpoints / references.
 */
typedef struct {
    float pitch_sp;    /**< Desired pitch angle [rad]  (STABILIZE)        */
    float rate_sp;     /**< Desired pitch rate  [rad/s] (RATE_ONLY)       */
    float alt_sp;      /**< Desired altitude    [m]     (ALTITUDE)        */
    float airspeed_sp; /**< Desired airspeed    [m/s]   (ALTITUDE)        */
} fw_pitch_setpoints_t;

/**
 * Controller output.
 */
typedef struct {
    float elevator;    /**< Normalized elevator command [-1, +1]          */
    float pitch_sp;    /**< Actual pitch setpoint used  [rad]             */
    float rate_sp;     /**< Actual rate setpoint used   [rad/s]           */
} fw_pitch_output_t;

/**
 * Complete controller instance (parameters + all sub-states).
 */
typedef struct {
    fw_pitch_params_t       params;
    fw_pitch_rate_state_t   rate_state;
    fw_pitch_att_state_t    att_state;
    fw_tecs_state_t         tecs_state;
    fw_tecs_params_t        tecs_params;
    fw_pitch_mode_t         mode;
    fw_pitch_mode_t         prev_mode;
} fw_pitch_ctrl_t;

/**
 * Initialise the controller with default parameters.
 */
void fw_pitch_ctrl_init(fw_pitch_ctrl_t *ctrl);

/**
 * Set operating mode (triggers integrator resets on transition).
 */
void fw_pitch_ctrl_set_mode(fw_pitch_ctrl_t *ctrl, fw_pitch_mode_t mode);

/**
 * Run one control iteration.
 *
 * @param ctrl        Controller instance.
 * @param inputs      Sensor measurements.
 * @param setpoints   Desired references.
 * @param dt          Time step [s].
 * @return            Controller output.
 */
fw_pitch_output_t fw_pitch_ctrl_update(
    fw_pitch_ctrl_t          *ctrl,
    const fw_pitch_inputs_t  *inputs,
    const fw_pitch_setpoints_t *setpoints,
    float                     dt);

#ifdef __cplusplus
}
#endif

#endif /* FW_PITCH_CONTROLLER_H */
