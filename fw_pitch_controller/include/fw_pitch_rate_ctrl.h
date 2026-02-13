/**
 * @file fw_pitch_rate_ctrl.h
 *
 * Fixed-wing pitch RATE controller (inner loop).
 *
 * Implements a PID controller that converts a pitch-rate error
 * into a normalized elevator command [-1, +1].
 *
 * Features:
 *  - Anti-windup integrator with clamping
 *  - Derivative-on-measurement (avoids setpoint kick)
 *  - Feed-forward on rate command
 *  - Output saturation
 */

#ifndef FW_PITCH_RATE_CTRL_H
#define FW_PITCH_RATE_CTRL_H

#include "fw_pitch_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Internal state of the pitch-rate controller.
 */
typedef struct {
    float integrator;       /**< Accumulated integral term               */
    float prev_rate;        /**< Previous pitch rate measurement [rad/s] */
    bool  initialized;      /**< First-call flag                         */
} fw_pitch_rate_state_t;

/**
 * Initialise / reset the rate controller state.
 */
void fw_pitch_rate_ctrl_init(fw_pitch_rate_state_t *state);

/**
 * Reset the integrator to zero (e.g. on mode transition).
 */
void fw_pitch_rate_ctrl_reset_integrator(fw_pitch_rate_state_t *state);

/**
 * Run one iteration of the pitch-rate PID.
 *
 * @param state       Controller state (updated in place).
 * @param params      Rate-loop gains and limits.
 * @param rate_sp     Commanded pitch rate  [rad/s].
 * @param rate_meas   Measured pitch rate   [rad/s] (gyro q).
 * @param dt          Time step             [s].
 * @return            Normalised elevator command [-1, +1].
 */
float fw_pitch_rate_ctrl_update(
    fw_pitch_rate_state_t       *state,
    const fw_pitch_rate_params_t *params,
    float                        rate_sp,
    float                        rate_meas,
    float                        dt);

#ifdef __cplusplus
}
#endif

#endif /* FW_PITCH_RATE_CTRL_H */
