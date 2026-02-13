/**
 * @file fw_pitch_att_ctrl.h
 *
 * Fixed-wing pitch ATTITUDE controller (outer loop).
 *
 * Converts a pitch angle setpoint into a pitch-rate setpoint
 * using a P (or PI) controller, with coordinated-turn gravity
 * compensation.
 *
 * The output rate setpoint is consumed by the inner rate loop
 * (fw_pitch_rate_ctrl).
 */

#ifndef FW_PITCH_ATT_CTRL_H
#define FW_PITCH_ATT_CTRL_H

#include "fw_pitch_params.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Internal state of the pitch-attitude controller.
 */
typedef struct {
    float integrator;        /**< Integral of pitch error [rad*s]       */
    float coord_rate_filt;   /**< Filtered coordination rate [rad/s]    */
    bool  initialized;
} fw_pitch_att_state_t;

/**
 * Initialise / reset the attitude controller state.
 */
void fw_pitch_att_ctrl_init(fw_pitch_att_state_t *state);

/**
 * Reset the integrator (e.g. on mode switch).
 */
void fw_pitch_att_ctrl_reset_integrator(fw_pitch_att_state_t *state);

/**
 * Run one iteration of the pitch-attitude outer loop.
 *
 * @param state       Controller state (updated in place).
 * @param att_params  Attitude-loop gains and limits.
 * @param coord_params Coordinated-turn parameters.
 * @param pitch_sp    Desired pitch angle  [rad].
 * @param pitch_meas  Measured pitch angle [rad] (theta from AHRS).
 * @param roll_meas   Measured roll angle  [rad] (phi, for turn comp).
 * @param airspeed    True airspeed        [m/s] (for gravity comp).
 * @param dt          Time step            [s].
 * @return            Commanded pitch rate [rad/s] for the inner loop.
 */
float fw_pitch_att_ctrl_update(
    fw_pitch_att_state_t          *state,
    const fw_pitch_att_params_t   *att_params,
    const fw_pitch_coord_params_t *coord_params,
    float                          pitch_sp,
    float                          pitch_meas,
    float                          roll_meas,
    float                          airspeed,
    float                          dt);

#ifdef __cplusplus
}
#endif

#endif /* FW_PITCH_ATT_CTRL_H */
