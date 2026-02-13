/**
 * @file fw_pitch_controller.c
 *
 * Top-level fixed-wing pitch attitude controller implementation.
 *
 * This file wires together the cascaded loops:
 *   TECS (optional) -> attitude (outer) -> rate (inner) -> elevator
 *
 * Mode transitions automatically reset the relevant integrators
 * to avoid transients.
 */

#include "fw_pitch_controller.h"

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void fw_pitch_ctrl_init(fw_pitch_ctrl_t *ctrl)
{
    ctrl->params      = fw_pitch_params_defaults();
    ctrl->tecs_params = fw_tecs_params_defaults();
    ctrl->mode        = FW_PITCH_MODE_MANUAL;
    ctrl->prev_mode   = FW_PITCH_MODE_MANUAL;

    fw_pitch_rate_ctrl_init(&ctrl->rate_state);
    fw_pitch_att_ctrl_init(&ctrl->att_state);
    fw_tecs_init(&ctrl->tecs_state);
}

void fw_pitch_ctrl_set_mode(fw_pitch_ctrl_t *ctrl, fw_pitch_mode_t mode)
{
    ctrl->prev_mode = ctrl->mode;
    ctrl->mode      = mode;

    /*
     * On any mode transition, reset integrators so stale state
     * from a previous mode doesn't cause a transient.
     */
    if (ctrl->mode != ctrl->prev_mode) {
        fw_pitch_rate_ctrl_reset_integrator(&ctrl->rate_state);
        fw_pitch_att_ctrl_reset_integrator(&ctrl->att_state);

        if (ctrl->mode == FW_PITCH_MODE_ALTITUDE) {
            fw_tecs_init(&ctrl->tecs_state);
        }
    }
}

fw_pitch_output_t fw_pitch_ctrl_update(
    fw_pitch_ctrl_t            *ctrl,
    const fw_pitch_inputs_t    *inputs,
    const fw_pitch_setpoints_t *setpoints,
    float                       dt)
{
    fw_pitch_output_t out;
    out.elevator = 0.0f;
    out.pitch_sp = 0.0f;
    out.rate_sp  = 0.0f;

    /* --- Guard dt --- */
    if (dt < ctrl->params.dt_min || dt > ctrl->params.dt_max) {
        return out;
    }

    /* ============================================================= */
    /*  MANUAL: no control                                            */
    /* ============================================================= */
    if (ctrl->mode == FW_PITCH_MODE_MANUAL) {
        return out;
    }

    /* ============================================================= */
    /*  RATE ONLY (Acro): inner loop only                             */
    /* ============================================================= */
    if (ctrl->mode == FW_PITCH_MODE_RATE_ONLY) {
        out.rate_sp  = setpoints->rate_sp;
        out.pitch_sp = inputs->pitch;  /* informational */

        out.elevator = fw_pitch_rate_ctrl_update(
            &ctrl->rate_state,
            &ctrl->params.rate,
            out.rate_sp,
            inputs->pitch_rate,
            dt);

        return out;
    }

    /* ============================================================= */
    /*  STABILIZE: attitude + rate loops                               */
    /* ============================================================= */
    if (ctrl->mode == FW_PITCH_MODE_STABILIZE) {
        out.pitch_sp = setpoints->pitch_sp;

        /* Outer loop: pitch error -> rate setpoint */
        out.rate_sp = fw_pitch_att_ctrl_update(
            &ctrl->att_state,
            &ctrl->params.att,
            &ctrl->params.coord,
            out.pitch_sp,
            inputs->pitch,
            inputs->roll,
            inputs->airspeed,
            dt);

        /* Inner loop: rate error -> elevator */
        out.elevator = fw_pitch_rate_ctrl_update(
            &ctrl->rate_state,
            &ctrl->params.rate,
            out.rate_sp,
            inputs->pitch_rate,
            dt);

        return out;
    }

    /* ============================================================= */
    /*  ALTITUDE: TECS -> attitude -> rate                            */
    /* ============================================================= */
    if (ctrl->mode == FW_PITCH_MODE_ALTITUDE) {
        /* TECS: altitude/airspeed errors -> pitch setpoint */
        out.pitch_sp = fw_tecs_update(
            &ctrl->tecs_state,
            &ctrl->tecs_params,
            setpoints->alt_sp,
            inputs->altitude,
            setpoints->airspeed_sp,
            inputs->airspeed,
            dt);

        /* Outer loop */
        out.rate_sp = fw_pitch_att_ctrl_update(
            &ctrl->att_state,
            &ctrl->params.att,
            &ctrl->params.coord,
            out.pitch_sp,
            inputs->pitch,
            inputs->roll,
            inputs->airspeed,
            dt);

        /* Inner loop */
        out.elevator = fw_pitch_rate_ctrl_update(
            &ctrl->rate_state,
            &ctrl->params.rate,
            out.rate_sp,
            inputs->pitch_rate,
            dt);

        return out;
    }

    return out;
}
