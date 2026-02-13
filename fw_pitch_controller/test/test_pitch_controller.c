/**
 * @file test_pitch_controller.c
 *
 * Unit tests for the PX4 fixed-wing pitch attitude controller.
 *
 * Simple test harness (no external test framework required).
 * Returns 0 on success, 1 on failure.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "fw_pitch_controller.h"
#include "fw_pitch_rate_ctrl.h"
#include "fw_pitch_att_ctrl.h"
#include "fw_pitch_tecs.h"

/* ------------------------------------------------------------------ */
/* Test infrastructure                                                 */
/* ------------------------------------------------------------------ */

static int tests_run    = 0;
static int tests_passed = 0;
static int tests_failed = 0;

#define ASSERT_TRUE(cond, msg) do {                                     \
    tests_run++;                                                        \
    if (!(cond)) {                                                      \
        printf("  FAIL: %s (line %d)\n", msg, __LINE__);                \
        tests_failed++;                                                 \
    } else {                                                            \
        tests_passed++;                                                 \
    }                                                                   \
} while(0)

#define ASSERT_NEAR(a, b, tol, msg) \
    ASSERT_TRUE(fabsf((a) - (b)) < (tol), msg)

#define RUN_TEST(fn) do {                                               \
    printf("[TEST] %s\n", #fn);                                         \
    fn();                                                               \
} while(0)

/* ------------------------------------------------------------------ */
/* Test: Rate controller basic functionality                           */
/* ------------------------------------------------------------------ */

static void test_rate_ctrl_zero_error(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;

    fw_pitch_rate_ctrl_init(&state);

    /* Zero error -> zero output */
    float out = fw_pitch_rate_ctrl_update(&state, &params, 0.0f, 0.0f, 0.004f);
    ASSERT_NEAR(out, 0.0f, 1e-5f, "zero rate error produces zero output");
}

static void test_rate_ctrl_positive_error(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;

    fw_pitch_rate_ctrl_init(&state);

    /* Positive rate error -> positive elevator */
    float out = fw_pitch_rate_ctrl_update(&state, &params, 1.0f, 0.0f, 0.004f);
    ASSERT_TRUE(out > 0.0f, "positive rate error gives positive output");
}

static void test_rate_ctrl_negative_error(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;

    fw_pitch_rate_ctrl_init(&state);

    /* Negative rate error -> negative elevator */
    float out = fw_pitch_rate_ctrl_update(&state, &params, -1.0f, 0.0f, 0.004f);
    ASSERT_TRUE(out < 0.0f, "negative rate error gives negative output");
}

static void test_rate_ctrl_output_saturation(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;
    /* Remove rate clamping to test pure output saturation */
    params.rate_max = 1000.0f;

    fw_pitch_rate_ctrl_init(&state);

    /* Large error should saturate at +1 */
    float out = fw_pitch_rate_ctrl_update(&state, &params, 100.0f, 0.0f, 0.004f);
    ASSERT_NEAR(out, 1.0f, 1e-5f, "output saturates at +1.0");

    /* Reset and test negative saturation */
    fw_pitch_rate_ctrl_init(&state);
    out = fw_pitch_rate_ctrl_update(&state, &params, -100.0f, 0.0f, 0.004f);
    ASSERT_NEAR(out, -1.0f, 1e-5f, "output saturates at -1.0");
}

static void test_rate_ctrl_integrator_accumulates(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;
    params.kp = 0.0f;  /* Disable P to isolate I */
    params.kd = 0.0f;  /* Disable D */

    fw_pitch_rate_ctrl_init(&state);

    float prev_out = 0.0f;
    for (int i = 0; i < 10; i++) {
        float out = fw_pitch_rate_ctrl_update(&state, &params,
                                              0.5f, 0.0f, 0.004f);
        ASSERT_TRUE(out >= prev_out, "integrator accumulates over time");
        prev_out = out;
    }
}

static void test_rate_ctrl_integrator_windup(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;
    params.kp = 0.0f;
    params.kd = 0.0f;
    params.integrator_max = 0.1f;

    fw_pitch_rate_ctrl_init(&state);

    /* Run many iterations with constant error */
    for (int i = 0; i < 10000; i++) {
        fw_pitch_rate_ctrl_update(&state, &params, 1.0f, 0.0f, 0.004f);
    }

    ASSERT_TRUE(state.integrator <= params.integrator_max + 1e-5f,
                "integrator respects windup limit");
}

static void test_rate_ctrl_zero_dt_safe(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;

    fw_pitch_rate_ctrl_init(&state);

    float out = fw_pitch_rate_ctrl_update(&state, &params, 1.0f, 0.0f, 0.0f);
    ASSERT_NEAR(out, 0.0f, 1e-5f, "zero dt returns zero (no crash)");
}

static void test_rate_ctrl_reset_integrator(void)
{
    fw_pitch_rate_state_t  state;
    fw_pitch_rate_params_t params = fw_pitch_params_defaults().rate;

    fw_pitch_rate_ctrl_init(&state);

    /* Accumulate some integrator */
    for (int i = 0; i < 100; i++) {
        fw_pitch_rate_ctrl_update(&state, &params, 1.0f, 0.0f, 0.004f);
    }
    ASSERT_TRUE(state.integrator != 0.0f, "integrator is non-zero before reset");

    fw_pitch_rate_ctrl_reset_integrator(&state);
    ASSERT_NEAR(state.integrator, 0.0f, 1e-6f, "integrator zeroed after reset");
}

/* ------------------------------------------------------------------ */
/* Test: Attitude controller                                           */
/* ------------------------------------------------------------------ */

static void test_att_ctrl_zero_error(void)
{
    fw_pitch_att_state_t    state;
    fw_pitch_params_t       p = fw_pitch_params_defaults();

    fw_pitch_att_ctrl_init(&state);

    float q = fw_pitch_att_ctrl_update(&state, &p.att, &p.coord,
                                       0.1f, 0.1f,   /* pitch_sp == pitch_meas */
                                       0.0f,          /* roll */
                                       15.0f,         /* airspeed */
                                       0.004f);
    ASSERT_NEAR(q, 0.0f, 0.01f, "zero pitch error gives ~zero rate cmd");
}

static void test_att_ctrl_nose_up_command(void)
{
    fw_pitch_att_state_t    state;
    fw_pitch_params_t       p = fw_pitch_params_defaults();
    p.coord.enabled = false;  /* Disable turn comp for clarity */

    fw_pitch_att_ctrl_init(&state);

    /* Command pitch up (sp > meas) */
    float q = fw_pitch_att_ctrl_update(&state, &p.att, &p.coord,
                                       0.2f, 0.0f,   /* 0.2 rad up */
                                       0.0f, 15.0f, 0.004f);
    ASSERT_TRUE(q > 0.0f, "positive pitch error -> positive rate command");
}

static void test_att_ctrl_nose_down_command(void)
{
    fw_pitch_att_state_t    state;
    fw_pitch_params_t       p = fw_pitch_params_defaults();
    p.coord.enabled = false;

    fw_pitch_att_ctrl_init(&state);

    /* Command pitch down (sp < meas) */
    float q = fw_pitch_att_ctrl_update(&state, &p.att, &p.coord,
                                       -0.1f, 0.1f,
                                       0.0f, 15.0f, 0.004f);
    ASSERT_TRUE(q < 0.0f, "negative pitch error -> negative rate command");
}

static void test_att_ctrl_turn_compensation(void)
{
    fw_pitch_att_state_t    state;
    fw_pitch_params_t       p = fw_pitch_params_defaults();
    p.coord.enabled = true;

    fw_pitch_att_ctrl_init(&state);

    /* Banked 45 degrees, zero pitch error. Turn comp should push q > 0 */
    float q = fw_pitch_att_ctrl_update(&state, &p.att, &p.coord,
                                       0.0f, 0.0f,     /* zero pitch error */
                                       0.7854f,         /* 45 deg roll */
                                       15.0f, 0.004f);
    ASSERT_TRUE(q > 0.0f, "turn compensation adds positive pitch rate in bank");
}

static void test_att_ctrl_pitch_limits(void)
{
    fw_pitch_att_state_t    state;
    fw_pitch_params_t       p = fw_pitch_params_defaults();
    p.coord.enabled = false;

    fw_pitch_att_ctrl_init(&state);

    /* Command far beyond limits */
    float q = fw_pitch_att_ctrl_update(&state, &p.att, &p.coord,
                                       2.0f, 0.0f,    /* 2 rad >> 45 deg limit */
                                       0.0f, 15.0f, 0.004f);
    /* The rate should be bounded by rate_max */
    ASSERT_TRUE(q <= p.att.rate_max + 1e-5f,
                "output rate respects rate_max");
}

/* ------------------------------------------------------------------ */
/* Test: TECS pitch-axis                                               */
/* ------------------------------------------------------------------ */

static void test_tecs_level_flight(void)
{
    fw_tecs_state_t  state;
    fw_tecs_params_t params = fw_tecs_params_defaults();

    fw_tecs_init(&state);

    /* Already at target altitude & airspeed */
    float pitch = fw_tecs_update(&state, &params,
                                 100.0f, 100.0f,   /* alt sp == meas */
                                 15.0f, 15.0f,     /* aspd sp == meas */
                                 0.004f);

    /* First iteration has no rate info, so expect ~0 */
    ASSERT_NEAR(pitch, 0.0f, 0.05f,
                "level flight at target -> near-zero pitch cmd");
}

static void test_tecs_climb_command(void)
{
    fw_tecs_state_t  state;
    fw_tecs_params_t params = fw_tecs_params_defaults();

    fw_tecs_init(&state);

    /* Prime with first sample */
    fw_tecs_update(&state, &params,
                   110.0f, 100.0f,    /* need to climb 10 m */
                   15.0f, 15.0f,
                   0.004f);

    /* Second iteration -> should command pitch up */
    float pitch = fw_tecs_update(&state, &params,
                                 110.0f, 100.0f,
                                 15.0f, 15.0f,
                                 0.004f);

    ASSERT_TRUE(pitch > 0.0f, "altitude deficit -> positive (up) pitch");
}

static void test_tecs_descend_command(void)
{
    fw_tecs_state_t  state;
    fw_tecs_params_t params = fw_tecs_params_defaults();

    fw_tecs_init(&state);

    /* Prime */
    fw_tecs_update(&state, &params,
                   90.0f, 100.0f,     /* need to descend 10 m */
                   15.0f, 15.0f,
                   0.004f);

    float pitch = fw_tecs_update(&state, &params,
                                 90.0f, 100.0f,
                                 15.0f, 15.0f,
                                 0.004f);

    ASSERT_TRUE(pitch < 0.0f, "altitude surplus -> negative (down) pitch");
}

static void test_tecs_pitch_limits(void)
{
    fw_tecs_state_t  state;
    fw_tecs_params_t params = fw_tecs_params_defaults();

    fw_tecs_init(&state);

    /* Prime */
    fw_tecs_update(&state, &params,
                   1000.0f, 0.0f,    /* huge altitude deficit */
                   15.0f, 15.0f,
                   0.004f);

    float pitch = fw_tecs_update(&state, &params,
                                 1000.0f, 0.0f,
                                 15.0f, 15.0f,
                                 0.004f);

    ASSERT_TRUE(pitch <= params.pitch_max + 1e-5f,
                "TECS respects upper pitch limit");
}

/* ------------------------------------------------------------------ */
/* Test: Top-level controller                                          */
/* ------------------------------------------------------------------ */

static void test_ctrl_manual_no_output(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_MANUAL);

    fw_pitch_inputs_t    in  = { .pitch = 0.1f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .pitch_sp = 0.2f };

    fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);
    ASSERT_NEAR(out.elevator, 0.0f, 1e-5f,
                "MANUAL mode outputs zero elevator");
}

static void test_ctrl_stabilize_corrects(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_STABILIZE);

    fw_pitch_inputs_t    in  = { .pitch = -0.1f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .pitch_sp = 0.1f };

    fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);
    ASSERT_TRUE(out.elevator > 0.0f,
                "STABILIZE corrects pitch-down error with positive elevator");
    ASSERT_TRUE(out.rate_sp > 0.0f,
                "STABILIZE generates positive rate setpoint for pitch-up");
}

static void test_ctrl_rate_only(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_RATE_ONLY);

    fw_pitch_inputs_t    in  = { .pitch = 0.0f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .rate_sp = 1.0f };

    fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);
    ASSERT_TRUE(out.elevator > 0.0f,
                "RATE_ONLY: positive rate cmd -> positive elevator");
}

static void test_ctrl_altitude_mode(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_ALTITUDE);

    fw_pitch_inputs_t    in  = { .pitch = 0.0f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .alt_sp = 110.0f, .airspeed_sp = 15.0f };

    /* First iteration primes TECS */
    fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);

    /* Second iteration should produce climb command */
    fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);
    ASSERT_TRUE(out.elevator > 0.0f,
                "ALTITUDE mode: alt deficit -> positive elevator");
    ASSERT_TRUE(out.pitch_sp > 0.0f,
                "ALTITUDE mode: TECS generates positive pitch sp");
}

static void test_ctrl_mode_transition_resets(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_STABILIZE);

    fw_pitch_inputs_t    in  = { .pitch = 0.0f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .pitch_sp = 0.3f };

    /* Run several iterations to build up integrator */
    for (int i = 0; i < 100; i++) {
        fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.004f);
    }

    /* Switch to RATE_ONLY -> should reset integrators */
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_RATE_ONLY);

    ASSERT_NEAR(ctrl.rate_state.integrator, 0.0f, 1e-6f,
                "mode transition resets rate integrator");
    ASSERT_NEAR(ctrl.att_state.integrator, 0.0f, 1e-6f,
                "mode transition resets attitude integrator");
}

static void test_ctrl_bad_dt_rejected(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_STABILIZE);

    fw_pitch_inputs_t    in  = { .pitch = 0.0f, .roll = 0.0f,
                                 .pitch_rate = 0.0f, .airspeed = 15.0f,
                                 .altitude = 100.0f };
    fw_pitch_setpoints_t sp  = { .pitch_sp = 0.3f };

    /* dt too small */
    fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 0.0001f);
    ASSERT_NEAR(out.elevator, 0.0f, 1e-5f,
                "dt below dt_min returns zero");

    /* dt too large */
    out = fw_pitch_ctrl_update(&ctrl, &in, &sp, 1.0f);
    ASSERT_NEAR(out.elevator, 0.0f, 1e-5f,
                "dt above dt_max returns zero");
}

/* ------------------------------------------------------------------ */
/* Test: Closed-loop convergence (multi-step simulation)               */
/* ------------------------------------------------------------------ */

static void test_closed_loop_stabilize_convergence(void)
{
    fw_pitch_ctrl_t ctrl;
    fw_pitch_ctrl_init(&ctrl);
    fw_pitch_ctrl_set_mode(&ctrl, FW_PITCH_MODE_STABILIZE);

    /* Simple 1-DOF pitch dynamics: I_yy * q_dot = M_elev * elev */
    float theta = 0.0f;          /* pitch angle [rad] */
    float q     = 0.0f;          /* pitch rate  [rad/s] */
    float theta_sp = 0.1f;       /* 5.7 deg nose up */
    float dt    = 0.004f;        /* 250 Hz */
    float I_yy  = 0.1f;          /* moment of inertia */
    float M_eff = 2.0f;          /* elevator effectiveness [Nm per unit] */

    fw_pitch_inputs_t in;
    fw_pitch_setpoints_t sp;
    memset(&in, 0, sizeof(in));
    memset(&sp, 0, sizeof(sp));
    sp.pitch_sp = theta_sp;
    in.airspeed = 15.0f;
    in.altitude = 100.0f;

    for (int i = 0; i < 2500; i++) {   /* 10 seconds */
        in.pitch      = theta;
        in.pitch_rate = q;

        fw_pitch_output_t out = fw_pitch_ctrl_update(&ctrl, &in, &sp, dt);

        /* Simple Euler integration of pitch dynamics */
        float moment = M_eff * out.elevator;
        q     += (moment / I_yy) * dt;
        theta += q * dt;
    }

    ASSERT_NEAR(theta, theta_sp, 0.02f,
                "closed-loop converges to pitch setpoint within 10s");
    ASSERT_NEAR(q, 0.0f, 0.1f,
                "pitch rate settles near zero");
}

/* ------------------------------------------------------------------ */
/* Main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("=== PX4 Fixed-Wing Pitch Controller Tests ===\n\n");

    /* Rate controller tests */
    printf("--- Rate Controller ---\n");
    RUN_TEST(test_rate_ctrl_zero_error);
    RUN_TEST(test_rate_ctrl_positive_error);
    RUN_TEST(test_rate_ctrl_negative_error);
    RUN_TEST(test_rate_ctrl_output_saturation);
    RUN_TEST(test_rate_ctrl_integrator_accumulates);
    RUN_TEST(test_rate_ctrl_integrator_windup);
    RUN_TEST(test_rate_ctrl_zero_dt_safe);
    RUN_TEST(test_rate_ctrl_reset_integrator);

    /* Attitude controller tests */
    printf("\n--- Attitude Controller ---\n");
    RUN_TEST(test_att_ctrl_zero_error);
    RUN_TEST(test_att_ctrl_nose_up_command);
    RUN_TEST(test_att_ctrl_nose_down_command);
    RUN_TEST(test_att_ctrl_turn_compensation);
    RUN_TEST(test_att_ctrl_pitch_limits);

    /* TECS tests */
    printf("\n--- TECS Pitch Axis ---\n");
    RUN_TEST(test_tecs_level_flight);
    RUN_TEST(test_tecs_climb_command);
    RUN_TEST(test_tecs_descend_command);
    RUN_TEST(test_tecs_pitch_limits);

    /* Top-level controller tests */
    printf("\n--- Top-Level Controller ---\n");
    RUN_TEST(test_ctrl_manual_no_output);
    RUN_TEST(test_ctrl_stabilize_corrects);
    RUN_TEST(test_ctrl_rate_only);
    RUN_TEST(test_ctrl_altitude_mode);
    RUN_TEST(test_ctrl_mode_transition_resets);
    RUN_TEST(test_ctrl_bad_dt_rejected);

    /* Closed-loop convergence */
    printf("\n--- Closed-Loop Convergence ---\n");
    RUN_TEST(test_closed_loop_stabilize_convergence);

    /* Summary */
    printf("\n=== Results: %d/%d passed, %d failed ===\n",
           tests_passed, tests_run, tests_failed);

    return (tests_failed > 0) ? 1 : 0;
}
