/**
 * @file fw_autotune_attitude_control.hpp
 *
 * Fixed-wing attitude controller auto-tuner -- system identification and
 * PID gain computation.
 *
 * ============================================================================
 * PURPOSE
 * ============================================================================
 *
 * This module performs online system identification during flight by injecting
 * controlled excitation signals into the rate controller and measuring the
 * aircraft's angular rate response. From the measured input-output data it
 * identifies a simplified dynamic model for each axis (roll, pitch, yaw) and
 * then computes optimal PID gains using well-established tuning rules.
 *
 * The workflow is fully automated:
 *
 *   1. The pilot enables the autotuner (FW_AT_EN = 1) and triggers start().
 *   2. For each axis the autotuner:
 *        a) Detects ambient noise amplitude to set a baseline.
 *        b) Injects an excitation signal (step or frequency sweep).
 *        c) Pauses to let transients settle.
 *   3. After all three axes, the identified gains are presented for
 *      confirmation or automatically applied depending on FW_AT_APPLY.
 *   4. A verification phase optionally tests the new gains under gentle
 *      manoeuvres before the tuning is finalised.
 *
 * Safety: the autotuner monitors the angular rates throughout. If any axis
 * exceeds 3x the expected excitation amplitude, tuning is immediately
 * aborted and original gains are restored.
 *
 * ============================================================================
 * SYSTEM IDENTIFICATION THEORY
 * ============================================================================
 *
 * The aircraft's angular rate response to a torque input on a single axis
 * is modelled as a first-order-plus-delay (FOPD) transfer function:
 *
 *                          K
 *       G(s) = -------------------------
 *               tau * s  +  1
 *
 * Where:
 *   K   = steady-state gain  [rad/s per unit torque]
 *   tau = time constant       [s]
 *
 * This is the simplest model that captures the dominant dynamics of a
 * conventional fixed-wing aircraft on any single axis. Higher-order modes
 * (structural flex, actuator dynamics) are treated as unmodelled and
 * robustness is ensured by conservative gain margins.
 *
 * For STEP excitation, the identification proceeds as follows:
 *
 *   1. A step input of known amplitude A is applied for a known duration.
 *   2. The steady-state angular rate response y_ss is measured once the
 *      rate has settled (within a tolerance band).
 *   3. The rise time t_r (10% to 90% of y_ss) is measured.
 *   4. The model parameters are computed:
 *        K   = y_ss / A
 *        tau = t_r / 2.2   (exact for a first-order system: t_r = 2.2*tau)
 *
 * For SINE SWEEP excitation:
 *   - A chirp signal sweeps from f0 to f1 Hz.
 *   - The magnitude and phase of the response are sampled at multiple
 *     frequencies.
 *   - K and tau are fit from the Bode magnitude roll-off (-3dB point
 *     gives 1/(2*pi*tau)).
 *
 * ============================================================================
 * PID TUNING METHODOLOGY
 * ============================================================================
 *
 * From the identified first-order model parameters (K, tau), PID gains
 * are computed using ITAE-optimal (Integral of Time-weighted Absolute
 * Error) relations for a first-order plant. These relations minimise the
 * time-weighted absolute error for a step disturbance, which provides a
 * good balance between speed of response and overshoot:
 *
 *   Kp  = 1.0 / (K * tau)           -- proportional gain
 *   Ki  = Kp  / (2.0 * tau)         -- integral gain
 *   Kd  = Kp  * tau * 0.25          -- derivative gain
 *   Kff = 1.0 / K                   -- feed-forward gain
 *
 * Physical interpretation:
 *   - Kp sets the closed-loop bandwidth to approximately 1/tau, matching
 *     the plant's natural speed of response.
 *   - Ki is set to remove steady-state error with a time constant of
 *     2*tau, slow enough to avoid integral windup.
 *   - Kd adds phase lead at the crossover frequency, improving damping.
 *     The 0.25 factor keeps the derivative action moderate.
 *   - Kff inverts the plant gain so that a 1 rad/s rate setpoint
 *     produces approximately 1 rad/s of actual rate, providing
 *     immediate open-loop tracking without waiting for error to build.
 *
 * These gains are starting points. The verification phase tests them
 * with gentle setpoint changes and can apply a global scaling factor
 * if the response is too aggressive or too sluggish.
 *
 * ============================================================================
 * REFERENCES
 * ============================================================================
 *
 *   - PX4 source: src/modules/fw_autotune_attitude_control/
 *   - Astrom & Hagglund, "Advanced PID Control", Chapter 7
 *   - Ljung, "System Identification: Theory for the User", Chapter 1
 *   - Ziegler & Nichols, "Optimum Settings for Automatic Controllers" (1942)
 *   - Rovira et al., "ITAE Optimal Tuning for First-Order Systems" (1969)
 */

#pragma once

/* ------------------------------------------------------------------ */
/* Includes                                                            */
/* ------------------------------------------------------------------ */

#include "../common/math_utils.hpp"
#include "../common/uorb_msgs.hpp"

#include <cstdint>
#include <cmath>

namespace fw_autotune {

/* ================================================================== */
/* Enumerations                                                        */
/* ================================================================== */

/**
 * Autotuner state machine states.
 *
 * The autotuner progresses linearly through these states for each
 * axis (roll -> pitch -> yaw). The sequence for a single axis is:
 *
 *   AMP_DETECT -> EXCITATION -> PAUSE -> (next axis or APPLY)
 *
 * IDLE:           Module enabled but not actively tuning.
 * INIT:           Transition state -- snapshot original gains, reset buffers.
 * ROLL_AMP_DETECT: Measure ambient roll rate to establish noise baseline.
 * ROLL:           Inject excitation signal on the roll axis.
 * ROLL_PAUSE:     Wait for roll transients to decay before moving on.
 * PITCH_AMP_DETECT / PITCH / PITCH_PAUSE: Same for pitch.
 * YAW_AMP_DETECT / YAW / YAW_PAUSE: Same for yaw.
 * APPLY:          Present identified gains; wait for confirmation or auto-apply.
 * TEST:           Apply new gains and perform gentle test manoeuvres.
 * VERIFICATION:   Evaluate test results and decide accept / reject.
 * COMPLETE:       Tuning finished successfully.
 */
enum class AutotuneState : uint8_t {
    IDLE            = 0,
    INIT            = 1,
    ROLL_AMP_DETECT = 2,
    ROLL            = 3,
    ROLL_PAUSE      = 4,
    PITCH_AMP_DETECT = 5,
    PITCH           = 6,
    PITCH_PAUSE     = 7,
    YAW_AMP_DETECT  = 8,
    YAW             = 9,
    YAW_PAUSE       = 10,
    APPLY           = 11,
    TEST            = 12,
    VERIFICATION    = 13,
    COMPLETE        = 14
};

/**
 * Excitation signal types.
 *
 * STEP:              Square-wave step input. Simple, fast, robust.
 *                    Positive for the first half of the duration,
 *                    negative for the second half.
 *
 * LINEAR_SINE_SWEEP: Chirp signal with frequency increasing linearly
 *                    from f0 to f1. Excites a broad band of frequencies
 *                    for richer identification data.
 *
 * LOG_SINE_SWEEP:    Chirp signal with frequency increasing on a
 *                    logarithmic scale. Spends more time at low
 *                    frequencies where the plant gain is highest,
 *                    giving better signal-to-noise at the frequencies
 *                    that matter most for PID tuning.
 */
enum class ExcitationType : uint8_t {
    STEP              = 0,
    LINEAR_SINE_SWEEP = 1,
    LOG_SINE_SWEEP    = 2
};

/**
 * Axis index constants for array access.
 */
enum AxisIndex : int {
    AXIS_ROLL  = 0,
    AXIS_PITCH = 1,
    AXIS_YAW   = 2
};

/* ================================================================== */
/* Parameter structure                                                  */
/* ================================================================== */

/**
 * All tunable parameters for the fixed-wing attitude autotuner.
 *
 * In PX4, each of these is a PARAM_DEFINE_FLOAT / INT32 that can be
 * changed at runtime via QGC or the parameter file. We store them as
 * plain struct members with sensible defaults so the module works
 * out-of-the-box in simulation.
 *
 * Naming follows PX4 conventions with prefix FW_AT_*.
 */
struct FwAutotuneParams {

    /**
     * FW_AT_EN -- Enable the autotuner module.
     *
     * 0 = disabled (default), 1 = enabled. When disabled, the module
     * does not subscribe to any topics and consumes no CPU.
     */
    int32_t enable{0};

    /**
     * FW_AT_SYSID_TYPE -- System identification excitation signal type.
     *
     * 0 = step response (default, simplest and most robust)
     * 1 = linear frequency sine sweep (better for detailed identification)
     * 2 = logarithmic frequency sine sweep (best low-frequency resolution)
     */
    int32_t sysid_type{0};

    /**
     * FW_AT_APPLY -- Gain application mode.
     *
     * 0 = manual confirmation required (default). The identified gains
     *     are reported via telemetry and the pilot must explicitly
     *     accept them.
     * 1 = automatic application. Gains are applied immediately after
     *     identification and tested. Use with caution.
     */
    int32_t apply_mode{0};

    /**
     * FW_AT_STEP_AMP -- Step excitation amplitude [rad/s].
     *
     * The magnitude of the angular rate step injected during system
     * identification. Larger values give better signal-to-noise but
     * cause more visible aircraft motion. Must be small enough to
     * keep the aircraft within safe attitude limits.
     *
     * Typical range: 0.02 to 0.15 rad/s (1 to 8 deg/s).
     */
    float step_amplitude{0.05f};

    /**
     * FW_AT_STEP_DUR -- Step excitation duration [s].
     *
     * Total duration of the step signal (positive half + negative half).
     * Must be long enough for the rate response to reach steady state,
     * typically at least 3x the expected time constant of the axis.
     *
     * Typical range: 0.3 to 2.0 s.
     */
    float step_duration{0.5f};

    /**
     * FW_AT_SWEEP_F0 -- Sine sweep start frequency [Hz].
     *
     * The lowest frequency in the chirp signal. Should be below the
     * expected closed-loop bandwidth (typically 0.5 to 1.0 Hz for
     * a fixed-wing aircraft).
     */
    float sweep_f0{0.5f};

    /**
     * FW_AT_SWEEP_F1 -- Sine sweep end frequency [Hz].
     *
     * The highest frequency in the chirp signal. Should be above the
     * expected open-loop crossover frequency (typically 3 to 10 Hz).
     * Higher values excite structural modes and may cause issues.
     */
    float sweep_f1{5.0f};

    /**
     * FW_AT_SWEEP_DUR -- Sine sweep duration [s].
     *
     * Total duration of the frequency sweep. Longer sweeps give more
     * data points and better identification accuracy at the cost of
     * longer flight time in the autotune mode.
     *
     * Typical range: 5.0 to 30.0 s.
     */
    float sweep_duration{10.0f};

    /**
     * FW_AT_PAUSE_DUR -- Pause duration between axis tests [s].
     *
     * Time to wait after completing one axis before starting the next.
     * This allows transients from the previous axis to decay and lets
     * the attitude controller re-stabilise the aircraft.
     *
     * Typical range: 1.0 to 5.0 s.
     */
    float pause_duration{2.0f};

    /**
     * FW_AT_AMP_THR -- Amplitude detection threshold [rad/s].
     *
     * During the AMP_DETECT phase, the autotuner measures the peak
     * angular rate on the current axis without any excitation. If
     * the ambient rate exceeds this threshold, the aircraft is too
     * disturbed (turbulence, pilot input) and the autotuner waits
     * until conditions calm down.
     *
     * Typical range: 0.05 to 0.2 rad/s.
     */
    float amp_detect_threshold{0.1f};
};

/* ================================================================== */
/* Identified gains structure                                          */
/* ================================================================== */

/**
 * Complete set of identified PID gains for all three axes.
 *
 * These are the gains that will be applied to the rate controller
 * after successful system identification and (optionally) pilot
 * confirmation.
 */
struct IdentifiedGains {

    /** Per-axis PID gains: index 0=roll, 1=pitch, 2=yaw */
    float kp[3]{0.0f, 0.0f, 0.0f};    ///< Proportional gain
    float ki[3]{0.0f, 0.0f, 0.0f};    ///< Integral gain
    float kd[3]{0.0f, 0.0f, 0.0f};    ///< Derivative gain
    float kff[3]{0.0f, 0.0f, 0.0f};   ///< Feed-forward gain

    /** Identified first-order model parameters */
    float K[3]{0.0f, 0.0f, 0.0f};     ///< Steady-state gain
    float tau[3]{0.0f, 0.0f, 0.0f};   ///< Time constant [s]

    /** True if identification completed successfully for this axis */
    bool valid[3]{false, false, false};
};

/* ================================================================== */
/* Main class                                                          */
/* ================================================================== */

/**
 * FwAutotuneAttitudeControl -- automated PID tuning for fixed-wing rate control.
 *
 * Lifecycle (mirrors a PX4 WorkItem):
 *   1. Constructor:  set defaults, zero state.
 *   2. init():       validate parameters, prepare internal buffers.
 *   3. start():      begin the autotuning sequence (called by pilot action).
 *   4. Run():        called each cycle (~250 Hz) to advance the state machine.
 *   5. stop():       abort tuning and restore original gains at any time.
 *
 * Inputs (via uORB subscriptions simulated as struct members):
 *   - vehicle_angular_velocity_s   (measured body rates from gyro)
 *   - vehicle_rates_setpoint_s     (current rate setpoints from att controller)
 *   - vehicle_status_s             (flight mode, safety checks)
 *
 * Outputs (via uORB publication):
 *   - autotune_attitude_control_status_s  (state, excitation signal to inject)
 */
class FwAutotuneAttitudeControl {
public:
    FwAutotuneAttitudeControl();
    ~FwAutotuneAttitudeControl() = default;

    /* ---- Lifecycle ---- */

    /**
     * Initialise the autotuner.
     *
     * Validates parameters and prepares internal data structures.
     * Must be called before the first Run().
     *
     * @return true on success, false if parameters are invalid.
     */
    bool init();

    /**
     * Execute one autotuner cycle.
     *
     * This is the main entry point, called at the rate controller
     * frequency (~250 Hz). It advances the state machine, generates
     * excitation signals, collects response data, and performs system
     * identification when enough data is available.
     *
     * @param timestamp_now  Current monotonic time [us].
     */
    void Run(uint64_t timestamp_now);

    /**
     * Begin the autotuning sequence.
     *
     * Transitions from IDLE to INIT, which snapshots the current PID
     * gains (for possible revert) and resets all identification buffers.
     * Does nothing if the autotuner is already running.
     */
    void start();

    /**
     * Abort autotuning and restore original gains.
     *
     * Can be called at any time. The autotuner immediately transitions
     * to IDLE and the backed-up original gains remain available for
     * the caller to restore.
     */
    void stop();

    /**
     * Check whether the autotuner is actively running.
     *
     * @return true if the state is anything other than IDLE or COMPLETE.
     */
    bool isRunning() const;

    /**
     * Get the current state machine state.
     *
     * @return Current AutotuneState value.
     */
    AutotuneState getState() const { return _state; }

    /**
     * Retrieve the identified gains for all axes.
     *
     * The returned struct contains valid flags indicating which axes
     * have been successfully identified. Gains for axes that have not
     * yet been identified (or where identification failed) are zero.
     *
     * @return IdentifiedGains struct with all computed gains and model parameters.
     */
    IdentifiedGains getIdentifiedGains() const { return _identified_gains; }

    /* ---- Input setters (simulate uORB subscriptions) ---- */

    void setAngularVelocity(const uorb::vehicle_angular_velocity_s& msg) { _angular_velocity = msg; }
    void setRatesSetpoint(const uorb::vehicle_rates_setpoint_s& msg)     { _rates_setpoint = msg; }
    void setVehicleStatus(const uorb::vehicle_status_s& msg)             { _vehicle_status = msg; }

    /* ---- Output getters (simulate uORB publications) ---- */

    const uorb::autotune_attitude_control_status_s& getStatus() const { return _status_pub; }

    /**
     * Get the backup of original gains taken at the start of tuning.
     *
     * Used by the caller to restore gains if the pilot rejects the
     * new values or if tuning is aborted.
     *
     * @return IdentifiedGains struct containing the pre-tuning gains.
     */
    IdentifiedGains getOriginalGains() const { return _original_gains; }

    /* ---- Parameters (public so tests can tweak) ---- */
    FwAutotuneParams params;

private:

    /* ---- Private methods ---- */

    /**
     * Generate an excitation signal for the given axis.
     *
     * Produces the angular rate perturbation to inject into the rate
     * controller setpoint for the specified axis, based on the configured
     * excitation type (step, linear sweep, or logarithmic sweep).
     *
     * For STEP:
     *   - First half of step_duration: +step_amplitude
     *   - Second half of step_duration: -step_amplitude
     *   This produces a balanced signal with zero mean, minimising
     *   net attitude drift during identification.
     *
     * For LINEAR_SINE_SWEEP:
     *   - Instantaneous frequency f(t) = f0 + (f1 - f0) * t / T
     *   - Phase integral: phi(t) = 2*pi * [f0*t + (f1-f0)*t^2 / (2*T)]
     *   - Signal: A * sin(phi(t))
     *
     * For LOG_SINE_SWEEP:
     *   - Instantaneous frequency f(t) = f0 * (f1/f0)^(t/T)
     *   - Phase integral: phi(t) = 2*pi * f0*T / ln(f1/f0) * [(f1/f0)^(t/T) - 1]
     *   - Signal: A * sin(phi(t))
     *
     * @param elapsed_time  Time since the excitation started [s].
     * @param axis          Axis index (0=roll, 1=pitch, 2=yaw).
     * @return              Excitation signal amplitude [rad/s].
     */
    float generateExcitationSignal(float elapsed_time, int axis);

    /**
     * Detect the ambient angular rate amplitude on a given axis.
     *
     * Measures the peak-to-peak angular rate over the detection window
     * to establish a noise baseline. If the ambient rate exceeds the
     * configured threshold, the conditions are too turbulent for
     * reliable identification.
     *
     * @param rates  Current body angular rates [rad/s] (3-element array).
     * @param axis   Axis index (0=roll, 1=pitch, 2=yaw).
     * @return       true if conditions are calm enough to proceed.
     */
    bool detectAmplitudeResponse(const float* rates, int axis);

    /**
     * Perform system identification for the specified axis.
     *
     * Analyses the collected excitation-response data to identify the
     * first-order model parameters (K, tau).
     *
     * For step response data:
     *   1. Compute the steady-state response y_ss as the mean of the
     *      last 20% of the response data.
     *   2. Find the 10% and 90% crossing times to compute rise time t_r.
     *   3. Compute: K = y_ss / excitation_amplitude
     *               tau = t_r / 2.2
     *
     * The method validates the results (K and tau must be positive and
     * within physically reasonable bounds) before storing them.
     *
     * @param axis  Axis index (0=roll, 1=pitch, 2=yaw).
     * @return      true if identification succeeded with valid parameters.
     */
    bool identifySystem(int axis);

    /**
     * Compute PID gains from identified model parameters.
     *
     * Uses ITAE-optimal tuning relations for a first-order plant:
     *
     *   Kp  = 1.0 / (K * tau)
     *   Ki  = Kp  / (2.0 * tau)
     *   Kd  = Kp  * tau * 0.25
     *   Kff = 1.0 / K
     *
     * These relations are derived from minimising the Integral of
     * Time-weighted Absolute Error (ITAE) criterion for a unit step
     * disturbance applied to a first-order plant with PID control.
     *
     * Safety bounds are applied to all computed gains to prevent
     * unreasonable values from entering the rate controller.
     *
     * @param axis  Axis index (0=roll, 1=pitch, 2=yaw).
     * @return      true if computed gains are within acceptable bounds.
     */
    bool computePidGains(int axis);

    /**
     * Advance the state machine to the next state.
     *
     * Handles the linear progression through the axis sequence:
     *   INIT -> ROLL_AMP_DETECT -> ROLL -> ROLL_PAUSE
     *        -> PITCH_AMP_DETECT -> PITCH -> PITCH_PAUSE
     *        -> YAW_AMP_DETECT -> YAW -> YAW_PAUSE
     *        -> APPLY -> TEST -> VERIFICATION -> COMPLETE
     *
     * Also resets per-state timers and buffers for the new state.
     */
    void advanceState();

    /* ---- State machine ---- */

    AutotuneState _state{AutotuneState::IDLE};

    /**
     * Timestamp [us] when the current state was entered.
     * Used to compute elapsed time within each state.
     */
    uint64_t _state_start_timestamp{0};

    /**
     * Timestamp [us] of the previous Run() call.
     * Used to compute dt for integration and signal generation.
     */
    uint64_t _last_run_timestamp{0};

    /** True after init() has been successfully called. */
    bool _initialized{false};

    /* ---- uORB input mirrors ---- */

    uorb::vehicle_angular_velocity_s  _angular_velocity{};
    uorb::vehicle_rates_setpoint_s    _rates_setpoint{};
    uorb::vehicle_status_s            _vehicle_status{};

    /* ---- uORB output storage ---- */

    uorb::autotune_attitude_control_status_s _status_pub{};

    /* ---- Identification data ---- */

    /**
     * Response data buffer for system identification.
     *
     * Stores angular rate measurements during the excitation phase.
     * The buffer is sized for the maximum expected excitation duration
     * at the control loop frequency (~250 Hz * 10s sweep = 2500 samples).
     * We use a fixed-size array with a write index to avoid dynamic
     * allocation, which is prohibited in PX4 flight code.
     */
    static constexpr int RESPONSE_BUFFER_SIZE = 4096;
    float _response_buffer[RESPONSE_BUFFER_SIZE]{};
    float _excitation_buffer[RESPONSE_BUFFER_SIZE]{};
    int   _buffer_index{0};

    /**
     * Ambient rate amplitude measured during AMP_DETECT phase.
     * Used as a noise floor reference for signal quality assessment.
     */
    float _ambient_amplitude[3]{0.0f, 0.0f, 0.0f};

    /**
     * Peak rate observed during AMP_DETECT (for each axis).
     * Reset when entering the AMP_DETECT state.
     */
    float _detect_rate_max{0.0f};
    float _detect_rate_min{0.0f};

    /* ---- Identified gains and backup ---- */

    /** Gains computed from system identification. */
    IdentifiedGains _identified_gains{};

    /**
     * Backup of the original PID gains taken at the start of tuning.
     *
     * These are restored if the pilot rejects the new gains or if
     * tuning is aborted for any reason (safety limit, pilot action,
     * or identification failure).
     */
    IdentifiedGains _original_gains{};

    /* ---- Safety monitoring ---- */

    /**
     * Maximum rate safety multiplier.
     *
     * If any angular rate exceeds this factor times the excitation
     * amplitude during identification, the autotuner aborts immediately.
     * This catches divergent oscillations that could result from
     * exciting an unstable or marginally-stable mode.
     */
    static constexpr float SAFETY_RATE_MULTIPLIER = 3.0f;

    /**
     * Minimum and maximum physically reasonable model parameters.
     * Used to validate identification results.
     */
    static constexpr float MIN_GAIN_K      = 0.01f;   ///< Minimum steady-state gain
    static constexpr float MAX_GAIN_K      = 100.0f;   ///< Maximum steady-state gain
    static constexpr float MIN_TAU         = 0.01f;   ///< Minimum time constant [s]
    static constexpr float MAX_TAU         = 5.0f;    ///< Maximum time constant [s]

    /**
     * PID gain safety bounds.
     * Computed gains outside these ranges are rejected.
     */
    static constexpr float MAX_KP  = 2.0f;
    static constexpr float MAX_KI  = 2.0f;
    static constexpr float MAX_KD  = 0.5f;
    static constexpr float MAX_KFF = 10.0f;

    /**
     * Amplitude detection window duration [s].
     * How long to observe ambient rates before deciding conditions are calm.
     */
    static constexpr float AMP_DETECT_DURATION = 1.0f;

    /**
     * Fraction of response buffer used for steady-state estimation.
     * The last STEADY_STATE_FRACTION of samples are averaged to
     * estimate the steady-state response.
     */
    static constexpr float STEADY_STATE_FRACTION = 0.2f;
};

} // namespace fw_autotune
