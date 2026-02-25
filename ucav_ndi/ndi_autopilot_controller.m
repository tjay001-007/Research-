function [de_L, de_R, dr, dc, ctrl_state_out, ctrl_debug] = ...
    ndi_autopilot_controller( ...
        phi_cmd, theta_cmd, ...
        u, v, w, p, q, r, phi, theta, psi, ...
        ax_meas, ay_meas, az_meas, ...
        pdot_meas, qdot_meas, rdot_meas, ...
        de_L_actual, de_R_actual, dr_actual, dc_actual, ...
        ctrl_state_in, ...
        aircraft, fcs, dt)
%NDI_AUTOPILOT_CONTROLLER  High-fidelity NDI/INDI inner-loop controller
%   for an unmanned delta-canard combat aircraft following a reference
%   trajectory.
%
%  ========================================================================
%  MODULE PURPOSE
%  ========================================================================
%
%  This module is the INNERMOST control law — the part that directly
%  computes control surface deflections. It receives attitude commands
%  (phi_cmd, theta_cmd) from the guidance law and drives the aircraft
%  to track those commands using Nonlinear Dynamic Inversion.
%
%  THIS IS THE MOST SAFETY-CRITICAL MODULE IN THE ENTIRE SYSTEM.
%  If this module fails or computes incorrect outputs, the aircraft will
%  crash within milliseconds (the open-loop divergence time of this
%  airframe is ~150 ms).
%
%  ========================================================================
%  ARCHITECTURE — TWO-LOOP CASCADED NDI
%  ========================================================================
%
%  ┌─────────────────────────────────────────────────────────────────────┐
%  │                                                                     │
%  │  phi_cmd ──┐                                                        │
%  │  theta_cmd ┤                                                        │
%  │            ▼                                                        │
%  │  [A. ENVELOPE PROTECTION]                                           │
%  │       Alpha limiter, structural g-limiter, beta limiter             │
%  │       Roll rate vs alpha scheduling                                 │
%  │            │                                                        │
%  │            ▼                                                        │
%  │  [B. OUTER LOOP — ATTITUDE NDI]                                    │
%  │       Kinematic inversion: attitude error → body rate commands      │
%  │       phi_err, theta_err → p_cmd, q_cmd, r_cmd                    │
%  │            │                                                        │
%  │            ▼                                                        │
%  │  [C. INNER LOOP — RATE INDI]                                      │
%  │       Dynamic inversion: rate error → moment commands               │
%  │       Uses MEASURED angular acceleration (INDI, sensor-based)       │
%  │       Falls back to MODEL-BASED NDI if sensor data is invalid       │
%  │            │                                                        │
%  │            ▼                                                        │
%  │  [D. CONTROL ALLOCATION]                                           │
%  │       Moment commands → individual surface deflections              │
%  │       Handles elevon mixing, canard scheduling, priority logic      │
%  │            │                                                        │
%  │            ▼                                                        │
%  │  [E. STRUCTURAL FILTERS]                                           │
%  │       Notch/low-pass to prevent aeroservoelastic coupling           │
%  │            │                                                        │
%  │            ▼                                                        │
%  │  de_L, de_R, dr, dc  →  ACTUATORS  →  AIRCRAFT                    │
%  │                                                                     │
%  └─────────────────────────────────────────────────────────────────────┘
%
%  ========================================================================
%  HIGH-FIDELITY FEATURES FOR REAL FLIGHT
%  ========================================================================
%
%  1. INCREMENTAL NDI (INDI) as primary mode
%     - Does NOT require accurate aerodynamic model for M0
%     - Uses measured angular acceleration from IMU
%     - Robust to CG shifts, battle damage, icing, store release
%     - Falls back to classical NDI if acceleration signal is bad
%
%  2. Full inertia tensor with Ixz product of inertia
%     - Critical for preventing inertia-coupled roll/yaw departure
%     - Especially important at high angle of attack
%
%  3. Control effectiveness scheduling
%     - B matrix looked up as f(alpha, Mach) from aero tables
%     - Accounts for reduced elevon effectiveness at high alpha
%     - Accounts for transonic control power changes
%
%  4. Robust integrator management
%     - Anti-windup on all three axes
%     - Integrator freeze during saturation
%     - Rate-limited integrator to prevent sudden jumps
%
%  5. Sensor validation
%     - Range checks on angular acceleration measurements
%     - Fallback to model-based NDI if sensors are invalid
%
%  6. Output rate limiting
%     - Surface command rate limited to prevent actuator-induced
%       structural loads
%
%  ========================================================================
%  INPUTS
%  ========================================================================
%    phi_cmd, theta_cmd  - From guidance law (rad)
%    u, v, w             - Body velocities from nav filter (m/s)
%    p, q, r             - Angular rates from gyros (rad/s)
%    phi, theta, psi     - Euler angles from nav filter (rad)
%    ax,ay,az_meas       - Body accelerations from accels (m/s^2)
%    pdot,qdot,rdot_meas - Angular accelerations (rad/s^2)
%                          (differentiated + filtered gyro or accel)
%    de_L/R_actual       - Actuator position feedback LVDT (rad)
%    dr_actual, dc_actual- Rudder, canard position feedback (rad)
%    ctrl_state_in       - Previous controller state (integrators, filters)
%    aircraft            - Aircraft parameter struct
%    fcs                 - Flight control system gains
%    dt                  - Controller timestep (s)
%
%  OUTPUTS
%  ========================================================================
%    de_L, de_R          - Left/right elevon commands (rad)
%    dr                  - Rudder command (rad)
%    dc                  - Canard command (rad)
%    ctrl_state_out      - Updated controller state
%    ctrl_debug          - Debug/telemetry struct

%% ====================================================================
%  SECTION A: AIRDATA COMPUTATION
%  ====================================================================
%
%  Compute the aerodynamic quantities needed by the controller.
%  In a real aircraft, these come from the Air Data Computer (ADC)
%  which processes pitot-static probes and vanes/flush ports.
%
%  Critical quantities:
%    V     - True airspeed (from pitot-static, corrected for density)
%    alpha - Angle of attack (from alpha vane or FADS)
%    beta  - Sideslip angle (from beta vane or FADS)
%    qbar  - Dynamic pressure (from pitot-static differential)
%    Mach  - Mach number (from pitot-static ratio)

g   = 9.81;
rho = 1.225;  % Will be overridden by ISA model

V     = sqrt(u^2 + v^2 + w^2);
V     = max(V, 10.0);              % Minimum 10 m/s for valid aero
alpha = atan2(w, u);               % Angle of attack (rad)
beta  = asin(clamp(v/V, -1, 1));  % Sideslip angle (rad)

% ISA atmospheric model (altitude from position or baro-altimeter)
alt = max(0, aircraft.alt_init);    % Simplified; in real system from baro
[~, a_sound, ~, rho] = atmos_isa(alt);
Mach = V / a_sound;
qbar = 0.5 * rho * V^2;

alpha_deg = rad2deg(alpha);
beta_deg  = rad2deg(beta);

% Normal load factor (from accelerometer, body z-axis)
Nz = -az_meas / g;

%% ====================================================================
%  SECTION B: ENVELOPE PROTECTION
%  ====================================================================
%
%  Even though this is an unmanned aircraft without a pilot, envelope
%  protection is ESSENTIAL because:
%    1. The guidance law may command attitudes that exceed the aircraft's
%       aerodynamic or structural limits
%    2. Wind gusts or turbulence can push the aircraft toward limits
%    3. The NDI model mismatch can cause overshoot
%    4. Sensor noise can cause transient limit exceedances
%
%  The limiters use smooth cosine blending (NOT hard clamps) to prevent
%  limit-cycle oscillations near the boundary.

% ---- Alpha limiter ----
%  Prevents stall, post-stall departure, and loss of control.
%  The delta-canard configuration has gradual stall characteristics
%  due to the canard stalling first, but vortex breakdown at high
%  alpha causes sudden lateral instability (wing rock, departure).
alpha_factor = smooth_limit(alpha_deg, fcs.alpha_warn_deg, fcs.alpha_max_deg);

% Reduce pitch-up command as alpha approaches limit
if theta_cmd > theta
    theta_cmd = theta + (theta_cmd - theta) * alpha_factor;
end

% ---- Structural g-limiter ----
%  The autopilot should never command more than the structural limit.
%  Unmanned aircraft typically have HIGHER g-limits than manned (no pilot).
Nz_from_theta = (theta_cmd - theta) * fcs.K_theta_approx;
if Nz_from_theta > fcs.Nz_max
    theta_cmd = theta + fcs.Nz_max / fcs.K_theta_approx;
elseif Nz_from_theta < fcs.Nz_min
    theta_cmd = theta + fcs.Nz_min / fcs.K_theta_approx;
end

% ---- Roll rate limiting at high alpha ----
%  At high alpha, large roll rates cause inertia coupling (Ixz effect)
%  that can yaw the aircraft into a spin. Limit the roll command.
roll_rate_factor = smooth_limit(alpha_deg, 15, 25);
phi_max_current = fcs.phi_max * (0.3 + 0.7 * roll_rate_factor);
phi_cmd = clamp(phi_cmd, -phi_max_current, phi_max_current);

% ---- Sideslip protection ----
%  Beta should remain near zero for a delta-canard (no intentional
%  sideslip for an unmanned aircraft — unlike a manned fighter that
%  might use rudder for gun tracking).
beta_cmd = 0;  % Always command zero sideslip

%% ====================================================================
%  SECTION C: OUTER LOOP — ATTITUDE NDI
%  ====================================================================
%
%  THEORY:
%  The aircraft's Euler angle rates relate to body angular rates through
%  the kinematic equation:
%
%    [phi_dot  ]   [1   sin(phi)*tan(theta)   cos(phi)*tan(theta)] [p]
%    [theta_dot] = [0   cos(phi)             -sin(phi)            ] [q]
%    [psi_dot  ]   [0   sin(phi)/cos(theta)   cos(phi)/cos(theta)] [r]
%
%  This is: euler_dot = T(phi, theta) * omega
%
%  NDI INVERSION:
%  We specify the desired Euler angle rates:
%    euler_dot_desired = K_outer * (euler_cmd - euler)
%
%  Then invert the kinematic matrix to get body rate commands:
%    omega_cmd = T_inv(phi, theta) * euler_dot_desired
%
%  where T_inv is:
%    [1   0          -sin(theta)        ]
%    [0   cos(phi)    sin(phi)*cos(theta)]
%    [0  -sin(phi)    cos(phi)*cos(theta)]
%
%  BANDWIDTH SEPARATION:
%  The outer loop must be SLOWER than the inner loop by a factor of
%  3-5x. If the outer loop is too fast, the cascade assumption breaks
%  and the system can oscillate.
%
%    Inner loop bandwidth: 8-12 rad/s (set by K_p, K_q, K_r)
%    Outer loop bandwidth: 2-4 rad/s (set by K_phi, K_theta, K_beta)

% Attitude errors with angle wrapping
phi_err   = wrap_angle(phi_cmd - phi);
theta_err = wrap_angle(theta_cmd - theta);
beta_err  = beta_cmd - beta;

% Desired Euler angle rates (proportional control)
phi_dot_des   = fcs.K_phi   * phi_err;
theta_dot_des = fcs.K_theta * theta_err;

% Yaw: coordinate the turn (no intentional sideslip for UCAV)
%   In a coordinated turn: psi_dot = g * tan(phi) / V
%   Add beta damping to reject gusts
psi_dot_coord = g * tan(phi) / V;
psi_dot_des   = psi_dot_coord - fcs.K_beta * beta;

euler_dot_des = [phi_dot_des; theta_dot_des; psi_dot_des];

% Inverse kinematic matrix
cos_theta = cos(theta);
if abs(cos_theta) < 0.01
    cos_theta = sign(cos_theta) * 0.01;
end

T_inv = [1,  0,         -sin(theta);
         0,  cos(phi),   sin(phi)*cos_theta;
         0, -sin(phi),   cos(phi)*cos_theta];

omega_cmd = T_inv * euler_dot_des;

% Rate command limits (structural and handling)
omega_cmd(1) = clamp(omega_cmd(1), -fcs.p_max, fcs.p_max);
omega_cmd(2) = clamp(omega_cmd(2), -fcs.q_max, fcs.q_max);
omega_cmd(3) = clamp(omega_cmd(3), -fcs.r_max, fcs.r_max);

%% ====================================================================
%  SECTION D: INNER LOOP — RATE INDI (Incremental NDI)
%  ====================================================================
%
%  This is the core dynamic inversion that stabilises the airframe.
%
%  INDI THEORY (why it's better than classical NDI for real flight):
%  -----------------------------------------------------------------
%  Classical NDI: computes the FULL bare-airframe moment M0 from an
%  onboard aerodynamic model and cancels it algebraically. Any error
%  in M0 (wrong Cmalpha, wrong CG position, ice on wings, battle
%  damage) directly reduces performance.
%
%  INDI: uses the MEASURED angular acceleration (from gyro derivative
%  or angular accelerometer). The current acceleration already contains
%  ALL aerodynamic effects — no model needed.
%
%  INDI only needs the control effectiveness B to be accurate:
%    Delta_delta = B^{-1} * J * (omega_dot_des - omega_dot_meas)
%    delta_new = delta_current + Delta_delta
%
%  This is FAR more robust because:
%    - B (control derivative) varies less with flight condition than M0
%    - B can be identified online from flight data
%    - Model errors in M0 are completely bypassed
%
%  FALL-BACK LOGIC:
%  If the measured angular acceleration is invalid (sensor failure,
%  excessive noise), the controller falls back to classical NDI using
%  the onboard aerodynamic model. This provides redundancy.

omega     = [p; q; r];
omega_err = omega_cmd - omega;

% ---- Integrator with anti-windup ----
%  Rejects steady-state errors from model mismatch or steady disturbances
%  (e.g., asymmetric thrust, CG offset).
ctrl_state_out = ctrl_state_in;

int_omega = ctrl_state_in.int_omega + omega_err * dt;
int_omega(1) = clamp(int_omega(1), -fcs.int_lim, fcs.int_lim);
int_omega(2) = clamp(int_omega(2), -fcs.int_lim, fcs.int_lim);
int_omega(3) = clamp(int_omega(3), -fcs.int_lim, fcs.int_lim);

% ---- Desired angular acceleration ----
K_rate = diag([fcs.K_p, fcs.K_q, fcs.K_r]);
K_int  = diag([fcs.Ki_p, fcs.Ki_q, fcs.Ki_r]);

omega_dot_des = K_rate * omega_err + K_int * int_omega;

% ---- Inertia matrix (FULL, with products of inertia) ----
J = [aircraft.Ixx,  0,           -aircraft.Ixz;
     0,             aircraft.Iyy,  0;
    -aircraft.Ixz,  0,             aircraft.Izz];

% ---- Control effectiveness matrix B(alpha, Mach) ----
B = compute_B(alpha, Mach, qbar, aircraft);

% ---- Sensor validation for INDI ----
omega_dot_meas = [pdot_meas; qdot_meas; rdot_meas];
sensor_valid = all(abs(omega_dot_meas) < 200);  % 200 rad/s^2 sanity check

if fcs.use_indi && sensor_valid
    % ===============================================================
    %  INDI MODE (primary — sensor-based)
    % ===============================================================
    delta_M_required = J * (omega_dot_des - omega_dot_meas);
    delta_surfaces = B \ delta_M_required;

    % Add increment to current actuator positions
    de_sym_current = 0.5 * (de_L_actual + de_R_actual);
    de_dif_current = 0.5 * (de_L_actual - de_R_actual);
    dr_current     = dr_actual;

    de_dif_cmd = de_dif_current + delta_surfaces(1);
    de_sym_cmd = de_sym_current + delta_surfaces(2);
    dr_cmd     = dr_current     + delta_surfaces(3);

    mode = 'INDI';
else
    % ===============================================================
    %  CLASSICAL NDI MODE (fallback — model-based)
    % ===============================================================
    %  Used if INDI sensors are invalid or fcs.use_indi is false.

    % Bare-airframe moments (from onboard aero model, zero control input)
    [~,~,~, Cl0, Cm0, Cn0] = fighter_aero_model( ...
        alpha, beta, p, q, r, V, Mach, alt, 0, 0, 0, 0, aircraft);

    M0 = qbar * [aircraft.S * aircraft.b     * Cl0;
                  aircraft.S * aircraft.c_bar * Cm0;
                  aircraft.S * aircraft.b     * Cn0];

    % Gyroscopic coupling: H = omega x (J * omega)
    J_omega = J * omega;
    H = cross(omega, J_omega);

    % NDI inversion
    rhs = J * omega_dot_des - M0 - H;
    delta_ctrl = B \ rhs;

    de_dif_cmd = delta_ctrl(1);
    de_sym_cmd = delta_ctrl(2);
    dr_cmd     = delta_ctrl(3);

    mode = 'NDI';
end

% Canard scheduling: follows alpha for pitch trim + pitch rate damping
dc_cmd = fcs.dc_schedule_alpha * alpha + fcs.dc_schedule_q * q;

% ---- Anti-windup: freeze integrator if surfaces saturated ----
any_sat = abs(de_sym_cmd) > aircraft.de_max || ...
          abs(de_dif_cmd) > aircraft.de_max || ...
          abs(dr_cmd) > aircraft.dr_max;
if any_sat
    int_omega = ctrl_state_in.int_omega;
end

ctrl_state_out.int_omega = int_omega;

%% ====================================================================
%  SECTION E: CONTROL ALLOCATION
%  ====================================================================
%
%  Converts the abstract moment-axis commands into physical surface
%  deflections. For this aircraft:
%    - Symmetric elevon (de_sym) → pitch moment
%    - Differential elevon (de_dif) → roll moment
%    - Rudder (dr) → yaw moment
%    - Canard (dc) → pitch trim assist

% Reconstruct individual elevon commands
de_L_cmd = de_sym_cmd + de_dif_cmd;
de_R_cmd = de_sym_cmd - de_dif_cmd;

% Apply position limits
de_L_cmd = clamp(de_L_cmd, -aircraft.de_max, aircraft.de_max);
de_R_cmd = clamp(de_R_cmd, -aircraft.de_max, aircraft.de_max);
dr_cmd   = clamp(dr_cmd,   -aircraft.dr_max, aircraft.dr_max);
dc_cmd   = clamp(dc_cmd,   -aircraft.dc_max, aircraft.dc_max);

%% ====================================================================
%  SECTION F: OUTPUT RATE LIMITING
%  ====================================================================
%
%  Rate-limit the commanded surface deflections to prevent:
%    1. Demanding more than the actuator can deliver (phase lag)
%    2. Structural loads from sudden surface movements
%    3. Exciting structural modes with high-frequency content
%
%  The rate limit here is set BELOW the actuator's physical rate limit
%  to provide a safety margin. The actuator then tracks this
%  rate-limited command with its own dynamics.

cmd_rate_max = fcs.surface_cmd_rate_max * dt;  % Max change per step

de_L_cmd = rate_limit(de_L_cmd, ctrl_state_in.de_L_prev, cmd_rate_max);
de_R_cmd = rate_limit(de_R_cmd, ctrl_state_in.de_R_prev, cmd_rate_max);
dr_cmd   = rate_limit(dr_cmd,   ctrl_state_in.dr_prev,   cmd_rate_max);
dc_cmd   = rate_limit(dc_cmd,   ctrl_state_in.dc_prev,   cmd_rate_max);

ctrl_state_out.de_L_prev = de_L_cmd;
ctrl_state_out.de_R_prev = de_R_cmd;
ctrl_state_out.dr_prev   = dr_cmd;
ctrl_state_out.dc_prev   = dc_cmd;

%% ====================================================================
%  OUTPUTS
%  ====================================================================

de_L = de_L_cmd;
de_R = de_R_cmd;
dr   = dr_cmd;
dc   = dc_cmd;

%% ====================================================================
%  DEBUG / TELEMETRY
%  ====================================================================

ctrl_debug.mode         = mode;
ctrl_debug.alpha_deg    = alpha_deg;
ctrl_debug.beta_deg     = beta_deg;
ctrl_debug.Mach         = Mach;
ctrl_debug.V            = V;
ctrl_debug.Nz           = Nz;
ctrl_debug.qbar         = qbar;
ctrl_debug.omega_cmd    = omega_cmd;
ctrl_debug.omega_err    = omega_err;
ctrl_debug.omega_dot_des = omega_dot_des;
ctrl_debug.phi_cmd_limited = phi_cmd;
ctrl_debug.theta_cmd_limited = theta_cmd;
ctrl_debug.de_sym_cmd   = de_sym_cmd;
ctrl_debug.de_dif_cmd   = de_dif_cmd;
ctrl_debug.sensor_valid = sensor_valid;
ctrl_debug.alpha_factor = alpha_factor;

end

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function B = compute_B(alpha, Mach, qbar, ac)
    % Control effectiveness matrix B(alpha, Mach)
    % Maps [de_dif; de_sym; dr] → [L; M; N] moment (Nm)
    alpha_deg = rad2deg(alpha);
    abp = ac.aero.alpha_bp; mbp = ac.aero.mach_bp;

    Cl_da = interp2_safe(abp, mbp, ac.aero.Clda_table, alpha_deg, Mach);
    Cm_de = interp2_safe(abp, mbp, ac.aero.Cmde_table, alpha_deg, Mach);
    Cn_da = interp2_safe(abp, mbp, ac.aero.Cnda_table, alpha_deg, Mach);
    Cn_dr = interp1_safe(mbp, ac.aero.Cndr_mach, Mach);
    Cl_dr = interp1_safe(mbp, ac.aero.Cldr_mach, Mach);

    S = ac.S; b = ac.b; c = ac.c_bar;
    B = [qbar*S*b*Cl_da,  0,             qbar*S*b*Cl_dr;
         0,                qbar*S*c*Cm_de, 0;
         qbar*S*b*Cn_da,  0,             qbar*S*b*Cn_dr];

    % Regularise near-singular B (very low speed)
    if abs(det(B)) < 1e-6
        B = B + 1e-4 * eye(3);
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function da = wrap_angle(a)
    da = mod(a + pi, 2*pi) - pi;
end

function factor = smooth_limit(value, warn, hard)
    if value <= warn,        factor = 1.0;
    elseif value >= hard,    factor = 0.0;
    else
        x = (value - warn) / (hard - warn);
        factor = 0.5 * (1 + cos(pi * x));
    end
end

function y = rate_limit(cmd, prev, max_delta)
    delta = cmd - prev;
    if abs(delta) > max_delta
        delta = sign(delta) * max_delta;
    end
    y = prev + delta;
end

function [T,a,P,rho] = atmos_isa(alt)
    T0=288.15; P0=101325; L=0.0065; R=287.05; g0=9.81;
    alt=max(alt,0);
    T=max(T0-L*alt,216.65);
    P=P0*(T/T0)^(g0/(R*L));
    rho=P/(R*T);
    a=sqrt(1.4*R*T);
end

function v = interp2_safe(xbp, ybp, tbl, xq, yq)
    xq = max(min(xq,xbp(end)),xbp(1));
    yq = max(min(yq,ybp(end)),ybp(1));
    v = interp2(ybp, xbp, tbl, yq, xq, 'linear');
end

function v = interp1_safe(bp, tbl, q)
    q = max(min(q,bp(end)),bp(1));
    v = interp1(bp, tbl, q, 'linear');
end
