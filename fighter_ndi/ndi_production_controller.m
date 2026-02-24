function [de_L, de_R, dr, dc, throttle_cmd, ctrl_debug] = ...
    ndi_production_controller( ...
        stick_lon, stick_lat, pedal, throttle_lever, ...
        u, v, w, p, q, r, phi, theta, psi, ...
        ax_meas, ay_meas, az_meas, ...
        pdot_meas, qdot_meas, rdot_meas, ...
        de_L_actual, de_R_actual, dr_actual, dc_actual, ...
        state_prev, ...
        aircraft, fcs, dt)
%NDI_PRODUCTION_CONTROLLER  Production-grade NDI flight control law for a
%   relaxed-stability delta-canard fighter aircraft.
%
%   This implements the complete flight control system as it would exist
%   in a real digital FBW fighter (Eurofighter, Tejas, Gripen class):
%
%   ┌────────────────────────────────────────────────────────────────────┐
%   │  PILOT INPUTS                                                      │
%   │  stick_lon, stick_lat, pedal, throttle                            │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [1. COMMAND SHAPING / REFERENCE MODEL]                           │
%   │       Converts stick to desired: Nz (g), roll rate, sideslip      │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [2. CAREFREE HANDLING LIMITERS]                                  │
%   │       Alpha limiter, g-limiter, beta limiter, rate limiters       │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [3. OUTER LOOP — ATTITUDE / FLIGHT PATH NDI]                    │
%   │       Generates body rate commands from attitude errors            │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [4. INNER LOOP — RATE NDI or INDI]                              │
%   │       Generates moment commands from rate errors                   │
%   │       Option: Classical NDI or Incremental NDI (INDI)             │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [5. CONTROL ALLOCATION]                                          │
%   │       Distributes moment commands to redundant surfaces            │
%   │       (elevons, canard, rudder) — handles limits and priority     │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  [6. STRUCTURAL FILTERS]                                          │
%   │       Notch filters at wing bending and fuselage bending freqs    │
%   │       Low-pass anti-aliasing                                       │
%   │       │                                                            │
%   │       ▼                                                            │
%   │  SURFACE COMMANDS → Actuators → Aircraft                          │
%   └────────────────────────────────────────────────────────────────────┘
%
%   Inputs:
%     stick_lon     - Longitudinal stick (-1 = full fwd, +1 = full aft)
%     stick_lat     - Lateral stick (-1 = full left, +1 = full right)
%     pedal         - Rudder pedal (-1 = full left, +1 = full right)
%     throttle_lever- Throttle lever (0 = idle, 1 = max dry, 1.5 = max AB)
%     u, v, w       - Body velocities (m/s)
%     p, q, r       - Body angular rates (rad/s)
%     phi, theta, psi - Euler angles (rad)
%     ax,ay,az_meas - Measured body accelerations from IMU (m/s^2)
%     pdot,qdot,rdot_meas - Measured angular accels (rad/s^2)
%                     (used by INDI mode; can be estimated from gyro diff)
%     de_L/R_actual - Actual actuator positions from LVDT feedback (rad)
%     dr_actual     - Actual rudder position (rad)
%     dc_actual     - Actual canard position (rad)
%     state_prev    - Controller state from previous step (integrators etc.)
%     aircraft      - Aircraft parameter struct
%     fcs           - Flight control system gains struct
%     dt            - Controller time step (s)
%
%   Outputs:
%     de_L, de_R    - Left/right elevon commands (rad)
%     dr            - Rudder command (rad)
%     dc            - Canard command (rad)
%     throttle_cmd  - Engine throttle command (0-1.5)
%     ctrl_debug    - Debug struct with all internal signals

%% ====================================================================
%  AIRDATA COMPUTATION
%  ====================================================================

g = 9.81;

V     = sqrt(u^2 + v^2 + w^2);
V     = max(V, 5.0);             % Minimum airspeed for valid aero
alpha = atan2(w, u);             % Angle of attack (rad)
beta  = asin(clamp(v/V, -1, 1));% Sideslip angle (rad)

% Atmospheric model (ISA)
[~, a_sound, ~, rho] = atmos_isa(aircraft.alt_init);  % Simplified
Mach  = V / a_sound;
qbar  = 0.5 * rho * V^2;        % Dynamic pressure (Pa)

% Normal load factor (body z-axis, in g's)
% Positive Nz = pull-up (conventional fighter definition)
Nz = -az_meas / g;              % az_meas is negative in a pull-up

%% ====================================================================
%  STAGE 1: COMMAND SHAPING — REFERENCE MODEL
%  ====================================================================
%  Real fighters use:
%    Pitch: stick → commanded Nz (load factor in g's)
%           Nz_cmd is proportional to stick deflection
%           At 1g wings-level, stick centered → Nz_cmd = 1g
%    Roll:  stick → commanded roll rate
%    Yaw:   pedals → commanded sideslip rate (or coordinated turn)
%
%  The reference model is a first-order or second-order filter that
%  shapes the desired response to have pleasant handling qualities.

% --- Pitch axis: stick to Nz command ---
%  At center stick, Nz_cmd = cos(phi)*cos(theta) (= 1g wings-level)
%  Stick deflection adds/subtracts g
Nz_trim     = cos(phi) * cos(theta);    % Gravity component in pull-up
Nz_stick    = stick_lon * fcs.Nz_per_stick;  % Typically 7-9 g per full stick
Nz_cmd_raw  = Nz_trim + Nz_stick;

% --- Roll axis: stick to roll rate command ---
p_cmd_raw = stick_lat * fcs.p_max_cmd;   % Max ~200-300 deg/s at subsonic

% --- Yaw axis: pedals to beta command ---
beta_cmd_raw = pedal * fcs.beta_max_cmd;  % Typically ±5 deg max sideslip

% Reference model filter (first-order lag for command smoothing)
% Prevents step-like commands that stress the structure
tau_ref = fcs.tau_ref;    % ~0.1-0.2 s
Nz_cmd   = ref_model_filter(Nz_cmd_raw,   state_prev.Nz_cmd_filt,   tau_ref, dt);
p_cmd    = ref_model_filter(p_cmd_raw,     state_prev.p_cmd_filt,    tau_ref, dt);
beta_cmd = ref_model_filter(beta_cmd_raw,  state_prev.beta_cmd_filt, tau_ref, dt);

%% ====================================================================
%  STAGE 2: CAREFREE HANDLING — ENVELOPE LIMITERS
%  ====================================================================
%  These protect the aircraft from exceeding structural or aerodynamic
%  limits. The pilot can pull full stick and slam the pedals — the FCS
%  will never let the aircraft depart controlled flight.
%
%  Implementation: smooth limiting functions that gradually reduce the
%  command as limits are approached. NOT hard clamps — those cause PIO.

alpha_deg = rad2deg(alpha);

% --- Alpha limiter (prevents stall / departure) ---
%  Alpha protection is THE most critical limiter for a delta-canard.
%  Soft limit starts at alpha_warn, hard limit at alpha_max.
alpha_factor = soft_limit_factor(alpha_deg, ...
    fcs.alpha_warn_deg, fcs.alpha_max_deg);

% Reduce Nz command as alpha approaches limit
if Nz_cmd > Nz_trim
    Nz_cmd = Nz_trim + (Nz_cmd - Nz_trim) * alpha_factor;
end

% --- Normal load factor limiter (structural protection) ---
Nz_cmd = clamp(Nz_cmd, fcs.Nz_min, fcs.Nz_max);

% --- Roll rate limiter at high alpha ---
%  Roll rate is reduced at high alpha to prevent inertia coupling
%  (wing rock, spin entry). This is critical for delta wings.
p_limit_factor = soft_limit_factor(alpha_deg, 15, 25);
p_max_current  = fcs.p_max_cmd * (0.3 + 0.7 * p_limit_factor);
p_cmd = clamp(p_cmd, -p_max_current, p_max_current);

% --- Sideslip limiter ---
beta_deg = rad2deg(beta);
beta_factor = soft_limit_factor(abs(beta_deg), ...
    fcs.beta_warn_deg, fcs.beta_max_deg);
beta_cmd = beta_cmd * beta_factor;

%% ====================================================================
%  STAGE 3: OUTER LOOP — FLIGHT PATH / ATTITUDE NDI
%  ====================================================================
%  Converts limited commands to body-axis angular rate commands.
%
%  Pitch: Nz_cmd → q_cmd (pitch rate that produces desired load factor)
%  Roll:  p_cmd is already a rate command (direct feedthrough)
%  Yaw:   beta_cmd → r_cmd (yaw rate for coordinated flight)

% --- Pitch: Nz to pitch rate command ---
%  From the force equation in the Z-body axis:
%    Nz = (qbar*S*CL)/(m*g) ≈ V*q/g + (alpha terms)
%  More precisely, the steady-state relationship is:
%    q_ss = (g/V) * (Nz_cmd - cos(phi)*cos(theta))
%  We add proportional feedback on Nz error for faster response:

Nz_err = Nz_cmd - Nz;
q_cmd  = (g / V) * (Nz_cmd - cos(phi)*cos(theta)) + ...
         fcs.K_Nz * Nz_err;

% --- Roll: direct rate command with attitude hold ---
%  When stick is centered, hold current phi (if phi < some bank angle).
%  When stick is deflected, track roll rate command.
if abs(stick_lat) < 0.05 && abs(rad2deg(phi)) < 5
    % Wings-level hold when stick centered and bank < 5 deg
    phi_err = -phi;   % Drive phi to zero
    p_cmd = fcs.K_phi * phi_err;
end

% --- Yaw: beta command to yaw rate ---
%  Coordinated flight: r = (g/V)*sin(phi)*cos(theta)/cos(beta) + ...
%  Simplified: command r to hold desired beta
beta_err = beta_cmd - beta;
r_cmd    = fcs.K_beta * beta_err + ...
           (g/V) * sin(phi) * cos(theta);  % Coordinated turn feedforward

% Rate limiting on commands
q_cmd = clamp(q_cmd, -fcs.q_max, fcs.q_max);
p_cmd = clamp(p_cmd, -fcs.p_max_struct, fcs.p_max_struct);
r_cmd = clamp(r_cmd, -fcs.r_max, fcs.r_max);

omega_cmd = [p_cmd; q_cmd; r_cmd];
omega     = [p; q; r];

%% ====================================================================
%  STAGE 4: INNER LOOP — RATE NDI / INDI
%  ====================================================================
%  This is the core dynamic inversion that cancels the nonlinear
%  aerodynamics and replaces them with commanded dynamics.
%
%  Two modes available:
%    fcs.use_indi = false → Classical NDI (model-based cancellation)
%    fcs.use_indi = true  → Incremental NDI (sensor-based, more robust)

omega_err = omega_cmd - omega;

% --- Integrator on rate error (for both NDI and INDI) ---
int_omega = state_prev.int_omega + omega_err * dt;
int_omega(1) = clamp(int_omega(1), -fcs.int_lim, fcs.int_lim);
int_omega(2) = clamp(int_omega(2), -fcs.int_lim, fcs.int_lim);
int_omega(3) = clamp(int_omega(3), -fcs.int_lim, fcs.int_lim);

% --- Desired angular acceleration ---
K_rate = diag([fcs.K_p, fcs.K_q, fcs.K_r]);
K_int  = diag([fcs.Ki_p, fcs.Ki_q, fcs.Ki_r]);

omega_dot_des = K_rate * omega_err + K_int * int_omega;

if fcs.use_indi
    % =================================================================
    %  INCREMENTAL NDI (INDI)
    % =================================================================
    %  Instead of cancelling the full model-based M0, we use the
    %  MEASURED angular acceleration and only command the INCREMENT.
    %
    %  The key insight:
    %    omega_dot = J^{-1} * [M(x, delta) + gyro]
    %
    %  At the current instant, omega_dot_measured already contains the
    %  effect of all aerodynamic moments. We only need to compute the
    %  CHANGE in control surface deflection to get the desired CHANGE
    %  in angular acceleration:
    %
    %    Delta_omega_dot = omega_dot_des - omega_dot_meas
    %    Delta_delta = B^{-1} * J * Delta_omega_dot
    %    delta_new = delta_current + Delta_delta
    %
    %  This eliminates dependence on M0 (bare-airframe model).
    %  Only B (control effectiveness) needs to be accurate.

    omega_dot_meas = [pdot_meas; qdot_meas; rdot_meas];

    % Inertia matrix (with products of inertia!)
    J = [aircraft.Ixx,  0,           -aircraft.Ixz;
         0,             aircraft.Iyy,  0;
        -aircraft.Ixz,  0,             aircraft.Izz];

    % Control effectiveness at current flight condition
    B = compute_control_effectiveness(alpha, Mach, qbar, aircraft);

    % Required moment increment
    delta_M_required = J * (omega_dot_des - omega_dot_meas);

    % Solve for control surface increment
    delta_surfaces = B \ delta_M_required;

    % Add increment to current actuator position (from LVDT feedback)
    de_sym_current = 0.5 * (de_L_actual + de_R_actual);
    de_dif_current = 0.5 * (de_L_actual - de_R_actual);
    dr_current     = dr_actual;

    de_dif_cmd = de_dif_current + delta_surfaces(1);  % Roll
    de_sym_cmd = de_sym_current + delta_surfaces(2);  % Pitch
    dr_cmd     = dr_current     + delta_surfaces(3);  % Yaw

    % Canard scheduled for pitch trim assist (reduces elevon authority usage)
    dc_cmd = fcs.dc_schedule_alpha * alpha + fcs.dc_schedule_q * q;

else
    % =================================================================
    %  CLASSICAL NDI (model-based cancellation)
    % =================================================================
    %  Computes the full bare-airframe moments using the onboard aero
    %  model and cancels them algebraically.

    % Inertia matrix (with products of inertia!)
    J = [aircraft.Ixx,  0,           -aircraft.Ixz;
         0,             aircraft.Iyy,  0;
        -aircraft.Ixz,  0,             aircraft.Izz];

    % Bare-airframe moments from onboard aero model (zero control input)
    [~, ~, ~, Cl0, Cm0, Cn0] = fighter_aero_model( ...
        alpha, beta, p, q, r, V, Mach, aircraft.alt_init, ...
        0, 0, 0, 0, ...   % Zero control surfaces for M0
        aircraft);

    M0 = qbar * [aircraft.S * aircraft.b * Cl0;
                  aircraft.S * aircraft.c_bar * Cm0;
                  aircraft.S * aircraft.b * Cn0];

    % Gyroscopic cross-coupling (full inertia matrix version)
    %   H = omega × (J * omega)
    J_omega = J * omega;
    H = cross(omega, J_omega);

    % Control effectiveness matrix
    B = compute_control_effectiveness(alpha, Mach, qbar, aircraft);

    % NDI inversion: B * delta_ctrl = J * omega_dot_des - M0 - H
    rhs = J * omega_dot_des - M0 - H;

    delta_ctrl = B \ rhs;

    de_dif_cmd = delta_ctrl(1);  % Differential elevon (roll)
    de_sym_cmd = delta_ctrl(2);  % Symmetric elevon (pitch)
    dr_cmd     = delta_ctrl(3);  % Rudder (yaw)

    % Canard: use as trim device + pitch augmentation
    dc_cmd = fcs.dc_schedule_alpha * alpha + fcs.dc_schedule_q * q;
end

% --- Anti-windup: freeze integrators if surfaces saturated ---
de_sym_sat = abs(de_sym_cmd) > aircraft.de_max;
de_dif_sat = abs(de_dif_cmd) > aircraft.de_max;
dr_sat     = abs(dr_cmd) > aircraft.dr_max;

if de_sym_sat || de_dif_sat || dr_sat
    int_omega = state_prev.int_omega;  % Freeze
end

%% ====================================================================
%  STAGE 5: CONTROL ALLOCATION
%  ====================================================================
%  Converts moment-axis commands (de_sym, de_dif, dr) into individual
%  surface commands (de_L, de_R, dr, dc).
%
%  For a real aircraft with many surfaces (Typhoon has 7+), this would
%  be a constrained optimization problem (linear programming or
%  quadratic programming). For our 4-surface aircraft, it's direct.

% Reconstruct individual elevon commands from sym/dif
de_L_cmd = de_sym_cmd + de_dif_cmd;
de_R_cmd = de_sym_cmd - de_dif_cmd;

% Apply position limits
de_L_cmd = clamp(de_L_cmd, -aircraft.de_max, aircraft.de_max);
de_R_cmd = clamp(de_R_cmd, -aircraft.de_max, aircraft.de_max);
dr_cmd   = clamp(dr_cmd,   -aircraft.dr_max, aircraft.dr_max);
dc_cmd   = clamp(dc_cmd,   -aircraft.dc_max, aircraft.dc_max);

%% ====================================================================
%  STAGE 6: STRUCTURAL COUPLING FILTERS
%  ====================================================================
%  Real aircraft wings and fuselage vibrate at specific frequencies.
%  Rate gyros pick up these vibrations. Without filtering, the FCS
%  amplifies them through the control loop → structural failure.
%
%  Notch filters are placed at known structural resonance frequencies.
%  These are determined during Ground Vibration Testing (GVT).
%
%  For this model, we apply second-order low-pass filters as a
%  simplified structural filter. Real aircraft have multiple notch
%  filters per axis (3-5 notches is common).
%
%  Implementation: the filters are applied to the surface commands
%  to roll off high-frequency content that could excite structure.

omega_struct = fcs.struct_filter_freq;  % Structural filter cutoff (rad/s)
zeta_struct  = fcs.struct_filter_zeta;  % Damping of structural filter

de_L_cmd = lowpass_2nd_order(de_L_cmd, state_prev.filt_de_L, ...
                              omega_struct, zeta_struct, dt);
de_R_cmd = lowpass_2nd_order(de_R_cmd, state_prev.filt_de_R, ...
                              omega_struct, zeta_struct, dt);
dr_cmd   = lowpass_2nd_order(dr_cmd, state_prev.filt_dr, ...
                              omega_struct, zeta_struct, dt);
dc_cmd   = lowpass_2nd_order(dc_cmd, state_prev.filt_dc, ...
                              omega_struct, zeta_struct, dt);

%% ====================================================================
%  THROTTLE MANAGEMENT
%  ====================================================================
%  Direct pass-through for now. A production system would include:
%  - FADEC (Full Authority Digital Engine Control) interface
%  - Idle/max power protection
%  - Compressor stall prevention at high alpha
throttle_cmd = clamp(throttle_lever, 0, 1.5);

%% ====================================================================
%  OUTPUTS
%  ====================================================================

de_L = de_L_cmd;
de_R = de_R_cmd;
dr   = dr_cmd;
dc   = dc_cmd;

%% ====================================================================
%  UPDATE CONTROLLER STATE
%  ====================================================================
ctrl_debug.state.int_omega     = int_omega;
ctrl_debug.state.Nz_cmd_filt   = Nz_cmd;
ctrl_debug.state.p_cmd_filt    = p_cmd;
ctrl_debug.state.beta_cmd_filt = beta_cmd;

% Structural filter states (for next step)
ctrl_debug.state.filt_de_L = struct('x1', de_L_cmd, 'x2', ...
    (de_L_cmd - state_prev.filt_de_L.x1) / dt);
ctrl_debug.state.filt_de_R = struct('x1', de_R_cmd, 'x2', ...
    (de_R_cmd - state_prev.filt_de_R.x1) / dt);
ctrl_debug.state.filt_dr   = struct('x1', dr_cmd,   'x2', ...
    (dr_cmd   - state_prev.filt_dr.x1) / dt);
ctrl_debug.state.filt_dc   = struct('x1', dc_cmd,   'x2', ...
    (dc_cmd   - state_prev.filt_dc.x1) / dt);

%% Debug signals for telemetry / analysis
ctrl_debug.Nz_cmd      = Nz_cmd;
ctrl_debug.Nz_actual   = Nz;
ctrl_debug.alpha_deg   = alpha_deg;
ctrl_debug.beta_deg    = beta_deg;
ctrl_debug.Mach        = Mach;
ctrl_debug.V           = V;
ctrl_debug.qbar        = qbar;
ctrl_debug.omega_cmd   = omega_cmd;
ctrl_debug.omega_err   = omega_err;
ctrl_debug.omega_dot_des = omega_dot_des;
ctrl_debug.de_sym_cmd  = de_sym_cmd;
ctrl_debug.de_dif_cmd  = de_dif_cmd;
ctrl_debug.alpha_factor = alpha_factor;
ctrl_debug.p_limit_factor = p_limit_factor;

end

%% ====================================================================
%  CONTROL EFFECTIVENESS MATRIX COMPUTATION
%  ====================================================================

function B = compute_control_effectiveness(alpha, Mach, qbar, aircraft)
%  Computes the 3x3 control effectiveness matrix B that maps
%  [de_dif; de_sym; dr] to [L_ctrl; M_ctrl; N_ctrl].
%
%  These are the control power derivatives, looked up from the aero
%  database at the current alpha and Mach number.
%
%  In a real FCS, this matrix is either:
%    (a) Pre-computed lookup tables scheduled on alpha/Mach
%    (b) Identified online using recursive least squares (adaptive)
%    (c) For INDI: estimated from control surface deflections and
%        measured angular acceleration changes

    alpha_deg = rad2deg(alpha);
    alpha_bp  = aircraft.aero.alpha_bp;
    mach_bp   = aircraft.aero.mach_bp;

    % Roll due to differential elevon
    Cl_da = interp2_local(alpha_bp, mach_bp, aircraft.aero.Clda_table, ...
                           alpha_deg, Mach);

    % Pitch due to symmetric elevon
    Cm_de = interp2_local(alpha_bp, mach_bp, aircraft.aero.Cmde_table, ...
                           alpha_deg, Mach);

    % Yaw due to differential elevon (adverse yaw)
    Cn_da = interp2_local(alpha_bp, mach_bp, aircraft.aero.Cnda_table, ...
                           alpha_deg, Mach);

    % Yaw due to rudder
    Cn_dr = interp1_local(mach_bp, aircraft.aero.Cndr_mach, Mach);

    % Roll due to rudder
    Cl_dr = interp1_local(mach_bp, aircraft.aero.Cldr_mach, Mach);

    S = aircraft.S;
    b = aircraft.b;
    c = aircraft.c_bar;

    % B maps [de_dif; de_sym; dr] → [L; M; N] in moment units (Nm)
    %
    %        de_dif (roll)          de_sym (pitch)      dr (yaw)
    B = [qbar*S*b*Cl_da,           0,                  qbar*S*b*Cl_dr;   % L (roll)
         0,                         qbar*S*c*Cm_de,     0;                 % M (pitch)
         qbar*S*b*Cn_da,           0,                  qbar*S*b*Cn_dr];   % N (yaw)

    % Regularize if near-singular (e.g., very low airspeed during takeoff)
    if abs(det(B)) < 1e-6
        B = B + 1e-4 * eye(3);
    end
end

%% ====================================================================
%  HELPER FUNCTIONS
%  ====================================================================

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function y_filt = ref_model_filter(y_cmd, y_prev, tau, dt)
    % First-order lag: provides smooth command shaping
    %   dy/dt = (y_cmd - y) / tau
    alpha_f = dt / (tau + dt);
    y_filt = y_prev + alpha_f * (y_cmd - y_prev);
end

function factor = soft_limit_factor(value, warn_limit, hard_limit)
    % Smooth limiting function: returns 1.0 below warn_limit, 0.0 above
    % hard_limit, smooth cosine blend in between.
    %
    % This prevents the sharp transitions of hard clamps that cause PIO
    % (Pilot Induced Oscillation).
    if value <= warn_limit
        factor = 1.0;
    elseif value >= hard_limit
        factor = 0.0;
    else
        x = (value - warn_limit) / (hard_limit - warn_limit);
        factor = 0.5 * (1 + cos(pi * x));  % Smooth cosine taper
    end
end

function y = lowpass_2nd_order(u, state, omega_n, zeta, dt)
    % Second-order low-pass filter (Tustin/bilinear discretization)
    %   H(s) = omega_n^2 / (s^2 + 2*zeta*omega_n*s + omega_n^2)
    %
    % Simplified implementation using state variables.
    % state.x1 = previous output, state.x2 = previous output derivative
    x1 = state.x1;  % y[k-1]
    x2 = state.x2;  % dy/dt[k-1]

    x2_new = x2 + dt * (omega_n^2 * (u - x1) - 2*zeta*omega_n*x2);
    y      = x1 + dt * x2_new;

    % Output clamped to input range (safety)
    y = clamp(y, -2*abs(u) - 0.1, 2*abs(u) + 0.1);
end

function [T, a, P, rho] = atmos_isa(alt)
    % International Standard Atmosphere (troposphere only)
    T0   = 288.15;     % Sea level temperature (K)
    P0   = 101325;     % Sea level pressure (Pa)
    rho0 = 1.225;      % Sea level density (kg/m^3)
    L    = 0.0065;     % Lapse rate (K/m)
    g0   = 9.81;
    R    = 287.05;     % Gas constant for air

    T   = T0 - L * alt;
    P   = P0 * (T/T0)^(g0/(R*L));
    rho = P / (R * T);
    a   = sqrt(1.4 * R * T);   % Speed of sound
end

function val = interp2_local(x_bp, y_bp, table, x_q, y_q)
    x_q = max(min(x_q, x_bp(end)), x_bp(1));
    y_q = max(min(y_q, y_bp(end)), y_bp(1));
    val = interp2(y_bp, x_bp, table, y_q, x_q, 'linear');
end

function val = interp1_local(bp, table, query)
    query = max(min(query, bp(end)), bp(1));
    val = interp1(bp, table, query, 'linear');
end
