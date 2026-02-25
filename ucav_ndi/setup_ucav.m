%% ========================================================================
%  UCAV NDI SYSTEM SETUP
%  ========================================================================
%
%  MODULE PURPOSE:
%  ---------------
%  Master setup script that configures the entire UCAV autonomous flight
%  control system:
%    1. Aircraft physical parameters (mass, inertia, geometry, aero)
%    2. NDI/INDI inner-loop controller gains
%    3. L1 guidance law parameters
%    4. Mission trajectory definition
%    5. Simulation parameters
%
%  This script MUST be run before:
%    - run_ucav_mission.m  (pure MATLAB simulation)
%    - build_ucav_simulink_model.m  (Simulink model creation)
%
%  All parameters are saved to base workspace as structs:
%    aircraft  - Airframe parameters + aero database
%    fcs       - NDI/INDI controller gains
%    guidance  - L1 + TECS guidance parameters
%    mission   - Waypoints and trajectory
%    initial   - Initial conditions
%    sim_params - Timestep, duration
%
%  ========================================================================

clear all; close all; clc;

fprintf('==========================================================\n');
fprintf(' UCAV AUTONOMOUS FLIGHT CONTROL SYSTEM SETUP\n');
fprintf(' Delta-Canard UCAV + NDI/INDI + L1 Guidance\n');
fprintf(' Trajectory Following in Lat/Lon/Alt\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  1. AIRCRAFT PARAMETERS
%  ====================================================================
%  Uses the same fighter_ndi aero database. We add the path so those
%  functions are accessible.

addpath('../fighter_ndi');

% Execute the aircraft parameter setup from fighter_ndi
% (This loads: aircraft struct with mass, inertia, geometry, aero tables)
fprintf('--- Loading aircraft parameters ---\n');

% Mass and inertia (same as fighter, typical for UCAV class)
aircraft.name    = 'UCAV Delta-Canard (autonomous)';
aircraft.mass    = 9000;
aircraft.Ixx     = 12875;
aircraft.Iyy     = 75674;
aircraft.Izz     = 85552;
aircraft.Ixz     = 1331;
aircraft.Ixy     = 0;
aircraft.Iyz     = 0;

% Geometry
aircraft.S       = 38.4;
aircraft.b       = 8.2;
aircraft.c_bar   = 4.68;

aircraft.static_margin = -0.08;

% Control surface limits
aircraft.de_max  = deg2rad(25);
aircraft.dr_max  = deg2rad(30);
aircraft.dc_max  = deg2rad(25);

% Actuator dynamics
aircraft.actuator.rate_max_de = deg2rad(80);
aircraft.actuator.rate_max_dr = deg2rad(60);
aircraft.actuator.rate_max_dc = deg2rad(60);
aircraft.actuator.tau         = 0.02;

% Engine
aircraft.engine.thrust_max_dry = 53000;
aircraft.engine.thrust_max_ab  = 83000;
aircraft.engine.tau_spool      = 1.5;
aircraft.engine.idle_thrust    = 5000;

% Initial altitude for atmosphere model
aircraft.alt_init = 500;   % Start at 500m (after takeoff)

% ---- Aerodynamic database (same tables as fighter_ndi) ----
aircraft.aero.alpha_bp = [-5, 0, 5, 10, 15, 20, 25, 30];
aircraft.aero.mach_bp  = [0.2, 0.4, 0.6, 0.8, 0.95, 1.1, 1.4];

aircraft.aero.CL_table = [
   -0.18 -0.18 -0.17 -0.16 -0.15 -0.13 -0.10;
    0.05  0.05  0.06  0.06  0.07  0.06  0.05;
    0.30  0.31  0.32  0.33  0.35  0.30  0.25;
    0.56  0.58  0.60  0.62  0.65  0.56  0.46;
    0.82  0.85  0.88  0.91  0.95  0.82  0.65;
    1.05  1.08  1.12  1.16  1.20  1.02  0.80;
    1.18  1.22  1.28  1.32  1.35  1.12  0.88;
    1.15  1.18  1.25  1.30  1.30  1.08  0.85];

aircraft.aero.CD_table = [
    0.022 0.021 0.021 0.024 0.040 0.055 0.048;
    0.018 0.017 0.017 0.020 0.035 0.050 0.044;
    0.022 0.021 0.022 0.026 0.042 0.058 0.050;
    0.040 0.038 0.040 0.048 0.068 0.085 0.072;
    0.075 0.072 0.076 0.088 0.115 0.130 0.110;
    0.128 0.125 0.130 0.148 0.178 0.195 0.165;
    0.200 0.196 0.205 0.228 0.260 0.275 0.235;
    0.280 0.275 0.290 0.315 0.350 0.365 0.315];

aircraft.aero.Cm_table = [
    0.020  0.018  0.015  0.010  0.005 -0.005 -0.010;
    0.040  0.038  0.035  0.028  0.015  0.000 -0.008;
    0.065  0.062  0.058  0.048  0.030  0.008 -0.005;
    0.098  0.095  0.088  0.072  0.048  0.018  0.000;
    0.130  0.126  0.118  0.098  0.068  0.028  0.005;
    0.155  0.150  0.140  0.118  0.082  0.035  0.008;
    0.165  0.160  0.150  0.128  0.088  0.038  0.010;
    0.145  0.140  0.132  0.115  0.078  0.032  0.008];

aircraft.aero.Cmde_table = [
   -0.48 -0.50 -0.52 -0.55 -0.52 -0.45 -0.38;
   -0.50 -0.52 -0.55 -0.58 -0.55 -0.48 -0.40;
   -0.52 -0.54 -0.57 -0.60 -0.57 -0.50 -0.42;
   -0.50 -0.52 -0.55 -0.58 -0.55 -0.48 -0.40;
   -0.46 -0.48 -0.50 -0.52 -0.50 -0.44 -0.36;
   -0.40 -0.42 -0.44 -0.46 -0.44 -0.38 -0.32;
   -0.32 -0.34 -0.36 -0.38 -0.36 -0.30 -0.25;
   -0.22 -0.24 -0.26 -0.28 -0.26 -0.22 -0.18];

aircraft.aero.Clbeta_table = [
   -0.060 -0.058 -0.055 -0.050 -0.045 -0.040 -0.035;
   -0.070 -0.068 -0.065 -0.060 -0.055 -0.048 -0.040;
   -0.085 -0.082 -0.078 -0.072 -0.065 -0.058 -0.048;
   -0.110 -0.108 -0.102 -0.095 -0.085 -0.075 -0.062;
   -0.150 -0.148 -0.140 -0.130 -0.118 -0.100 -0.082;
   -0.200 -0.198 -0.188 -0.175 -0.158 -0.135 -0.110;
   -0.250 -0.248 -0.235 -0.218 -0.198 -0.170 -0.138;
   -0.280 -0.276 -0.262 -0.245 -0.222 -0.190 -0.155];

aircraft.aero.Clda_table = [
    0.140 0.142 0.145 0.150 0.148 0.135 0.120;
    0.148 0.150 0.155 0.160 0.158 0.142 0.125;
    0.152 0.155 0.160 0.165 0.162 0.148 0.130;
    0.148 0.150 0.155 0.160 0.158 0.142 0.125;
    0.130 0.132 0.138 0.142 0.140 0.128 0.112;
    0.105 0.108 0.112 0.118 0.115 0.105 0.092;
    0.075 0.078 0.082 0.088 0.085 0.078 0.068;
    0.048 0.050 0.055 0.058 0.055 0.050 0.044];

aircraft.aero.Cnbeta_table = [
    0.095 0.098 0.100 0.105 0.102 0.095 0.085;
    0.100 0.102 0.105 0.110 0.108 0.100 0.088;
    0.095 0.098 0.100 0.105 0.102 0.095 0.085;
    0.080 0.082 0.085 0.090 0.088 0.082 0.072;
    0.055 0.058 0.062 0.065 0.062 0.058 0.050;
    0.025 0.028 0.030 0.035 0.032 0.028 0.022;
   -0.010 -0.008 -0.005 0.000 -0.002 -0.005 -0.008;
   -0.045 -0.042 -0.038 -0.032 -0.035 -0.038 -0.042];

aircraft.aero.Cnda_table = [
   -0.008 -0.008 -0.009 -0.010 -0.009 -0.008 -0.007;
   -0.010 -0.010 -0.012 -0.013 -0.012 -0.010 -0.009;
   -0.015 -0.015 -0.018 -0.020 -0.018 -0.015 -0.012;
   -0.022 -0.022 -0.025 -0.028 -0.025 -0.022 -0.018;
   -0.032 -0.032 -0.035 -0.038 -0.035 -0.032 -0.026;
   -0.045 -0.044 -0.048 -0.052 -0.048 -0.042 -0.035;
   -0.058 -0.056 -0.060 -0.065 -0.060 -0.054 -0.045;
   -0.065 -0.063 -0.068 -0.072 -0.068 -0.060 -0.050];

aircraft.aero.CLde_mach  = [0.28 0.30 0.32 0.35 0.33 0.28 0.22];
aircraft.aero.CLdc_mach  = [0.20 0.22 0.24 0.26 0.25 0.20 0.16];
aircraft.aero.CYbeta_mach= [-0.72 -0.75 -0.78 -0.82 -0.80 -0.72 -0.62];
aircraft.aero.CYdr_mach  = [0.15 0.16 0.17 0.18 0.17 0.15 0.12];
aircraft.aero.Clp_mach   = [-0.35 -0.36 -0.38 -0.40 -0.38 -0.34 -0.28];
aircraft.aero.Clr_mach   = [0.08 0.08 0.09 0.10 0.09 0.08 0.07];
aircraft.aero.Cldr_mach  = [0.012 0.013 0.014 0.015 0.014 0.012 0.010];
aircraft.aero.Cmq_mach   = [-8.5 -9.0 -9.5 -10.5 -12.0 -10.0 -8.0];
aircraft.aero.Cmdc_mach  = [-0.35 -0.38 -0.40 -0.42 -0.40 -0.35 -0.28];
aircraft.aero.Cnr_mach   = [-0.18 -0.20 -0.22 -0.25 -0.22 -0.18 -0.15];
aircraft.aero.Cnp_mach   = [-0.03 -0.03 -0.04 -0.04 -0.04 -0.03 -0.02];
aircraft.aero.Cndr_mach  = [-0.080 -0.085 -0.090 -0.095 -0.090 -0.080 -0.068];

fprintf('  Aircraft: %s\n', aircraft.name);
fprintf('  Mass: %.0f kg, Static margin: %.0f%% MAC (UNSTABLE)\n\n', ...
    aircraft.mass, aircraft.static_margin*100);

%% ====================================================================
%  2. NDI/INDI CONTROLLER GAINS
%  ====================================================================

fcs.use_indi = true;        % INDI primary, NDI fallback
fcs.dt       = 1/250;       % 250 Hz

% Outer loop (attitude)
fcs.K_phi    = 3.0;
fcs.K_theta  = 4.0;
fcs.K_beta   = 3.0;

% Inner loop (rate)
fcs.K_p  = 10.0;   fcs.Ki_p = 2.0;
fcs.K_q  = 12.0;   fcs.Ki_q = 4.0;
fcs.K_r  = 8.0;    fcs.Ki_r = 2.0;

fcs.int_lim = 0.5;

% Rate limits
fcs.p_max = deg2rad(200);
fcs.q_max = deg2rad(60);
fcs.r_max = deg2rad(40);

% Envelope protection
fcs.alpha_warn_deg = 22;
fcs.alpha_max_deg  = 26;
fcs.Nz_max         = 9.0;
fcs.Nz_min         = -3.0;
fcs.phi_max        = deg2rad(65);
fcs.K_theta_approx = 5.0;      % Approximate Nz per radian of theta change

% Canard scheduling
fcs.dc_schedule_alpha = 0.3;
fcs.dc_schedule_q     = 0.1;

% Surface command rate limit (for output smoothing)
fcs.surface_cmd_rate_max = deg2rad(60);  % deg/s

fprintf('  FCS mode: %s\n', tern(fcs.use_indi, 'INDI (primary)', 'NDI'));
fprintf('  Inner loop: K_q = %.1f rad/s\n', fcs.K_q);
fprintf('  Outer loop: K_theta = %.1f rad/s\n\n', fcs.K_theta);

%% ====================================================================
%  3. L1 GUIDANCE PARAMETERS
%  ====================================================================

% L1 lateral guidance
guidance.L1_ratio = 20;        % L1 = V * L1_ratio (seconds of lookahead)
guidance.L1_min   = 1500;      % Minimum L1 distance (m)
guidance.phi_max  = deg2rad(55); % Max bank angle for trajectory tracking

% Altitude controller (TECS-inspired)
guidance.K_alt      = 0.5;     % Altitude to climb-rate gain (1/s)
guidance.K_hdot     = 0.08;    % Climb-rate to gamma gain
guidance.Ki_hdot    = 0.02;    % Integral on climb rate error
guidance.int_hdot_lim = 0.3;   % Integrator limit (rad)
guidance.hdot_max   = 30;      % Max climb/descent rate (m/s)
guidance.gamma_max  = deg2rad(20); % Max flight path angle
guidance.theta_max  = deg2rad(30); % Max pitch angle command

% Trim alpha for flight path to pitch conversion
guidance.alpha_trim_1g = deg2rad(3); % Alpha at 1g level flight (rad)

% Speed controller
guidance.K_V       = 0.005;    % Speed-to-throttle gain
guidance.Ki_V      = 0.002;    % Integral on speed error
guidance.int_V_lim = 5.0;      % Integrator limit

fprintf('  Guidance: L1 lateral + TECS longitudinal\n');
fprintf('  L1 ratio: %.0f s (L1 = %.0f m at 180 m/s)\n', ...
    guidance.L1_ratio, 180*guidance.L1_ratio);
fprintf('  Max bank: %.0f deg\n\n', rad2deg(guidance.phi_max));

%% ====================================================================
%  4. MISSION TRAJECTORY
%  ====================================================================

mission = ucav_define_mission();

%% ====================================================================
%  5. INITIAL CONDITIONS
%  ====================================================================

% Start at WP0 position, heading toward WP1, at climb speed
wp0_ned = mission.wp_ned(1, :);
wp1_ned = mission.wp_ned(2, :);

% Initial heading: toward WP1
psi_init = atan2(wp1_ned(2) - wp0_ned(2), wp1_ned(1) - wp0_ned(1));

V_init     = 120;           % Initial airspeed (m/s)
theta_init = deg2rad(5);    % Slight pitch-up (climbing)
alpha_init = deg2rad(3);    % Trim alpha

initial.u     = V_init * cos(alpha_init);
initial.v     = 0;
initial.w     = V_init * sin(alpha_init);
initial.p     = 0;
initial.q     = 0;
initial.r     = 0;
initial.x     = wp0_ned(1);
initial.y     = wp0_ned(2);
initial.z     = wp0_ned(3);     % Down (negative = above origin)
initial.phi   = 0;
initial.theta = theta_init;
initial.psi   = psi_init;

initial.state = [initial.u; initial.v; initial.w;
                 initial.p; initial.q; initial.r;
                 initial.x; initial.y; initial.z;
                 initial.phi; initial.theta; initial.psi];

initial.de_L = deg2rad(-2);
initial.de_R = deg2rad(-2);
initial.dr   = 0;
initial.dc   = deg2rad(3);
initial.throttle = 0.4;

fprintf('Initial Conditions:\n');
fprintf('  Position:  N=%.0fm, E=%.0fm, Alt=%.0fm\n', ...
    initial.x, initial.y, -initial.z);
fprintf('  Heading:   %.1f deg (toward WP1)\n', rad2deg(psi_init));
fprintf('  Airspeed:  %.0f m/s\n', V_init);
fprintf('\n');

%% ====================================================================
%  6. SIMULATION PARAMETERS
%  ====================================================================

sim_params.dt    = fcs.dt;        % 0.004 s = 250 Hz
sim_params.t_end = 600;           % 10 minutes (adjust for full mission)

fprintf('Simulation: dt = %.4f s (%d Hz), duration = %.0f s (%.1f min)\n\n', ...
    sim_params.dt, round(1/sim_params.dt), sim_params.t_end, sim_params.t_end/60);

fprintf('==========================================================\n');
fprintf(' SETUP COMPLETE — Run: run_ucav_mission\n');
fprintf('==========================================================\n\n');

function r = tern(c,a,b)
    if c, r=a; else, r=b; end
end
