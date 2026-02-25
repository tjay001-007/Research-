%% ========================================================================
%  SETUP_UCAV  —  Master configuration for UCAV INDI flight control system
%  ========================================================================
%
%  Configures all parameters for the autonomous UCAV simulation:
%    1. Aircraft physical and aerodynamic properties (NO canard)
%    2. INDI controller gains and limits
%    3. Guidance law parameters (L1 lateral + PI altitude/speed)
%    4. Mission waypoints (WGS-84 → NED)
%    5. Trim computation and initial conditions
%    6. Simulation parameters
%
%  Aircraft configuration:
%    - Conventional fixed-wing UCAV (wing + horizontal tail + vertical tail)
%    - Control surfaces: aileron, elevator, rudder (3 surfaces)
%    - Statically unstable in pitch (Cm_alpha > 0, CG aft of AC)
%    - Similar to MALE UCAV class (~1800 kg MTOW)
%
%  Usage:
%    >> setup_ucav              % Loads everything to workspace
%    >> run_ucav_mission        % Pure MATLAB simulation
%    >> build_ucav_simulink_model  % Create Simulink model
%
%  ========================================================================

clear; clc;

fprintf('================================================================\n');
fprintf('  UCAV INDI Flight Control System — Parameter Setup\n');
fprintf('================================================================\n\n');

%% ====================================================================
%  1. AIRCRAFT PHYSICAL PROPERTIES
%  ====================================================================
%  Conventional fixed-wing UCAV: wing + H-tail + V-tail
%  NO canard. Three control surfaces: aileron, elevator, rudder.

fprintf('  [1/6] Aircraft properties...\n');

aircraft = struct();

% --- Mass and geometry ---
aircraft.mass    = 1800;          % Mass (kg)
aircraft.S       = 25.0;          % Wing reference area (m^2)
aircraft.b       = 16.0;          % Wingspan (m)
aircraft.c_bar   = 1.6;           % Mean aerodynamic chord (m)

% --- Inertia tensor (kg*m^2) ---
%   Full tensor including product of inertia Ixz.
%   This causes roll-yaw coupling in the equations of motion.
aircraft.Ixx = 3200;              % Roll inertia
aircraft.Iyy = 4800;              % Pitch inertia
aircraft.Izz = 7200;              % Yaw inertia
aircraft.Ixz = 320;               % Roll-yaw product of inertia

% Precompute inertia-related constants (used by EOM and INDI)
aircraft.Gamma  = aircraft.Ixx * aircraft.Izz - aircraft.Ixz^2;

% --- Engine ---
aircraft.engine.thrust_max = 8000;     % Maximum thrust (N)
aircraft.engine.idle_thrust = 200;     % Idle thrust (N)

%% ====================================================================
%  2. AERODYNAMIC DERIVATIVES
%  ====================================================================
%  Stability-derivative based model (linear in alpha, beta, rates, surfaces).
%  Valid for alpha in [-5, 15] deg, beta in [-10, 10] deg.
%
%  Sign conventions (standard stability axis):
%    - Positive alpha = nose up
%    - Positive beta  = nose right (sideslip to right)
%    - Positive da    = right aileron trailing-edge down → right roll (Cl > 0)
%    - Positive de    = elevator trailing-edge down → nose down (Cm < 0)
%    - Positive dr    = rudder trailing-edge left → nose right yaw (Cn > 0)
%
%  CRITICAL: Cm_alpha > 0 makes this aircraft pitch-unstable.
%  Without active control, any pitch perturbation diverges.

fprintf('  [2/6] Aerodynamic derivatives...\n');

aero = struct();

% --- Lift ---
aero.CL_0     =  0.15;           % Zero-alpha lift
aero.CL_alpha =  4.80;           % Lift curve slope (per rad)
aero.CL_de    =  0.38;           % Elevator lift effectiveness (per rad)
aero.CL_q     =  7.50;           % Pitch rate effect on lift (per rad)

% --- Drag (parabolic polar: CD = CD_0 + K*CL^2) ---
aero.CD_0     =  0.025;          % Zero-lift drag
aero.K_drag   =  0.042;          % Induced drag factor (1/(pi*e*AR))

% --- Side force ---
aero.CY_beta  = -0.56;           % Sideslip → side force (per rad)
aero.CY_dr    =  0.16;           % Rudder → side force (per rad)
aero.CY_p     = -0.08;           % Roll rate → side force (per rad)
aero.CY_r     =  0.21;           % Yaw rate → side force (per rad)

% --- Rolling moment ---
aero.Cl_beta  = -0.07;           % Dihedral effect (per rad)
aero.Cl_p     = -0.34;           % Roll damping (per rad) — MUST be negative
aero.Cl_r     =  0.06;           % Yaw-to-roll coupling (per rad)
aero.Cl_da    =  0.17;           % Aileron roll effectiveness (per rad)
aero.Cl_dr    =  0.008;          % Rudder-to-roll coupling (per rad)

% --- Pitching moment ---
%  Cm_alpha > 0 → UNSTABLE (CG aft of aerodynamic center)
%  This is the defining characteristic of a relaxed-static-stability aircraft.
aero.Cm_0     =  0.025;          % Zero-alpha pitching moment
aero.Cm_alpha =  0.08;           % Pitch stiffness — POSITIVE = UNSTABLE
aero.Cm_de    = -0.52;           % Elevator pitch effectiveness (per rad)
aero.Cm_q     = -12.0;           % Pitch damping (per rad) — MUST be negative

% --- Yawing moment ---
aero.Cn_beta  =  0.065;          % Weathercock stability (per rad)
aero.Cn_p     = -0.020;          % Roll-to-yaw coupling (per rad)
aero.Cn_r     = -0.090;          % Yaw damping (per rad) — MUST be negative
aero.Cn_da    = -0.004;          % Adverse yaw from aileron (per rad)
aero.Cn_dr    = -0.080;          % Rudder yaw effectiveness (per rad)

% --- Stall model ---
aero.alpha_stall  = deg2rad(14); % Stall onset angle (rad)
aero.alpha_max    = deg2rad(18); % Full stall angle (rad)
aero.CL_max       = 1.4;        % Maximum lift coefficient at stall

aircraft.aero = aero;

%% ====================================================================
%  3. ACTUATOR SPECIFICATIONS
%  ====================================================================
%  First-order lag + rate limit + position saturation for each surface.

fprintf('  [3/6] Actuator specifications...\n');

act = struct();
act.tau       = 0.025;                    % Time constant (s) → 40 Hz bandwidth

act.aileron.rate_max  = deg2rad(100);     % Max deflection rate (rad/s)
act.aileron.pos_max   = deg2rad(25);      % Max deflection (rad)

act.elevator.rate_max = deg2rad(80);      % Max deflection rate (rad/s)
act.elevator.pos_max  = deg2rad(25);      % Max deflection (rad)

act.rudder.rate_max   = deg2rad(60);      % Max deflection rate (rad/s)
act.rudder.pos_max    = deg2rad(30);      % Max deflection (rad)

aircraft.act = act;

%% ====================================================================
%  4. INDI CONTROLLER GAINS
%  ====================================================================
%  Two-loop cascaded architecture:
%    Outer loop: attitude error → body rate commands (kinematic inversion)
%    Inner loop: INDI → surface deflection increments
%
%  The INDI inner loop uses:
%    delta_new = delta_old + B_inv * (omega_dot_desired - omega_dot_measured)
%  which cancels all aerodynamic nonlinearities incrementally.

fprintf('  [4/6] INDI controller gains...\n');

fcs = struct();
fcs.use_indi = true;              % true = INDI, false = classical NDI fallback

% --- Outer loop: attitude tracking bandwidth (rad/s) ---
%  These set how fast the aircraft tracks attitude commands.
%  Rule: outer loop BW should be ~3-5x slower than inner loop.
fcs.K_phi     = 2.5;             % Roll attitude proportional
fcs.K_theta   = 3.0;             % Pitch attitude proportional (higher for unstable)
fcs.K_beta    = 1.5;             % Sideslip regulation gain

% --- Inner loop: rate tracking (rad/s) ---
%  These set the desired angular acceleration per unit rate error.
fcs.K_p       = 8.0;             % Roll rate proportional
fcs.K_q       = 10.0;            % Pitch rate proportional (highest — unstable axis)
fcs.K_r       = 6.0;             % Yaw rate proportional

% --- Inner loop: integral action (rad/s^2) ---
%  Integral rejects steady-state errors from B-matrix inaccuracy.
%  For INDI, integral gain can be lower than for NDI (better cancellation).
fcs.Ki_p      = 1.5;             % Roll rate integral
fcs.Ki_q      = 3.0;             % Pitch rate integral
fcs.Ki_r      = 1.0;             % Yaw rate integral

% --- Anti-windup limits for integrators (rad/s) ---
fcs.int_lim_p = deg2rad(10);
fcs.int_lim_q = deg2rad(15);
fcs.int_lim_r = deg2rad(8);

% --- Rate command limits (rad/s) ---
fcs.p_max     = deg2rad(120);    % Max roll rate command
fcs.q_max     = deg2rad(40);     % Max pitch rate command
fcs.r_max     = deg2rad(30);     % Max yaw rate command

% --- Envelope protection ---
fcs.alpha_max   = deg2rad(12);   % Max alpha command (deg)
fcs.alpha_min   = deg2rad(-5);   % Min alpha command (deg)
fcs.phi_max     = deg2rad(40);   % Max bank angle (deg)
fcs.nz_max      = 3.5;           % Max normal load factor (g)
fcs.nz_min      = -1.0;          % Min normal load factor (g)

% --- Output rate limiting (rad/s) ---
%  Prevents step changes in surface commands that could excite structural modes.
fcs.da_rate_max = deg2rad(150);  % Aileron command rate limit
fcs.de_rate_max = deg2rad(120);  % Elevator command rate limit
fcs.dr_rate_max = deg2rad(100);  % Rudder command rate limit

%% ====================================================================
%  5. GUIDANCE LAW PARAMETERS
%  ====================================================================
%  Lateral:  L1 nonlinear guidance (same algorithm as PX4/ArduPilot)
%  Altitude: PI altitude hold → pitch angle command
%  Speed:    PI speed hold → throttle command

fprintf('  [5/6] Guidance parameters...\n');

guidance = struct();

% --- L1 lateral guidance ---
guidance.L1_ratio     = 12;      % L1 distance = V * L1_ratio (s)
guidance.L1_min       = 200;     % Minimum L1 distance (m)
guidance.phi_max      = deg2rad(35);  % Max commanded bank angle (rad)

% --- Altitude hold (PI) ---
guidance.Kh           = 0.06;    % Altitude error → climb rate (1/s)
guidance.Ki_h         = 0.008;   % Altitude integral gain
guidance.max_climb    = 5.0;     % Max climb rate command (m/s)
guidance.max_descend  = -4.0;    % Max descend rate command (m/s)
guidance.theta_max    = deg2rad(15);  % Max pitch command from guidance

% --- Speed hold (PI) ---
guidance.Kv           = 0.08;    % Speed error → throttle
guidance.Ki_v         = 0.02;    % Speed integral gain
guidance.thr_max      = 0.95;    % Max throttle
guidance.thr_min      = 0.05;    % Min throttle

% --- Waypoint navigation ---
guidance.accept_rad   = 400;     % Waypoint acceptance radius (m)

%% ====================================================================
%  6. MISSION TRAJECTORY
%  ====================================================================
%  Rectangular racetrack patrol pattern, defined in WGS-84 (lat/lon/alt).
%  All coordinates are converted to NED (North-East-Down) for navigation.
%
%  Pattern: 8 km x 4 km rectangle at 1000 m AGL
%  Location: Near Bengaluru, India (13.0 N, 77.5 E)
%  Speed: 70 m/s (252 km/h) cruise

fprintf('  [6/6] Mission trajectory...\n');

mission = struct();

% Reference origin (WGS-84)
mission.ref_lat = 13.0;          % Reference latitude (deg)
mission.ref_lon = 77.5;          % Reference longitude (deg)

% Waypoints in WGS-84: [latitude_deg, longitude_deg, altitude_m]
mission.waypoints_lla = [
    13.0000,  77.5000,  1000;    % WP1: Start / home
    13.0720,  77.5000,  1000;    % WP2: 8 km North
    13.0720,  77.5370,  1000;    % WP3: 4 km East
    13.0000,  77.5370,  1000;    % WP4: 8 km South
];

% Racetrack loop: after reaching WP4, go back to WP1
mission.racetrack_start = 1;     % Loop start waypoint index
mission.racetrack_end   = 4;     % Loop end waypoint index
mission.num_laps        = 3;     % Number of racetrack laps

% Commanded airspeed
mission.V_cmd = 70;              % Cruise airspeed (m/s)

% Convert WGS-84 to NED coordinates
%   Using flat-earth approximation with WGS-84 ellipsoid radii.
%   Accurate to < 0.1% for distances under 100 km.
ref_lat_rad = deg2rad(mission.ref_lat);
a_earth = 6378137.0;                               % WGS-84 semi-major axis (m)
e2 = 0.00669437999014;                              % WGS-84 eccentricity squared
sin_lat = sin(ref_lat_rad);
R_N = a_earth * (1 - e2) / (1 - e2*sin_lat^2)^1.5; % Meridional radius
R_E = a_earth / sqrt(1 - e2*sin_lat^2);             % Prime vertical radius

num_wp = size(mission.waypoints_lla, 1);
mission.waypoints_ned = zeros(num_wp, 3);

for i = 1:num_wp
    dlat = deg2rad(mission.waypoints_lla(i,1) - mission.ref_lat);
    dlon = deg2rad(mission.waypoints_lla(i,2) - mission.ref_lon);
    mission.waypoints_ned(i,1) = dlat * R_N;                     % North (m)
    mission.waypoints_ned(i,2) = dlon * R_E * cos(ref_lat_rad);  % East (m)
    mission.waypoints_ned(i,3) = -mission.waypoints_lla(i,3);    % Down (m)
end

fprintf('    Waypoints (NED):\n');
for i = 1:num_wp
    fprintf('      WP%d: N=%7.0f m, E=%7.0f m, Alt=%5.0f m\n', ...
        i, mission.waypoints_ned(i,1), mission.waypoints_ned(i,2), ...
        -mission.waypoints_ned(i,3));
end

%% ====================================================================
%  7. TRIM COMPUTATION
%  ====================================================================
%  Compute straight-and-level flight trim at cruise conditions.
%  This provides correct initial conditions for the simulation.

fprintf('\n  Computing trim...\n');

V_trim = mission.V_cmd;
alt_trim = mission.waypoints_lla(1,3);

% Atmosphere at trim altitude
[~, ~, ~, rho_trim] = isa_atmosphere(alt_trim);
qbar_trim = 0.5 * rho_trim * V_trim^2;

% Trim angle of attack: L = W
CL_req = (aircraft.mass * 9.81) / (qbar_trim * aircraft.S);
alpha_trim = (CL_req - aero.CL_0) / aero.CL_alpha;

% Trim elevator: Cm = 0
de_trim = -(aero.Cm_0 + aero.Cm_alpha * alpha_trim) / aero.Cm_de;

% Trim drag and thrust
CL_trim = aero.CL_0 + aero.CL_alpha * alpha_trim;
CD_trim = aero.CD_0 + aero.K_drag * CL_trim^2;
D_trim  = qbar_trim * aircraft.S * CD_trim;
thr_trim = D_trim / aircraft.engine.thrust_max;
thr_trim = max(min(thr_trim, 1.0), 0.0);

% Trim body velocities
u_trim = V_trim * cos(alpha_trim);
w_trim = V_trim * sin(alpha_trim);
theta_trim = alpha_trim;    % gamma = 0 for level flight

fprintf('    V_trim     = %.1f m/s\n', V_trim);
fprintf('    alpha_trim = %.2f deg\n', rad2deg(alpha_trim));
fprintf('    de_trim    = %.2f deg\n', rad2deg(de_trim));
fprintf('    thr_trim   = %.3f\n', thr_trim);
fprintf('    CL_trim    = %.3f\n', CL_trim);
fprintf('    CD_trim    = %.4f\n', CD_trim);

% Store trim values
trim = struct();
trim.V         = V_trim;
trim.alpha     = alpha_trim;
trim.de        = de_trim;
trim.throttle  = thr_trim;
trim.theta     = theta_trim;
trim.qbar      = qbar_trim;

%% ====================================================================
%  8. INITIAL CONDITIONS
%  ====================================================================
%  Aircraft starts at WP1, heading toward WP2, in trimmed level flight.

% Heading from WP1 to WP2
dN = mission.waypoints_ned(2,1) - mission.waypoints_ned(1,1);
dE = mission.waypoints_ned(2,2) - mission.waypoints_ned(1,2);
psi_init = atan2(dE, dN);    % Initial heading (rad)

% State vector: [u, v, w, p, q, r, N, E, D, phi, theta, psi]
initial = struct();
initial.state = [
    u_trim;                   %  1: u (m/s)
    0;                        %  2: v (m/s)
    w_trim;                   %  3: w (m/s)
    0;                        %  4: p (rad/s)
    0;                        %  5: q (rad/s)
    0;                        %  6: r (rad/s)
    mission.waypoints_ned(1,1);  %  7: N (m)
    mission.waypoints_ned(1,2);  %  8: E (m)
    mission.waypoints_ned(1,3);  %  9: D (m) — negative = up
    0;                        % 10: phi (rad)
    theta_trim;               % 11: theta (rad)
    psi_init;                 % 12: psi (rad)
];

% Actuator initial positions (trimmed)
initial.da = 0;               % Aileron trim = 0
initial.de = de_trim;          % Elevator at trim
initial.dr = 0;               % Rudder trim = 0

%% ====================================================================
%  9. SIMULATION PARAMETERS
%  ====================================================================

sim_params = struct();
sim_params.dt       = 0.004;      % Time step (s) = 250 Hz
sim_params.t_end    = 800;        % Simulation duration (s)
sim_params.fs       = 1/0.004;    % Sample rate (Hz)

%% ====================================================================
%  10. STABILITY ANALYSIS — Open-loop pitch divergence
%  ====================================================================

fprintf('\n  Open-loop stability analysis:\n');

% Short-period approximation eigenvalues
%   For unstable aircraft: lambda = Mq/2 +/- sqrt((Mq/2)^2 + M_alpha)
%   where M_alpha = qbar*S*c*Cm_alpha / Iyy, Mq = qbar*S*c^2*Cm_q / (2*Iyy*V)
M_alpha = qbar_trim * aircraft.S * aircraft.c_bar * aero.Cm_alpha / aircraft.Iyy;
Mq      = qbar_trim * aircraft.S * aircraft.c_bar^2 * aero.Cm_q / (2 * aircraft.Iyy * V_trim);

disc = (Mq/2)^2 + M_alpha;
if disc > 0
    lam1 = Mq/2 + sqrt(disc);
    lam2 = Mq/2 - sqrt(disc);
    fprintf('    Short-period eigenvalues: %.3f, %.3f\n', lam1, lam2);
    if lam1 > 0
        t_double = 0.693 / lam1;
        fprintf('    Unstable mode: time-to-double = %.2f s\n', t_double);
        fprintf('    -> INDI controller MUST run at >%.0f Hz to stabilize\n', ...
            max(10, 2/t_double));
    end
else
    fprintf('    Short-period eigenvalues are complex (stable oscillatory)\n');
end

% INDI control effectiveness at trim
B_info = compute_B_matrix(alpha_trim, qbar_trim, aircraft);
fprintf('    B matrix condition number: %.1f\n', cond(B_info));
if cond(B_info) < 100
    fprintf('    -> Good controllability (cond < 100)\n');
else
    fprintf('    -> WARNING: Poor controllability\n');
end

fprintf('\n================================================================\n');
fprintf('  Setup complete. Variables in workspace:\n');
fprintf('    aircraft, fcs, guidance, mission, trim, initial, sim_params\n');
fprintf('================================================================\n\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function [T, a, P, rho] = isa_atmosphere(alt)
%ISA_ATMOSPHERE  International Standard Atmosphere model.
    T0 = 288.15; P0 = 101325; L = 0.0065; R = 287.05; g0 = 9.81;
    alt = max(alt, 0);
    T = max(T0 - L*alt, 216.65);        % Temperature (K)
    P = P0 * (T/T0)^(g0/(R*L));         % Pressure (Pa)
    rho = P / (R * T);                   % Density (kg/m^3)
    a = sqrt(1.4 * R * T);              % Speed of sound (m/s)
end

function B = compute_B_matrix(~, qbar, ac)
%COMPUTE_B_MATRIX  INDI control effectiveness matrix B = J_inv * G.
%   Maps surface deflection increments to angular acceleration increments.
    ar = ac.aero;
    G = qbar * ac.S * [
        ac.b     * ar.Cl_da,   0,                  ac.b     * ar.Cl_dr;
        0,                     ac.c_bar * ar.Cm_de, 0;
        ac.b     * ar.Cn_da,   0,                  ac.b     * ar.Cn_dr
    ];
    % J_inv with Ixz coupling
    Gam = ac.Gamma;
    J_inv = [
         ac.Izz/Gam,   0,          ac.Ixz/Gam;
         0,            1/ac.Iyy,   0;
         ac.Ixz/Gam,   0,          ac.Ixx/Gam
    ];
    B = J_inv * G;
end
