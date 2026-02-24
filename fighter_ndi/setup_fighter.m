%% Fighter Aircraft + NDI Flight Control System Setup
%  Configures a generic delta-canard light combat aircraft (LCA class)
%  with realistic parameters based on publicly available data for
%  aircraft in the Tejas/Gripen/Rafale class.
%
%  This script sets up:
%    1. Aircraft mass, inertia, and geometry
%    2. Aerodynamic database (lookup tables vs alpha and Mach)
%    3. Control surface limits and actuator parameters
%    4. NDI/INDI flight control system gains
%    5. Structural filter parameters
%    6. Initial conditions
%
%  After running this script, launch: run_fighter_sim
%
%  Data sources (all open literature):
%    - Stevens & Lewis, "Aircraft Control and Simulation" (F-16 model)
%    - Jategaonkar, "Flight Vehicle System Identification"
%    - ADA/DRDO published Tejas LCA reports
%    - Brockhaus, "Flugregelung" (Flight Control)
%    - Etkin & Reid, "Dynamics of Flight"

clear all; close all; clc;

fprintf('==========================================================\n');
fprintf(' FIGHTER NDI SETUP\n');
fprintf(' Generic Delta-Canard Light Combat Aircraft\n');
fprintf(' Relaxed Static Stability Configuration\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  1. AIRCRAFT PHYSICAL PROPERTIES
%  ====================================================================

aircraft.name = 'Generic LCA (Tejas/Gripen class)';

% --- Mass properties (combat weight, mid-CG) ---
aircraft.mass = 9000;        % kg (typical combat weight with 50% internal fuel)
aircraft.Ixx  = 12875;       % kg*m^2  (roll axis)
aircraft.Iyy  = 75674;       % kg*m^2  (pitch axis)
aircraft.Izz  = 85552;       % kg*m^2  (yaw axis)
aircraft.Ixz  = 1331;        % kg*m^2  (roll-yaw product of inertia)
                              %   THIS IS CRITICAL — causes inertia coupling
                              %   in rolls at high alpha. Swept wing aircraft
                              %   always have significant Ixz.
aircraft.Ixy  = 0;           % Symmetric aircraft
aircraft.Iyz  = 0;

% --- Geometry ---
aircraft.S     = 38.4;       % Wing reference area (m^2)
aircraft.b     = 8.2;        % Wing span (m)
aircraft.c_bar = 4.68;       % Mean aerodynamic chord (m)

% --- CG position (fraction of MAC, from leading edge) ---
aircraft.xcg_mac = 0.35;     % 35% MAC (aft of neutral point → unstable)

% --- Static margin ---
%  Negative = CG aft of aerodynamic center = UNSTABLE
%  Typical fighters: -5% to -15% MAC
%  This aircraft: ~-8% MAC
aircraft.static_margin = -0.08;  % -8% MAC

fprintf('Aircraft: %s\n', aircraft.name);
fprintf('  Mass:    %.0f kg\n', aircraft.mass);
fprintf('  Ixx:     %.0f kg*m^2\n', aircraft.Ixx);
fprintf('  Iyy:     %.0f kg*m^2\n', aircraft.Iyy);
fprintf('  Izz:     %.0f kg*m^2\n', aircraft.Izz);
fprintf('  Ixz:     %.0f kg*m^2  <-- Inertia coupling!\n', aircraft.Ixz);
fprintf('  S:       %.1f m^2\n', aircraft.S);
fprintf('  b:       %.1f m\n', aircraft.b);
fprintf('  c_bar:   %.2f m\n', aircraft.c_bar);
fprintf('  Static margin: %.0f%% MAC  (UNSTABLE)\n\n', ...
         aircraft.static_margin * 100);

%% ====================================================================
%  2. CONTROL SURFACE DEFINITIONS AND LIMITS
%  ====================================================================
%
%  Control surfaces for a delta-canard fighter:
%    - LEFT ELEVON:  Combined elevator + aileron on left wing TE
%    - RIGHT ELEVON: Combined elevator + aileron on right wing TE
%    - RUDDER:       On vertical tail
%    - CANARD:       Close-coupled canard (primarily pitch trim)
%
%  Symmetric elevon deflection → pitch control (like elevator)
%  Differential elevon deflection → roll control (like ailerons)

aircraft.de_max = deg2rad(25);     % Elevon max deflection (rad)
aircraft.dr_max = deg2rad(30);     % Rudder max deflection (rad)
aircraft.dc_max = deg2rad(25);     % Canard max deflection (rad)

% Actuator dynamics
aircraft.actuator.rate_max_de = deg2rad(80);   % Elevon rate limit (rad/s) = 80 deg/s
aircraft.actuator.rate_max_dr = deg2rad(60);   % Rudder rate limit (rad/s)
aircraft.actuator.rate_max_dc = deg2rad(60);   % Canard rate limit (rad/s)
aircraft.actuator.tau         = 0.02;          % Actuator time constant (s) = 50 Hz bandwidth
                                                % This is a first-order lag model:
                                                % H(s) = 1 / (tau*s + 1)

fprintf('Control Surfaces:\n');
fprintf('  Elevon: +/- %.0f deg, rate limit %.0f deg/s\n', ...
    rad2deg(aircraft.de_max), rad2deg(aircraft.actuator.rate_max_de));
fprintf('  Rudder: +/- %.0f deg, rate limit %.0f deg/s\n', ...
    rad2deg(aircraft.dr_max), rad2deg(aircraft.actuator.rate_max_dr));
fprintf('  Canard: +/- %.0f deg, rate limit %.0f deg/s\n', ...
    rad2deg(aircraft.dc_max), rad2deg(aircraft.actuator.rate_max_dc));
fprintf('  Actuator bandwidth: %.0f Hz (tau = %.0f ms)\n\n', ...
    1/(2*pi*aircraft.actuator.tau), aircraft.actuator.tau*1000);

%% ====================================================================
%  3. ENGINE
%  ====================================================================

aircraft.engine.thrust_max_dry = 53000;   % Max dry thrust (N) ~53 kN
aircraft.engine.thrust_max_ab  = 83000;   % Max afterburner thrust (N) ~83 kN
aircraft.engine.tau_spool      = 1.5;     % Engine spool time constant (s)
aircraft.engine.idle_thrust    = 5000;    % Idle thrust (N)

fprintf('Engine:\n');
fprintf('  Max dry:    %.0f kN\n', aircraft.engine.thrust_max_dry/1000);
fprintf('  Max AB:     %.0f kN\n', aircraft.engine.thrust_max_ab/1000);
fprintf('  Spool time: %.1f s\n\n', aircraft.engine.tau_spool);

%% ====================================================================
%  4. AERODYNAMIC DATABASE
%  ====================================================================
%
%  These lookup tables represent the output of a wind tunnel test
%  campaign. In reality, each table would have hundreds of data points.
%  The tables here capture the essential nonlinear character of a
%  delta-canard configuration:
%
%  KEY PHYSICS:
%  - Delta wing generates strong leading-edge vortices at high alpha
%    → CL continues to increase well beyond where a conventional wing stalls
%    → But vortex breakdown causes sudden lateral instability
%  - Canard provides pitch control and delays wing stall
%  - Aft CG → positive Cm_alpha (UNSTABLE pitch moment slope)
%  - Transonic: aerodynamic center shifts aft → less unstable or even stable
%  - Strong roll-yaw coupling at high alpha due to wing sweep + Ixz

% Table breakpoints
aircraft.aero.alpha_bp = [-5, 0, 5, 10, 15, 20, 25, 30];     % deg
aircraft.aero.mach_bp  = [0.2, 0.4, 0.6, 0.8, 0.95, 1.1, 1.4];  % Mach

n_alpha = length(aircraft.aero.alpha_bp);
n_mach  = length(aircraft.aero.mach_bp);

% ---- CL(alpha, Mach) ----
%  Delta wing: high CLalpha, gradual stall, CL_max at ~30-35 deg alpha
%  Rows = alpha breakpoints, Columns = Mach breakpoints
aircraft.aero.CL_table = [
%   M=0.2   0.4     0.6     0.8     0.95    1.1     1.4
   -0.18   -0.18   -0.17   -0.16   -0.15   -0.13   -0.10;   % a=-5
    0.05    0.05    0.06    0.06    0.07    0.06    0.05;   % a= 0
    0.30    0.31    0.32    0.33    0.35    0.30    0.25;   % a= 5
    0.56    0.58    0.60    0.62    0.65    0.56    0.46;   % a=10
    0.82    0.85    0.88    0.91    0.95    0.82    0.65;   % a=15
    1.05    1.08    1.12    1.16    1.20    1.02    0.80;   % a=20
    1.18    1.22    1.28    1.32    1.35    1.12    0.88;   % a=25
    1.15    1.18    1.25    1.30    1.30    1.08    0.85;   % a=30 (vortex breakdown starts)
];

% ---- CD(alpha, Mach) ----
%  Includes zero-lift drag + induced drag + transonic wave drag
aircraft.aero.CD_table = [
%   M=0.2   0.4     0.6     0.8     0.95    1.1     1.4
    0.022   0.021   0.021   0.024   0.040   0.055   0.048;  % a=-5
    0.018   0.017   0.017   0.020   0.035   0.050   0.044;  % a= 0
    0.022   0.021   0.022   0.026   0.042   0.058   0.050;  % a= 5
    0.040   0.038   0.040   0.048   0.068   0.085   0.072;  % a=10
    0.075   0.072   0.076   0.088   0.115   0.130   0.110;  % a=15
    0.128   0.125   0.130   0.148   0.178   0.195   0.165;  % a=20
    0.200   0.196   0.205   0.228   0.260   0.275   0.235;  % a=25
    0.280   0.275   0.290   0.315   0.350   0.365   0.315;  % a=30
];

% ---- Cm(alpha, Mach) — THE INSTABILITY ----
%
%  THIS IS THE CRITICAL TABLE.
%  A stable aircraft would have dCm/dalpha < 0 everywhere.
%  Our relaxed-stability fighter has dCm/dalpha > 0 at subsonic:
%    - Cm increases with alpha → nose pitches up further
%    - Without FCS, this diverges in ~150 ms
%  At transonic/supersonic, the AC shifts aft and stability increases.
aircraft.aero.Cm_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
    0.020    0.018    0.015    0.010    0.005   -0.005   -0.010;  % a=-5
    0.040    0.038    0.035    0.028    0.015    0.000   -0.008;  % a= 0
    0.065    0.062    0.058    0.048    0.030    0.008   -0.005;  % a= 5
    0.098    0.095    0.088    0.072    0.048    0.018    0.000;  % a=10
    0.130    0.126    0.118    0.098    0.068    0.028    0.005;  % a=15
    0.155    0.150    0.140    0.118    0.082    0.035    0.008;  % a=20
    0.165    0.160    0.150    0.128    0.088    0.038    0.010;  % a=25
    0.145    0.140    0.132    0.115    0.078    0.032    0.008;  % a=30 (vortex breakdown)
];

% ---- Cm_de(alpha, Mach) — Elevator (symmetric elevon) pitch control power ----
%  Negative: trailing-edge-down elevon produces nose-down moment
%  Effectiveness reduces at high alpha (wake impingement)
aircraft.aero.Cmde_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
   -0.48    -0.50    -0.52    -0.55    -0.52    -0.45    -0.38;  % a=-5
   -0.50    -0.52    -0.55    -0.58    -0.55    -0.48    -0.40;  % a= 0
   -0.52    -0.54    -0.57    -0.60    -0.57    -0.50    -0.42;  % a= 5
   -0.50    -0.52    -0.55    -0.58    -0.55    -0.48    -0.40;  % a=10
   -0.46    -0.48    -0.50    -0.52    -0.50    -0.44    -0.36;  % a=15
   -0.40    -0.42    -0.44    -0.46    -0.44    -0.38    -0.32;  % a=20
   -0.32    -0.34    -0.36    -0.38    -0.36    -0.30    -0.25;  % a=25
   -0.22    -0.24    -0.26    -0.28    -0.26    -0.22    -0.18;  % a=30
];

% ---- Cl_beta(alpha, Mach) — Dihedral effect ----
%  Becomes strongly negative at high alpha (delta wing + sweep effect)
aircraft.aero.Clbeta_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
   -0.060   -0.058   -0.055   -0.050   -0.045   -0.040   -0.035;
   -0.070   -0.068   -0.065   -0.060   -0.055   -0.048   -0.040;
   -0.085   -0.082   -0.078   -0.072   -0.065   -0.058   -0.048;
   -0.110   -0.108   -0.102   -0.095   -0.085   -0.075   -0.062;
   -0.150   -0.148   -0.140   -0.130   -0.118   -0.100   -0.082;
   -0.200   -0.198   -0.188   -0.175   -0.158   -0.135   -0.110;
   -0.250   -0.248   -0.235   -0.218   -0.198   -0.170   -0.138;
   -0.280   -0.276   -0.262   -0.245   -0.222   -0.190   -0.155;
];

% ---- Cl_da(alpha, Mach) — Differential elevon roll effectiveness ----
%  Reduces at high alpha as flow separates on the wing
aircraft.aero.Clda_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
    0.140    0.142    0.145    0.150    0.148    0.135    0.120;
    0.148    0.150    0.155    0.160    0.158    0.142    0.125;
    0.152    0.155    0.160    0.165    0.162    0.148    0.130;
    0.148    0.150    0.155    0.160    0.158    0.142    0.125;
    0.130    0.132    0.138    0.142    0.140    0.128    0.112;
    0.105    0.108    0.112    0.118    0.115    0.105    0.092;
    0.075    0.078    0.082    0.088    0.085    0.078    0.068;
    0.048    0.050    0.055    0.058    0.055    0.050    0.044;
];

% ---- Cn_beta(alpha, Mach) — Weathercock stability ----
%  Positive at low alpha (stable directionally)
%  Can go negative at high alpha → directional departure!
aircraft.aero.Cnbeta_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
    0.095    0.098    0.100    0.105    0.102    0.095    0.085;
    0.100    0.102    0.105    0.110    0.108    0.100    0.088;
    0.095    0.098    0.100    0.105    0.102    0.095    0.085;
    0.080    0.082    0.085    0.090    0.088    0.082    0.072;
    0.055    0.058    0.062    0.065    0.062    0.058    0.050;
    0.025    0.028    0.030    0.035    0.032    0.028    0.022;
   -0.010   -0.008   -0.005    0.000   -0.002   -0.005   -0.008;  % Unstable!
   -0.045   -0.042   -0.038   -0.032   -0.035   -0.038   -0.042;  % Very unstable!
];

% ---- Cn_da(alpha, Mach) — Adverse yaw from differential elevon ----
aircraft.aero.Cnda_table = [
%   M=0.2    0.4      0.6      0.8      0.95     1.1      1.4
   -0.008   -0.008   -0.009   -0.010   -0.009   -0.008   -0.007;
   -0.010   -0.010   -0.012   -0.013   -0.012   -0.010   -0.009;
   -0.015   -0.015   -0.018   -0.020   -0.018   -0.015   -0.012;
   -0.022   -0.022   -0.025   -0.028   -0.025   -0.022   -0.018;
   -0.032   -0.032   -0.035   -0.038   -0.035   -0.032   -0.026;
   -0.045   -0.044   -0.048   -0.052   -0.048   -0.042   -0.035;
   -0.058   -0.056   -0.060   -0.065   -0.060   -0.054   -0.045;
   -0.065   -0.063   -0.068   -0.072   -0.068   -0.060   -0.050;
];

% ---- 1D tables (Mach-dependent only) ----

% Elevator lift effectiveness
aircraft.aero.CLde_mach = [0.28, 0.30, 0.32, 0.35, 0.33, 0.28, 0.22];

% Canard lift effectiveness
aircraft.aero.CLdc_mach = [0.20, 0.22, 0.24, 0.26, 0.25, 0.20, 0.16];

% Side force due to sideslip
aircraft.aero.CYbeta_mach = [-0.72, -0.75, -0.78, -0.82, -0.80, -0.72, -0.62];

% Side force due to rudder
aircraft.aero.CYdr_mach = [0.15, 0.16, 0.17, 0.18, 0.17, 0.15, 0.12];

% Roll damping
aircraft.aero.Clp_mach = [-0.35, -0.36, -0.38, -0.40, -0.38, -0.34, -0.28];

% Roll due to yaw rate
aircraft.aero.Clr_mach = [0.08, 0.08, 0.09, 0.10, 0.09, 0.08, 0.07];

% Roll due to rudder
aircraft.aero.Cldr_mach = [0.012, 0.013, 0.014, 0.015, 0.014, 0.012, 0.010];

% Pitch damping (Cmq + Cm_alpha_dot lumped together)
aircraft.aero.Cmq_mach = [-8.5, -9.0, -9.5, -10.5, -12.0, -10.0, -8.0];

% Canard pitch effectiveness
aircraft.aero.Cmdc_mach = [-0.35, -0.38, -0.40, -0.42, -0.40, -0.35, -0.28];

% Yaw damping
aircraft.aero.Cnr_mach = [-0.18, -0.20, -0.22, -0.25, -0.22, -0.18, -0.15];

% Yaw due to roll rate
aircraft.aero.Cnp_mach = [-0.03, -0.03, -0.04, -0.04, -0.04, -0.03, -0.02];

% Yaw due to rudder
aircraft.aero.Cndr_mach = [-0.080, -0.085, -0.090, -0.095, -0.090, -0.080, -0.068];

fprintf('Aerodynamic Database:\n');
fprintf('  Alpha range: [%d, %d] deg\n', ...
    aircraft.aero.alpha_bp(1), aircraft.aero.alpha_bp(end));
fprintf('  Mach range:  [%.1f, %.1f]\n', ...
    aircraft.aero.mach_bp(1), aircraft.aero.mach_bp(end));
fprintf('  PITCH INSTABILITY: Cm slope positive at subsonic Mach!\n');
fprintf('    Cm(a=0,M=0.4) = %.3f\n', aircraft.aero.Cm_table(2,2));
fprintf('    Cm(a=15,M=0.4) = %.3f\n', aircraft.aero.Cm_table(5,2));
fprintf('    dCm/dalpha ~ +%.4f /deg  (UNSTABLE)\n', ...
    (aircraft.aero.Cm_table(5,2) - aircraft.aero.Cm_table(2,2)) / 15);
fprintf('\n');

%% ====================================================================
%  5. NDI FLIGHT CONTROL SYSTEM GAINS
%  ====================================================================

fprintf('Flight Control System Configuration:\n\n');

% --- Control mode ---
fcs.use_indi = true;           % true = INDI (modern, sensor-based)
                                % false = classical NDI (model-based)
if fcs.use_indi
    fprintf('  Mode: INCREMENTAL NDI (INDI)\n');
    fprintf('  → Does not require M0 model — uses measured angular accel\n');
    fprintf('  → More robust to aero model errors, CG shifts, damage\n');
else
    fprintf('  Mode: CLASSICAL NDI\n');
    fprintf('  → Requires accurate onboard aero model for M0 cancellation\n');
end
fprintf('\n');

% --- Controller sample rate ---
fcs.dt = 1/250;               % 250 Hz = 4 ms (typical for modern FBW)
                                % Eurofighter: 250 Hz, Tejas: 200 Hz
                                % F-35: 400+ Hz

% --- Command shaping (Stage 1) ---
fcs.Nz_per_stick  = 7.0;      % g's per full stick deflection
                                % Most fighters: 7-9 g at full aft stick
fcs.p_max_cmd     = deg2rad(250);  % Max commanded roll rate (rad/s)
                                    % ~250 deg/s is typical for a fighter
fcs.beta_max_cmd  = deg2rad(5);    % Max commanded sideslip (rad)
fcs.tau_ref       = 0.15;          % Reference model time constant (s)
                                    % Shapes the "feel" of the aircraft

% --- Carefree handling limits (Stage 2) ---
fcs.alpha_warn_deg = 22;      % Alpha warning start (deg)
fcs.alpha_max_deg  = 26;      % Alpha hard limit (deg)
                                % Tejas: 24 deg, F-16: 26 deg, Typhoon: 28 deg
fcs.Nz_max         = 9.0;     % Max positive g (structural limit)
fcs.Nz_min         = -3.0;    % Max negative g
fcs.beta_warn_deg  = 8;       % Beta warning start (deg)
fcs.beta_max_deg   = 12;      % Beta hard limit (deg)

fprintf('  Carefree Handling:\n');
fprintf('    Alpha limit: %.0f deg\n', fcs.alpha_max_deg);
fprintf('    g limits:    %+.0f / %+.0f g\n', fcs.Nz_max, fcs.Nz_min);
fprintf('    Beta limit:  %.0f deg\n', fcs.beta_max_deg);
fprintf('\n');

% --- Outer loop gains (Stage 3) ---
fcs.K_Nz   = 1.5;             % Nz tracking gain (1/s)
                                % Higher = faster g response
fcs.K_phi   = 2.5;            % Roll attitude hold gain (rad/s)
fcs.K_beta  = 3.0;            % Sideslip tracking gain (rad/s)

% --- Inner loop rate gains (Stage 4) ---
%
%  These are the most critical gains for an unstable aircraft.
%  They set the closed-loop angular rate response AFTER NDI/INDI
%  cancellation of the plant dynamics.
%
%  For perfect inversion: closed-loop pole at -K_x
%  For pitch: K_q MUST exceed the open-loop divergence rate.
%
%  Open-loop divergence rate (at M=0.6, 3km altitude):
%    rho = 0.91 kg/m^3, V = 200 m/s, qbar = 18200 Pa
%    Cmalpha ~ +0.004 /deg ~ +0.23 /rad
%    lambda_diverge = sqrt(qbar * S * c_bar * Cmalpha / Iyy)
%                   = sqrt(18200 * 38.4 * 4.68 * 0.23 / 75674)
%                   ≈ 4.5 rad/s
%
%  So K_q must be >> 4.5 rad/s. We use K_q = 12 → ~2.7x margin.

V_design  = 200;   % Design airspeed (m/s) ~ M=0.6 at 3km
rho_design = 0.91; % Density at 3km (kg/m^3)
qbar_design = 0.5 * rho_design * V_design^2;
Cmalpha_rad = (aircraft.aero.Cm_table(5,3) - aircraft.aero.Cm_table(2,3)) / ...
              deg2rad(15);  % Average Cmalpha at M=0.6
diverge_rate = sqrt(qbar_design * aircraft.S * aircraft.c_bar * ...
               abs(Cmalpha_rad) / aircraft.Iyy);

fprintf('  Open-loop pitch divergence rate: %.1f rad/s (time to double: %.0f ms)\n', ...
    diverge_rate, log(2)/diverge_rate * 1000);
fprintf('\n');

fcs.K_p  = 10.0;              % Roll rate gain (rad/s)
fcs.K_q  = 12.0;              % Pitch rate gain (rad/s)
                                % MUST BE >> divergence_rate
fcs.K_r  = 8.0;               % Yaw rate gain (rad/s)

% Integral gains (reject model errors / CG shift / store release)
fcs.Ki_p = 2.0;               % Roll integral (rad/s^2)
fcs.Ki_q = 4.0;               % Pitch integral (rad/s^2)
fcs.Ki_r = 2.0;               % Yaw integral (rad/s^2)

fcs.int_lim = 0.5;            % Integrator anti-windup limit (rad)

fprintf('  Inner Loop Gains:\n');
fprintf('    K_p  = %.1f rad/s  (roll rate bandwidth)\n', fcs.K_p);
fprintf('    K_q  = %.1f rad/s  (pitch rate bandwidth) — %.1fx divergence\n', ...
    fcs.K_q, fcs.K_q / diverge_rate);
fprintf('    K_r  = %.1f rad/s  (yaw rate bandwidth)\n', fcs.K_r);
fprintf('    Ki_q = %.1f rad/s^2 (pitch integral)\n', fcs.Ki_q);
fprintf('\n');

% Rate limits (structural / handling)
fcs.q_max = deg2rad(60);      % Max pitch rate command (rad/s)
fcs.p_max_struct = deg2rad(300);  % Structural roll rate limit
fcs.r_max = deg2rad(40);      % Max yaw rate command (rad/s)

% --- Canard scheduling (pitch trim assist) ---
fcs.dc_schedule_alpha = 0.3;  % Canard follows alpha to reduce elevon trim
fcs.dc_schedule_q     = 0.1;  % Small q-damping via canard

% --- Structural filters (Stage 6) ---
%  Wing first bending mode: ~8 Hz for a fighter
%  Fuselage first bending mode: ~12 Hz
%  The low-pass filter should roll off before the lowest structural mode.
fcs.struct_filter_freq = 2*pi*6;    % 6 Hz cutoff (well below 8 Hz wing mode)
fcs.struct_filter_zeta = 0.7;       % Critically damped

fprintf('  Structural Filters:\n');
fprintf('    Low-pass cutoff: %.0f Hz (wing 1st bending ~8 Hz)\n', ...
    fcs.struct_filter_freq / (2*pi));
fprintf('\n');

%% ====================================================================
%  6. INITIAL CONDITIONS
%  ====================================================================

aircraft.alt_init = 3000;     % Initial altitude (m)

% Compute trim at M=0.6, 3000m, wings-level, 1g flight
V_init    = 200;              % Initial airspeed (m/s) ~ M=0.6 at 3km
theta_init = deg2rad(2);      % Small initial pitch angle
alpha_init = deg2rad(2);      % Initial AoA ≈ theta for level flight

initial.u     = V_init * cos(alpha_init);
initial.v     = 0;
initial.w     = V_init * sin(alpha_init);
initial.p     = 0;
initial.q     = 0;
initial.r     = 0;
initial.x     = 0;            % North (m)
initial.y     = 0;            % East (m)
initial.z     = -aircraft.alt_init;  % Down (m), negative = altitude
initial.phi   = 0;
initial.theta = theta_init;
initial.psi   = 0;

initial.state = [initial.u; initial.v; initial.w;
                 initial.p; initial.q; initial.r;
                 initial.x; initial.y; initial.z;
                 initial.phi; initial.theta; initial.psi];

% Initial actuator positions (approximate trim)
initial.de_L = deg2rad(-2);   % Slight trailing-edge-up for trim
initial.de_R = deg2rad(-2);
initial.dr   = 0;
initial.dc   = deg2rad(3);    % Canard slightly deflected for trim
initial.throttle = 0.4;       % ~40% thrust for level flight

fprintf('Initial Conditions:\n');
fprintf('  Altitude:  %.0f m (%.0f ft)\n', aircraft.alt_init, aircraft.alt_init*3.281);
fprintf('  Airspeed:  %.0f m/s (Mach %.2f)\n', V_init, V_init/340);
fprintf('  Alpha:     %.1f deg\n', rad2deg(alpha_init));
fprintf('  Throttle:  %.0f%%\n\n', initial.throttle*100);

%% ====================================================================
%  7. STABILITY ANALYSIS (linearized at design point)
%  ====================================================================

fprintf('==========================================================\n');
fprintf(' CLOSED-LOOP STABILITY ANALYSIS\n');
fprintf('==========================================================\n\n');

% Under perfect NDI/INDI, each axis reduces to:
%   s^2 + K*s + Ki = 0
axes_names = {'Roll (p)', 'Pitch (q)', 'Yaw (r)'};
K_vec  = [fcs.K_p,  fcs.K_q,  fcs.K_r];
Ki_vec = [fcs.Ki_p, fcs.Ki_q, fcs.Ki_r];

for i = 1:3
    wn   = sqrt(Ki_vec(i));
    zeta = K_vec(i) / (2 * wn);
    poles = roots([1, K_vec(i), Ki_vec(i)]);

    fprintf('  %s:\n', axes_names{i});
    fprintf('    omega_n = %.2f rad/s,  zeta = %.2f\n', wn, zeta);
    fprintf('    CL poles: %.2f, %.2f\n', real(poles(1)), real(poles(2)));
    if zeta >= 0.5 && zeta <= 1.5
        fprintf('    STATUS: Well-damped\n');
    elseif zeta < 0.5
        fprintf('    WARNING: Under-damped — may ring\n');
    else
        fprintf('    NOTE: Over-damped — slower but no overshoot\n');
    end
    fprintf('\n');
end

% Check actuator bandwidth constraint
actuator_bw = 1 / aircraft.actuator.tau;  % rad/s
fprintf('  Actuator bandwidth: %.0f rad/s (%.0f Hz)\n', ...
    actuator_bw, actuator_bw/(2*pi));
fprintf('  Max inner loop BW:  %.0f rad/s (K_q)\n', fcs.K_q);
fprintf('  BW ratio (actuator/controller): %.1f\n', actuator_bw / fcs.K_q);
if actuator_bw / fcs.K_q > 3
    fprintf('  STATUS: OK — sufficient actuator bandwidth margin\n');
elseif actuator_bw / fcs.K_q > 2
    fprintf('  CAUTION: Marginal actuator bandwidth — watch phase lag\n');
else
    fprintf('  WARNING: Actuator too slow! Reduce inner loop gains.\n');
end
fprintf('\n');

% Check control authority
M_destab_max = qbar_design * aircraft.S * aircraft.c_bar * ...
               abs(aircraft.aero.Cm_table(6,3));  % at alpha=20, M=0.6
Cmde_design  = aircraft.aero.Cmde_table(3,3);     % at alpha=5, M=0.6
M_ctrl_max   = abs(qbar_design * aircraft.S * aircraft.c_bar * ...
               Cmde_design * aircraft.de_max);

fprintf('  Control Authority (pitch, M=0.6, 3km):\n');
fprintf('    Max destabilizing moment: %.0f Nm (at alpha=20 deg)\n', M_destab_max);
fprintf('    Max control moment:       %.0f Nm (full elevon)\n', M_ctrl_max);
fprintf('    Authority ratio: %.1f\n', M_ctrl_max / M_destab_max);
if M_ctrl_max / M_destab_max > 1.5
    fprintf('    STATUS: Adequate control authority\n');
else
    fprintf('    WARNING: Low authority margin!\n');
end
fprintf('\n');

%% ====================================================================
%  8. SAVE
%  ====================================================================

fprintf('==========================================================\n');
fprintf(' SETUP COMPLETE\n');
fprintf('==========================================================\n\n');
fprintf('Workspace variables:\n');
fprintf('  aircraft  — Aircraft parameters + aero database\n');
fprintf('  fcs       — Flight control system gains\n');
fprintf('  initial   — Initial conditions\n\n');
fprintf('Next: run_fighter_sim\n\n');
