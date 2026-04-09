%% MAIN_6DOF_SIMULATION  Entry point for the 6DOF suborbital trajectory tool.
%
%   Velos-1 Suborbital Winged Rocket — 6DOF Trajectory Analysis
%   ============================================================
%
%   ARCHITECTURE (call graph):
%
%   main_6dof_simulation
%   ├── configs/
%   │   ├── vehicle_config       → vehicle struct
%   │   ├── propulsion_config    → prop_cfg struct
%   │   ├── aero_database        → aero_db struct
%   │   └── sim_config           → sim_cfg struct (switches/flags)
%   │
%   ├── gnc/lqr_gain_schedule    → gain_table (pre-computed, ~5s)
%   │
%   ├── [optional] optimization/trajectory_optimizer  → opt_params
%   │
%   └── MAIN SIMULATION LOOP (RK4, 250 Hz)
%       ├── gnc/guidance_system      → [theta_cmd, phi_cmd, psi_cmd]
%       ├── gnc/navigation_system    → x_nav (INS/EKF)
%       ├── gnc/control_system       → controls struct
%       │   ├── propulsion/tvc_model
%       │   └── propulsion/rcs_model
%       ├── eom/eom_6dof             → xdot [14x1]  (RK4 integration)
%       │   ├── aerodynamics/aero_forces_moments
%       │   └── propulsion/propulsion_model
%       │       ├── propulsion/aerospike_engine
%       │       └── propulsion/tvc_model
%       └── gnc/control_authority    → authority margins
%
%   KEY SWITCHES (in sim_config.m):
%     sim_cfg.tvc_on         — Toggle TVC
%     sim_cfg.rcs_on         — Toggle RCS
%     sim_cfg.aerospike_on   — Toggle aerospike engine
%     sim_cfg.nav_ekf_on     — Toggle EKF navigation (false = true state)
%     sim_cfg.run_optimizer  — Run GA before simulation
%     sim_cfg.guidance_mode  — 'pitch_program' or 'predictor_corrector'
%
%   TOOLBOXES REQUIRED:
%     Aerospace Toolbox:       atmosisa, quat2dcm, angle2quat, quatmultiply
%     Control System Toolbox:  lqr (for gain schedule computation)
%
%   Author: 6DOF Suborbital Trajectory Tool
%   Ref:    Stevens & Lewis (2003), Zipfel (2007)

clear; close all; clc;

fprintf('================================================\n');
fprintf('   6DOF SUBORBITAL TRAJECTORY ANALYSIS TOOL\n');
fprintf('   Velos-1 Winged Rocket / Lifting Body\n');
fprintf('================================================\n\n');

%% ========================================================================
%  ADD PATHS
%  ========================================================================

root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'configs'));
addpath(fullfile(root, 'eom'));
addpath(fullfile(root, 'propulsion'));
addpath(fullfile(root, 'aerodynamics'));
addpath(fullfile(root, 'gnc'));
addpath(fullfile(root, 'optimization'));
addpath(fullfile(root, 'utils'));
addpath(fullfile(root, 'examples'));

%% ========================================================================
%  LOAD CONFIGURATIONS
%  ========================================================================

vehicle  = vehicle_config();
prop_cfg = propulsion_config();
aero_db  = aero_database();
sim_cfg  = sim_config();

% Pass dry mass to plot_results
sim_cfg.veh_mass_dry = vehicle.mass_dry;

%% ========================================================================
%  INITIAL CONDITIONS
%  ========================================================================

phi0   = deg2rad(sim_cfg.ic.euler_deg(1));   % Roll  (rad)
theta0 = deg2rad(sim_cfg.ic.euler_deg(2));   % Pitch (rad) — 90° = straight up
psi0   = deg2rad(sim_cfg.ic.euler_deg(3));   % Yaw   (rad)

% Quaternion from Euler angles (Aerospace Toolbox: ZYX convention)
q_init = angle2quat(psi0, theta0, phi0, 'ZYX')';   % [q0;q1;q2;q3] column

% Full 14-state initial condition
x0 = [sim_cfg.ic.pos_NED;    % [xN; xE; xD] (m)
      sim_cfg.ic.vel_body;    % [u; v; w] (m/s)
      q_init;                 % [q0;q1;q2;q3]
      sim_cfg.ic.omega;       % [p; qr; r] (rad/s)
      vehicle.mass_total];    % m (kg)

fprintf('[IC] Launch state set:\n');
fprintf('  Position: [%.0f, %.0f, %.0f] m (NED)\n', x0(1), x0(2), x0(3));
fprintf('  Velocity: [%.1f, %.1f, %.1f] m/s (body)\n', x0(4), x0(5), x0(6));
fprintf('  Euler:    [%.1f, %.1f, %.1f] deg (phi,theta,psi)\n', sim_cfg.ic.euler_deg);
fprintf('  Mass:     %.0f kg\n\n', x0(14));

%% ========================================================================
%  LQR GAIN SCHEDULE (Control System Toolbox)
%  ========================================================================

fprintf('[GNC] Computing LQR gain schedule...\n');
gain_table = lqr_gain_schedule(vehicle, aero_db, prop_cfg, sim_cfg);

%% ========================================================================
%  TRAJECTORY OPTIMISATION (optional — uses GA)
%  ========================================================================

% Default trajectory parameters (override by GA if run_optimizer=true)
traj_params.pitch_coeffs  = [12.0, -45.0, 15.0, -8.0, -30.0, 88.0];
traj_params.t_vertical    = sim_cfg.t_vertical;
traj_params.t_pitchover   = sim_cfg.t_pitchover_default;
traj_params.psi_launch    = deg2rad(sim_cfg.launch_azimuth_deg);
traj_params.target_apogee = sim_cfg.target_apogee_km * 1000;
h_ignite                  = prop_cfg.eng2.h_ignite_default;

if sim_cfg.run_optimizer
    fprintf('[Optimizer] Running GA trajectory optimisation...\n');
    [opt_params, best_score, ga_history] = trajectory_optimizer(vehicle, aero_db, prop_cfg, sim_cfg);

    % Apply optimised parameters
    traj_params.pitch_coeffs = opt_params.pitch_coeffs;
    traj_params.t_pitchover  = opt_params.t_pitchover;
    h_ignite                 = opt_params.h_ignite;
    fprintf('[Optimizer] Optimised h_ignite = %.1f km\n', h_ignite/1000);
else
    ga_history = [];
    fprintf('[Optimizer] Skipping GA (sim_cfg.run_optimizer = false)\n');
    fprintf('  Using default h_ignite = %.1f km\n\n', h_ignite/1000);
end

%% ========================================================================
%  SIMULATION SETUP
%  ========================================================================

dt      = sim_cfg.dt;        % Integration step (s)
t_end   = sim_cfg.t_end;     % Max simulation time (s)
t_vec   = sim_cfg.t_start : dt : t_end;
N_steps = length(t_vec);

% Logging decimation
log_every = max(1, round(sim_cfg.log_dt / dt));
N_log     = ceil(N_steps / log_every);

% Pre-allocate logs
log.t          = zeros(N_log, 1);
log.x          = zeros(N_log, 14);
log.controls   = zeros(N_log, 8);    % [de,da,dr,tp,ty,th1,th2,rcs_duty]
log.auth       = zeros(N_log, 7);    % [el,ai,ru,tvcp,tvcy,rcs_p,rcs_pi]
log.qbar       = zeros(N_log, 1);
log.alpha_deg  = zeros(N_log, 1);
log.Mach       = zeros(N_log, 1);
log.euler_deg  = zeros(N_log, 3);
log.prop       = zeros(N_log, 3);    % [T1, T2, mass_remaining]
log.M_demand   = zeros(N_log, 3);
log.cond_B     = zeros(N_log, 1);
log.rcs_duty   = zeros(N_log, 3);

% Initialise subsystem states
x        = x0;
nav_state = struct('initialized', false);

% GNC state
gnc_state.int_state      = zeros(3,1);   % Rate integrators [roll; pitch; yaw]
gnc_state.tvc_state      = zeros(2,1);   % [dp; dy] current gimbal angles
gnc_state.T1_current     = prop_cfg.eng1.T_sl;   % Initial thrust estimate
gnc_state.throttle1_cmd  = 1.0;
gnc_state.throttle2_cmd  = 0.85;

% Guidance state
guid_state.phase         = 1;
guid_state.t_last_update = -1e6;
guid_state.theta_pc      = deg2rad(45);

log_idx  = 1;
apogee_detected = false;

fprintf('\n[Sim] Starting 6DOF simulation (dt=%.3fs, t_end=%.0fs)...\n', dt, t_end);
fprintf('[Sim] TVC=%d  RCS=%d  Aerospike=%d  EKF=%d  Guidance=%s\n\n', ...
    sim_cfg.tvc_on, sim_cfg.rcs_on, sim_cfg.aerospike_on, sim_cfg.nav_ekf_on, ...
    sim_cfg.guidance_mode);

t_sim_start = tic;

%% ========================================================================
%  MAIN SIMULATION LOOP (RK4, 250 Hz)
%  ========================================================================

for k = 1:N_steps
    t = t_vec(k);

    %% --- GUIDANCE (at guidance rate) ---
    if mod(k-1, round(sim_cfg.dt_guidance/dt)) == 0
        [theta_cmd, phi_cmd, psi_cmd, guid_state] = guidance_system( ...
            t, x, traj_params, sim_cfg, guid_state);
        cmd.theta_cmd = theta_cmd;
        cmd.phi_cmd   = phi_cmd;
        cmd.psi_cmd   = psi_cmd;
    end

    %% --- NAVIGATION (EKF at nav rate) ---
    if mod(k-1, round(sim_cfg.dt_nav/dt)) == 0 || ~isfield(nav_state,'initialized') || ~nav_state.initialized
        [x_nav, nav_state] = navigation_system(t, x, nav_state, sim_cfg, dt);
    else
        x_nav = x;   % Between updates, propagate true state
    end

    %% --- CONTROL (at GNC rate) ---
    if mod(k-1, round(sim_cfg.dt_gnc/dt)) == 0
        [controls, gnc_state] = control_system( ...
            t, x, x_nav, cmd, vehicle, aero_db, gain_table, gnc_state, sim_cfg, dt);
    end

    %% --- PROPULSION STATE (for throttle tracking) ---
    altitude = -x(3);
    [~, a_s, Pa_s, rho_s] = atmosisa(max(0, min(86000, altitude)));
    atm_cur.rho = rho_s;  atm_cur.a = a_s;  atm_cur.Pa = Pa_s;

    [~, ~, ~, prop_state_cur] = propulsion_model(x, controls, vehicle, prop_cfg, atm_cur, sim_cfg, h_ignite);
    gnc_state.T1_current = prop_state_cur.T1;

    %% --- CONTROL AUTHORITY MONITOR ---
    auth = control_authority(controls, vehicle, sim_cfg);

    %% --- RK4 INTEGRATION ---
    k1 = eom_6dof(t,          x,              controls, vehicle, aero_db, prop_cfg, sim_cfg, h_ignite);
    k2 = eom_6dof(t+dt/2,     x+0.5*dt*k1,   controls, vehicle, aero_db, prop_cfg, sim_cfg, h_ignite);
    k3 = eom_6dof(t+dt/2,     x+0.5*dt*k2,   controls, vehicle, aero_db, prop_cfg, sim_cfg, h_ignite);
    k4 = eom_6dof(t+dt,       x+dt*k3,       controls, vehicle, aero_db, prop_cfg, sim_cfg, h_ignite);
    x  = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % Normalise quaternion after each full RK4 step
    q_n = x(7:10) / norm(x(7:10));
    x(7:10) = q_n;

    % Mass floor
    x(14) = max(x(14), vehicle.mass_dry);

    %% --- LOGGING (decimated) ---
    if mod(k-1, log_every) == 0 && log_idx <= N_log
        % Derived quantities for logging
        u_l=x(4); v_l=x(5); w_l=x(6);
        V_l  = max(sqrt(u_l^2+v_l^2+w_l^2), 1);
        Mach_l = V_l / a_s;
        qbar_l = 0.5 * rho_s * V_l^2;
        alpha_l = rad2deg(atan2(w_l, u_l));

        q0l=x(7); q1l=x(8); q2l=x(9); q3l=x(10);
        phi_l   = rad2deg(atan2(2*(q0l*q1l+q2l*q3l), 1-2*(q1l^2+q2l^2)));
        theta_l = rad2deg(asin(min(max(2*(q0l*q2l-q3l*q1l),-1),1)));
        psi_l   = rad2deg(atan2(2*(q0l*q3l+q1l*q2l), 1-2*(q2l^2+q3l^2)));

        log.t(log_idx)         = t;
        log.x(log_idx,:)       = x';
        log.controls(log_idx,:)= [controls.delta_e, controls.delta_a, controls.delta_r, ...
                                   controls.tvc_pitch, controls.tvc_yaw, ...
                                   controls.throttle1, controls.throttle2, 0];
        log.auth(log_idx,:)    = [auth.elevator_pct, auth.aileron_pct, auth.rudder_pct, ...
                                   auth.tvc_pitch_pct, auth.tvc_yaw_pct, ...
                                   auth.rcs_pitch_pct, auth.rcs_roll_pct];
        log.qbar(log_idx)      = qbar_l;
        log.alpha_deg(log_idx) = alpha_l;
        log.Mach(log_idx)      = Mach_l;
        log.euler_deg(log_idx,:) = [phi_l, theta_l, psi_l];
        log.prop(log_idx,:)    = [prop_state_cur.T1, prop_state_cur.T2, x(14)];
        if isfield(controls, 'M_demand')
            log.M_demand(log_idx,:)= controls.M_demand';
        end
        if isfield(controls, 'cond_B')
            log.cond_B(log_idx) = controls.cond_B;
        end
        if isfield(controls, 'rcs_cmd')
            rcs_max_M = 2*vehicle.rcs_thrust*[vehicle.l_rcs_roll;vehicle.l_rcs_pitch;vehicle.l_rcs_yaw];
            log.rcs_duty(log_idx,:) = min(abs(controls.rcs_cmd)./max(rcs_max_M,1), 1)';
        end

        log_idx = log_idx + 1;
    end

    %% --- STOP CONDITIONS ---

    % Apogee detection
    alt_cur = -x(3);
    if k > 100 && alt_cur < prev_alt - 50 && ~apogee_detected
        apogee_detected = true;
        fprintf('[Sim] Apogee detected at t=%.1fs: h=%.1f km, V=%.0f m/s\n', ...
            t, prev_alt/1000, sqrt(x(4)^2+x(5)^2+x(6)^2));
    end
    prev_alt = alt_cur;

    if k == 1;  prev_alt = 0;  end

    % Ground impact (after liftoff)
    if sim_cfg.stop_on_impact && t > 10 && x(3) > 100
        fprintf('[Sim] Ground impact at t=%.1fs\n', t);
        break;
    end

    % Watchdog: diverging (check q norm drift)
    if norm(x(7:10)) < 0.5 || any(isnan(x)) || any(isinf(x))
        fprintf('[Sim] WARNING: State diverged at t=%.1fs — stopping.\n', t);
        break;
    end

    % Progress print
    if mod(k, round(10/dt)) == 0 && sim_cfg.verbose
        fprintf('[Sim] t=%6.1fs  h=%6.1f km  V=%5.0f m/s  Mach=%.2f  m=%4.0f kg\n', ...
            t, -x(3)/1000, sqrt(x(4)^2+x(5)^2+x(6)^2), Mach_l, x(14));
    end
end

sim_time = toc(t_sim_start);

%% ========================================================================
%  TRIM LOG (remove unwritten rows)
%  ========================================================================

actual_rows = log_idx - 1;
fields_log  = fieldnames(log);
for fi = 1:length(fields_log)
    f = fields_log{fi};
    if size(log.(f), 1) > actual_rows
        log.(f) = log.(f)(1:actual_rows, :);
    end
end

%% ========================================================================
%  POST-SIMULATION SUMMARY
%  ========================================================================

[h_apo_m, idx_apo] = max(-log.x(:,3));
V_apo    = sqrt(log.x(idx_apo,4)^2 + log.x(idx_apo,5)^2 + log.x(idx_apo,6)^2);
m_prop_used = vehicle.mass_total - min(log.x(:,14));
downrange   = sqrt(log.x(end,1)^2 + log.x(end,2)^2) / 1000;

fprintf('\n========================================\n');
fprintf('SIMULATION COMPLETE  (Wall time: %.1fs)\n', sim_time);
fprintf('========================================\n');
fprintf('Apogee altitude:       %.2f km   (target: %.0f km)\n', h_apo_m/1000, sim_cfg.target_apogee_km);
fprintf('Speed at apogee:       %.1f m/s  (Mach ~%.2f)\n', V_apo, V_apo/330);
fprintf('Peak qbar:             %.1f kPa  (limit: %.0f kPa)\n', max(log.qbar)/1000, sim_cfg.con.qbar_limit/1000);
fprintf('Peak AoA:              %.1f deg  (limit: %.0f deg)\n', max(abs(log.alpha_deg)), sim_cfg.con.AoA_limit_deg);
fprintf('Propellant consumed:   %.0f kg  (of %.0f kg total)\n', m_prop_used, vehicle.mass_prop1+vehicle.mass_prop2);
fprintf('Downrange:             %.1f km\n', downrange);
fprintf('Aerospike ignition:    %.1f km altitude\n', h_ignite/1000);
fprintf('Logged data points:    %d\n', actual_rows);
fprintf('========================================\n\n');

%% ========================================================================
%  PLOTS
%  ========================================================================

if sim_cfg.plot_on
    plot_results(log, sim_cfg, ga_history);
end

fprintf('[Done] Run example_vehicle_data for vehicle reference data.\n');
fprintf('[Done] Run example_trajectory for pre-optimised trajectory reference.\n');
