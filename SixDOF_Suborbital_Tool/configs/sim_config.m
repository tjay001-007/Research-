function cfg = sim_config()
%SIM_CONFIG  Simulation configuration, switches, and flags.
%
%   All on/off switches, tolerances, objective weights, and constraint
%   limits are centralised here.  Modify this file to change simulation
%   behaviour without touching any physics code.
%
%   Returns:
%     cfg  - Struct with all simulation settings

%% ========================================================================
%  SUBSYSTEM SWITCHES  (true = enabled)
%  ========================================================================

cfg.tvc_on          = true;     % Thrust Vector Control
cfg.rcs_on          = true;     % Reaction Control System
cfg.aerospike_on    = true;     % Aerospike second engine
cfg.nav_ekf_on      = true;     % EKF navigation (false = use true state)
cfg.aero_on         = true;     % Aerodynamic forces/moments
cfg.gravity_on      = true;     % Gravitational force

%% ========================================================================
%  GUIDANCE MODE
%   'pitch_program'      : Open-loop polynomial pitch angle schedule (fast)
%   'predictor_corrector': Closed-loop correction toward target apogee
%  ========================================================================

cfg.guidance_mode    = 'pitch_program';   % Start with this; GA optimises pitch coeffs
cfg.t_vertical       = 5.0;   % Vertical ascent duration (s) before pitch-over
cfg.t_pitchover_default = 70; % Default pitch program end time (s)

%% ========================================================================
%  TRAJECTORY TARGETS
%  ========================================================================

cfg.target_apogee_km    = 100.0;   % Target apogee altitude (km, Karman line)
cfg.target_downrange_km = 80.0;    % Acceptable downrange distance (km)
cfg.launch_azimuth_deg  = 90.0;    % Launch azimuth (deg, East)

%% ========================================================================
%  SIMULATION TIME SETTINGS
%  ========================================================================

cfg.t_start     = 0.0;      % Simulation start time (s)
cfg.t_end       = 300.0;    % Maximum simulation end time (s)
cfg.dt          = 0.004;    % Integration time step (s) — 250 Hz
cfg.dt_gnc      = 0.010;    % GNC update interval (s) — 100 Hz
cfg.dt_guidance = 0.100;    % Guidance update interval (s) — 10 Hz
cfg.dt_nav      = 0.100;    % GPS measurement update interval (s) — 10 Hz

%  Stop simulation when vehicle reaches apogee (descending + below threshold)
cfg.stop_on_apogee   = true;
cfg.stop_on_impact   = true;   % Stop when z_NED > 0 after launch (ground impact)

%% ========================================================================
%  OPTIMISATION SETTINGS (Genetic Algorithm)
%  ========================================================================

cfg.run_optimizer    = false;   % Set true to run GA before simulation
cfg.ga.pop_size      = 60;      % Population size
cfg.ga.n_gen         = 100;     % Number of generations
cfg.ga.elite_n       = 5;       % Elite individuals preserved each generation
cfg.ga.p_cross       = 0.70;    % Crossover probability
cfg.ga.p_mut         = 0.15;    % Mutation probability per gene
cfg.ga.sigma_mut     = 0.10;    % Mutation step size (fraction of parameter range)
cfg.ga.tournament_k  = 3;       % Tournament selection size
cfg.ga.print_interval = 10;     % Print progress every N generations
cfg.ga.fast_eval     = true;    % Use simplified EOM for fitness eval (no nav, no EKF)

%% ========================================================================
%  OBJECTIVE WEIGHTS  (0 = disable, positive = active)
%  ========================================================================

cfg.obj.maximize_apogee_speed   = 1.0;   % Maximise speed at apogee
cfg.obj.minimize_fuel           = 0.4;   % Minimise total propellant consumed
cfg.obj.maximize_apogee_alt     = 0.8;   % Maximise peak altitude
cfg.obj.minimize_peak_qbar      = 0.2;   % Minimise peak dynamic pressure

%% ========================================================================
%  CONSTRAINTS  (true = active in optimisation penalty)
%  ========================================================================

cfg.con.max_qbar             = true;
cfg.con.qbar_limit           = 75000;   % Pa — structural dynamic pressure limit

cfg.con.min_apogee_alt       = true;
cfg.con.apogee_alt_min_km    = 80.0;    % km — must achieve at least this altitude

cfg.con.max_AoA              = true;
cfg.con.AoA_limit_deg        = 18.0;    % deg — max sustained angle of attack

cfg.con.landing_range        = false;   % off by default (free-ranging trajectory)
cfg.con.range_limit_km       = 120.0;   % km — max downrange if enabled

cfg.con.aerospike_h_range    = true;    % Constrain aerospike ignition band

cfg.penalty_weight           = 1e4;     % Penalty multiplier for constraint violations

%% ========================================================================
%  NAVIGATION SENSOR NOISE (1-sigma)
%  ========================================================================

cfg.nav.sigma_accel    = 1e-3;    % Accelerometer noise (m/s^2)
cfg.nav.sigma_gyro     = 1e-4;    % Gyroscope noise (rad/s)
cfg.nav.sigma_gps_pos  = 3.0;     % GPS position noise (m)
cfg.nav.sigma_gps_vel  = 0.1;     % GPS velocity noise (m/s)
cfg.nav.gps_rate       = 10.0;    % GPS update rate (Hz)
cfg.nav.accel_bias_0   = [0.002; -0.001; 0.003];  % Initial accel bias (m/s^2)
cfg.nav.gyro_bias_0    = [1e-4; -5e-5; 8e-5];     % Initial gyro bias (rad/s)

%% ========================================================================
%  CONTROL AUTHORITY WARNING THRESHOLD
%  ========================================================================

cfg.authority_warn_pct  = 15.0;   % Warn when authority remaining < 15%
cfg.cond_B_warn         = 100.0;  % Warn when B-matrix condition number > 100

%% ========================================================================
%  INITIAL CONDITIONS
%  ========================================================================

cfg.ic.pos_NED     = [0; 0; 0];          % Launch site (m, NED)
cfg.ic.vel_body    = [0; 0; 0];          % Initial body velocity (m/s)
cfg.ic.euler_deg   = [0; 90; 90];        % [roll, pitch, yaw] deg — vertical, heading East
%   pitch=90 deg → nose pointing straight up (theta = 90 deg)
cfg.ic.omega       = [0; 0; 0];          % Initial body rates (rad/s)
% Initial quaternion computed from Euler in main script

%% ========================================================================
%  OUTPUT / LOGGING
%  ========================================================================

cfg.log_dt       = 0.04;    % Logging decimation (s) — 25 Hz output, saves memory
cfg.plot_on      = true;    % Generate plots after simulation
cfg.verbose      = true;    % Print progress to console

fprintf('[Sim Config] Loaded. TVC=%d RCS=%d Aerospike=%d EKF=%d\n', ...
    cfg.tvc_on, cfg.rcs_on, cfg.aerospike_on, cfg.nav_ekf_on);
fprintf('  Optimizer: %s | Guidance: %s\n', ...
    mat2str(cfg.run_optimizer), cfg.guidance_mode);

end
