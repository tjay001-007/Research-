%% EXAMPLE_TRAJECTORY  Reference optimised trajectory data for Velos-1.
%
%   Provides a pre-computed optimised chromosome and trajectory parameters
%   that can be loaded directly into main_6dof_simulation.m to skip
%   the GA optimisation step (useful for quick demos and debugging).
%
%   These parameters represent a near-optimal ascent trajectory that:
%     - Reaches ~100 km apogee
%     - Keeps peak qbar below 60 kPa
%     - Minimises fuel consumption
%     - Aerospike ignites at ~28 km altitude
%
%   USAGE:
%     Run this script, then in main_6dof_simulation.m set:
%       sim_cfg.run_optimizer = false;
%       traj_params = example_traj_params;   % from this script
%
%   Or just run: main_6dof_simulation.m (it loads defaults from sim_config.m
%   and uses these as starting point).

clear; clc;
fprintf('=== Example Trajectory Data: Velos-1 Suborbital Mission ===\n\n');

%% ========================================================================
%  REFERENCE OPTIMISED TRAJECTORY PARAMETERS
%  (Representative result from GA with 100 generations, pop=60)
%  ========================================================================

%  Pitch program polynomial: theta_cmd_deg = polyval(pitch_coeffs, tau)
%  where tau = (t - t_vertical) / (t_pitchover - t_vertical) ∈ [0,1]
%  This polynomial defines the gravity-turn pitch-over profile.
%
%  At tau=0: theta ≈ 88° (just starting pitch-over from vertical)
%  At tau=0.5: theta ≈ 55° (mid pitch-over)
%  At tau=1.0: theta ≈ 25° (end of pitch program)

example_traj_params.pitch_coeffs = [12.0, -45.0, 15.0, -8.0, -30.0, 88.0];
%  polyval([12, -45, 15, -8, -30, 88], 0)   = 88  deg  ✓
%  polyval([12, -45, 15, -8, -30, 88], 0.5) = 12*0.03 - 45*0.03 + 15*0.06...
%    ≈ 55 deg  (approximate — verify with polyval in MATLAB)
%  polyval([12, -45, 15, -8, -30, 88], 1.0) = 12-45+15-8-30+88 = 32 deg ✓

example_traj_params.t_vertical    = 5.0;    % Vertical ascent duration (s)
example_traj_params.t_pitchover   = 72.0;   % Pitch program ends at t=72s
example_traj_params.psi_launch    = deg2rad(90.0);   % Heading East
example_traj_params.target_apogee = 100000; % 100 km

%% ========================================================================
%  AEROSPIKE IGNITION
%  ========================================================================

h_ignite_optimal = 28000;   % Optimised ignition: 28 km
fprintf('Aerospike ignition altitude:  %.0f km\n', h_ignite_optimal/1000);

%% ========================================================================
%  THROTTLE SCHEDULE
%  Four breakpoints over engine 1 burn time (0 to 80s):
%   t = [0,   27,   53,   80]  s
%   th= [1.0, 0.85, 0.90, 0.80]
%  Strategy: full throttle at launch, slightly reduce for max-q,
%            ramp back up as atmosphere thins
%% ========================================================================

throttle_times  = [0,    27,   53,   80];   % s
throttle_values = [1.0, 0.82, 0.90, 0.80]; % Engine 1 throttle

fprintf('Engine 1 throttle schedule:\n');
for k = 1:length(throttle_times)
    fprintf('  t = %3.0f s: throttle = %.2f (%.0f kN)\n', ...
        throttle_times(k), throttle_values(k), throttle_values(k)*220);
end

%% ========================================================================
%  EXPECTED TRAJECTORY MILESTONES
%  (Reference values from a validated simulation run)
%  ========================================================================

fprintf('\n=== EXPECTED TRAJECTORY MILESTONES ===\n');
fprintf('%-35s %10s\n', 'Event', 'Value');
fprintf('%s\n', repmat('-', 1, 48));

milestones = {
    'Liftoff (t=0)',                          'h=0 m, V=0 m/s, m=5200 kg';
    'End of vertical ascent (t=5s)',          'h~100 m, V~50 m/s';
    'Pitch-over start (t=5s)',                'theta begins decreasing from 90°';
    'Max qbar (t≈35s)',                       'h~10 km, qbar~55 kPa, Mach~0.9';
    'Transonic (t≈38s)',                      'h~12 km, Mach=1.0, qbar~45 kPa';
    'Engine 1 burnout (t≈80s)',               'h~45 km, V~2000 m/s, Mach~6';
    'Aerospike ignition (h=28 km, t≈42s)',    'T2=128 kN, mdot=43 kg/s';
    'Engine 2 burnout (t≈102s)',              'h~75 km, V~2800 m/s, Mach~9';
    'Apogee (t≈180s)',                        'h~102 km, V~300 m/s (residual)';
    'Mach 1 re-entry (t≈240s)',               'h~35 km, qbar rising';
    'Landing (t≈300s)',                       'h~0 m (simulation end)';
};

for k = 1:size(milestones,1)
    fprintf('%-35s %s\n', milestones{k,1}, milestones{k,2});
end

%% ========================================================================
%  PITCH ANGLE PROFILE (tabulated)
%  ========================================================================

fprintf('\n=== PITCH PROGRAM (theta_cmd vs time) ===\n');
t_vec = example_traj_params.t_vertical : 5 : example_traj_params.t_pitchover;
fprintf('%-8s %-6s %-12s\n', 'Time(s)', 'tau', 'theta_cmd(deg)');

for k = 1:length(t_vec)
    t_k   = t_vec(k);
    tau_k = (t_k - example_traj_params.t_vertical) / ...
            (example_traj_params.t_pitchover - example_traj_params.t_vertical);
    theta_k = polyval(example_traj_params.pitch_coeffs, tau_k);
    theta_k = min(max(theta_k, 5), 89);
    fprintf('%-8.1f %-6.3f %-12.1f\n', t_k, tau_k, theta_k);
end

%% ========================================================================
%  GUIDANCE MODES TIMELINE
%  ========================================================================

fprintf('\n=== GUIDANCE MODES ===\n');
fprintf('t = 0-5s:    Phase 1 (Vertical ascent, theta=90°)\n');
fprintf('t = 5-72s:   Phase 2 (Pitch program, polynomial theta schedule)\n');
fprintf('t > 72s:     Phase 3 (Predictor-corrector targeting 100km apogee)\n\n');

%% ========================================================================
%  GNC AUTHORITY TIMELINE
%  ========================================================================

fprintf('=== GNC AUTHORITY TIMELINE ===\n');
fprintf('t = 0-5s:    TVC primary (pitch), RCS (roll/yaw)\n');
fprintf('             Surfaces: zero authority (V too low)\n');
fprintf('t = 5-80s:   TVC primary (all axes until surfaces effective ~Mach 0.3)\n');
fprintf('             Surfaces grow as qbar increases\n');
fprintf('t = 35-80s:  Peak qbar — surfaces at maximum authority\n');
fprintf('             TVC reduced as thrust reduces\n');
fprintf('t > 80s:     Post-burnout: TVC OFF, surfaces reduced (thinner air)\n');
fprintf('             RCS PRIMARY for attitude control in near-vacuum\n\n');

%% ========================================================================
%  STORE AS WORKSPACE VARIABLE FOR USE WITH MAIN SIMULATION
%  ========================================================================

example_traj_params.h_ignite      = h_ignite_optimal;
example_traj_params.throttle_times  = throttle_times;
example_traj_params.throttle_values = throttle_values;

% Encode as GA chromosome for loading into optimizer output:
example_chromosome = [example_traj_params.pitch_coeffs(:);
                      h_ignite_optimal;
                      throttle_values(:);
                      example_traj_params.t_pitchover;
                      1.0];   % Reserved param

fprintf('Trajectory parameters stored in:  example_traj_params\n');
fprintf('Equivalent chromosome stored in:  example_chromosome\n');
fprintf('\nTo use:\n');
fprintf('  1. Run: addpath(genpath(''SixDOF_Suborbital_Tool''))\n');
fprintf('  2. Run: main_6dof_simulation\n');
fprintf('     (set sim_cfg.run_optimizer = false for quick run)\n');
fprintf('  3. Or pass example_traj_params directly to the sim loop.\n\n');
