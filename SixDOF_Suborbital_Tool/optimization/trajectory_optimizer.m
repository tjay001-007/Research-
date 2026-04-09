function [opt_params, best_score, history] = trajectory_optimizer(vehicle, aero_db, prop_cfg, sim_cfg)
%TRAJECTORY_OPTIMIZER  Top-level GA trajectory optimisation wrapper.
%
%   Sets up chromosome bounds, fitness function handle, runs the GA,
%   decodes the optimal chromosome, and prints a results summary.
%
%   Chromosome layout (13 parameters):
%     [1:6]  — Pitch program polynomial coefficients (theta in deg vs tau)
%     [7]    — Aerospike ignition altitude h_ignite (m)
%     [8:11] — Throttle schedule [th1, th2, th3, th4] ∈ [0.4, 1.0]
%     [12]   — Pitch program end time t_pitchover (s)
%     [13]   — (placeholder, set to 1.0 — reserved for future extension)
%
%   Switchable objectives (via sim_cfg.obj.*):
%     maximize_apogee_speed, minimize_fuel, maximize_apogee_alt, minimize_peak_qbar
%
%   Switchable constraints (via sim_cfg.con.*):
%     max_qbar, min_apogee_alt, max_AoA, landing_range, aerospike_h_range
%
%   Inputs:
%     vehicle   — Vehicle config
%     aero_db   — Aerodynamic database
%     prop_cfg  — Propulsion config
%     sim_cfg   — Sim config (GA params, objectives, constraints)
%
%   Outputs:
%     opt_params — Decoded optimal trajectory parameters struct
%     best_score — Final best fitness value
%     history    — GA convergence history

fprintf('\n========================================\n');
fprintf('TRAJECTORY OPTIMISER — Genetic Algorithm\n');
fprintf('========================================\n');
fprintf('Population: %d  |  Generations: %d\n', sim_cfg.ga.pop_size, sim_cfg.ga.n_gen);
fprintf('Active objectives:\n');
if sim_cfg.obj.maximize_apogee_speed > 0
    fprintf('  + Maximise apogee speed    (w=%.2f)\n', sim_cfg.obj.maximize_apogee_speed);
end
if sim_cfg.obj.minimize_fuel > 0
    fprintf('  + Minimise fuel consumed   (w=%.2f)\n', sim_cfg.obj.minimize_fuel);
end
if sim_cfg.obj.maximize_apogee_alt > 0
    fprintf('  + Maximise apogee altitude (w=%.2f)\n', sim_cfg.obj.maximize_apogee_alt);
end
if sim_cfg.obj.minimize_peak_qbar > 0
    fprintf('  + Minimise peak qbar       (w=%.2f)\n', sim_cfg.obj.minimize_peak_qbar);
end
fprintf('Active constraints:\n');
if sim_cfg.con.max_qbar
    fprintf('  - Max qbar <= %.0f Pa\n', sim_cfg.con.qbar_limit);
end
if sim_cfg.con.min_apogee_alt
    fprintf('  - Min apogee >= %.0f km\n', sim_cfg.con.apogee_alt_min_km);
end
if sim_cfg.con.max_AoA
    fprintf('  - Max AoA <= %.0f deg\n', sim_cfg.con.AoA_limit_deg);
end
fprintf('========================================\n\n');

%% ========================================================================
%  CHROMOSOME BOUNDS
%  ========================================================================

%  Pitch polynomial coefficients [c0..c5]:
%    At tau=0: theta ≈ 89 deg (just after vertical)
%    At tau=1: theta ≈ 10–40 deg (end of pitch program)
%    Constrain via bounds (GA will find valid combination):
lb_pitch = [-100, -100, -100, -100, -100, 10];   % c5 (constant) >= 10 deg (min final pitch)
ub_pitch = [ 200,  200,  200,  200,  200, 89];   % c5 (constant) <= 89 deg (max initial)

lb_h_ign = prop_cfg.eng2.h_ignite_min;
ub_h_ign = prop_cfg.eng2.h_ignite_max;

lb_th    = [0.4, 0.4, 0.4, 0.4];   % Min throttle at each breakpoint
ub_th    = [1.0, 1.0, 1.0, 1.0];   % Max throttle

lb_tpit  = 30;     % Min pitch program duration (s)
ub_tpit  = 150;    % Max pitch program duration (s)

lb_res   = 1.0;    % Reserved parameter
ub_res   = 1.0;

lb = [lb_pitch, lb_h_ign, lb_th, lb_tpit, lb_res]';
ub = [ub_pitch, ub_h_ign, ub_th, ub_tpit, ub_res]';

%% ========================================================================
%  FITNESS FUNCTION HANDLE
%  ========================================================================

fitness_fn = @(chrom) fitness_function(chrom, vehicle, aero_db, prop_cfg, sim_cfg);

%% ========================================================================
%  RUN GA
%  ========================================================================

[best_chrom, best_score, history] = ga_engine(fitness_fn, lb, ub, sim_cfg.ga);

%% ========================================================================
%  DECODE OPTIMAL CHROMOSOME
%  ========================================================================

opt_params.pitch_coeffs = best_chrom(1:6);
opt_params.h_ignite     = best_chrom(7);
opt_params.throttle_pts = best_chrom(8:11);
opt_params.t_pitchover  = best_chrom(12);
opt_params.t_vertical   = sim_cfg.t_vertical;
opt_params.psi_launch   = deg2rad(sim_cfg.launch_azimuth_deg);
opt_params.target_apogee = sim_cfg.target_apogee_km * 1000;

% Re-evaluate best to get full metrics
[~, best_metrics] = fitness_function(best_chrom, vehicle, aero_db, prop_cfg, sim_cfg);

%% ========================================================================
%  RESULTS SUMMARY
%  ========================================================================

fprintf('\n========================================\n');
fprintf('OPTIMISATION RESULTS\n');
fprintf('========================================\n');
fprintf('Best score:            %.4f\n', best_score);
fprintf('Apogee altitude:       %.1f km\n', best_metrics.h_apogee / 1000);
fprintf('Apogee speed:          %.1f m/s (Mach ~%.2f)\n', best_metrics.V_apogee, best_metrics.V_apogee/330);
fprintf('Propellant used:       %.0f kg (%.1f%% of total)\n', ...
    best_metrics.m_prop_used, 100*best_metrics.m_prop_used/(vehicle.mass_prop1+vehicle.mass_prop2));
fprintf('Peak dynamic pressure: %.0f Pa (%.1f%% of limit)\n', ...
    best_metrics.qbar_peak, 100*best_metrics.qbar_peak/sim_cfg.con.qbar_limit);
fprintf('Peak AoA:              %.1f deg\n', best_metrics.alpha_peak_deg);
fprintf('Aerospike ignition:    %.1f km altitude\n', opt_params.h_ignite/1000);
fprintf('Pitch program end:     %.1f s\n', opt_params.t_pitchover);
fprintf('Downrange:             %.1f km\n', best_metrics.downrange_km);

if best_metrics.violations.total > 0
    fprintf('\nConstraint violations:\n');
    if best_metrics.violations.qbar > 0
        fprintf('  qbar excess: %.0f Pa\n', best_metrics.violations.qbar);
    end
    if best_metrics.violations.apogee_altitude > 0
        fprintf('  Altitude shortfall: %.1f km\n', best_metrics.violations.apogee_altitude/1000);
    end
    if best_metrics.violations.AoA > 0
        fprintf('  AoA excess: %.1f deg\n', best_metrics.violations.AoA);
    end
else
    fprintf('\nAll constraints satisfied.\n');
end
fprintf('========================================\n\n');

end
