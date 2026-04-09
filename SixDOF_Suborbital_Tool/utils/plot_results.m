function plot_results(log, sim_cfg, history)
%PLOT_RESULTS  Comprehensive visualisation of the 6DOF simulation results.
%
%   Generates:
%     Figure 1 — 8-panel flight performance overview
%     Figure 2 — Control surface activity + TVC + RCS
%     Figure 3 — Control authority margins (with 15% warning band)
%     Figure 4 — 3D flight path (NED → ENU for display)
%     Figure 5 — GA convergence (if optimizer was run)
%
%   Inputs:
%     log      — Logging struct from simulation (fields: t, x, controls,
%                auth, prop_state, guidance, nav_err)
%     sim_cfg  — Sim config (switches, targets)
%     history  — GA history struct (optional, pass [] to skip Figure 5)
%
%   Log fields expected:
%     log.t           — Time vector (s)
%     log.x           — State matrix [N x 14]
%     log.controls    — Controls matrix [N x 8]: [de,da,dr,tp,ty,th1,th2,rcs_dc]
%     log.auth        — Authority matrix [N x 7]: [el,ai,ru,tvcp,tvcy,rcs_p,rcs_r]
%     log.qbar        — Dynamic pressure (Pa) [N x 1]
%     log.alpha_deg   — AoA (deg) [N x 1]
%     log.Mach        — Mach number [N x 1]
%     log.prop        — Propulsion state matrix [N x 3]: [T1,T2,m_prop]
%     log.euler_deg   — Euler angles (deg) [N x 3]: [phi,theta,psi]

t = log.t;
N = length(t);

%  Altitude and speed
altitude_km = -log.x(:,3) / 1000;
u = log.x(:,4); v = log.x(:,5); w = log.x(:,6);
V_ms = sqrt(u.^2 + v.^2 + w.^2);

%  Euler angles (from log)
phi_deg   = log.euler_deg(:,1);
theta_deg = log.euler_deg(:,2);
psi_deg   = log.euler_deg(:,3);

%  Rates
p_dps  = rad2deg(log.x(:,11));
qr_dps = rad2deg(log.x(:,12));
r_dps  = rad2deg(log.x(:,13));

%  Mass
mass   = log.x(:,14);

%% ========================================================================
%  FIGURE 1 — FLIGHT PERFORMANCE OVERVIEW (8 panels)
%  ========================================================================

fig1 = figure('Name', '6DOF Trajectory — Flight Performance', 'Position', [50, 50, 1400, 900]);

% 1.1 — Altitude
subplot(4, 2, 1);
plot(t, altitude_km, 'b', 'LineWidth', 1.8);
ylabel('Altitude (km)');
title('Altitude Profile');
if isfield(sim_cfg, 'target_apogee_km')
    yline(sim_cfg.target_apogee_km, 'r--', 'Target', 'LineWidth', 1.2);
end
grid on;

% 1.2 — Speed + Mach
subplot(4, 2, 2);
yyaxis left
plot(t, V_ms/1000, 'b', 'LineWidth', 1.5);
ylabel('Speed (km/s)');
yyaxis right
plot(t, log.Mach, 'r', 'LineWidth', 1.2);
ylabel('Mach');
title('Airspeed & Mach Number');
legend('Speed', 'Mach', 'Location', 'best');
grid on;

% 1.3 — Euler angles
subplot(4, 2, 3);
plot(t, phi_deg, 'b', t, theta_deg, 'r', t, psi_deg, 'g', 'LineWidth', 1.2);
ylabel('Angle (deg)');
title('Euler Angles');
legend('\phi', '\theta', '\psi', 'Location', 'best');
grid on; ylim([-200, 200]);

% 1.4 — Angular rates
subplot(4, 2, 4);
plot(t, p_dps, 'b', t, qr_dps, 'r', t, r_dps, 'g', 'LineWidth', 1.2);
ylabel('Rate (deg/s)');
title('Body Angular Rates');
legend('p (roll)', 'q (pitch)', 'r (yaw)', 'Location', 'best');
grid on;

% 1.5 — Dynamic pressure
subplot(4, 2, 5);
plot(t, log.qbar/1000, 'b', 'LineWidth', 1.5);
hold on;
if sim_cfg.con.max_qbar
    yline(sim_cfg.con.qbar_limit/1000, 'r--', 'Structural Limit', 'LineWidth', 1.2);
end
ylabel('q_{bar} (kPa)');
title('Dynamic Pressure');
xlabel('Time (s)');
grid on;

% 1.6 — Angle of attack
subplot(4, 2, 6);
plot(t, log.alpha_deg, 'b', 'LineWidth', 1.5);
hold on;
if sim_cfg.con.max_AoA
    yline(sim_cfg.con.AoA_limit_deg, 'r--', 'AoA Limit', 'LineWidth', 1.2);
    yline(-sim_cfg.con.AoA_limit_deg, 'r--', 'LineWidth', 1.2);
end
ylabel('\alpha (deg)');
title('Angle of Attack');
xlabel('Time (s)');
grid on;

% 1.7 — Thrust
subplot(4, 2, 7);
if isfield(log, 'prop')
    plot(t, log.prop(:,1)/1000, 'b', t, log.prop(:,2)/1000, 'r', 'LineWidth', 1.5);
    legend('T_1 (bell)', 'T_2 (aerospike)', 'Location', 'best');
end
ylabel('Thrust (kN)');
title('Propulsion Thrust');
xlabel('Time (s)');
grid on;

% 1.8 — Vehicle mass
subplot(4, 2, 8);
plot(t, mass, 'b', 'LineWidth', 1.5);
hold on;
yline(sim_cfg.veh_mass_dry, 'r--', 'Dry Mass', 'LineWidth', 1.2);
ylabel('Mass (kg)');
title('Vehicle Mass (Propellant Depletion)');
xlabel('Time (s)');
grid on;

sgtitle('6DOF Suborbital Trajectory — Performance Overview', 'FontSize', 14, 'FontWeight', 'bold');

%% ========================================================================
%  FIGURE 2 — CONTROL ACTIVITY
%  ========================================================================

fig2 = figure('Name', '6DOF Trajectory — Control Activity', 'Position', [100, 100, 1200, 800]);

% 2.1 — Surfaces
subplot(3, 2, 1);
de_deg_log = log.controls(:,1) * 25;  % normalised → degrees
da_deg_log = log.controls(:,2) * 20;
dr_deg_log = log.controls(:,3) * 20;
plot(t, de_deg_log, 'b', t, da_deg_log, 'r', t, dr_deg_log, 'g', 'LineWidth', 1.2);
ylabel('Deflection (deg)');
title('Control Surface Commands');
legend('Elevator', 'Aileron', 'Rudder', 'Location', 'best');
grid on; ylim([-30, 30]);

% 2.2 — TVC
if sim_cfg.tvc_on
    subplot(3, 2, 2);
    tvc_p_deg = rad2deg(log.controls(:,4));
    tvc_y_deg = rad2deg(log.controls(:,5));
    plot(t, tvc_p_deg, 'b', t, tvc_y_deg, 'r', 'LineWidth', 1.2);
    hold on;
    yline(8, 'k--', '+8°', 'LineWidth', 1);
    yline(-8, 'k--', '-8°', 'LineWidth', 1);
    ylabel('TVC Angle (deg)');
    title('Thrust Vector Control');
    legend('\delta_p (pitch)', '\delta_y (yaw)', 'Location', 'best');
    grid on; ylim([-10, 10]);
end

% 2.3 — Throttle schedule
subplot(3, 2, 3);
plot(t, log.controls(:,6), 'b', 'LineWidth', 1.5);
if sim_cfg.aerospike_on
    hold on;
    plot(t, log.controls(:,7), 'r', 'LineWidth', 1.5);
    legend('Engine 1 (Bell)', 'Engine 2 (Aerospike)', 'Location', 'best');
else
    legend('Engine 1 Throttle', 'Location', 'best');
end
ylabel('Throttle');
title('Engine Throttle Schedule');
xlabel('Time (s)');
grid on; ylim([0, 1.1]);

% 2.4 — Moment demand
subplot(3, 2, 4);
if isfield(log, 'M_demand')
    plot(t, log.M_demand(:,1)/1000, 'b', t, log.M_demand(:,2)/1000, 'r', ...
         t, log.M_demand(:,3)/1000, 'g', 'LineWidth', 1.2);
    ylabel('Moment (kNm)');
    title('Control Moment Demand');
    legend('L (roll)', 'M (pitch)', 'N (yaw)', 'Location', 'best');
    grid on;
end

% 2.5 — B-matrix condition number
subplot(3, 2, 5);
if isfield(log, 'cond_B')
    semilogy(t, log.cond_B, 'b', 'LineWidth', 1.2);
    hold on;
    yline(sim_cfg.cond_B_warn, 'r--', 'Warn Limit', 'LineWidth', 1.2);
    ylabel('Condition Number');
    title('Control Effectiveness Matrix Conditioning');
    xlabel('Time (s)');
    grid on;
end

% 2.6 — RCS duty cycle
if sim_cfg.rcs_on && isfield(log, 'rcs_duty')
    subplot(3, 2, 6);
    plot(t, log.rcs_duty(:,1)*100, 'b', t, log.rcs_duty(:,2)*100, 'r', ...
         t, log.rcs_duty(:,3)*100, 'g', 'LineWidth', 1.2);
    ylabel('Duty Cycle (%)');
    title('RCS Duty Cycle');
    xlabel('Time (s)');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    grid on; ylim([0, 110]);
end

sgtitle('Control Surface & Actuator Activity', 'FontSize', 13, 'FontWeight', 'bold');

%% ========================================================================
%  FIGURE 3 — CONTROL AUTHORITY MARGINS
%  ========================================================================

fig3 = figure('Name', '6DOF Trajectory — Control Authority', 'Position', [150, 150, 1000, 700]);

effectors = {'Elevator', 'Aileron', 'Rudder'};
col_idx   = {1, 2, 3};
colors    = {'b', 'r', 'g', 'm', 'c', [0.8 0.4 0], [0.5 0 0.5]};

if sim_cfg.tvc_on
    effectors{end+1} = 'TVC Pitch';  col_idx{end+1} = 4;
    effectors{end+1} = 'TVC Yaw';    col_idx{end+1} = 5;
end

n_eff = length(effectors);

for k = 1:n_eff
    subplot(ceil(n_eff/2), 2, k);
    ci = col_idx{k};
    if size(log.auth, 2) >= ci
        auth_val = log.auth(:, ci);
    else
        auth_val = 100 * ones(N, 1);
    end

    % Shade warning zone
    patch([t(1), t(end), t(end), t(1)], ...
          [0, 0, sim_cfg.authority_warn_pct, sim_cfg.authority_warn_pct], ...
          'r', 'FaceAlpha', 0.15, 'EdgeColor', 'none');
    hold on;
    plot(t, auth_val, colors{k}, 'LineWidth', 1.5);
    yline(sim_cfg.authority_warn_pct, 'r--', 'Warning', 'LineWidth', 1);

    ylabel('Authority (%)');
    title([effectors{k}, ' Authority']);
    ylim([0, 105]);
    grid on;
    if k >= n_eff - 1;  xlabel('Time (s)');  end
end

sgtitle('Real-Time Control Authority Margins', 'FontSize', 13, 'FontWeight', 'bold');

%% ========================================================================
%  FIGURE 4 — 3D FLIGHT PATH
%  ========================================================================

fig4 = figure('Name', '6DOF Trajectory — 3D Flight Path', 'Position', [200, 200, 700, 600]);

% Convert NED to ENU for a standard "up=Z" display
xN = log.x(:,1);
xE = log.x(:,2);
xD = log.x(:,3);

% ENU: x=East, y=North, z=Up (altitude)
plot3(xE/1000, xN/1000, -xD/1000, 'b', 'LineWidth', 2);
hold on;
plot3(xE(1)/1000, xN(1)/1000, 0, 'go', 'MarkerSize', 10, 'LineWidth', 2);   % Launch
[~, idx_apo] = max(-xD);
plot3(xE(idx_apo)/1000, xN(idx_apo)/1000, -xD(idx_apo)/1000, 'r*', 'MarkerSize', 12, 'LineWidth', 2);  % Apogee

xlabel('East (km)');
ylabel('North (km)');
zlabel('Altitude (km)');
title(sprintf('3D Flight Path  |  Apogee: %.1f km', -xD(idx_apo)/1000));
legend('Trajectory', 'Launch', 'Apogee', 'Location', 'best');
grid on; box on;
view(35, 25);

%% ========================================================================
%  FIGURE 5 — GA CONVERGENCE (if optimizer was run)
%  ========================================================================

if ~isempty(history) && sim_cfg.run_optimizer
    fig5 = figure('Name', 'GA Convergence', 'Position', [250, 250, 700, 450]);

    subplot(1,2,1);
    plot(history.best_score, 'b-', 'LineWidth', 1.8);
    hold on;
    plot(history.mean_score, 'r--', 'LineWidth', 1.2);
    xlabel('Generation');
    ylabel('Fitness Score');
    title('GA Convergence');
    legend('Best', 'Mean', 'Location', 'best');
    grid on;

    subplot(1,2,2);
    improvement = 100 * (history.best_score(1) - history.best_score) / abs(history.best_score(1) + 1e-10);
    plot(improvement, 'g-', 'LineWidth', 1.8);
    xlabel('Generation');
    ylabel('Improvement (%)');
    title('Relative Improvement from Gen 1');
    grid on;

    sgtitle('Genetic Algorithm Trajectory Optimisation', 'FontSize', 12, 'FontWeight', 'bold');
end

fprintf('[Plot] Figures generated.\n');

end
