%% Post-Simulation Plotting Script
%  Run this after simulating fighter_ndi_sim to generate analysis plots.
%
%  Reads from workspace variables logged by To Workspace blocks:
%    log_debug      [N x 8]  Nz_cmd, Nz_act, alpha, beta, Mach, V, p_cmd, q_cmd
%    log_euler      [N x 3]  phi, theta, psi (rad)
%    log_surfaces   [N x 4]  de_L, de_R, dr, dc actual (rad)
%    log_position   [N x 3]  x, y, z NED (m)
%    log_airdata    [N x 6]  alpha, beta, Mach, V, qbar, alt
%    tout           [N x 1]  time vector

fprintf('Generating post-simulation analysis plots...\n\n');

if ~exist('tout', 'var')
    error('No simulation data found. Run the Simulink model first.');
end

t = tout;

%% Figure 1: Primary Flight Display
figure('Name', 'Flight Parameters', 'Position', [50 50 1100 800], 'Color', 'w');

subplot(3,2,1);
plot(t, log_debug(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(t, log_debug(:,1), 'r--', 'LineWidth', 1.2);
yline(fcs.Nz_max, 'k:', 'LineWidth', 0.8);
yline(fcs.Nz_min, 'k:', 'LineWidth', 0.8);
ylabel('N_z (g)'); title('Load Factor (g-command tracking)');
legend('Actual', 'Commanded', 'Structural limit');
grid on;

subplot(3,2,2);
plot(t, log_debug(:,3), 'b', 'LineWidth', 1.5); hold on;
yline(fcs.alpha_max_deg, 'r--', 'Alpha limit');
ylabel('Alpha (deg)'); title('Angle of Attack');
grid on;

subplot(3,2,3);
plot(t, rad2deg(log_euler(:,1)), 'b', 'LineWidth', 1.5);
ylabel('Phi (deg)'); title('Roll Angle');
grid on;

subplot(3,2,4);
plot(t, rad2deg(log_euler(:,2)), 'b', 'LineWidth', 1.5);
ylabel('Theta (deg)'); title('Pitch Angle');
grid on;

subplot(3,2,5);
plot(t, log_debug(:,5), 'b', 'LineWidth', 1.5);
ylabel('Mach'); xlabel('Time (s)'); title('Mach Number');
grid on;

subplot(3,2,6);
plot(t, -log_position(:,3), 'b', 'LineWidth', 1.5);
ylabel('Altitude (m)'); xlabel('Time (s)'); title('Altitude');
grid on;

sgtitle('Fighter NDI — Primary Flight Parameters', 'FontSize', 14);

%% Figure 2: Control Surfaces
figure('Name', 'Control Surfaces', 'Position', [100 100 1000 600], 'Color', 'w');

labels = {'Left Elevon', 'Right Elevon', 'Rudder', 'Canard'};
limits_deg = [rad2deg(aircraft.de_max), rad2deg(aircraft.de_max), ...
              rad2deg(aircraft.dr_max), rad2deg(aircraft.dc_max)];

for i = 1:4
    subplot(2,2,i);
    plot(t, rad2deg(log_surfaces(:,i)), 'b', 'LineWidth', 1.2);
    hold on;
    yline(limits_deg(i), 'r--', 'LineWidth', 0.8);
    yline(-limits_deg(i), 'r--', 'LineWidth', 0.8);
    ylabel('Deflection (deg)');
    if i > 2, xlabel('Time (s)'); end
    title(labels{i});
    grid on;
end

sgtitle('Control Surface Deflections (after actuator)', 'FontSize', 14);

%% Figure 3: 3D Trajectory
figure('Name', '3D Trajectory', 'Position', [150 150 700 600], 'Color', 'w');
plot3(log_position(:,1)/1000, log_position(:,2)/1000, ...
    -log_position(:,3)/1000, 'b', 'LineWidth', 1.5);
hold on;
plot3(log_position(1,1)/1000, log_position(1,2)/1000, ...
    -log_position(1,3)/1000, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot3(log_position(end,1)/1000, log_position(end,2)/1000, ...
    -log_position(end,3)/1000, 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('North (km)'); ylabel('East (km)'); zlabel('Altitude (km)');
title('3D Flight Path'); grid on; view(-35, 25);
legend('Path', 'Start', 'End');

%% Figure 4: Sideslip and coordination
figure('Name', 'Coordination', 'Position', [200 200 700 400], 'Color', 'w');

subplot(1,2,1);
plot(t, log_debug(:,4), 'b', 'LineWidth', 1.5);
hold on;
yline(fcs.beta_max_deg, 'r--'); yline(-fcs.beta_max_deg, 'r--');
ylabel('Beta (deg)'); xlabel('Time (s)');
title('Sideslip (coordinated flight check)');
grid on;

subplot(1,2,2);
plot(t, log_debug(:,6), 'b', 'LineWidth', 1.5);
ylabel('V (m/s)'); xlabel('Time (s)');
title('True Airspeed');
grid on;

%% Print summary
fprintf('=== SIMULATION SUMMARY ===\n');
fprintf('  Duration:     %.1f s\n', t(end));
fprintf('  Max Nz:       %.1f g (limit: %.1f)\n', max(log_debug(:,2)), fcs.Nz_max);
fprintf('  Min Nz:       %.1f g (limit: %.1f)\n', min(log_debug(:,2)), fcs.Nz_min);
fprintf('  Max alpha:    %.1f deg (limit: %.0f)\n', max(log_debug(:,3)), fcs.alpha_max_deg);
fprintf('  Max |beta|:   %.1f deg (limit: %.0f)\n', max(abs(log_debug(:,4))), fcs.beta_max_deg);
fprintf('  Mach range:   %.2f - %.2f\n', min(log_debug(:,5)), max(log_debug(:,5)));
fprintf('  Alt range:    %.0f - %.0f m\n', min(-log_position(:,3)), max(-log_position(:,3)));
fprintf('\n');
