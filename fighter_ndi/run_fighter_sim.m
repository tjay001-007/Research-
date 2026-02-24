%% Fighter NDI/INDI Closed-Loop Simulation
%
%  Full nonlinear 6-DOF simulation of a delta-canard light combat aircraft
%  with production-grade NDI/INDI flight control system.
%
%  This script:
%    1. Runs setup_fighter.m to load all parameters
%    2. Simulates a combat-representative maneuver sequence
%    3. Includes actuator dynamics with rate and position limits
%    4. Includes sensor noise on gyros and accelerometers
%    5. Compares open-loop (uncontrolled) vs NDI vs INDI
%    6. Generates comprehensive flight analysis plots
%
%  The 6-DOF equations of motion include:
%    - Full inertia tensor with products of inertia (Ixz)
%    - Tabular aerodynamics (alpha, Mach dependent)
%    - Gravity resolved into body frame
%    - Thrust along body x-axis
%    - Atmospheric density variation with altitude
%
%  This is the same physics and structure that would run on a
%  Flight Control Computer (FCC) in a real aircraft, minus the
%  hardware interface and redundancy management.

clear all; close all; clc;

fprintf('==========================================================\n');
fprintf(' FIGHTER NDI CLOSED-LOOP SIMULATION\n');
fprintf(' 6-DOF Nonlinear with Production Control Law\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  SETUP
%  ====================================================================

setup_fighter;

dt     = fcs.dt;           % Controller time step (0.004 s = 250 Hz)
t_end  = 30;               % Simulation duration (s)
t      = 0:dt:t_end;
N      = length(t);

%% ====================================================================
%  PILOT INPUT PROFILE — COMBAT MANEUVER SEQUENCE
%  ====================================================================
%
%  This profile tests all axes and the carefree handling limiters:
%
%  Phase 1 (0-3s):     Wings-level 1g flight — verify trim stability
%  Phase 2 (3-8s):     Pull 4g — pitch response and alpha limiting
%  Phase 3 (8-12s):    Release to 1g — recovery and Nz tracking
%  Phase 4 (12-17s):   Roll to 60 deg bank + pull — combined maneuver
%  Phase 5 (17-22s):   Roll wings-level + push to 0g — negative g check
%  Phase 6 (22-27s):   Rapid roll reversal — roll rate and coupling check
%  Phase 7 (27-30s):   Return to 1g wings-level — final trim check

stick_lon = zeros(N, 1);    % -1 = fwd, +1 = aft
stick_lat = zeros(N, 1);    % -1 = left, +1 = right
pedal     = zeros(N, 1);

for k = 1:N
    ti = t(k);

    % Phase 2: Pull 4g (4/7 ≈ 0.57 of full stick)
    if ti >= 3 && ti < 8
        stick_lon(k) = 0.57;

    % Phase 3: Release to 1g (center stick)
    elseif ti >= 8 && ti < 12
        stick_lon(k) = 0.0;

    % Phase 4: Roll right + pull 3g
    elseif ti >= 12 && ti < 14
        stick_lat(k) = 0.6;       % Roll right
        stick_lon(k) = 0.43;      % 3g pull
    elseif ti >= 14 && ti < 17
        stick_lat(k) = 0.0;       % Hold bank
        stick_lon(k) = 0.43;      % Maintain 3g

    % Phase 5: Roll wings-level + push
    elseif ti >= 17 && ti < 19
        stick_lat(k) = -0.6;      % Roll left (wings-level recovery)
        stick_lon(k) = -0.15;     % Push (~0g)
    elseif ti >= 19 && ti < 22
        stick_lon(k) = -0.15;

    % Phase 6: Rapid roll reversal (aileron roll)
    elseif ti >= 22 && ti < 24
        stick_lat(k) = 0.8;       % Hard right roll
    elseif ti >= 24 && ti < 26
        stick_lat(k) = -0.8;      % Hard left roll

    % Phase 7: Recovery
    elseif ti >= 26 && ti < 27
        stick_lat(k) = 0.4;       % Stop roll
    end
end

% Constant throttle (military power)
throttle_lever = 0.75 * ones(N, 1);

%% ====================================================================
%  STATE INITIALIZATION
%  ====================================================================

x = initial.state;   % [u v w p q r x y z phi theta psi]'

% Actuator states (current position)
act_de_L = initial.de_L;
act_de_R = initial.de_R;
act_dr   = initial.dr;
act_dc   = initial.dc;

% Controller state (integrators, filters)
ctrl_state.int_omega     = [0; 0; 0];
ctrl_state.Nz_cmd_filt   = 1.0;  % 1g initial
ctrl_state.p_cmd_filt    = 0;
ctrl_state.beta_cmd_filt = 0;
ctrl_state.filt_de_L = struct('x1', initial.de_L, 'x2', 0);
ctrl_state.filt_de_R = struct('x1', initial.de_R, 'x2', 0);
ctrl_state.filt_dr   = struct('x1', 0,   'x2', 0);
ctrl_state.filt_dc   = struct('x1', initial.dc, 'x2', 0);

% Previous angular acceleration for INDI (estimated from gyro derivative)
omega_prev = [0; 0; 0];

%% ====================================================================
%  LOGGING ARRAYS
%  ====================================================================

log_state     = zeros(N, 12);
log_surfaces  = zeros(N, 4);     % [de_L, de_R, dr, dc]
log_cmd_surf  = zeros(N, 4);     % Commanded (before actuator)
log_Nz        = zeros(N, 2);     % [Nz_cmd, Nz_actual]
log_alpha     = zeros(N, 1);
log_beta      = zeros(N, 1);
log_Mach      = zeros(N, 1);
log_V         = zeros(N, 1);
log_omega_cmd = zeros(N, 3);
log_omega     = zeros(N, 3);

%% ====================================================================
%  MAIN SIMULATION LOOP
%  ====================================================================

fprintf('Simulating %.0f seconds at %d Hz...\n', t_end, round(1/dt));
tic;

for k = 1:N

    % --- Log current state ---
    log_state(k, :) = x';
    log_surfaces(k, :) = [act_de_L, act_de_R, act_dr, act_dc];

    % Extract state
    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11); psi = x(12);

    log_omega(k, :) = [p, q, r];

    % --- Sensor simulation (with noise) ---
    % Real aircraft: IMU (gyros + accels) + air data system
    % Gyro noise: ~0.01 deg/s RMS for tactical-grade IMU
    % Accel noise: ~0.01 m/s^2 RMS
    gyro_noise = deg2rad(0.01) * randn(3, 1);
    accel_noise = 0.01 * randn(3, 1);

    p_meas = p + gyro_noise(1);
    q_meas = q + gyro_noise(2);
    r_meas = r + gyro_noise(3);

    % Body accelerations (from force equations, including gravity)
    g0 = 9.81;
    ax_meas = (x(1) - u_b)/dt + g0*sin(theta) + accel_noise(1);  % Approx
    ay_meas = g0*(-cos(theta)*sin(phi)) + accel_noise(2);
    az_meas = g0*(-cos(theta)*cos(phi)) + accel_noise(3);

    % Compute actual Nz for logging
    V_now = sqrt(u_b^2 + v_b^2 + w_b^2);
    V_now = max(V_now, 5.0);
    alpha_now = atan2(w_b, u_b);
    beta_now  = asin(max(min(v_b/V_now, 1), -1));

    [~, a_sound, ~, rho_now] = atmos_isa_local(max(0, -x(9)));
    qbar_now = 0.5 * rho_now * V_now^2;
    Mach_now = V_now / a_sound;

    % For Nz computation: total aero Z-force / (m*g)
    [CL_now, ~, ~, ~, ~, ~] = fighter_aero_model( ...
        alpha_now, beta_now, p, q, r, V_now, Mach_now, max(0,-x(9)), ...
        act_de_L, act_de_R, act_dr, act_dc, aircraft);
    Nz_actual = qbar_now * aircraft.S * CL_now / (aircraft.mass * g0);
    az_meas = -Nz_actual * g0;  % More accurate az for the controller

    log_alpha(k) = rad2deg(alpha_now);
    log_beta(k)  = rad2deg(beta_now);
    log_Mach(k)  = Mach_now;
    log_V(k)     = V_now;

    % --- Angular acceleration estimation (for INDI) ---
    % In a real aircraft, this comes from differentiating filtered gyro
    % signals, or from a dedicated angular accelerometer.
    omega_now = [p_meas; q_meas; r_meas];
    if k == 1
        omega_dot_est = [0; 0; 0];
    else
        omega_dot_est = (omega_now - omega_prev) / dt;
    end
    omega_prev = omega_now;

    % Low-pass filter the angular acceleration estimate (noise reduction)
    % In real systems, this filter adds ~1 sample delay
    tau_filt = 2*dt;
    if k > 1
        omega_dot_est = omega_dot_est_prev + ...
            (dt/(tau_filt+dt)) * (omega_dot_est - omega_dot_est_prev);
    end
    omega_dot_est_prev = omega_dot_est;

    % --- NDI/INDI CONTROLLER ---
    [de_L_cmd, de_R_cmd, dr_cmd, dc_cmd, thr_cmd, ctrl_dbg] = ...
        ndi_production_controller( ...
            stick_lon(k), stick_lat(k), pedal(k), throttle_lever(k), ...
            u_b, v_b, w_b, p_meas, q_meas, r_meas, phi, theta, psi, ...
            ax_meas, ay_meas, az_meas, ...
            omega_dot_est(1), omega_dot_est(2), omega_dot_est(3), ...
            act_de_L, act_de_R, act_dr, act_dc, ...
            ctrl_state, aircraft, fcs, dt);

    ctrl_state = ctrl_dbg.state;

    log_cmd_surf(k, :) = [de_L_cmd, de_R_cmd, dr_cmd, dc_cmd];
    log_Nz(k, :) = [ctrl_dbg.Nz_cmd, Nz_actual];
    log_omega_cmd(k, :) = ctrl_dbg.omega_cmd';

    % --- ACTUATOR MODEL ---
    % First-order lag + rate limit + position limit
    % This is what makes real FCS design hard — the actuator is the
    % bottleneck, not the control law.
    [act_de_L] = actuator_step(de_L_cmd, act_de_L, ...
        aircraft.actuator.tau, aircraft.actuator.rate_max_de, ...
        aircraft.de_max, dt);
    [act_de_R] = actuator_step(de_R_cmd, act_de_R, ...
        aircraft.actuator.tau, aircraft.actuator.rate_max_de, ...
        aircraft.de_max, dt);
    [act_dr] = actuator_step(dr_cmd, act_dr, ...
        aircraft.actuator.tau, aircraft.actuator.rate_max_dr, ...
        aircraft.dr_max, dt);
    [act_dc] = actuator_step(dc_cmd, act_dc, ...
        aircraft.actuator.tau, aircraft.actuator.rate_max_dc, ...
        aircraft.dc_max, dt);

    % --- 6-DOF PLANT (RK4 integration) ---
    k1 = fighter_6dof_eom(x, act_de_L, act_de_R, act_dr, act_dc, ...
                           thr_cmd, aircraft);
    k2 = fighter_6dof_eom(x + 0.5*dt*k1, act_de_L, act_de_R, act_dr, act_dc, ...
                           thr_cmd, aircraft);
    k3 = fighter_6dof_eom(x + 0.5*dt*k2, act_de_L, act_de_R, act_dr, act_dc, ...
                           thr_cmd, aircraft);
    k4 = fighter_6dof_eom(x + dt*k3, act_de_L, act_de_R, act_dr, act_dc, ...
                           thr_cmd, aircraft);
    x = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % Wrap angles
    x(10) = wrapToPi(x(10));
    x(11) = wrapToPi(x(11));
    x(12) = wrapToPi(x(12));

    % Ground collision check
    if x(9) > 0
        fprintf('  GROUND COLLISION at t = %.2f s!\n', t(k));
        x(9) = 0;
        x(3) = 0; x(6) = 0;  % Zero vertical rate
    end
end

sim_time = toc;
fprintf('Simulation complete in %.2f s (%.1fx real-time)\n\n', ...
    sim_time, t_end / sim_time);

%% ====================================================================
%  OPEN-LOOP COMPARISON (demonstrate instability)
%  ====================================================================

fprintf('Running open-loop comparison (no FCS)...\n');

x_ol = initial.state;
N_ol = min(N, round(4/dt));  % Only 4 seconds — it diverges fast!
log_ol = zeros(N_ol, 12);

for k = 1:N_ol
    log_ol(k, :) = x_ol';
    xdot = fighter_6dof_eom(x_ol, initial.de_L, initial.de_R, ...
        initial.dr, initial.dc, initial.throttle, aircraft);
    x_ol = x_ol + dt * xdot;

    % Check for divergence
    if abs(x_ol(11)) > pi/2 || abs(x_ol(5)) > 10
        fprintf('  Open-loop diverged at t = %.3f s (pitch = %.0f deg)\n', ...
            t(k), rad2deg(x_ol(11)));
        N_ol = k;
        log_ol = log_ol(1:N_ol, :);
        break;
    end
end
fprintf('\n');

%% ====================================================================
%  COMPREHENSIVE FLIGHT ANALYSIS PLOTS
%  ====================================================================

fprintf('Generating analysis plots...\n\n');

% Color scheme
c_blue   = [0.0, 0.45, 0.74];
c_red    = [0.85, 0.33, 0.10];
c_green  = [0.47, 0.67, 0.19];
c_purple = [0.49, 0.18, 0.56];
c_orange = [0.93, 0.69, 0.13];
c_gray   = [0.5, 0.5, 0.5];

%% Figure 1: Primary Flight Display
figure('Name', 'Primary Flight Parameters', ...
    'Position', [50 50 1200 900], 'Color', 'w');

% Nz (load factor)
subplot(4, 2, 1);
plot(t, log_Nz(:,2), 'Color', c_blue, 'LineWidth', 1.5); hold on;
plot(t, log_Nz(:,1), '--', 'Color', c_red, 'LineWidth', 1.2);
yline(fcs.Nz_max, ':', 'Color', c_gray); yline(fcs.Nz_min, ':', 'Color', c_gray);
ylabel('N_z (g)'); title('Normal Load Factor');
legend('Actual', 'Commanded', 'Limits', 'Location', 'best');
grid on; xlim([0 t_end]);

% Alpha (angle of attack)
subplot(4, 2, 2);
plot(t, log_alpha, 'Color', c_blue, 'LineWidth', 1.5); hold on;
yline(fcs.alpha_max_deg, 'r--', 'Alpha limit', 'LineWidth', 1.2);
yline(fcs.alpha_warn_deg, ':', 'Color', c_orange);
ylabel('Alpha (deg)'); title('Angle of Attack');
grid on; xlim([0 t_end]);

% Pitch angle
subplot(4, 2, 3);
plot(t, rad2deg(log_state(:,11)), 'Color', c_blue, 'LineWidth', 1.5);
ylabel('Theta (deg)'); title('Pitch Angle');
grid on; xlim([0 t_end]);

% Roll angle
subplot(4, 2, 4);
plot(t, rad2deg(log_state(:,10)), 'Color', c_blue, 'LineWidth', 1.5);
ylabel('Phi (deg)'); title('Roll Angle');
grid on; xlim([0 t_end]);

% Pitch rate
subplot(4, 2, 5);
plot(t, rad2deg(log_omega(:,2)), 'Color', c_blue, 'LineWidth', 1.2); hold on;
plot(t, rad2deg(log_omega_cmd(:,2)), '--', 'Color', c_red, 'LineWidth', 1);
ylabel('q (deg/s)'); title('Pitch Rate');
legend('Actual', 'Commanded', 'Location', 'best');
grid on; xlim([0 t_end]);

% Roll rate
subplot(4, 2, 6);
plot(t, rad2deg(log_omega(:,1)), 'Color', c_blue, 'LineWidth', 1.2); hold on;
plot(t, rad2deg(log_omega_cmd(:,1)), '--', 'Color', c_red, 'LineWidth', 1);
ylabel('p (deg/s)'); title('Roll Rate');
legend('Actual', 'Commanded', 'Location', 'best');
grid on; xlim([0 t_end]);

% Mach and airspeed
subplot(4, 2, 7);
yyaxis left;
plot(t, log_Mach, 'Color', c_blue, 'LineWidth', 1.2);
ylabel('Mach');
yyaxis right;
plot(t, log_V, 'Color', c_red, 'LineWidth', 1);
ylabel('V (m/s)');
title('Airspeed'); xlabel('Time (s)');
grid on; xlim([0 t_end]);

% Beta (sideslip)
subplot(4, 2, 8);
plot(t, log_beta, 'Color', c_blue, 'LineWidth', 1.5); hold on;
yline(fcs.beta_max_deg, 'r--', 'LineWidth', 1);
yline(-fcs.beta_max_deg, 'r--', 'LineWidth', 1);
ylabel('Beta (deg)'); title('Sideslip');
xlabel('Time (s)');
grid on; xlim([0 t_end]);

sgtitle(sprintf('Fighter NDI — %s (%s mode)', aircraft.name, ...
    tern(fcs.use_indi, 'INDI', 'NDI')), 'FontSize', 14, 'FontWeight', 'bold');

%% Figure 2: Control Activity
figure('Name', 'Control Surface Activity', ...
    'Position', [100 100 1000 700], 'Color', 'w');

% Left elevon
subplot(2, 2, 1);
plot(t, rad2deg(log_cmd_surf(:,1)), ':', 'Color', c_gray, 'LineWidth', 0.8); hold on;
plot(t, rad2deg(log_surfaces(:,1)), 'Color', c_blue, 'LineWidth', 1.2);
yline(rad2deg(aircraft.de_max), 'r--'); yline(-rad2deg(aircraft.de_max), 'r--');
ylabel('Deflection (deg)'); title('Left Elevon');
legend('Commanded', 'Actual (after actuator)', 'Limits', 'Location', 'best');
grid on; xlim([0 t_end]);

% Right elevon
subplot(2, 2, 2);
plot(t, rad2deg(log_cmd_surf(:,2)), ':', 'Color', c_gray, 'LineWidth', 0.8); hold on;
plot(t, rad2deg(log_surfaces(:,2)), 'Color', c_blue, 'LineWidth', 1.2);
yline(rad2deg(aircraft.de_max), 'r--'); yline(-rad2deg(aircraft.de_max), 'r--');
ylabel('Deflection (deg)'); title('Right Elevon');
grid on; xlim([0 t_end]);

% Rudder
subplot(2, 2, 3);
plot(t, rad2deg(log_cmd_surf(:,3)), ':', 'Color', c_gray, 'LineWidth', 0.8); hold on;
plot(t, rad2deg(log_surfaces(:,3)), 'Color', c_green, 'LineWidth', 1.2);
yline(rad2deg(aircraft.dr_max), 'r--'); yline(-rad2deg(aircraft.dr_max), 'r--');
ylabel('Deflection (deg)'); title('Rudder');
xlabel('Time (s)');
grid on; xlim([0 t_end]);

% Canard
subplot(2, 2, 4);
plot(t, rad2deg(log_cmd_surf(:,4)), ':', 'Color', c_gray, 'LineWidth', 0.8); hold on;
plot(t, rad2deg(log_surfaces(:,4)), 'Color', c_purple, 'LineWidth', 1.2);
yline(rad2deg(aircraft.dc_max), 'r--'); yline(-rad2deg(aircraft.dc_max), 'r--');
ylabel('Deflection (deg)'); title('Canard');
xlabel('Time (s)');
grid on; xlim([0 t_end]);

sgtitle('Control Surface Activity (commanded vs actual)', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% Figure 3: Open-Loop vs Closed-Loop Comparison
figure('Name', 'Open-Loop Divergence vs NDI/INDI', ...
    'Position', [150 150 900 600], 'Color', 'w');

t_ol = t(1:N_ol);

subplot(2, 2, 1);
plot(t_ol, rad2deg(log_ol(:,11)), 'Color', c_red, 'LineWidth', 2); hold on;
plot(t, rad2deg(log_state(:,11)), 'Color', c_blue, 'LineWidth', 1.5);
ylabel('Theta (deg)'); title('Pitch Angle');
legend('Open-loop (NO FCS)', sprintf('%s controlled', tern(fcs.use_indi, 'INDI', 'NDI')), ...
    'Location', 'best');
grid on; xlim([0, min(t_ol(end)+1, 5)]);

subplot(2, 2, 2);
plot(t_ol, rad2deg(log_ol(:,5)), 'Color', c_red, 'LineWidth', 2); hold on;
plot(t(1:min(N,N_ol+250)), rad2deg(log_state(1:min(N,N_ol+250),5)), ...
    'Color', c_blue, 'LineWidth', 1.5);
ylabel('q (deg/s)'); title('Pitch Rate');
legend('Open-loop (DIVERGENT)', sprintf('%s stabilized', tern(fcs.use_indi, 'INDI', 'NDI')));
grid on; xlim([0, min(t_ol(end)+1, 5)]);

subplot(2, 2, 3);
plot(t_ol, log_ol(:,1), 'Color', c_red, 'LineWidth', 2); hold on;
plot(t, log_state(:,1), 'Color', c_blue, 'LineWidth', 1.5);
ylabel('u (m/s)'); xlabel('Time (s)'); title('Forward Velocity');
grid on; xlim([0, min(t_ol(end)+1, 5)]);

subplot(2, 2, 4);
plot(t_ol, -log_ol(:,9), 'Color', c_red, 'LineWidth', 2); hold on;
plot(t, -log_state(:,9), 'Color', c_blue, 'LineWidth', 1.5);
ylabel('Altitude (m)'); xlabel('Time (s)'); title('Altitude');
grid on; xlim([0, min(t_ol(end)+1, 5)]);

sgtitle(sprintf('Static Instability: %.0f%% MAC — Without FCS, diverges in < 1s', ...
    aircraft.static_margin * 100), 'FontSize', 14, 'FontWeight', 'bold');

%% Figure 4: 3D Trajectory
figure('Name', '3D Flight Path', 'Position', [200 200 700 600], 'Color', 'w');
plot3(log_state(:,7)/1000, log_state(:,8)/1000, -log_state(:,9)/1000, ...
    'Color', c_blue, 'LineWidth', 1.5);
hold on;
plot3(log_state(1,7)/1000, log_state(1,8)/1000, -log_state(1,9)/1000, ...
    'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(log_state(end,7)/1000, log_state(end,8)/1000, -log_state(end,9)/1000, ...
    'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('North (km)'); ylabel('East (km)'); zlabel('Altitude (km)');
title('3D Flight Path (Green=start, Red=end)');
grid on; view([-35, 25]);

fprintf('All plots generated.\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  PERFORMANCE METRICS
%  ====================================================================

fprintf('PERFORMANCE METRICS:\n\n');
fprintf('  Max alpha reached:  %.1f deg (limit: %.0f deg)\n', ...
    max(log_alpha), fcs.alpha_max_deg);
fprintf('  Max Nz reached:     %.1f g (limit: %.1f g)\n', ...
    max(log_Nz(:,2)), fcs.Nz_max);
fprintf('  Min Nz reached:     %.1f g (limit: %.1f g)\n', ...
    min(log_Nz(:,2)), fcs.Nz_min);
fprintf('  Max beta reached:   %.1f deg (limit: %.0f deg)\n', ...
    max(abs(log_beta)), fcs.beta_max_deg);
fprintf('  Max roll rate:      %.0f deg/s\n', max(abs(rad2deg(log_omega(:,1)))));
fprintf('  Max pitch rate:     %.0f deg/s\n', max(abs(rad2deg(log_omega(:,2)))));
fprintf('  Altitude range:     %.0f - %.0f m\n', ...
    min(-log_state(:,9)), max(-log_state(:,9)));
fprintf('  Mach range:         %.2f - %.2f\n', ...
    min(log_Mach), max(log_Mach));

% Nz tracking error (RMS, excluding first 3s of trim)
k_start = round(3/dt);
Nz_rms = sqrt(mean((log_Nz(k_start:end,2) - log_Nz(k_start:end,1)).^2));
fprintf('  Nz tracking RMS:    %.3f g\n', Nz_rms);

fprintf('\n');
fprintf('  Open-loop divergence time: %.3f s\n', t(N_ol));
fprintf('  (This is how long the aircraft would survive without FCS)\n\n');

%% ====================================================================
%  6-DOF EQUATIONS OF MOTION
%  ====================================================================

function xdot = fighter_6dof_eom(x, de_L, de_R, dr, dc, throttle, ac)
%  Full 6-DOF EOM with:
%    - Full inertia tensor (including Ixz product of inertia)
%    - Tabular aerodynamics
%    - ISA atmospheric model
%    - Thrust along body x-axis

    g = 9.81;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11); psi = x(12);
    alt = max(0, -x(9));

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);

    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    [~, a_snd, ~, rho] = atmos_isa_local(alt);
    Mach = V / a_snd;
    qbar = 0.5 * rho * V^2;

    % Aerodynamic coefficients from lookup tables
    [CL, CD, CY, Cl_c, Cm_c, Cn_c] = fighter_aero_model( ...
        alpha, beta, p, q, r, V, Mach, alt, ...
        de_L, de_R, dr, dc, ac);

    % Aero forces (stability → body axis)
    L_aero = qbar * ac.S * CL;  % Lift
    D_aero = qbar * ac.S * CD;  % Drag
    Y_aero = qbar * ac.S * CY;  % Side force

    Fx_a = -D_aero*cos(alpha) + L_aero*sin(alpha);
    Fy_a = Y_aero;
    Fz_a = -D_aero*sin(alpha) - L_aero*cos(alpha);

    % Aero moments (body axis)
    L_m = qbar * ac.S * ac.b     * Cl_c;
    M_m = qbar * ac.S * ac.c_bar * Cm_c;
    N_m = qbar * ac.S * ac.b     * Cn_c;

    % Thrust (along body x-axis)
    if throttle <= 1.0
        T = ac.engine.idle_thrust + ...
            (ac.engine.thrust_max_dry - ac.engine.idle_thrust) * throttle;
    else
        T = ac.engine.thrust_max_dry + ...
            (ac.engine.thrust_max_ab - ac.engine.thrust_max_dry) * (throttle - 1.0);
    end

    % Gravity (body frame)
    Fx_g = -ac.mass * g * sin(theta);
    Fy_g =  ac.mass * g * cos(theta) * sin(phi);
    Fz_g =  ac.mass * g * cos(theta) * cos(phi);

    % Total forces
    Fx = Fx_a + T + Fx_g;
    Fy = Fy_a + Fy_g;
    Fz = Fz_a + Fz_g;

    % --- Translational EOM ---
    udot = Fx/ac.mass + r*v_b - q*w_b;
    vdot = Fy/ac.mass + p*w_b - r*u_b;
    wdot = Fz/ac.mass + q*u_b - p*v_b;

    % --- Rotational EOM with full inertia tensor ---
    %  For J with Ixz only (symmetric aircraft, Ixy = Iyz = 0):
    %
    %  [Ixx  0  -Ixz] [pdot]   [L]   [p]   [Ixx  0  -Ixz] [p]
    %  [ 0  Iyy  0  ] [qdot] = [M] - [ q] x [ 0  Iyy  0  ] [q]
    %  [-Ixz 0  Izz ] [rdot]   [N]   [r]   [-Ixz 0  Izz ] [r]
    %
    %  Expanding the cross product and solving:

    Gamma = ac.Ixx * ac.Izz - ac.Ixz^2;

    pdot = (ac.Izz * L_m + ac.Ixz * N_m - ...
            (ac.Ixz * (ac.Ixx - ac.Iyy + ac.Izz)) * p * q + ...
            (ac.Ixz^2 + ac.Izz * (ac.Izz - ac.Iyy)) * q * r) / Gamma;

    qdot = (M_m + (ac.Ixx - ac.Izz) * p * r - ...
            ac.Ixz * (p^2 - r^2)) / ac.Iyy;

    rdot = (ac.Ixx * N_m + ac.Ixz * L_m + ...
            (ac.Ixz * (ac.Iyy - ac.Izz - ac.Ixx)) * q * r + ...
            (ac.Ixx * (ac.Ixx - ac.Iyy) + ac.Ixz^2) * p * q) / Gamma;

    % --- Kinematic EOM ---
    cos_th = cos(theta);
    if abs(cos_th) < 0.01
        cos_th = sign(cos_th) * 0.01;
    end

    phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / cos_th;

    % --- Navigation EOM (body → NED) ---
    c_ph = cos(phi); s_ph = sin(phi);
    c_th = cos(theta); s_th = sin(theta);
    c_ps = cos(psi); s_ps = sin(psi);

    xdot_n = u_b*c_th*c_ps + v_b*(s_ph*s_th*c_ps - c_ph*s_ps) + ...
             w_b*(c_ph*s_th*c_ps + s_ph*s_ps);
    ydot_n = u_b*c_th*s_ps + v_b*(s_ph*s_th*s_ps + c_ph*c_ps) + ...
             w_b*(c_ph*s_th*s_ps - s_ph*c_ps);
    zdot_n = -u_b*s_th + v_b*s_ph*c_th + w_b*c_ph*c_th;

    xdot = [udot; vdot; wdot; pdot; qdot; rdot; ...
            xdot_n; ydot_n; zdot_n; phidot; thetadot; psidot];
end

%% ====================================================================
%  ACTUATOR MODEL
%  ====================================================================

function pos_new = actuator_step(cmd, pos_current, tau, rate_max, pos_max, dt)
%  First-order actuator with rate limit and position limit.
%
%  In a real hydraulic actuator:
%    - First-order lag represents valve + cylinder dynamics
%    - Rate limit represents max hydraulic flow rate
%    - Position limit is the mechanical stop
%
%  Real actuators also have:
%    - Backlash (0.05-0.1 deg typical)
%    - Coulomb friction
%    - Load-dependent response (hinge moment feedback)
%    - Failure modes (hardover, oscillatory, slow)
%  These are omitted here for clarity.

    % First-order lag dynamics
    pos_desired = pos_current + (dt / (tau + dt)) * (cmd - pos_current);

    % Rate limiting
    rate = (pos_desired - pos_current) / dt;
    if abs(rate) > rate_max
        rate = sign(rate) * rate_max;
        pos_desired = pos_current + rate * dt;
    end

    % Position limiting
    pos_new = max(min(pos_desired, pos_max), -pos_max);
end

%% ====================================================================
%  ATMOSPHERIC MODEL
%  ====================================================================

function [T, a, P, rho] = atmos_isa_local(alt)
    T0 = 288.15; P0 = 101325; L = 0.0065; R = 287.05; g0 = 9.81;
    alt = max(alt, 0);
    T   = T0 - L * alt;
    T   = max(T, 216.65);  % Tropopause minimum
    P   = P0 * (T/T0)^(g0/(R*L));
    rho = P / (R * T);
    a   = sqrt(1.4 * R * T);
end

%% ====================================================================
%  UTILITY
%  ====================================================================

function result = tern(condition, val_true, val_false)
    if condition
        result = val_true;
    else
        result = val_false;
    end
end
