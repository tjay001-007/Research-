%% ========================================================================
%  RUN_UCAV_MISSION  —  Pure-MATLAB UCAV trajectory-following simulation
%  ========================================================================
%
%  Runs the complete autonomous UCAV mission in pure MATLAB (no Simulink).
%  Integrates the 6-DOF equations of motion with INDI controller and
%  L1/PI guidance at 250 Hz using RK4 integration.
%
%  This script validates the control and guidance algorithms before
%  Simulink integration. The same functions (indi_controller, ucav_guidance_law,
%  ucav_aerodynamics) are used in both this script and the S-functions.
%
%  Simulation includes:
%    - Full 6-DOF nonlinear EOM with Ixz inertia coupling
%    - ISA atmosphere (density varies with altitude)
%    - Actuator dynamics (first-order lag + rate limit + saturation)
%    - INDI inner loop at 250 Hz
%    - L1 lateral + PI altitude/speed guidance
%    - Mission telemetry and analysis plots
%
%  Usage:
%    >> setup_ucav          % Load parameters (MUST run first)
%    >> run_ucav_mission    % This script
%
%  ========================================================================

fprintf('================================================================\n');
fprintf('  UCAV Mission Simulation (Pure MATLAB + INDI)\n');
fprintf('================================================================\n\n');

%% Check prerequisites
if ~exist('aircraft', 'var') || ~exist('fcs', 'var') || ...
   ~exist('mission', 'var') || ~exist('guidance', 'var')
    fprintf('  Running setup_ucav...\n\n');
    setup_ucav;
end

%% Simulation setup
dt     = sim_params.dt;
t_end  = sim_params.t_end;
N_sim  = round(t_end / dt) + 1;
t_vec  = (0:N_sim-1)' * dt;

fprintf('  Duration:   %.0f s (%.1f min)\n', t_end, t_end/60);
fprintf('  Time step:  %.4f s (%.0f Hz)\n', dt, 1/dt);
fprintf('  Total steps: %d\n\n', N_sim);

%% Preallocate storage
% Aircraft state: [u,v,w, p,q,r, N,E,D, phi,theta,psi]
state = zeros(N_sim, 12);
state(1,:) = initial.state';

% Actuator states (actual positions after dynamics)
act_state = zeros(N_sim, 3);  % [da_act, de_act, dr_act]
act_state(1,:) = [initial.da, initial.de, initial.dr];

% Controller commands
cmd_log = zeros(N_sim, 3);    % [da_cmd, de_cmd, dr_cmd]
cmd_log(1,:) = [initial.da, initial.de, initial.dr];

% Guidance outputs
guid_log = zeros(N_sim, 8);   % [phi_cmd, theta_cmd, thr_cmd, wp_idx, xtrack, alt_cmd, alt, V]
thr_log  = zeros(N_sim, 1);
thr_log(1) = trim.throttle;

% Angular accelerations (for INDI)
omega_dot_log = zeros(N_sim, 3);

% Controller integrator state
ctrl_state.int_p = 0;
ctrl_state.int_q = 0;
ctrl_state.int_r = 0;

% Guidance state
guid_state.wp_idx    = 2;      % Start targeting WP2
guid_state.lap_count = 0;
guid_state.int_alt   = 0;
guid_state.int_V     = 0;

%% Main simulation loop
fprintf('  Simulating');
print_interval = round(N_sim / 20);  % Print progress every 5%

for k = 1:N_sim-1
    % Progress indicator
    if mod(k, print_interval) == 0
        fprintf('.');
    end

    % Current state
    x = state(k,:)';
    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4);   q = x(5);   r = x(6);
    pos_N = x(7); pos_E = x(8); pos_D = x(9);
    phi = x(10); theta = x(11); psi = x(12);

    % Current actuator positions
    da_act = act_state(k,1);
    de_act = act_state(k,2);
    dr_act = act_state(k,3);

    % Airspeed
    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 10);
    alt = -pos_D;

    % --- Compute angular accelerations (for INDI) ---
    %   In real flight: differentiate gyro measurements
    %   In simulation: compute from EOM directly (cleaner)
    [~, M_aero, ~] = ucav_aerodynamics(atan2(w_b, u_b), ...
        asin(max(min(v_b/V,1),-1)), V, alt, p, q, r, da_act, de_act, dr_act, aircraft);

    % Add gravity and thrust moments = 0 (thrust through CG, no moment arm assumed)
    Gam = aircraft.Gamma;
    pdot = (aircraft.Izz * M_aero(1) + aircraft.Ixz * M_aero(3) ...
            - (aircraft.Ixz*(aircraft.Ixx - aircraft.Iyy + aircraft.Izz))*p*q ...
            + (aircraft.Ixz^2 + aircraft.Izz*(aircraft.Izz - aircraft.Iyy))*q*r) / Gam;
    qdot = (M_aero(2) + (aircraft.Ixx - aircraft.Izz)*p*r ...
            - aircraft.Ixz*(p^2 - r^2)) / aircraft.Iyy;
    rdot = (aircraft.Ixx * M_aero(3) + aircraft.Ixz * M_aero(1) ...
            + (aircraft.Ixz*(aircraft.Iyy - aircraft.Izz - aircraft.Ixx))*q*r ...
            + (aircraft.Ixx*(aircraft.Ixx - aircraft.Iyy) + aircraft.Ixz^2)*p*q) / Gam;

    omega_dot_log(k,:) = [pdot, qdot, rdot];

    % --- Guidance law ---
    [phi_cmd, theta_cmd, thr_cmd, guid_state, g_debug] = ucav_guidance_law( ...
        pos_N, pos_E, pos_D, u_b, v_b, w_b, phi, theta, psi, V, ...
        guid_state, mission, guidance, trim, dt);

    guid_log(k,:) = [phi_cmd, theta_cmd, thr_cmd, g_debug.wp_idx, ...
                      g_debug.xtrack, g_debug.alt_cmd, alt, V];
    thr_log(k) = thr_cmd;

    % --- INDI controller ---
    [da_cmd, de_cmd, dr_cmd, ctrl_state, ~] = indi_controller( ...
        phi_cmd, theta_cmd, ...
        u_b, v_b, w_b, p, q, r, phi, theta, psi, ...
        pdot, qdot, rdot, ...
        da_act, de_act, dr_act, ...
        ctrl_state, aircraft, fcs, dt);

    cmd_log(k,:) = [da_cmd, de_cmd, dr_cmd];

    % --- Actuator dynamics ---
    %   First-order lag + rate limit + position saturation
    [da_new, de_new, dr_new] = actuator_dynamics( ...
        da_cmd, de_cmd, dr_cmd, da_act, de_act, dr_act, aircraft.act, dt);

    % --- Integrate 6-DOF EOM using RK4 ---
    x_new = rk4_step(@(xx) eom_6dof(xx, da_new, de_new, dr_new, thr_cmd, aircraft), x, dt);

    % --- Store results ---
    state(k+1,:) = x_new';
    act_state(k+1,:) = [da_new, de_new, dr_new];
    cmd_log(k+1,:) = [da_cmd, de_cmd, dr_cmd];
end

% Fill last step for logging
guid_log(N_sim,:) = guid_log(N_sim-1,:);
thr_log(N_sim) = thr_log(N_sim-1);
omega_dot_log(N_sim,:) = omega_dot_log(N_sim-1,:);

fprintf(' Done!\n\n');

%% ====================================================================
%  POST-PROCESSING
%  ====================================================================

% Extract signals for plotting
pos_N_log = state(:,7);
pos_E_log = state(:,8);
alt_log   = -state(:,9);
phi_log   = rad2deg(state(:,10));
theta_log = rad2deg(state(:,11));
psi_log   = rad2deg(state(:,12));
V_log     = sqrt(state(:,1).^2 + state(:,2).^2 + state(:,3).^2);
alpha_log = rad2deg(atan2(state(:,3), state(:,1)));
beta_log  = rad2deg(asin(max(min(state(:,2)./max(V_log,10), 1), -1)));

% Waypoints for plotting
wp_N = mission.waypoints_ned(:,1);
wp_E = mission.waypoints_ned(:,2);

%% ====================================================================
%  FIGURE 1: GROUND TRACK AND TRAJECTORY
%  ====================================================================

figure('Name', 'UCAV Mission - Ground Track', 'Position', [50, 400, 800, 600]);

subplot(2,2,[1,3]);
plot(pos_E_log, pos_N_log, 'b-', 'LineWidth', 1.5); hold on;
plot(wp_E, wp_N, 'rs', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
% Close the racetrack for visualization
wp_E_closed = [wp_E; wp_E(1)];
wp_N_closed = [wp_N; wp_N(1)];
plot(wp_E_closed, wp_N_closed, 'r--', 'LineWidth', 1);
for i = 1:length(wp_N)
    text(wp_E(i)+100, wp_N(i)+100, sprintf('WP%d', i), 'FontWeight', 'bold');
end
plot(pos_E_log(1), pos_N_log(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(pos_E_log(end), pos_N_log(end), 'kx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('East (m)'); ylabel('North (m)');
title('Ground Track'); axis equal; grid on;
legend('Flight path', 'Waypoints', 'Racetrack', 'Start', 'End', 'Location', 'best');

subplot(2,2,2);
plot(t_vec, alt_log, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, guid_log(:,6), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Altitude Profile'); grid on;
legend('Actual', 'Commanded');

subplot(2,2,4);
plot(t_vec, V_log, 'b-', 'LineWidth', 1.5); hold on;
plot(t_vec, mission.V_cmd * ones(size(t_vec)), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Airspeed (m/s)');
title('Airspeed'); grid on;
legend('Actual', 'Commanded');

%% ====================================================================
%  FIGURE 2: NAVIGATION PERFORMANCE
%  ====================================================================

figure('Name', 'UCAV Mission - Navigation', 'Position', [100, 350, 800, 600]);

subplot(3,1,1);
plot(t_vec, guid_log(:,5), 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Cross-track (m)');
title('Cross-Track Error'); grid on;
yline(0, 'k--');

subplot(3,1,2);
plot(t_vec, guid_log(:,6) - alt_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Alt error (m)');
title('Altitude Error'); grid on;
yline(0, 'k--');

subplot(3,1,3);
stairs(t_vec, guid_log(:,4), 'r-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('WP Index');
title('Waypoint Sequencing'); grid on;

%% ====================================================================
%  FIGURE 3: ATTITUDE AND CONTROL
%  ====================================================================

figure('Name', 'UCAV Mission - Attitude & Control', 'Position', [150, 300, 800, 700]);

subplot(3,2,1);
plot(t_vec, phi_log, 'b-', 'LineWidth', 1); hold on;
plot(t_vec, rad2deg(guid_log(:,1)), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Roll (deg)'); title('Roll Angle');
grid on; legend('Actual', 'Commanded');

subplot(3,2,2);
plot(t_vec, theta_log, 'b-', 'LineWidth', 1); hold on;
plot(t_vec, rad2deg(guid_log(:,2)), 'r--', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Pitch (deg)'); title('Pitch Angle');
grid on; legend('Actual', 'Commanded');

subplot(3,2,3);
plot(t_vec, alpha_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Alpha (deg)'); title('Angle of Attack');
grid on;

subplot(3,2,4);
plot(t_vec, beta_log, 'b-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Beta (deg)'); title('Sideslip');
grid on;

subplot(3,2,5);
plot(t_vec, rad2deg(act_state(:,1)), 'b-', ...
     t_vec, rad2deg(act_state(:,2)), 'r-', ...
     t_vec, rad2deg(act_state(:,3)), 'g-', 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Deflection (deg)'); title('Actuator Positions');
grid on; legend('Aileron', 'Elevator', 'Rudder');

subplot(3,2,6);
plot(t_vec, thr_log, 'k-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Throttle (0-1)'); title('Throttle');
grid on; ylim([0 1]);

%% ====================================================================
%  FIGURE 4: 3D TRAJECTORY
%  ====================================================================

figure('Name', 'UCAV Mission - 3D Trajectory', 'Position', [200, 250, 700, 500]);
plot3(pos_E_log, pos_N_log, alt_log, 'b-', 'LineWidth', 1.5); hold on;
plot3(wp_E, wp_N, -mission.waypoints_ned(:,3), 'rs', ...
    'MarkerSize', 12, 'MarkerFaceColor', 'r');
xlabel('East (m)'); ylabel('North (m)'); zlabel('Altitude (m)');
title('3D Trajectory'); grid on;
view(30, 25);

%% ====================================================================
%  MISSION SUMMARY
%  ====================================================================

total_dist = sum(sqrt(diff(pos_N_log).^2 + diff(pos_E_log).^2));
max_xtrack = max(abs(guid_log(:,5)));
mean_xtrack = mean(abs(guid_log(:,5)));
alt_rmse = sqrt(mean((guid_log(:,6) - alt_log).^2));
V_rmse = sqrt(mean((mission.V_cmd - V_log).^2));

fprintf('  ============ MISSION SUMMARY ============\n');
fprintf('  Total distance flown:  %.1f km\n', total_dist/1000);
fprintf('  Laps completed:        %d\n', guid_state.lap_count);
fprintf('  Max cross-track error: %.1f m\n', max_xtrack);
fprintf('  Mean cross-track error: %.1f m\n', mean_xtrack);
fprintf('  Altitude RMSE:         %.1f m\n', alt_rmse);
fprintf('  Airspeed RMSE:         %.2f m/s\n', V_rmse);
fprintf('  Alpha range:           [%.1f, %.1f] deg\n', min(alpha_log), max(alpha_log));
fprintf('  Beta range:            [%.1f, %.1f] deg\n', min(beta_log), max(beta_log));
fprintf('  Max bank angle:        %.1f deg\n', max(abs(phi_log)));
fprintf('  ==========================================\n\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function xdot = eom_6dof(x, da, de, dr, thr, ac)
%EOM_6DOF  Full nonlinear 6-DOF rigid body equations of motion.
%   Includes full inertia tensor with Ixz product of inertia.
    g = 9.81;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11);
    alt = max(0, -x(9));

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);
    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    % Aerodynamic forces and moments
    [F_aero, M_aero, ~] = ucav_aerodynamics(alpha, beta, V, alt, ...
        p, q, r, da, de, dr, ac);

    % Thrust (along body x-axis)
    T = ac.engine.idle_thrust + (ac.engine.thrust_max - ac.engine.idle_thrust) * max(min(thr, 1), 0);

    % Gravity in body frame
    Fg_x = -ac.mass * g * sin(theta);
    Fg_y =  ac.mass * g * cos(theta) * sin(phi);
    Fg_z =  ac.mass * g * cos(theta) * cos(phi);

    % Total forces
    Fx = F_aero(1) + T + Fg_x;
    Fy = F_aero(2) + Fg_y;
    Fz = F_aero(3) + Fg_z;

    % --- Translational dynamics ---
    udot = Fx/ac.mass + r*v_b - q*w_b;
    vdot = Fy/ac.mass + p*w_b - r*u_b;
    wdot = Fz/ac.mass + q*u_b - p*v_b;

    % --- Rotational dynamics (full inertia with Ixz) ---
    Gam = ac.Gamma;
    pdot = (ac.Izz*M_aero(1) + ac.Ixz*M_aero(3) ...
            - (ac.Ixz*(ac.Ixx - ac.Iyy + ac.Izz))*p*q ...
            + (ac.Ixz^2 + ac.Izz*(ac.Izz - ac.Iyy))*q*r) / Gam;
    qdot = (M_aero(2) + (ac.Ixx - ac.Izz)*p*r - ac.Ixz*(p^2 - r^2)) / ac.Iyy;
    rdot = (ac.Ixx*M_aero(3) + ac.Ixz*M_aero(1) ...
            + (ac.Ixz*(ac.Iyy - ac.Izz - ac.Ixx))*q*r ...
            + (ac.Ixx*(ac.Ixx - ac.Iyy) + ac.Ixz^2)*p*q) / Gam;

    % --- Euler angle kinematics ---
    ct = cos(theta);
    if abs(ct) < 0.001, ct = sign(ct)*0.001; end

    phidot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / ct;

    % --- Navigation (body to NED) ---
    sp = sin(phi); cp = cos(phi);
    st = sin(theta); cth = cos(theta);
    sps = sin(x(12)); cps = cos(x(12));

    Ndot = u_b*cth*cps + v_b*(sp*st*cps - cp*sps) + w_b*(cp*st*cps + sp*sps);
    Edot = u_b*cth*sps + v_b*(sp*st*sps + cp*cps) + w_b*(cp*st*sps - sp*cps);
    Ddot = -u_b*st     + v_b*sp*cth                + w_b*cp*cth;

    xdot = [udot; vdot; wdot; pdot; qdot; rdot; Ndot; Edot; Ddot; phidot; thetadot; psidot];
end

function x_new = rk4_step(f, x, dt)
%RK4_STEP  Fourth-order Runge-Kutta integration step.
    k1 = f(x);
    k2 = f(x + 0.5*dt*k1);
    k3 = f(x + 0.5*dt*k2);
    k4 = f(x + dt*k3);
    x_new = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function [da_new, de_new, dr_new] = actuator_dynamics(da_cmd, de_cmd, dr_cmd, ...
    da_act, de_act, dr_act, act, dt)
%ACTUATOR_DYNAMICS  First-order lag + rate limit + position saturation.
    da_new = actuate_single(da_cmd, da_act, act.tau, ...
        act.aileron.rate_max, act.aileron.pos_max, dt);
    de_new = actuate_single(de_cmd, de_act, act.tau, ...
        act.elevator.rate_max, act.elevator.pos_max, dt);
    dr_new = actuate_single(dr_cmd, dr_act, act.tau, ...
        act.rudder.rate_max, act.rudder.pos_max, dt);
end

function y = actuate_single(cmd, pos, tau, rate_max, pos_max, dt)
%ACTUATE_SINGLE  Single-channel actuator with lag, rate limit, and saturation.
    % First-order lag: dy/dt = (cmd - y) / tau
    y_dot = (cmd - pos) / tau;
    % Rate limit
    y_dot = max(min(y_dot, rate_max), -rate_max);
    % Integrate
    y = pos + y_dot * dt;
    % Position saturation
    y = max(min(y, pos_max), -pos_max);
end
