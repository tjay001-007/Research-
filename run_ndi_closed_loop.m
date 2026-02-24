%% NDI Closed-Loop Simulation Runner
%  Runs the pitch-unstable aircraft with NDI controller in pure MATLAB
%  (no Simulink required). Useful for validation and tuning before
%  connecting to the full Simulink/Gazebo pipeline.
%
%  This script:
%    1. Sets up the aircraft and controller
%    2. Runs a time-domain simulation with Euler integration
%    3. Injects step commands in pitch and roll
%    4. Plots the response and control activity
%
%  The same ndi_flight_controller.m function used here is identical to
%  what runs inside ndi_controller_sfunc.m in Simulink.

clear all; close all; clc;

disp('================================================');
disp('NDI Closed-Loop Simulation');
disp('Pitch-Unstable Aircraft');
disp('================================================');
disp(' ');

%% Setup

setup_aircraft_parameters;
setup_ndi_controller;

%% Simulation Parameters

dt       = 0.004;          % 250 Hz (matches Simulink model)
t_end    = 20;             % Simulation duration (s)
t        = 0:dt:t_end;
N        = length(t);

%% Command Profile
%
%  Time (s):   0-2     2-6     6-10    10-14   14-20
%  theta_cmd:  0       5 deg   0       -3 deg  0
%  phi_cmd:    0       0       15 deg  15 deg  0

phi_cmd_deg   = zeros(N, 1);
theta_cmd_deg = zeros(N, 1);

for k = 1:N
    if t(k) >= 2 && t(k) < 6
        theta_cmd_deg(k) = 5;
    elseif t(k) >= 10 && t(k) < 14
        theta_cmd_deg(k) = -3;
    end

    if t(k) >= 6 && t(k) < 14
        phi_cmd_deg(k) = 15;
    end
end

phi_cmd   = deg2rad(phi_cmd_deg);
theta_cmd = deg2rad(theta_cmd_deg);

%% State Initialization
%  State: [u v w p q r x y z phi theta psi]

x = initial.state;

% Preallocate logging arrays
log_state   = zeros(N, 12);
log_ctrl    = zeros(N, 3);    % [da, de, dr]
log_omega_c = zeros(N, 3);    % [p_cmd, q_cmd, r_cmd]
log_alpha   = zeros(N, 1);
log_beta    = zeros(N, 1);
log_V       = zeros(N, 1);

% NDI integrator state
int_state = zeros(3, 1);

% Fixed throttle for this demo (simple altitude hold would go here)
throttle = 0.5;

disp('Running simulation...');

%% Main Simulation Loop

for k = 1:N

    % --- Log current state ---
    log_state(k, :) = x';

    u_b = x(1);  v_b = x(2);  w_b = x(3);
    p   = x(4);  q   = x(5);  r   = x(6);
    phi = x(10); theta = x(11); psi = x(12);

    % --- NDI Controller ---
    [da, de, dr, int_state, dbg] = ndi_flight_controller( ...
        phi_cmd(k), theta_cmd(k), ...
        u_b, v_b, w_b, p, q, r, phi, theta, ...
        int_state, aircraft, ndi_gains, dt);

    log_ctrl(k, :)    = [da, de, dr];
    log_omega_c(k, :) = dbg.omega_cmd';
    log_alpha(k)      = dbg.alpha;
    log_beta(k)       = dbg.beta;
    log_V(k)          = dbg.V;

    % --- Propagate 6-DOF dynamics (RK4 integration) ---
    k1 = aircraft_eom(x, da, de, dr, throttle, aircraft);
    k2 = aircraft_eom(x + 0.5*dt*k1, da, de, dr, throttle, aircraft);
    k3 = aircraft_eom(x + 0.5*dt*k2, da, de, dr, throttle, aircraft);
    k4 = aircraft_eom(x + dt*k3, da, de, dr, throttle, aircraft);
    x  = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    % Wrap Euler angles to [-pi, pi]
    x(10) = wrapToPi(x(10));
    x(11) = wrapToPi(x(11));
    x(12) = wrapToPi(x(12));
end

disp('Simulation complete.');
disp(' ');

%% ====================================================================
%  PLOTS
%  ====================================================================

figure('Name', 'NDI Closed-Loop Response', 'Position', [100 100 1000 800]);

% --- Pitch angle ---
subplot(3, 2, 1);
plot(t, rad2deg(log_state(:,11)), 'b', 'LineWidth', 1.5); hold on;
plot(t, theta_cmd_deg, 'r--', 'LineWidth', 1.2);
ylabel('Pitch (deg)');
title('Pitch Angle Tracking');
legend('Actual', 'Command', 'Location', 'best');
grid on;

% --- Roll angle ---
subplot(3, 2, 2);
plot(t, rad2deg(log_state(:,10)), 'b', 'LineWidth', 1.5); hold on;
plot(t, phi_cmd_deg, 'r--', 'LineWidth', 1.2);
ylabel('Roll (deg)');
title('Roll Angle Tracking');
legend('Actual', 'Command', 'Location', 'best');
grid on;

% --- Angular rates ---
subplot(3, 2, 3);
plot(t, rad2deg(log_state(:,5)), 'b', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(log_omega_c(:,2)), 'r--', 'LineWidth', 1);
ylabel('q (deg/s)');
title('Pitch Rate');
legend('Actual', 'Commanded', 'Location', 'best');
grid on;

subplot(3, 2, 4);
plot(t, rad2deg(log_state(:,4)), 'b', 'LineWidth', 1.2); hold on;
plot(t, rad2deg(log_omega_c(:,1)), 'r--', 'LineWidth', 1);
ylabel('p (deg/s)');
title('Roll Rate');
legend('Actual', 'Commanded', 'Location', 'best');
grid on;

% --- Control surfaces ---
subplot(3, 2, 5);
plot(t, log_ctrl(:,1), 'b', 'LineWidth', 1); hold on;
plot(t, log_ctrl(:,2), 'r', 'LineWidth', 1);
plot(t, log_ctrl(:,3), 'g', 'LineWidth', 1);
ylabel('Deflection (norm)');
xlabel('Time (s)');
title('Control Surface Commands');
legend('Aileron', 'Elevator', 'Rudder', 'Location', 'best');
grid on; ylim([-1.2, 1.2]);

% --- Airspeed and AoA ---
subplot(3, 2, 6);
yyaxis left;
plot(t, log_V, 'b', 'LineWidth', 1.2);
ylabel('Airspeed (m/s)');
yyaxis right;
plot(t, rad2deg(log_alpha), 'r', 'LineWidth', 1);
ylabel('Alpha (deg)');
xlabel('Time (s)');
title('Airspeed & Angle of Attack');
grid on;

sgtitle('NDI Controller — Pitch-Unstable Aircraft', 'FontSize', 14);

% --- 3D trajectory ---
figure('Name', 'Flight Path', 'Position', [200 200 600 500]);
plot3(log_state(:,7), log_state(:,8), -log_state(:,9), 'b', 'LineWidth', 1.5);
xlabel('North (m)'); ylabel('East (m)'); zlabel('Altitude (m)');
title('3D Flight Path');
grid on; axis equal;

% --- Sideslip ---
figure('Name', 'Sideslip', 'Position', [300 300 500 300]);
plot(t, rad2deg(log_beta), 'b', 'LineWidth', 1.2);
ylabel('Beta (deg)'); xlabel('Time (s)');
title('Sideslip Angle (coordinated flight check)');
grid on;

disp('Figures generated.');

%% ====================================================================
%  OPEN-LOOP COMPARISON — demonstrate the instability
%  ====================================================================

disp(' ');
disp('Running open-loop comparison (no controller)...');

x_ol = initial.state;
log_ol = zeros(min(N, round(3/dt)), 12);  % Only 3 seconds — it diverges fast
N_ol = size(log_ol, 1);

for k = 1:N_ol
    log_ol(k, :) = x_ol';
    xdot = aircraft_eom(x_ol, 0, 0, 0, 0.5, aircraft);
    x_ol = x_ol + dt * xdot;
end

figure('Name', 'Open-Loop Divergence', 'Position', [400 400 600 400]);
subplot(2,1,1);
plot(t(1:N_ol), rad2deg(log_ol(:,11)), 'r', 'LineWidth', 1.5); hold on;
plot(t(1:min(N,N_ol)), rad2deg(log_state(1:min(N,N_ol), 11)), 'b', 'LineWidth', 1.5);
ylabel('Pitch (deg)'); xlabel('Time (s)');
title('Open-Loop vs NDI Closed-Loop');
legend('Open-loop (UNSTABLE)', 'NDI controlled', 'Location', 'best');
grid on;

subplot(2,1,2);
plot(t(1:N_ol), log_ol(:,1), 'r', 'LineWidth', 1.5); hold on;
plot(t(1:min(N,N_ol)), log_state(1:min(N,N_ol), 1), 'b', 'LineWidth', 1.5);
ylabel('u (m/s)'); xlabel('Time (s)');
title('Forward Velocity');
legend('Open-loop', 'NDI controlled', 'Location', 'best');
grid on;

sgtitle('Effect of NDI on Pitch-Unstable Aircraft', 'FontSize', 14);

disp('Done. Compare open-loop divergence with NDI-stabilized flight.');

%% ====================================================================
%  6-DOF EQUATIONS OF MOTION (standalone — mirrors aircraft_6dof_sfunc.m)
%  ====================================================================

function xdot = aircraft_eom(x, aileron, elevator, rudder, throttle, ac)
    % Implements the same dynamics as aircraft_6dof_sfunc.m Derivatives()
    %
    % State: [u v w p q r x y z phi theta psi]'

    g   = 9.81;
    rho = 1.225;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4);   q = x(5);   r = x(6);
    phi = x(10); theta = x(11); psi = x(12);

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 1.0);

    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));
    qbar  = 0.5 * rho * V^2;

    p_hat = p * ac.b / (2*V);
    q_hat = q * ac.c / (2*V);
    r_hat = r * ac.b / (2*V);

    % Aero coefficients
    CL = ac.CL0 + ac.CLalpha*alpha + ac.CLde*elevator;
    CD = ac.CD0 + ac.CDalpha*alpha^2;
    CY = ac.CYbeta*beta + ac.CYdr*rudder;

    Cl = ac.Clbeta*beta + ac.Clp*p_hat + ac.Clr*r_hat + ac.Clda*aileron;
    Cm = ac.Cm0 + ac.Cmalpha*alpha + ac.Cmq*q_hat + ac.Cmde*elevator;
    Cn = ac.Cnbeta*beta + ac.Cnr*r_hat + ac.Cnda*aileron + ac.Cndr*rudder;

    % Forces (body axis)
    L_aero = qbar * ac.S * CL;
    D_aero = qbar * ac.S * CD;
    Y_aero = qbar * ac.S * CY;

    Fx_aero = -D_aero*cos(alpha) + L_aero*sin(alpha);
    Fy_aero = Y_aero;
    Fz_aero = -D_aero*sin(alpha) - L_aero*cos(alpha);

    T_max = 25.0;
    Fx_thrust = T_max * throttle;

    Fx_grav = -ac.mass * g * sin(theta);
    Fy_grav =  ac.mass * g * cos(theta) * sin(phi);
    Fz_grav =  ac.mass * g * cos(theta) * cos(phi);

    Fx = Fx_aero + Fx_thrust + Fx_grav;
    Fy = Fy_aero + Fy_grav;
    Fz = Fz_aero + Fz_grav;

    % Moments
    L_m = qbar * ac.S * ac.b * Cl;
    M_m = qbar * ac.S * ac.c * Cm;
    N_m = qbar * ac.S * ac.b * Cn;

    % Translational EOM
    udot = Fx/ac.mass + r*v_b - q*w_b;
    vdot = Fy/ac.mass + p*w_b - r*u_b;
    wdot = Fz/ac.mass + q*u_b - p*v_b;

    % Rotational EOM
    pdot = (L_m + (ac.Iyy - ac.Izz)*q*r) / ac.Ixx;
    qdot = (M_m + (ac.Izz - ac.Ixx)*p*r) / ac.Iyy;
    rdot = (N_m + (ac.Ixx - ac.Iyy)*p*q) / ac.Izz;

    % Kinematic EOM
    cos_theta = cos(theta);
    if abs(cos_theta) < 0.01
        cos_theta = sign(cos_theta) * 0.01;
    end

    phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / cos_theta;

    % Navigation EOM
    c_phi = cos(phi); s_phi = sin(phi);
    c_th  = cos(theta); s_th = sin(theta);
    c_psi = cos(psi); s_psi = sin(psi);

    xdot_n = u_b*c_th*c_psi + v_b*(s_phi*s_th*c_psi - c_phi*s_psi) + ...
             w_b*(c_phi*s_th*c_psi + s_phi*s_psi);
    ydot_n = u_b*c_th*s_psi + v_b*(s_phi*s_th*s_psi + c_phi*c_psi) + ...
             w_b*(c_phi*s_th*s_psi - s_phi*c_psi);
    zdot_n = -u_b*s_th + v_b*s_phi*c_th + w_b*c_phi*c_th;

    xdot = [udot; vdot; wdot; pdot; qdot; rdot; ...
            xdot_n; ydot_n; zdot_n; phidot; thetadot; psidot];
end
