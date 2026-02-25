%% ========================================================================
%  UCAV AUTONOMOUS MISSION SIMULATION
%  ========================================================================
%
%  MODULE PURPOSE:
%  ---------------
%  This is the top-level simulation script that runs the complete
%  autonomous UCAV mission from takeoff through an ISR racetrack patrol
%  pattern and return. It integrates all modules:
%
%    1. Mission trajectory (lat/lon/alt waypoints) → NED conversion
%    2. L1 lateral guidance + TECS altitude/speed control
%    3. NDI/INDI attitude controller (stabilises the unstable airframe)
%    4. Actuator dynamics (rate + position limits)
%    5. 6-DOF nonlinear plant (tabular aero, full inertia tensor)
%    6. Navigation (NED → lat/lon/alt conversion for telemetry)
%
%  DATA FLOW:
%  ----------
%    ┌─────────────┐   ┌────────────┐   ┌──────────────┐   ┌────────┐   ┌───────┐
%    │  Mission    │──>│  Guidance   │──>│  NDI/INDI    │──>│Actuator│──>│6-DOF  │
%    │  Waypoints  │   │  (L1+TECS) │   │  Controller  │   │ Model  │   │Plant  │
%    │  (lat/lon/  │   │            │   │              │   │        │   │       │
%    │   alt)      │   │ phi_cmd    │   │ de_L, de_R   │   │rate+pos│   │forces │
%    │             │   │ theta_cmd  │   │ dr, dc       │   │limits  │   │moments│
%    │             │   │ throttle   │   │              │   │        │   │       │
%    └─────────────┘   └────────────┘   └──────────────┘   └────────┘   └───────┘
%                           ↑                   ↑               │           │
%                           │                   │               │           │
%                           └───────────────────┴───────────────┴───────────┘
%                                        State feedback
%                                  (position, velocity, attitude,
%                                   angular rates, accelerations)
%
%  ========================================================================

clear all; close all; clc;

fprintf('==========================================================\n');
fprintf(' UCAV AUTONOMOUS MISSION — FULL SIMULATION\n');
fprintf(' NDI/INDI + L1 Guidance + Trajectory Following\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  STEP 1: SETUP
%  ====================================================================
setup_ucav;

dt    = sim_params.dt;
t_end = sim_params.t_end;
t     = 0:dt:t_end;
N     = length(t);

%% ====================================================================
%  STEP 2: INITIALISE STATE AND CONTROLLER MEMORY
%  ====================================================================

x = initial.state;

% Actuator positions
act_de_L = initial.de_L;
act_de_R = initial.de_R;
act_dr   = initial.dr;
act_dc   = initial.dc;

% Controller persistent state
ctrl_state.int_omega = [0;0;0];
ctrl_state.de_L_prev = initial.de_L;
ctrl_state.de_R_prev = initial.de_R;
ctrl_state.dr_prev   = initial.dr;
ctrl_state.dc_prev   = initial.dc;

% Guidance persistent state
guid_state.wp_idx    = 2;       % Start targeting WP1 (1-indexed, WP0 is origin)
guid_state.lap_count = 0;
guid_state.int_hdot  = 0;
guid_state.int_V     = 0;

% INDI angular acceleration estimate
omega_prev = [0;0;0];
omega_dot_est = [0;0;0];
omega_dot_filt = [0;0;0];

%% ====================================================================
%  STEP 3: LOGGING ARRAYS
%  ====================================================================

log_state     = zeros(N, 12);
log_lla       = zeros(N, 3);      % lat, lon, alt (for telemetry)
log_surfaces  = zeros(N, 4);      % de_L, de_R, dr, dc (actual)
log_phi_cmd   = zeros(N, 1);
log_theta_cmd = zeros(N, 1);
log_thr_cmd   = zeros(N, 1);
log_alpha     = zeros(N, 1);
log_beta      = zeros(N, 1);
log_V         = zeros(N, 1);
log_Mach      = zeros(N, 1);
log_alt       = zeros(N, 1);
log_wp_idx    = zeros(N, 1);
log_crosstrack= zeros(N, 1);
log_dist_wp   = zeros(N, 1);
log_alt_cmd   = zeros(N, 1);
log_V_cmd     = zeros(N, 1);
log_omega_cmd = zeros(N, 3);

%% ====================================================================
%  STEP 4: MAIN SIMULATION LOOP
%  ====================================================================

fprintf('Simulating %.0f seconds (%.1f min) at %d Hz...\n', ...
    t_end, t_end/60, round(1/dt));
fprintf('Mission: %d waypoints, %.1f km, %d racetrack laps\n\n', ...
    mission.n_waypoints, mission.total_distance_km, mission.racetrack_laps);

tic;
last_print_time = -10;  % For progress reporting

for k = 1:N
    % ---- Extract aircraft state ----
    u_b = x(1); v_b = x(2); w_b = x(3);
    p   = x(4); q   = x(5); r   = x(6);
    pos_n = x(7); pos_e = x(8); pos_d = x(9);
    phi = x(10); theta = x(11); psi = x(12);

    V   = sqrt(u_b^2 + v_b^2 + w_b^2);
    V   = max(V, 10);
    alt = max(0, -pos_d);

    % ---- Log state ----
    log_state(k,:) = x';
    log_V(k)       = V;
    log_alt(k)     = alt;
    log_surfaces(k,:) = [act_de_L, act_de_R, act_dr, act_dc];

    % ---- Convert NED position back to lat/lon/alt for telemetry ----
    lat_current = mission.ref_lat + rad2deg(pos_n / mission.R_N);
    lon_current = mission.ref_lon + rad2deg(pos_e / (mission.R_E * ...
                  cos(deg2rad(mission.ref_lat))));
    log_lla(k,:) = [lat_current, lon_current, alt];

    % ---- Sensor simulation ----
    % Gyros (with realistic noise for tactical-grade IMU)
    gyro_noise  = deg2rad(0.01) * randn(3,1);
    accel_noise = 0.01 * randn(3,1);
    p_m = p + gyro_noise(1);
    q_m = q + gyro_noise(2);
    r_m = r + gyro_noise(3);

    % Body accelerations (simplified — real system uses IMU + nav filter)
    g0 = 9.81;
    ax_m = accel_noise(1);
    ay_m = g0*(-cos(theta)*sin(phi)) + accel_noise(2);
    az_m = g0*(-cos(theta)*cos(phi)) + accel_noise(3);

    % Angular acceleration estimate (differentiate filtered gyro)
    omega_now = [p_m; q_m; r_m];
    if k > 1
        omega_dot_raw = (omega_now - omega_prev) / dt;
        tau_f = 3*dt;
        omega_dot_filt = omega_dot_filt + (dt/(tau_f+dt)) * (omega_dot_raw - omega_dot_filt);
    end
    omega_prev = omega_now;

    % Compute actual Nz for az_meas (more accurate)
    alpha_now = atan2(w_b, u_b);
    beta_now  = asin(max(min(v_b/V,1),-1));
    [~,a_snd,~,rho_now] = atmos_local(alt);
    qbar_now = 0.5*rho_now*V^2;
    Mach_now = V / a_snd;
    [CL_now,~,~,~,~,~] = fighter_aero_model( ...
        alpha_now,beta_now,p,q,r,V,Mach_now,alt, ...
        act_de_L,act_de_R,act_dr,act_dc, aircraft);
    Nz_actual = qbar_now*aircraft.S*CL_now / (aircraft.mass*g0);
    az_m = -Nz_actual * g0;

    log_alpha(k) = rad2deg(alpha_now);
    log_beta(k)  = rad2deg(beta_now);
    log_Mach(k)  = Mach_now;

    % ==================================================================
    %  GUIDANCE LAW (L1 lateral + TECS longitudinal)
    % ==================================================================
    [phi_cmd, theta_cmd, thr_cmd, guid_state, gd] = ucav_guidance_law( ...
        pos_n, pos_e, pos_d, u_b, v_b, w_b, phi, theta, psi, V, ...
        guid_state, mission, guidance, dt);

    log_phi_cmd(k)    = phi_cmd;
    log_theta_cmd(k)  = theta_cmd;
    log_thr_cmd(k)    = thr_cmd;
    log_wp_idx(k)     = gd.wp_idx;
    log_crosstrack(k) = gd.crosstrack_error;
    log_dist_wp(k)    = gd.dist_to_wp;
    log_alt_cmd(k)    = gd.alt_cmd;
    log_V_cmd(k)      = gd.V_cmd;

    % ==================================================================
    %  NDI/INDI AUTOPILOT CONTROLLER
    % ==================================================================
    [de_L_cmd, de_R_cmd, dr_cmd, dc_cmd, ctrl_state, cd] = ...
        ndi_autopilot_controller( ...
            phi_cmd, theta_cmd, ...
            u_b, v_b, w_b, p_m, q_m, r_m, phi, theta, psi, ...
            ax_m, ay_m, az_m, ...
            omega_dot_filt(1), omega_dot_filt(2), omega_dot_filt(3), ...
            act_de_L, act_de_R, act_dr, act_dc, ...
            ctrl_state, aircraft, fcs, dt);

    log_omega_cmd(k,:) = cd.omega_cmd';

    % ==================================================================
    %  ACTUATOR DYNAMICS
    % ==================================================================
    act_de_L = actuator_step(de_L_cmd, act_de_L, aircraft.actuator.tau, ...
        aircraft.actuator.rate_max_de, aircraft.de_max, dt);
    act_de_R = actuator_step(de_R_cmd, act_de_R, aircraft.actuator.tau, ...
        aircraft.actuator.rate_max_de, aircraft.de_max, dt);
    act_dr = actuator_step(dr_cmd, act_dr, aircraft.actuator.tau, ...
        aircraft.actuator.rate_max_dr, aircraft.dr_max, dt);
    act_dc = actuator_step(dc_cmd, act_dc, aircraft.actuator.tau, ...
        aircraft.actuator.rate_max_dc, aircraft.dc_max, dt);

    % ==================================================================
    %  6-DOF PLANT (RK4)
    % ==================================================================
    k1 = fighter_6dof(x, act_de_L, act_de_R, act_dr, act_dc, thr_cmd, aircraft);
    k2 = fighter_6dof(x+0.5*dt*k1, act_de_L, act_de_R, act_dr, act_dc, thr_cmd, aircraft);
    k3 = fighter_6dof(x+0.5*dt*k2, act_de_L, act_de_R, act_dr, act_dc, thr_cmd, aircraft);
    k4 = fighter_6dof(x+dt*k3, act_de_L, act_de_R, act_dr, act_dc, thr_cmd, aircraft);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    x(10) = wrapToPi(x(10));
    x(11) = wrapToPi(x(11));
    x(12) = wrapToPi(x(12));

    % Ground collision
    if x(9) > 0, x(9) = 0; end

    % ---- Progress reporting ----
    if t(k) - last_print_time >= 30
        fprintf('  t=%5.0fs | WP%d (lap %d) | Alt=%.0fm | V=%.0fm/s | M=%.2f | XTE=%.0fm\n', ...
            t(k), gd.wp_idx-1, gd.lap_count, alt, V, Mach_now, gd.crosstrack_error);
        last_print_time = t(k);
    end
end

sim_time = toc;
fprintf('\nSimulation complete: %.1fs (%.1fx real-time)\n\n', ...
    sim_time, t_end/sim_time);

%% ====================================================================
%  STEP 5: COMPREHENSIVE PLOTS
%  ====================================================================

fprintf('Generating analysis plots...\n');

% ---- Figure 1: Ground Track with Waypoints ----
figure('Name','Ground Track','Position',[50 50 900 700],'Color','w');

% Plot NED ground track
subplot(2,2,[1,3]);
plot(log_state(:,8)/1000, log_state(:,7)/1000, 'b', 'LineWidth', 1.2); hold on;
plot(mission.wp_ned(:,2)/1000, mission.wp_ned(:,1)/1000, 'rs-', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'r', 'LineWidth', 1.5);
for i = 1:mission.n_waypoints
    text(mission.wp_ned(i,2)/1000 + 0.2, mission.wp_ned(i,1)/1000 + 0.2, ...
        sprintf('WP%d', i-1), 'FontSize', 8, 'Color', 'r');
end
plot(log_state(1,8)/1000, log_state(1,7)/1000, 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(log_state(end,8)/1000, log_state(end,7)/1000, 'kx', 'MarkerSize', 12, 'LineWidth', 2);
xlabel('East (km)'); ylabel('North (km)');
title('Ground Track (NED) with Waypoints');
grid on; axis equal; legend('Flight path', 'Waypoints', 'Start', 'End');

% Plot in lat/lon
subplot(2,2,2);
plot(log_lla(:,2), log_lla(:,1), 'b', 'LineWidth', 1); hold on;
plot(mission.wp_lon, mission.wp_lat, 'rs-', 'MarkerFaceColor', 'r');
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
title('Ground Track (Lat/Lon)');
grid on; axis equal;

% Altitude profile
subplot(2,2,4);
plot(t, log_alt, 'b', 'LineWidth', 1.2); hold on;
plot(t, log_alt_cmd, 'r--', 'LineWidth', 1);
ylabel('Altitude (m)'); xlabel('Time (s)');
title('Altitude Profile'); legend('Actual', 'Commanded');
grid on;

sgtitle('UCAV Mission — Ground Track and Altitude', 'FontSize', 14);

% ---- Figure 2: Navigation Performance ----
figure('Name','Nav Performance','Position',[100 100 1000 700],'Color','w');

subplot(3,2,1);
plot(t, log_crosstrack, 'b', 'LineWidth', 1);
ylabel('XTE (m)'); title('Cross-Track Error');
grid on;

subplot(3,2,2);
plot(t, log_dist_wp, 'b', 'LineWidth', 1);
ylabel('Distance (m)'); title('Distance to Next Waypoint');
grid on;

subplot(3,2,3);
plot(t, log_V, 'b', 'LineWidth', 1.2); hold on;
plot(t, log_V_cmd, 'r--', 'LineWidth', 1);
ylabel('V (m/s)'); title('Airspeed Tracking');
legend('Actual', 'Commanded'); grid on;

subplot(3,2,4);
plot(t, log_wp_idx - 1, 'b', 'LineWidth', 1.5);
ylabel('WP Index'); title('Active Waypoint');
grid on; ylim([-0.5, mission.n_waypoints+0.5]);

subplot(3,2,5);
plot(t, log_alt, 'b', 'LineWidth', 1.2); hold on;
plot(t, log_alt_cmd, 'r--', 'LineWidth', 1);
ylabel('Alt (m)'); xlabel('Time (s)');
title('Altitude Tracking'); legend('Actual', 'Cmd'); grid on;

subplot(3,2,6);
plot(t, log_Mach, 'b', 'LineWidth', 1);
ylabel('Mach'); xlabel('Time (s)'); title('Mach Number');
grid on;

sgtitle('Navigation Performance', 'FontSize', 14);

% ---- Figure 3: Attitude and Control ----
figure('Name','Attitude & Control','Position',[150 150 1000 700],'Color','w');

subplot(3,2,1);
plot(t, rad2deg(log_state(:,10)), 'b', 'LineWidth', 1); hold on;
plot(t, rad2deg(log_phi_cmd), 'r--', 'LineWidth', 1);
ylabel('Phi (deg)'); title('Roll Angle'); legend('Actual','Cmd'); grid on;

subplot(3,2,2);
plot(t, rad2deg(log_state(:,11)), 'b', 'LineWidth', 1); hold on;
plot(t, rad2deg(log_theta_cmd), 'r--', 'LineWidth', 1);
ylabel('Theta (deg)'); title('Pitch Angle'); legend('Actual','Cmd'); grid on;

subplot(3,2,3);
plot(t, log_alpha, 'b', 'LineWidth', 1);
ylabel('Alpha (deg)'); title('Angle of Attack'); grid on;

subplot(3,2,4);
plot(t, log_beta, 'b', 'LineWidth', 1);
ylabel('Beta (deg)'); title('Sideslip'); grid on;

subplot(3,2,5);
plot(t, rad2deg(log_surfaces(:,1)), 'b', 'LineWidth', 0.8); hold on;
plot(t, rad2deg(log_surfaces(:,2)), 'r', 'LineWidth', 0.8);
ylabel('Deflection (deg)'); xlabel('Time (s)');
title('Elevons'); legend('Left','Right'); grid on;

subplot(3,2,6);
plot(t, log_thr_cmd, 'b', 'LineWidth', 1);
ylabel('Throttle'); xlabel('Time (s)');
title('Throttle Command'); grid on; ylim([0 1.6]);

sgtitle('Attitude and Control Activity', 'FontSize', 14);

% ---- Figure 4: 3D Trajectory ----
figure('Name','3D Trajectory','Position',[200 200 800 600],'Color','w');
plot3(log_state(:,8)/1000, log_state(:,7)/1000, log_alt/1000, ...
    'b', 'LineWidth', 1.5);
hold on;
plot3(mission.wp_ned(:,2)/1000, mission.wp_ned(:,1)/1000, ...
    mission.wp_alt/1000, 'rs-', 'MarkerSize', 10, 'MarkerFaceColor', 'r', ...
    'LineWidth', 2);
xlabel('East (km)'); ylabel('North (km)'); zlabel('Altitude (km)');
title('3D Mission Trajectory'); grid on; view(-40, 25);

%% ====================================================================
%  STEP 6: MISSION PERFORMANCE SUMMARY
%  ====================================================================

fprintf('\n==========================================================\n');
fprintf(' MISSION PERFORMANCE SUMMARY\n');
fprintf('==========================================================\n\n');
fprintf('  Final waypoint reached:  WP%d of WP%d\n', ...
    max(log_wp_idx)-1, mission.n_waypoints-1);
fprintf('  Racetrack laps completed: %d of %d\n', ...
    guid_state.lap_count, mission.racetrack_laps);
fprintf('  Duration:    %.0f s (%.1f min)\n', t_end, t_end/60);
fprintf('\n');
fprintf('  Cross-track error:\n');
fprintf('    RMS:  %.0f m\n', sqrt(mean(log_crosstrack.^2)));
fprintf('    Max:  %.0f m\n', max(abs(log_crosstrack)));
fprintf('\n');
fprintf('  Altitude tracking:\n');
fprintf('    RMS error: %.0f m\n', sqrt(mean((log_alt - log_alt_cmd).^2)));
fprintf('    Max error: %.0f m\n', max(abs(log_alt - log_alt_cmd)));
fprintf('\n');
fprintf('  Speed tracking:\n');
fprintf('    RMS error: %.1f m/s\n', sqrt(mean((log_V - log_V_cmd).^2)));
fprintf('\n');
fprintf('  Max alpha: %.1f deg (limit: %d deg)\n', max(log_alpha), fcs.alpha_max_deg);
fprintf('  Max |beta|: %.1f deg\n', max(abs(log_beta)));
fprintf('  Max |bank|: %.0f deg\n', max(abs(rad2deg(log_state(:,10)))));
fprintf('  Mach range: %.2f — %.2f\n', min(log_Mach), max(log_Mach));
fprintf('  Alt range:  %.0f — %.0f m\n', min(log_alt), max(log_alt));
fprintf('\n==========================================================\n');

%% ====================================================================
%  LOCAL FUNCTIONS
%  ====================================================================

function xdot = fighter_6dof(x, de_L, de_R, dr, dc, thr, ac)
    g=9.81;
    u_b=x(1);v_b=x(2);w_b=x(3);p=x(4);q=x(5);r=x(6);
    phi=x(10);theta=x(11);psi=x(12);
    alt=max(0,-x(9));
    V=max(sqrt(u_b^2+v_b^2+w_b^2),5);
    alpha=atan2(w_b,u_b); beta=asin(max(min(v_b/V,1),-1));
    [~,a_s,~,rho]=atmos_local(alt); Mach=V/a_s; qbar=0.5*rho*V^2;
    [CL,CD,CY,Cl_c,Cm_c,Cn_c]=fighter_aero_model(alpha,beta,p,q,r,V,Mach,alt,de_L,de_R,dr,dc,ac);
    La=qbar*ac.S*CL; Da=qbar*ac.S*CD; Ya=qbar*ac.S*CY;
    Fxa=-Da*cos(alpha)+La*sin(alpha); Fya=Ya; Fza=-Da*sin(alpha)-La*cos(alpha);
    Lm=qbar*ac.S*ac.b*Cl_c; Mm=qbar*ac.S*ac.c_bar*Cm_c; Nm=qbar*ac.S*ac.b*Cn_c;
    if thr<=1,T=ac.engine.idle_thrust+(ac.engine.thrust_max_dry-ac.engine.idle_thrust)*thr;
    else,T=ac.engine.thrust_max_dry+(ac.engine.thrust_max_ab-ac.engine.thrust_max_dry)*(thr-1);end
    Fxg=-ac.mass*g*sin(theta); Fyg=ac.mass*g*cos(theta)*sin(phi); Fzg=ac.mass*g*cos(theta)*cos(phi);
    Fx=Fxa+T+Fxg; Fy=Fya+Fyg; Fz=Fza+Fzg;
    udot=Fx/ac.mass+r*v_b-q*w_b; vdot=Fy/ac.mass+p*w_b-r*u_b; wdot=Fz/ac.mass+q*u_b-p*v_b;
    Gam=ac.Ixx*ac.Izz-ac.Ixz^2;
    pdot=(ac.Izz*Lm+ac.Ixz*Nm-(ac.Ixz*(ac.Ixx-ac.Iyy+ac.Izz))*p*q+(ac.Ixz^2+ac.Izz*(ac.Izz-ac.Iyy))*q*r)/Gam;
    qdot=(Mm+(ac.Ixx-ac.Izz)*p*r-ac.Ixz*(p^2-r^2))/ac.Iyy;
    rdot=(ac.Ixx*Nm+ac.Ixz*Lm+(ac.Ixz*(ac.Iyy-ac.Izz-ac.Ixx))*q*r+(ac.Ixx*(ac.Ixx-ac.Iyy)+ac.Ixz^2)*p*q)/Gam;
    ct=cos(theta); if abs(ct)<0.01,ct=sign(ct)*0.01;end
    phidot=p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
    thetadot=q*cos(phi)-r*sin(phi);
    psidot=(q*sin(phi)+r*cos(phi))/ct;
    cp=cos(phi);sp=sin(phi);cth=cos(theta);sth=sin(theta);cps=cos(psi);sps=sin(psi);
    xd=u_b*cth*cps+v_b*(sp*sth*cps-cp*sps)+w_b*(cp*sth*cps+sp*sps);
    yd=u_b*cth*sps+v_b*(sp*sth*sps+cp*cps)+w_b*(cp*sth*sps-sp*cps);
    zd=-u_b*sth+v_b*sp*cth+w_b*cp*cth;
    xdot=[udot;vdot;wdot;pdot;qdot;rdot;xd;yd;zd;phidot;thetadot;psidot];
end

function pos_new = actuator_step(cmd, pos, tau, rate_max, pos_max, dt)
    pos_des = pos + (dt/(tau+dt))*(cmd-pos);
    rate = (pos_des-pos)/dt;
    if abs(rate)>rate_max, rate=sign(rate)*rate_max; pos_des=pos+rate*dt; end
    pos_new = max(min(pos_des,pos_max),-pos_max);
end

function [T,a,P,rho] = atmos_local(alt)
    T0=288.15;P0=101325;L=0.0065;R=287.05;g0=9.81;
    alt=max(alt,0); T=max(T0-L*alt,216.65);
    P=P0*(T/T0)^(g0/(R*L)); rho=P/(R*T); a=sqrt(1.4*R*T);
end
