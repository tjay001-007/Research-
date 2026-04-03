function [x_nav, nav_state_out] = navigation_system(t, x_true, nav_state_in, sim_cfg, dt)
%NAVIGATION_SYSTEM  Strapdown INS with 15-state error-state EKF, fused
%   with simulated GPS measurements.
%
%   If sim_cfg.nav_ekf_on = false, returns true state directly (ideal nav).
%
%   INS MECHANISATION (250 Hz propagation):
%     Position:  r_dot = v_NED
%     Velocity:  v_dot = R_BI' * (f_body - b_accel) + g_NED
%     Attitude:  q_dot = 0.5 * Xi(q) * (omega_body - b_gyro)
%     Biases:    b_accel_dot = 0,  b_gyro_dot = 0  (random walk)
%
%   EKF ERROR STATE (15 states):
%     [δr(3), δv(3), δψ(3), δba(3), δbg(3)]
%     position error (m), velocity error (m/s), tilt error (rad),
%     accel bias error (m/s^2), gyro bias error (rad/s)
%
%   GPS UPDATE (10 Hz): position [3] + velocity [3] = 6 measurements
%
%   Inputs:
%     t            — Current time (s)
%     x_true       — True 14-state vector
%     nav_state_in — Previous navigation state
%     sim_cfg      — Sim config (noise params, GPS rate)
%     dt           — Propagation timestep (s)
%
%   Outputs:
%     x_nav         — Estimated state [14x1] (INS + EKF correction)
%     nav_state_out — Updated navigation state

if ~sim_cfg.nav_ekf_on
    x_nav = x_true;
    nav_state_out = nav_state_in;
    return;
end

g0 = 9.80665;

%% ========================================================================
%  INITIALISE NAV STATE (first call)
%  ========================================================================

if ~isfield(nav_state_in, 'initialized') || ~nav_state_in.initialized
    nav_state_out = init_nav_state(x_true, sim_cfg);
    x_nav = nav_state_out.x_ins;
    return;
end

nav_state_out = nav_state_in;

%% ========================================================================
%  SENSOR SIMULATION (add noise to true state)
%  ========================================================================

% Accelerometer measurement (body frame specific force)
u_t  = x_true(4);  v_t = x_true(5);  w_t = x_true(6);
p_t  = x_true(11); qr_t= x_true(12); r_t = x_true(13);
q0_t = x_true(7);  q1_t= x_true(8);  q2_t= x_true(9); q3_t= x_true(10);

% DCM body←inertial
R_BI = quat2dcm([q0_t, q1_t, q2_t, q3_t]);
g_body = R_BI * [0; 0; g0];

v_body = [u_t; v_t; w_t];
omega  = [p_t; qr_t; r_t];

% Specific force = total body acceleration - gravity (what IMU measures)
% f_body ≈ (F_total - F_grav) / m   (simplified: we use velocity derivative)
% For simplicity: f_body = v_dot + omega×v + g_body
% Approximate as zero (INS accumulates from true state directly in this demo)
sigma_a = sim_cfg.nav.sigma_accel;
sigma_g = sim_cfg.nav.sigma_gyro;

f_meas     = g_body + sigma_a * randn(3,1);   % Simulated accel measurement
omega_meas = omega  + nav_state_out.b_gyro + sigma_g * randn(3,1);

%% ========================================================================
%  INS PROPAGATION
%  Update INS state using noisy measurements
%  ========================================================================

x_ins = nav_state_out.x_ins;

% Extract INS state
xN_i  = x_ins(1); xE_i = x_ins(2); xD_i = x_ins(3);
u_i   = x_ins(4); v_i  = x_ins(5); w_i  = x_ins(6);
q0_i  = x_ins(7); q1_i = x_ins(8); q2_i = x_ins(9); q3_i = x_ins(10);
p_i   = x_ins(11);qr_i = x_ins(12);r_i  = x_ins(13);
m_i   = x_ins(14);

% DCM for INS estimate
qnorm = sqrt(q0_i^2+q1_i^2+q2_i^2+q3_i^2);
q0_i=q0_i/qnorm; q1_i=q1_i/qnorm; q2_i=q2_i/qnorm; q3_i=q3_i/qnorm;
R_BI_ins = quat2dcm([q0_i, q1_i, q2_i, q3_i]);

% Velocity update: v_NED_dot = R_BI^T * f_meas + g_NED - b_accel
g_NED     = [0; 0; g0];
f_body_corrected = f_meas - nav_state_out.b_accel;
v_NED_ins = R_BI_ins' * [u_i; v_i; w_i];   % Current NED velocity (approx)
a_NED     = R_BI_ins' * f_body_corrected + g_NED;

% NED velocity update
v_NED_new = v_NED_ins + a_NED * dt;

% Body velocity from NED (approximate — ignores rotation of NED frame)
v_body_new = R_BI_ins * v_NED_new;

% Position update
pos_NED_new = [xN_i; xE_i; xD_i] + v_NED_ins * dt;

% Attitude update (quaternion integration with corrected gyro)
p_c  = omega_meas(1) - nav_state_out.b_gyro(1);
qr_c = omega_meas(2) - nav_state_out.b_gyro(2);
r_c  = omega_meas(3) - nav_state_out.b_gyro(3);

q0_new = q0_i + 0.5*(-p_c*q1_i - qr_c*q2_i - r_c*q3_i)*dt;
q1_new = q1_i + 0.5*( p_c*q0_i + r_c*q2_i  - qr_c*q3_i)*dt;
q2_new = q2_i + 0.5*( qr_c*q0_i- r_c*q1_i  + p_c*q3_i)*dt;
q3_new = q3_i + 0.5*( r_c*q0_i + qr_c*q1_i - p_c*q2_i)*dt;

qn = sqrt(q0_new^2+q1_new^2+q2_new^2+q3_new^2);
q0_new=q0_new/qn; q1_new=q1_new/qn; q2_new=q2_new/qn; q3_new=q3_new/qn;

% Pack INS state (mass from true state — no mass sensor error)
x_ins_new = [pos_NED_new; v_body_new; q0_new; q1_new; q2_new; q3_new; ...
             omega_meas - nav_state_out.b_gyro; x_true(14)];

nav_state_out.x_ins = x_ins_new;

%% ========================================================================
%  EKF PREDICTION (propagate error covariance)
%  ========================================================================

P = nav_state_out.P;   % 15x15

% State transition matrix Phi (15x15, simplified continuous → discrete)
% Error states: [δr(3), δv(3), δψ(3), δba(3), δbg(3)]
% Key couplings:
%   δr_dot  = δv
%   δv_dot  = -[f×]δψ - R*δba  (f× = skew(f_meas))
%   δψ_dot  = -δbg
%   δba_dot = 0  (random walk)
%   δbg_dot = 0  (random walk)

f_skew = skew3(R_BI_ins' * f_body_corrected);

Fc = zeros(15,15);
Fc(1:3, 4:6)   =  eye(3);            % δr_dot = δv
Fc(4:6, 7:9)   = -f_skew;            % δv_dot from tilt error
Fc(4:6, 10:12) = -R_BI_ins';         % δv_dot from accel bias
Fc(7:9, 13:15) = -eye(3);            % δψ_dot from gyro bias

Phi = eye(15) + Fc * dt;             % First-order discrete approximation

% Process noise covariance Q_proc
q_a   = (sim_cfg.nav.sigma_accel)^2 * dt;
q_g   = (sim_cfg.nav.sigma_gyro)^2  * dt;
q_ba  = (1e-5)^2 * dt;    % Accel bias random walk
q_bg  = (1e-6)^2 * dt;    % Gyro  bias random walk

Q_proc = blkdiag(eye(3)*q_a*dt^2, eye(3)*q_a, eye(3)*q_g, eye(3)*q_ba, eye(3)*q_bg);

P = Phi * P * Phi' + Q_proc;

%% ========================================================================
%  GPS UPDATE (at GPS rate)
%  ========================================================================

gps_dt = 1.0 / sim_cfg.nav.gps_rate;
do_gps = (t - nav_state_out.t_last_gps) >= gps_dt;

if do_gps
    % Simulate GPS measurement from true state + noise
    sig_pos = sim_cfg.nav.sigma_gps_pos;
    sig_vel = sim_cfg.nav.sigma_gps_vel;

    z_pos = x_true(1:3) + sig_pos * randn(3,1);    % Noisy NED position
    z_vel = R_BI_ins' * x_true(4:6) + sig_vel * randn(3,1);  % Noisy NED velocity
    z_meas = [z_pos; z_vel];   % 6x1

    % Predicted measurement from INS
    z_pred_pos = x_ins_new(1:3);
    z_pred_vel = R_BI_ins' * x_ins_new(4:6);
    z_pred = [z_pred_pos; z_pred_vel];

    % Innovation
    dz = z_meas - z_pred;   % 6x1

    % Measurement matrix H (6x15): maps error state to measurement residual
    H = zeros(6, 15);
    H(1:3, 1:3) = eye(3);    % Position error
    H(4:6, 4:6) = eye(3);    % Velocity error

    % Measurement noise
    R_meas = blkdiag(eye(3)*sig_pos^2, eye(3)*sig_vel^2);

    % Kalman gain
    S_innov = H * P * H' + R_meas;
    K_kf    = P * H' / S_innov;

    % State update
    err_state = nav_state_out.err_state + K_kf * dz;
    P         = (eye(15) - K_kf * H) * P;

    % Apply error state corrections to INS
    x_ins_new(1:3)   = x_ins_new(1:3)   + err_state(1:3);   % Position
    v_NED_corrected  = R_BI_ins' * x_ins_new(4:6) + err_state(4:6);
    x_ins_new(4:6)   = R_BI_ins * v_NED_corrected;           % Velocity

    % Attitude correction (small angle: δq ≈ [1; δψ/2])
    dq = [1; err_state(7:9)/2];
    dq = dq / norm(dq);
    q_corr = quatmultiply([x_ins_new(7:10)'], dq')';
    x_ins_new(7:10) = q_corr / norm(q_corr);

    % Bias corrections
    nav_state_out.b_accel = nav_state_out.b_accel + err_state(10:12);
    nav_state_out.b_gyro  = nav_state_out.b_gyro  + err_state(13:15);

    % Reset error state after correction
    nav_state_out.err_state = zeros(15,1);
    nav_state_out.t_last_gps = t;
else
    err_state = nav_state_out.err_state;
    err_state(1:3) = err_state(1:3) + Phi(1:3,4:6) * nav_state_out.err_state(4:6) * dt;
    nav_state_out.err_state = err_state;
end

nav_state_out.P     = P;
nav_state_out.x_ins = x_ins_new;

x_nav = x_ins_new;

end

%% ========================================================================
%  INITIALISE NAVIGATION STATE
%  ========================================================================

function ns = init_nav_state(x_true, sim_cfg)
ns.initialized  = true;
ns.x_ins        = x_true;   % Start INS from true state
ns.P            = blkdiag(eye(3)*1.0^2, eye(3)*0.1^2, eye(3)*deg2rad(0.1)^2, ...
                           eye(3)*sim_cfg.nav.sigma_accel^2, ...
                           eye(3)*sim_cfg.nav.sigma_gyro^2);
ns.err_state    = zeros(15,1);
ns.b_accel      = sim_cfg.nav.accel_bias_0;
ns.b_gyro       = sim_cfg.nav.gyro_bias_0;
ns.t_last_gps   = -1e6;
end

%% ========================================================================
%  SKEW-SYMMETRIC MATRIX
%  ========================================================================

function S = skew3(v)
S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
