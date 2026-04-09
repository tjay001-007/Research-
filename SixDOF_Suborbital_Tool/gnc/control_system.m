function [controls, gnc_state_out] = control_system(t, x, x_nav, cmd, vehicle, aero_db, gain_table, gnc_state_in, sim_cfg, dt)
%CONTROL_SYSTEM  Cascaded GNC controller for suborbital winged rocket.
%
%   Three-loop architecture:
%
%   Loop 1 — Attitude error → Rate commands (NDI outer loop)
%     Reuses ndi_flight_controller.m outer-loop pattern:
%       omega_cmd = T_inv * [K_phi*phi_err; K_theta*theta_err; K_psi*psi_err]
%
%   Loop 2 — Rate error → Moment demand (LQR inner loop)
%     Uses gain-scheduled LQR gains from gain_table.
%     Interpolated at current Mach and qbar.
%     M_demand = -K_lqr * [attitude_err(3); rate_err(3)]
%
%   Loop 3 — Moment → Effectors (Control Allocation)
%     Build combined effectiveness matrix B_total [3 x n_eff]
%     Allocate: effectors = pinv(B_total) * M_demand
%     Priority: TVC > surfaces > RCS
%
%   All patterns (clamp, wrap_angle, B\rhs, anti-windup) match
%   the existing ndi_flight_controller.m codebase conventions.
%
%   Inputs:
%     t, x       — Time and full state vector [14x1]
%     x_nav      — Navigated state (from EKF or = x if nav off)
%     cmd        — Guidance commands: .theta_cmd, .phi_cmd, .psi_cmd
%     vehicle    — Vehicle config
%     aero_db    — Aero database (for B-matrix)
%     gain_table — LQR gain table from lqr_gain_schedule
%     gnc_state_in — Previous GNC state (integrators, TVC state, etc.)
%     sim_cfg    — Sim config (tvc_on, rcs_on flags)
%     dt         — Controller timestep (s)
%
%   Outputs:
%     controls       — Controls struct
%     gnc_state_out  — Updated GNC state

%% ========================================================================
%  EXTRACT NAVIGATED STATE
%  ========================================================================

u_nav  = x_nav(4);   v_nav  = x_nav(5);   w_nav  = x_nav(6);
q0n    = x_nav(7);   q1n    = x_nav(8);   q2n    = x_nav(9);   q3n = x_nav(10);
p_nav  = x_nav(11);  qr_nav = x_nav(12);  r_nav  = x_nav(13);
m_nav  = x_nav(14);

V_nav = max(sqrt(u_nav^2 + v_nav^2 + w_nav^2), 1.0);

% Euler angles from quaternion
phi_nav   = atan2(2*(q0n*q1n + q2n*q3n), 1 - 2*(q1n^2 + q2n^2));
theta_nav = asin(clamp(2*(q0n*q2n - q3n*q1n), -1, 1));
psi_nav   = atan2(2*(q0n*q3n + q1n*q2n), 1 - 2*(q2n^2 + q3n^2));

% Atmosphere at navigated altitude
alt_nav = -x_nav(3);
alt_nav_clamped = max(0, min(86000, alt_nav));
[~, a_nav, ~, rho_nav] = atmosisa(alt_nav_clamped);
Mach_nav = V_nav / a_nav;
qbar_nav = 0.5 * rho_nav * V_nav^2;

% Aerodynamic angles
alpha_nav = atan2(w_nav, u_nav);

%% ========================================================================
%  ATTITUDE ERROR (outer loop)
%  ========================================================================

phi_err   = wrap_angle(cmd.phi_cmd   - phi_nav);
theta_err = wrap_angle(cmd.theta_cmd - theta_nav);
psi_err   = wrap_angle(cmd.psi_cmd   - psi_nav);

%% ========================================================================
%  LOOP 1 — OUTER LOOP: attitude → rate commands (NDI kinematic inversion)
%  (Reuses ndi_flight_controller.m outer-loop pattern exactly)
%  ========================================================================

g0 = 9.80665;

% Outer-loop bandwidths (gain-scheduled with qbar)
qbar_scale = min(max(qbar_nav / 5000, 0.3), 2.0);   % Normalised around 5 kPa
K_phi   = 3.0 / qbar_scale;
K_theta = 4.5 / qbar_scale;   % Higher for pitch instability
K_psi   = 2.0 / qbar_scale;

phi_dot_des   = K_phi   * phi_err;
theta_dot_des = K_theta * theta_err;

% Coordinated turn (psi tracking + beta suppression)
% Use navigated alpha/beta for beta damping
beta_nav = asin(clamp(v_nav / V_nav, -1, 1));
psi_dot_coord = g0 * tan(phi_nav) / V_nav;
psi_dot_des   = psi_dot_coord + K_psi * psi_err - 2.0 * beta_nav;

euler_dot_des = [phi_dot_des; theta_dot_des; psi_dot_des];

% Kinematic inversion T^{-1} (identical to ndi_flight_controller.m)
cos_theta = cos(theta_nav);
if abs(cos_theta) < 0.01
    cos_theta = sign(cos_theta) * 0.01;
end

T_inv = [1,  0,          -sin(theta_nav);
         0,  cos(phi_nav), sin(phi_nav)*cos_theta;
         0, -sin(phi_nav), cos(phi_nav)*cos_theta];

omega_cmd = T_inv * euler_dot_des;

% Rate limits
p_max  = deg2rad(30);   % More generous for rocket
qr_max = deg2rad(20);
r_max  = deg2rad(15);
omega_cmd(1) = clamp(omega_cmd(1), -p_max,  p_max);
omega_cmd(2) = clamp(omega_cmd(2), -qr_max, qr_max);
omega_cmd(3) = clamp(omega_cmd(3), -r_max,  r_max);

%% ========================================================================
%  LOOP 2 — LQR INNER LOOP: rate error → moment demand
%  ========================================================================

omega_nav = [p_nav; qr_nav; r_nav];
omega_err = omega_cmd - omega_nav;

% Build 6-state error vector for LQR
att_err   = [phi_err; theta_err; psi_err];
state_err = [att_err; omega_err];

% Interpolate LQR gain at current flight condition
K_lqr = interpolate_lqr_gain(Mach_nav, qbar_nav, gain_table);

% Moment demand: M_demand = -K_lqr * state_err
M_demand = -K_lqr * state_err;   % [L_demand; M_pitch_demand; N_demand] (Nm)

% Integral action for steady-state error rejection (anti-windup, like ndi_flight_controller.m)
int_state = gnc_state_in.int_state + omega_err * dt;
int_state(1) = clamp(int_state(1), -0.5, 0.5);
int_state(2) = clamp(int_state(2), -0.5, 0.5);
int_state(3) = clamp(int_state(3), -0.5, 0.5);

Ki_vec = [2.0; 4.0; 1.5];   % [roll; pitch; yaw] integral gains (higher pitch)
M_demand = M_demand + diag(Ki_vec) * (ones(3,1) .* int_state);

%% ========================================================================
%  LOOP 3 — CONTROL ALLOCATION: moments → effectors
%  Build combined B matrix: [B_tvc | B_surfaces | B_rcs_dummy]
%  ========================================================================

% Effectiveness of aerodynamic surfaces
Mach_c   = min(max(Mach_nav, aero_db.Mach_vec(1)), aero_db.Mach_vec(end));
alpha_c  = min(max(rad2deg(alpha_nav), aero_db.alpha_vec(1)), aero_db.alpha_vec(end));

Cmde = interp2(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cmde', Mach_c, alpha_c, 'linear');
Clda = interp2(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Clda', Mach_c, alpha_c, 'linear');
Cndr = interp2(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cndr', Mach_c, alpha_c, 'linear');
Cldr = interp2(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cldr', Mach_c, alpha_c, 'linear');
Cnda = interp2(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cnda', Mach_c, alpha_c, 'linear');

S = vehicle.S;  b = vehicle.b;  c = vehicle.c;

% Compute de/da/dr in rad (from normalised commands)
de_scale = vehicle.de_to_rad;
da_scale = vehicle.da_to_rad;
dr_scale = vehicle.dr_to_rad;

% Effectiveness matrix B_surf: maps [da; de; dr] (rad) to [L; M; N] (Nm)
B_surf = [qbar_nav*S*b*Clda,         0,                         qbar_nav*S*b*Cldr;
          0,                          qbar_nav*S*c*Cmde,         0;
          qbar_nav*S*b*Cnda,         0,                         qbar_nav*S*b*Cndr];

% Scale to normalised [-1,1] inputs
B_surf_norm = B_surf .* [da_scale, de_scale, dr_scale];

% TVC effectiveness: [L_tvc; M_tvc; N_tvc] per [dp_rad; dy_rad]
if sim_cfg.tvc_on
    T1_est = gnc_state_in.T1_current;   % Estimated from propulsion state
    delta_m  = vehicle.mass_total - m_nav;
    cg_shift = vehicle.CG_shift_per_kg_prop1 * min(delta_m, vehicle.mass_prop1) + ...
               vehicle.CG_shift_per_kg_prop2 * max(0, delta_m - vehicle.mass_prop1);
    CG_current = vehicle.CG_initial + cg_shift;
    l_tvc      = max(0.5, vehicle.L - CG_current);

    % Approximate: M_tvc = l_tvc*T1*dp,  N_tvc = -l_tvc*T1*dy
    B_tvc = [0,                    0;
             l_tvc * T1_est,       0;
             0,                    -l_tvc * T1_est];
else
    B_tvc = zeros(3,2);
end

% Combined B matrix [3 x 5]: [da; de; dr; dp_tvc; dy_tvc]
B_total = [B_surf_norm, B_tvc];

% Condition check (from ndi_flight_controller.m pattern)
det_Bs = det(B_surf_norm);
if abs(det_Bs) < 1e-8 || qbar_nav < 200
    % Low airspeed or degenerate: use pseudo-inverse
    eff_cmd = pinv(B_total) * M_demand;
else
    % Normal allocation via pseudo-inverse (least-squares, no priority)
    eff_cmd = pinv(B_total) * M_demand;
end

% Distribute: [da_norm; de_norm; dr_norm; dp_tvc_rad; dy_tvc_rad]
da_norm = clamp(eff_cmd(1), -1, 1);
de_norm = clamp(eff_cmd(2), -1, 1);
dr_norm = clamp(eff_cmd(3), -1, 1);
dp_tvc  = clamp(eff_cmd(4), -deg2rad(vehicle.tvc_max), deg2rad(vehicle.tvc_max));
dy_tvc  = clamp(eff_cmd(5), -deg2rad(vehicle.tvc_max), deg2rad(vehicle.tvc_max));

% Anti-windup: freeze integrator if saturated (ndi_flight_controller.m pattern)
if any(abs([da_norm; de_norm; dr_norm]) >= 0.99)
    int_state = gnc_state_in.int_state;  % Hold previous
end

%% ========================================================================
%  RCS ALLOCATION (residual moments that surfaces/TVC can't deliver)
%  ========================================================================

if sim_cfg.rcs_on
    % Estimate achieved moment from surfaces + TVC
    M_achieved = B_surf_norm * [da_norm; de_norm; dr_norm] + B_tvc * [dp_tvc; dy_tvc];
    M_residual = M_demand - M_achieved;
    rcs_cmd = M_residual;
else
    rcs_cmd = zeros(3,1);
end

%% ========================================================================
%  THROTTLE (open-loop for now — guidance manages pitch, not throttle)
%  ========================================================================

throttle1 = clamp(gnc_state_in.throttle1_cmd, 0.4, 1.0);
throttle2 = clamp(gnc_state_in.throttle2_cmd, 0.6, 1.0);

%% ========================================================================
%  PACK OUTPUTS
%  ========================================================================

controls.delta_e   = de_norm;
controls.delta_a   = da_norm;
controls.delta_r   = dr_norm;
controls.tvc_pitch = dp_tvc;
controls.tvc_yaw   = dy_tvc;
controls.throttle1 = throttle1;
controls.throttle2 = throttle2;
controls.rcs_cmd   = rcs_cmd;

% Debug / authority monitoring fields
controls.M_demand     = M_demand;
controls.M_achieved   = B_surf_norm*[da_norm;de_norm;dr_norm] + B_tvc*[dp_tvc;dy_tvc];
controls.B_surf_norm  = B_surf_norm;
controls.cond_B       = cond(B_surf_norm);
controls.omega_cmd    = omega_cmd;
controls.omega_err    = omega_err;
controls.att_err      = att_err;
controls.Mach         = Mach_nav;
controls.qbar         = qbar_nav;
controls.alpha_deg    = rad2deg(alpha_nav);

% Update GNC state
gnc_state_out               = gnc_state_in;
gnc_state_out.int_state     = int_state;
gnc_state_out.tvc_state     = [dp_tvc; dy_tvc];

end

%% ========================================================================
%  HELPER: Interpolate LQR gain from table
%  ========================================================================

function K = interpolate_lqr_gain(Mach, qbar, gain_table)

Mg = gain_table.Mach_grid;
Qg = gain_table.qbar_grid;
K_arr = gain_table.K_table;   % [3, 6, nM, nQ]

% Clamp to grid
Mc = min(max(Mach, Mg(1)), Mg(end));
Qc = min(max(qbar, Qg(1)), Qg(end));

% Find bounding indices
im1 = find(Mg <= Mc, 1, 'last');
im2 = min(im1 + 1, length(Mg));
iq1 = find(Qg <= Qc, 1, 'last');
iq2 = min(iq1 + 1, length(Qg));

% Bilinear interpolation weights
if Mg(im2) > Mg(im1)
    wm = (Mc - Mg(im1)) / (Mg(im2) - Mg(im1));
else
    wm = 0;
end
if Qg(iq2) > Qg(iq1)
    wq = (Qc - Qg(iq1)) / (Qg(iq2) - Qg(iq1));
else
    wq = 0;
end

K00 = K_arr(:,:,im1,iq1);
K10 = K_arr(:,:,im2,iq1);
K01 = K_arr(:,:,im1,iq2);
K11 = K_arr(:,:,im2,iq2);

K = (1-wm)*(1-wq)*K00 + wm*(1-wq)*K10 + (1-wm)*wq*K01 + wm*wq*K11;

end

%% ========================================================================
%  LOCAL HELPERS (mirror ndi_flight_controller.m)
%  ========================================================================

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function da = wrap_angle(a)
    da = mod(a + pi, 2*pi) - pi;
end
