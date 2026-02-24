function [delta_a, delta_e, delta_r, int_state_out, debug] = ndi_flight_controller( ...
    phi_cmd, theta_cmd, ...                     % Attitude commands (rad)
    u, v, w, p, q, r, phi, theta, ...          % Aircraft states
    int_state_in, ...                           % Integrator states [3x1]
    aircraft, gains, dt)
%NDI_FLIGHT_CONTROLLER  Cascaded Nonlinear Dynamic Inversion controller
%   for a pitch-unstable fixed-wing aircraft.
%
%   Implements a two-loop NDI architecture:
%     Outer loop: Euler angle tracking -> body rate commands
%     Inner loop: Body rate tracking   -> control surface deflections
%
%   The inner loop algebraically cancels the bare-airframe aerodynamic
%   moments (including the destabilizing Cmalpha > 0) and replaces them
%   with designer-chosen first-order error dynamics augmented by integral
%   action for robustness to modelling errors.
%
%   Inputs:
%     phi_cmd, theta_cmd  - Commanded roll and pitch angles (rad)
%     u, v, w             - Body-axis velocities (m/s)
%     p, q, r             - Body-axis angular rates (rad/s)
%     phi, theta          - Euler angles (rad)
%     int_state_in        - Integrator states from previous step [3x1]
%     aircraft            - Struct of aerodynamic & mass properties
%     gains               - Struct of controller gains
%     dt                  - Controller time step (s)
%
%   Outputs:
%     delta_a             - Aileron command  [-1, 1]
%     delta_e             - Elevator command [-1, 1]
%     delta_r             - Rudder command   [-1, 1]
%     int_state_out       - Updated integrator states [3x1]
%     debug               - Struct with internal signals for monitoring
%
%   Reference:
%     Snell, Enns & Garrard, "Nonlinear Inversion Flight Control for a
%     Supermaneuverable Aircraft," AIAA J. Guidance, Vol. 15, No. 4, 1992.
%
%   Compatible with: aircraft_6dof_sfunc.m (state vector and aero model)
%
%   Author: NDI Controller for Pitch-Unstable Aircraft Research
%   Date:   2026-02-24

%% ========================================================================
%  AIRDATA COMPUTATION
%  ========================================================================

g   = 9.81;
rho = 1.225;

V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1.0);                       % Protect against zero airspeed

alpha = atan2(w, u);                    % Angle of attack (rad)
beta  = asin(clamp(v / V, -1, 1));     % Sideslip angle (rad)
qbar  = 0.5 * rho * V^2;              % Dynamic pressure (Pa)

% Non-dimensional angular rates (for aero coefficient lookup)
p_hat = p * aircraft.b / (2*V);
q_hat = q * aircraft.c / (2*V);
r_hat = r * aircraft.b / (2*V);

%% ========================================================================
%  OUTER LOOP — ATTITUDE NDI (Euler angle -> body rate commands)
%  ========================================================================
%
%  Kinematic relation:  euler_dot = T(phi,theta) * omega
%
%            [ 1   sin(phi)*tan(theta)   cos(phi)*tan(theta) ] [ p ]
%  euler_dot=[ 0   cos(phi)             -sin(phi)            ] [ q ]
%            [ 0   sin(phi)/cos(theta)   cos(phi)/cos(theta) ] [ r ]
%
%  Inverting: omega_cmd = T^{-1} * euler_dot_desired
%
%  We command phi and theta; for the third axis we command beta -> 0
%  (coordinated flight) which maps to a yaw rate constraint.

% Desired Euler-angle rates (proportional tracking)
phi_err   = wrap_angle(phi_cmd - phi);
theta_err = wrap_angle(theta_cmd - theta);

phi_dot_des   = gains.K_phi   * phi_err;
theta_dot_des = gains.K_theta * theta_err;

% Coordinated turn: command steady-state yaw rate for the current bank
% psi_dot_coordinated = g * tan(phi) / V   (derived from steady turn)
% We add a beta-damping term to drive sideslip to zero.
psi_dot_coord = g * tan(phi) / V;
psi_dot_des   = psi_dot_coord - gains.K_beta * beta;

euler_dot_des = [phi_dot_des; theta_dot_des; psi_dot_des];

% Inverse kinematic matrix T^{-1}
cos_theta = cos(theta);
if abs(cos_theta) < 0.01
    cos_theta = sign(cos_theta) * 0.01;  % Protect near gimbal lock
end

T_inv = [1,  0,         -sin(theta);
         0,  cos(phi),   sin(phi)*cos_theta;
         0, -sin(phi),   cos(phi)*cos_theta];

omega_cmd = T_inv * euler_dot_des;      % [p_cmd; q_cmd; r_cmd]

% Limit commanded rates to physical envelope
omega_cmd(1) = clamp(omega_cmd(1), -gains.p_max, gains.p_max);
omega_cmd(2) = clamp(omega_cmd(2), -gains.q_max, gains.q_max);
omega_cmd(3) = clamp(omega_cmd(3), -gains.r_max, gains.r_max);

%% ========================================================================
%  INNER LOOP — RATE NDI (body rate commands -> surface deflections)
%  ========================================================================
%
%  Rotational EOM (simplified diagonal inertia):
%
%    Ixx * pdot = L_aero + (Iyy - Izz)*q*r
%    Iyy * qdot = M_aero + (Izz - Ixx)*p*r
%    Izz * rdot = N_aero + (Ixx - Iyy)*p*q
%
%  Aerodynamic moments split into bare-airframe + control:
%
%    [L]   [L0]       [da]
%    [M] = [M0] + B * [de]
%    [N]   [N0]       [dr]
%
%  NDI inversion:
%    B * delta = J * omega_dot_des - M0 - gyro_coupling
%    delta = B^{-1} * (J * omega_dot_des - M0 - gyro_coupling)

% --- Bare-airframe moments M0 (everything NOT dependent on control surfaces)
L0 = qbar * aircraft.S * aircraft.b * ...
     (aircraft.Clbeta*beta + aircraft.Clp*p_hat + aircraft.Clr*r_hat);

M0 = qbar * aircraft.S * aircraft.c * ...
     (aircraft.Cm0 + aircraft.Cmalpha*alpha + aircraft.Cmq*q_hat);

N0 = qbar * aircraft.S * aircraft.b * ...
     (aircraft.Cnbeta*beta + aircraft.Cnr*r_hat);

M0_vec = [L0; M0; N0];

% --- Control effectiveness matrix B
%     Maps [da; de; dr] to [L_ctrl; M_ctrl; N_ctrl]
%
%  From the aero model:
%    L_ctrl = qbar*S*b * Clda * da
%    M_ctrl = qbar*S*c * Cmde * de
%    N_ctrl = qbar*S*b * (Cnda*da + Cndr*dr)
%
B = [qbar*aircraft.S*aircraft.b*aircraft.Clda,  0,                                  0;
     0,                                          qbar*aircraft.S*aircraft.c*aircraft.Cmde,  0;
     qbar*aircraft.S*aircraft.b*aircraft.Cnda,  0,                                  qbar*aircraft.S*aircraft.b*aircraft.Cndr];

% --- Gyroscopic cross-coupling terms
gyro = [(aircraft.Iyy - aircraft.Izz) * q * r;
        (aircraft.Izz - aircraft.Ixx) * p * r;
        (aircraft.Ixx - aircraft.Iyy) * p * q];

% --- Inertia matrix (diagonal — no products of inertia)
J = diag([aircraft.Ixx, aircraft.Iyy, aircraft.Izz]);

% --- Rate tracking error
omega     = [p; q; r];
omega_err = omega_cmd - omega;

% --- Integrator update (trapezoidal with anti-windup clamping)
int_state_out = int_state_in + omega_err * dt;
int_state_out(1) = clamp(int_state_out(1), -gains.int_lim_p, gains.int_lim_p);
int_state_out(2) = clamp(int_state_out(2), -gains.int_lim_q, gains.int_lim_q);
int_state_out(3) = clamp(int_state_out(3), -gains.int_lim_r, gains.int_lim_r);

% --- Desired angular acceleration (PI on rate error)
K_rate = diag([gains.K_p, gains.K_q, gains.K_r]);
K_int  = diag([gains.Ki_p, gains.Ki_q, gains.Ki_r]);

omega_dot_des = K_rate * omega_err + K_int * int_state_out;

% --- NDI inversion: solve B * delta = J * omega_dot_des - M0 - gyro
%
%     This is the key step: the right-hand side cancels the nonlinear
%     plant dynamics and injects the desired linear closed-loop response.
rhs = J * omega_dot_des - M0_vec - gyro;

% Check control effectiveness (condition number of B)
det_B = det(B);
if abs(det_B) < 1e-10
    % Near-singular B: fall back to pseudo-inverse (e.g., very low airspeed)
    delta = pinv(B) * rhs;
else
    delta = B \ rhs;
end

%% ========================================================================
%  OUTPUT SATURATION
%  ========================================================================

delta_a = clamp(delta(1), -1, 1);
delta_e = clamp(delta(2), -1, 1);
delta_r = clamp(delta(3), -1, 1);

% --- Anti-windup: freeze integrator if any surface is saturated
if abs(delta(1)) > 1 || abs(delta(2)) > 1 || abs(delta(3)) > 1
    int_state_out = int_state_in;   % Hold integrator at previous value
end

%% ========================================================================
%  DEBUG OUTPUT
%  ========================================================================

debug.alpha       = alpha;
debug.beta        = beta;
debug.V           = V;
debug.qbar        = qbar;
debug.omega_cmd   = omega_cmd;
debug.omega_err   = omega_err;
debug.omega_dot_des = omega_dot_des;
debug.M0_vec      = M0_vec;
debug.B           = B;
debug.gyro        = gyro;
debug.rhs         = rhs;
debug.delta_raw   = delta;
debug.det_B       = det_B;

end

%% ========================================================================
%  LOCAL HELPER FUNCTIONS
%  ========================================================================

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function da = wrap_angle(a)
    % Wrap angle to [-pi, pi]
    da = mod(a + pi, 2*pi) - pi;
end
