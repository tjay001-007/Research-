function xdot = eom_6dof(t, x, controls, vehicle, aero_db, prop_cfg, sim_cfg, h_ignite)
%EOM_6DOF  Full 6-degree-of-freedom equations of motion for a suborbital
%   winged rocket / lifting body using quaternion attitude representation.
%
%   STATE VECTOR (14 elements):
%     x(1)  = xN    — North position (m, NED frame)
%     x(2)  = xE    — East position  (m, NED frame)
%     x(3)  = xD    — Down position  (m, NED frame, negative = altitude)
%     x(4)  = u     — Body-frame forward velocity (m/s)
%     x(5)  = v     — Body-frame lateral velocity (m/s)
%     x(6)  = w     — Body-frame normal velocity  (m/s)
%     x(7)  = q0    — Quaternion scalar part
%     x(8)  = q1    — Quaternion vector part i
%     x(9)  = q2    — Quaternion vector part j
%     x(10) = q3    — Quaternion vector part k
%     x(11) = p     — Roll rate  (rad/s, body frame)
%     x(12) = qr    — Pitch rate (rad/s, body frame, 'qr' avoids clash with q)
%     x(13) = r     — Yaw rate   (rad/s, body frame)
%     x(14) = m     — Total vehicle mass (kg), includes remaining propellant
%
%   CONTROLS STRUCT:
%     controls.delta_e    — Elevator normalised [-1, 1]
%     controls.delta_a    — Aileron  normalised [-1, 1]
%     controls.delta_r    — Rudder   normalised [-1, 1]
%     controls.tvc_pitch  — TVC pitch gimbal (rad)
%     controls.tvc_yaw    — TVC yaw gimbal   (rad)
%     controls.throttle1  — Engine 1 throttle [0.4, 1.0]
%     controls.throttle2  — Engine 2 throttle [0.6, 1.0]
%     controls.rcs_cmd    — RCS moment command [L; M; N] (Nm), from rcs_model
%
%   Coordinate frames:
%     NED (North-East-Down): inertial navigation frame
%     Body: x-forward, y-right, z-down
%
%   Quaternion convention: q = [q0; q1; q2; q3], scalar-first
%     Rotation from body to inertial: R_BI = quat2dcm([q0,q1,q2,q3])'
%     (Aerospace Toolbox quat2dcm gives body←inertial; transpose for inertial←body)
%
%   Author: 6DOF Suborbital Trajectory Tool
%   Reference: Stevens & Lewis, "Aircraft Simulation and Control", 2003.

%% ========================================================================
%  EXTRACT STATE
%  ========================================================================

xN  = x(1);   xE  = x(2);   xD  = x(3);
u   = x(4);   v   = x(5);   w   = x(6);
q0  = x(7);   q1  = x(8);   q2  = x(9);  q3 = x(10);
p   = x(11);  qr  = x(12);  r   = x(13);
m   = x(14);

%% ========================================================================
%  DERIVED QUANTITIES
%  ========================================================================

g0  = 9.80665;
altitude = -xD;                    % Altitude above ground (m), z_NED is down

% --- Atmosphere (Aerospace Toolbox atmosisa) ---
alt_clamped = max(0, min(86000, altitude));   % atmosisa valid 0-86 km
[T_atm, a_sound, Pa, rho] = atmosisa(alt_clamped);

% --- Airspeed ---
V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1.0);        % Protect against near-zero airspeed
Mach = V / a_sound;

% --- Aerodynamic angles ---
alpha = atan2(w, u);                              % Angle of attack (rad)
beta  = asin(clamp_scalar(v/V, -1.0, 1.0));      % Sideslip angle (rad)
qbar  = 0.5 * rho * V^2;                         % Dynamic pressure (Pa)

% --- Quaternion → DCM (body ← inertial using Aerospace Toolbox) ---
% Normalise quaternion defensively
q_norm = sqrt(q0^2 + q1^2 + q2^2 + q3^2);
if q_norm < 1e-10
    q_norm = 1.0;
end
q0n = q0/q_norm;  q1n = q1/q_norm;  q2n = q2/q_norm;  q3n = q3/q_norm;

% Rotation matrix body ← inertial (R_BI):
%   row = body axis,  col = NED axis
R_BI = quat2dcm([q0n, q1n, q2n, q3n]);   % Aerospace Toolbox

% --- Euler angles from quaternion (for control loops and logging) ---
phi   = atan2(2*(q0n*q1n + q2n*q3n), 1 - 2*(q1n^2 + q2n^2));
theta = asin(clamp_scalar(2*(q0n*q2n - q3n*q1n), -1, 1));
% psi not needed in EOM, extracted by control_system as needed

%% ========================================================================
%  INERTIA TENSOR (varies with mass as propellant burns)
%  ========================================================================

delta_m = vehicle.mass_total - m;   % Propellant burned so far (kg), always >= 0

Ixx = max(100, vehicle.Ixx + vehicle.dIxx_dm * delta_m);
Iyy = max(500, vehicle.Iyy + vehicle.dIyy_dm * delta_m);
Izz = max(500, vehicle.Izz + vehicle.dIzz_dm * delta_m);
Ixz = vehicle.Ixz + vehicle.dIxz_dm * delta_m;

% Full inertia tensor (symmetric, with Ixz cross-term)
I_mat = [Ixx, 0,  -Ixz;
         0,  Iyy,  0;
        -Ixz, 0,   Izz];

%% ========================================================================
%  AERODYNAMIC FORCES & MOMENTS
%  ========================================================================

if sim_cfg.aero_on
    atm.rho = rho;  atm.a = a_sound;  atm.Pa = Pa;
    [F_aero, M_aero] = aero_forces_moments(x, controls, vehicle, aero_db, atm);
else
    F_aero = zeros(3,1);
    M_aero = zeros(3,1);
end

%% ========================================================================
%  PROPULSION FORCES & MOMENTS
%  ========================================================================

atm.rho = rho;  atm.a = a_sound;  atm.Pa = Pa;
[F_prop, M_prop, mdot_total] = propulsion_model(x, controls, vehicle, prop_cfg, atm, sim_cfg, h_ignite);

%% ========================================================================
%  GRAVITY (NED frame → body frame)
%  g_NED = [0; 0; g0] (down is positive z in NED)
%  g_body = R_BI * g_NED
%  ========================================================================

if sim_cfg.gravity_on
    g_NED  = [0; 0; g0];
    g_body = R_BI * g_NED;
    F_grav = m * g_body;
else
    F_grav = zeros(3,1);
end

%% ========================================================================
%  TOTAL FORCES & MOMENTS
%  ========================================================================

F_total = F_aero + F_prop + F_grav;   % Body frame [Fx; Fy; Fz] (N)
M_total = M_aero + M_prop;            % Body frame [L; M_pitch; N] (Nm)

%% ========================================================================
%  TRANSLATIONAL EOM  (Newton's 2nd law, body frame)
%  m * (dv/dt + omega × v) = F_total
%  => dv/dt = F/m - omega × v
%  ========================================================================

omega  = [p; qr; r];
v_body = [u; v; w];

acc_body = F_total / m - cross(omega, v_body);

udot = acc_body(1);
vdot = acc_body(2);
wdot = acc_body(3);

%% ========================================================================
%  ROTATIONAL EOM  (Euler's equation with full inertia tensor)
%  I * domega/dt = M_total - omega × (I * omega)
%  ========================================================================

gyro_coupling = cross(omega, I_mat * omega);
omega_dot     = I_mat \ (M_total - gyro_coupling);

pdot  = omega_dot(1);
qrdot = omega_dot(2);
rdot  = omega_dot(3);

%% ========================================================================
%  QUATERNION KINEMATICS
%  dq/dt = 0.5 * Xi(q) * omega
%
%  [q0_dot]   [ 0  -p  -qr  -r ] [q0]
%  [q1_dot] = [p    0   r  -qr ] [q1] * 0.5
%  [q2_dot]   [qr  -r   0   p  ] [q2]
%  [q3_dot]   [r   qr  -p   0  ] [q3]
%  ========================================================================

q0dot =  0.5 * (-p*q1n  - qr*q2n - r*q3n);
q1dot =  0.5 * ( p*q0n  + r*q2n  - qr*q3n);
q2dot =  0.5 * ( qr*q0n - r*q1n  + p*q3n);
q3dot =  0.5 * ( r*q0n  + qr*q1n - p*q2n);

%% ========================================================================
%  NED POSITION KINEMATICS
%  r_dot_NED = R_BI' * v_body   (transpose of body←inertial = inertial←body)
%  ========================================================================

r_dot_NED = R_BI' * v_body;   % [xN_dot; xE_dot; xD_dot]

%% ========================================================================
%  MASS DEPLETION
%  ========================================================================

m_dot = -mdot_total;   % Negative: mass decreases as propellant burns

% Protect: mass cannot go below dry mass
if m <= vehicle.mass_dry
    m_dot = 0.0;
end

%% ========================================================================
%  ASSEMBLE STATE DERIVATIVE
%  ========================================================================

xdot = zeros(14, 1);
xdot(1)  = r_dot_NED(1);   % xN_dot
xdot(2)  = r_dot_NED(2);   % xE_dot
xdot(3)  = r_dot_NED(3);   % xD_dot
xdot(4)  = udot;
xdot(5)  = vdot;
xdot(6)  = wdot;
xdot(7)  = q0dot;
xdot(8)  = q1dot;
xdot(9)  = q2dot;
xdot(10) = q3dot;
xdot(11) = pdot;
xdot(12) = qrdot;
xdot(13) = rdot;
xdot(14) = m_dot;

end

%% ========================================================================
%  LOCAL HELPER
%  ========================================================================

function y = clamp_scalar(x, lo, hi)
    y = min(max(x, lo), hi);
end
