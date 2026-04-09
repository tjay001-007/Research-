function [F_tvc, M_tvc, tvc_state_out] = tvc_model(T1, tvc_cmd, vehicle, m_current, tvc_state_in, dt)
%TVC_MODEL  Thrust Vector Control gimbal kinematics and moment computation.
%
%   The main engine nozzle can be gimballed in pitch and yaw to produce
%   control moments.  Gimbal angles are rate-limited and position-limited.
%
%   Gimbal geometry (body frame):
%     Nozzle is located at x = x_nozzle (aft), on centreline y=z=0.
%     Pitching the nozzle by angle delta_p tilts thrust vector in xz-plane.
%     Yawing  the nozzle by angle delta_y tilts thrust vector in xy-plane.
%
%   Thrust vector (body frame):
%     F_tvc = T1 * [cos(dp)*cos(dy); sin(dy); -sin(dp)*cos(dy)]
%
%   Moment about CG (body frame):
%     The thrust force is applied at the nozzle exit plane.
%     Moment arm l_tvc = distance from nozzle exit plane to current CG.
%     As propellant burns, CG shifts aft, reducing l_tvc.
%
%   Inputs:
%     T1           — Current engine 1 thrust magnitude (N)
%     tvc_cmd      — Commanded TVC [delta_p_cmd; delta_y_cmd] (rad)
%     vehicle      — Vehicle config struct
%     m_current    — Current vehicle mass (kg) for CG calculation
%     tvc_state_in — Previous TVC state: [delta_p_actual; delta_y_actual] (rad)
%     dt           — Integration time step (s)
%
%   Outputs:
%     F_tvc         — TVC force in body frame [Fx; Fy; Fz] (N)
%     M_tvc         — TVC moment about CG in body frame [L; M; N] (Nm)
%     tvc_state_out — Updated TVC state [delta_p; delta_y] (rad)

%% ========================================================================
%  GIMBAL LIMITS
%  ========================================================================

tvc_max_rad  = deg2rad(vehicle.tvc_max);          % ±8 deg → ±0.1396 rad
tvc_rate_rad = deg2rad(vehicle.tvc_rate_max);     % 30 deg/s → 0.5236 rad/s

%% ========================================================================
%  RATE LIMITER + POSITION LIMITER
%  ========================================================================

dp_prev = tvc_state_in(1);
dy_prev = tvc_state_in(2);

% Clamp command to position limits
dp_cmd = clamp_s(tvc_cmd(1), -tvc_max_rad, tvc_max_rad);
dy_cmd = clamp_s(tvc_cmd(2), -tvc_max_rad, tvc_max_rad);

% Rate limit: gimbal cannot move faster than tvc_rate_max
delta_p_max_step = tvc_rate_rad * dt;
dp_actual = dp_prev + clamp_s(dp_cmd - dp_prev, -delta_p_max_step, delta_p_max_step);
dy_actual = dy_prev + clamp_s(dy_cmd - dy_prev, -delta_p_max_step, delta_p_max_step);

% Clamp to position limits after rate limiting
dp_actual = clamp_s(dp_actual, -tvc_max_rad, tvc_max_rad);
dy_actual = clamp_s(dy_actual, -tvc_max_rad, tvc_max_rad);

tvc_state_out = [dp_actual; dy_actual];

%% ========================================================================
%  TVC MOMENT ARM (varies with propellant burn → CG shift)
%  ========================================================================

delta_m    = vehicle.mass_total - m_current;   % Propellant burned (kg)
% CG moves aft by CG_shift_per_kg * mass_burned
cg_shift   = vehicle.CG_shift_per_kg_prop1 * min(delta_m, vehicle.mass_prop1) + ...
             vehicle.CG_shift_per_kg_prop2 * max(0, delta_m - vehicle.mass_prop1);
% CG current position from nose
CG_current = vehicle.CG_initial + cg_shift;

% Moment arm: nozzle exit plane is at x = vehicle.L (aft of nose)
% Positive arm means nozzle is aft of CG (standard for rockets)
l_tvc = vehicle.L - CG_current;      % Effective TVC moment arm (m)
l_tvc = max(0.5, l_tvc);             % Floor to avoid zero (structural limit)

%% ========================================================================
%  THRUST VECTOR IN BODY FRAME
%
%  Gimbal pitch angle dp: tilts thrust in xz-plane (nose-up moment for dp>0)
%  Gimbal yaw   angle dy: tilts thrust in xy-plane (nose-right moment for dy>0)
%
%  F_tvc = T1 * R_gimbal * [1;0;0]
%  ========================================================================

Fx_tvc =  T1 * cos(dp_actual) * cos(dy_actual);
Fy_tvc =  T1 * sin(dy_actual);
Fz_tvc = -T1 * sin(dp_actual) * cos(dy_actual);

F_tvc = [Fx_tvc; Fy_tvc; Fz_tvc];

%% ========================================================================
%  MOMENT ABOUT CG
%  The thrust force acts at point r_nozzle = [x_nozzle - CG; 0; 0] from CG
%  In body frame: r_from_CG = [l_tvc; 0; 0] (nozzle is aft → positive x offset)
%  Wait: nozzle is aft of CG, so in body frame (x forward):
%    r_nozzle_from_CG = [-l_tvc; 0; 0]  (nozzle behind CG → negative x)
%
%  Moment = r × F
%  r = [-l_tvc; 0; 0]
%  F = [Fx_tvc; Fy_tvc; Fz_tvc]
%
%  M = r × F:
%   L (roll)  = ry*Fz - rz*Fy =   0*Fz - 0*Fy = 0
%   M (pitch) = rz*Fx - rx*Fz =   0*Fx - (-l_tvc)*Fz = l_tvc * Fz_tvc
%   N (yaw)   = rx*Fy - ry*Fx = (-l_tvc)*Fy - 0*Fx   = -l_tvc * Fy_tvc
%  ========================================================================

r_from_CG = [-l_tvc; 0; 0];
M_tvc = cross(r_from_CG, F_tvc);   % [L; M_pitch; N] (Nm)

end

%% ========================================================================
%  LOCAL HELPER
%  ========================================================================

function y = clamp_s(x, lo, hi)
    y = min(max(x, lo), hi);
end
