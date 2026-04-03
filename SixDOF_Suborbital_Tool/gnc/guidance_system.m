function [theta_cmd, phi_cmd, psi_cmd, guid_state_out] = guidance_system(t, x, traj_params, sim_cfg, guid_state_in)
%GUIDANCE_SYSTEM  Ascent guidance for suborbital winged rocket.
%
%   Three-phase guidance:
%
%   Phase 1 — Vertical ascent (0 to t_vertical):
%     Maintain theta = 90 deg (vertical), phi = 0.
%     Pure TVC/RCS attitude hold.
%
%   Phase 2 — Pitch program (t_vertical to t_pitchover):
%     Polynomial pitch angle schedule:
%       theta_cmd(t) = polyval(pitch_coeffs, tau)
%     where tau = (t - t_vertical) / (t_pitchover - t_vertical) ∈ [0,1].
%     Coefficients optimised by GA to shape gravity-turn trajectory.
%
%   Phase 3 — Predictor-corrector (t > t_pitchover):
%     Every guidance_dt seconds, run a coarse forward simulation to predict
%     apogee altitude.  Adjust theta_cmd to drive predicted_apogee → target.
%     Uses secant method: two predictions with different theta offsets.
%
%   Inputs:
%     t           — Current simulation time (s)
%     x           — Current state vector [14x1]
%     traj_params — Trajectory parameters struct:
%                     .pitch_coeffs  [6x1] polynomial (highest degree first)
%                     .t_vertical    (s)
%                     .t_pitchover   (s)
%                     .psi_launch    (rad) launch azimuth
%                     .target_apogee (m)
%     sim_cfg     — Sim config struct
%     guid_state_in — Previous guidance state
%
%   Outputs:
%     theta_cmd     — Commanded pitch angle (rad)
%     phi_cmd       — Commanded roll angle  (rad)
%     psi_cmd       — Commanded yaw angle   (rad)
%     guid_state_out — Updated guidance state

%% ========================================================================
%  EXTRACT
%  ========================================================================

altitude = -x(3);   % m

%% ========================================================================
%  GUIDANCE PHASE DETERMINATION
%  ========================================================================

t_vert = traj_params.t_vertical;
t_pit  = traj_params.t_pitchover;

if t < t_vert
    phase = 1;   % Vertical ascent
elseif t < t_pit
    phase = 2;   % Pitch program
else
    phase = 3;   % Predictor-corrector
end

%% ========================================================================
%  PHASE 1 — VERTICAL ASCENT
%  ========================================================================

if phase == 1
    theta_cmd = deg2rad(90);    % Straight up
    phi_cmd   = 0;
    psi_cmd   = traj_params.psi_launch;

%% ========================================================================
%  PHASE 2 — PITCH PROGRAM
%  ========================================================================

elseif phase == 2
    tau = (t - t_vert) / max(t_pit - t_vert, 0.1);   % Normalised time [0,1]
    tau = min(tau, 1.0);

    % Polynomial pitch schedule: theta decreases from 90 deg to some angle
    % pitch_coeffs are for theta in DEGREES as function of tau
    theta_deg = polyval(traj_params.pitch_coeffs, tau);
    theta_deg = min(max(theta_deg, 5.0), 89.0);   % Clamp [5°, 89°]

    theta_cmd = deg2rad(theta_deg);
    phi_cmd   = 0;
    psi_cmd   = traj_params.psi_launch;

%% ========================================================================
%  PHASE 3 — PREDICTOR-CORRECTOR
%  ========================================================================

else
    % Update guidance command only at guidance_dt intervals
    guid_state_out = guid_state_in;
    update_due = (t - guid_state_in.t_last_update) >= sim_cfg.dt_guidance;

    if update_due || ~isfield(guid_state_in, 'theta_pc')
        theta_pc = predictor_corrector_update(t, x, traj_params, sim_cfg, guid_state_in);
        guid_state_out.theta_pc = theta_pc;
        guid_state_out.t_last_update = t;
    else
        theta_pc = guid_state_in.theta_pc;
    end

    theta_cmd = theta_pc;
    phi_cmd   = 0;
    psi_cmd   = traj_params.psi_launch;

    guid_state_out.phase = 3;
    return;
end

%% ========================================================================
%  OUTPUT PACKAGING (phases 1 & 2)
%  ========================================================================

guid_state_out          = guid_state_in;
guid_state_out.phase    = phase;
guid_state_out.altitude = altitude;

if ~isfield(guid_state_out, 'theta_pc')
    guid_state_out.theta_pc = theta_cmd;
end
if ~isfield(guid_state_out, 't_last_update')
    guid_state_out.t_last_update = -1e6;
end

end

%% ========================================================================
%  PREDICTOR-CORRECTOR (local function)
%  Run two coarse mini-simulations to find theta that hits target apogee.
%  Uses secant method: one step in most cases.
%  ========================================================================

function theta_pc = predictor_corrector_update(t, x, traj_params, sim_cfg, guid_state)

target_h = traj_params.target_apogee;   % m

%  Current theta estimate
if isfield(guid_state, 'theta_pc')
    theta0 = guid_state.theta_pc;
else
    theta0 = deg2rad(30);   % Initial guess
end

%  Step size for secant gradient
d_theta = deg2rad(2.0);

%  Predict apogee for theta0 and theta0 + d_theta
h_apo0 = predict_apogee(t, x, theta0,            traj_params, sim_cfg);
h_apo1 = predict_apogee(t, x, theta0 + d_theta,  traj_params, sim_cfg);

err0 = h_apo0 - target_h;
err1 = h_apo1 - target_h;

dh_dtheta = (err1 - err0) / d_theta;

if abs(dh_dtheta) > 1.0   % At least 1 m altitude change per rad of theta
    correction = -err0 / dh_dtheta;
else
    correction = 0;
end

% Limit correction step
correction = min(max(correction, deg2rad(-5)), deg2rad(5));

theta_pc = theta0 + correction;
theta_pc = min(max(theta_pc, deg2rad(2)), deg2rad(85));  % Hard limits

end

%% ========================================================================
%  APOGEE PREDICTOR — coarse forward simulation
%  ========================================================================

function h_apo = predict_apogee(t0, x0, theta_fixed, traj_params, sim_cfg)

dt_c = 0.5;      % Coarse timestep (s) — fast propagation
t_max = 300;     % Max propagation time (s)
g0   = 9.80665;

x = x0;
h_prev = -x(3);
h_apo  = h_prev;

for t_p = t0 : dt_c : (t0 + t_max)
    altitude = -x(3);
    u = x(4);  v = x(5);  w = x(6);
    V = max(sqrt(u^2 + v^2 + w^2), 1);

    % Simplified: enforce commanded pitch via gravity-turn shortcut
    % Extract current flight path angle
    gamma = asin(min(max((-x(6)) / V, -1), 1));   % positive = climbing

    % Stop if descending below apogee
    if t_p > t0 + 5 && x(3) > x0(3) + 1000
        break;   % Passed apogee (xD increasing means descending in NED)
    end

    % Very simple point-mass EOM for prediction (no control, gravity turn)
    [~, a_sound, ~, rho] = atmosisa(max(0, min(86000, altitude)));
    V_dot = -g0 * sin(gamma);         % Gravity deceleration along path
    gamma_dot = -(g0/V) * cos(gamma); % Gravity turn rate

    Vnew = max(1, V + V_dot * dt_c);
    gamma_new = gamma + gamma_dot * dt_c;

    % Update simplified state: just position and velocity
    x(3) = x(3) + Vnew * (-sin(gamma_new)) * dt_c;   % xD_dot = -V*sin(gamma)
    x(4) = Vnew * cos(gamma_new) * cos(traj_params.psi_launch);
    x(6) = -Vnew * sin(gamma_new);

    h_new = -x(3);
    if h_new > h_apo
        h_apo = h_new;
    end

    if h_new < h_apo - 100 && t_p > t0 + 10
        break;   % Clearly descended from apogee
    end
end

end
