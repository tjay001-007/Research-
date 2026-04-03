function gain_table = lqr_gain_schedule(vehicle, aero_db, prop_cfg, sim_cfg)
%LQR_GAIN_SCHEDULE  Design LQR gain matrices at multiple flight conditions
%   and store in a gain table for runtime interpolation.
%
%   Approach:
%     1. Define a grid of operating points (Mach, dynamic pressure qbar)
%     2. At each point, numerically linearise the reduced attitude/rate dynamics
%     3. Solve discrete LQR: [K, ~, ~] = lqr(A_att, B_att, Q, R)
%        using MATLAB Control System Toolbox
%     4. Store gain matrices in table
%
%   Linearisation state (6-state attitude subsystem):
%     z = [phi_err; theta_err; psi_err; p_err; qr_err; r_err]
%
%   Inputs:
%     vehicle   — Vehicle config struct
%     aero_db   — Aerodynamic database
%     prop_cfg  — Propulsion config
%     sim_cfg   — Sim config
%
%   Outputs:
%     gain_table — Struct with Mach grid, qbar grid, and 3D gain array

fprintf('\n[LQR] Computing gain schedule...\n');

%% ========================================================================
%  SCHEDULE GRID
%  ========================================================================

Mach_grid = [0.2, 0.5, 0.8, 1.0, 1.5, 2.0, 3.0, 5.0];
qbar_grid = [500, 2000, 5000, 10000, 20000, 50000];   % Pa

nM = length(Mach_grid);
nQ = length(qbar_grid);

% Pre-allocate gain matrices: K is [3 x 6] (3 moment axes, 6 attitude/rate states)
% Stored as 4D array: [3, 6, nM, nQ]
K_table = zeros(3, 6, nM, nQ);

%% ========================================================================
%  LQR WEIGHTING MATRICES
%  ========================================================================

% State weighting Q — heavier on pitch (destabilising) and attitude accuracy
% State: [phi_err, theta_err, psi_err, p_err, qr_err, r_err]
%          roll_att  pitch_att  yaw_att  roll_rate  pitch_rate  yaw_rate
Q = diag([  8.0,      20.0,      5.0,    2.0,        8.0,       1.5]);
% - Pitch gets highest attitude weight (stabilise the unstable axis)
% - Roll attitude moderate, yaw least (open-loop yaw is marginally stable)
% - Pitch rate gets higher rate weight than roll for same reason

% Control weighting R — [L_moment; M_moment; N_moment]
% Heavy on pitch moment (conservative — pitch instability means aggressive control
% can excite structural modes)
R = diag([1.0, 0.8, 1.2]);

%% ========================================================================
%  LINEARISATION AND LQR DESIGN
%  ========================================================================

for im = 1:nM
    for iq = 1:nQ
        Mach = Mach_grid(im);
        qbar = qbar_grid(iq);

        [A6, B6] = linearise_attitude_subsystem(Mach, qbar, vehicle, aero_db);

        % Check stabilisability before LQR
        [~, eigA] = eig(A6);
        ev = diag(eigA);

        try
            [K, ~, ~] = lqr(A6, B6, Q, R);
        catch lqr_err
            % If LQR fails (e.g., not stabilisable at this op point),
            % use a robust proportional gain as fallback
            fprintf('[LQR] Warning at Mach=%.1f qbar=%.0f: %s — using fallback\n', ...
                Mach, qbar, lqr_err.message);
            K = fallback_gain(Mach, qbar, vehicle, aero_db);
        end

        % Verify closed-loop stability
        A_cl = A6 - B6 * K;
        ev_cl = eig(A_cl);
        if any(real(ev_cl) > 0)
            fprintf('[LQR] WARNING: Mach=%.1f qbar=%.0f has unstable CL poles. Increasing Q_pitch.\n', Mach, qbar);
            Q_heavy = Q;
            Q_heavy(2,2) = Q(2,2) * 5;   % 5x pitch attitude weight
            Q_heavy(5,5) = Q(5,5) * 5;   % 5x pitch rate weight
            try
                [K, ~, ~] = lqr(A6, B6, Q_heavy, R);
            catch
                K = fallback_gain(Mach, qbar, vehicle, aero_db);
            end
        end

        K_table(:, :, im, iq) = K;
    end
end

%% ========================================================================
%  PACK OUTPUT
%  ========================================================================

gain_table.Mach_grid = Mach_grid;
gain_table.qbar_grid = qbar_grid;
gain_table.K_table   = K_table;
gain_table.Q         = Q;
gain_table.R         = R;

fprintf('[LQR] Gain schedule complete: %d Mach × %d qbar points.\n', nM, nQ);

end

%% ========================================================================
%  LINEARISE ATTITUDE SUBSYSTEM
%  Reduced 6-state model around trim (small perturbation):
%    State:   z = [phi; theta; psi; p; qr; r]
%    Control: u = [L_moment; M_moment; N_moment]  (moments, not surface deflections)
%  ========================================================================

function [A6, B6] = linearise_attitude_subsystem(Mach, qbar, vehicle, aero_db)

S = vehicle.S;
b = vehicle.b;
c = vehicle.c;

% Use mid-fuel inertia (representative)
m_half = (vehicle.mass_total + vehicle.mass_dry) / 2;
delta_m_half = vehicle.mass_total - m_half;
Ixx = max(100, vehicle.Ixx + vehicle.dIxx_dm * delta_m_half);
Iyy = max(500, vehicle.Iyy + vehicle.dIyy_dm * delta_m_half);
Izz = max(500, vehicle.Izz + vehicle.dIzz_dm * delta_m_half);
Ixz = vehicle.Ixz + vehicle.dIxz_dm * delta_m_half;

% Interpolate aero derivatives at this Mach (at alpha=0 trim)
alpha_trim = 0;
Mach_c = min(max(Mach, aero_db.Mach_vec(1)), aero_db.Mach_vec(end));

Cmalpha = interp1(aero_db.Mach_vec, aero_db.Cmalpha_vec, Mach_c, 'linear', 'extrap');
Clp     = interp1(aero_db.Mach_vec, aero_db.Clp, Mach_c, 'linear', 'extrap');
Cmq     = interp1(aero_db.Mach_vec, aero_db.Cmq, Mach_c, 'linear', 'extrap');
Cnr     = interp1(aero_db.Mach_vec, aero_db.Cnr, Mach_c, 'linear', 'extrap');
Clr     = interp1(aero_db.Mach_vec, aero_db.Clr, Mach_c, 'linear', 'extrap');
Cnp     = interp1(aero_db.Mach_vec, aero_db.Cnp, Mach_c, 'linear', 'extrap');

% Dimensional derivatives (moment per unit state)
% Pitch stiffness (key: Malphadot ≈ qbar*S*c*Cmalpha)
Malpha = qbar * S * c * Cmalpha / Iyy;    % 1/s^2 — positive = unstable

Lp = qbar * S * b * Clp * (b/(2)) / Ixx;  % (Actually Clp*b/(2V), but we absorb V)
Mq = qbar * S * c * Cmq * (c/(2)) / Iyy;
Nr = qbar * S * b * Cnr * (b/(2)) / Izz;
Lr = qbar * S * b * Clr * (b/(2)) / Ixx;
Np = qbar * S * b * Cnp * (b/(2)) / Izz;

% Note: for linearisation velocity V ~ Mach * 340 (approximate sea-level sound speed)
% The p_hat = p*b/(2V), so Lp_dim = Clp * qbar*S*b^2 / (2*V*Ixx)
% Re-scale with representative V:
V_rep = max(10, Mach * 340);
Lp = qbar * S * b^2 / (2 * V_rep * Ixx) * Clp;
Mq = qbar * S * c^2 / (2 * V_rep * Iyy) * Cmq;
Nr = qbar * S * b^2 / (2 * V_rep * Izz) * Cnr;
Lr = qbar * S * b^2 / (2 * V_rep * Ixx) * Clr;
Np = qbar * S * b^2 / (2 * V_rep * Izz) * Cnp;

% Full inertia tensor effects (Ixz cross-coupling)
% Simplified: ignore cross-coupling for linearisation (second-order effect)

% A matrix for [phi; theta; psi; p; qr; r]
% Kinematics (rows 1-3): euler_dot ≈ omega for small angles
% Dynamics (rows 4-6): from Euler's equations
A6 = zeros(6,6);

% Kinematics: phi_dot≈p, theta_dot≈qr, psi_dot≈r  (small angle)
A6(1,4) = 1;   % phi_dot = p
A6(2,5) = 1;   % theta_dot = qr
A6(3,6) = 1;   % psi_dot = r

% Roll rate dynamics: p_dot = Lp*p + Lr*r + ...  (Mach-dependent damping)
A6(4,4) = Lp;
A6(4,6) = Lr;

% Pitch rate dynamics: qr_dot = Malpha*theta + Mq*qr  (pitch instability!)
A6(5,2) = Malpha;   % POSITIVE for unstable aircraft
A6(5,5) = Mq;

% Yaw rate dynamics: r_dot = Nr*r + Np*p
A6(6,4) = Np;
A6(6,6) = Nr;

% B matrix: moments → angular acceleration
% Control input u = [L_moment; M_moment; N_moment]
B6 = zeros(6,3);
B6(4,1) = 1/Ixx;   % p_dot from roll moment
B6(5,2) = 1/Iyy;   % qr_dot from pitch moment
B6(6,3) = 1/Izz;   % r_dot from yaw moment

end

%% ========================================================================
%  FALLBACK GAIN (used when LQR fails)
%  Based on pole-placement heuristic for unstable pitch axis
%  ========================================================================

function K = fallback_gain(Mach, qbar, vehicle, aero_db)
% Proportional gains scaled by dynamic pressure
q_scale = max(qbar, 100);
K = zeros(3,6);
K(1, 1) = 3.0 / q_scale * 1e4;   % Roll: phi_err gain
K(1, 4) = 8.0 / q_scale * 1e4;   % Roll: p_err gain
K(2, 2) = 6.0 / q_scale * 1e4;   % Pitch: theta_err gain (higher for instability)
K(2, 5) = 12.0 / q_scale * 1e4;  % Pitch: qr_err gain
K(3, 3) = 2.0 / q_scale * 1e4;   % Yaw: psi_err gain
K(3, 6) = 5.0 / q_scale * 1e4;   % Yaw: r_err gain
end
