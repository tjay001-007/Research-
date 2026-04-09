function [score, metrics] = fitness_function(chrom, vehicle, aero_db, prop_cfg, sim_cfg)
%FITNESS_FUNCTION  Evaluates a trajectory chromosome by running a fast 6DOF
%   simulation and scoring it against the weighted objectives.
%
%   CHROMOSOME ENCODING (13 parameters):
%     chrom(1:6)  — Pitch program polynomial coefficients [c0..c5]
%                   theta_cmd_deg = polyval(chrom(1:6), tau), tau ∈ [0,1]
%     chrom(7)    — Aerospike ignition altitude h_ignite (m)
%     chrom(8:11) — Throttle schedule breakpoints [t1,t2,t3,t4], each ∈ [0.4,1.0]
%     chrom(12)   — t_pitchover: pitch program end time (s) ∈ [30,120]
%     chrom(13)   — (reserved for future use, e.g. thrust split ratio)
%
%   OBJECTIVE (minimise):
%     score = w1*(-V_apogee_norm) + w2*(m_prop_used_norm) +
%             w3*(-h_apogee_norm) + w4*(qbar_peak_norm) + penalty
%
%   All objectives are normalised to O(1) for balanced weighting.
%
%   Inputs:
%     chrom     — Chromosome vector [13x1]
%     vehicle   — Vehicle config
%     aero_db   — Aero database
%     prop_cfg  — Propulsion config
%     sim_cfg   — Sim config (fast_eval=true for GA speed)
%
%   Outputs:
%     score     — Scalar fitness (lower = better)
%     metrics   — Struct with trajectory performance metrics

metrics.flight_ok = false;   % Assume failure until simulation completes

%% ========================================================================
%  DECODE CHROMOSOME
%  ========================================================================

pitch_coeffs = chrom(1:6)';   % [6x1] or row — polyval expects row
h_ignite     = chrom(7);
throttle_pts = chrom(8:11);
t_pitchover  = chrom(12);

% Clamp to physical bounds
h_ignite    = min(max(h_ignite, prop_cfg.eng2.h_ignite_min), prop_cfg.eng2.h_ignite_max);
t_pitchover = min(max(t_pitchover, 20), 150);

%% ========================================================================
%  BUILD TRAJECTORY PARAMETERS
%  ========================================================================

traj_params.pitch_coeffs  = pitch_coeffs;
traj_params.t_vertical    = sim_cfg.t_vertical;
traj_params.t_pitchover   = t_pitchover;
traj_params.psi_launch    = deg2rad(sim_cfg.launch_azimuth_deg);
traj_params.target_apogee = sim_cfg.target_apogee_km * 1000;

% Throttle schedule: linearly interpolated over burn time
% throttle_pts define values at t = [0, burn1/3, 2*burn1/3, burn1]
t_sched  = linspace(0, prop_cfg.eng1.burn_time, 4);
th_sched = throttle_pts;
th_sched = min(max(th_sched, prop_cfg.eng1.throttle_min), prop_cfg.eng1.throttle_max);

%% ========================================================================
%  FAST SIMULATION (simplified: no EKF, simplified aero for speed)
%  ========================================================================

% Initial state
phi0 = deg2rad(sim_cfg.ic.euler_deg(1));
theta0 = deg2rad(sim_cfg.ic.euler_deg(2));
psi0   = deg2rad(sim_cfg.ic.euler_deg(3));
q_init = angle2quat(psi0, theta0, phi0, 'ZYX')';  % Aerospace Toolbox

x = [sim_cfg.ic.pos_NED;
     sim_cfg.ic.vel_body;
     q_init;
     sim_cfg.ic.omega;
     vehicle.mass_total];

dt_fast = 0.05;      % 20 Hz integration for speed
t_max   = sim_cfg.t_end;

h_apogee   = 0;
V_apogee   = 0;
qbar_peak  = 0;
alpha_peak = 0;
m_start    = x(14);

% Simplified GNC state (no LQR, open-loop pitch program for fitness eval)
guid_state.phase         = 1;
guid_state.t_last_update = -1e6;
guid_state.theta_pc      = deg2rad(45);

sim_cfg_fast          = sim_cfg;
sim_cfg_fast.nav_ekf_on = false;   % No EKF overhead during fitness eval

try
    for t_sim = 0 : dt_fast : t_max

        altitude = -x(3);
        u = x(4); v = x(5); w = x(6);
        V = max(sqrt(u^2 + v^2 + w^2), 1);

        % Track peak values
        [~, a_s, ~, rho_s] = atmosisa(max(0, min(86000, altitude)));
        qbar_c = 0.5 * rho_s * V^2;
        if qbar_c > qbar_peak;  qbar_peak = qbar_c;   end

        alpha_c = rad2deg(atan2(w, u));
        if abs(alpha_c) > alpha_peak;  alpha_peak = abs(alpha_c);  end

        if altitude > h_apogee
            h_apogee = altitude;
            V_apogee = V;
        end

        % Stop conditions
        if altitude < -100 && t_sim > 5;  break;  end   % Ground impact
        if x(14) < vehicle.mass_dry + 10;          end   % Propellant out (continue coast)

        % Guidance command (open-loop pitch program only during GA eval)
        [theta_cmd, phi_cmd, psi_cmd, guid_state] = guidance_system( ...
            t_sim, x, traj_params, sim_cfg, guid_state);

        % Throttle from schedule
        th1 = interp1(t_sched, th_sched, min(t_sim, t_sched(end)), 'linear', th_sched(end));

        % Build simplified controls (no LQR feedback for speed)
        controls_fa.delta_e   = 0;
        controls_fa.delta_a   = 0;
        controls_fa.delta_r   = 0;
        controls_fa.tvc_pitch = 0;
        controls_fa.tvc_yaw   = 0;
        controls_fa.throttle1 = th1;
        controls_fa.throttle2 = 0.85;   % Fixed aerospike throttle
        controls_fa.rcs_cmd   = zeros(3,1);

        % Integrate (RK4)
        k1 = eom_6dof(t_sim,          x,              controls_fa, vehicle, aero_db, prop_cfg, sim_cfg_fast, h_ignite);
        k2 = eom_6dof(t_sim+dt_fast/2, x+0.5*dt_fast*k1, controls_fa, vehicle, aero_db, prop_cfg, sim_cfg_fast, h_ignite);
        k3 = eom_6dof(t_sim+dt_fast/2, x+0.5*dt_fast*k2, controls_fa, vehicle, aero_db, prop_cfg, sim_cfg_fast, h_ignite);
        k4 = eom_6dof(t_sim+dt_fast,   x+dt_fast*k3,     controls_fa, vehicle, aero_db, prop_cfg, sim_cfg_fast, h_ignite);
        x  = x + (dt_fast/6) * (k1 + 2*k2 + 2*k3 + k4);

        % Normalise quaternion
        q_n = x(7:10) / norm(x(7:10));
        x(7:10) = q_n;
        x(14) = max(x(14), vehicle.mass_dry);
    end

    m_prop_used = m_start - x(14);
    downrange_km = sqrt(x(1)^2 + x(2)^2) / 1000;
    metrics.flight_ok = true;

catch sim_err
    h_apogee    = 0;
    V_apogee    = 0;
    qbar_peak   = 1e7;   % Penalise crash
    alpha_peak  = 90;
    m_prop_used = vehicle.mass_prop1 + vehicle.mass_prop2;
    downrange_km = 0;
    metrics.flight_ok = false;
end

%% ========================================================================
%  PACK METRICS
%  ========================================================================

metrics.h_apogee      = h_apogee;
metrics.V_apogee      = V_apogee;
metrics.qbar_peak     = qbar_peak;
metrics.alpha_peak_deg= alpha_peak;
metrics.m_prop_used   = m_prop_used;
metrics.downrange_km  = downrange_km;
metrics.h_ignite      = h_ignite;
metrics.t_pitchover   = t_pitchover;

%% ========================================================================
%  NORMALISED OBJECTIVES
%  ========================================================================

% Reference values for normalisation
V_ref    = 2000;      % m/s  (representative apogee speed)
h_ref    = 100000;    % m    (100 km)
m_ref    = 3700;      % kg   (total propellant)
qbar_ref = 50000;     % Pa

w = sim_cfg.obj;

obj_speed  = w.maximize_apogee_speed  * (-metrics.V_apogee  / V_ref);
obj_fuel   = w.minimize_fuel          * (metrics.m_prop_used / m_ref);
obj_alt    = w.maximize_apogee_alt    * (-metrics.h_apogee   / h_ref);
obj_qbar   = w.minimize_peak_qbar     * (metrics.qbar_peak   / qbar_ref);

% Add ignition hint to sim_cfg for constraint_manager
sim_cfg.h_ignite_min = prop_cfg.eng2.h_ignite_min;
sim_cfg.h_ignite_max = prop_cfg.eng2.h_ignite_max;

[penalty, violations] = constraint_manager(metrics, sim_cfg);

score = obj_speed + obj_fuel + obj_alt + obj_qbar + penalty;

metrics.objectives   = [obj_speed, obj_fuel, obj_alt, obj_qbar];
metrics.penalty      = penalty;
metrics.violations   = violations;
metrics.score        = score;

end
