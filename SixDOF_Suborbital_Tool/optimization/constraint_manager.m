function [penalty, violations] = constraint_manager(metrics, sim_cfg)
%CONSTRAINT_MANAGER  Evaluates switchable trajectory constraints and
%   returns a penalty value for use in the GA fitness function.
%
%   Each constraint can be toggled on/off via sim_cfg.con.<name> flags.
%   Penalty is quadratic in the violation magnitude:
%     penalty_i = weight * max(0, violation_i)^2
%
%   Inputs:
%     metrics   — Trajectory metrics struct from fitness_function.m:
%                   .h_apogee        Peak altitude (m)
%                   .V_apogee        Speed at apogee (m/s)
%                   .qbar_peak       Peak dynamic pressure (Pa)
%                   .alpha_peak_deg  Peak angle of attack (deg)
%                   .downrange_km    Downrange at apogee (km)
%                   .m_prop_used     Propellant consumed (kg)
%                   .h_ignite        Aerospike ignition altitude (m)
%                   .flight_ok       false if simulation crashed/diverged
%
%     sim_cfg   — Sim config with constraint flags and limits
%
%   Outputs:
%     penalty    — Total penalty value (non-negative scalar)
%     violations — Struct with individual violation details

penalty    = 0;
violations = struct();
w = sim_cfg.penalty_weight;

%% ========================================================================
%  SIMULATION VALIDITY PENALTY
%  ========================================================================

if ~metrics.flight_ok
    penalty = penalty + w * 1e4;   % Heavy penalty for crashed/diverged sim
    violations.sim_crash = true;
    return;
end
violations.sim_crash = false;

%% ========================================================================
%  MAX DYNAMIC PRESSURE
%  ========================================================================

if sim_cfg.con.max_qbar
    excess = max(0, metrics.qbar_peak - sim_cfg.con.qbar_limit);
    p_qbar = w * (excess / sim_cfg.con.qbar_limit)^2;
    penalty = penalty + p_qbar;
    violations.qbar = excess;
    violations.qbar_penalty = p_qbar;
else
    violations.qbar = 0;
end

%% ========================================================================
%  MINIMUM APOGEE ALTITUDE
%  ========================================================================

if sim_cfg.con.min_apogee_alt
    shortfall = max(0, sim_cfg.con.apogee_alt_min_km * 1000 - metrics.h_apogee);
    p_alt = w * (shortfall / (sim_cfg.con.apogee_alt_min_km * 1000))^2;
    penalty = penalty + p_alt;
    violations.apogee_altitude = shortfall;
    violations.alt_penalty = p_alt;
else
    violations.apogee_altitude = 0;
end

%% ========================================================================
%  MAX ANGLE OF ATTACK
%  ========================================================================

if sim_cfg.con.max_AoA
    excess_alpha = max(0, metrics.alpha_peak_deg - sim_cfg.con.AoA_limit_deg);
    p_alpha = w * (excess_alpha / sim_cfg.con.AoA_limit_deg)^2;
    penalty = penalty + p_alpha;
    violations.AoA = excess_alpha;
    violations.AoA_penalty = p_alpha;
else
    violations.AoA = 0;
end

%% ========================================================================
%  LANDING RANGE
%  ========================================================================

if sim_cfg.con.landing_range
    excess_range = max(0, metrics.downrange_km - sim_cfg.con.range_limit_km);
    p_range = w * (excess_range / sim_cfg.con.range_limit_km)^2;
    penalty = penalty + p_range;
    violations.range = excess_range;
    violations.range_penalty = p_range;
else
    violations.range = 0;
end

%% ========================================================================
%  AEROSPIKE IGNITION ALTITUDE RANGE
%  ========================================================================

if sim_cfg.con.aerospike_h_range && sim_cfg.aerospike_on
    h_min = prop_cfg_h_range_min(sim_cfg);
    h_max = prop_cfg_h_range_max(sim_cfg);
    h_ign = metrics.h_ignite;

    under_min = max(0, h_min - h_ign);
    over_max  = max(0, h_ign - h_max);
    excess_h  = max(under_min, over_max);

    p_h = w * (excess_h / max(h_min, 1))^2;
    penalty = penalty + p_h;
    violations.h_ignite = excess_h;
    violations.h_ignite_penalty = p_h;
else
    violations.h_ignite = 0;
end

violations.total = penalty;

end

%% ========================================================================
%  LOCAL: Read ignition limits from sim_cfg (stored as copy during GA eval)
%  ========================================================================

function h_min = prop_cfg_h_range_min(sim_cfg)
if isfield(sim_cfg, 'h_ignite_min')
    h_min = sim_cfg.h_ignite_min;
else
    h_min = 15000;
end
end

function h_max = prop_cfg_h_range_max(sim_cfg)
if isfield(sim_cfg, 'h_ignite_max')
    h_max = sim_cfg.h_ignite_max;
else
    h_max = 55000;
end
end
