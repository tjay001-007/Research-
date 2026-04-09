function [F_prop, M_prop, mdot_total, prop_state] = propulsion_model(x, controls, vehicle, prop_cfg, atm, sim_cfg, h_ignite)
%PROPULSION_MODEL  Dual propulsion system orchestrator.
%
%   Manages both engines:
%     Engine 1 (conventional bell-nozzle): active from T=0, with TVC
%     Engine 2 (aerospike): activates at altitude h_ignite (optimisable)
%
%   Both engines can run simultaneously during the transition window
%   (prop_cfg.transition_window seconds after aerospike ignition).
%
%   Engine shutdown conditions:
%     - Engine 1: throttle1 = 0, or propellant exhausted
%     - Engine 2: throttle2 = 0, or not yet at h_ignite, or prop exhausted
%
%   Inputs:
%     x          — State vector [14x1]
%     controls   — Controls struct (throttle1, throttle2, tvc_pitch, tvc_yaw)
%     vehicle    — Vehicle config
%     prop_cfg   — Propulsion config
%     atm        — Atmosphere struct (Pa, rho, a)
%     sim_cfg    — Sim config (tvc_on, aerospike_on flags)
%     h_ignite   — Aerospike ignition altitude (m) — GA-optimised parameter
%
%   Outputs:
%     F_prop     — Total propulsion force in body frame [3x1] (N)
%     M_prop     — Total propulsion moment about CG [3x1] (Nm)
%     mdot_total — Total propellant mass flow rate (kg/s, positive)
%     prop_state — Diagnostics struct

%% ========================================================================
%  EXTRACT STATE
%  ========================================================================

xD  = x(3);
m   = x(14);
altitude = -xD;   % Altitude (m)
Pa       = atm.Pa;

%  Remaining propellant in each tank
m_prop_remaining = max(0, m - vehicle.mass_dry);
% Fraction: engine 1 burns first (sequential staging)
m_prop1_remaining = min(m_prop_remaining, vehicle.mass_prop1);
m_prop2_remaining = max(0, m_prop_remaining - vehicle.mass_prop1);
%  (Note: in a real design, tanks are sized separately. Here engine 2
%   propellant is considered available only after engine 1 fuel is depleted
%   OR when aerospike ignites — both engines can draw simultaneously from
%   their respective tanks. Here we simplify to separate propellant budgets.)

% Simpler model: each engine draws from its own propellant budget
% Track via mass: once m_prop1 gone, engine 1 stops; once m_prop2 gone, engine 2 stops
prop_delta = vehicle.mass_total - m;   % Total prop burned (kg)
% Conservative estimate of each engine's consumption (50/50 initially)
m_prop1_burned = min(prop_delta, vehicle.mass_prop1);
m_prop2_burned = max(0, prop_delta - vehicle.mass_prop1);

m_prop1_remaining = max(0, vehicle.mass_prop1 - m_prop1_burned);
m_prop2_remaining = max(0, vehicle.mass_prop2 - m_prop2_burned);

%% ========================================================================
%  ENGINE 1 — CONVENTIONAL BELL NOZZLE
%  ========================================================================

eng1_active = (controls.throttle1 > 0) && (m_prop1_remaining > 0);

if eng1_active
    % Thrust lapse: T = T_sl + (T_vac - T_sl) * (1 - Pa/Pa_sl)
    lapse_factor = (prop_cfg.eng1.T_vac - prop_cfg.eng1.T_sl) / prop_cfg.eng1.T_sl;
    T1 = prop_cfg.eng1.T_sl * controls.throttle1 * (1 + lapse_factor * (1 - Pa/prop_cfg.Pa_sl));
    T1 = max(0, T1);

    % Mass flow rate
    Isp1    = prop_cfg.eng1.Isp_sl + (prop_cfg.eng1.Isp_vac - prop_cfg.eng1.Isp_sl) * (1 - Pa/prop_cfg.Pa_sl);
    mdot1   = T1 / (Isp1 * prop_cfg.g0);
else
    T1     = 0;
    mdot1  = 0;
end

%% ========================================================================
%  TVC ON ENGINE 1
%  ========================================================================

tvc_state_dummy = [0; 0];   % No previous state here — TVC state managed by control_system
if sim_cfg.tvc_on && eng1_active
    tvc_cmd = [controls.tvc_pitch; controls.tvc_yaw];
    dt_prop  = sim_cfg.dt;
    [F_tvc, M_tvc, ~] = tvc_model(T1, tvc_cmd, vehicle, m, tvc_state_dummy, dt_prop);
    % Note: tvc_state tracking is done in control_system; here we use current cmd directly
    F_eng1 = F_tvc;
    M_eng1 = M_tvc;
else
    % Aligned thrust (no gimbal)
    F_eng1 = [T1; 0; 0];
    M_eng1 = zeros(3,1);
end

%% ========================================================================
%  ENGINE 2 — AEROSPIKE
%  ========================================================================

ae_alt_ok   = (altitude >= h_ignite);
ae_prop_ok  = (m_prop2_remaining > 0);
ae_system_on = sim_cfg.aerospike_on;

eng2_active = ae_alt_ok && ae_prop_ok && ae_system_on && (controls.throttle2 > 0);

if eng2_active
    [T2, mdot2, F_ae_body, ~] = aerospike_engine(controls.throttle2, altitude, Pa, prop_cfg);
    F_eng2 = F_ae_body;
    % Aerospike is body-centred, no gimbal → no pitch/yaw moment from offset
    % Small roll moment if misaligned (neglected in this model)
    M_eng2 = zeros(3,1);
else
    T2    = 0;
    mdot2 = 0;
    F_eng2 = zeros(3,1);
    M_eng2 = zeros(3,1);
end

%% ========================================================================
%  TOTALS
%  ========================================================================

F_prop     = F_eng1 + F_eng2;
M_prop     = M_eng1 + M_eng2;
mdot_total = mdot1 + mdot2;

%% ========================================================================
%  DIAGNOSTICS
%  ========================================================================

prop_state.T1              = T1;
prop_state.T2              = T2;
prop_state.mdot1           = mdot1;
prop_state.mdot2           = mdot2;
prop_state.mdot_total      = mdot_total;
prop_state.eng1_active     = eng1_active;
prop_state.eng2_active     = eng2_active;
prop_state.m_prop1_remaining = m_prop1_remaining;
prop_state.m_prop2_remaining = m_prop2_remaining;
prop_state.altitude        = altitude;
prop_state.h_ignite        = h_ignite;

end
