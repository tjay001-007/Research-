function [T2, mdot2, F_ae_body, valid] = aerospike_engine(throttle2, altitude, Pa, prop_cfg)
%AEROSPIKE_ENGINE  Altitude-compensating aerospike rocket thrust model.
%
%   Unlike a conventional bell nozzle, the aerospike nozzle self-adjusts its
%   effective exit pressure to match ambient across a broad altitude range.
%   This means:
%     - No over-expansion loss at sea level (small correction only)
%     - No under-expansion loss at altitude
%     - Thrust ≈ T_vac for Pa << Pc (high altitude)
%
%   Model:
%     T2 = mdot2 * Isp_eff * g0
%     Isp_eff = Isp_vac * (1 - eta_ae * Pa / Pc)
%     mdot2   = throttle2 * T_vac / (Isp_vac * g0)
%
%   The correction factor eta_ae ≈ 0.25 captures residual over-expansion
%   losses at the spike tip at sea level (small compared to bell nozzle).
%
%   Inputs:
%     throttle2  — Engine 2 throttle setting [0.6, 1.0]
%     altitude   — Current altitude (m)
%     Pa         — Ambient pressure (Pa)
%     prop_cfg   — Propulsion config struct (from propulsion_config.m)
%
%   Outputs:
%     T2          — Aerospike thrust magnitude (N)
%     mdot2       — Mass flow rate (kg/s, positive = consumption)
%     F_ae_body   — Thrust force vector in body frame [3x1] (N)
%                   (Acts in +x body direction for axial engine)
%     valid       — true if engine is producing thrust

g0        = prop_cfg.g0;
eng2      = prop_cfg.eng2;

%% ========================================================================
%  THROTTLE LIMITS
%  ========================================================================

throttle2 = min(max(throttle2, eng2.throttle_min), eng2.throttle_max);

%% ========================================================================
%  ALTITUDE-COMPENSATED ISP
%  Isp_eff = Isp_vac * (1 - eta_ae * Pa/Pc)
%  At sea level (Pa=101325, Pc=8e6):
%    Isp_eff = Isp_vac * (1 - 0.25 * 101325/8e6) = 0.9968 * Isp_vac
%    → Only 0.3% loss at sea level (far better than bell nozzle ~10-15% loss)
%  ========================================================================

Isp_eff = eng2.Isp_vac * (1 - eng2.correction * Pa / eng2.Pc);
Isp_eff = max(Isp_eff, eng2.Isp_sl * 0.8);  % Floor at 80% of sea-level Isp

%% ========================================================================
%  MASS FLOW & THRUST
%  ========================================================================

% Reference mass flow at full throttle (derived from vacuum conditions)
mdot_ref = eng2.T_vac / (eng2.Isp_vac * g0);   % kg/s at full throttle

mdot2 = throttle2 * mdot_ref;                    % Actual mass flow (kg/s)
T2    = mdot2 * Isp_eff * g0;                    % Thrust (N)

%% ========================================================================
%  BODY-FRAME FORCE VECTOR
%  Aerospike is annular, centred on body x-axis → thrust in +x direction
%  (No thrust vector control on aerospike in this model; TVC is on engine 1)
%  ========================================================================

F_ae_body = [T2; 0; 0];

valid = (mdot2 > 0) && (T2 > 0);

end
