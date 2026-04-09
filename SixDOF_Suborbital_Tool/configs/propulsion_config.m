function prop = propulsion_config()
%PROPULSION_CONFIG  Returns dual propulsion system parameters.
%
%   Engine 1: Conventional bell-nozzle rocket (active from T=0)
%   Engine 2: Aerospike rocket (ignites at optimised altitude, ~20-50 km)
%
%   The aerospike is altitude-compensating: its effective exhaust pressure
%   self-adjusts to ambient, so Pe ≈ Pa across a wide altitude range.
%   This gives near-vacuum Isp performance from high altitude through burnout.
%
%   Thrust model:
%     Engine 1 (bell):     T1 = T1_sl + (T1_vac - T1_sl) * (1 - Pa/Pa_sl)
%     Engine 2 (aerospike): T2 = mdot2 * Isp_eff * g0
%                            Isp_eff = Isp_vac * (1 - 0.25*(Pa/Pc2))
%
%   Returns:
%     prop  - Struct with engine 1 and engine 2 parameters

g0 = 9.80665;   % Standard gravity (m/s^2)

%% ========================================================================
%  ENGINE 1 — CONVENTIONAL BELL-NOZZLE ROCKET
%  (Main boost engine, centreline-mounted, gimballed for TVC)
%  ========================================================================

prop.eng1.name         = 'Main Bell Nozzle';
prop.eng1.T_sl         = 180000;    % Sea-level thrust (N)  [~40,400 lbf]
prop.eng1.T_vac        = 220000;    % Vacuum thrust (N)
prop.eng1.Isp_sl       = 265;       % Sea-level specific impulse (s)
prop.eng1.Isp_vac      = 320;       % Vacuum specific impulse (s)
prop.eng1.mdot_max     = prop.eng1.T_vac / (prop.eng1.Isp_vac * g0);  % Max mass flow (kg/s)
prop.eng1.nozzle_area  = 0.45;      % Nozzle exit area (m^2)
prop.eng1.Pc           = 7e6;       % Chamber pressure (Pa)
prop.eng1.throttle_min = 0.40;      % Minimum throttle setting (40%)
prop.eng1.throttle_max = 1.00;      % Maximum throttle setting (100%)
prop.eng1.burn_time    = 80;        % Max burn duration (s) at full throttle

%  Thrust offset from vehicle centreline (for moment calculation)
prop.eng1.r_offset     = [0; 0; 0];  % On centreline [x;y;z] body frame (m)

%% ========================================================================
%  ENGINE 2 — AEROSPIKE ROCKET
%  (Altitude-compensating, aft-mounted, ignites at h_ignite)
%  The aerospike nozzle wraps around the base of the vehicle (annular spike).
%  ========================================================================

prop.eng2.name         = 'Aerospike';
prop.eng2.T_vac        = 150000;    % Vacuum thrust (N) — aerospike reference
prop.eng2.T_sl         = 128000;    % Effective sea-level thrust (N) (over-expanded loss)
prop.eng2.Isp_vac      = 350;       % Vacuum Isp (s) — higher than bell nozzle
prop.eng2.Isp_sl       = 300;       % Effective sea-level Isp (altitude compensated)
prop.eng2.mdot_max     = prop.eng2.T_vac / (prop.eng2.Isp_vac * g0);  % kg/s
prop.eng2.Pc           = 8e6;       % Chamber pressure (Pa)
prop.eng2.correction   = 0.25;      % Altitude compensation factor: eta_ae
%   T2 = mdot2 * Isp_eff * g0,  Isp_eff = Isp_vac*(1 - eta_ae * Pa/Pc)
%   At Pa=0 (vacuum):  Isp_eff = Isp_vac      (full vacuum performance)
%   At Pa=101325 Pa:   Isp_eff = Isp_vac*(1 - 0.25*101325/8e6) ≈ 0.997*Isp_vac
%   → Minimal sea-level loss; aerospike self-compensates
prop.eng2.throttle_min = 0.60;      % Min throttle (aerospike has narrower range)
prop.eng2.throttle_max = 1.00;
prop.eng2.burn_time    = 60;        % Max burn duration (s)

%  Aerospike ignition altitude (default — GA will optimise this)
prop.eng2.h_ignite_default = 25000;   % Default ignition altitude (m) — 25 km
prop.eng2.h_ignite_min     = 15000;   % Min allowed ignition altitude (m)
prop.eng2.h_ignite_max     = 55000;   % Max allowed ignition altitude (m)

%  Thrust offset: aerospike is annular, centred on body x-axis
prop.eng2.r_offset     = [0; 0; 0];  % On centreline [x;y;z] body frame (m)

%% ========================================================================
%  COMBINED SYSTEM PROPERTIES
%  ========================================================================

prop.g0 = g0;

%  Propellant masses (must match vehicle_config)
prop.m_prop1_total = 2200;   % Engine 1 propellant (kg)
prop.m_prop2_total = 1500;   % Engine 2 propellant (kg)

%  Staging / engine sequencing
prop.t_ignition_eng1    = 0.0;   % Engine 1 ignition time (s) — at launch
prop.transition_window  = 5.0;   % Both engines on simultaneously for (s) after aerospike ignition

%  Sea-level ambient pressure for thrust lapse model
prop.Pa_sl = 101325;   % Pa

%% ========================================================================
%  DISPLAY SUMMARY
%  ========================================================================

fprintf('\n[Propulsion] Dual engine system:\n');
fprintf('  Engine 1 (Bell):     T_vac=%.0f kN, Isp_vac=%.0f s\n', ...
    prop.eng1.T_vac/1e3, prop.eng1.Isp_vac);
fprintf('  Engine 2 (Aerospike): T_vac=%.0f kN, Isp_vac=%.0f s\n', ...
    prop.eng2.T_vac/1e3, prop.eng2.Isp_vac);
fprintf('  Aerospike ignition: default %.0f km (optimisable %.0f–%.0f km)\n', ...
    prop.eng2.h_ignite_default/1e3, ...
    prop.eng2.h_ignite_min/1e3, prop.eng2.h_ignite_max/1e3);
fprintf('  Total propellant: %.0f kg\n', prop.m_prop1_total + prop.m_prop2_total);

end
