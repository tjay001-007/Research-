function vehicle = vehicle_config()
%VEHICLE_CONFIG  Returns vehicle parameters struct for the suborbital
%   winged rocket / lifting body.
%
%   Vehicle concept: Single-stage suborbital winged rocket with delta-wing
%   lifting body configuration (X-15 / SpaceShipTwo class).
%   Target: ~100 km apogee, Mach 5+ peak speed.
%
%   Coordinate system (body frame):
%     x: forward along vehicle centreline
%     y: right wing
%     z: down (belly of vehicle)
%
%   Returns:
%     vehicle  - Struct with all vehicle parameters

%% ========================================================================
%  MASS PROPERTIES (at launch, fully fuelled)
%  ========================================================================

vehicle.mass_total   = 5200;      % Gross liftoff mass (kg)
vehicle.mass_dry     = 1500;      % Dry mass — structure + avionics + engine (kg)
vehicle.mass_prop1   = 2200;      % Propellant mass, engine 1 (conventional) (kg)
vehicle.mass_prop2   = 1500;      % Propellant mass, engine 2 (aerospike) (kg)

%% ========================================================================
%  GEOMETRY & AERODYNAMIC REFERENCE
%  ========================================================================

vehicle.S  = 15.0;    % Wing reference area (m^2)
vehicle.b  = 6.8;     % Wing span (m)
vehicle.c  = 2.3;     % Mean aerodynamic chord (m)
vehicle.L  = 12.5;    % Vehicle total length (m)

%% ========================================================================
%  MOMENTS OF INERTIA (at launch CG, body axes)
%  Products of inertia: Ixy = Iyz = 0 (symmetric about xz-plane),
%  Ixz non-zero (asymmetric mass distribution fore-aft on lifting body)
%  ========================================================================

vehicle.Ixx = 4500;   % Roll MOI  (kg.m^2)
vehicle.Iyy = 85000;  % Pitch MOI (kg.m^2)  — large, long body
vehicle.Izz = 88000;  % Yaw MOI   (kg.m^2)
vehicle.Ixz = 1200;   % Roll-yaw product of inertia (kg.m^2)

%% ========================================================================
%  CG LOCATION (measured from nose, positive aft)
%  ========================================================================

vehicle.CG_initial   = 6.2;         % Initial CG from nose (m) — at launch
vehicle.CP_location  = 7.1;         % Centre of pressure from nose (m) at trim
%  Note: CP behind CG → aerodynamically stable in pitch at trim condition
%  Static margin = (CP - CG)/MAC = (7.1-6.2)/2.3 = 0.39 MACs (positive = stable)
%  However at high AoA or low q, vehicle becomes unstable requiring active control

%  CG shift model: as propellant burns, CG moves aft (aerospike tank is aft)
vehicle.CG_shift_per_kg_prop1 = +0.0008;  % CG shift per kg of prop1 burned (m/kg)
vehicle.CG_shift_per_kg_prop2 = +0.0015;  % CG shift per kg of prop2 burned (m/kg)

%% ========================================================================
%  INERTIA VARIATION WITH MASS
%  (Linear model: dI/dm, calibrated from structural analysis)
%  ========================================================================

vehicle.dIxx_dm = -0.4;   % Change in Ixx per kg of propellant burned (m^2)
vehicle.dIyy_dm = -12.0;  % Change in Iyy per kg of propellant burned (m^2)
vehicle.dIzz_dm = -12.5;  % Change in Izz per kg of propellant burned (m^2)
vehicle.dIxz_dm = -0.05;  % Change in Ixz per kg of propellant burned (m^2)

%% ========================================================================
%  CONTROL SURFACE LIMITS (physical deflection)
%  ========================================================================

vehicle.de_max  = 25.0;   % Max elevator deflection (deg)
vehicle.da_max  = 20.0;   % Max aileron deflection  (deg)
vehicle.dr_max  = 20.0;   % Max rudder deflection   (deg)
vehicle.de_rate = 60.0;   % Max elevator rate (deg/s)
vehicle.da_rate = 80.0;   % Max aileron rate  (deg/s)
vehicle.dr_rate = 60.0;   % Max rudder rate   (deg/s)

%  Surface effectiveness scaling (linear to rad conversion used in aero model)
vehicle.de_to_rad = deg2rad(vehicle.de_max);  % normalised [-1,1] -> [rad]
vehicle.da_to_rad = deg2rad(vehicle.da_max);
vehicle.dr_to_rad = deg2rad(vehicle.dr_max);

%% ========================================================================
%  THRUST VECTOR CONTROL (TVC) — on main nozzle(s)
%  ========================================================================

vehicle.tvc_max       = 8.0;    % Max TVC gimbal angle (deg)
vehicle.tvc_rate_max  = 30.0;   % Max TVC slew rate (deg/s)
vehicle.l_tvc         = 5.8;    % Nozzle exit plane to initial CG (m, positive aft of CG)
%  When propellant burns, CG moves aft → l_tvc decreases
%  l_tvc_eff = vehicle.l_tvc - CG_shift (computed in tvc_model.m)

%% ========================================================================
%  REACTION CONTROL SYSTEM (RCS)
%  ========================================================================

vehicle.rcs_thrust        = 500;   % Single thruster force (N)
vehicle.rcs_n_thrusters   = 12;    % Total number of thrusters (6 pairs)

%  Moment arms from vehicle CG (m)
vehicle.l_rcs_pitch = 5.0;   % Pitch RCS thruster moment arm from CG (m)
vehicle.l_rcs_yaw   = 5.0;   % Yaw RCS thruster moment arm from CG (m)
vehicle.l_rcs_roll  = 2.5;   % Roll RCS thruster moment arm from CG (m)

%  RCS activation thresholds
vehicle.rcs_att_deadband  = deg2rad(0.5);   % Fire if attitude error > 0.5 deg (rad)
vehicle.rcs_rate_deadband = 0.05;           % Fire if rate error > 0.05 rad/s

%% ========================================================================
%  AEROSPIKE ENGINE LOCATION (for moment arm calculation)
%  ========================================================================

vehicle.aerospike_exit_plane = 12.0;  % Aerospike exit plane from nose (m, at tail)
%  Aerospike moment arm (for thrust offset) computed relative to current CG

%% ========================================================================
%  STRUCTURAL LIMITS
%  ========================================================================

vehicle.qbar_max      = 80000;    % Max dynamic pressure (Pa) — structural limit
vehicle.alpha_max_deg = 20.0;     % Max AoA (deg) — aerodynamic structural limit
vehicle.nz_max        = 5.0;      % Max normal load factor (g)

%% ========================================================================
%  DISPLAY SUMMARY
%  ========================================================================

fprintf('\n[Vehicle] Config loaded: Suborbital Winged Rocket\n');
fprintf('  Gross mass:     %.0f kg (dry: %.0f kg, prop: %.0f kg)\n', ...
    vehicle.mass_total, vehicle.mass_dry, ...
    vehicle.mass_prop1 + vehicle.mass_prop2);
fprintf('  Ref area: %.1f m^2, Span: %.1f m, MAC: %.1f m\n', ...
    vehicle.S, vehicle.b, vehicle.c);
fprintf('  CG (initial): %.2f m from nose\n', vehicle.CG_initial);
fprintf('  MOI: Ixx=%.0f, Iyy=%.0f, Izz=%.0f kg.m^2\n', ...
    vehicle.Ixx, vehicle.Iyy, vehicle.Izz);

end
