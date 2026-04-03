%% EXAMPLE_VEHICLE_DATA  Reference data and notes for the default vehicle.
%
%   This script documents the physical basis for the default vehicle
%   parameters in vehicle_config.m and propulsion_config.m.
%
%   Vehicle concept: "Velos-1" — a single-stage suborbital winged rocket
%   in the X-15 / SpaceShipTwo class.
%
%   =======================================================================
%   VEHICLE DESCRIPTION
%   =======================================================================
%
%   Role:        Suborbital science / tourism / hypersonic test vehicle
%   Target:      100 km apogee (Karman line), Mach 5+ peak speed
%   Config:      Winged lifting body with delta wing, ventral fins
%   Propulsion:  Dual — conventional bell nozzle (boost) +
%                       annular aerospike (mid-to-high altitude)
%   Control:     Elevons + rudder + TVC (on bell nozzle) + cold-gas RCS
%   Crew:        Unmanned (autonomous GNC)
%
%   =======================================================================
%   MASS BUDGET
%   =======================================================================

fprintf('\n=== VELOS-1 MASS BUDGET ===\n');
fprintf('Gross Liftoff Mass (GLOM): 5,200 kg\n');
fprintf('  Structure + thermal:       650 kg\n');
fprintf('  Engines (both):            350 kg\n');
fprintf('  Avionics + GNC:             80 kg\n');
fprintf('  Payload + misc:            170 kg\n');
fprintf('  Landing gear / recovery:   250 kg\n');
fprintf('  -------------------------\n');
fprintf('  Total dry mass:          1,500 kg\n');
fprintf('  -------------------------\n');
fprintf('  Engine 1 propellant:     2,200 kg (LOX/RP-1 at OF=2.6)\n');
fprintf('  Engine 2 propellant:     1,500 kg (LOX/LH2 at OF=5.0)\n');
fprintf('  Total propellant:        3,700 kg  (71.2%% of GLOM)\n\n');

%   =======================================================================
%   GEOMETRIC PROPERTIES (from vehicle_config.m)
%   =======================================================================

fprintf('=== GEOMETRY ===\n');
fprintf('Overall length:            12.5 m\n');
fprintf('Wing span:                  6.8 m\n');
fprintf('Wing reference area:       15.0 m^2\n');
fprintf('Mean aerodynamic chord:     2.3 m\n');
fprintf('Wing sweep (estimated):    ~65 degrees (delta configuration)\n');
fprintf('Nose fineness ratio:       ~5 (low wave drag at hypersonic)\n\n');

%   =======================================================================
%   CG / CP STABILITY
%   =======================================================================

fprintf('=== CG & STATIC STABILITY ===\n');
fprintf('CG at launch (from nose):   6.2 m\n');
fprintf('CP location (trim alpha):   7.1 m\n');
fprintf('Static margin (subsonic):   +0.39 MAC (stable in pitch)\n');
fprintf('Static margin variation:\n');
fprintf('  - Subsonic:     Positive → aerodynamically stable pitch\n');
fprintf('  - Transonic:    Reduces → requires active TVC pitch control\n');
fprintf('  - Supersonic/hypersonic: CP moves aft → stable again\n');
fprintf('  - At propellant burnout: CG shifts aft ~1.5 m → reduced margin\n\n');
fprintf('NOTE: LQR gain scheduling covers CG shift effects.\n');
fprintf('NOTE: This vehicle is pitch-UNSTABLE at low dynamic pressure (pad to ~Mach 0.3)\n');
fprintf('      and during transonic transition — active control essential.\n\n');

%   =======================================================================
%   PROPULSION COMPARISON
%   =======================================================================

fprintf('=== PROPULSION COMPARISON ===\n');
fprintf('                    Bell Nozzle    Aerospike\n');
fprintf('                    -----------    ---------\n');
fprintf('Propellant:         LOX/RP-1       LOX/LH2\n');
fprintf('Thrust (vac, kN):   220            150\n');
fprintf('Thrust (SL, kN):    180            128*\n');
fprintf('Isp (vac, s):       320            350\n');
fprintf('Isp (SL, s):        265            300*\n');
fprintf('Chamber pressure:   70 bar         80 bar\n');
fprintf('Nozzle type:        Fixed bell      Annular spike\n');
fprintf('TVC:                Yes (±8°)      No\n');
fprintf('Altitude adapt:     None           Self-compensating\n');
fprintf('\n');
fprintf('* Aerospike SL performance nearly as good as vacuum — only ~0.3%% loss\n');
fprintf('  at sea level (vs. ~15%% loss for bell nozzle at sea level vs vacuum)\n\n');

%   =======================================================================
%   AEROSPIKE IGNITION TRADE STUDY
%   =======================================================================

fprintf('=== AEROSPIKE IGNITION ALTITUDE TRADE ===\n');
fprintf('h_ignite    Benefit                        Drawback\n');
fprintf('--------    -------                        --------\n');
fprintf('15 km:     More total aerospike burn time  Dense air, higher drag, heat flux\n');
fprintf('25 km:     Reduced atmospheric drag losses  Less aerospike burn time\n');
fprintf('40 km:     Thin air, low heat flux          Very short aerospike phase\n');
fprintf('55 km:     Near-vacuum Isp from start       Minimal duration, wasted prop\n');
fprintf('\nOptimal ignition altitude: ~20-35 km (GA will determine exact value)\n');
fprintf('Default: 25 km\n\n');

%   =======================================================================
%   CONTROL EFFECTIVENESS AT KEY FLIGHT CONDITIONS
%   =======================================================================

fprintf('=== CONTROL AUTHORITY SUMMARY ===\n');
fprintf('Condition         Elevator     TVC       RCS\n');
fprintf('---------         --------     ---       ---\n');
fprintf('Launch (0 m/s):   None*        Full      Full\n');
fprintf('Subsonic (q~10kPa):  Good      Good      Backup\n');
fprintf('Max-q (~50kPa):   Max          Good      Not needed\n');
fprintf('Transonic:        Reduced      Primary   Backup\n');
fprintf('Supersonic:       Reduced      Reduced   Primary\n');
fprintf('Post-burnout:     Minimal      None      Primary\n');
fprintf('\n* Control surfaces ineffective below ~30 m/s — TVC + RCS primary during launch\n\n');

%   =======================================================================
%   SIMULATION VALIDATION CHECKS
%   =======================================================================

fprintf('=== QUICK VALIDATION CHECKS ===\n');
fprintf('Run main_6dof_simulation.m with default settings and verify:\n');
fprintf('  1. Apogee altitude >= 80 km  (target: 100 km)\n');
fprintf('  2. Peak speed >= 1500 m/s   (target: Mach 5 ~ 1650 m/s)\n');
fprintf('  3. Peak qbar <= 75 kPa      (structural limit)\n');
fprintf('  4. Quaternion norm ≈ 1.0 throughout (numerical integrity)\n');
fprintf('  5. Vehicle mass monotonically decreases during burn\n');
fprintf('  6. CG shifts aft by ~1.0-1.5 m over full burn\n');
fprintf('  7. Control surfaces effective after ~5s post-launch\n\n');

fprintf('See also: example_trajectory.m for reference optimised trajectory data.\n\n');
