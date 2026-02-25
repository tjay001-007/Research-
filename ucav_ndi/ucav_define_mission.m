%% ========================================================================
%  UCAV MISSION TRAJECTORY DEFINITION
%  ========================================================================
%
%  MODULE PURPOSE:
%  ---------------
%  This module defines the reference trajectory that the unmanned combat
%  aerial vehicle (UCAV) must follow. The trajectory is specified as an
%  ordered sequence of waypoints in the WGS-84 geodetic coordinate system
%  (latitude, longitude, altitude) — the same coordinate system used by
%  GPS receivers and all modern navigation systems.
%
%  In a real UCAV system, this trajectory would come from:
%    - A Ground Control Station (GCS) mission planner
%    - An onboard mission computer (pre-programmed before takeoff)
%    - A tactical data link (updated in flight via Link-16 or similar)
%    - An autonomous mission planning algorithm
%
%  COORDINATE SYSTEMS:
%  -------------------
%  Three coordinate frames are used in this simulation:
%
%  1. WGS-84 GEODETIC (latitude, longitude, altitude)
%     - The global reference frame used by GPS
%     - latitude:  angle north of equator (degrees)
%     - longitude: angle east of prime meridian (degrees)
%     - altitude:  height above WGS-84 ellipsoid (metres)
%     - Used for: mission planning, waypoint definition, GCS interface
%
%  2. LOCAL NED (North, East, Down)
%     - A flat-earth tangent plane centred at a reference point
%     - North: positive toward geographic north (metres)
%     - East:  positive toward geographic east (metres)
%     - Down:  positive toward earth centre (metres); altitude = -Down
%     - Used for: guidance law, position tracking, 6-DOF simulation
%     - Valid approximation within ~100 km of reference point
%
%  3. BODY FRAME (x_b, y_b, z_b)
%     - Fixed to the aircraft centre of gravity
%     - x_b: forward through the nose
%     - y_b: out the right wing
%     - z_b: downward through the belly
%     - Used for: aerodynamics, equations of motion, control surfaces
%
%  TRAJECTORY STRUCTURE:
%  ---------------------
%  Each waypoint contains:
%    - latitude (deg), longitude (deg), altitude_msl (m)
%    - commanded_airspeed (m/s)
%    - waypoint_type: 'flyby' (smooth through) or 'flyover' (must pass over)
%    - loiter_time: seconds to orbit at this waypoint (0 = transit)
%
%  The guidance law interpolates between consecutive waypoints using
%  great-circle (or flat-earth) segments. The aircraft flies from WP_i
%  to WP_{i+1} and transitions to the next segment when the along-track
%  distance to WP_{i+1} is less than the acceptance radius.
%
%  EXAMPLE MISSION:
%  ----------------
%  An ISR (Intelligence, Surveillance, Reconnaissance) patrol mission:
%
%    Phase 1: Takeoff and climb from airfield
%    Phase 2: Transit to patrol area at cruise altitude
%    Phase 3: Racetrack surveillance pattern (two laps)
%    Phase 4: Transit back and descend
%    Phase 5: Approach to airfield
%
%  Total mission duration: approximately 20-30 minutes
%  Total distance: approximately 80 km
%
%  OUTPUT:
%  -------
%  Creates 'mission' struct in workspace with fields:
%    mission.waypoints      - Table of waypoints (lat, lon, alt, speed, type)
%    mission.n_waypoints    - Number of waypoints
%    mission.ref_lat        - Reference latitude for NED origin (deg)
%    mission.ref_lon        - Reference longitude for NED origin (deg)
%    mission.ref_alt        - Reference altitude for NED origin (m)
%    mission.wp_ned         - Waypoints converted to NED [N, E, D] (m)
%    mission.wp_speed       - Commanded speed at each waypoint (m/s)
%    mission.acceptance_rad - Waypoint acceptance radius (m)
%
%  ========================================================================

function mission = ucav_define_mission()

fprintf('==========================================================\n');
fprintf(' UCAV MISSION TRAJECTORY DEFINITION\n');
fprintf(' ISR Patrol Mission — Racetrack Pattern\n');
fprintf('==========================================================\n\n');

%% ====================================================================
%  REFERENCE ORIGIN
%  ====================================================================
%  The NED frame origin is placed at the takeoff airfield.
%  All internal navigation is done in NED relative to this point.
%
%  In a real system, the reference origin is set at:
%    - GPS position at power-on (for tactical operations)
%    - A pre-defined survey point at the home airfield
%    - The catapult/launcher position for smaller UCAVs

mission.ref_lat = 12.9700;    % degrees North (generic airfield location)
mission.ref_lon = 80.2200;    % degrees East
mission.ref_alt = 15.0;       % metres above MSL (airfield elevation)

fprintf('Reference Origin (NED Frame):\n');
fprintf('  Latitude:  %.4f deg N\n', mission.ref_lat);
fprintf('  Longitude: %.4f deg E\n', mission.ref_lon);
fprintf('  Altitude:  %.1f m MSL\n\n', mission.ref_alt);

%% ====================================================================
%  WAYPOINT DEFINITION (WGS-84 Geodetic)
%  ====================================================================
%
%  MISSION PROFILE:
%
%                    WP4 ─────────────── WP5
%                     │   Patrol Leg 1    │
%                     │   (4500m, 180m/s) │
%                     │                   │
%                    WP7 ─────────────── WP6
%                     │   Patrol Leg 2    │
%                     │   (Return leg)    │
%                     │                   │
%                    WP3
%                   ╱    (Patrol entry, 4500m)
%                 ╱
%               WP2       (Transit climb, 3000m)
%             ╱
%           WP1           (After takeoff, 500m)
%           │
%          WP0            (Runway, 15m)    ← HOME
%           │
%           WP1
%             ╲
%               WP8       (Return descent, 2000m)
%                 ╲
%                   WP9   (Approach, 500m)
%                     ╲
%                      WP10 (Final, 100m) ← HOME

%  Column format: [lat_deg, lon_deg, alt_msl_m, speed_m_s]
%
%  Design rationale for each waypoint:
%
%  WP0:  Takeoff roll completion / catapult launch point
%        At airfield elevation, initial climb speed
%
%  WP1:  End of initial climb, turn toward patrol area
%        500m AGL gives terrain clearance for departure
%
%  WP2:  Transit waypoint — aircraft climbs to cruise altitude
%        3000m is a typical transit altitude for medium-altitude UCAV
%
%  WP3:  Patrol area entry — begin surveillance pattern
%        4500m is optimal for ISR sensor coverage area
%
%  WP4-WP7: Racetrack pattern
%        Four corners of a rectangular orbit
%        Legs are ~5.5 km long (30 seconds at 180 m/s)
%        Pattern width ~5.5 km (provides overlap for sensor swath)
%        The aircraft will fly WP4→WP5→WP6→WP7→WP4 repeatedly
%        (guidance law handles the looping)
%
%  WP8:  Egress from patrol area, begin descent
%        Same track as WP2 (return along ingress route)
%
%  WP9:  Approach waypoint — lined up with runway
%        500m altitude, slowed to approach speed
%
%  WP10: Short final — low altitude, slow speed
%        Hands off to autoland system (not modelled here)

waypoints = [
%   Lat (deg)   Lon (deg)   Alt (m MSL)  Speed (m/s)
    12.9700,    80.2200,    15,          70;     % WP0:  Takeoff point (runway)
    12.9750,    80.2250,    500,         120;    % WP1:  Initial climb
    13.0000,    80.2600,    3000,        180;    % WP2:  Transit / climb
    13.0400,    80.3100,    4500,        180;    % WP3:  Patrol entry
    13.0900,    80.3100,    4500,        180;    % WP4:  Racetrack corner 1 (NW)
    13.0900,    80.3600,    4500,        180;    % WP5:  Racetrack corner 2 (NE)
    13.0400,    80.3600,    4500,        180;    % WP6:  Racetrack corner 3 (SE)
    13.0400,    80.3100,    4500,        180;    % WP7:  Racetrack corner 4 (SW) = WP3
    13.0000,    80.2600,    2000,        170;    % WP8:  Return / descent
    12.9750,    80.2280,    500,         100;    % WP9:  Approach
    12.9710,    80.2210,    100,         80;     % WP10: Short final
];

mission.n_waypoints = size(waypoints, 1);
mission.wp_lat   = waypoints(:, 1);
mission.wp_lon   = waypoints(:, 2);
mission.wp_alt   = waypoints(:, 3);
mission.wp_speed = waypoints(:, 4);

%% ====================================================================
%  WAYPOINT TYPE AND ACCEPTANCE RADIUS
%  ====================================================================
%
%  acceptance_radius: When the aircraft is within this distance of the
%  target waypoint (in the horizontal plane), the guidance switches to
%  the next waypoint segment. This prevents the aircraft from trying to
%  hit the exact point (which would cause overshoot and orbit).
%
%  For a fighter-class UCAV at 180 m/s:
%    - Minimum turn radius at 60 deg bank: R = V^2/(g*tan(phi)) ≈ 5.6 km
%    - At 45 deg bank: R ≈ 3.3 km
%  The acceptance radius should be comparable to the turn radius so the
%  aircraft begins turning before reaching the waypoint.
%
%  Typical values:
%    - Waypoint acceptance radius: 500-2000 m (depending on speed/agility)
%    - For the racetrack: tighter radius since we want neat corners

mission.acceptance_rad = 800;   % metres (default for transit waypoints)

% Racetrack corners get tighter acceptance to keep the pattern clean
mission.wp_accept = ones(mission.n_waypoints, 1) * mission.acceptance_rad;
mission.wp_accept(4:7) = 500;   % Tighter at racetrack corners

% Racetrack looping: after reaching WP7, go back to WP4 for another lap
% The guidance law handles this via mission.racetrack_start/end
mission.racetrack_start = 4;    % WP index where racetrack begins (WP4, 1-indexed)
mission.racetrack_end   = 8;    % WP index after last racetrack corner (WP7+1)
mission.racetrack_laps  = 2;    % Number of laps before egress

fprintf('Mission Waypoints:\n');
fprintf('  %-4s  %-10s  %-10s  %-8s  %-8s  %-8s\n', ...
    'WP', 'Lat(deg)', 'Lon(deg)', 'Alt(m)', 'Spd(m/s)', 'Accept(m)');
fprintf('  %s\n', repmat('-', 1, 58));
for i = 1:mission.n_waypoints
    fprintf('  WP%-2d  %9.4f  %9.4f  %6.0f   %6.0f    %6.0f\n', ...
        i-1, waypoints(i,1), waypoints(i,2), waypoints(i,3), ...
        waypoints(i,4), mission.wp_accept(i));
end
fprintf('\n');

%% ====================================================================
%  COORDINATE CONVERSION:  WGS-84 (lat, lon, alt) → NED (N, E, D)
%  ====================================================================
%
%  MATHEMATICAL BACKGROUND:
%  The conversion from geodetic to local tangent plane uses the flat-earth
%  approximation, which is accurate to within 0.1% for distances up to
%  ~100 km from the reference point.
%
%  The WGS-84 ellipsoid has:
%    Semi-major axis:  a = 6378137.0 m
%    Flattening:       f = 1/298.257223563
%    Eccentricity^2:   e^2 = 2f - f^2
%
%  At a given latitude, the radii of curvature are:
%    R_N = a(1-e^2) / (1 - e^2*sin^2(lat))^(3/2)   (meridional, N-S)
%    R_E = a / (1 - e^2*sin^2(lat))^(1/2)           (prime vertical, E-W)
%
%  Conversion:
%    North = (lat - lat_ref) * R_N                   (radians!)
%    East  = (lon - lon_ref) * R_E * cos(lat_ref)    (radians!)
%    Down  = -(alt - alt_ref)
%
%  For higher accuracy (e.g., precision approach), one would use the
%  full ECEF intermediate conversion. For en-route navigation at the
%  distances in this mission (~50 km), flat-earth is adequate.

% WGS-84 ellipsoid parameters
a_wgs84 = 6378137.0;                           % Semi-major axis (m)
f_wgs84 = 1 / 298.257223563;                   % Flattening
e2_wgs84 = 2*f_wgs84 - f_wgs84^2;             % Eccentricity squared

% Compute radii of curvature at the reference latitude
sin_lat0 = sin(deg2rad(mission.ref_lat));
denom    = sqrt(1 - e2_wgs84 * sin_lat0^2);
R_N      = a_wgs84 * (1 - e2_wgs84) / denom^3;    % Meridional radius
R_E      = a_wgs84 / denom;                         % Prime vertical radius

mission.R_N = R_N;
mission.R_E = R_E;

% Convert each waypoint from LLA to NED
mission.wp_ned = zeros(mission.n_waypoints, 3);
for i = 1:mission.n_waypoints
    dlat = deg2rad(mission.wp_lat(i) - mission.ref_lat);
    dlon = deg2rad(mission.wp_lon(i) - mission.ref_lon);
    dalt = mission.wp_alt(i) - mission.ref_alt;

    N_i =  dlat * R_N;                              % North (m)
    E_i =  dlon * R_E * cos(deg2rad(mission.ref_lat)); % East (m)
    D_i = -dalt;                                     % Down (m), negative = above ref

    mission.wp_ned(i, :) = [N_i, E_i, D_i];
end

fprintf('Waypoints in NED (relative to reference origin):\n');
fprintf('  %-4s  %10s  %10s  %10s\n', 'WP', 'North(m)', 'East(m)', 'Alt(m)');
fprintf('  %s\n', repmat('-', 1, 40));
for i = 1:mission.n_waypoints
    fprintf('  WP%-2d  %9.0f  %9.0f  %9.0f\n', i-1, ...
        mission.wp_ned(i,1), mission.wp_ned(i,2), -mission.wp_ned(i,3));
end
fprintf('\n');

%% ====================================================================
%  MISSION DISTANCE AND TIME ESTIMATES
%  ====================================================================

total_dist = 0;
for i = 2:mission.n_waypoints
    seg_dist = norm(mission.wp_ned(i, 1:2) - mission.wp_ned(i-1, 1:2));
    total_dist = total_dist + seg_dist;
end

% Add racetrack laps
rt_perim = 0;
for i = mission.racetrack_start : (mission.racetrack_end - 1)
    i_next = i + 1;
    if i_next > mission.racetrack_end
        i_next = mission.racetrack_start;
    end
    rt_perim = rt_perim + norm(mission.wp_ned(i_next,1:2) - mission.wp_ned(i,1:2));
end
total_dist = total_dist + rt_perim * (mission.racetrack_laps - 1);

avg_speed = mean(mission.wp_speed);
est_time  = total_dist / avg_speed;

mission.total_distance_km = total_dist / 1000;
mission.estimated_time_min = est_time / 60;

fprintf('Mission Estimates:\n');
fprintf('  Total distance:  %.1f km\n', mission.total_distance_km);
fprintf('  Average speed:   %.0f m/s\n', avg_speed);
fprintf('  Estimated time:  %.1f min\n\n', mission.estimated_time_min);

fprintf('==========================================================\n\n');

end
