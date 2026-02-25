function [phi_cmd, theta_cmd, throttle_cmd, guid_state_out, guid_debug] = ...
    ucav_guidance_law( ...
        pos_n, pos_e, pos_d, ...
        u, v, w, ...
        phi, theta, psi, ...
        V, ...
        guid_state_in, ...
        mission, guidance, dt)
%UCAV_GUIDANCE_LAW  Autonomous guidance for waypoint-following UCAV.
%
%  MODULE PURPOSE:
%  ---------------
%  This module converts a sequence of geographic waypoints into the three
%  commands that the NDI autopilot needs:
%
%    phi_cmd    - Commanded bank angle (rad)
%                 Controls the lateral flight path (heading changes)
%    theta_cmd  - Commanded pitch angle (rad)
%                 Controls the vertical flight path (climb/descend)
%    throttle_cmd - Commanded throttle (0 to 1.5)
%                   Controls the airspeed
%
%  It implements THREE sub-controllers:
%
%  1. LATERAL GUIDANCE — L1 Nonlinear Path Following
%     Computes the bank angle needed to track the line between waypoints.
%     Based on: Park, Deyst & How, "A New Nonlinear Guidance Logic for
%     Trajectory Tracking," AIAA GNC 2004.
%     This is the SAME guidance law used in PX4 and ArduPilot autopilots.
%
%  2. LONGITUDINAL GUIDANCE — TECS-inspired altitude + speed controller
%     Manages the total energy (kinetic + potential) to simultaneously
%     control altitude and airspeed using pitch and throttle.
%     Based on: Lambregts, "Automatic Flight Control — Concepts and
%     Methods," NASA/FAA report, 1983.
%
%  3. WAYPOINT SEQUENCING
%     Determines which waypoint segment the aircraft should track,
%     handles waypoint switching, and manages racetrack looping.
%
%  DESIGN PHILOSOPHY (for real flight):
%  ------------------------------------
%  Every computation in this module must be:
%    - Numerically stable (no division by zero, bounded outputs)
%    - Physically meaningful (outputs are within aircraft capability)
%    - Tested at envelope boundaries (low speed, high altitude, steep turns)
%    - Deterministic (same inputs always produce same outputs)
%
%  Inputs:
%    pos_n, pos_e, pos_d - Aircraft position in NED frame (m)
%    u, v, w             - Body velocities (m/s)
%    phi, theta, psi     - Euler angles (rad)
%    V                   - Total airspeed (m/s)
%    guid_state_in       - Persistent guidance state (waypoint index, integ.)
%    mission             - Mission struct from ucav_define_mission()
%    guidance            - Guidance parameter struct
%    dt                  - Time step (s)
%
%  Outputs:
%    phi_cmd       - Commanded bank angle (rad)
%    theta_cmd     - Commanded pitch angle (rad)
%    throttle_cmd  - Commanded throttle (0 to 1.5)
%    guid_state_out - Updated guidance state
%    guid_debug    - Debug signals for telemetry
%
%  ========================================================================

g = 9.81;

%% ====================================================================
%  SECTION 1: WAYPOINT SEQUENCING
%  ====================================================================
%
%  The sequencer determines which two waypoints define the current path
%  segment (WP_from → WP_to). It switches to the next waypoint when the
%  aircraft enters the acceptance radius of WP_to.
%
%  Acceptance condition (horizontal only — altitude tracked separately):
%    dist_to_wp < acceptance_radius
%
%  For racetrack patterns, the sequencer loops back to the racetrack
%  start waypoint after completing a lap.

wp_idx = guid_state_in.wp_idx;          % Current target waypoint index
lap_count = guid_state_in.lap_count;    % Completed racetrack laps

% Ensure wp_idx is within bounds
wp_idx = max(2, min(wp_idx, mission.n_waypoints));

% Current and previous waypoint positions (NED)
wp_from = mission.wp_ned(wp_idx - 1, :)';   % [N; E; D]
wp_to   = mission.wp_ned(wp_idx, :)';

% Horizontal distance to target waypoint
pos_horiz = [pos_n; pos_e];
wp_to_horiz = wp_to(1:2);
dist_to_wp = norm(pos_horiz - wp_to_horiz);

% Waypoint switching logic
wp_switched = false;
accept_rad = mission.wp_accept(wp_idx);

if dist_to_wp < accept_rad
    % Aircraft has reached the current target waypoint

    % Check if we're in the racetrack and need to loop
    if wp_idx == mission.racetrack_end && ...
       lap_count < mission.racetrack_laps

        % Loop back to racetrack start for another lap
        wp_idx = mission.racetrack_start;
        lap_count = lap_count + 1;
        wp_switched = true;

    elseif wp_idx < mission.n_waypoints
        % Advance to next waypoint
        wp_idx = wp_idx + 1;
        wp_switched = true;
    end
    % else: at final waypoint — hold position (orbit)

    if wp_switched
        wp_from = mission.wp_ned(wp_idx - 1, :)';
        wp_to   = mission.wp_ned(wp_idx, :)';
    end
end

% Target airspeed from mission plan
V_cmd = mission.wp_speed(wp_idx);

% Target altitude from mission plan (interpolate along segment)
alt_from = -mission.wp_ned(max(1, wp_idx-1), 3);   % altitude = -Down
alt_to   = -wp_to(3);

% Compute progress along current segment for altitude interpolation
seg_vec   = wp_to(1:2) - wp_from(1:2);
seg_len   = norm(seg_vec);
if seg_len > 1.0
    pos_vec    = pos_horiz - wp_from(1:2);
    along_frac = dot(pos_vec, seg_vec) / (seg_len^2);
    along_frac = max(0, min(1, along_frac));
else
    along_frac = 1.0;
end

% Linearly interpolate desired altitude along segment
alt_cmd = alt_from + along_frac * (alt_to - alt_from);

%% ====================================================================
%  SECTION 2: L1 LATERAL GUIDANCE LAW
%  ====================================================================
%
%  THEORY:
%  -------
%  The L1 guidance law generates a lateral acceleration command to make
%  the aircraft converge onto the line between WP_from and WP_to.
%
%  The key idea is to define a reference point on the desired path at a
%  lookahead distance L1 ahead of the aircraft. The guidance then commands
%  a lateral acceleration to curve toward that point.
%
%  MATHEMATICS:
%  1. Define the desired path vector: d_hat = (WP_to - WP_from) / |...|
%  2. Compute crosstrack error: e_cross = (pos - WP_from) x d_hat
%  3. Compute the L1 reference point on the path
%  4. Bearing angle eta = angle between velocity and line to L1 point
%  5. Lateral acceleration: a_cmd = 2 * V^2 * sin(eta) / L1
%  6. Bank angle: phi_cmd = atan(a_cmd / g)
%
%  The L1 distance determines the tracking bandwidth:
%    - Smaller L1 → tighter tracking, more aggressive turns
%    - Larger L1 → smoother tracking, gentler turns
%    - L1 is typically 1-3x the turn radius at current speed
%
%  L1 PARAMETER TUNING (critical for real flight):
%    L1_ratio = L1 / V  (seconds of lookahead)
%    Typical:  L1_ratio = 15-25 for cruise, 8-12 for patrol
%    For a fighter at 180 m/s: L1 = 180 * 20 = 3600 m
%
%  This is a NONLINEAR guidance law — it works at any bank angle and
%  does not linearise around small errors. This makes it suitable for
%  large course corrections (e.g., turning 90 degrees at a waypoint).
%
%  REFERENCE:
%    Park, Deyst & How, "A New Nonlinear Guidance Logic for Trajectory
%    Tracking," AIAA 2004-4900.
%    Implemented in: PX4 (ecl_l1_pos_controller), ArduPilot (L1_controller)

% L1 lookahead distance (metres)
% Adaptive: scales with airspeed for consistent tracking bandwidth
L1 = max(guidance.L1_ratio * V, guidance.L1_min);

% Path direction vector (horizontal only)
path_vec = wp_to(1:2) - wp_from(1:2);
path_len = norm(path_vec);

if path_len < 1.0
    % Degenerate segment (from and to are the same point)
    % Fall back to direct-to-waypoint guidance
    path_dir = (wp_to_horiz - pos_horiz);
    plen = norm(path_dir);
    if plen > 1.0
        path_dir = path_dir / plen;
    else
        path_dir = [cos(psi); sin(psi)];  % Continue current heading
    end
else
    path_dir = path_vec / path_len;
end

% Crosstrack error: perpendicular distance from aircraft to desired path
% (positive = aircraft is to the RIGHT of the desired path)
pos_relative = pos_horiz - wp_from(1:2);
crosstrack_error = pos_relative(1)*path_dir(2) - pos_relative(2)*path_dir(1);

% Along-track position on the desired path
alongtrack = dot(pos_relative, path_dir);

% L1 reference point: a point on the desired path, L1 ahead of the
% closest point on the path to the aircraft
% First, find the closest point on the path to the aircraft
closest_point = wp_from(1:2) + max(0, min(path_len, alongtrack)) * path_dir;

% Then advance by L1 along the path to get the L1 reference point
l1_point = closest_point + L1 * path_dir;

% Vector from aircraft to L1 reference point
l1_vec = l1_point - pos_horiz;
l1_dist = norm(l1_vec);

if l1_dist < 1.0
    l1_dist = 1.0;  % Prevent division by zero
end

% Aircraft ground-track velocity vector
Vn = V * cos(psi) * cos(theta);    % North velocity component
Ve = V * sin(psi) * cos(theta);    % East velocity component
V_ground = [Vn; Ve];
V_ground_mag = norm(V_ground);

if V_ground_mag < 5.0
    V_ground_mag = 5.0;  % Minimum for valid guidance
end

% Angle between velocity vector and line to L1 point
% eta = angle between V_ground and l1_vec
sin_eta = (V_ground(1)*l1_vec(2) - V_ground(2)*l1_vec(1)) / ...
          (V_ground_mag * l1_dist);
sin_eta = max(-1, min(1, sin_eta));  % Clamp for asin safety

% L1 lateral acceleration command
a_lat_cmd = 2 * V^2 * sin_eta / L1;

% Convert lateral acceleration to bank angle command
% From steady-turn equation: a_lat = g * tan(phi)
% Therefore: phi = atan(a_lat / g)
%
% We use atan (not atan2) because phi must be in [-pi/2, pi/2]
% In practice, limit bank angle to safe maximum
phi_cmd = atan(a_lat_cmd / g);

% Bank angle limiting
%   At high alpha or low speed, limit bank to prevent departure
%   At cruise, allow up to 60 degrees for agile waypoint tracking
phi_cmd = max(-guidance.phi_max, min(guidance.phi_max, phi_cmd));

%% ====================================================================
%  SECTION 3: LONGITUDINAL GUIDANCE — ALTITUDE AND SPEED CONTROL
%  ====================================================================
%
%  THEORY (TECS — Total Energy Control System):
%  ---------------------------------------------
%  The aircraft has two longitudinal controls: pitch and throttle.
%  And two longitudinal objectives: altitude and airspeed.
%
%  Key insight from energy conservation:
%    Total energy = Kinetic energy + Potential energy
%    E_total = 0.5*m*V^2 + m*g*h
%
%    Throttle controls TOTAL energy rate (add or remove energy)
%    Pitch angle controls ENERGY DISTRIBUTION (trade speed for altitude)
%
%  Therefore:
%    Energy rate error = (V_dot_cmd*V + g*hdot_cmd) - (V_dot*V + g*hdot)
%    Energy distribution error = (V_dot_cmd - g*hdot_cmd/V) - (V_dot - g*hdot/V)
%
%    Throttle corrects total energy error
%    Pitch corrects energy distribution error
%
%  In this simplified implementation:
%    - Altitude error → flight path angle command → pitch angle command
%    - Speed error → throttle command
%    - With cross-coupling compensation
%
%  For a REAL flight system, a full TECS implementation (as in PX4)
%  would be used, with proper energy-rate integrators and wind
%  compensation. The simplified version here captures the essential
%  physics while being easier to tune.

% Current altitude (metres above reference)
alt_current = -pos_d;

% ---- Altitude controller (outer loop → flight path angle) ----
%
%  altitude error → proportional → climb rate command
%  climb rate command → PI → flight path angle command
%  flight path angle → pitch angle (theta_cmd = gamma + alpha_trim)
%
%  Flight path angle: gamma = asin(hdot / V)
%    For small angles: gamma ≈ hdot / V

alt_error = alt_cmd - alt_current;

% Proportional gain on altitude error → climb rate command
%   Typical: 0.5-1.0 (m/s per m of error)
%   At K_alt = 0.5: 100m error → 50 m/s climb rate command
hdot_cmd = guidance.K_alt * alt_error;

% Limit climb/descent rate to structural and performance limits
%   Fighter UCAV: ±50 m/s climb rate is achievable
%   For energy management, limit to ±30 m/s normally
hdot_cmd = max(-guidance.hdot_max, min(guidance.hdot_max, hdot_cmd));

% Current climb rate (NED frame: hdot = -zdot = -Vd)
hdot_current = -(u*sin(theta) - v*sin(phi)*cos(theta) - w*cos(phi)*cos(theta));
% Simplified: hdot ≈ V * sin(gamma) where gamma = theta - alpha
hdot_error = hdot_cmd - hdot_current;

% PI controller on climb rate → flight path angle command
guid_state_out = guid_state_in;
guid_state_out.int_hdot = guid_state_in.int_hdot + hdot_error * dt;
guid_state_out.int_hdot = max(-guidance.int_hdot_lim, ...
    min(guidance.int_hdot_lim, guid_state_out.int_hdot));

gamma_cmd = guidance.K_hdot * hdot_error + ...
            guidance.Ki_hdot * guid_state_out.int_hdot;

% Limit flight path angle
gamma_cmd = max(-guidance.gamma_max, min(guidance.gamma_max, gamma_cmd));

% Convert flight path angle to pitch angle
%   theta = gamma + alpha
%   In steady flight, alpha depends on CL which depends on load factor.
%   For 1g level flight: alpha_trim ≈ (W/(qbar*S) - CL0) / CLalpha
%
%   For a first approximation, use a constant alpha_trim.
%   A production system would compute this from the aerodynamic model.
%
%   In a banked turn, the load factor is n = 1/cos(phi), so alpha
%   increases. We account for this:
n_load = 1 / max(cos(phi_cmd), 0.5);       % Load factor in turn
alpha_trim = guidance.alpha_trim_1g * n_load; % Alpha scales with load factor

theta_cmd = gamma_cmd + alpha_trim;

% Limit pitch angle
theta_cmd = max(-guidance.theta_max, min(guidance.theta_max, theta_cmd));

% ---- Speed controller (throttle management) ----
%
%  Airspeed error → PI controller → throttle command
%
%  The throttle primarily controls total energy rate.
%  During climbs, more throttle is needed even at constant speed
%  (converting kinetic → potential energy costs power).
%
%  Power-compensated throttle = thrust for level flight
%                             + thrust for climb
%                             + thrust for acceleration

V_error = V_cmd - V;

guid_state_out.int_V = guid_state_in.int_V + V_error * dt;
guid_state_out.int_V = max(-guidance.int_V_lim, ...
    min(guidance.int_V_lim, guid_state_out.int_V));

% Base throttle from energy balance (feedforward)
%   In level flight at current speed, the required thrust ≈ drag
%   T_level ≈ m*g*CD/CL ≈ m*g / (L/D)
%   For a fighter at cruise: L/D ≈ 8-10
%   T_level / T_max ≈ m*g / (L/D * T_max) ≈ 9000*9.81 / (9*83000) ≈ 0.12
%   Add climb power: T_climb = m*g*sin(gamma) / T_max
LD_ratio = 9.0;        % Approximate cruise L/D
T_max    = 83000;      % Max thrust (N)
m        = 9000;       % Mass (kg)

thr_level = m * g / (LD_ratio * T_max);               % Level flight throttle
thr_climb = m * g * sin(gamma_cmd) / T_max;            % Additional for climb
thr_feedforward = thr_level + thr_climb;

% PI feedback on airspeed error
thr_feedback = guidance.K_V * V_error + guidance.Ki_V * guid_state_out.int_V;

throttle_cmd = thr_feedforward + thr_feedback;
throttle_cmd = max(0.05, min(1.5, throttle_cmd));      % Clamp to valid range

% Anti-windup: freeze integrators if throttle is saturated
if throttle_cmd >= 1.5 || throttle_cmd <= 0.05
    guid_state_out.int_V = guid_state_in.int_V;
end

%% ====================================================================
%  SECTION 4: UPDATE GUIDANCE STATE
%  ====================================================================

guid_state_out.wp_idx    = wp_idx;
guid_state_out.lap_count = lap_count;

%% ====================================================================
%  SECTION 5: DEBUG OUTPUT FOR TELEMETRY
%  ====================================================================

guid_debug.wp_idx          = wp_idx;
guid_debug.lap_count       = lap_count;
guid_debug.dist_to_wp      = dist_to_wp;
guid_debug.crosstrack_error = crosstrack_error;
guid_debug.L1              = L1;
guid_debug.eta             = asin(sin_eta);
guid_debug.a_lat_cmd       = a_lat_cmd;
guid_debug.phi_cmd         = phi_cmd;
guid_debug.alt_cmd         = alt_cmd;
guid_debug.alt_error       = alt_error;
guid_debug.hdot_cmd        = hdot_cmd;
guid_debug.hdot_current    = hdot_current;
guid_debug.gamma_cmd       = gamma_cmd;
guid_debug.theta_cmd       = theta_cmd;
guid_debug.V_cmd           = V_cmd;
guid_debug.V_error         = V_error;
guid_debug.throttle_cmd    = throttle_cmd;
guid_debug.wp_switched     = wp_switched;

end
