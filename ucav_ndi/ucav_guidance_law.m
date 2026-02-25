function [phi_cmd, theta_cmd, thr_cmd, guid_out, debug] = ucav_guidance_law( ...
    N, E, D, u, v, w, phi, theta, psi, V, ...
    guid_in, mission, guidance, trim, dt)
%UCAV_GUIDANCE_LAW  L1 lateral guidance + PI altitude/speed control.
%
%  Generates attitude and throttle commands to follow a waypoint trajectory.
%
%  Algorithms:
%    Lateral:    L1 nonlinear guidance (same as PX4/ArduPilot)
%                - Computes cross-track error to current leg
%                - Generates lateral acceleration command
%                - Converts to bank angle command
%    Altitude:   PI controller
%                - Altitude error -> climb rate command -> pitch angle command
%    Speed:      PI controller
%                - Airspeed error -> throttle command
%    Navigation: Waypoint sequencing with acceptance radius and racetrack loop
%
%  Inputs:
%    N, E, D           - Aircraft position in NED (m)
%    u, v, w           - Body velocities (m/s)
%    phi, theta, psi   - Euler angles (rad)
%    V                 - True airspeed (m/s)
%    guid_in           - Guidance state: .wp_idx, .lap_count, .int_alt, .int_V
%    mission           - Mission parameters (waypoints, racetrack config)
%    guidance          - Guidance gains
%    trim              - Trim conditions
%    dt                - Time step (s)
%
%  Outputs:
%    phi_cmd           - Bank angle command (rad)
%    theta_cmd         - Pitch angle command (rad)
%    thr_cmd           - Throttle command (0-1)
%    guid_out          - Updated guidance state
%    debug             - Debug signals

    g = 9.81;
    wp_ned = mission.waypoints_ned;
    num_wp = size(wp_ned, 1);

    % Read guidance state
    wp_idx    = guid_in.wp_idx;
    lap_count = guid_in.lap_count;
    int_alt   = guid_in.int_alt;
    int_V     = guid_in.int_V;

    % Current altitude (positive up)
    alt = -D;

    % ================================================================
    %  SECTION 1: WAYPOINT NAVIGATION
    % ================================================================
    %  Determine current and previous waypoints. Check if aircraft
    %  has reached the next waypoint (within acceptance radius).
    %  Handle racetrack looping.

    % Ensure valid wp_idx
    wp_idx = max(2, min(wp_idx, num_wp));

    % Previous and next waypoints
    wp_prev_idx = wp_idx - 1;
    if wp_prev_idx < 1
        wp_prev_idx = num_wp;   % Wrap around for racetrack
    end

    wp_prev = wp_ned(wp_prev_idx, :);   % [N, E, D]
    wp_next = wp_ned(wp_idx, :);

    % Distance to next waypoint (2D, horizontal plane)
    dist_to_wp = sqrt((N - wp_next(1))^2 + (E - wp_next(2))^2);

    % Check waypoint acceptance
    if dist_to_wp < guidance.accept_rad
        % Advance to next waypoint
        wp_idx = wp_idx + 1;

        % Handle racetrack loop
        if wp_idx > mission.racetrack_end
            if lap_count < mission.num_laps
                wp_idx = mission.racetrack_start + 1;  % Loop back
                lap_count = lap_count + 1;
            else
                wp_idx = mission.racetrack_end;  % Hold last WP
            end
        end

        % Update indices for this step
        wp_idx = max(2, min(wp_idx, num_wp));
        wp_prev_idx = wp_idx - 1;
        if wp_prev_idx < 1
            wp_prev_idx = num_wp;
        end
        wp_prev = wp_ned(wp_prev_idx, :);
        wp_next = wp_ned(wp_idx, :);
    end

    % Commanded altitude from next waypoint
    alt_cmd = -wp_next(3);       % Convert D to altitude

    % ================================================================
    %  SECTION 2: L1 LATERAL GUIDANCE
    % ================================================================
    %  L1 guidance tracks a straight-line path between waypoints.
    %  It computes a lateral acceleration proportional to the cross-track
    %  error, producing smooth path following without overshoots.
    %
    %  Algorithm:
    %    1. Compute path direction vector from wp_prev to wp_next
    %    2. Compute cross-track error (perpendicular distance to path)
    %    3. Compute L1 reference point on the path
    %    4. Command bank angle to generate required lateral acceleration

    % Path vector and unit vector
    path_vec = wp_next(1:2) - wp_prev(1:2);   % [dN, dE]
    path_len = norm(path_vec);
    if path_len < 1.0
        path_hat = [cos(psi); sin(psi)];      % Fallback: fly current heading
    else
        path_hat = path_vec(:) / path_len;
    end

    % Aircraft position relative to previous waypoint
    rel_pos = [N; E] - wp_prev(1:2)';

    % Cross-track error: positive = aircraft is to the RIGHT of the path
    xtrack = path_hat(1) * rel_pos(2) - path_hat(2) * rel_pos(1);

    % Along-track position
    along_track = dot(rel_pos, path_hat);

    % L1 distance (proportional to airspeed)
    L1 = max(V * guidance.L1_ratio, guidance.L1_min);

    % L1 angle: how much to correct toward the path
    %   sin(eta) = xtrack / L1, clamped for safety
    sin_eta = max(min(xtrack / L1, 0.7071), -0.7071);  % Limit to 45 deg correction

    % Lateral acceleration command
    a_lat = 2.0 * V^2 / L1 * sin_eta;

    % Convert to bank angle command: phi_cmd = atan(a_lat / g)
    phi_cmd = atan(a_lat / g);

    % Limit bank angle
    phi_cmd = max(min(phi_cmd, guidance.phi_max), -guidance.phi_max);

    % ================================================================
    %  SECTION 3: ALTITUDE CONTROL
    % ================================================================
    %  PI altitude hold that converts altitude error to a pitch angle command.
    %
    %  altitude_error -> (P gain) -> climb_rate_cmd -> (1/V) -> gamma_cmd -> theta_cmd
    %  Plus integral for steady-state altitude accuracy.

    alt_error = alt_cmd - alt;

    % Proportional: altitude error -> climb rate command
    hdot_cmd = guidance.Kh * alt_error;
    hdot_cmd = max(min(hdot_cmd, guidance.max_climb), guidance.max_descend);

    % Integral on altitude error (slow, for steady-state accuracy)
    int_alt = int_alt + guidance.Ki_h * alt_error * dt;
    int_alt = max(min(int_alt, deg2rad(5)), -deg2rad(5));   % Anti-windup

    % Flight path angle command
    gamma_cmd = asin(max(min(hdot_cmd / max(V, 20), 0.25), -0.25));

    % Pitch angle command = flight path angle + trim alpha + integral correction
    theta_cmd = gamma_cmd + trim.alpha + int_alt;

    % Limit pitch command
    theta_cmd = max(min(theta_cmd, guidance.theta_max), -guidance.theta_max);

    % ================================================================
    %  SECTION 4: SPEED CONTROL
    % ================================================================
    %  PI airspeed hold using throttle.
    %  Decoupled from altitude control for simplicity and robustness.

    V_cmd = mission.V_cmd;
    V_error = V_cmd - V;

    % PI throttle controller
    int_V = int_V + guidance.Ki_v * V_error * dt;
    int_V = max(min(int_V, 0.3), -0.3);   % Anti-windup

    thr_cmd = trim.throttle + guidance.Kv * V_error + int_V;
    thr_cmd = max(min(thr_cmd, guidance.thr_max), guidance.thr_min);

    % ================================================================
    %  SECTION 5: UPDATE GUIDANCE STATE
    % ================================================================

    guid_out.wp_idx    = wp_idx;
    guid_out.lap_count = lap_count;
    guid_out.int_alt   = int_alt;
    guid_out.int_V     = int_V;

    % ================================================================
    %  SECTION 6: DEBUG OUTPUTS
    % ================================================================

    debug.wp_idx     = wp_idx;
    debug.dist_to_wp = dist_to_wp;
    debug.xtrack     = xtrack;
    debug.along_track = along_track;
    debug.alt_cmd    = alt_cmd;
    debug.alt        = alt;
    debug.V_cmd      = V_cmd;
    debug.lap_count  = lap_count;
    debug.L1         = L1;
    debug.a_lat      = a_lat;
    debug.gamma_cmd  = gamma_cmd;

end
