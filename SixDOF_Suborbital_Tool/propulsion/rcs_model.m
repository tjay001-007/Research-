function [F_rcs, M_rcs, duty_cycle, rcs_state_out] = rcs_model(rcs_cmd, attitude_err, rate_err, vehicle, sim_cfg)
%RCS_MODEL  Reaction Control System — 12-thruster on/off + PWM model.
%
%   12 thrusters arranged in 6 opposing pairs:
%     Roll+  / Roll-   (2 pairs, wingtip-mounted, moment arm l_rcs_roll)
%     Pitch+ / Pitch-  (2 pairs, nose/tail-mounted, moment arm l_rcs_pitch)
%     Yaw+   / Yaw-    (2 pairs, nose/tail-mounted, moment arm l_rcs_yaw)
%
%   Each thruster produces rcs_thrust (N) force.
%
%   Activation logic:
%     - Fire if attitude error > rcs_att_deadband  OR
%            rate error    > rcs_rate_deadband
%     - PWM modulation: duty_cycle = clamp(|error| / error_scale, 0, 1)
%     - Applied moment = duty_cycle * max_moment_per_axis
%
%   Inputs:
%     rcs_cmd       — Commanded moment [L_rcs; M_rcs; N_rcs] (Nm)
%                     (Output from control_system allocator)
%     attitude_err  — [phi_err; theta_err; psi_err] (rad) for deadband check
%     rate_err      — [p_err; qr_err; r_err] (rad/s) for deadband check
%     vehicle       — Vehicle config struct
%     sim_cfg       — Sim config struct (rcs_on flag, deadband thresholds)
%
%   Outputs:
%     F_rcs         — RCS force in body frame [3x1] (N) (typically ~0 for axial RCS)
%     M_rcs         — RCS moment in body frame [L; M; N] (Nm)
%     duty_cycle    — [roll_dc; pitch_dc; yaw_dc] as fraction [0,1]
%     rcs_state_out — Diagnostics struct

F_rcs = zeros(3,1);   % RCS thrusters produce negligible net linear force (opposing pairs)
M_rcs = zeros(3,1);
duty_cycle = zeros(3,1);
rcs_state_out.fired = false(12,1);

if ~sim_cfg.rcs_on
    rcs_state_out.active = false;
    return;
end

%% ========================================================================
%  MAXIMUM MOMENT PER AXIS
%  ========================================================================

F_t = vehicle.rcs_thrust;   % Single thruster force (N)

% Two thrusters per roll pair, moment arm l_rcs_roll
M_roll_max  = 2 * F_t * vehicle.l_rcs_roll;    % Nm
% Two thrusters per pitch pair, moment arm l_rcs_pitch
M_pitch_max = 2 * F_t * vehicle.l_rcs_pitch;   % Nm
% Two thrusters per yaw pair, moment arm l_rcs_yaw
M_yaw_max   = 2 * F_t * vehicle.l_rcs_yaw;     % Nm

M_max = [M_roll_max; M_pitch_max; M_yaw_max];

%% ========================================================================
%  DEADBAND CHECK — only fire if error exceeds threshold
%  ========================================================================

att_db   = vehicle.rcs_att_deadband;    % rad
rate_db  = vehicle.rcs_rate_deadband;   % rad/s

% Per-axis enable flags
att_err_norm  = abs(attitude_err);
rate_err_norm = abs(rate_err);

axis_enable = (att_err_norm > att_db) | (rate_err_norm > rate_db);

%% ========================================================================
%  PWM DUTY CYCLE — proportional to commanded moment
%  ========================================================================

% Scale factor: normalise commanded moment to [0,1] range
for k = 1:3
    if axis_enable(k) && M_max(k) > 0
        dc = abs(rcs_cmd(k)) / M_max(k);
        duty_cycle(k) = min(dc, 1.0);
    else
        duty_cycle(k) = 0.0;
    end
end

%% ========================================================================
%  MOMENT OUTPUT
%  Apply moment in the direction of command, scaled by duty cycle
%  ========================================================================

for k = 1:3
    if axis_enable(k)
        M_rcs(k) = sign(rcs_cmd(k)) * duty_cycle(k) * M_max(k);
    end
end

%% ========================================================================
%  THRUSTER FIRING FLAGS (for diagnostics / telemetry)
%  Thrusters 1-12: [Roll+a, Roll+b, Roll-a, Roll-b, Pitch+a, Pitch+b,
%                   Pitch-a, Pitch-b, Yaw+a, Yaw+b, Yaw-a, Yaw-b]
%  ========================================================================

dc_roll  = duty_cycle(1);
dc_pitch = duty_cycle(2);
dc_yaw   = duty_cycle(3);

cmd_roll  = rcs_cmd(1);
cmd_pitch = rcs_cmd(2);
cmd_yaw   = rcs_cmd(3);

rcs_state_out.fired(1:2)   = (cmd_roll  > 0) & (dc_roll  > 0);
rcs_state_out.fired(3:4)   = (cmd_roll  < 0) & (dc_roll  > 0);
rcs_state_out.fired(5:6)   = (cmd_pitch > 0) & (dc_pitch > 0);
rcs_state_out.fired(7:8)   = (cmd_pitch < 0) & (dc_pitch > 0);
rcs_state_out.fired(9:10)  = (cmd_yaw   > 0) & (dc_yaw   > 0);
rcs_state_out.fired(11:12) = (cmd_yaw   < 0) & (dc_yaw   > 0);

rcs_state_out.active        = any(rcs_state_out.fired);
rcs_state_out.duty_cycle    = duty_cycle;
rcs_state_out.M_max         = M_max;

end
