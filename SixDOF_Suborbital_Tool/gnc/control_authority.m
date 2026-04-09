function auth = control_authority(controls, vehicle, sim_cfg)
%CONTROL_AUTHORITY  Real-time control authority monitor.
%
%   Computes remaining authority margin for each control effector as a
%   percentage of maximum available deflection/effort.
%
%   Authority margin = (max_deflection - |current_command|) / max_deflection * 100%
%
%   Issues warnings (to console + auth.warnings struct) when any effector
%   drops below sim_cfg.authority_warn_pct (default: 15%).
%
%   Also monitors the B-matrix condition number from the control allocation,
%   warning when it exceeds sim_cfg.cond_B_warn (default: 100).
%
%   Inputs:
%     controls   — Controls struct from control_system.m
%     vehicle    — Vehicle config
%     sim_cfg    — Sim config
%
%   Outputs:
%     auth       — Struct with authority margins and warning flags

warn_pct = sim_cfg.authority_warn_pct;

%% ========================================================================
%  SURFACE AUTHORITY (normalised commands, limits in physical degrees)
%  ========================================================================

% Convert normalised [-1,1] commands to physical degrees
de_deg = controls.delta_e * vehicle.de_max;
da_deg = controls.delta_a * vehicle.da_max;
dr_deg = controls.delta_r * vehicle.dr_max;

auth.elevator_pct  = (vehicle.de_max - abs(de_deg)) / vehicle.de_max * 100;
auth.aileron_pct   = (vehicle.da_max - abs(da_deg)) / vehicle.da_max * 100;
auth.rudder_pct    = (vehicle.dr_max - abs(dr_deg)) / vehicle.dr_max * 100;

auth.elevator_deg  = de_deg;
auth.aileron_deg   = da_deg;
auth.rudder_deg    = dr_deg;

%% ========================================================================
%  TVC AUTHORITY
%  ========================================================================

if sim_cfg.tvc_on
    dp_deg = rad2deg(controls.tvc_pitch);
    dy_deg = rad2deg(controls.tvc_yaw);

    auth.tvc_pitch_pct = (vehicle.tvc_max - abs(dp_deg)) / vehicle.tvc_max * 100;
    auth.tvc_yaw_pct   = (vehicle.tvc_max - abs(dy_deg)) / vehicle.tvc_max * 100;
    auth.tvc_pitch_deg = dp_deg;
    auth.tvc_yaw_deg   = dy_deg;
else
    auth.tvc_pitch_pct = NaN;
    auth.tvc_yaw_pct   = NaN;
    auth.tvc_pitch_deg = 0;
    auth.tvc_yaw_deg   = 0;
end

%% ========================================================================
%  RCS AUTHORITY (duty cycle as % of max)
%  ========================================================================

if sim_cfg.rcs_on && isfield(controls, 'rcs_cmd')
    % Estimate used RCS capacity
    rcs_max_moment = [2 * vehicle.rcs_thrust * vehicle.l_rcs_roll;
                      2 * vehicle.rcs_thrust * vehicle.l_rcs_pitch;
                      2 * vehicle.rcs_thrust * vehicle.l_rcs_yaw];

    rcs_demand = abs(controls.rcs_cmd);
    rcs_dc = min(rcs_demand ./ max(rcs_max_moment, 1), ones(3,1));

    auth.rcs_roll_pct  = (1 - rcs_dc(1)) * 100;
    auth.rcs_pitch_pct = (1 - rcs_dc(2)) * 100;
    auth.rcs_yaw_pct   = (1 - rcs_dc(3)) * 100;
    auth.rcs_duty      = rcs_dc;
else
    auth.rcs_roll_pct  = NaN;
    auth.rcs_pitch_pct = NaN;
    auth.rcs_yaw_pct   = NaN;
    auth.rcs_duty      = zeros(3,1);
end

%% ========================================================================
%  MOMENT DEMAND VS AVAILABLE (per axis)
%  ========================================================================

if isfield(controls, 'M_demand') && isfield(controls, 'M_achieved')
    M_err = controls.M_demand - controls.M_achieved;
    auth.M_error_Nm      = M_err;
    auth.M_error_pct_max = norm(M_err) / max(norm(controls.M_demand), 1) * 100;
else
    auth.M_error_Nm      = zeros(3,1);
    auth.M_error_pct_max = 0;
end

%% ========================================================================
%  B-MATRIX CONDITIONING
%  ========================================================================

if isfield(controls, 'cond_B')
    auth.cond_B    = controls.cond_B;
    auth.cond_warn = controls.cond_B > sim_cfg.cond_B_warn;
else
    auth.cond_B    = 1;
    auth.cond_warn = false;
end

%% ========================================================================
%  WARNING DETECTION
%  ========================================================================

warnings = {};
warn_flag = false;

check_fields = {'elevator_pct', 'aileron_pct', 'rudder_pct'};
check_names  = {'Elevator', 'Aileron', 'Rudder'};

if sim_cfg.tvc_on
    check_fields{end+1} = 'tvc_pitch_pct';  check_names{end+1} = 'TVC Pitch';
    check_fields{end+1} = 'tvc_yaw_pct';    check_names{end+1} = 'TVC Yaw';
end
if sim_cfg.rcs_on
    check_fields{end+1} = 'rcs_pitch_pct';  check_names{end+1} = 'RCS Pitch';
    check_fields{end+1} = 'rcs_roll_pct';   check_names{end+1} = 'RCS Roll';
end

for k = 1:length(check_fields)
    val = auth.(check_fields{k});
    if ~isnan(val) && val < warn_pct
        warnings{end+1} = sprintf('%s: %.1f%% remaining', check_names{k}, val);
        warn_flag = true;
    end
end

if auth.cond_warn
    warnings{end+1} = sprintf('B-matrix ill-conditioned: cond=%.0f', auth.cond_B);
    warn_flag = true;
end

auth.warnings  = warnings;
auth.warn_flag = warn_flag;

end
