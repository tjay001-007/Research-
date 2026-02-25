%% ========================================================================
%  BUILD_UCAV_SIMULINK_MODEL — Programmatic Simulink model for UCAV INDI
%  ========================================================================
%
%  Creates a complete Simulink model (ucav_indi_sim.slx) implementing:
%
%   ┌──────────────┐   ┌───────────┐   ┌──────────┐   ┌──────────────┐
%   │  L1+PI       │──>│   INDI    │──>│Actuators │──>│  6-DOF Plant │
%   │  Guidance    │   │ Controller│   │(TF+RL+Sat)│  │ (ucav_plant_ │
%   │              │   │           │   │          │   │  sfunc)      │
%   └──────────────┘   └───────────┘   └──────────┘   └──────────────┘
%         ↑  ↑              ↑               │                │
%         │  └── act pos ───┘               │                │
%         └─── full state ──────────────────┴────────────────┘
%
%  Block details:
%    1. Guidance S-Function (ucav_guidance_sfunc)
%       - 4 input ports: position, velocity, euler, airdata
%       - 3 output ports: att_cmd, throttle, debug
%    2. INDI Controller S-Function (indi_controller_sfunc)
%       - 6 input ports: att_cmd, velocity, omega, euler, omega_dot, act_pos
%       - 2 output ports: surf_cmd, debug
%    3. Actuator Subsystem
%       - 3 channels: aileron, elevator, rudder (NO canard)
%       - Each channel: TF(1/(tau*s+1)) + Rate Limiter + Saturation
%    4. Plant S-Function (ucav_plant_sfunc)
%       - 4 input ports: da, de, dr, throttle
%       - 7 output ports: pos, vel, omega, euler, accel, omega_dot, airdata
%
%  Prerequisites: Run setup_ucav.m before this script.
%
%  Usage:
%    >> setup_ucav
%    >> build_ucav_simulink_model
%    >> sim('ucav_indi_sim')
%
%  ========================================================================

fprintf('==========================================================\n');
fprintf(' Building UCAV INDI Simulink Model\n');
fprintf('==========================================================\n\n');

%% Check prerequisites
if ~exist('aircraft','var') || ~exist('fcs','var') || ~exist('mission','var')
    fprintf('  Running setup_ucav first...\n\n');
    setup_ucav;
end

%% Model name
mdl = 'ucav_indi_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

dt = sim_params.dt;
fprintf('  Creating model: %s\n\n', mdl);

%% ====================================================================
%  LAYOUT CONSTANTS
%  ====================================================================
col1 = 50;    % Guidance
col2 = 400;   % INDI Controller
col3 = 750;   % Actuators
col4 = 1100;  % Plant
col5 = 1500;  % Scopes/Logging
row_main = 200;
bw = 200;     % Block width
bh = 140;     % Block height

%% ====================================================================
%  1. GUIDANCE S-FUNCTION
%  ====================================================================
fprintf('  [1/7] Adding guidance S-function...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Guidance'], ...
    'Position', [col1, row_main-50, col1+bw, row_main+bh-50], ...
    'FunctionName', 'ucav_guidance_sfunc');

%% ====================================================================
%  2. INDI CONTROLLER S-FUNCTION
%  ====================================================================
fprintf('  [2/7] Adding INDI controller S-function...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/INDI_Controller'], ...
    'Position', [col2, row_main-50, col2+bw, row_main+bh-50], ...
    'FunctionName', 'indi_controller_sfunc');

%% ====================================================================
%  3. ACTUATOR SUBSYSTEM (3 channels: aileron, elevator, rudder)
%  ====================================================================
fprintf('  [3/7] Adding actuator subsystem...\n');

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Actuators']);
set_param([mdl '/Actuators'], 'Position', ...
    [col3, row_main-30, col3+bw-20, row_main+bh-60]);

% Clean default content
act_sub = [mdl '/Actuators'];
delete_line(act_sub, 'In1/1', 'Out1/1');
delete_block([act_sub '/In1']);
delete_block([act_sub '/Out1']);

% Input: commanded surfaces [3] = [da_cmd; de_cmd; dr_cmd]
add_block('simulink/Sources/In1', [act_sub '/Cmd_In'], ...
    'Position', [20, 80, 50, 94], 'Port', '1');
add_block('simulink/Signal Routing/Demux', [act_sub '/Dmx'], ...
    'Position', [80, 30, 85, 170], 'Inputs', '3');
add_line(act_sub, 'Cmd_In/1', 'Dmx/1');

% Actuator channel parameters
names     = {'Aileron', 'Elevator', 'Rudder'};
tau_a     = aircraft.act.tau;
rates_max = [aircraft.act.aileron.rate_max, ...
             aircraft.act.elevator.rate_max, ...
             aircraft.act.rudder.rate_max];
pos_max   = [aircraft.act.aileron.pos_max, ...
             aircraft.act.elevator.pos_max, ...
             aircraft.act.rudder.pos_max];
yo = [20, 80, 140];

for i = 1:3
    nm = names{i}; y = yo(i);

    % Transfer function: 1/(tau*s + 1)
    add_block('simulink/Continuous/Transfer Fcn', [act_sub '/TF_' nm], ...
        'Position', [130, y, 210, y+25], ...
        'Numerator', '[1]', ...
        'Denominator', sprintf('[%g 1]', tau_a));

    % Rate limiter
    add_block('simulink/Discontinuities/Rate Limiter', [act_sub '/RL_' nm], ...
        'Position', [240, y, 310, y+25], ...
        'RisingSlewLimit', sprintf('%g', rates_max(i)), ...
        'FallingSlewLimit', sprintf('%g', -rates_max(i)));

    % Saturation
    add_block('simulink/Discontinuities/Saturation', [act_sub '/Sat_' nm], ...
        'Position', [340, y, 400, y+25], ...
        'UpperLimit', sprintf('%g', pos_max(i)), ...
        'LowerLimit', sprintf('%g', -pos_max(i)));

    % Wire: Demux -> TF -> RL -> Sat
    add_line(act_sub, sprintf('Dmx/%d', i), ['TF_' nm '/1']);
    add_line(act_sub, ['TF_' nm '/1'], ['RL_' nm '/1']);
    add_line(act_sub, ['RL_' nm '/1'], ['Sat_' nm '/1']);
end

% Mux actuator outputs back to [3]
add_block('simulink/Signal Routing/Mux', [act_sub '/Mux_Out'], ...
    'Position', [440, 30, 445, 170], 'Inputs', '3');
for i = 1:3
    add_line(act_sub, ['Sat_' names{i} '/1'], sprintf('Mux_Out/%d', i));
end

% Output 1: actual surfaces -> Plant
add_block('simulink/Sinks/Out1', [act_sub '/Surf_Out'], ...
    'Position', [490, 80, 520, 94], 'Port', '1');
add_line(act_sub, 'Mux_Out/1', 'Surf_Out/1');

% Output 2: actual positions -> INDI feedback
add_block('simulink/Sinks/Out1', [act_sub '/Pos_FB'], ...
    'Position', [490, 120, 520, 134], 'Port', '2');
add_line(act_sub, 'Mux_Out/1', 'Pos_FB/1');

%% ====================================================================
%  4. 6-DOF PLANT S-FUNCTION
%  ====================================================================
fprintf('  [4/7] Adding 6-DOF plant...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Plant_6DOF'], ...
    'Position', [col4, row_main-70, col4+bw, row_main+bh], ...
    'FunctionName', 'ucav_plant_sfunc');

%% ====================================================================
%  5. ACTUATOR-TO-PLANT WIRING
%  ====================================================================
fprintf('  [5/7] Wiring actuators to plant...\n');

% Demux actuator [3] output into 3 scalar plant inputs
add_block('simulink/Signal Routing/Demux', [mdl '/Dmx_Surf'], ...
    'Position', [col3+bw+10, row_main-20, col3+bw+15, row_main+bh-70], ...
    'Outputs', '3');
add_line(mdl, 'Actuators/1', 'Dmx_Surf/1');

% Connect 3 surfaces to plant inputs 1-3 (da, de, dr)
for i = 1:3
    add_line(mdl, sprintf('Dmx_Surf/%d', i), sprintf('Plant_6DOF/%d', i));
end

% Throttle: guidance output -> plant input 4
add_line(mdl, 'Guidance/2', 'Plant_6DOF/4', 'autorouting', 'smart');

%% ====================================================================
%  6. SIGNAL WIRING
%  ====================================================================
fprintf('  [6/7] Wiring signal connections...\n');

% --- Guidance -> INDI Controller ---
% Guidance port 1 [phi_cmd; theta_cmd] -> INDI port 1
add_line(mdl, 'Guidance/1', 'INDI_Controller/1', 'autorouting', 'smart');

% --- INDI Controller -> Actuators ---
% INDI port 1 [da_cmd; de_cmd; dr_cmd] -> Actuators port 1
add_line(mdl, 'INDI_Controller/1', 'Actuators/1', 'autorouting', 'smart');

% --- Plant -> Guidance feedback ---
% Plant port 1 (position) -> Guidance port 1
add_line(mdl, 'Plant_6DOF/1', 'Guidance/1', 'autorouting', 'smart');
% Plant port 2 (velocity) -> Guidance port 2
add_line(mdl, 'Plant_6DOF/2', 'Guidance/2', 'autorouting', 'smart');
% Plant port 4 (euler) -> Guidance port 3
add_line(mdl, 'Plant_6DOF/4', 'Guidance/3', 'autorouting', 'smart');
% Plant port 7 (airdata) -> Guidance port 4
add_line(mdl, 'Plant_6DOF/7', 'Guidance/4', 'autorouting', 'smart');

% --- Plant -> INDI Controller feedback ---
% Plant port 2 (velocity) -> INDI port 2
add_line(mdl, 'Plant_6DOF/2', 'INDI_Controller/2', 'autorouting', 'smart');
% Plant port 3 (omega) -> INDI port 3
add_line(mdl, 'Plant_6DOF/3', 'INDI_Controller/3', 'autorouting', 'smart');
% Plant port 4 (euler) -> INDI port 4
add_line(mdl, 'Plant_6DOF/4', 'INDI_Controller/4', 'autorouting', 'smart');
% Plant port 6 (omega_dot) -> INDI port 5
add_line(mdl, 'Plant_6DOF/6', 'INDI_Controller/5', 'autorouting', 'smart');

% --- Actuator position feedback -> INDI port 6 ---
add_line(mdl, 'Actuators/2', 'INDI_Controller/6', 'autorouting', 'smart');

%% ====================================================================
%  7. SCOPES AND LOGGING
%  ====================================================================
fprintf('  [7/7] Adding scopes and logging...\n');

sc_x = col5;

% Scope: Guidance Debug
add_block('simulink/Sinks/Scope', [mdl '/Guidance_Debug'], ...
    'Position', [sc_x, 30, sc_x+50, 80], ...
    'NumInputPorts', '1', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'Guidance/3', 'Guidance_Debug/1', 'autorouting', 'smart');

% Scope: Attitude (euler + omega)
add_block('simulink/Sinks/Scope', [mdl '/Attitude_Scope'], ...
    'Position', [sc_x, 110, sc_x+50, 160], ...
    'NumInputPorts', '2', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'Plant_6DOF/4', 'Attitude_Scope/1', 'autorouting', 'smart');
add_line(mdl, 'Plant_6DOF/3', 'Attitude_Scope/2', 'autorouting', 'smart');

% Scope: Surface deflections
add_block('simulink/Sinks/Scope', [mdl '/Surface_Scope'], ...
    'Position', [sc_x, 190, sc_x+50, 240], ...
    'NumInputPorts', '2');
add_line(mdl, 'INDI_Controller/1', 'Surface_Scope/1', 'autorouting', 'smart');
add_line(mdl, 'Actuators/1', 'Surface_Scope/2', 'autorouting', 'smart');

% Scope: Position
add_block('simulink/Sinks/Scope', [mdl '/Position_Scope'], ...
    'Position', [sc_x, 270, sc_x+50, 320], ...
    'NumInputPorts', '1');
add_line(mdl, 'Plant_6DOF/1', 'Position_Scope/1', 'autorouting', 'smart');

% Scope: Controller Debug
add_block('simulink/Sinks/Scope', [mdl '/INDI_Debug'], ...
    'Position', [sc_x, 350, sc_x+50, 400], ...
    'NumInputPorts', '1');
add_line(mdl, 'INDI_Controller/2', 'INDI_Debug/1', 'autorouting', 'smart');

% --- To Workspace blocks for post-simulation analysis ---
log_items = {
    'log_position',   'Plant_6DOF/1';
    'log_velocity',   'Plant_6DOF/2';
    'log_euler',      'Plant_6DOF/4';
    'log_airdata',    'Plant_6DOF/7';
    'log_surfaces',   'Actuators/1';
    'log_guid_debug', 'Guidance/3';
    'log_indi_debug', 'INDI_Controller/2';
};

for i = 1:size(log_items, 1)
    bname = [mdl '/TW_' log_items{i,1}];
    add_block('simulink/Sinks/To Workspace', bname, ...
        'Position', [sc_x+80, 30+(i-1)*55, sc_x+170, 50+(i-1)*55], ...
        'VariableName', log_items{i,1}, ...
        'SaveFormat', 'Array');
    add_line(mdl, log_items{i,2}, ['TW_' log_items{i,1} '/1'], ...
        'autorouting', 'smart');
end

%% ====================================================================
%  SOLVER CONFIGURATION
%  ====================================================================

set_param(mdl, 'Solver', 'ode4');             % Fixed-step RK4
set_param(mdl, 'FixedStep', num2str(dt));     % 0.004 s = 250 Hz
set_param(mdl, 'StopTime', num2str(sim_params.t_end));
set_param(mdl, 'SaveFormat', 'Array');
set_param(mdl, 'SaveOutput', 'on');
set_param(mdl, 'SaveTime', 'on');
set_param(mdl, 'LimitDataPoints', 'off');

%% ====================================================================
%  TITLE ANNOTATION
%  ====================================================================

add_block('simulink/Annotations/Note', [mdl '/Title'], ...
    'Position', [50, -50, 900, -10]);
set_param([mdl '/Title'], 'Text', ...
    ['UCAV INDI Autonomous Flight Control  |  ' ...
     'Conventional Tail (Aileron+Elevator+Rudder)  |  ' ...
     'Cm_alpha > 0 (UNSTABLE)  |  Run setup_ucav.m first']);

%% ====================================================================
%  SAVE MODEL
%  ====================================================================

save_system(mdl);

fprintf('\n==========================================================\n');
fprintf(' MODEL BUILT: %s.slx\n', mdl);
fprintf('==========================================================\n\n');
fprintf('Architecture:\n');
fprintf('  [L1+PI Guidance] -> [INDI Controller] -> [Actuators] -> [6-DOF Plant]\n');
fprintf('       ^                    ^                  |              |\n');
fprintf('       +--------------------+------------------+--------------+\n');
fprintf('                         (feedback)\n\n');
fprintf('Control surfaces: Aileron, Elevator, Rudder (3 channels, NO canard)\n\n');
fprintf('To run:\n');
fprintf('  1. setup_ucav                    %% Load parameters\n');
fprintf('  2. build_ucav_simulink_model     %% This script (already done)\n');
fprintf('  3. sim(''%s'')            %% Run simulation\n\n', mdl);
fprintf('Scopes (open at sim start):\n');
fprintf('  - Guidance_Debug:  wp_idx, dist_wp, xtrack, alt_cmd, alt, V_cmd, V, lap\n');
fprintf('  - Attitude_Scope:  euler angles + angular rates\n');
fprintf('  - Surface_Scope:   commanded vs actual deflections\n');
fprintf('  - Position_Scope:  NED position\n');
fprintf('  - INDI_Debug:      rate commands, alpha, beta, V, accel commands\n\n');
fprintf('To Workspace variables:\n');
fprintf('  log_position, log_velocity, log_euler, log_airdata,\n');
fprintf('  log_surfaces, log_guid_debug, log_indi_debug\n\n');
fprintf('Solver: ODE4 (fixed-step RK4), dt = %g s (%d Hz)\n', dt, round(1/dt));
fprintf('Stop time: %g s\n\n', sim_params.t_end);
