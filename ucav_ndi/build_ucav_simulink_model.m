%% ========================================================================
%  BUILD UCAV NDI SIMULINK MODEL
%  ========================================================================
%
%  MODULE PURPOSE:
%  ---------------
%  Programmatically creates a Simulink model (fighter_ucav_sim.slx) that
%  implements the full autonomous UCAV trajectory-following system:
%
%   ┌──────────┐  ┌──────────┐  ┌───────────┐  ┌──────────┐  ┌─────────┐
%   │ Mission  │─>│ Guidance  │─>│ NDI/INDI  │─>│Actuators │─>│ 6-DOF   │
%   │ Waypoint │  │ (L1+TECS)│  │ Autopilot │  │          │  │ Plant   │
%   │ Source   │  │          │  │           │  │          │  │         │
%   └──────────┘  └──────────┘  └───────────┘  └──────────┘  └─────────┘
%                      ↑  ↑            ↑             │             │
%                      │  └── act pos ─┘             │             │
%                      └─── full state ──────────────┴─────────────┘
%
%  BLOCK DESCRIPTIONS:
%    1. Mission Waypoint Source — From Workspace blocks feeding current
%       waypoint commands to the guidance law
%    2. Guidance S-Function — L1 lateral + TECS longitudinal → phi/theta/thr
%    3. NDI Autopilot S-Function — NDI/INDI → surface commands
%    4. Actuator Subsystem — TF + Rate Limiter + Saturation per surface
%    5. 6-DOF Plant S-Function — Full nonlinear fighter dynamics
%    6. Scopes — Flight params, ground track, surfaces, attitude
%    7. To Workspace — All signals logged for post-sim analysis
%
%  PREREQUISITES:
%    Run setup_ucav.m before this script.
%
%  ========================================================================

fprintf('==========================================================\n');
fprintf(' Building UCAV NDI Simulink Model\n');
fprintf('==========================================================\n\n');

%% Check prerequisites
if ~exist('aircraft','var') || ~exist('fcs','var') || ~exist('mission','var')
    fprintf('Running setup_ucav first...\n\n');
    setup_ucav;
end

addpath('../fighter_ndi');

%% Model name
mdl = 'ucav_ndi_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

dt = sim_params.dt;

fprintf('Creating model: %s\n\n', mdl);

%% ====================================================================
%  LAYOUT
%  ====================================================================
col1 = 50;   col2 = 350;  col3 = 700; col4 = 1050; col5 = 1400;
row_main = 200; bw = 180; bh = 140;

%% ====================================================================
%  1. GUIDANCE S-FUNCTION
%  ====================================================================
fprintf('  [1/6] Adding guidance S-function...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Guidance_L1_TECS'], ...
    'Position', [col2, row_main-60, col2+bw, row_main+bh-40], ...
    'FunctionName', 'ucav_guidance_sfunc');

%% ====================================================================
%  2. NDI AUTOPILOT S-FUNCTION
%  ====================================================================
fprintf('  [2/6] Adding NDI autopilot S-function...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/NDI_Autopilot'], ...
    'Position', [col3, row_main-60, col3+bw, row_main+bh-40], ...
    'FunctionName', 'ucav_autopilot_sfunc');

%% ====================================================================
%  3. ACTUATOR SUBSYSTEM
%  ====================================================================
fprintf('  [3/6] Adding actuator subsystem...\n');

add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Actuators']);
set_param([mdl '/Actuators'], 'Position', [col4-50, row_main-40, col4+130, row_main+bh-60]);

% Build internal actuator channels
act_sub = [mdl '/Actuators'];
delete_line(act_sub, 'In1/1', 'Out1/1');
delete_block([act_sub '/In1']); delete_block([act_sub '/Out1']);

add_block('simulink/Sources/In1', [act_sub '/Cmd'], 'Position', [20,100,50,114], 'Port', '1');
add_block('simulink/Signal Routing/Demux', [act_sub '/Dmx'], 'Position', [80,40,85,220], 'Inputs', '4');
add_line(act_sub, 'Cmd/1', 'Dmx/1');

tau_a = 0.02;
names = {'deL','deR','dr','dc'};
rates = [80,80,60,60]; pos_lim = [25,25,30,25];
yo = [30,90,150,210];
for i = 1:4
    nm = names{i}; y=yo(i);
    add_block('simulink/Continuous/Transfer Fcn', [act_sub '/TF_' nm], 'Position', [130,y,210,y+25], ...
        'Numerator','[1]','Denominator',sprintf('[%g 1]',tau_a));
    add_block('simulink/Discontinuities/Rate Limiter', [act_sub '/RL_' nm], 'Position', [240,y,300,y+25], ...
        'RisingSlewLimit',sprintf('%g',deg2rad(rates(i))),'FallingSlewLimit',sprintf('%g',-deg2rad(rates(i))));
    add_block('simulink/Discontinuities/Saturation', [act_sub '/Sat_' nm], 'Position', [330,y,380,y+25], ...
        'UpperLimit',sprintf('%g',deg2rad(pos_lim(i))),'LowerLimit',sprintf('%g',-deg2rad(pos_lim(i))));
    add_line(act_sub, sprintf('Dmx/%d',i), ['TF_' nm '/1']);
    add_line(act_sub, ['TF_' nm '/1'], ['RL_' nm '/1']);
    add_line(act_sub, ['RL_' nm '/1'], ['Sat_' nm '/1']);
end

add_block('simulink/Signal Routing/Mux', [act_sub '/Mux_Out'], 'Position', [420,50,425,240], 'Inputs', '4');
for i = 1:4
    add_line(act_sub, ['Sat_' names{i} '/1'], sprintf('Mux_Out/%d',i));
end

add_block('simulink/Sinks/Out1', [act_sub '/Out_Surf'], 'Position', [470,130,500,144], 'Port', '1');
add_block('simulink/Sinks/Out1', [act_sub '/Out_FB'],   'Position', [470,170,500,184], 'Port', '2');
add_line(act_sub, 'Mux_Out/1', 'Out_Surf/1');
add_line(act_sub, 'Mux_Out/1', 'Out_FB/1');

%% ====================================================================
%  4. 6-DOF PLANT S-FUNCTION
%  ====================================================================
fprintf('  [4/6] Adding 6-DOF plant...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Plant_6DOF'], ...
    'Position', [col5-50, row_main-80, col5+130, row_main+bh-20], ...
    'FunctionName', 'fighter_plant_sfunc');

% Demux actuator output to 4 scalar plant inputs
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Surf'], ...
    'Position', [col4+160, row_main-40, col4+165, row_main+bh-70], 'Outputs', '4');
add_line(mdl, 'Actuators/1', 'Demux_Surf/1');
for i = 1:4
    add_line(mdl, sprintf('Demux_Surf/%d',i), sprintf('Plant_6DOF/%d',i));
end

% Throttle: guidance → NDI → separate line to plant port 5
% (Handled via autopilot output port 2)
add_line(mdl, 'NDI_Autopilot/2', 'Plant_6DOF/5', 'autorouting', 'smart');

%% ====================================================================
%  5. WIRING
%  ====================================================================
fprintf('  [5/6] Wiring connections...\n');

% Guidance outputs → NDI Autopilot
add_line(mdl, 'Guidance_L1_TECS/1', 'NDI_Autopilot/1', 'autorouting', 'smart');  % [phi_cmd;theta_cmd]

% NDI Autopilot surface commands → Actuators
add_line(mdl, 'NDI_Autopilot/1', 'Actuators/1', 'autorouting', 'smart');  % [de_L;de_R;dr;dc]

% Plant feedback → Guidance (position, velocity, euler, airdata)
add_line(mdl, 'Plant_6DOF/1', 'Guidance_L1_TECS/1', 'autorouting', 'smart'); % position
add_line(mdl, 'Plant_6DOF/2', 'Guidance_L1_TECS/2', 'autorouting', 'smart'); % velocity
add_line(mdl, 'Plant_6DOF/4', 'Guidance_L1_TECS/3', 'autorouting', 'smart'); % euler
add_line(mdl, 'Plant_6DOF/7', 'Guidance_L1_TECS/4', 'autorouting', 'smart'); % airdata

% Plant feedback → NDI Autopilot
add_line(mdl, 'Plant_6DOF/2', 'NDI_Autopilot/2', 'autorouting', 'smart'); % velocity
add_line(mdl, 'Plant_6DOF/3', 'NDI_Autopilot/3', 'autorouting', 'smart'); % omega
add_line(mdl, 'Plant_6DOF/4', 'NDI_Autopilot/4', 'autorouting', 'smart'); % euler
add_line(mdl, 'Plant_6DOF/5', 'NDI_Autopilot/5', 'autorouting', 'smart'); % accel
add_line(mdl, 'Plant_6DOF/6', 'NDI_Autopilot/6', 'autorouting', 'smart'); % omega_dot

% Actuator position feedback → NDI Autopilot
add_line(mdl, 'Actuators/2', 'NDI_Autopilot/7', 'autorouting', 'smart');

%% ====================================================================
%  6. SCOPES AND LOGGING
%  ====================================================================
fprintf('  [6/6] Adding scopes and logging...\n');

sc_x = col5 + 200;

% Scope: Guidance debug
add_block('simulink/Sinks/Scope', [mdl '/Guidance_Scope'], ...
    'Position', [sc_x, 50, sc_x+50, 100], 'NumInputPorts', '1', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'Guidance_L1_TECS/2', 'Guidance_Scope/1', 'autorouting', 'smart');

% Scope: Attitude
add_block('simulink/Sinks/Scope', [mdl '/Attitude_Scope'], ...
    'Position', [sc_x, 130, sc_x+50, 180], 'NumInputPorts', '2', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'Plant_6DOF/4', 'Attitude_Scope/1', 'autorouting', 'smart');
add_line(mdl, 'Plant_6DOF/3', 'Attitude_Scope/2', 'autorouting', 'smart');

% Scope: Surfaces
add_block('simulink/Sinks/Scope', [mdl '/Surface_Scope'], ...
    'Position', [sc_x, 210, sc_x+50, 260], 'NumInputPorts', '1');
add_line(mdl, 'Actuators/1', 'Surface_Scope/1', 'autorouting', 'smart');

% Scope: Position
add_block('simulink/Sinks/Scope', [mdl '/Position_Scope'], ...
    'Position', [sc_x, 290, sc_x+50, 340], 'NumInputPorts', '1');
add_line(mdl, 'Plant_6DOF/1', 'Position_Scope/1', 'autorouting', 'smart');

% To Workspace blocks
log_items = {'log_position', 'Plant_6DOF/1'; ...
             'log_euler',    'Plant_6DOF/4'; ...
             'log_airdata',  'Plant_6DOF/7'; ...
             'log_surfaces', 'Actuators/1'};
for i = 1:size(log_items,1)
    bname = [mdl '/TW_' log_items{i,1}];
    add_block('simulink/Sinks/To Workspace', bname, ...
        'Position', [sc_x+80, 50+(i-1)*80, sc_x+160, 70+(i-1)*80], ...
        'VariableName', log_items{i,1}, 'SaveFormat', 'Array');
    add_line(mdl, log_items{i,2}, ['TW_' log_items{i,1} '/1'], 'autorouting', 'smart');
end

%% ====================================================================
%  SOLVER CONFIGURATION
%  ====================================================================

set_param(mdl, 'Solver', 'ode4');
set_param(mdl, 'FixedStep', num2str(dt));
set_param(mdl, 'StopTime', num2str(sim_params.t_end));
set_param(mdl, 'SaveFormat', 'Array');
set_param(mdl, 'SaveOutput', 'on');
set_param(mdl, 'SaveTime', 'on');
set_param(mdl, 'LimitDataPoints', 'off');

%% ====================================================================
%  SAVE
%  ====================================================================

save_system(mdl);

fprintf('\n==========================================================\n');
fprintf(' MODEL BUILT: %s.slx\n', mdl);
fprintf('==========================================================\n\n');
fprintf('Architecture:\n');
fprintf('  [Mission WPs] → [L1/TECS Guidance] → [NDI/INDI Autopilot] → [Actuators] → [6-DOF Plant]\n');
fprintf('                        ↑                      ↑                    │              │\n');
fprintf('                        └──────────────────────┴────────────────────┴──────────────┘\n');
fprintf('\n');
fprintf('To run:\n');
fprintf('  1. setup_ucav         (load parameters)\n');
fprintf('  2. open_system(''%s'')  (open model)\n', mdl);
fprintf('  3. sim(''%s'')          (run simulation)\n\n', mdl);
