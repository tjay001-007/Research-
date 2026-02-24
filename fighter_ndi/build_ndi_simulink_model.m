%% Build NDI Simulink Model
%  Programmatically creates the complete Simulink block diagram for the
%  fighter NDI/INDI closed-loop flight control system.
%
%  Model architecture:
%
%  ┌─────────────┐   ┌──────────────┐   ┌──────────────┐   ┌──────────────┐
%  │ Pilot Input │──>│  NDI / INDI  │──>│  Actuators   │──>│  6-DOF Plant │
%  │ (Commands)  │   │  Controller  │   │ (Rate+Pos)   │   │  (Fighter)   │
%  └─────────────┘   └──────────────┘   └──────────────┘   └──────────────┘
%                          ↑  ↑                  │                │
%                          │  └─ act positions ──┘                │
%                          └──── state feedback ──────────────────┘
%                               [vel, omega, euler, accel, omega_dot]
%
%  Scopes:
%    - Flight Parameters (Nz, alpha, Mach, altitude)
%    - Attitude (phi, theta, psi, p, q, r)
%    - Control Surfaces (de_L, de_R, dr, dc commanded + actual)
%    - Pilot Inputs (stick, pedal, throttle)
%
%  Run setup_fighter.m first, then this script, then open the model.
%
%  Usage:
%    >> setup_fighter
%    >> build_ndi_simulink_model
%    >> open_system('fighter_ndi_sim')

fprintf('==========================================================\n');
fprintf(' Building Fighter NDI Simulink Model\n');
fprintf('==========================================================\n\n');

%% Check prerequisites
if ~exist('aircraft', 'var') || ~exist('fcs', 'var')
    fprintf('Running setup_fighter first...\n\n');
    setup_fighter;
end

%% Model name
mdl = 'fighter_ndi_sim';

% Close if already open
if bdIsLoaded(mdl)
    close_system(mdl, 0);
end

% Create new model
new_system(mdl);
open_system(mdl);

fprintf('Creating model: %s\n\n', mdl);

%% ====================================================================
%  LAYOUT CONSTANTS
%  ====================================================================
%  Block positions: [left top right bottom]
%  Flow is left-to-right, with feedback going bottom

% Column x-positions (left edge)
col_input  = 50;
col_fcs    = 350;
col_act    = 700;
col_plant  = 1050;
col_scope  = 1400;
col_fb     = 700;    % Feedback routing

% Row y-positions (top edge)
row1 = 50;     % Pilot inputs
row2 = 250;    % Main signal path
row3 = 500;    % Feedback path
row4 = 700;    % Scopes bottom row

bw = 180;  % Block width
bh = 120;  % Block height
bhs = 30;  % Small block height

%% ====================================================================
%  1. PILOT INPUT BLOCKS
%  ====================================================================
fprintf('  Adding pilot input blocks...\n');

% --- Longitudinal stick (pitch) ---
add_block('simulink/Sources/Signal Builder', [mdl '/Stick_Lon'], ...
    'Position', [col_input, row1, col_input+120, row1+50]);

% Replace with a Step + Sum for a cleaner demo:
% Use a repeating sequence for the combat maneuver profile
delete_block([mdl '/Stick_Lon']);

% Stick longitudinal: From Workspace block
add_block('simulink/Sources/From Workspace', [mdl '/Stick_Lon'], ...
    'Position', [col_input, row1, col_input+100, row1+bhs], ...
    'VariableName', 'stick_lon_ts', ...
    'SampleTime', '0');

% Stick lateral
add_block('simulink/Sources/From Workspace', [mdl '/Stick_Lat'], ...
    'Position', [col_input, row1+50, col_input+100, row1+50+bhs], ...
    'VariableName', 'stick_lat_ts', ...
    'SampleTime', '0');

% Rudder pedal
add_block('simulink/Sources/From Workspace', [mdl '/Pedal'], ...
    'Position', [col_input, row1+100, col_input+100, row1+100+bhs], ...
    'VariableName', 'pedal_ts', ...
    'SampleTime', '0');

% Throttle
add_block('simulink/Sources/Constant', [mdl '/Throttle'], ...
    'Position', [col_input, row1+150, col_input+100, row1+150+bhs], ...
    'Value', '0.75');

% Mux pilot commands into [4x1]
add_block('simulink/Signal Routing/Mux', [mdl '/Pilot_Mux'], ...
    'Position', [col_input+140, row1+20, col_input+145, row1+170], ...
    'Inputs', '4');

% Connect pilot inputs to mux
add_line(mdl, 'Stick_Lon/1', 'Pilot_Mux/1');
add_line(mdl, 'Stick_Lat/1', 'Pilot_Mux/2');
add_line(mdl, 'Pedal/1',     'Pilot_Mux/3');
add_line(mdl, 'Throttle/1',  'Pilot_Mux/4');

%% ====================================================================
%  2. NDI FLIGHT CONTROL SYSTEM
%  ====================================================================
fprintf('  Adding NDI FCS block...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/NDI_FCS'], ...
    'Position', [col_fcs, row2-60, col_fcs+bw, row2+bh-20], ...
    'FunctionName', 'ndi_fcs_sfunc');

% Annotate the FCS block
add_block('simulink/Annotations/Note', [mdl '/FCS_Label'], ...
    'Position', [col_fcs+20, row2-80, col_fcs+160, row2-65]);
set_param([mdl '/FCS_Label'], 'Text', 'NDI/INDI Flight Control System');

% Connect pilot mux → FCS port 1 (pilot_cmd)
add_line(mdl, 'Pilot_Mux/1', 'NDI_FCS/1');

%% ====================================================================
%  3. ACTUATOR SUBSYSTEM
%  ====================================================================
fprintf('  Adding actuator subsystem...\n');

% Create actuator subsystem
add_block('simulink/Ports & Subsystems/Subsystem', [mdl '/Actuators']);
set_param([mdl '/Actuators'], 'Position', ...
    [col_act, row2-60, col_act+bw, row2+bh-20]);

% Delete default content
delete_line([mdl '/Actuators'], 'In1/1', 'Out1/1');
delete_block([mdl '/Actuators/In1']);
delete_block([mdl '/Actuators/Out1']);

% Build actuator internals:
%   Input: surf_cmd [4] → Demux → 4x (TF + RateLim + Sat) → Mux → Output: surf_act [4]
%   Plus second output: act_pos [4] (copy for FCS feedback)

act = [mdl '/Actuators'];

% Input port: commanded surfaces [4]
add_block('simulink/Sources/In1', [act '/Cmd_In'], ...
    'Position', [30, 100, 60, 114], 'Port', '1');

% Demux into 4 channels
add_block('simulink/Signal Routing/Demux', [act '/Demux_Cmd'], ...
    'Position', [100, 50, 105, 220], 'Outputs', '4');

add_line(act, 'Cmd_In/1', 'Demux_Cmd/1');

% Actuator names and limits
act_names = {'deL', 'deR', 'dr', 'dc'};
act_rate_max = [80, 80, 60, 60];  % deg/s
act_pos_max  = [25, 25, 30, 25];  % deg
tau_act = 0.02;  % 20 ms time constant

y_offsets = [30, 100, 170, 240];

for i = 1:4
    yo = y_offsets(i);
    nm = act_names{i};

    % Transfer function: 1/(tau*s + 1)
    tf_name = ['TF_' nm];
    add_block('simulink/Continuous/Transfer Fcn', [act '/' tf_name], ...
        'Position', [160, yo, 250, yo+30], ...
        'Numerator', '[1]', ...
        'Denominator', sprintf('[%g 1]', tau_act));

    % Rate limiter
    rl_name = ['RateLim_' nm];
    add_block('simulink/Discontinuities/Rate Limiter', [act '/' rl_name], ...
        'Position', [280, yo, 340, yo+30], ...
        'RisingSlewLimit', sprintf('%g', deg2rad(act_rate_max(i))), ...
        'FallingSlewLimit', sprintf('%g', -deg2rad(act_rate_max(i))));

    % Saturation (position limit)
    sat_name = ['Sat_' nm];
    add_block('simulink/Discontinuities/Saturation', [act '/' sat_name], ...
        'Position', [370, yo, 420, yo+30], ...
        'UpperLimit', sprintf('%g', deg2rad(act_pos_max(i))), ...
        'LowerLimit', sprintf('%g', -deg2rad(act_pos_max(i))));

    % Wire: Demux → TF → RateLim → Sat
    add_line(act, sprintf('Demux_Cmd/%d', i), [tf_name '/1']);
    add_line(act, [tf_name '/1'], [rl_name '/1']);
    add_line(act, [rl_name '/1'], [sat_name '/1']);
end

% Mux actuator outputs back to [4]
add_block('simulink/Signal Routing/Mux', [act '/Mux_Act'], ...
    'Position', [460, 50, 465, 260], 'Inputs', '4');

for i = 1:4
    add_line(act, ['Sat_' act_names{i} '/1'], sprintf('Mux_Act/%d', i));
end

% Output port 1: actual surface positions → Plant
add_block('simulink/Sinks/Out1', [act '/Surf_Out'], ...
    'Position', [510, 140, 540, 154], 'Port', '1');
add_line(act, 'Mux_Act/1', 'Surf_Out/1');

% Output port 2: actual positions → FCS feedback
add_block('simulink/Sinks/Out1', [act '/Pos_FB_Out'], ...
    'Position', [510, 180, 540, 194], 'Port', '2');
add_line(act, 'Mux_Act/1', 'Pos_FB_Out/1');

%% ====================================================================
%  4. 6-DOF FIGHTER PLANT
%  ====================================================================
fprintf('  Adding 6-DOF plant block...\n');

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [mdl '/Fighter_Plant'], ...
    'Position', [col_plant, row2-80, col_plant+bw, row2+bh], ...
    'FunctionName', 'fighter_plant_sfunc');

% --- Demux actuator outputs into 4 scalar inputs for plant ---
add_block('simulink/Signal Routing/Demux', [mdl '/Demux_Surf'], ...
    'Position', [col_act+bw+30, row2-50, col_act+bw+35, row2+bh-30], ...
    'Outputs', '4');

add_line(mdl, 'Actuators/1', 'Demux_Surf/1');

% Connect 4 surface channels to plant inputs 1-4
for i = 1:4
    add_line(mdl, sprintf('Demux_Surf/%d', i), sprintf('Fighter_Plant/%d', i));
end

% --- Throttle → Plant input 5 ---
% FCS outputs throttle on port 2 — route to plant
add_line(mdl, 'NDI_FCS/2', 'Fighter_Plant/5');

%% ====================================================================
%  5. FCS-TO-ACTUATOR CONNECTION
%  ====================================================================
fprintf('  Wiring FCS → Actuators...\n');

add_line(mdl, 'NDI_FCS/1', 'Actuators/1');

%% ====================================================================
%  6. FEEDBACK ROUTING — Plant → FCS
%  ====================================================================
fprintf('  Wiring feedback path...\n');

% Plant outputs:
%   Port 1: position [3]     → (scope only)
%   Port 2: velocity [3]     → FCS port 2
%   Port 3: omega    [3]     → FCS port 3
%   Port 4: euler    [3]     → FCS port 4
%   Port 5: accel    [3]     → FCS port 5
%   Port 6: omega_dot [3]    → FCS port 6
%   Port 7: airdata  [6]     → (scope only)
% Actuators output:
%   Port 2: act_pos  [4]     → FCS port 7

% Direct feedback lines
add_line(mdl, 'Fighter_Plant/2', 'NDI_FCS/2', 'autorouting', 'smart');  % velocity
add_line(mdl, 'Fighter_Plant/3', 'NDI_FCS/3', 'autorouting', 'smart');  % omega
add_line(mdl, 'Fighter_Plant/4', 'NDI_FCS/4', 'autorouting', 'smart');  % euler
add_line(mdl, 'Fighter_Plant/5', 'NDI_FCS/5', 'autorouting', 'smart');  % accel
add_line(mdl, 'Fighter_Plant/6', 'NDI_FCS/6', 'autorouting', 'smart');  % omega_dot
add_line(mdl, 'Actuators/2',     'NDI_FCS/7', 'autorouting', 'smart');  % act positions

%% ====================================================================
%  7. SCOPES AND DISPLAYS
%  ====================================================================
fprintf('  Adding scopes and displays...\n');

% --- Scope 1: Flight Parameters ---
add_block('simulink/Sinks/Scope', [mdl '/Flight_Params'], ...
    'Position', [col_scope, row1, col_scope+60, row1+60], ...
    'NumInputPorts', '1', ...
    'OpenAtSimulationStart', 'on');
% FCS debug output: [Nz_cmd, Nz_act, alpha, beta, Mach, V, p_cmd, q_cmd]
add_line(mdl, 'NDI_FCS/3', 'Flight_Params/1', 'autorouting', 'smart');

% --- Scope 2: Attitude ---
add_block('simulink/Sinks/Scope', [mdl '/Attitude_Scope'], ...
    'Position', [col_scope, row1+100, col_scope+60, row1+160], ...
    'NumInputPorts', '2', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'Fighter_Plant/4', 'Attitude_Scope/1', 'autorouting', 'smart'); % euler
add_line(mdl, 'Fighter_Plant/3', 'Attitude_Scope/2', 'autorouting', 'smart'); % omega

% --- Scope 3: Control Surfaces ---
add_block('simulink/Sinks/Scope', [mdl '/Surface_Scope'], ...
    'Position', [col_scope, row1+200, col_scope+60, row1+260], ...
    'NumInputPorts', '2', ...
    'OpenAtSimulationStart', 'on');
add_line(mdl, 'NDI_FCS/1',   'Surface_Scope/1', 'autorouting', 'smart'); % commanded
add_line(mdl, 'Actuators/1', 'Surface_Scope/2', 'autorouting', 'smart'); % actual

% --- Scope 4: Position / Trajectory ---
add_block('simulink/Sinks/Scope', [mdl '/Position_Scope'], ...
    'Position', [col_scope, row1+300, col_scope+60, row1+360], ...
    'NumInputPorts', '1');
add_line(mdl, 'Fighter_Plant/1', 'Position_Scope/1', 'autorouting', 'smart');

% --- Scope 5: Air Data ---
add_block('simulink/Sinks/Scope', [mdl '/AirData_Scope'], ...
    'Position', [col_scope, row1+400, col_scope+60, row1+460], ...
    'NumInputPorts', '1');
add_line(mdl, 'Fighter_Plant/7', 'AirData_Scope/1', 'autorouting', 'smart');

% --- To Workspace blocks for post-sim analysis ---
add_block('simulink/Sinks/To Workspace', [mdl '/Log_Debug'], ...
    'Position', [col_scope+100, row1, col_scope+180, row1+30], ...
    'VariableName', 'log_debug', ...
    'SaveFormat', 'Array');
add_line(mdl, 'NDI_FCS/3', 'Log_Debug/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_Euler'], ...
    'Position', [col_scope+100, row1+100, col_scope+180, row1+130], ...
    'VariableName', 'log_euler', ...
    'SaveFormat', 'Array');
add_line(mdl, 'Fighter_Plant/4', 'Log_Euler/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_Surfaces'], ...
    'Position', [col_scope+100, row1+200, col_scope+180, row1+230], ...
    'VariableName', 'log_surfaces', ...
    'SaveFormat', 'Array');
add_line(mdl, 'Actuators/1', 'Log_Surfaces/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_Position'], ...
    'Position', [col_scope+100, row1+300, col_scope+180, row1+330], ...
    'VariableName', 'log_position', ...
    'SaveFormat', 'Array');
add_line(mdl, 'Fighter_Plant/1', 'Log_Position/1', 'autorouting', 'smart');

add_block('simulink/Sinks/To Workspace', [mdl '/Log_AirData'], ...
    'Position', [col_scope+100, row1+400, col_scope+180, row1+430], ...
    'VariableName', 'log_airdata', ...
    'SaveFormat', 'Array');
add_line(mdl, 'Fighter_Plant/7', 'Log_AirData/1', 'autorouting', 'smart');

%% ====================================================================
%  8. MODEL-LEVEL ANNOTATIONS
%  ====================================================================

% Title annotation
add_block('simulink/Annotations/Note', [mdl '/Title_Note'], ...
    'Position', [50, -60, 800, -10]);
set_param([mdl '/Title_Note'], 'Text', ...
    ['Fighter NDI/INDI Flight Control System  |  ' ...
     'Delta-Canard LCA  |  Static Margin: -8% MAC (UNSTABLE)  |  ' ...
     'Run setup_fighter.m before simulation']);

%% ====================================================================
%  9. SOLVER AND SIMULATION PARAMETERS
%  ====================================================================
fprintf('  Configuring solver...\n');

set_param(mdl, 'Solver', 'ode4');             % Fixed-step RK4
set_param(mdl, 'FixedStep', num2str(dt));     % 0.004 s = 250 Hz
set_param(mdl, 'StopTime', '30');             % 30 second simulation
set_param(mdl, 'SaveFormat', 'Array');
set_param(mdl, 'SaveOutput', 'on');
set_param(mdl, 'SaveTime', 'on');
set_param(mdl, 'LimitDataPoints', 'off');
set_param(mdl, 'SignalLogging', 'on');
set_param(mdl, 'ReturnWorkspaceOutputs', 'on');

%% ====================================================================
%  10. CREATE PILOT INPUT TIMESERIES FOR FROM WORKSPACE BLOCKS
%  ====================================================================
fprintf('  Creating pilot input timeseries...\n');

t_sim = 0:dt:30;
N_sim = length(t_sim);

stick_lon_data = zeros(N_sim, 1);
stick_lat_data = zeros(N_sim, 1);
pedal_data     = zeros(N_sim, 1);

for k = 1:N_sim
    ti = t_sim(k);
    % Combat maneuver profile (same as run_fighter_sim.m)
    if ti >= 3 && ti < 8
        stick_lon_data(k) = 0.57;          % 4g pull
    elseif ti >= 12 && ti < 14
        stick_lat_data(k) = 0.6;           % Roll right
        stick_lon_data(k) = 0.43;          % 3g pull
    elseif ti >= 14 && ti < 17
        stick_lon_data(k) = 0.43;          % Hold 3g
    elseif ti >= 17 && ti < 19
        stick_lat_data(k) = -0.6;          % Roll left
        stick_lon_data(k) = -0.15;         % Push
    elseif ti >= 19 && ti < 22
        stick_lon_data(k) = -0.15;         % Hold push
    elseif ti >= 22 && ti < 24
        stick_lat_data(k) = 0.8;           % Hard right roll
    elseif ti >= 24 && ti < 26
        stick_lat_data(k) = -0.8;          % Hard left roll
    elseif ti >= 26 && ti < 27
        stick_lat_data(k) = 0.4;           % Stop roll
    end
end

% Create timeseries objects for From Workspace blocks
stick_lon_ts = timeseries(stick_lon_data, t_sim);
stick_lat_ts = timeseries(stick_lat_data, t_sim);
pedal_ts     = timeseries(pedal_data, t_sim);

% Export to base workspace
assignin('base', 'stick_lon_ts', stick_lon_ts);
assignin('base', 'stick_lat_ts', stick_lat_ts);
assignin('base', 'pedal_ts',     pedal_ts);

%% ====================================================================
%  11. SAVE MODEL
%  ====================================================================
fprintf('  Saving model...\n');

save_system(mdl);

fprintf('\n==========================================================\n');
fprintf(' MODEL BUILT SUCCESSFULLY: %s.slx\n', mdl);
fprintf('==========================================================\n\n');
fprintf('Block diagram structure:\n\n');
fprintf('  ┌──────────────┐   ┌──────────────────────┐   ┌──────────────┐   ┌──────────────────┐\n');
fprintf('  │ Pilot Inputs │──>│  NDI/INDI Controller  │──>│  Actuators   │──>│  6-DOF Fighter   │\n');
fprintf('  │ (stick,pedal │   │  (ndi_fcs_sfunc)      │   │ (TF+RateLim │   │  (fighter_plant_  │\n');
fprintf('  │  throttle)   │   │  250 Hz discrete      │   │  +Saturation)│   │   sfunc)         │\n');
fprintf('  └──────────────┘   └──────────────────────┘   └──────────────┘   └──────────────────┘\n');
fprintf('                           ↑  ↑                        │                     │\n');
fprintf('                           │  └── act positions ───────┘                     │\n');
fprintf('                           └──── vel, omega, euler, accel, omega_dot ────────┘\n');
fprintf('\n');
fprintf('Scopes (open at sim start):\n');
fprintf('  • Flight_Params    — Nz cmd/actual, alpha, beta, Mach, V\n');
fprintf('  • Attitude_Scope   — Euler angles + angular rates\n');
fprintf('  • Surface_Scope    — Commanded vs actual surface deflections\n');
fprintf('  • Position_Scope   — NED position\n');
fprintf('  • AirData_Scope    — Air data (alpha, beta, Mach, V, qbar, alt)\n');
fprintf('\n');
fprintf('To Workspace logging:\n');
fprintf('  log_debug, log_euler, log_surfaces, log_position, log_airdata\n');
fprintf('\n');
fprintf('Solver: ODE4 (fixed-step Runge-Kutta), dt = %g s (%d Hz)\n', dt, round(1/dt));
fprintf('Stop time: 30 s\n\n');
fprintf('=== HOW TO RUN ===\n');
fprintf('  1. Ensure setup_fighter has been run (check workspace for ''aircraft'')\n');
fprintf('  2. open_system(''%s'')\n', mdl);
fprintf('  3. Click Run (or: sim(''%s''))\n', mdl);
fprintf('  4. After simulation, run: plot_results  (see below)\n');
fprintf('\n');
fprintf('=== MODIFY PILOT INPUTS ===\n');
fprintf('  Edit stick_lon_ts, stick_lat_ts, pedal_ts timeseries in workspace\n');
fprintf('  Or replace From Workspace blocks with Joystick/Signal Builder\n');
fprintf('\n');
fprintf('=== SWITCH BETWEEN NDI and INDI ===\n');
fprintf('  >> fcs.use_indi = true;   %% INDI (sensor-based, more robust)\n');
fprintf('  >> fcs.use_indi = false;  %% Classical NDI (model-based)\n');
fprintf('  Then re-run the simulation.\n\n');
