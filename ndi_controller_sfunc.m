function ndi_controller_sfunc(block)
%NDI_CONTROLLER_SFUNC  Level-2 MATLAB S-Function implementing cascaded NDI
%   for pitch-unstable aircraft stabilization in Simulink.
%
%   This S-function wraps ndi_flight_controller.m and manages the discrete
%   integrator states internally via DWork vectors.
%
%   Inputs (4 ports):
%     Port 1: attitude_cmd  [phi_cmd; theta_cmd]  (rad)       [2x1]
%     Port 2: velocity      [u; v; w]             (m/s)       [3x1]
%     Port 3: omega         [p; q; r]             (rad/s)     [3x1]
%     Port 4: euler         [phi; theta; psi]     (rad)       [3x1]
%
%   Outputs (2 ports):
%     Port 1: surfaces      [aileron; elevator; rudder]       [3x1]
%     Port 2: debug         [p_cmd; q_cmd; r_cmd; alpha; beta; V] [6x1]
%
%   Compatible with: aircraft_6dof_sfunc.m
%
%   Usage in Simulink:
%     Add a Level-2 MATLAB S-Function block, set function name to
%     'ndi_controller_sfunc'. Ensure 'aircraft' and 'ndi_gains' structs
%     exist in the base workspace (run setup_ndi_controller.m first).

setup(block);

%% ====================================================================
function setup(block)

    % --- Input ports ---
    block.NumInputPorts = 4;

    % Port 1: attitude commands [phi_cmd; theta_cmd]
    block.InputPort(1).Dimensions        = 2;
    block.InputPort(1).DatatypeID        = 0;    % double
    block.InputPort(1).Complexity        = 'Real';
    block.InputPort(1).DirectFeedthrough = true;
    block.InputPort(1).SamplingMode      = 'Sample';

    % Port 2: body velocities [u; v; w]
    block.InputPort(2).Dimensions        = 3;
    block.InputPort(2).DatatypeID        = 0;
    block.InputPort(2).Complexity        = 'Real';
    block.InputPort(2).DirectFeedthrough = true;
    block.InputPort(2).SamplingMode      = 'Sample';

    % Port 3: body angular rates [p; q; r]
    block.InputPort(3).Dimensions        = 3;
    block.InputPort(3).DatatypeID        = 0;
    block.InputPort(3).Complexity        = 'Real';
    block.InputPort(3).DirectFeedthrough = true;
    block.InputPort(3).SamplingMode      = 'Sample';

    % Port 4: Euler angles [phi; theta; psi]
    block.InputPort(4).Dimensions        = 3;
    block.InputPort(4).DatatypeID        = 0;
    block.InputPort(4).Complexity        = 'Real';
    block.InputPort(4).DirectFeedthrough = true;
    block.InputPort(4).SamplingMode      = 'Sample';

    % --- Output ports ---
    block.NumOutputPorts = 2;

    % Port 1: control surface commands [da; de; dr]
    block.OutputPort(1).Dimensions   = 3;
    block.OutputPort(1).DatatypeID   = 0;
    block.OutputPort(1).Complexity   = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    % Port 2: debug signals [p_cmd; q_cmd; r_cmd; alpha; beta; V]
    block.OutputPort(2).Dimensions   = 6;
    block.OutputPort(2).DatatypeID   = 0;
    block.OutputPort(2).Complexity   = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';

    % --- Discrete sample time (matches plant at 250 Hz) ---
    block.SampleTimes = [0.004, 0];

    % --- DWork vectors for integrator states ---
    block.NumDworks = 1;
    block.Dwork(1).Name            = 'IntState';
    block.Dwork(1).Dimensions      = 3;       % [int_p; int_q; int_r]
    block.Dwork(1).DatatypeID      = 0;       % double
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    % --- Register methods ---
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);
    block.RegBlockMethod('Start',                @Start);

%% ====================================================================
function Start(block)
    % Load controller parameters from workspace
    if evalin('base', 'exist(''ndi_gains'', ''var'')')
        ndi_gains = evalin('base', 'ndi_gains');
        setappdata(0, 'NDI_Gains', ndi_gains);
        disp('[NDI] Loaded controller gains from workspace');
    else
        ndi_gains = get_default_ndi_gains();
        setappdata(0, 'NDI_Gains', ndi_gains);
        disp('[NDI] Using default controller gains');
    end

    if evalin('base', 'exist(''aircraft'', ''var'')')
        aircraft = evalin('base', 'aircraft');
        setappdata(0, 'NDI_Aircraft', aircraft);
        disp('[NDI] Loaded aircraft parameters from workspace');
    else
        error('[NDI] Aircraft parameters not found in workspace. Run setup_ndi_controller.m first.');
    end

%% ====================================================================
function InitializeConditions(block)
    % Zero the integrator states
    block.Dwork(1).Data = zeros(3, 1);

%% ====================================================================
function Outputs(block)
    % --- Read parameters ---
    aircraft  = getappdata(0, 'NDI_Aircraft');
    ndi_gains = getappdata(0, 'NDI_Gains');

    % --- Read inputs ---
    att_cmd = block.InputPort(1).Data;    % [phi_cmd; theta_cmd]
    vel     = block.InputPort(2).Data;    % [u; v; w]
    omega   = block.InputPort(3).Data;    % [p; q; r]
    euler   = block.InputPort(4).Data;    % [phi; theta; psi]

    phi_cmd   = att_cmd(1);
    theta_cmd = att_cmd(2);
    u = vel(1);  v = vel(2);  w = vel(3);
    p = omega(1); q = omega(2); r = omega(3);
    phi = euler(1); theta = euler(2);

    % --- Read integrator state ---
    int_state = block.Dwork(1).Data;

    % --- Call NDI controller ---
    [da, de, dr, ~, dbg] = ndi_flight_controller( ...
        phi_cmd, theta_cmd, ...
        u, v, w, p, q, r, phi, theta, ...
        int_state, aircraft, ndi_gains, block.SampleTimes(1));

    % --- Write outputs ---
    block.OutputPort(1).Data = [da; de; dr];
    block.OutputPort(2).Data = [dbg.omega_cmd(1); dbg.omega_cmd(2); dbg.omega_cmd(3); ...
                                dbg.alpha; dbg.beta; dbg.V];

%% ====================================================================
function Update(block)
    % --- Re-run the controller to get updated integrator state ---
    %     (Simulink separates Output and Update to handle algebraic loops)
    aircraft  = getappdata(0, 'NDI_Aircraft');
    ndi_gains = getappdata(0, 'NDI_Gains');

    att_cmd = block.InputPort(1).Data;
    vel     = block.InputPort(2).Data;
    omega   = block.InputPort(3).Data;
    euler   = block.InputPort(4).Data;

    int_state = block.Dwork(1).Data;

    [~, ~, ~, int_new, ~] = ndi_flight_controller( ...
        att_cmd(1), att_cmd(2), ...
        vel(1), vel(2), vel(3), ...
        omega(1), omega(2), omega(3), ...
        euler(1), euler(2), ...
        int_state, aircraft, ndi_gains, block.SampleTimes(1));

    block.Dwork(1).Data = int_new;

%% ====================================================================
function gains = get_default_ndi_gains()
    % Default NDI gains — tuned for the pitch-unstable aircraft in
    % setup_aircraft_parameters.m

    % Outer loop bandwidths (rad/s)
    gains.K_phi   = 3.0;       % Roll attitude bandwidth
    gains.K_theta = 4.0;       % Pitch attitude bandwidth (higher for unstable)
    gains.K_beta  = 2.0;       % Sideslip suppression gain

    % Inner loop proportional gains (rad/s) — sets closed-loop bandwidth
    gains.K_p = 10.0;          % Roll rate
    gains.K_q = 12.0;          % Pitch rate (higher for unstable aircraft)
    gains.K_r = 8.0;           % Yaw rate

    % Inner loop integral gains (rad/s^2) — rejects model mismatch
    gains.Ki_p = 2.0;
    gains.Ki_q = 4.0;          % Higher integral for pitch (Cmalpha uncertainty)
    gains.Ki_r = 1.5;

    % Integrator anti-windup limits (rad)
    gains.int_lim_p = 0.5;
    gains.int_lim_q = 0.5;
    gains.int_lim_r = 0.5;

    % Rate command limits (rad/s)
    gains.p_max = 3.0;         % Max roll rate
    gains.q_max = 2.0;         % Max pitch rate
    gains.r_max = 1.5;         % Max yaw rate
