function ucav_guidance_sfunc(block)
%UCAV_GUIDANCE_SFUNC  Level-2 S-Function: L1 lateral + PI altitude/speed guidance.
%
%  Wraps ucav_guidance_law.m for Simulink integration.
%  Manages waypoint index, lap count, and integrator states via DWork.
%
%  Inputs (4 ports):
%    Port 1: position  [N; E; D] (m)                       [3]
%    Port 2: velocity  [u; v; w] body (m/s)                [3]
%    Port 3: euler     [phi; theta; psi] (rad)             [3]
%    Port 4: airdata   [alpha; beta; Mach; V; qbar; alt]   [6]
%
%  Outputs (3 ports):
%    Port 1: att_cmd   [phi_cmd; theta_cmd] (rad)          [2]
%    Port 2: thr_cmd   throttle (0-1)                      [1]
%    Port 3: debug     [wp_idx; dist_wp; xtrack; alt_cmd;
%                        alt; V_cmd; V; lap_count]          [8]
%
%  DWork(1): [wp_idx; lap_count; int_alt; int_V]           [4]
%
%  Sample time: 0.004 s (250 Hz, discrete)

setup(block);

function setup(block)
    % --- 4 input ports ---
    block.NumInputPorts = 4;
    dims_in = [3, 3, 3, 6];
    for i = 1:4
        block.InputPort(i).Dimensions        = dims_in(i);
        block.InputPort(i).DatatypeID        = 0;
        block.InputPort(i).Complexity        = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode      = 'Sample';
    end

    % --- 3 output ports ---
    block.NumOutputPorts = 3;
    block.OutputPort(1).Dimensions   = 2;    % [phi_cmd; theta_cmd]
    block.OutputPort(1).DatatypeID   = 0;
    block.OutputPort(1).Complexity   = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    block.OutputPort(2).Dimensions   = 1;    % throttle
    block.OutputPort(2).DatatypeID   = 0;
    block.OutputPort(2).Complexity   = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';

    block.OutputPort(3).Dimensions   = 8;    % debug
    block.OutputPort(3).DatatypeID   = 0;
    block.OutputPort(3).Complexity   = 'Real';
    block.OutputPort(3).SamplingMode = 'Sample';

    % Discrete sample time: 250 Hz
    block.SampleTimes = [0.004, 0];

    % DWork: guidance state [wp_idx; lap_count; int_alt; int_V]
    block.NumDworks = 1;
    block.Dwork(1).Name            = 'GuidState';
    block.Dwork(1).Dimensions      = 4;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.RegBlockMethod('Start',                @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);

function Start(block)
    % Load mission and guidance parameters from workspace
    for v = {'mission', 'guidance', 'trim'}
        if ~evalin('base', ['exist(''' v{1} ''',''var'')'])
            error('[Guidance] Run setup_ucav.m first. Missing: %s', v{1});
        end
    end
    setappdata(0, 'GUID_mission',  evalin('base', 'mission'));
    setappdata(0, 'GUID_guidance', evalin('base', 'guidance'));
    setappdata(0, 'GUID_trim',    evalin('base', 'trim'));
    fprintf('[Guidance] L1 + PI guidance initialized.\n');

function InitCond(block)
    % wp_idx=2 (target WP2 first), lap=0, integrators=0
    block.Dwork(1).Data = [2; 0; 0; 0];

function Outputs(block)
    mis  = getappdata(0, 'GUID_mission');
    gp   = getappdata(0, 'GUID_guidance');
    trm  = getappdata(0, 'GUID_trim');
    dt   = block.SampleTimes(1);

    pos = block.InputPort(1).Data;     % [N; E; D]
    vel = block.InputPort(2).Data;     % [u; v; w]
    eul = block.InputPort(3).Data;     % [phi; theta; psi]
    ad  = block.InputPort(4).Data;     % [alpha; beta; Mach; V; qbar; alt]

    V = ad(4);

    % Reconstruct guidance state from DWork
    gs_d = block.Dwork(1).Data;
    gs.wp_idx    = gs_d(1);
    gs.lap_count = gs_d(2);
    gs.int_alt   = gs_d(3);
    gs.int_V     = gs_d(4);

    % Call guidance law
    [phi_cmd, theta_cmd, thr_cmd, ~, gd] = ucav_guidance_law( ...
        pos(1), pos(2), pos(3), vel(1), vel(2), vel(3), ...
        eul(1), eul(2), eul(3), V, gs, mis, gp, trm, dt);

    % Write outputs
    block.OutputPort(1).Data = [phi_cmd; theta_cmd];
    block.OutputPort(2).Data = thr_cmd;
    block.OutputPort(3).Data = [gd.wp_idx; gd.dist_to_wp; gd.xtrack; ...
        gd.alt_cmd; gd.alt; gd.V_cmd; V; gd.lap_count];

function Update(block)
    mis  = getappdata(0, 'GUID_mission');
    gp   = getappdata(0, 'GUID_guidance');
    trm  = getappdata(0, 'GUID_trim');
    dt   = block.SampleTimes(1);

    pos = block.InputPort(1).Data;
    vel = block.InputPort(2).Data;
    eul = block.InputPort(3).Data;
    ad  = block.InputPort(4).Data;

    gs_d = block.Dwork(1).Data;
    gs.wp_idx    = gs_d(1);
    gs.lap_count = gs_d(2);
    gs.int_alt   = gs_d(3);
    gs.int_V     = gs_d(4);

    [~, ~, ~, gs_out, ~] = ucav_guidance_law( ...
        pos(1), pos(2), pos(3), vel(1), vel(2), vel(3), ...
        eul(1), eul(2), eul(3), ad(4), gs, mis, gp, trm, dt);

    % Store updated guidance state
    block.Dwork(1).Data = [gs_out.wp_idx; gs_out.lap_count; ...
        gs_out.int_alt; gs_out.int_V];
