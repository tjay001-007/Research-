function ucav_guidance_sfunc(block)
%UCAV_GUIDANCE_SFUNC  Level-2 S-Function wrapper for L1+TECS guidance law.
%
%  Inputs (4 ports):
%    Port 1: position [N; E; D] (m)                       [3]
%    Port 2: velocity [u; v; w] body (m/s)                [3]
%    Port 3: euler    [phi; theta; psi] (rad)             [3]
%    Port 4: airdata  [alpha; beta; Mach; V; qbar; alt]   [6]
%
%  Outputs (2 ports):
%    Port 1: commands [phi_cmd; theta_cmd] (rad)           [2]
%    Port 2: debug    [wp_idx; dist_wp; XTE; alt_cmd;
%                      V_cmd; throttle_cmd; alt; lap]      [8]

setup(block);

function setup(block)
    block.NumInputPorts = 4;
    dims_in = [3, 3, 3, 6];
    for i = 1:4
        block.InputPort(i).Dimensions = dims_in(i);
        block.InputPort(i).DatatypeID = 0;
        block.InputPort(i).Complexity = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode = 'Sample';
    end

    block.NumOutputPorts = 3;
    block.OutputPort(1).Dimensions = 2;   % [phi_cmd; theta_cmd]
    block.OutputPort(2).Dimensions = 1;   % throttle_cmd
    block.OutputPort(3).Dimensions = 8;   % debug
    for i = 1:3
        block.OutputPort(i).DatatypeID = 0;
        block.OutputPort(i).Complexity = 'Real';
        block.OutputPort(i).SamplingMode = 'Sample';
    end

    block.SampleTimes = [0.004, 0];

    % DWork: guidance state [wp_idx, lap_count, int_hdot, int_V]
    block.NumDworks = 1;
    block.Dwork(1).Name = 'GuidState';
    block.Dwork(1).Dimensions = 4;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);

function Start(block)
    for v = {'mission','guidance'}
        if ~evalin('base', ['exist(''' v{1} ''',''var'')'])
            error('[Guidance] Run setup_ucav.m first.');
        end
    end
    setappdata(0, 'GUID_mission', evalin('base', 'mission'));
    setappdata(0, 'GUID_params', evalin('base', 'guidance'));

function InitCond(block)
    block.Dwork(1).Data = [2; 0; 0; 0]; % wp_idx=2, lap=0, int_hdot=0, int_V=0

function Outputs(block)
    mis = getappdata(0, 'GUID_mission');
    gp  = getappdata(0, 'GUID_params');
    dt  = block.SampleTimes(1);

    pos = block.InputPort(1).Data;
    vel = block.InputPort(2).Data;
    eul = block.InputPort(3).Data;
    ad  = block.InputPort(4).Data;

    gs_d = block.Dwork(1).Data;
    gs.wp_idx = gs_d(1); gs.lap_count = gs_d(2);
    gs.int_hdot = gs_d(3); gs.int_V = gs_d(4);

    V = ad(4);

    [phi_cmd, theta_cmd, thr_cmd, gs_out, gd] = ucav_guidance_law( ...
        pos(1), pos(2), pos(3), vel(1), vel(2), vel(3), ...
        eul(1), eul(2), eul(3), V, gs, mis, gp, dt);

    block.OutputPort(1).Data = [phi_cmd; theta_cmd];
    block.OutputPort(2).Data = thr_cmd;
    block.OutputPort(3).Data = [gd.wp_idx; gd.dist_to_wp; gd.crosstrack_error; ...
        gd.alt_cmd; gd.V_cmd; thr_cmd; -pos(3); gd.lap_count];

function Update(block)
    mis = getappdata(0, 'GUID_mission');
    gp  = getappdata(0, 'GUID_params');
    dt  = block.SampleTimes(1);

    pos = block.InputPort(1).Data;
    vel = block.InputPort(2).Data;
    eul = block.InputPort(3).Data;
    ad  = block.InputPort(4).Data;

    gs_d = block.Dwork(1).Data;
    gs.wp_idx = gs_d(1); gs.lap_count = gs_d(2);
    gs.int_hdot = gs_d(3); gs.int_V = gs_d(4);

    [~,~,~, gs_out, ~] = ucav_guidance_law( ...
        pos(1), pos(2), pos(3), vel(1), vel(2), vel(3), ...
        eul(1), eul(2), eul(3), ad(4), gs, mis, gp, dt);

    block.Dwork(1).Data = [gs_out.wp_idx; gs_out.lap_count; ...
        gs_out.int_hdot; gs_out.int_V];
