function ucav_autopilot_sfunc(block)
%UCAV_AUTOPILOT_SFUNC  Level-2 S-Function wrapper for NDI/INDI autopilot.
%
%  Inputs (7 ports):
%    Port 1: att_cmd   [phi_cmd; theta_cmd] from guidance (rad)  [2]
%    Port 2: velocity  [u; v; w] body (m/s)                      [3]
%    Port 3: omega     [p; q; r] body (rad/s)                    [3]
%    Port 4: euler     [phi; theta; psi] (rad)                   [3]
%    Port 5: accel     [ax; ay; az] body (m/s^2)                 [3]
%    Port 6: omega_dot [pdot; qdot; rdot] (rad/s^2)              [3]
%    Port 7: act_pos   [de_L; de_R; dr; dc] actual (rad)         [4]
%
%  Outputs (2 ports):
%    Port 1: surf_cmd  [de_L; de_R; dr; dc] commanded (rad)      [4]
%    Port 2: throttle  throttle command (from guidance passthru)  [1]
%
%  Note: Throttle is passed through from guidance; this block focuses
%  on attitude control surfaces only.

setup(block);

function setup(block)
    block.NumInputPorts = 7;
    dims_in = [2, 3, 3, 3, 3, 3, 4];
    for i = 1:7
        block.InputPort(i).Dimensions = dims_in(i);
        block.InputPort(i).DatatypeID = 0;
        block.InputPort(i).Complexity = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode = 'Sample';
    end

    block.NumOutputPorts = 2;
    block.OutputPort(1).Dimensions = 4;  % [de_L; de_R; dr; dc]
    block.OutputPort(1).DatatypeID = 0;
    block.OutputPort(1).Complexity = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    block.OutputPort(2).Dimensions = 1;  % throttle passthrough
    block.OutputPort(2).DatatypeID = 0;
    block.OutputPort(2).Complexity = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';

    block.SampleTimes = [0.004, 0];

    % DWork: [int_omega(3), de_L_prev, de_R_prev, dr_prev, dc_prev]
    block.NumDworks = 1;
    block.Dwork(1).Name = 'CtrlState';
    block.Dwork(1).Dimensions = 7;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.RegBlockMethod('Start', @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Update', @Update);

function Start(block)
    for v = {'aircraft','fcs'}
        if ~evalin('base', ['exist(''' v{1} ''',''var'')'])
            error('[Autopilot] Run setup_ucav.m first.');
        end
    end
    setappdata(0, 'AP_aircraft', evalin('base', 'aircraft'));
    setappdata(0, 'AP_fcs', evalin('base', 'fcs'));

function InitCond(block)
    if evalin('base', 'exist(''initial'',''var'')')
        ini = evalin('base', 'initial');
        block.Dwork(1).Data = [0;0;0; ini.de_L; ini.de_R; ini.dr; ini.dc];
    else
        block.Dwork(1).Data = zeros(7,1);
    end

function Outputs(block)
    ac  = getappdata(0, 'AP_aircraft');
    fcs = getappdata(0, 'AP_fcs');
    dt  = block.SampleTimes(1);

    att  = block.InputPort(1).Data;
    vel  = block.InputPort(2).Data;
    omg  = block.InputPort(3).Data;
    eul  = block.InputPort(4).Data;
    acc  = block.InputPort(5).Data;
    odot = block.InputPort(6).Data;
    apos = block.InputPort(7).Data;

    d = block.Dwork(1).Data;
    cs.int_omega = d(1:3);
    cs.de_L_prev = d(4); cs.de_R_prev = d(5);
    cs.dr_prev = d(6); cs.dc_prev = d(7);

    [deL, deR, dr, dc, ~, ~] = ndi_autopilot_controller( ...
        att(1), att(2), ...
        vel(1), vel(2), vel(3), omg(1), omg(2), omg(3), ...
        eul(1), eul(2), eul(3), ...
        acc(1), acc(2), acc(3), ...
        odot(1), odot(2), odot(3), ...
        apos(1), apos(2), apos(3), apos(4), ...
        cs, ac, fcs, dt);

    block.OutputPort(1).Data = [deL; deR; dr; dc];
    block.OutputPort(2).Data = 0.5;  % Default throttle (overridden by guidance)

function Update(block)
    ac  = getappdata(0, 'AP_aircraft');
    fcs = getappdata(0, 'AP_fcs');
    dt  = block.SampleTimes(1);

    att  = block.InputPort(1).Data;
    vel  = block.InputPort(2).Data;
    omg  = block.InputPort(3).Data;
    eul  = block.InputPort(4).Data;
    acc  = block.InputPort(5).Data;
    odot = block.InputPort(6).Data;
    apos = block.InputPort(7).Data;

    d = block.Dwork(1).Data;
    cs.int_omega = d(1:3);
    cs.de_L_prev = d(4); cs.de_R_prev = d(5);
    cs.dr_prev = d(6); cs.dc_prev = d(7);

    [deL, deR, dr, dc, cs_out, ~] = ndi_autopilot_controller( ...
        att(1), att(2), ...
        vel(1), vel(2), vel(3), omg(1), omg(2), omg(3), ...
        eul(1), eul(2), eul(3), ...
        acc(1), acc(2), acc(3), ...
        odot(1), odot(2), odot(3), ...
        apos(1), apos(2), apos(3), apos(4), ...
        cs, ac, fcs, dt);

    block.Dwork(1).Data = [cs_out.int_omega; ...
        cs_out.de_L_prev; cs_out.de_R_prev; ...
        cs_out.dr_prev; cs_out.dc_prev];
