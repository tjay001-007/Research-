function indi_controller_sfunc(block)
%INDI_CONTROLLER_SFUNC  Level-2 MATLAB S-Function: INDI flight controller.
%
%  Wraps indi_controller.m for Simulink integration. Manages integrator
%  and previous-command states via DWork vectors.
%
%  INDI (Incremental Nonlinear Dynamic Inversion):
%    delta = delta_prev + B_inv * (omega_dot_desired - omega_dot_measured)
%    - Only needs control effectiveness B (not full aero model)
%    - Uses measured angular acceleration for implicit model cancellation
%    - Robust to model errors, wind, and damage
%
%  Inputs (6 ports):
%    Port 1: att_cmd    [phi_cmd; theta_cmd] from guidance (rad)    [2]
%    Port 2: velocity   [u; v; w] body (m/s)                       [3]
%    Port 3: omega      [p; q; r] body (rad/s)                     [3]
%    Port 4: euler      [phi; theta; psi] (rad)                    [3]
%    Port 5: omega_dot  [pdot; qdot; rdot] measured (rad/s^2)      [3]
%    Port 6: act_pos    [da; de; dr] actual positions (rad)         [3]
%
%  Outputs (2 ports):
%    Port 1: surf_cmd   [da_cmd; de_cmd; dr_cmd] (rad)             [3]
%    Port 2: debug      [p_cmd; q_cmd; r_cmd; alpha; beta; V;
%                         pdot_des; qdot_des]                       [8]
%
%  DWork vectors:
%    DWork(1): int_omega [3]  — Rate integrator states (int_p, int_q, int_r)
%
%  Sample time: 0.004 s (250 Hz, discrete)

setup(block);

function setup(block)
    % --- 6 input ports ---
    block.NumInputPorts = 6;
    in_dims = [2, 3, 3, 3, 3, 3];
    for i = 1:6
        block.InputPort(i).Dimensions        = in_dims(i);
        block.InputPort(i).DatatypeID        = 0;    % double
        block.InputPort(i).Complexity        = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode      = 'Sample';
    end

    % --- 2 output ports ---
    block.NumOutputPorts = 2;
    block.OutputPort(1).Dimensions   = 3;     % [da_cmd; de_cmd; dr_cmd]
    block.OutputPort(1).DatatypeID   = 0;
    block.OutputPort(1).Complexity   = 'Real';
    block.OutputPort(1).SamplingMode = 'Sample';

    block.OutputPort(2).Dimensions   = 8;     % debug signals
    block.OutputPort(2).DatatypeID   = 0;
    block.OutputPort(2).Complexity   = 'Real';
    block.OutputPort(2).SamplingMode = 'Sample';

    % Discrete sample time: 250 Hz
    block.SampleTimes = [0.004, 0];

    % --- DWork: integrator states [int_p; int_q; int_r] ---
    block.NumDworks = 1;
    block.Dwork(1).Name            = 'int_omega';
    block.Dwork(1).Dimensions      = 3;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    % Register callbacks
    block.RegBlockMethod('Start',                @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);

function Start(block)
    % Load parameters from workspace
    if ~evalin('base', 'exist(''aircraft'',''var'')') || ...
       ~evalin('base', 'exist(''fcs'',''var'')')
        error('[INDI] Run setup_ucav.m before starting the model.');
    end
    setappdata(0, 'INDI_aircraft', evalin('base', 'aircraft'));
    setappdata(0, 'INDI_fcs',      evalin('base', 'fcs'));
    fprintf('[INDI] Controller initialized (INDI mode = %d).\n', ...
        evalin('base', 'fcs.use_indi'));

function InitCond(block)
    % Zero integrators
    block.Dwork(1).Data = zeros(3, 1);

function Outputs(block)
    ac  = getappdata(0, 'INDI_aircraft');
    fcs = getappdata(0, 'INDI_fcs');
    dt  = block.SampleTimes(1);

    % Read inputs
    att_cmd   = block.InputPort(1).Data;    % [phi_cmd; theta_cmd]
    vel       = block.InputPort(2).Data;    % [u; v; w]
    omega     = block.InputPort(3).Data;    % [p; q; r]
    euler     = block.InputPort(4).Data;    % [phi; theta; psi]
    omega_dot = block.InputPort(5).Data;    % [pdot; qdot; rdot]
    act_pos   = block.InputPort(6).Data;    % [da; de; dr] actual

    % Build controller state from DWork
    int_w = block.Dwork(1).Data;
    ctrl_in.int_p = int_w(1);
    ctrl_in.int_q = int_w(2);
    ctrl_in.int_r = int_w(3);

    % Call INDI controller
    [da_cmd, de_cmd, dr_cmd, ~, dbg] = indi_controller( ...
        att_cmd(1), att_cmd(2), ...
        vel(1), vel(2), vel(3), ...
        omega(1), omega(2), omega(3), ...
        euler(1), euler(2), euler(3), ...
        omega_dot(1), omega_dot(2), omega_dot(3), ...
        act_pos(1), act_pos(2), act_pos(3), ...
        ctrl_in, ac, fcs, dt);

    % Write outputs
    block.OutputPort(1).Data = [da_cmd; de_cmd; dr_cmd];
    block.OutputPort(2).Data = [dbg.p_cmd; dbg.q_cmd; dbg.r_cmd; ...
        dbg.alpha; dbg.beta; dbg.V; dbg.pdot_des; dbg.qdot_des];

function Update(block)
    ac  = getappdata(0, 'INDI_aircraft');
    fcs = getappdata(0, 'INDI_fcs');
    dt  = block.SampleTimes(1);

    att_cmd   = block.InputPort(1).Data;
    vel       = block.InputPort(2).Data;
    omega     = block.InputPort(3).Data;
    euler     = block.InputPort(4).Data;
    omega_dot = block.InputPort(5).Data;
    act_pos   = block.InputPort(6).Data;

    int_w = block.Dwork(1).Data;
    ctrl_in.int_p = int_w(1);
    ctrl_in.int_q = int_w(2);
    ctrl_in.int_r = int_w(3);

    % Re-run controller to get updated state
    [~, ~, ~, ctrl_out, ~] = indi_controller( ...
        att_cmd(1), att_cmd(2), ...
        vel(1), vel(2), vel(3), ...
        omega(1), omega(2), omega(3), ...
        euler(1), euler(2), euler(3), ...
        omega_dot(1), omega_dot(2), omega_dot(3), ...
        act_pos(1), act_pos(2), act_pos(3), ...
        ctrl_in, ac, fcs, dt);

    % Store updated integrator states
    block.Dwork(1).Data = [ctrl_out.int_p; ctrl_out.int_q; ctrl_out.int_r];
