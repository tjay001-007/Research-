function ndi_fcs_sfunc(block)
%NDI_FCS_SFUNC  Level-2 MATLAB S-Function: Production NDI/INDI flight
%   control system for the delta-canard fighter in Simulink.
%
%   Wraps ndi_production_controller.m and manages all discrete internal
%   states (integrators, filters, reference model memory).
%
%   Inputs (7 ports):
%     Port 1: pilot_cmd   [stick_lon; stick_lat; pedal; throttle]  [4]
%     Port 2: velocity    [u; v; w]  (m/s)                        [3]
%     Port 3: omega       [p; q; r]  (rad/s)                      [3]
%     Port 4: euler       [phi; theta; psi]  (rad)                 [3]
%     Port 5: accel_body  [ax; ay; az]  (m/s^2)                   [3]
%     Port 6: omega_dot   [pdot; qdot; rdot]  (rad/s^2)           [3]
%     Port 7: act_pos     [de_L; de_R; dr; dc]  actual (rad)      [4]
%
%   Outputs (3 ports):
%     Port 1: surf_cmd    [de_L; de_R; dr; dc]  commanded (rad)   [4]
%     Port 2: thr_cmd     throttle command (0-1.5)                 [1]
%     Port 3: debug       [Nz_cmd; Nz_act; alpha; beta; Mach; V;
%                          p_cmd; q_cmd]                           [8]
%
%   DWork (discrete state) layout:
%     DWork(1): int_omega [3]     Rate integrator
%     DWork(2): ref_model [3]     Nz_filt, p_filt, beta_filt
%     DWork(3): filt_x1   [4]     Structural filter position states
%     DWork(4): filt_x2   [4]     Structural filter velocity states
%     DWork(5): omega_prev [3]    Previous omega for INDI accel estimate

setup(block);

function setup(block)
    % --- Input ports ---
    block.NumInputPorts = 7;
    in_dims = [4, 3, 3, 3, 3, 3, 4];
    for i = 1:7
        block.InputPort(i).Dimensions        = in_dims(i);
        block.InputPort(i).DatatypeID        = 0;
        block.InputPort(i).Complexity        = 'Real';
        block.InputPort(i).DirectFeedthrough = true;
        block.InputPort(i).SamplingMode      = 'Sample';
    end

    % --- Output ports ---
    block.NumOutputPorts = 3;
    out_dims = [4, 1, 8];
    for i = 1:3
        block.OutputPort(i).Dimensions   = out_dims(i);
        block.OutputPort(i).DatatypeID   = 0;
        block.OutputPort(i).Complexity   = 'Real';
        block.OutputPort(i).SamplingMode = 'Sample';
    end

    % Discrete sample time: 250 Hz
    block.SampleTimes = [0.004, 0];

    % --- DWork vectors ---
    block.NumDworks = 5;

    block.Dwork(1).Name = 'int_omega';
    block.Dwork(1).Dimensions = 3;
    block.Dwork(1).DatatypeID = 0;
    block.Dwork(1).Complexity = 'Real';
    block.Dwork(1).UsedAsDiscState = true;

    block.Dwork(2).Name = 'ref_model';
    block.Dwork(2).Dimensions = 3;
    block.Dwork(2).DatatypeID = 0;
    block.Dwork(2).Complexity = 'Real';
    block.Dwork(2).UsedAsDiscState = true;

    block.Dwork(3).Name = 'filt_x1';
    block.Dwork(3).Dimensions = 4;
    block.Dwork(3).DatatypeID = 0;
    block.Dwork(3).Complexity = 'Real';
    block.Dwork(3).UsedAsDiscState = true;

    block.Dwork(4).Name = 'filt_x2';
    block.Dwork(4).Dimensions = 4;
    block.Dwork(4).DatatypeID = 0;
    block.Dwork(4).Complexity = 'Real';
    block.Dwork(4).UsedAsDiscState = true;

    block.Dwork(5).Name = 'omega_prev';
    block.Dwork(5).Dimensions = 3;
    block.Dwork(5).DatatypeID = 0;
    block.Dwork(5).Complexity = 'Real';
    block.Dwork(5).UsedAsDiscState = true;

    % Register methods
    block.RegBlockMethod('Start',                @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Update',               @Update);

function Start(block)
    ok = true;
    if evalin('base', 'exist(''aircraft'', ''var'')')
        setappdata(0, 'NDI_FCS_AC', evalin('base', 'aircraft'));
    else
        ok = false;
    end
    if evalin('base', 'exist(''fcs'', ''var'')')
        setappdata(0, 'NDI_FCS_Gains', evalin('base', 'fcs'));
    else
        ok = false;
    end
    if ~ok
        error('[NDI FCS] Run setup_fighter.m before starting model.');
    end
    fprintf('[NDI FCS] Controller initialised (%s mode)\n', ...
        tern(evalin('base', 'fcs.use_indi'), 'INDI', 'NDI'));

function InitCond(block)
    block.Dwork(1).Data = zeros(3,1);          % int_omega
    block.Dwork(2).Data = [1; 0; 0];           % ref_model (1g, 0, 0)
    if evalin('base', 'exist(''initial'', ''var'')')
        ini = evalin('base', 'initial');
        block.Dwork(3).Data = [ini.de_L; ini.de_R; ini.dr; ini.dc];
    else
        block.Dwork(3).Data = zeros(4,1);
    end
    block.Dwork(4).Data = zeros(4,1);          % filt_x2
    block.Dwork(5).Data = zeros(3,1);          % omega_prev

function Outputs(block)
    ac  = getappdata(0, 'NDI_FCS_AC');
    fcs = getappdata(0, 'NDI_FCS_Gains');
    dt  = block.SampleTimes(1);

    % Read inputs
    pilot    = block.InputPort(1).Data;
    vel      = block.InputPort(2).Data;
    omega    = block.InputPort(3).Data;
    euler    = block.InputPort(4).Data;
    accel    = block.InputPort(5).Data;
    omega_dt = block.InputPort(6).Data;
    act_pos  = block.InputPort(7).Data;

    % Reconstruct controller state from DWork
    ctrl_st = build_ctrl_state(block);

    % Call production controller
    [de_L, de_R, dr, dc, thr, dbg] = ndi_production_controller( ...
        pilot(1), pilot(2), pilot(3), pilot(4), ...
        vel(1), vel(2), vel(3), ...
        omega(1), omega(2), omega(3), ...
        euler(1), euler(2), euler(3), ...
        accel(1), accel(2), accel(3), ...
        omega_dt(1), omega_dt(2), omega_dt(3), ...
        act_pos(1), act_pos(2), act_pos(3), act_pos(4), ...
        ctrl_st, ac, fcs, dt);

    % Write outputs
    block.OutputPort(1).Data = [de_L; de_R; dr; dc];
    block.OutputPort(2).Data = thr;
    block.OutputPort(3).Data = [dbg.Nz_cmd; dbg.Nz_actual; ...
        dbg.alpha_deg; dbg.beta_deg; dbg.Mach; dbg.V; ...
        dbg.omega_cmd(1); dbg.omega_cmd(2)];

function Update(block)
    ac  = getappdata(0, 'NDI_FCS_AC');
    fcs = getappdata(0, 'NDI_FCS_Gains');
    dt  = block.SampleTimes(1);

    pilot    = block.InputPort(1).Data;
    vel      = block.InputPort(2).Data;
    omega    = block.InputPort(3).Data;
    euler    = block.InputPort(4).Data;
    accel    = block.InputPort(5).Data;
    omega_dt = block.InputPort(6).Data;
    act_pos  = block.InputPort(7).Data;

    ctrl_st = build_ctrl_state(block);

    [~, ~, ~, ~, ~, dbg] = ndi_production_controller( ...
        pilot(1), pilot(2), pilot(3), pilot(4), ...
        vel(1), vel(2), vel(3), ...
        omega(1), omega(2), omega(3), ...
        euler(1), euler(2), euler(3), ...
        accel(1), accel(2), accel(3), ...
        omega_dt(1), omega_dt(2), omega_dt(3), ...
        act_pos(1), act_pos(2), act_pos(3), act_pos(4), ...
        ctrl_st, ac, fcs, dt);

    % Store updated controller state back into DWork
    st = dbg.state;
    block.Dwork(1).Data = st.int_omega;
    block.Dwork(2).Data = [st.Nz_cmd_filt; st.p_cmd_filt; st.beta_cmd_filt];
    block.Dwork(3).Data = [st.filt_de_L.x1; st.filt_de_R.x1; ...
                           st.filt_dr.x1;   st.filt_dc.x1];
    block.Dwork(4).Data = [st.filt_de_L.x2; st.filt_de_R.x2; ...
                           st.filt_dr.x2;   st.filt_dc.x2];
    block.Dwork(5).Data = omega;

% =====================================================================
%  HELPERS
% =====================================================================

function st = build_ctrl_state(block)
    % Reconstruct the state struct expected by ndi_production_controller
    st.int_omega     = block.Dwork(1).Data;
    rm               = block.Dwork(2).Data;
    st.Nz_cmd_filt   = rm(1);
    st.p_cmd_filt    = rm(2);
    st.beta_cmd_filt = rm(3);
    x1 = block.Dwork(3).Data;
    x2 = block.Dwork(4).Data;
    st.filt_de_L = struct('x1', x1(1), 'x2', x2(1));
    st.filt_de_R = struct('x1', x1(2), 'x2', x2(2));
    st.filt_dr   = struct('x1', x1(3), 'x2', x2(3));
    st.filt_dc   = struct('x1', x1(4), 'x2', x2(4));

function r = tern(c, a, b)
    if c, r = a; else, r = b; end
