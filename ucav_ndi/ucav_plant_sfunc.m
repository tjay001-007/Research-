function ucav_plant_sfunc(block)
%UCAV_PLANT_SFUNC  Level-2 MATLAB S-Function: 6-DOF UCAV dynamics.
%
%  Full nonlinear rigid-body dynamics with:
%    - Full inertia tensor (Ixz product of inertia)
%    - Stability-derivative aerodynamics via ucav_aerodynamics.m
%    - ISA atmospheric model (density varies with altitude)
%    - Conventional config: aileron + elevator + rudder (NO canard)
%
%  Inputs (4 ports):
%    Port 1: da        Aileron deflection (rad)           [1]
%    Port 2: de        Elevator deflection (rad)          [1]
%    Port 3: dr        Rudder deflection (rad)            [1]
%    Port 4: throttle  Throttle command (0-1)             [1]
%
%  Outputs (7 ports):
%    Port 1: position  [N; E; D] NED (m)                  [3]
%    Port 2: velocity  [u; v; w] body (m/s)               [3]
%    Port 3: omega     [p; q; r] body (rad/s)             [3]
%    Port 4: euler     [phi; theta; psi] (rad)            [3]
%    Port 5: accel     [ax; ay; az] body (m/s^2)          [3]
%    Port 6: omega_dot [pdot; qdot; rdot] (rad/s^2)       [3]
%    Port 7: airdata   [alpha; beta; Mach; V; qbar; alt]  [6]
%
%  State vector (12 continuous states):
%    [u v w p q r N E D phi theta psi]'
%
%  Usage:
%    1. Run setup_ucav to load 'aircraft' and 'initial' into workspace
%    2. Add Level-2 MATLAB S-Function block in Simulink
%    3. Set S-function name to 'ucav_plant_sfunc'

setup(block);

function setup(block)
    % --- 4 scalar inputs: da, de, dr, throttle ---
    block.NumInputPorts = 4;
    for i = 1:4
        block.InputPort(i).Dimensions        = 1;
        block.InputPort(i).DatatypeID        = 0;    % double
        block.InputPort(i).Complexity        = 'Real';
        block.InputPort(i).DirectFeedthrough = false;
        block.InputPort(i).SamplingMode      = 'Sample';
    end

    % --- 7 output ports ---
    block.NumOutputPorts = 7;
    out_dims = [3, 3, 3, 3, 3, 3, 6];
    for i = 1:7
        block.OutputPort(i).Dimensions   = out_dims(i);
        block.OutputPort(i).DatatypeID   = 0;
        block.OutputPort(i).Complexity   = 'Real';
        block.OutputPort(i).SamplingMode = 'Sample';
    end

    % Continuous sample time (plant is continuous)
    block.SampleTimes = [0, 0];

    % 12 continuous states
    block.NumContStates = 12;

    % Register callbacks
    block.RegBlockMethod('Start',                @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);

function Start(block)
    % Load aircraft parameters from workspace
    if evalin('base', 'exist(''aircraft'', ''var'')')
        ac = evalin('base', 'aircraft');
        setappdata(0, 'UCAV_Plant_AC', ac);
        fprintf('[UCAV Plant] Aircraft parameters loaded.\n');
    else
        error('[UCAV Plant] ''aircraft'' not in workspace. Run setup_ucav.m first.');
    end

function InitCond(block)
    % Set initial conditions from workspace
    if evalin('base', 'exist(''initial'', ''var'')')
        ini = evalin('base', 'initial');
        block.ContStates.Data = ini.state;
    else
        % Default: level flight at 70 m/s, 1000 m altitude, heading North
        block.ContStates.Data = [70;0;1.2; 0;0;0; 0;0;-1000; 0;0.017;0];
    end

function Outputs(block)
    ac = getappdata(0, 'UCAV_Plant_AC');
    x  = block.ContStates.Data;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11); psi = x(12);
    alt = max(0, -x(9));

    % Air data
    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);
    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    [~, a_snd, ~, rho] = isa_atm_plant(alt);
    Mach = V / a_snd;
    qbar = 0.5 * rho * V^2;

    % Read control inputs for acceleration computation
    da  = block.InputPort(1).Data;
    de  = block.InputPort(2).Data;
    dr  = block.InputPort(3).Data;
    thr = block.InputPort(4).Data;

    % Compute body accelerations and angular accelerations (for INDI feedback)
    [accel_body, omega_dot] = compute_accels(x, da, de, dr, thr, ac);

    % Write outputs
    block.OutputPort(1).Data = x(7:9);                              % position NED
    block.OutputPort(2).Data = x(1:3);                              % velocity body
    block.OutputPort(3).Data = x(4:6);                              % omega body
    block.OutputPort(4).Data = x(10:12);                            % euler angles
    block.OutputPort(5).Data = accel_body;                          % body accels
    block.OutputPort(6).Data = omega_dot;                           % angular accels
    block.OutputPort(7).Data = [alpha; beta; Mach; V; qbar; alt];  % air data

function Derivatives(block)
    ac = getappdata(0, 'UCAV_Plant_AC');
    x  = block.ContStates.Data;

    da  = block.InputPort(1).Data;
    de  = block.InputPort(2).Data;
    dr  = block.InputPort(3).Data;
    thr = block.InputPort(4).Data;

    xdot = eom_6dof_plant(x, da, de, dr, thr, ac);
    block.Derivatives.Data = xdot;

% =====================================================================
%  EQUATIONS OF MOTION
% =====================================================================

function xdot = eom_6dof_plant(x, da, de, dr, thr, ac)
    g = 9.81;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11);
    alt = max(0, -x(9));

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);
    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    % Aerodynamic forces and moments
    [F_aero, M_aero, ~] = ucav_aerodynamics(alpha, beta, V, alt, ...
        p, q, r, da, de, dr, ac);

    % Thrust (along body x-axis)
    T = ac.engine.idle_thrust + ...
        (ac.engine.thrust_max - ac.engine.idle_thrust) * max(min(thr, 1), 0);

    % Gravity in body frame
    Fg_x = -ac.mass * g * sin(theta);
    Fg_y =  ac.mass * g * cos(theta) * sin(phi);
    Fg_z =  ac.mass * g * cos(theta) * cos(phi);

    Fx = F_aero(1) + T + Fg_x;
    Fy = F_aero(2) + Fg_y;
    Fz = F_aero(3) + Fg_z;

    % Translational dynamics
    udot = Fx/ac.mass + r*v_b - q*w_b;
    vdot = Fy/ac.mass + p*w_b - r*u_b;
    wdot = Fz/ac.mass + q*u_b - p*v_b;

    % Rotational dynamics (full inertia with Ixz)
    Gam = ac.Gamma;
    pdot = (ac.Izz*M_aero(1) + ac.Ixz*M_aero(3) ...
            - (ac.Ixz*(ac.Ixx - ac.Iyy + ac.Izz))*p*q ...
            + (ac.Ixz^2 + ac.Izz*(ac.Izz - ac.Iyy))*q*r) / Gam;
    qdot = (M_aero(2) + (ac.Ixx - ac.Izz)*p*r - ac.Ixz*(p^2 - r^2)) / ac.Iyy;
    rdot = (ac.Ixx*M_aero(3) + ac.Ixz*M_aero(1) ...
            + (ac.Ixz*(ac.Iyy - ac.Izz - ac.Ixx))*q*r ...
            + (ac.Ixx*(ac.Ixx - ac.Iyy) + ac.Ixz^2)*p*q) / Gam;

    % Euler kinematics
    ct = cos(theta);
    if abs(ct) < 0.001, ct = sign(ct)*0.001; end

    phidot   = p + (q*sin(phi) + r*cos(phi)) * tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / ct;

    % Navigation (body to NED)
    sp = sin(phi); cp = cos(phi);
    st = sin(theta); cth = cos(theta);
    sps = sin(x(12)); cps = cos(x(12));

    Ndot = u_b*cth*cps + v_b*(sp*st*cps - cp*sps) + w_b*(cp*st*cps + sp*sps);
    Edot = u_b*cth*sps + v_b*(sp*st*sps + cp*cps) + w_b*(cp*st*sps - sp*cps);
    Ddot = -u_b*st     + v_b*sp*cth                + w_b*cp*cth;

    xdot = [udot;vdot;wdot; pdot;qdot;rdot; Ndot;Edot;Ddot; phidot;thetadot;psidot];

function [ab, od] = compute_accels(x, da, de, dr, thr, ac)
    g = 9.81;
    u_b=x(1); v_b=x(2); w_b=x(3); p=x(4); q=x(5); r=x(6);
    phi=x(10); theta=x(11);
    alt = max(0, -x(9));

    V = max(sqrt(u_b^2+v_b^2+w_b^2), 5);
    alpha = atan2(w_b, u_b);
    beta = asin(max(min(v_b/V,1),-1));

    [F_aero, M_aero, ~] = ucav_aerodynamics(alpha, beta, V, alt, ...
        p, q, r, da, de, dr, ac);

    T = ac.engine.idle_thrust + ...
        (ac.engine.thrust_max - ac.engine.idle_thrust) * max(min(thr,1),0);

    % Body accelerations (specific force, as measured by accelerometer)
    ab = [(F_aero(1)+T)/ac.mass; F_aero(2)/ac.mass; F_aero(3)/ac.mass];

    % Angular accelerations
    Gam = ac.Gamma;
    pd = (ac.Izz*M_aero(1)+ac.Ixz*M_aero(3) ...
          -(ac.Ixz*(ac.Ixx-ac.Iyy+ac.Izz))*p*q ...
          +(ac.Ixz^2+ac.Izz*(ac.Izz-ac.Iyy))*q*r)/Gam;
    qd = (M_aero(2)+(ac.Ixx-ac.Izz)*p*r-ac.Ixz*(p^2-r^2))/ac.Iyy;
    rd = (ac.Ixx*M_aero(3)+ac.Ixz*M_aero(1) ...
          +(ac.Ixz*(ac.Iyy-ac.Izz-ac.Ixx))*q*r ...
          +(ac.Ixx*(ac.Ixx-ac.Iyy)+ac.Ixz^2)*p*q)/Gam;
    od = [pd; qd; rd];

function [T, a, P, rho] = isa_atm_plant(alt)
    T0=288.15; P0=101325; L=0.0065; R=287.05; g0=9.81;
    alt=max(alt,0);
    T=max(T0-L*alt, 216.65);
    P=P0*(T/T0)^(g0/(R*L));
    rho=P/(R*T);
    a=sqrt(1.4*R*T);
