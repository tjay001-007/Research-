function fighter_plant_sfunc(block)
%FIGHTER_PLANT_SFUNC  Level-2 MATLAB S-Function: 6-DOF fighter aircraft
%   dynamics for Simulink integration.
%
%   Full nonlinear 6-DOF rigid body with:
%     - Full inertia tensor (Ixz product of inertia)
%     - Tabular aerodynamics via fighter_aero_model.m
%     - ISA atmospheric model (density varies with altitude)
%     - Engine thrust model (dry + afterburner)
%
%   Inputs (5 ports):
%     Port 1: de_L      Left elevon deflection (rad)         [1]
%     Port 2: de_R      Right elevon deflection (rad)        [1]
%     Port 3: dr        Rudder deflection (rad)              [1]
%     Port 4: dc        Canard deflection (rad)              [1]
%     Port 5: throttle  Throttle command (0-1.5)             [1]
%
%   Outputs (7 ports):
%     Port 1: position  [x; y; z] NED (m)                   [3]
%     Port 2: velocity  [u; v; w] body (m/s)                [3]
%     Port 3: omega     [p; q; r] body (rad/s)              [3]
%     Port 4: euler     [phi; theta; psi] (rad)             [3]
%     Port 5: accel     [ax; ay; az] body (m/s^2)           [3]
%     Port 6: omega_dot [pdot; qdot; rdot] (rad/s^2)        [3]
%     Port 7: airdata   [alpha; beta; Mach; V; qbar; alt]   [6]
%
%   State vector (12 continuous states):
%     [u v w p q r x y z phi theta psi]'

setup(block);

function setup(block)
    % --- 5 scalar inputs ---
    block.NumInputPorts = 5;
    for i = 1:5
        block.InputPort(i).Dimensions        = 1;
        block.InputPort(i).DatatypeID        = 0;
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

    block.SampleTimes = [0, 0];  % Continuous
    block.NumContStates = 12;

    block.RegBlockMethod('Start',                @Start);
    block.RegBlockMethod('InitializeConditions', @InitCond);
    block.RegBlockMethod('Outputs',              @Outputs);
    block.RegBlockMethod('Derivatives',          @Derivatives);

function Start(block)
    if evalin('base', 'exist(''aircraft'', ''var'')')
        ac = evalin('base', 'aircraft');
        setappdata(0, 'FighterPlantAC', ac);
    else
        error('[Plant] ''aircraft'' struct not in workspace. Run setup_fighter.m');
    end

function InitCond(block)
    if evalin('base', 'exist(''initial'', ''var'')')
        ini = evalin('base', 'initial');
        block.ContStates.Data = ini.state;
    else
        block.ContStates.Data = [200;0;7;0;0;0;0;0;-3000;0;0.035;0];
    end

function Outputs(block)
    ac = getappdata(0, 'FighterPlantAC');
    x  = block.ContStates.Data;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11); psi = x(12);
    alt = max(0, -x(9));

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);
    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    [~, a_snd, ~, rho] = isa_atm(alt);
    Mach = V / a_snd;
    qbar = 0.5 * rho * V^2;

    % Read inputs to compute accelerations for output
    de_L     = block.InputPort(1).Data;
    de_R     = block.InputPort(2).Data;
    dr       = block.InputPort(3).Data;
    dc       = block.InputPort(4).Data;
    throttle = block.InputPort(5).Data;

    % Compute forces and moments for acceleration outputs
    [accel_body, omega_dot] = compute_accels( ...
        x, de_L, de_R, dr, dc, throttle, ac);

    % === Write outputs ===
    block.OutputPort(1).Data = x(7:9);                          % position
    block.OutputPort(2).Data = x(1:3);                          % velocity
    block.OutputPort(3).Data = x(4:6);                          % omega
    block.OutputPort(4).Data = x(10:12);                        % euler
    block.OutputPort(5).Data = accel_body;                      % body accels
    block.OutputPort(6).Data = omega_dot;                       % angular accels
    block.OutputPort(7).Data = [alpha; beta; Mach; V; qbar; alt]; % airdata

function Derivatives(block)
    ac = getappdata(0, 'FighterPlantAC');
    x  = block.ContStates.Data;

    de_L     = block.InputPort(1).Data;
    de_R     = block.InputPort(2).Data;
    dr       = block.InputPort(3).Data;
    dc       = block.InputPort(4).Data;
    throttle = block.InputPort(5).Data;

    xdot = fighter_eom(x, de_L, de_R, dr, dc, throttle, ac);
    block.Derivatives.Data = xdot;

% =====================================================================
%  EQUATIONS OF MOTION
% =====================================================================

function xdot = fighter_eom(x, de_L, de_R, dr, dc, throttle, ac)
    g = 9.81;

    u_b = x(1); v_b = x(2); w_b = x(3);
    p = x(4); q = x(5); r = x(6);
    phi = x(10); theta = x(11); psi = x(12);
    alt = max(0, -x(9));

    V = sqrt(u_b^2 + v_b^2 + w_b^2);
    V = max(V, 5.0);
    alpha = atan2(w_b, u_b);
    beta  = asin(max(min(v_b/V, 1), -1));

    [~, a_snd, ~, rho] = isa_atm(alt);
    Mach = V / a_snd;
    qbar = 0.5 * rho * V^2;

    % Aerodynamic coefficients
    [CL, CD, CY, Cl_c, Cm_c, Cn_c] = fighter_aero_model( ...
        alpha, beta, p, q, r, V, Mach, alt, ...
        de_L, de_R, dr, dc, ac);

    % Forces
    L_a = qbar * ac.S * CL;
    D_a = qbar * ac.S * CD;
    Y_a = qbar * ac.S * CY;

    Fx_a = -D_a*cos(alpha) + L_a*sin(alpha);
    Fy_a = Y_a;
    Fz_a = -D_a*sin(alpha) - L_a*cos(alpha);

    L_m = qbar * ac.S * ac.b     * Cl_c;
    M_m = qbar * ac.S * ac.c_bar * Cm_c;
    N_m = qbar * ac.S * ac.b     * Cn_c;

    % Thrust
    if throttle <= 1.0
        T = ac.engine.idle_thrust + ...
            (ac.engine.thrust_max_dry - ac.engine.idle_thrust) * throttle;
    else
        T = ac.engine.thrust_max_dry + ...
            (ac.engine.thrust_max_ab - ac.engine.thrust_max_dry) * (throttle - 1);
    end

    % Gravity
    Fx_g = -ac.mass * g * sin(theta);
    Fy_g =  ac.mass * g * cos(theta) * sin(phi);
    Fz_g =  ac.mass * g * cos(theta) * cos(phi);

    Fx = Fx_a + T + Fx_g;
    Fy = Fy_a + Fy_g;
    Fz = Fz_a + Fz_g;

    % Translational
    udot = Fx/ac.mass + r*v_b - q*w_b;
    vdot = Fy/ac.mass + p*w_b - r*u_b;
    wdot = Fz/ac.mass + q*u_b - p*v_b;

    % Rotational (full inertia with Ixz)
    Gam = ac.Ixx * ac.Izz - ac.Ixz^2;

    pdot = (ac.Izz * L_m + ac.Ixz * N_m - ...
            (ac.Ixz*(ac.Ixx - ac.Iyy + ac.Izz))*p*q + ...
            (ac.Ixz^2 + ac.Izz*(ac.Izz - ac.Iyy))*q*r) / Gam;
    qdot = (M_m + (ac.Ixx - ac.Izz)*p*r - ac.Ixz*(p^2 - r^2)) / ac.Iyy;
    rdot = (ac.Ixx * N_m + ac.Ixz * L_m + ...
            (ac.Ixz*(ac.Iyy - ac.Izz - ac.Ixx))*q*r + ...
            (ac.Ixx*(ac.Ixx - ac.Iyy) + ac.Ixz^2)*p*q) / Gam;

    % Kinematics
    ct = cos(theta);
    if abs(ct) < 0.01, ct = sign(ct)*0.01; end

    phidot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    thetadot = q*cos(phi) - r*sin(phi);
    psidot   = (q*sin(phi) + r*cos(phi)) / ct;

    % Navigation
    cp = cos(phi); sp = sin(phi);
    cth = cos(theta); sth = sin(theta);
    cps = cos(psi); sps = sin(psi);

    xd = u_b*cth*cps + v_b*(sp*sth*cps - cp*sps) + w_b*(cp*sth*cps + sp*sps);
    yd = u_b*cth*sps + v_b*(sp*sth*sps + cp*cps) + w_b*(cp*sth*sps - sp*cps);
    zd = -u_b*sth + v_b*sp*cth + w_b*cp*cth;

    xdot = [udot;vdot;wdot; pdot;qdot;rdot; xd;yd;zd; phidot;thetadot;psidot];

function [ab, od] = compute_accels(x, de_L, de_R, dr, dc, thr, ac)
    % Compute body-axis accelerations and angular accelerations
    % (used by INDI controller for angular acceleration feedback)
    g = 9.81;
    u_b=x(1); v_b=x(2); w_b=x(3); p=x(4); q=x(5); r=x(6);
    phi=x(10); theta=x(11);
    alt = max(0, -x(9));

    V = max(sqrt(u_b^2+v_b^2+w_b^2), 5);
    alpha = atan2(w_b, u_b);
    beta = asin(max(min(v_b/V,1),-1));
    [~,a_s,~,rho] = isa_atm(alt);
    qbar = 0.5*rho*V^2; Mach = V/a_s;

    [CL,CD,CY,Cl_c,Cm_c,Cn_c] = fighter_aero_model( ...
        alpha,beta,p,q,r,V,Mach,alt, de_L,de_R,dr,dc, ac);

    L_a=qbar*ac.S*CL; D_a=qbar*ac.S*CD; Y_a=qbar*ac.S*CY;
    Fxa=-D_a*cos(alpha)+L_a*sin(alpha);
    Fya=Y_a;
    Fza=-D_a*sin(alpha)-L_a*cos(alpha);

    if thr<=1, T=ac.engine.idle_thrust+(ac.engine.thrust_max_dry-ac.engine.idle_thrust)*thr;
    else, T=ac.engine.thrust_max_dry+(ac.engine.thrust_max_ab-ac.engine.thrust_max_dry)*(thr-1); end

    Fxg=-ac.mass*g*sin(theta);
    Fyg= ac.mass*g*cos(theta)*sin(phi);
    Fzg= ac.mass*g*cos(theta)*cos(phi);

    % Body accelerations (including gravity, as measured by accelerometer)
    ax = (Fxa + T)/ac.mass;   % Specific force x (excludes gravity for real accel)
    ay = Fya/ac.mass;
    az = Fza/ac.mass;
    ab = [ax; ay; az];

    % Angular accelerations
    Lm=qbar*ac.S*ac.b*Cl_c; Mm=qbar*ac.S*ac.c_bar*Cm_c; Nm=qbar*ac.S*ac.b*Cn_c;
    Gam = ac.Ixx*ac.Izz - ac.Ixz^2;
    pd = (ac.Izz*Lm+ac.Ixz*Nm-(ac.Ixz*(ac.Ixx-ac.Iyy+ac.Izz))*p*q+...
          (ac.Ixz^2+ac.Izz*(ac.Izz-ac.Iyy))*q*r)/Gam;
    qd = (Mm+(ac.Ixx-ac.Izz)*p*r-ac.Ixz*(p^2-r^2))/ac.Iyy;
    rd = (ac.Ixx*Nm+ac.Ixz*Lm+(ac.Ixz*(ac.Iyy-ac.Izz-ac.Ixx))*q*r+...
          (ac.Ixx*(ac.Ixx-ac.Iyy)+ac.Ixz^2)*p*q)/Gam;
    od = [pd; qd; rd];

function [T, a, P, rho] = isa_atm(alt)
    T0=288.15; P0=101325; L=0.0065; R=287.05; g0=9.81;
    alt=max(alt,0);
    T=max(T0-L*alt, 216.65);
    P=P0*(T/T0)^(g0/(R*L));
    rho=P/(R*T);
    a=sqrt(1.4*R*T);
