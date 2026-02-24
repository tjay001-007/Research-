function [CL, CD, CY, Cl, Cm, Cn] = fighter_aero_model( ...
    alpha, beta, p, q, r, V, Mach, alt, ...
    de_L, de_R, dr, dc, ...
    aircraft)
%FIGHTER_AERO_MODEL  Tabular aerodynamic model for a generic delta-canard
%   light combat aircraft (LCA) class fighter.
%
%   Aerodynamic data is stored as 2D lookup tables f(alpha, Mach), which is
%   the standard representation used in production flight control systems.
%   Real aircraft databases would also include beta, altitude (Reynolds
%   number), and store configuration dependencies.
%
%   This model represents a compound-delta / canard-delta configuration
%   similar in class to Tejas LCA, Gripen, or Rafale:
%     - Compound delta wing (leading edge sweep ~50/62 deg)
%     - Close-coupled canards
%     - Single vertical tail with rudder
%     - Elevons (combined aileron/elevator function on wing trailing edge)
%     - Relaxed static stability (CG aft of aerodynamic center)
%
%   Control surfaces:
%     de_L  - Left elevon  deflection (rad, positive trailing edge down)
%     de_R  - Right elevon deflection (rad, positive trailing edge down)
%     dr    - Rudder deflection (rad, positive trailing edge left)
%     dc    - Canard deflection (rad, positive trailing edge down)
%
%   Symmetric elevator:  de_sym = (de_L + de_R) / 2  (pitch control)
%   Differential elevon: de_dif = (de_L - de_R) / 2  (roll control)
%
%   Outputs: Force and moment coefficients in stability/body axes
%     CL - Lift coefficient (stability axis)
%     CD - Drag coefficient (stability axis)
%     CY - Side force coefficient (body axis)
%     Cl - Rolling moment coefficient (body axis)
%     Cm - Pitching moment coefficient (body axis)
%     Cn - Yawing moment coefficient (body axis)
%
%   Reference:
%     Aero data structure inspired by publicly available F-16 model
%     (Stevens & Lewis, "Aircraft Control and Simulation") and published
%     Tejas LCA data from ADA/NAL/DRDO open literature.

%% Unpack geometry
S = aircraft.S;          % Wing reference area (m^2)
b = aircraft.b;          % Wing span (m)
c = aircraft.c_bar;      % Mean aerodynamic chord (m)

%% Decompose elevon into symmetric (pitch) and differential (roll)
de_sym = 0.5 * (de_L + de_R);    % Symmetric = elevator function
de_dif = 0.5 * (de_L - de_R);    % Differential = aileron function

%% Convert alpha, beta to degrees for table lookup
alpha_deg = rad2deg(alpha);
beta_deg  = rad2deg(beta);

%% Non-dimensional rates
p_hat = p * b / (2 * V);      % Non-dim roll rate
q_hat = q * c / (2 * V);      % Non-dim pitch rate
r_hat = r * b / (2 * V);      % Non-dim yaw rate

%% ====================================================================
%  AERODYNAMIC DATABASE — 2D LOOKUP TABLES  f(alpha, Mach)
%  ====================================================================
%
%  In a production FCS, these tables come from thousands of wind tunnel
%  data points and CFD runs, validated by flight test. The tables below
%  are representative of a delta-canard fighter.
%
%  Table breakpoints:

alpha_bp = aircraft.aero.alpha_bp;    % deg: [-5 0 5 10 15 20 25 30]
mach_bp  = aircraft.aero.mach_bp;     % [0.2 0.4 0.6 0.8 0.95 1.1 1.4]

%% LIFT COEFFICIENT  CL_basic(alpha, Mach)
CL_basic = interp2_aero(alpha_bp, mach_bp, aircraft.aero.CL_table, ...
                         alpha_deg, Mach);

% Elevator (symmetric elevon) lift increment
CL_de = interp1_aero(mach_bp, aircraft.aero.CLde_mach, Mach);

% Canard lift increment
CL_dc = interp1_aero(mach_bp, aircraft.aero.CLdc_mach, Mach);

CL = CL_basic + CL_de * de_sym + CL_dc * dc;

%% DRAG COEFFICIENT  CD(alpha, Mach)
%  Includes zero-lift drag, induced drag, and transonic wave drag
CD_basic = interp2_aero(alpha_bp, mach_bp, aircraft.aero.CD_table, ...
                         alpha_deg, Mach);

% Trim drag from control deflections (simplified)
CD_de = 0.002 * de_sym^2;    % Elevon trim drag
CD_dc = 0.001 * dc^2;        % Canard trim drag

CD = CD_basic + CD_de + CD_dc;

%% SIDE FORCE COEFFICIENT  CY(beta, Mach)
CY_beta = interp1_aero(mach_bp, aircraft.aero.CYbeta_mach, Mach);
CY_dr   = interp1_aero(mach_bp, aircraft.aero.CYdr_mach, Mach);

CY = CY_beta * beta + CY_dr * dr;

%% ROLLING MOMENT COEFFICIENT  Cl
Cl_beta = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Clbeta_table, ...
                         alpha_deg, Mach);
Cl_p    = interp1_aero(mach_bp, aircraft.aero.Clp_mach, Mach);
Cl_r    = interp1_aero(mach_bp, aircraft.aero.Clr_mach, Mach);
Cl_da   = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Clda_table, ...
                         alpha_deg, Mach);
Cl_dr   = interp1_aero(mach_bp, aircraft.aero.Cldr_mach, Mach);

Cl = Cl_beta * beta + Cl_p * p_hat + Cl_r * r_hat + ...
     Cl_da * de_dif + Cl_dr * dr;

%% PITCHING MOMENT COEFFICIENT  Cm
%  THIS IS WHERE THE INSTABILITY LIVES.
%  For a relaxed-stability fighter:
%    - Cm0 > 0 (positive nose-up moment at zero alpha)
%    - dCm/dalpha > 0 at low alpha (UNSTABLE slope)
%    - The slope may become less positive or negative at high alpha
%      (delta wing nonlinear aerodynamics)
Cm_basic = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Cm_table, ...
                          alpha_deg, Mach);
Cm_q     = interp1_aero(mach_bp, aircraft.aero.Cmq_mach, Mach);
Cm_de    = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Cmde_table, ...
                          alpha_deg, Mach);
Cm_dc    = interp1_aero(mach_bp, aircraft.aero.Cmdc_mach, Mach);

% Pitch damping (Cmq) uses non-dimensional pitch rate
% Cm_alphadot is often lumped with Cmq in simplified models
Cm = Cm_basic + Cm_q * q_hat + Cm_de * de_sym + Cm_dc * dc;

%% YAWING MOMENT COEFFICIENT  Cn
Cn_beta = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Cnbeta_table, ...
                         alpha_deg, Mach);
Cn_r    = interp1_aero(mach_bp, aircraft.aero.Cnr_mach, Mach);
Cn_p    = interp1_aero(mach_bp, aircraft.aero.Cnp_mach, Mach);
Cn_da   = interp2_aero(alpha_bp, mach_bp, aircraft.aero.Cnda_table, ...
                         alpha_deg, Mach);
Cn_dr   = interp1_aero(mach_bp, aircraft.aero.Cndr_mach, Mach);

Cn = Cn_beta * beta + Cn_r * r_hat + Cn_p * p_hat + ...
     Cn_da * de_dif + Cn_dr * dr;

end

%% ====================================================================
%  INTERPOLATION HELPERS
%  ====================================================================
%  These replicate the behavior of Simulink lookup table blocks.
%  In production FCS, these are implemented as fixed-point C code on the
%  flight control computer with bounds checking and extrapolation hold.

function val = interp2_aero(x_bp, y_bp, table, x_query, y_query)
    % 2D interpolation with extrapolation clamping (hold at boundaries)
    x_query = max(min(x_query, x_bp(end)), x_bp(1));
    y_query = max(min(y_query, y_bp(end)), y_bp(1));
    val = interp2(y_bp, x_bp, table, y_query, x_query, 'linear');
end

function val = interp1_aero(bp, table, query)
    % 1D interpolation with extrapolation clamping
    query = max(min(query, bp(end)), bp(1));
    val = interp1(bp, table, query, 'linear');
end
