function [F_body, M_body] = aero_forces_moments(x, controls, vehicle, aero_db, atm)
%AERO_FORCES_MOMENTS  Computes aerodynamic forces and moments in body frame.
%
%   Uses bilinear interpolation into the lookup tables in aero_db.
%   All forces returned in body frame (x-forward, y-right, z-down).
%
%   Force model:
%     Lift:      L_aero = qbar * S * CL(Mach, alpha)
%     Drag:      D_aero = qbar * S * CD(Mach, alpha)
%     Side:      Y_aero = qbar * S * CYbeta * beta
%
%   Moment model:
%     Roll:  L_m = qbar*S*b * [Clbeta*beta + Clp*p_hat + Clr*r_hat + Clda*da + Cldr*dr]
%     Pitch: M_m = qbar*S*c * [Cm(Mach,alpha) + Cmq*q_hat + Cmde*de]
%     Yaw:   N_m = qbar*S*b * [Cnbeta*beta + Cnr*r_hat + Cnp*p_hat + Cndr*dr + Cnda*da]
%
%   All control surface deflections are normalised [-1, 1] in controls struct
%   and scaled by vehicle.*_to_rad inside this function.
%
%   Inputs:
%     x        — State vector [14x1]
%     controls — Controls struct (delta_e, delta_a, delta_r normalised [-1,1])
%     vehicle  — Vehicle config
%     aero_db  — Aerodynamic database (from aero_database.m)
%     atm      — Atmosphere struct: .rho, .a (speed of sound), .Pa
%
%   Outputs:
%     F_body   — Aerodynamic force  in body frame [Fx; Fy; Fz] (N)
%     M_body   — Aerodynamic moment in body frame [L; M_pitch; N] (Nm)

%% ========================================================================
%  EXTRACT STATE & ATMOSPHERE
%  ========================================================================

u = x(4);  v = x(5);  w = x(6);
p = x(11); qr = x(12); r = x(13);

rho = atm.rho;
a   = atm.a;

%% ========================================================================
%  AIRDATA
%  ========================================================================

V = sqrt(u^2 + v^2 + w^2);
V = max(V, 1.0);

Mach  = V / a;
alpha = atan2(w, u);                           % rad
beta  = asin(min(max(v/V, -1), 1));            % rad
qbar  = 0.5 * rho * V^2;                      % Pa

alpha_deg = rad2deg(alpha);
beta_deg  = rad2deg(beta);

%% ========================================================================
%  NON-DIMENSIONAL RATES
%  ========================================================================

p_hat = p  * vehicle.b / (2 * V);   % Roll rate  non-dimensional
q_hat = qr * vehicle.c / (2 * V);   % Pitch rate non-dimensional
r_hat = r  * vehicle.b / (2 * V);   % Yaw rate   non-dimensional

%% ========================================================================
%  CONTROL DEFLECTIONS (normalised → rad)
%  ========================================================================

de = controls.delta_e * vehicle.de_to_rad;   % Elevator (rad)
da = controls.delta_a * vehicle.da_to_rad;   % Aileron  (rad)
dr = controls.delta_r * vehicle.dr_to_rad;   % Rudder   (rad)

%% ========================================================================
%  COEFFICIENT INTERPOLATION (Mach × alpha tables)
%  ========================================================================

% Clamp to table limits for extrapolation safety
Mach_c   = min(max(Mach,      aero_db.Mach_vec(1)),  aero_db.Mach_vec(end));
alpha_c  = min(max(alpha_deg, aero_db.alpha_vec(1)), aero_db.alpha_vec(end));

% Force coefficients
CL     = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.CL',     Mach_c, alpha_c);
CD     = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.CD',     Mach_c, alpha_c);
CYbeta = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.CYbeta', Mach_c, alpha_c);

% Moment base coefficients
Clbeta = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Clbeta', Mach_c, alpha_c);
Cm_base= interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cm',     Mach_c, alpha_c);
Cnbeta = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cnbeta', Mach_c, alpha_c);

% Control effectiveness
Cmde   = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cmde',   Mach_c, alpha_c);
Clda   = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Clda',   Mach_c, alpha_c);
Cndr   = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cndr',   Mach_c, alpha_c);
Cldr   = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cldr',   Mach_c, alpha_c);
Cnda   = interp2_db(aero_db.Mach_vec, aero_db.alpha_vec, aero_db.Cnda',   Mach_c, alpha_c);

% Dynamic derivatives (Mach-only interpolation)
Clp = interp1(aero_db.Mach_vec, aero_db.Clp, Mach_c, 'linear', 'extrap');
Cmq = interp1(aero_db.Mach_vec, aero_db.Cmq, Mach_c, 'linear', 'extrap');
Cnr = interp1(aero_db.Mach_vec, aero_db.Cnr, Mach_c, 'linear', 'extrap');
Clr = interp1(aero_db.Mach_vec, aero_db.Clr, Mach_c, 'linear', 'extrap');
Cnp = interp1(aero_db.Mach_vec, aero_db.Cnp, Mach_c, 'linear', 'extrap');

%% ========================================================================
%  AERODYNAMIC FORCE COEFFICIENTS (total)
%  ========================================================================

CY = CYbeta * beta;   % Side force (beta in rad)

%% ========================================================================
%  FORCE TRANSFORMATION: wind frame → body frame
%  Lift is perpendicular to velocity, Drag is opposite to velocity.
%
%  In body frame (assuming small sideslip, wind frame approximation):
%    Fx_aero = -D*cos(alpha) + L*sin(alpha)   (thrust axis, net result of drag/lift)
%    Fy_aero =  Y                              (side force)
%    Fz_aero = -D*sin(alpha) - L*cos(alpha)   (normal force, negative = upward)
%  ========================================================================

S = vehicle.S;

L_aero = qbar * S * CL;
D_aero = qbar * S * CD;
Y_aero = qbar * S * CY;

Fx_aero = -D_aero * cos(alpha) + L_aero * sin(alpha);
Fy_aero =  Y_aero;
Fz_aero = -D_aero * sin(alpha) - L_aero * cos(alpha);

F_body = [Fx_aero; Fy_aero; Fz_aero];

%% ========================================================================
%  AERODYNAMIC MOMENT COEFFICIENTS
%  ========================================================================

b = vehicle.b;
c = vehicle.c;

% Roll moment
Cl_total = Clbeta*beta + Clp*p_hat + Clr*r_hat + Clda*da + Cldr*dr;

% Pitch moment
Cm_total = Cm_base + Cmq*q_hat + Cmde*de;

% Yaw moment
Cn_total = Cnbeta*beta + Cnr*r_hat + Cnp*p_hat + Cndr*dr + Cnda*da;

%% ========================================================================
%  AERODYNAMIC MOMENTS (body frame)
%  ========================================================================

L_m = qbar * S * b * Cl_total;   % Roll  moment (Nm)
M_m = qbar * S * c * Cm_total;   % Pitch moment (Nm)
N_m = qbar * S * b * Cn_total;   % Yaw   moment (Nm)

M_body = [L_m; M_m; N_m];

end

%% ========================================================================
%  LOCAL INTERPOLATION HELPER (table in column-major: rows=alpha, cols=Mach)
%  ========================================================================

function val = interp2_db(Mach_vec, alpha_vec, table, Mach_q, alpha_q)
    % table is [nA x nM] after transposing in caller
    val = interp2(Mach_vec, alpha_vec, table, Mach_q, alpha_q, 'linear');
    if isnan(val)
        % Fallback: clamp and retry
        Mc = min(max(Mach_q,  Mach_vec(1)),  Mach_vec(end));
        Ac = min(max(alpha_q, alpha_vec(1)), alpha_vec(end));
        val = interp2(Mach_vec, alpha_vec, table, Mc, Ac, 'linear');
    end
    if isnan(val)
        val = 0;
    end
end
