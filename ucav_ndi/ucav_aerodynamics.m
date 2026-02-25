function [F_body, M_body, coeffs] = ucav_aerodynamics(alpha, beta, V, alt, ...
    p, q, r, da, de, dr, ac)
%UCAV_AERODYNAMICS  Compute aerodynamic forces and moments for a conventional
%   fixed-wing UCAV (NO canard). Uses stability derivatives with stall model.
%
%  Aircraft configuration:
%    - Conventional: wing + horizontal tail + vertical tail
%    - Three control surfaces: aileron (da), elevator (de), rudder (dr)
%    - Statically unstable in pitch (Cm_alpha > 0)
%
%  Inputs:
%    alpha     - Angle of attack (rad)
%    beta      - Sideslip angle (rad)
%    V         - True airspeed (m/s)
%    alt       - Altitude above MSL (m)
%    p, q, r   - Body angular rates (rad/s)
%    da        - Aileron deflection (rad), positive = right roll
%    de        - Elevator deflection (rad), positive = trailing edge down
%    dr        - Rudder deflection (rad), positive = trailing edge left
%    ac        - Aircraft parameter struct (from setup_ucav)
%
%  Outputs:
%    F_body    - [Fx; Fy; Fz] aerodynamic forces in body frame (N)
%    M_body    - [L; M; N] aerodynamic moments about CG in body frame (N*m)
%    coeffs    - Struct with all computed aero coefficients
%
%  Sign conventions:
%    - Body x forward, y right, z down
%    - Fx positive forward, Fy positive right, Fz positive down
%    - L positive right roll, M positive nose up, N positive nose right

    ar = ac.aero;
    g = 9.81;

    % --- Atmosphere ---
    [~, a_snd, ~, rho] = isa_atm(alt);
    qbar = 0.5 * rho * V^2;
    Mach = V / a_snd;

    % Protect against zero airspeed
    V_safe = max(V, 5.0);

    % --- Nondimensional rates ---
    %   These normalize angular rates by the aerodynamic time scales
    %   (wingspan for roll/yaw, chord for pitch).
    p_hat = p * ac.b     / (2 * V_safe);    % Nondim roll rate
    q_hat = q * ac.c_bar / (2 * V_safe);    % Nondim pitch rate
    r_hat = r * ac.b     / (2 * V_safe);    % Nondim yaw rate

    % ================================================================
    %  LIFT COEFFICIENT
    % ================================================================
    %  Linear region + smooth stall model above alpha_stall.
    %  The stall model uses a cosine taper to limit CL growth
    %  and smoothly reduce CL above alpha_max.

    CL_linear = ar.CL_0 + ar.CL_alpha * alpha + ar.CL_de * de + ar.CL_q * q_hat;

    % Stall model
    if abs(alpha) < ar.alpha_stall
        % Below stall — fully linear
        CL = CL_linear;
    elseif abs(alpha) < ar.alpha_max
        % Transition region — cosine blend
        frac = (abs(alpha) - ar.alpha_stall) / (ar.alpha_max - ar.alpha_stall);
        blend = 0.5 * (1 + cos(pi * frac));    % 1 at stall_onset → 0 at alpha_max
        CL_stall_val = ar.CL_max * sign(alpha);
        CL = blend * CL_linear + (1 - blend) * CL_stall_val;
    else
        % Post-stall — flat plate approximation
        CL = ar.CL_max * sign(alpha) * cos(alpha - sign(alpha)*ar.alpha_max);
    end

    % ================================================================
    %  DRAG COEFFICIENT
    % ================================================================
    %  Parabolic drag polar: CD = CD_0 + K * CL^2
    %  Additional drag from control surface deflections.

    CD = ar.CD_0 + ar.K_drag * CL^2;

    % Control surface drag (small but nonzero)
    CD = CD + 0.01 * (da^2 + de^2 + dr^2);

    % ================================================================
    %  SIDE FORCE COEFFICIENT
    % ================================================================

    CY = ar.CY_beta * beta + ar.CY_dr * dr + ar.CY_p * p_hat + ar.CY_r * r_hat;

    % ================================================================
    %  ROLLING MOMENT COEFFICIENT
    % ================================================================

    Cl = ar.Cl_beta * beta ...
       + ar.Cl_da * da ...
       + ar.Cl_dr * dr ...
       + ar.Cl_p * p_hat ...
       + ar.Cl_r * r_hat;

    % ================================================================
    %  PITCHING MOMENT COEFFICIENT
    % ================================================================
    %  NOTE: Cm_alpha > 0 means moment increases with alpha → UNSTABLE.
    %  The INDI controller must actively cancel this destabilizing tendency.

    Cm = ar.Cm_0 ...
       + ar.Cm_alpha * alpha ...
       + ar.Cm_de * de ...
       + ar.Cm_q * q_hat;

    % ================================================================
    %  YAWING MOMENT COEFFICIENT
    % ================================================================

    Cn = ar.Cn_beta * beta ...
       + ar.Cn_da * da ...
       + ar.Cn_dr * dr ...
       + ar.Cn_p * p_hat ...
       + ar.Cn_r * r_hat;

    % ================================================================
    %  CONVERT TO BODY-AXIS FORCES
    % ================================================================
    %  Aerodynamic forces are naturally in wind axes (lift/drag/sideforce).
    %  Convert to body axes using angle of attack.

    % Lift and drag in wind axes
    L_a = qbar * ac.S * CL;     % Lift (perpendicular to V, in symmetry plane)
    D_a = qbar * ac.S * CD;     % Drag (along -V)
    Y_a = qbar * ac.S * CY;     % Side force (perpendicular to V, lateral)

    % Body-axis forces (rotate from wind to body by alpha)
    Fx = -D_a * cos(alpha) + L_a * sin(alpha);   % Forward
    Fy =  Y_a;                                     % Right
    Fz = -D_a * sin(alpha) - L_a * cos(alpha);    % Down

    F_body = [Fx; Fy; Fz];

    % ================================================================
    %  BODY-AXIS MOMENTS
    % ================================================================

    L_m = qbar * ac.S * ac.b     * Cl;     % Rolling moment (N*m)
    M_m = qbar * ac.S * ac.c_bar * Cm;     % Pitching moment (N*m)
    N_m = qbar * ac.S * ac.b     * Cn;     % Yawing moment (N*m)

    M_body = [L_m; M_m; N_m];

    % ================================================================
    %  OUTPUT COEFFICIENTS (for logging/debugging)
    % ================================================================

    coeffs.CL   = CL;
    coeffs.CD   = CD;
    coeffs.CY   = CY;
    coeffs.Cl   = Cl;
    coeffs.Cm   = Cm;
    coeffs.Cn   = Cn;
    coeffs.qbar = qbar;
    coeffs.Mach = Mach;
    coeffs.rho  = rho;

end

% =====================================================================
%  ISA ATMOSPHERE
% =====================================================================

function [T, a, P, rho] = isa_atm(alt)
%ISA_ATM  Standard atmosphere model (troposphere only).
    T0 = 288.15; P0 = 101325; L = 0.0065; R = 287.05; g0 = 9.81;
    alt = max(alt, 0);
    T = max(T0 - L*alt, 216.65);
    P = P0 * (T/T0)^(g0/(R*L));
    rho = P / (R*T);
    a = sqrt(1.4*R*T);
end
