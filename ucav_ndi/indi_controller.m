function [da_cmd, de_cmd, dr_cmd, ctrl_out, debug] = indi_controller( ...
    phi_cmd, theta_cmd, ...
    u, v, w, p, q, r, phi, theta, psi, ...
    pdot_meas, qdot_meas, rdot_meas, ...
    da_prev, de_prev, dr_prev, ...
    ctrl_in, ac, fcs, dt)
%INDI_CONTROLLER  Incremental Nonlinear Dynamic Inversion flight controller
%   for a pitch-unstable UCAV with conventional tail (aileron, elevator, rudder).
%
%  INDI vs classical NDI:
%    Classical NDI:  delta = B_inv * (omega_dot_des - f(x))
%      → Requires full aerodynamic model f(x) to cancel nonlinear moments.
%      → Sensitive to model errors in f(x).
%
%    INDI:  delta = delta_prev + B_inv * (omega_dot_des - omega_dot_meas)
%      → Uses MEASURED angular acceleration instead of modeled moments.
%      → Only needs B matrix (control effectiveness) to be accurate.
%      → Inherently robust: unmodeled effects are automatically cancelled.
%
%  Architecture (two-loop cascade):
%    ┌─────────────────────────────────────────────────────────────────┐
%    │ OUTER LOOP: Attitude → Rate commands (kinematic inversion)     │
%    │   p_cmd = K_phi * (phi_cmd - phi)                              │
%    │   q_cmd = K_theta * (theta_cmd - theta)                        │
%    │   r_cmd = -K_beta * beta  (sideslip regulation)                │
%    ├─────────────────────────────────────────────────────────────────┤
%    │ INNER LOOP: INDI (incremental dynamic inversion)               │
%    │   omega_dot_des = K * (omega_cmd - omega) + Ki * integral      │
%    │   delta_u = B_inv * (omega_dot_des - omega_dot_meas)           │
%    │   delta_new = delta_prev + delta_u                             │
%    └─────────────────────────────────────────────────────────────────┘
%
%  Inputs:
%    phi_cmd, theta_cmd   - Attitude commands from guidance (rad)
%    u, v, w              - Body-axis velocities (m/s)
%    p, q, r              - Body angular rates (rad/s)
%    phi, theta, psi      - Euler angles (rad)
%    pdot_meas, qdot_meas, rdot_meas - Measured angular accelerations (rad/s^2)
%    da_prev, de_prev, dr_prev       - Previous actuator positions (rad)
%    ctrl_in              - Controller state struct:
%                             .int_p, .int_q, .int_r  (integrator states)
%    ac                   - Aircraft parameters (from setup_ucav)
%    fcs                  - FCS gains (from setup_ucav)
%    dt                   - Time step (s)
%
%  Outputs:
%    da_cmd, de_cmd, dr_cmd  - Surface deflection commands (rad)
%    ctrl_out                - Updated controller state struct
%    debug                   - Debug signals struct

    g = 9.81;

    % ================================================================
    %  SECTION A: AIR DATA COMPUTATION
    % ================================================================
    %  Compute angle of attack, sideslip, airspeed, and dynamic pressure
    %  from body-axis velocities.

    V = sqrt(u^2 + v^2 + w^2);
    V = max(V, 10.0);                           % Protect against zero airspeed

    alpha = atan2(w, u);                          % Angle of attack (rad)
    beta  = asin(max(min(v/V, 1), -1));           % Sideslip angle (rad)

    alt = max(0, -0);  % Placeholder — altitude computed from state in simulation
    % For the controller, we use V to estimate qbar. The actual altitude
    % comes from the plant state, but qbar is what matters for B matrix.
    [~, ~, ~, rho] = isa_atm_ctrl(1000);         % Approximate — use design altitude
    qbar = 0.5 * rho * V^2;

    % ================================================================
    %  SECTION B: ENVELOPE PROTECTION
    % ================================================================
    %  Limit attitude commands to prevent exceeding structural/aero limits.
    %  Uses hard clipping (simple and reliable for UCAV).

    % Limit bank angle command
    phi_cmd = max(min(phi_cmd, fcs.phi_max), -fcs.phi_max);

    % Limit pitch angle based on alpha protection
    % theta = alpha + gamma, so limiting theta approximately limits alpha
    theta_min = fcs.alpha_min;
    theta_max = fcs.alpha_max + deg2rad(5);   % Allow some gamma margin
    theta_cmd = max(min(theta_cmd, theta_max), theta_min);

    % ================================================================
    %  SECTION C: OUTER LOOP — Attitude to rate commands
    % ================================================================
    %  Kinematic inversion: computes required body rates to achieve
    %  desired attitude changes.
    %
    %  For small bank angles, this simplifies to:
    %    p_cmd = K_phi * (phi_cmd - phi)
    %    q_cmd = K_theta * (theta_cmd - theta)
    %    r_cmd = coordinated turn rate - K_beta * beta
    %
    %  For large bank angles, we use the full kinematic relations
    %  to account for Euler angle coupling.

    % Roll rate command
    phi_err = phi_cmd - phi;
    % Wrap phi_err to [-pi, pi]
    phi_err = atan2(sin(phi_err), cos(phi_err));
    p_cmd = fcs.K_phi * phi_err;

    % Pitch rate command
    theta_err = theta_cmd - theta;
    q_cmd = fcs.K_theta * theta_err;

    % Yaw rate command: coordinated turn + sideslip regulation
    %   In a coordinated turn: r_coordinated = g * sin(phi) / V
    %   Plus sideslip damping
    r_coord = g * sin(phi) * cos(theta) / V;
    r_cmd = r_coord - fcs.K_beta * beta;

    % Rate command limiting
    p_cmd = max(min(p_cmd, fcs.p_max), -fcs.p_max);
    q_cmd = max(min(q_cmd, fcs.q_max), -fcs.q_max);
    r_cmd = max(min(r_cmd, fcs.r_max), -fcs.r_max);

    % ================================================================
    %  SECTION D: INNER LOOP — INDI
    % ================================================================
    %  The core INDI algorithm:
    %    1. Compute desired angular acceleration from rate error
    %    2. Compute control increment using B-matrix inverse
    %    3. Add increment to previous surface command
    %
    %  Key insight: omega_dot_meas already contains ALL aerodynamic effects
    %  (including the unstable Cm_alpha). The INDI only needs to compute
    %  the DIFFERENCE between desired and measured angular acceleration,
    %  and map that to a surface deflection INCREMENT.

    % Rate errors
    ep = p_cmd - p;
    eq = q_cmd - q;
    er = r_cmd - r;

    % Integrator update (trapezoidal integration with anti-windup)
    int_p = ctrl_in.int_p + fcs.Ki_p * ep * dt;
    int_q = ctrl_in.int_q + fcs.Ki_q * eq * dt;
    int_r = ctrl_in.int_r + fcs.Ki_r * er * dt;

    % Anti-windup: clamp integrators
    int_p = max(min(int_p, fcs.int_lim_p), -fcs.int_lim_p);
    int_q = max(min(int_q, fcs.int_lim_q), -fcs.int_lim_q);
    int_r = max(min(int_r, fcs.int_lim_r), -fcs.int_lim_r);

    % Desired angular acceleration = proportional + integral
    pdot_des = fcs.K_p * ep + int_p;
    qdot_des = fcs.K_q * eq + int_q;
    rdot_des = fcs.K_r * er + int_r;

    omega_dot_des  = [pdot_des; qdot_des; rdot_des];
    omega_dot_meas = [pdot_meas; qdot_meas; rdot_meas];

    % --- Compute B matrix (control effectiveness) ---
    %  B maps surface deflection increments [da; de; dr] to angular
    %  acceleration increments [pdot; qdot; rdot].
    %  B = J_inv * qbar * S * [b*Cl_da, 0, b*Cl_dr; 0, c*Cm_de, 0; b*Cn_da, 0, b*Cn_dr]
    ar = ac.aero;
    G = qbar * ac.S * [
        ac.b     * ar.Cl_da,   0,                  ac.b     * ar.Cl_dr;
        0,                     ac.c_bar * ar.Cm_de, 0;
        ac.b     * ar.Cn_da,   0,                  ac.b     * ar.Cn_dr
    ];

    Gam = ac.Gamma;
    J_inv = [
        ac.Izz/Gam,  0,          ac.Ixz/Gam;
        0,           1/ac.Iyy,   0;
        ac.Ixz/Gam,  0,          ac.Ixx/Gam
    ];

    B = J_inv * G;

    % --- Invert B matrix ---
    %  B is 3x3: [B11 0 B13; 0 B22 0; B31 0 B33]
    %  Analytical inverse for this sparse structure:
    B11 = B(1,1); B13 = B(1,3);
    B22 = B(2,2);
    B31 = B(3,1); B33 = B(3,3);

    det_roll_yaw = B11 * B33 - B13 * B31;

    if abs(det_roll_yaw) < 1e-10 || abs(B22) < 1e-10
        % Fallback: no control increment if B is singular
        % This should not happen in normal flight
        delta_u = [0; 0; 0];
    else
        B_inv = [
             B33/det_roll_yaw,  0,       -B13/det_roll_yaw;
             0,                 1/B22,    0;
            -B31/det_roll_yaw,  0,        B11/det_roll_yaw
        ];

        % --- INDI control law ---
        %  delta_increment = B_inv * (desired_accel - measured_accel)
        delta_u = B_inv * (omega_dot_des - omega_dot_meas);
    end

    % --- Add increment to previous surface command ---
    da_raw = da_prev + delta_u(1);
    de_raw = de_prev + delta_u(2);
    dr_raw = dr_prev + delta_u(3);

    % ================================================================
    %  SECTION E: OUTPUT LIMITING
    % ================================================================
    %  Apply rate limiting and position saturation to prevent
    %  commanding beyond actuator physical limits.

    % Rate limiting (limit how fast the command can change)
    da_cmd = rate_limit(da_raw, da_prev, fcs.da_rate_max, dt);
    de_cmd = rate_limit(de_raw, de_prev, fcs.de_rate_max, dt);
    dr_cmd = rate_limit(dr_raw, dr_prev, fcs.dr_rate_max, dt);

    % Position saturation (limit to actuator range)
    da_cmd = max(min(da_cmd,  ac.act.aileron.pos_max),  -ac.act.aileron.pos_max);
    de_cmd = max(min(de_cmd,  ac.act.elevator.pos_max), -ac.act.elevator.pos_max);
    dr_cmd = max(min(dr_cmd,  ac.act.rudder.pos_max),   -ac.act.rudder.pos_max);

    % ================================================================
    %  SECTION F: UPDATE CONTROLLER STATE
    % ================================================================

    ctrl_out.int_p = int_p;
    ctrl_out.int_q = int_q;
    ctrl_out.int_r = int_r;

    % ================================================================
    %  SECTION G: DEBUG OUTPUTS
    % ================================================================

    debug.alpha     = rad2deg(alpha);
    debug.beta      = rad2deg(beta);
    debug.V         = V;
    debug.qbar      = qbar;
    debug.p_cmd     = p_cmd;
    debug.q_cmd     = q_cmd;
    debug.r_cmd     = r_cmd;
    debug.pdot_des  = pdot_des;
    debug.qdot_des  = qdot_des;
    debug.rdot_des  = rdot_des;
    debug.pdot_meas = pdot_meas;
    debug.qdot_meas = qdot_meas;
    debug.rdot_meas = rdot_meas;
    debug.delta_u   = delta_u;
    debug.B_cond    = cond(B);

end

% =====================================================================
%  HELPER FUNCTIONS
% =====================================================================

function y = rate_limit(cmd, prev, max_rate, dt)
%RATE_LIMIT  Limit the rate of change of a signal.
    delta = cmd - prev;
    max_delta = max_rate * dt;
    delta = max(min(delta, max_delta), -max_delta);
    y = prev + delta;
end

function [T, a, P, rho] = isa_atm_ctrl(alt)
%ISA_ATM_CTRL  Standard atmosphere for controller (approximate).
    T0 = 288.15; P0 = 101325; L = 0.0065; R = 287.05; g0 = 9.81;
    alt = max(alt, 0);
    T = max(T0 - L*alt, 216.65);
    P = P0 * (T/T0)^(g0/(R*L));
    rho = P / (R*T);
    a = sqrt(1.4*R*T);
end
