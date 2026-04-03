function aero_db = aero_database()
%AERO_DATABASE  Aerodynamic coefficient lookup tables for the suborbital
%   winged rocket / lifting body.
%
%   Tables are functions of Mach number and angle of attack (alpha).
%   Interpolation: bilinear via MATLAB interp2() (method 'linear', extrapolation
%   uses 'linear' clamped to table edges via clamp_alpha / clamp_mach).
%
%   Sign conventions (body frame, x-forward, y-right, z-down):
%     CL  > 0 → lift force in -z direction (upward for positive AoA)
%     CD  > 0 → drag force in -x direction (retarding)
%     CY  > 0 → side force in +y direction (rightward)
%     Cl  > 0 → rolling moment (right wing down)
%     Cm  > 0 → pitching moment (nose up — destabilising for this vehicle)
%     Cn  > 0 → yawing moment (nose right)
%
%   Control effectiveness coefficients:
%     Cmde — elevator: dCm/d(de) [per rad]
%     Clda — aileron:  dCl/d(da) [per rad]
%     Cndr — rudder:   dCn/d(dr) [per rad]
%     Cldr — rudder:   dCl/d(dr) [per rad] (roll from rudder, dihedral)
%
%   Dynamic derivatives (Mach-only, scalar per operating point):
%     Clp, Cmq, Cnr, Clr, Cnp  [per rad/s nondimensional]

%% ========================================================================
%  AXIS GRIDS
%  ========================================================================

aero_db.Mach_vec  = [0.1,  0.3,  0.5,  0.7,  0.85, 1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0];
aero_db.alpha_vec = [-10, -8, -6, -4, -2, 0, 2, 4, 6, 8, 10, 12, 15, 18, 20, 25, 30];  % deg
aero_db.beta_vec  = [-15, -10, -5, 0, 5, 10, 15];  % deg (for lateral tables)

nM = length(aero_db.Mach_vec);
nA = length(aero_db.alpha_vec);

%% ========================================================================
%  LIFT COEFFICIENT  CL(Mach, alpha)
%  ========================================================================
%  Subsonic: CL ~ CLalpha * alpha  (CLalpha ~ 2*pi per rad for thin wings)
%  Transonic: reduced CLalpha, early stall
%  Supersonic: CL lower (swept delta wing), linear to higher alpha

CL_alpha_slope = [3.8, 3.9, 4.0, 4.0, 3.8, 3.2, 2.8, 2.5, 2.2, 1.9, 1.7, 1.6];  % per rad

aero_db.CL = zeros(nM, nA);
for i = 1:nM
    alpha_rad = deg2rad(aero_db.alpha_vec);
    % Nonlinear: linear up to ~12 deg, then stall onset
    CL_lin = CL_alpha_slope(i) * alpha_rad;
    % Stall model: plateau above alpha_stall
    alpha_stall_rad = deg2rad(14 - 2*(aero_db.Mach_vec(i) > 0.8));  % stall earlier transonic
    CL_stall = CL_alpha_slope(i) * alpha_stall_rad;
    stall_blend = 1 ./ (1 + exp(8*(abs(alpha_rad) - alpha_stall_rad)));
    aero_db.CL(i,:) = CL_lin .* stall_blend + CL_stall*(1-stall_blend) .* sign(alpha_rad);
end

%% ========================================================================
%  DRAG COEFFICIENT  CD(Mach, alpha)
%  ========================================================================
%  CD = CD0(Mach) + CDi * CL^2  (induced drag)
%  Wave drag peak at Mach 1.0–1.2

CD0_vec = [0.018, 0.018, 0.019, 0.022, 0.030, 0.055, 0.048, 0.040, 0.033, 0.026, 0.022, 0.020];
CDi_vec = [0.12,  0.12,  0.11,  0.10,  0.10,  0.12,  0.13,  0.14,  0.15,  0.16,  0.16,  0.16];

aero_db.CD = zeros(nM, nA);
for i = 1:nM
    aero_db.CD(i,:) = CD0_vec(i) + CDi_vec(i) * aero_db.CL(i,:).^2;
    % Add alpha^2 term for body drag at high AoA
    aero_db.CD(i,:) = aero_db.CD(i,:) + 0.008 * deg2rad(aero_db.alpha_vec).^2;
end

%% ========================================================================
%  SIDE-FORCE COEFFICIENT  CY(Mach, alpha)
%  ========================================================================
%  CY = CYbeta(Mach, alpha) * beta
%  Stored as CYbeta table [nM x nA]

CYbeta_base = [-0.60, -0.60, -0.58, -0.56, -0.55, -0.55, -0.52, -0.50, -0.46, -0.42, -0.40, -0.38];

aero_db.CYbeta = zeros(nM, nA);
for i = 1:nM
    % CYbeta magnitude increases slightly with AoA
    alpha_effect = 1 + 0.02 * abs(aero_db.alpha_vec);
    aero_db.CYbeta(i,:) = CYbeta_base(i) * alpha_effect;
end

%% ========================================================================
%  PITCH MOMENT COEFFICIENT  Cm(Mach, alpha)
%  ========================================================================
%  Cm = Cm0 + Cmalpha * alpha + (dynamic derivatives applied separately)
%  This vehicle has Cmalpha > 0 at low dynamic pressure (unstable)
%  and Cmalpha < 0 at high dynamic pressure (aerodynamically stable in pitch).

Cm0_vec    = [ 0.02,  0.02,  0.01,  0.01,  0.00, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01];

% Cmalpha: positive (destabilising) at subsonic, transitions through zero
% at about Mach 1.5 (vehicle reaches neutral then stable at higher Mach)
Cmalpha_vec = [+0.35, +0.30, +0.25, +0.18, +0.10, +0.02, -0.08, -0.15, -0.22, -0.28, -0.30, -0.32];  % per rad

aero_db.Cm = zeros(nM, nA);
for i = 1:nM
    alpha_rad = deg2rad(aero_db.alpha_vec);
    aero_db.Cm(i,:) = Cm0_vec(i) + Cmalpha_vec(i) * alpha_rad;
    % Nonlinear pitch-up at high AoA (typical delta wing)
    aero_db.Cm(i,:) = aero_db.Cm(i,:) + 0.015 * max(0, alpha_rad - deg2rad(12)) .^ 2;
end

%% ========================================================================
%  ROLL MOMENT COEFFICIENT  Cl(Mach, alpha)
%  Cl = Clbeta*beta + Clp*p_hat + Clr*r_hat + Clda*da
%  Clbeta stored as [nM x nA] table (dihedral + sweep effect)
%  ========================================================================

Clbeta_base = [-0.08, -0.09, -0.09, -0.10, -0.12, -0.13, -0.12, -0.11, -0.10, -0.09, -0.08, -0.07];

aero_db.Clbeta = zeros(nM, nA);
for i = 1:nM
    alpha_effect = 1 + 0.03 * max(0, aero_db.alpha_vec);  % more dihedral effect at high AoA
    aero_db.Clbeta(i,:) = Clbeta_base(i) * alpha_effect;
end

%% ========================================================================
%  YAW MOMENT COEFFICIENT  Cn(Mach, alpha)
%  Cn = Cnbeta*beta + Cnr*r_hat + Cnp*p_hat + Cndr*dr + Cnda*da
%  ========================================================================

Cnbeta_base = [+0.12, +0.12, +0.11, +0.11, +0.12, +0.13, +0.12, +0.11, +0.10, +0.09, +0.08, +0.08];

aero_db.Cnbeta = zeros(nM, nA);
for i = 1:nM
    % Cn_beta decreases at high AoA (directional instability onset)
    alpha_factor = max(0.2, 1.0 - 0.04 * max(0, aero_db.alpha_vec - 8));
    aero_db.Cnbeta(i,:) = Cnbeta_base(i) * alpha_factor;
end

%% ========================================================================
%  CONTROL EFFECTIVENESS TABLES  [nM x nA]
%  ========================================================================

%  Elevator/elevon pitch effectiveness  dCm/d(de) [per rad]
Cmde_base = [-1.20, -1.20, -1.18, -1.15, -1.10, -1.00, -0.90, -0.80, -0.70, -0.58, -0.50, -0.44];
aero_db.Cmde = zeros(nM, nA);
for i = 1:nM
    alpha_factor = 1 - 0.015 * abs(aero_db.alpha_vec);
    alpha_factor = max(0.3, alpha_factor);
    aero_db.Cmde(i,:) = Cmde_base(i) * alpha_factor;
end

%  Aileron roll effectiveness  dCl/d(da) [per rad]
Clda_base = [+0.20, +0.20, +0.20, +0.19, +0.18, +0.17, +0.16, +0.15, +0.13, +0.11, +0.09, +0.08];
aero_db.Clda = zeros(nM, nA);
for i = 1:nM
    aero_db.Clda(i,:) = Clda_base(i) * ones(1, nA);
end

%  Rudder yaw effectiveness  dCn/d(dr) [per rad]
Cndr_base = [-0.10, -0.10, -0.10, -0.10, -0.11, -0.12, -0.11, -0.10, -0.09, -0.08, -0.07, -0.06];
aero_db.Cndr = zeros(nM, nA);
for i = 1:nM
    aero_db.Cndr(i,:) = Cndr_base(i) * ones(1, nA);
end

%  Rudder roll effectiveness  dCl/d(dr) [per rad] (dihedral coupling)
Cldr_base = [+0.015, +0.015, +0.014, +0.013, +0.012, +0.010, +0.009, +0.008, +0.007, +0.006, +0.005, +0.005];
aero_db.Cldr = zeros(nM, nA);
for i = 1:nM
    aero_db.Cldr(i,:) = Cldr_base(i) * ones(1, nA);
end

%  Aileron adverse yaw  dCn/d(da) [per rad]
Cnda_base = [+0.003, +0.003, +0.003, +0.003, +0.003, +0.003, +0.002, +0.002, +0.002, +0.001, +0.001, +0.001];
aero_db.Cnda = zeros(nM, nA);
for i = 1:nM
    aero_db.Cnda(i,:) = Cnda_base(i) * ones(1, nA);
end

%% ========================================================================
%  DYNAMIC DERIVATIVES (Mach vector only — scalar at each Mach)
%  Non-dimensional: p_hat = p*b/(2V),  q_hat = q*c/(2V),  r_hat = r*b/(2V)
%  ========================================================================

%  Roll damping  dCl/d(p_hat)
aero_db.Clp = [-0.50, -0.50, -0.48, -0.45, -0.42, -0.40, -0.38, -0.35, -0.32, -0.28, -0.25, -0.23];

%  Pitch damping  dCm/d(q_hat)
aero_db.Cmq = [-8.5, -8.5, -8.3, -8.0, -7.5, -6.5, -6.0, -5.5, -5.0, -4.5, -4.0, -3.8];

%  Yaw damping  dCn/d(r_hat)
aero_db.Cnr = [-0.15, -0.15, -0.15, -0.14, -0.14, -0.13, -0.12, -0.11, -0.10, -0.09, -0.08, -0.07];

%  Roll due to yaw rate  dCl/d(r_hat)
aero_db.Clr = [+0.08, +0.08, +0.08, +0.07, +0.07, +0.06, +0.06, +0.05, +0.05, +0.04, +0.04, +0.03];

%  Yaw due to roll rate  dCn/d(p_hat)
aero_db.Cnp = [-0.02, -0.02, -0.02, -0.02, -0.02, -0.02, -0.01, -0.01, -0.01, -0.01, -0.01, -0.01];

%% ========================================================================
%  STORE DERIVED QUANTITIES
%  ========================================================================

aero_db.Cmalpha_vec = Cmalpha_vec;  % Pitch stiffness vs Mach (used by LQR lineariser)
aero_db.Cm0_vec     = Cm0_vec;
aero_db.CLalpha_vec = CL_alpha_slope;

fprintf('[Aero DB] Loaded: %d Mach points x %d alpha points\n', nM, nA);
fprintf('  Mach range: %.1f – %.1f\n', aero_db.Mach_vec(1), aero_db.Mach_vec(end));
fprintf('  Alpha range: %.0f° to %.0f°\n', aero_db.alpha_vec(1), aero_db.alpha_vec(end));

end
