%% NDI Controller Setup Script
%  Configures the Nonlinear Dynamic Inversion flight controller
%  for the pitch-unstable aircraft defined in setup_aircraft_parameters.m
%
%  Run this script AFTER setup_aircraft_parameters.m and BEFORE starting
%  the Simulink model.
%
%  This script:
%    1. Loads the aircraft parameters
%    2. Sets NDI controller gains (with guidance on tuning)
%    3. Analyses closed-loop stability margins
%    4. Saves everything to workspace for the S-function

disp('========================================');
disp('NDI Controller Setup');
disp('Nonlinear Dynamic Inversion for');
disp('Pitch-Unstable Aircraft');
disp('========================================');
disp(' ');

%% Step 1: Load aircraft parameters

if ~exist('aircraft', 'var')
    if exist('aircraft_params.mat', 'file')
        load('aircraft_params.mat');
        disp('[NDI] Loaded aircraft parameters from MAT file');
    else
        disp('[NDI] Running setup_aircraft_parameters first...');
        setup_aircraft_parameters;
    end
end

disp(' ');

%% Step 2: NDI Gain Selection
%
%  The NDI controller has two loops, each with selectable bandwidth.
%  The key constraint is TIME-SCALE SEPARATION:
%
%    Inner loop bandwidth >> Outer loop bandwidth
%
%  Rule of thumb: inner loop 3-5x faster than outer loop.
%  Actuator bandwidth sets the upper bound on the inner loop.
%
%  For this pitch-unstable aircraft:
%    - Open-loop divergence rate ~3 rad/s (from Cmalpha = +0.3)
%    - Inner loop must be FASTER than the divergence to stabilize
%    - Minimum inner-loop bandwidth: ~2x divergence rate

disp('NDI Gain Configuration:');
disp(' ');

% ---- Outer Loop Gains (attitude tracking) ----
%
%  These set the bandwidth of Euler angle response.
%  Higher = faster tracking but more sensitive to noise.
%  Typical range: 1-5 rad/s for fixed-wing aircraft.

ndi_gains.K_phi   = 3.0;     % Roll angle bandwidth (rad/s)
ndi_gains.K_theta = 4.0;     % Pitch angle bandwidth (rad/s)
                              %   Set higher because pitch is unstable —
                              %   we need fast pitch attitude correction
ndi_gains.K_beta  = 2.0;     % Sideslip suppression gain (rad/s)
                              %   Drives beta -> 0 for coordinated flight

disp('  Outer Loop (attitude):');
disp(['    K_phi   = ' num2str(ndi_gains.K_phi)   ' rad/s (roll)']);
disp(['    K_theta = ' num2str(ndi_gains.K_theta)  ' rad/s (pitch)']);
disp(['    K_beta  = ' num2str(ndi_gains.K_beta)   ' rad/s (sideslip)']);
disp(' ');

% ---- Inner Loop Gains (rate tracking) ----
%
%  These set the closed-loop angular rate response after NDI cancellation.
%  With perfect model: closed-loop pole at -K_q for pitch rate, etc.
%  With model mismatch: the integral term compensates steady-state errors.
%
%  CRITICAL for unstable aircraft:
%    K_q MUST exceed the open-loop divergence rate.
%    Open-loop pitch divergence ~ sqrt(qbar*S*c*Cmalpha / Iyy)

V_nom    = 20;  % Nominal airspeed (m/s)
qbar_nom = 0.5 * 1.225 * V_nom^2;

% Estimate open-loop pitch divergence rate
divergence_rate = sqrt(qbar_nom * aircraft.S * aircraft.c * ...
                       abs(aircraft.Cmalpha) / aircraft.Iyy);
disp(['  Open-loop pitch divergence rate: ' ...
      num2str(divergence_rate, '%.1f') ' rad/s']);
disp(['  (Inner loop MUST be faster than this)']);
disp(' ');

ndi_gains.K_p  = 10.0;       % Roll rate bandwidth (rad/s)
ndi_gains.K_q  = 12.0;       % Pitch rate bandwidth (rad/s)
                              %   Must be >> divergence_rate
ndi_gains.K_r  = 8.0;        % Yaw rate bandwidth (rad/s)

% Integral gains: reject steady-state errors from model mismatch
% Typical: Ki = 0.1 to 0.5 * Kp
ndi_gains.Ki_p = 2.0;        % Roll rate integral (rad/s^2)
ndi_gains.Ki_q = 4.0;        % Pitch rate integral (rad/s^2)
                              %   Higher for pitch to reject Cmalpha errors
ndi_gains.Ki_r = 1.5;        % Yaw rate integral (rad/s^2)

disp('  Inner Loop (rate):');
disp(['    K_p  = ' num2str(ndi_gains.K_p)  ' rad/s  (roll rate)']);
disp(['    K_q  = ' num2str(ndi_gains.K_q)  ' rad/s  (pitch rate) — ' ...
      num2str(ndi_gains.K_q / divergence_rate, '%.1f') 'x divergence']);
disp(['    K_r  = ' num2str(ndi_gains.K_r)  ' rad/s  (yaw rate)']);
disp(['    Ki_p = ' num2str(ndi_gains.Ki_p) ' rad/s^2 (roll integral)']);
disp(['    Ki_q = ' num2str(ndi_gains.Ki_q) ' rad/s^2 (pitch integral)']);
disp(['    Ki_r = ' num2str(ndi_gains.Ki_r) ' rad/s^2 (yaw integral)']);
disp(' ');

% ---- Integrator anti-windup limits ----
ndi_gains.int_lim_p = 0.5;   % Roll integrator limit (rad)
ndi_gains.int_lim_q = 0.5;   % Pitch integrator limit (rad)
ndi_gains.int_lim_r = 0.5;   % Yaw integrator limit (rad)

% ---- Rate command limits ----
ndi_gains.p_max = 3.0;       % Max roll rate command (rad/s)  ~172 deg/s
ndi_gains.q_max = 2.0;       % Max pitch rate command (rad/s) ~115 deg/s
ndi_gains.r_max = 1.5;       % Max yaw rate command (rad/s)   ~86 deg/s

disp('  Limits:');
disp(['    Max roll rate:  ' num2str(rad2deg(ndi_gains.p_max), '%.0f') ' deg/s']);
disp(['    Max pitch rate: ' num2str(rad2deg(ndi_gains.q_max), '%.0f') ' deg/s']);
disp(['    Max yaw rate:   ' num2str(rad2deg(ndi_gains.r_max), '%.0f') ' deg/s']);
disp(['    Integrator limits: +/- ' num2str(ndi_gains.int_lim_q) ' rad']);
disp(' ');

%% Step 3: Closed-Loop Stability Analysis (linearized)
%
%  Under perfect NDI inversion, the closed-loop pitch rate dynamics are:
%
%    q_dot = -K_q * (q - q_cmd) - Ki_q * integral(q - q_cmd)
%
%  Characteristic equation: s^2 + K_q*s + Ki_q = 0
%  Natural frequency: omega_n = sqrt(Ki_q)
%  Damping ratio:     zeta = K_q / (2 * omega_n)

disp('========================================');
disp('Closed-Loop Analysis (ideal NDI)');
disp('========================================');
disp(' ');

axes_names = {'Roll (p)', 'Pitch (q)', 'Yaw (r)'};
K_vec  = [ndi_gains.K_p,  ndi_gains.K_q,  ndi_gains.K_r];
Ki_vec = [ndi_gains.Ki_p, ndi_gains.Ki_q, ndi_gains.Ki_r];

for i = 1:3
    wn   = sqrt(Ki_vec(i));
    zeta = K_vec(i) / (2 * wn);
    poles = roots([1, K_vec(i), Ki_vec(i)]);

    disp(['  ' axes_names{i} ':']);
    disp(['    omega_n = ' num2str(wn, '%.2f') ' rad/s']);
    disp(['    zeta    = ' num2str(zeta, '%.2f')]);
    disp(['    poles   = ' num2str(poles(1), '%.2f') ', ' num2str(poles(2), '%.2f')]);
    if zeta < 0.3
        disp('    WARNING: Low damping — increase proportional gain');
    elseif zeta > 2.0
        disp('    Note: Overdamped — slower but no overshoot');
    else
        disp('    OK: Well-damped response');
    end
    disp(' ');
end

%% Step 4: Control Authority Check
%
%  Verify that the aircraft has enough control authority at nominal
%  flight condition to produce the required moments.

disp('========================================');
disp('Control Authority Check');
disp('========================================');
disp(' ');

B_nom = [qbar_nom*aircraft.S*aircraft.b*aircraft.Clda,  0,                                            0;
         0,                                               qbar_nom*aircraft.S*aircraft.c*aircraft.Cmde, 0;
         qbar_nom*aircraft.S*aircraft.b*aircraft.Cnda,   0,                                            qbar_nom*aircraft.S*aircraft.b*aircraft.Cndr];

disp('Control effectiveness matrix B at V=20 m/s:');
disp(B_nom);

cond_B = cond(B_nom);
disp(['Condition number of B: ' num2str(cond_B, '%.1f')]);
if cond_B > 100
    disp('WARNING: B is ill-conditioned — control allocation may be poor');
elseif cond_B > 20
    disp('Caution: Moderate conditioning — monitor at low/high speeds');
else
    disp('OK: B is well-conditioned');
end
disp(' ');

% Max destabilizing pitch moment that must be cancelled
M_destab_max = qbar_nom * aircraft.S * aircraft.c * ...
               (abs(aircraft.Cm0) + abs(aircraft.Cmalpha) * deg2rad(15));
M_ctrl_max   = abs(qbar_nom * aircraft.S * aircraft.c * aircraft.Cmde) * 1.0;

disp(['Max destabilizing pitch moment (alpha=15deg): ' ...
      num2str(M_destab_max, '%.3f') ' Nm']);
disp(['Max available pitch control moment:           ' ...
      num2str(M_ctrl_max, '%.3f') ' Nm']);

if M_ctrl_max > M_destab_max
    disp('OK: Sufficient pitch control authority');
    disp(['  Authority margin: ' ...
          num2str((M_ctrl_max/M_destab_max - 1)*100, '%.0f') '%%']);
else
    disp('WARNING: Insufficient control authority at high alpha!');
    disp('  Consider reducing alpha limits or increasing Cmde.');
end
disp(' ');

%% Step 5: Save to workspace

disp('========================================');
disp('Setup Complete');
disp('========================================');
disp(' ');
disp('Variables saved to workspace:');
disp('  aircraft   - Aircraft aerodynamic & mass parameters');
disp('  ndi_gains  - NDI controller gains');
disp(' ');
disp('To use in Simulink:');
disp('  1. Add a Level-2 MATLAB S-Function block');
disp('  2. Set function name: ndi_controller_sfunc');
disp('  3. Connect inputs from aircraft_6dof_sfunc outputs:');
disp('     Port 1 (att_cmd):  [phi_cmd; theta_cmd] from command source');
disp('     Port 2 (velocity): OutputPort(3) of 6DOF  [u; v; w]');
disp('     Port 3 (omega):    OutputPort(4) of 6DOF  [p; q; r]');
disp('     Port 4 (euler):    Derived from OutputPort(2) quaternion');
disp('  4. Connect outputs to aircraft_6dof_sfunc inputs:');
disp('     OutputPort(1)(1) -> InputPort(1) aileron');
disp('     OutputPort(1)(2) -> InputPort(2) elevator');
disp('     OutputPort(1)(3) -> InputPort(3) rudder');
disp(' ');
disp('Or run:  run_ndi_closed_loop  for automated setup.');
disp(' ');
