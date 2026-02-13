%% px4_fw_pitch_parameters.m
%  =============================================================
%  PX4 Fixed-Wing Pitch Controller Parameter Definitions
%
%  This file defines all tunable parameters used in the Simulink
%  model, mapped to their PX4 firmware parameter names.
%
%  Usage:
%    >> px4_fw_pitch_parameters   % loads params into workspace
%    >> build_px4_fw_pitch_architecture  % then build model
%  =============================================================

function params = px4_fw_pitch_parameters()

    fprintf('Loading PX4 FW Pitch Controller Parameters...\n\n');

    %% ================================================================
    %  PITCH RATE CONTROLLER (Inner Loop)
    %  PX4 module: fw_att_control / ECL pitch rate controller
    %  ================================================================

    % FW_PR_P - Pitch rate proportional gain
    %   Maps pitch rate error [rad/s] to elevator deflection [normalized]
    %   Higher values give faster rate response but can cause oscillation
    params.FW_PR_P = 0.4;

    % FW_PR_I - Pitch rate integral gain
    %   Eliminates steady-state pitch rate error
    %   Important for pitch-unstable aircraft (Cmalpha > 0)
    params.FW_PR_I = 0.3;

    % FW_PR_D - Pitch rate derivative gain
    %   Applied on measurement (not setpoint) to avoid kick
    %   Provides additional damping
    params.FW_PR_D = 0.015;

    % FW_PR_IMAX - Pitch rate integrator limit
    %   Anti-windup: clamps integrator output magnitude
    params.FW_PR_IMAX = 0.4;

    % FW_R_RMAX - Maximum pitch rate [deg/s]
    %   Limits the commanded pitch rate from the attitude loop
    params.FW_P_RMAX = 120.0;    % deg/s (2.094 rad/s)

    fprintf('  Rate Loop (Inner):\n');
    fprintf('    FW_PR_P    = %.3f\n', params.FW_PR_P);
    fprintf('    FW_PR_I    = %.3f\n', params.FW_PR_I);
    fprintf('    FW_PR_D    = %.4f\n', params.FW_PR_D);
    fprintf('    FW_PR_IMAX = %.3f\n', params.FW_PR_IMAX);
    fprintf('    FW_P_RMAX  = %.1f deg/s\n\n', params.FW_P_RMAX);

    %% ================================================================
    %  PITCH ATTITUDE CONTROLLER (Outer Loop)
    %  PX4 module: fw_att_control / ECL pitch controller
    %  ================================================================

    % FW_P_TC - Pitch time constant
    %   Attitude loop proportional gain: Kp = 1/TC
    %   Lower TC = more aggressive response
    %   For pitch-unstable aircraft, use high Kp (low TC)
    params.FW_P_TC = 1.0 / 12.0;   % TC such that Kp = 12.0

    % Derived Kp for the Simulink model
    params.Kp_pitch_att = 1.0 / params.FW_P_TC;  % = 12.0

    % FW_P_LIM_MAX - Maximum pitch angle [deg]
    params.FW_P_LIM_MAX = 45.0;   % deg (0.7854 rad)

    % FW_P_LIM_MIN - Minimum pitch angle [deg]
    params.FW_P_LIM_MIN = -45.0;  % deg (-0.7854 rad)

    fprintf('  Attitude Loop (Outer):\n');
    fprintf('    Kp_pitch   = %.1f  (FW_P_TC = %.4f s)\n', ...
            params.Kp_pitch_att, params.FW_P_TC);
    fprintf('    FW_P_LIM   = [%.0f, +%.0f] deg\n\n', ...
            params.FW_P_LIM_MIN, params.FW_P_LIM_MAX);

    %% ================================================================
    %  COORDINATED TURN COMPENSATION
    %  ================================================================

    % Turn compensation feed-forward:
    %   q_coord = (g / V) * (1/cos(phi) - 1)
    %
    % This adds pitch rate during banked turns to maintain altitude
    params.TURN_COMP_ENABLED = true;
    params.TURN_COMP_TAU     = 0.1;   % filter time constant [s]

    fprintf('  Turn Compensation:\n');
    fprintf('    Enabled    = %d\n', params.TURN_COMP_ENABLED);
    fprintf('    Filter tau = %.2f s\n\n', params.TURN_COMP_TAU);

    %% ================================================================
    %  TECS (Total Energy Control System)
    %  PX4 module: fw_pos_control / TECS
    %  ================================================================

    % FW_T_CLMB_MAX - Maximum climb rate [m/s]
    params.FW_T_CLMB_MAX = 5.0;

    % FW_T_SINK_MAX - Maximum sink rate [m/s]
    params.FW_T_SINK_MAX = 3.0;

    % FW_T_TAU - TECS altitude time constant [s]
    %   Controls how aggressively altitude errors are corrected
    params.FW_T_TAU = 5.0;

    % FW_T_THR_DAMP - TECS throttle damping
    params.FW_T_THR_DAMP = 0.7;

    % FW_T_SPDWEIGHT - Speed/altitude priority weight
    %   0 = pure altitude priority
    %   1 = balanced (default)
    %   2 = pure speed priority
    params.FW_T_SPDWEIGHT = 1.0;

    % FW_T_PTCH_DAMP - TECS pitch damping
    params.FW_T_PTCH_DAMP = 0.7;

    % TECS pitch limits (tighter than attitude limits)
    params.FW_T_PTCH_MAX = 25.0;   % deg (0.4363 rad)
    params.FW_T_PTCH_MIN = -20.0;  % deg (-0.3491 rad)

    fprintf('  TECS:\n');
    fprintf('    FW_T_TAU       = %.1f s\n', params.FW_T_TAU);
    fprintf('    FW_T_CLMB_MAX  = %.1f m/s\n', params.FW_T_CLMB_MAX);
    fprintf('    FW_T_SINK_MAX  = %.1f m/s\n', params.FW_T_SINK_MAX);
    fprintf('    FW_T_SPDWEIGHT = %.1f\n', params.FW_T_SPDWEIGHT);
    fprintf('    FW_T_PTCH_DAMP = %.1f\n', params.FW_T_PTCH_DAMP);
    fprintf('    FW_T_PTCH_LIM  = [%.0f, +%.0f] deg\n\n', ...
            params.FW_T_PTCH_MIN, params.FW_T_PTCH_MAX);

    %% ================================================================
    %  AIRCRAFT PLANT PARAMETERS
    %  (Matching the companion Simulink aircraft model)
    %  ================================================================

    params.mass  = 2.0;      % kg
    params.Iyy   = 0.1;      % kg*m^2
    params.S     = 0.3;      % wing area [m^2]
    params.c     = 0.3;      % mean chord [m]
    params.b     = 1.0;      % wingspan [m]

    % Pitch aerodynamic coefficients
    params.Cm0      = 0.05;   % zero-lift pitching moment
    params.Cmalpha  = 0.3;    % POSITIVE = pitch unstable!
    params.Cmq      = -3.0;   % pitch damping (stabilizing)
    params.Cmde     = -0.5;   % elevator effectiveness

    % Static margin
    params.static_margin = -params.Cmalpha / 4.5;  % CL_alpha ~ 4.5

    fprintf('  Aircraft Plant:\n');
    fprintf('    Mass     = %.1f kg\n', params.mass);
    fprintf('    Iyy      = %.2f kg*m^2\n', params.Iyy);
    fprintf('    Cm0      = %.2f\n', params.Cm0);
    fprintf('    Cmalpha  = %.2f  (UNSTABLE)\n', params.Cmalpha);
    fprintf('    Cmq      = %.2f  (damping)\n', params.Cmq);
    fprintf('    Cmde     = %.2f  (elevator)\n', params.Cmde);
    fprintf('    Static margin = %.3f  (negative = unstable)\n\n', ...
            params.static_margin);

    %% ================================================================
    %  SENSOR MODELS
    %  ================================================================

    params.ahrs_tau  = 0.02;  % AHRS filter time constant [s]
    params.gyro_tau  = 0.01;  % Gyro filter time constant [s]
    params.baro_tau  = 0.05;  % Barometer filter time constant [s]

    fprintf('  Sensors:\n');
    fprintf('    AHRS tau  = %.3f s  (pitch angle)\n', params.ahrs_tau);
    fprintf('    Gyro tau  = %.3f s  (pitch rate)\n', params.gyro_tau);
    fprintf('    Baro tau  = %.3f s  (altitude)\n', params.baro_tau);

    %% ================================================================
    %  TIMING
    %  ================================================================

    params.dt = 0.004;        % Control loop period [s] (250 Hz)
    params.sim_time = 60;     % Default simulation time [s]

    fprintf('\n  Timing:\n');
    fprintf('    dt       = %.4f s  (%.0f Hz)\n', params.dt, 1/params.dt);
    fprintf('    Sim time = %.0f s\n', params.sim_time);

    %% Save to workspace
    assignin('base', 'px4_params', params);
    fprintf('\n  Parameters saved to workspace as "px4_params"\n');
    fprintf('==============================================\n');
end
