%% build_px4_fw_pitch_architecture.m
%  =============================================================
%  Programmatically builds a Simulink model that replicates the
%  ORIGINAL PX4 fixed-wing pitch attitude control architecture.
%
%  Architecture (from PX4 firmware: fw_att_control & TECS):
%
%   ┌─────────────────────────────────────────────────────────────┐
%   │                    TECS (Outer Guidance)                    │
%   │  alt_sp ─┐                                                  │
%   │  alt ────┼──► Energy Balance ──► pitch_sp                   │
%   │  aspd_sp ┘       + Throttle  ──► throttle_sp                │
%   └────────────────────┬────────────────────────────────────────┘
%                        │ pitch_sp
%   ┌────────────────────▼────────────────────────────────────────┐
%   │              Pitch Attitude Controller (Outer Loop)          │
%   │  pitch_sp ─┐                                                │
%   │  pitch ────┼──► P-controller ──► pitch_rate_sp              │
%   │  roll ─────┘  + Turn Compensation                           │
%   └────────────────────┬────────────────────────────────────────┘
%                        │ pitch_rate_sp
%   ┌────────────────────▼────────────────────────────────────────┐
%   │              Pitch Rate Controller (Inner Loop)              │
%   │  pitch_rate_sp ─┐                                           │
%   │  pitch_rate ────┼──► PID ──► elevator_cmd [-1,+1]           │
%   └────────────────────┬────────────────────────────────────────┘
%                        │ elevator
%                        ▼
%                   Actuator / Mixer
%
%  Usage:
%    >> build_px4_fw_pitch_architecture
%    Opens the model 'px4_fw_pitch_architecture' in Simulink.
%
%  Requirements: MATLAB R2020b+ with Simulink
%  =============================================================

function build_px4_fw_pitch_architecture()

    modelName = 'px4_fw_pitch_architecture';

    %% Close existing model if open
    if bdIsLoaded(modelName)
        close_system(modelName, 0);
    end

    %% Create new model
    new_system(modelName);
    open_system(modelName);

    % Set solver for fixed-wing flight control (250 Hz = 0.004s)
    set_param(modelName, 'Solver', 'ode4');
    set_param(modelName, 'FixedStep', '0.004');
    set_param(modelName, 'StopTime', '60');
    set_param(modelName, 'SimulationMode', 'normal');

    %% ================================================================
    %  COLOR SCHEME (PX4 style)
    %  ================================================================
    color_tecs      = 'orange';
    color_att_outer = '[0.2, 0.6, 1.0]';
    color_rate_inner= '[0.2, 0.8, 0.4]';
    color_plant     = '[0.8, 0.4, 0.4]';
    color_sensor    = '[0.7, 0.7, 0.7]';
    color_input     = '[1.0, 1.0, 0.6]';

    %% ================================================================
    %  SECTION 1: INPUT SETPOINTS
    %  ================================================================
    fprintf('  Building input setpoint blocks...\n');

    % Altitude setpoint
    add_block('simulink/Sources/Constant', ...
        [modelName '/Alt_Setpoint'], ...
        'Value', '100', ...
        'Position', [50 50 130 80]);
    set_param([modelName '/Alt_Setpoint'], 'BackgroundColor', color_input);

    % Airspeed setpoint
    add_block('simulink/Sources/Constant', ...
        [modelName '/Airspeed_Setpoint'], ...
        'Value', '15', ...
        'Position', [50 120 130 150]);
    set_param([modelName '/Airspeed_Setpoint'], 'BackgroundColor', color_input);

    % Pitch setpoint (manual / from TECS)
    add_block('simulink/Sources/Constant', ...
        [modelName '/Pitch_SP_Manual'], ...
        'Value', '0.0', ...
        'Position', [50 200 130 230]);
    set_param([modelName '/Pitch_SP_Manual'], 'BackgroundColor', color_input);

    % Mode switch: 0=manual pitch_sp, 1=TECS pitch_sp
    add_block('simulink/Sources/Constant', ...
        [modelName '/Mode_Select'], ...
        'Value', '1', ...
        'Position', [50 280 130 310]);

    %% ================================================================
    %  SECTION 2: TECS - Total Energy Control System
    %  ================================================================
    fprintf('  Building TECS subsystem...\n');

    tecs_path = [modelName '/TECS'];
    add_block('simulink/Ports & Subsystems/Subsystem', tecs_path);
    set_param(tecs_path, 'Position', [250 40 500 180]);
    set_param(tecs_path, 'BackgroundColor', color_tecs);

    % Delete default line inside TECS
    delete_line(tecs_path, 'In1/1', 'Out1/1');
    delete_block([tecs_path '/In1']);
    delete_block([tecs_path '/Out1']);

    % --- TECS Inputs ---
    add_block('simulink/Ports & Subsystems/In1', [tecs_path '/alt_sp']);
    set_param([tecs_path '/alt_sp'], 'Position', [30 30 60 50]);

    add_block('simulink/Ports & Subsystems/In1', [tecs_path '/alt_meas']);
    set_param([tecs_path '/alt_meas'], 'Position', [30 80 60 100]);

    add_block('simulink/Ports & Subsystems/In1', [tecs_path '/aspd_sp']);
    set_param([tecs_path '/aspd_sp'], 'Position', [30 130 60 150]);

    add_block('simulink/Ports & Subsystems/In1', [tecs_path '/aspd_meas']);
    set_param([tecs_path '/aspd_meas'], 'Position', [30 180 60 200]);

    % --- TECS: Altitude Error ---
    add_block('simulink/Math Operations/Sum', [tecs_path '/Alt_Error']);
    set_param([tecs_path '/Alt_Error'], 'Inputs', '+-', ...
        'Position', [120 50 150 80]);

    add_line(tecs_path, 'alt_sp/1', 'Alt_Error/1');
    add_line(tecs_path, 'alt_meas/1', 'Alt_Error/2');

    % --- TECS: Climb Rate Command (altitude error / time_const) ---
    add_block('simulink/Math Operations/Gain', [tecs_path '/Tau_Inv']);
    set_param([tecs_path '/Tau_Inv'], 'Gain', '0.2', ...
        'Position', [190 50 230 80]);  % 1/tau = 1/5.0
    add_line(tecs_path, 'Alt_Error/1', 'Tau_Inv/1');

    % --- TECS: Climb Rate Saturation ---
    add_block('simulink/Discontinuities/Saturation', [tecs_path '/ClimbRate_Sat']);
    set_param([tecs_path '/ClimbRate_Sat'], ...
        'UpperLimit', '5.0', 'LowerLimit', '-3.0', ...
        'Position', [270 50 320 80]);
    add_line(tecs_path, 'Tau_Inv/1', 'ClimbRate_Sat/1');

    % --- TECS: Specific Potential Energy Rate (SPE_rate = hdot_cmd) ---
    %  SPE_rate_error = hdot_cmd - hdot_meas
    %  (For simplicity, hdot_meas comes from derivative of altitude)

    % --- TECS: Energy Balance -> Pitch ---
    add_block('simulink/Math Operations/Gain', [tecs_path '/Energy_to_Pitch']);
    set_param([tecs_path '/Energy_to_Pitch'], ...
        'Gain', '1/(9.81*5.0)', ...  % 1/(g*tau)
        'Position', [370 50 430 80]);
    add_line(tecs_path, 'ClimbRate_Sat/1', 'Energy_to_Pitch/1');

    % --- TECS: Pitch Saturation ---
    add_block('simulink/Discontinuities/Saturation', [tecs_path '/Pitch_Sat']);
    set_param([tecs_path '/Pitch_Sat'], ...
        'UpperLimit', '0.4363', 'LowerLimit', '-0.3491', ...
        'Position', [480 50 540 80]);
    add_line(tecs_path, 'Energy_to_Pitch/1', 'Pitch_Sat/1');

    % --- TECS: Throttle path (energy total -> throttle) ---
    add_block('simulink/Math Operations/Sum', [tecs_path '/Spd_Error']);
    set_param([tecs_path '/Spd_Error'], 'Inputs', '+-', ...
        'Position', [120 150 150 180]);
    add_line(tecs_path, 'aspd_sp/1', 'Spd_Error/1');
    add_line(tecs_path, 'aspd_meas/1', 'Spd_Error/2');

    add_block('simulink/Math Operations/Gain', [tecs_path '/Spd_to_Throttle']);
    set_param([tecs_path '/Spd_to_Throttle'], 'Gain', '0.1', ...
        'Position', [190 150 250 180]);
    add_line(tecs_path, 'Spd_Error/1', 'Spd_to_Throttle/1');

    add_block('simulink/Discontinuities/Saturation', [tecs_path '/Throttle_Sat']);
    set_param([tecs_path '/Throttle_Sat'], ...
        'UpperLimit', '1.0', 'LowerLimit', '0.0', ...
        'Position', [290 150 350 180]);
    add_line(tecs_path, 'Spd_to_Throttle/1', 'Throttle_Sat/1');

    % --- TECS Outputs ---
    add_block('simulink/Ports & Subsystems/Out1', [tecs_path '/pitch_sp']);
    set_param([tecs_path '/pitch_sp'], 'Position', [590 55 620 75]);
    add_line(tecs_path, 'Pitch_Sat/1', 'pitch_sp/1');

    add_block('simulink/Ports & Subsystems/Out1', [tecs_path '/throttle_cmd']);
    set_param([tecs_path '/throttle_cmd'], 'Position', [400 155 430 175]);
    add_line(tecs_path, 'Throttle_Sat/1', 'throttle_cmd/1');

    %% ================================================================
    %  SECTION 3: PITCH SP MODE SWITCH
    %  ================================================================
    fprintf('  Building mode switch...\n');

    add_block('simulink/Signal Routing/Switch', ...
        [modelName '/Pitch_SP_Switch']);
    set_param([modelName '/Pitch_SP_Switch'], ...
        'Criteria', 'u2 > Threshold', 'Threshold', '0.5', ...
        'Position', [580 190 640 260]);

    %% ================================================================
    %  SECTION 4: PITCH ATTITUDE CONTROLLER (Outer Loop)
    %  ================================================================
    fprintf('  Building pitch attitude controller (outer loop)...\n');

    att_path = [modelName '/Pitch_Attitude_Ctrl'];
    add_block('simulink/Ports & Subsystems/Subsystem', att_path);
    set_param(att_path, 'Position', [740 160 1000 310]);
    set_param(att_path, 'BackgroundColor', color_att_outer);

    % Clear default contents
    delete_line(att_path, 'In1/1', 'Out1/1');
    delete_block([att_path '/In1']);
    delete_block([att_path '/Out1']);

    % --- Attitude Inputs ---
    add_block('simulink/Ports & Subsystems/In1', [att_path '/pitch_sp']);
    set_param([att_path '/pitch_sp'], 'Position', [30 30 60 50]);

    add_block('simulink/Ports & Subsystems/In1', [att_path '/pitch_meas']);
    set_param([att_path '/pitch_meas'], 'Position', [30 90 60 110]);

    add_block('simulink/Ports & Subsystems/In1', [att_path '/roll_meas']);
    set_param([att_path '/roll_meas'], 'Position', [30 150 60 170]);

    add_block('simulink/Ports & Subsystems/In1', [att_path '/airspeed']);
    set_param([att_path '/airspeed'], 'Position', [30 210 60 230]);

    % --- Pitch Error = pitch_sp - pitch_meas ---
    add_block('simulink/Math Operations/Sum', [att_path '/Pitch_Error']);
    set_param([att_path '/Pitch_Error'], 'Inputs', '+-', ...
        'Position', [140 50 170 80]);
    add_line(att_path, 'pitch_sp/1', 'Pitch_Error/1');
    add_line(att_path, 'pitch_meas/1', 'Pitch_Error/2');

    % --- P Gain: Kp_pitch = 12.0 (PX4: FW_PR_P) ---
    add_block('simulink/Math Operations/Gain', [att_path '/Kp_Pitch']);
    set_param([att_path '/Kp_Pitch'], 'Gain', '12.0', ...
        'Position', [220 50 280 80]);
    add_line(att_path, 'Pitch_Error/1', 'Kp_Pitch/1');

    % --- Coordinated Turn Compensation ---
    %  q_coord = (g/V) * (1/cos(roll) - 1)
    add_block('simulink/Math Operations/Trigonometric Function', ...
        [att_path '/Cos_Roll']);
    set_param([att_path '/Cos_Roll'], 'Operator', 'cos', ...
        'Position', [120 150 170 180]);
    add_line(att_path, 'roll_meas/1', 'Cos_Roll/1');

    add_block('simulink/Math Operations/Math Function', ...
        [att_path '/Inv_CosRoll']);
    set_param([att_path '/Inv_CosRoll'], 'Operator', 'reciprocal', ...
        'Position', [200 150 250 180]);
    add_line(att_path, 'Cos_Roll/1', 'Inv_CosRoll/1');

    add_block('simulink/Sources/Constant', [att_path '/One']);
    set_param([att_path '/One'], 'Value', '1', ...
        'Position', [200 200 230 220]);

    add_block('simulink/Math Operations/Sum', [att_path '/LoadFactor_Minus1']);
    set_param([att_path '/LoadFactor_Minus1'], 'Inputs', '+-', ...
        'Position', [280 160 310 190]);
    add_line(att_path, 'Inv_CosRoll/1', 'LoadFactor_Minus1/1');
    add_line(att_path, 'One/1', 'LoadFactor_Minus1/2');

    add_block('simulink/Sources/Constant', [att_path '/Gravity']);
    set_param([att_path '/Gravity'], 'Value', '9.81', ...
        'Position', [280 220 320 240]);

    add_block('simulink/Math Operations/Product', [att_path '/g_times_LF']);
    set_param([att_path '/g_times_LF'], ...
        'Position', [360 170 400 210]);
    add_line(att_path, 'LoadFactor_Minus1/1', 'g_times_LF/1');
    add_line(att_path, 'Gravity/1', 'g_times_LF/2');

    add_block('simulink/Math Operations/Divide', [att_path '/g_over_V']);
    set_param([att_path '/g_over_V'], ...
        'Position', [440 180 480 220]);
    add_line(att_path, 'g_times_LF/1', 'g_over_V/1');
    add_line(att_path, 'airspeed/1', 'g_over_V/2');

    % --- Sum: q_cmd = Kp * error + q_coord ---
    add_block('simulink/Math Operations/Sum', [att_path '/Rate_Sum']);
    set_param([att_path '/Rate_Sum'], 'Inputs', '++', ...
        'Position', [520 60 550 100]);
    add_line(att_path, 'Kp_Pitch/1', 'Rate_Sum/1');
    add_line(att_path, 'g_over_V/1', 'Rate_Sum/2');

    % --- Rate Saturation: +/- 120 deg/s = 2.094 rad/s ---
    add_block('simulink/Discontinuities/Saturation', [att_path '/Rate_Sat']);
    set_param([att_path '/Rate_Sat'], ...
        'UpperLimit', '2.094', 'LowerLimit', '-2.094', ...
        'Position', [590 60 650 100]);
    add_line(att_path, 'Rate_Sum/1', 'Rate_Sat/1');

    % --- Output ---
    add_block('simulink/Ports & Subsystems/Out1', [att_path '/rate_sp']);
    set_param([att_path '/rate_sp'], 'Position', [700 70 730 90]);
    add_line(att_path, 'Rate_Sat/1', 'rate_sp/1');

    %% ================================================================
    %  SECTION 5: PITCH RATE CONTROLLER (Inner Loop)
    %  ================================================================
    fprintf('  Building pitch rate controller (inner loop)...\n');

    rate_path = [modelName '/Pitch_Rate_Ctrl'];
    add_block('simulink/Ports & Subsystems/Subsystem', rate_path);
    set_param(rate_path, 'Position', [1100 180 1350 310]);
    set_param(rate_path, 'BackgroundColor', color_rate_inner);

    % Clear default contents
    delete_line(rate_path, 'In1/1', 'Out1/1');
    delete_block([rate_path '/In1']);
    delete_block([rate_path '/Out1']);

    % --- Rate Inputs ---
    add_block('simulink/Ports & Subsystems/In1', [rate_path '/rate_sp']);
    set_param([rate_path '/rate_sp'], 'Position', [30 40 60 60]);

    add_block('simulink/Ports & Subsystems/In1', [rate_path '/rate_meas']);
    set_param([rate_path '/rate_meas'], 'Position', [30 110 60 130]);

    % --- Rate Error ---
    add_block('simulink/Math Operations/Sum', [rate_path '/Rate_Error']);
    set_param([rate_path '/Rate_Error'], 'Inputs', '+-', ...
        'Position', [120 60 150 90]);
    add_line(rate_path, 'rate_sp/1', 'Rate_Error/1');
    add_line(rate_path, 'rate_meas/1', 'Rate_Error/2');

    % --- PID: Proportional (Kp = 0.4, PX4: FW_PR_P) ---
    add_block('simulink/Math Operations/Gain', [rate_path '/Kp_Rate']);
    set_param([rate_path '/Kp_Rate'], 'Gain', '0.4', ...
        'Position', [200 30 260 60]);
    add_line(rate_path, 'Rate_Error/1', 'Kp_Rate/1');

    % --- PID: Integrator (Ki = 0.3, PX4: FW_PR_I) ---
    add_block('simulink/Math Operations/Gain', [rate_path '/Ki_Rate']);
    set_param([rate_path '/Ki_Rate'], 'Gain', '0.3', ...
        'Position', [200 80 260 110]);
    add_line(rate_path, 'Rate_Error/1', 'Ki_Rate/1');

    add_block('simulink/Continuous/Integrator', [rate_path '/Integrator']);
    set_param([rate_path '/Integrator'], ...
        'UpperSaturationLimit', '0.4', ...
        'LowerSaturationLimit', '-0.4', ...
        'Position', [300 80 360 110]);
    add_line(rate_path, 'Ki_Rate/1', 'Integrator/1');

    % --- PID: Derivative (Kd = 0.015, PX4: FW_PR_D) on measurement ---
    add_block('simulink/Continuous/Derivative', [rate_path '/Deriv']);
    set_param([rate_path '/Deriv'], 'Position', [120 150 180 180]);
    add_line(rate_path, 'rate_meas/1', 'Deriv/1');

    add_block('simulink/Math Operations/Gain', [rate_path '/Kd_Rate']);
    set_param([rate_path '/Kd_Rate'], 'Gain', '-0.015', ...
        'Position', [220 150 280 180]);
    add_line(rate_path, 'Deriv/1', 'Kd_Rate/1');

    % --- PID Sum ---
    add_block('simulink/Math Operations/Sum', [rate_path '/PID_Sum']);
    set_param([rate_path '/PID_Sum'], 'Inputs', '+++', ...
        'Position', [400 60 440 120]);
    add_line(rate_path, 'Kp_Rate/1', 'PID_Sum/1');
    add_line(rate_path, 'Integrator/1', 'PID_Sum/2');
    add_line(rate_path, 'Kd_Rate/1', 'PID_Sum/3');

    % --- Output Saturation [-1, +1] -> Elevator ---
    add_block('simulink/Discontinuities/Saturation', [rate_path '/Elev_Sat']);
    set_param([rate_path '/Elev_Sat'], ...
        'UpperLimit', '1.0', 'LowerLimit', '-1.0', ...
        'Position', [490 70 550 110]);
    add_line(rate_path, 'PID_Sum/1', 'Elev_Sat/1');

    % --- Output ---
    add_block('simulink/Ports & Subsystems/Out1', [rate_path '/elevator_cmd']);
    set_param([rate_path '/elevator_cmd'], 'Position', [600 80 630 100]);
    add_line(rate_path, 'Elev_Sat/1', 'elevator_cmd/1');

    %% ================================================================
    %  SECTION 6: SIMPLIFIED AIRCRAFT PLANT
    %  ================================================================
    fprintf('  Building aircraft plant model...\n');

    plant_path = [modelName '/Aircraft_Plant'];
    add_block('simulink/Ports & Subsystems/Subsystem', plant_path);
    set_param(plant_path, 'Position', [1100 400 1350 540]);
    set_param(plant_path, 'BackgroundColor', color_plant);

    delete_line(plant_path, 'In1/1', 'Out1/1');
    delete_block([plant_path '/In1']);
    delete_block([plant_path '/Out1']);

    % Input: elevator
    add_block('simulink/Ports & Subsystems/In1', [plant_path '/elevator']);
    set_param([plant_path '/elevator'], 'Position', [30 30 60 50]);

    % Input: throttle
    add_block('simulink/Ports & Subsystems/In1', [plant_path '/throttle']);
    set_param([plant_path '/throttle'], 'Position', [30 100 60 120]);

    % --- Pitch dynamics: Iyy * q_dot = M_elev * elevator + M_aero ---
    %  Simplified: q_dot = (M_eff / Iyy) * elev + Cmalpha/Iyy * theta
    %  M_eff = 2.0 Nm, Iyy = 0.1, Cmalpha = +0.3 (unstable)

    % Elevator torque
    add_block('simulink/Math Operations/Gain', [plant_path '/M_elev']);
    set_param([plant_path '/M_elev'], 'Gain', '20.0', ...  % M_eff/Iyy
        'Position', [120 30 180 60]);
    add_line(plant_path, 'elevator/1', 'M_elev/1');

    % Aerodynamic destabilizing moment: Cmalpha_eff * theta
    add_block('simulink/Math Operations/Gain', [plant_path '/Cma_eff']);
    set_param([plant_path '/Cma_eff'], 'Gain', '3.0', ...  % Cmalpha/Iyy (unstable!)
        'Position', [370 100 430 130]);

    % Pitch damping: Cmq_eff * q
    add_block('simulink/Math Operations/Gain', [plant_path '/Cmq_eff']);
    set_param([plant_path '/Cmq_eff'], 'Gain', '-30.0', ...  % Cmq/Iyy
        'Position', [370 160 430 190]);

    % Sum of moments
    add_block('simulink/Math Operations/Sum', [plant_path '/Moment_Sum']);
    set_param([plant_path '/Moment_Sum'], 'Inputs', '+++', ...
        'Position', [240 40 280 100]);
    add_line(plant_path, 'M_elev/1', 'Moment_Sum/1');

    % q_dot -> q (integrate)
    add_block('simulink/Continuous/Integrator', [plant_path '/q_Integrator']);
    set_param([plant_path '/q_Integrator'], 'InitialCondition', '0', ...
        'Position', [330 50 380 80]);
    add_line(plant_path, 'Moment_Sum/1', 'q_Integrator/1');

    % q -> theta (integrate)
    add_block('simulink/Continuous/Integrator', [plant_path '/theta_Integrator']);
    set_param([plant_path '/theta_Integrator'], 'InitialCondition', '0', ...
        'Position', [470 50 520 80]);
    add_line(plant_path, 'q_Integrator/1', 'theta_Integrator/1');

    % Feedback: theta -> Cma moment
    add_line(plant_path, 'theta_Integrator/1', 'Cma_eff/1');
    add_line(plant_path, 'Cma_eff/1', 'Moment_Sum/2');

    % Feedback: q -> Cmq damping
    add_line(plant_path, 'q_Integrator/1', 'Cmq_eff/1');
    add_line(plant_path, 'Cmq_eff/1', 'Moment_Sum/3');

    % --- Altitude dynamics (simplified): h_dot = V * sin(theta) ---
    add_block('simulink/Math Operations/Trigonometric Function', ...
        [plant_path '/Sin_Theta']);
    set_param([plant_path '/Sin_Theta'], 'Operator', 'sin', ...
        'Position', [560 50 610 80]);
    add_line(plant_path, 'theta_Integrator/1', 'Sin_Theta/1');

    add_block('simulink/Sources/Constant', [plant_path '/Airspeed_V']);
    set_param([plant_path '/Airspeed_V'], 'Value', '15', ...
        'Position', [560 110 600 130]);

    add_block('simulink/Math Operations/Product', [plant_path '/V_sin_theta']);
    set_param([plant_path '/V_sin_theta'], ...
        'Position', [650 60 690 100]);
    add_line(plant_path, 'Sin_Theta/1', 'V_sin_theta/1');
    add_line(plant_path, 'Airspeed_V/1', 'V_sin_theta/2');

    add_block('simulink/Continuous/Integrator', [plant_path '/Alt_Integrator']);
    set_param([plant_path '/Alt_Integrator'], 'InitialCondition', '100', ...
        'Position', [730 60 780 90]);
    add_line(plant_path, 'V_sin_theta/1', 'Alt_Integrator/1');

    % --- Outputs ---
    add_block('simulink/Ports & Subsystems/Out1', [plant_path '/pitch']);
    set_param([plant_path '/pitch'], 'Position', [600 15 630 35]);
    add_line(plant_path, 'theta_Integrator/1', 'pitch/1');

    add_block('simulink/Ports & Subsystems/Out1', [plant_path '/pitch_rate']);
    set_param([plant_path '/pitch_rate'], 'Position', [430 55 460 75]);
    add_line(plant_path, 'q_Integrator/1', 'pitch_rate/1');

    add_block('simulink/Ports & Subsystems/Out1', [plant_path '/altitude']);
    set_param([plant_path '/altitude'], 'Position', [830 65 860 85]);
    add_line(plant_path, 'Alt_Integrator/1', 'altitude/1');

    %% ================================================================
    %  SECTION 7: SENSOR BLOCK (add noise, delays)
    %  ================================================================
    fprintf('  Building sensor models...\n');

    sensor_path = [modelName '/Sensors'];
    add_block('simulink/Ports & Subsystems/Subsystem', sensor_path);
    set_param(sensor_path, 'Position', [740 400 1000 540]);
    set_param(sensor_path, 'BackgroundColor', color_sensor);

    delete_line(sensor_path, 'In1/1', 'Out1/1');
    delete_block([sensor_path '/In1']);
    delete_block([sensor_path '/Out1']);

    % Inputs
    add_block('simulink/Ports & Subsystems/In1', [sensor_path '/pitch_true']);
    set_param([sensor_path '/pitch_true'], 'Position', [30 30 60 50]);
    add_block('simulink/Ports & Subsystems/In1', [sensor_path '/rate_true']);
    set_param([sensor_path '/rate_true'], 'Position', [30 80 60 100]);
    add_block('simulink/Ports & Subsystems/In1', [sensor_path '/alt_true']);
    set_param([sensor_path '/alt_true'], 'Position', [30 130 60 150]);

    % AHRS filter (low-pass on pitch) - 1st order, tau=0.02s
    add_block('simulink/Continuous/Transfer Fcn', [sensor_path '/AHRS_Filter']);
    set_param([sensor_path '/AHRS_Filter'], ...
        'Numerator', '[1]', 'Denominator', '[0.02 1]', ...
        'Position', [150 25 250 55]);
    add_line(sensor_path, 'pitch_true/1', 'AHRS_Filter/1');

    % Gyro filter (low-pass on rate) - 1st order, tau=0.01s
    add_block('simulink/Continuous/Transfer Fcn', [sensor_path '/Gyro_Filter']);
    set_param([sensor_path '/Gyro_Filter'], ...
        'Numerator', '[1]', 'Denominator', '[0.01 1]', ...
        'Position', [150 75 250 105]);
    add_line(sensor_path, 'rate_true/1', 'Gyro_Filter/1');

    % Baro filter (low-pass on altitude) - 1st order, tau=0.05s
    add_block('simulink/Continuous/Transfer Fcn', [sensor_path '/Baro_Filter']);
    set_param([sensor_path '/Baro_Filter'], ...
        'Numerator', '[1]', 'Denominator', '[0.05 1]', ...
        'Position', [150 125 250 155]);
    add_line(sensor_path, 'alt_true/1', 'Baro_Filter/1');

    % Outputs
    add_block('simulink/Ports & Subsystems/Out1', [sensor_path '/pitch_meas']);
    set_param([sensor_path '/pitch_meas'], 'Position', [320 35 350 55]);
    add_line(sensor_path, 'AHRS_Filter/1', 'pitch_meas/1');

    add_block('simulink/Ports & Subsystems/Out1', [sensor_path '/rate_meas']);
    set_param([sensor_path '/rate_meas'], 'Position', [320 85 350 105]);
    add_line(sensor_path, 'Gyro_Filter/1', 'rate_meas/1');

    add_block('simulink/Ports & Subsystems/Out1', [sensor_path '/alt_meas']);
    set_param([sensor_path '/alt_meas'], 'Position', [320 135 350 155]);
    add_line(sensor_path, 'Baro_Filter/1', 'alt_meas/1');

    %% ================================================================
    %  SECTION 8: SCOPES / LOGGING
    %  ================================================================
    fprintf('  Building scopes...\n');

    % Pitch angle scope
    add_block('simulink/Sinks/Scope', [modelName '/Pitch_Scope']);
    set_param([modelName '/Pitch_Scope'], ...
        'Position', [1450 50 1510 90], 'NumInputPorts', '2');

    % Elevator scope
    add_block('simulink/Sinks/Scope', [modelName '/Elevator_Scope']);
    set_param([modelName '/Elevator_Scope'], ...
        'Position', [1450 180 1510 220]);

    % Altitude scope
    add_block('simulink/Sinks/Scope', [modelName '/Altitude_Scope']);
    set_param([modelName '/Altitude_Scope'], ...
        'Position', [1450 300 1510 340], 'NumInputPorts', '2');

    % Rate scope
    add_block('simulink/Sinks/Scope', [modelName '/Rate_Scope']);
    set_param([modelName '/Rate_Scope'], ...
        'Position', [1450 400 1510 440], 'NumInputPorts', '2');

    %% ================================================================
    %  SECTION 9: AIRSPEED & ROLL (constant for pitch-axis test)
    %  ================================================================
    add_block('simulink/Sources/Constant', ...
        [modelName '/Airspeed_Const'], ...
        'Value', '15.0', ...
        'Position', [580 320 640 350]);

    add_block('simulink/Sources/Constant', ...
        [modelName '/Roll_Const'], ...
        'Value', '0.0', ...
        'Position', [580 370 640 400]);

    %% ================================================================
    %  SECTION 10: WIRE EVERYTHING TOGETHER
    %  ================================================================
    fprintf('  Wiring top-level connections...\n');

    % --- Setpoints -> TECS ---
    add_line(modelName, 'Alt_Setpoint/1', 'TECS/1');
    add_line(modelName, 'Airspeed_Setpoint/1', 'TECS/3');

    % --- TECS output -> Pitch SP Switch ---
    add_line(modelName, 'TECS/1', 'Pitch_SP_Switch/1');          % TECS pitch_sp (top)
    add_line(modelName, 'Mode_Select/1', 'Pitch_SP_Switch/2');   % Mode
    add_line(modelName, 'Pitch_SP_Manual/1', 'Pitch_SP_Switch/3'); % Manual (bottom)

    % --- Pitch SP Switch -> Attitude Controller ---
    add_line(modelName, 'Pitch_SP_Switch/1', 'Pitch_Attitude_Ctrl/1');

    % --- Airspeed/Roll -> Attitude Controller ---
    add_line(modelName, 'Roll_Const/1', 'Pitch_Attitude_Ctrl/3');
    add_line(modelName, 'Airspeed_Const/1', 'Pitch_Attitude_Ctrl/4');

    % --- Attitude -> Rate Controller ---
    add_line(modelName, 'Pitch_Attitude_Ctrl/1', 'Pitch_Rate_Ctrl/1');

    % --- Rate Controller -> Plant ---
    add_line(modelName, 'Pitch_Rate_Ctrl/1', 'Aircraft_Plant/1');

    % --- TECS throttle -> Plant ---
    add_line(modelName, 'TECS/2', 'Aircraft_Plant/2');

    % --- Plant -> Sensors ---
    add_line(modelName, 'Aircraft_Plant/1', 'Sensors/1');   % pitch
    add_line(modelName, 'Aircraft_Plant/2', 'Sensors/2');   % pitch_rate
    add_line(modelName, 'Aircraft_Plant/3', 'Sensors/3');   % altitude

    % --- Sensors feedback -> Controllers ---
    add_line(modelName, 'Sensors/1', 'Pitch_Attitude_Ctrl/2');  % pitch_meas
    add_line(modelName, 'Sensors/2', 'Pitch_Rate_Ctrl/2');      % rate_meas
    add_line(modelName, 'Sensors/3', 'TECS/2');                 % alt_meas

    % Airspeed feedback to TECS (constant for now)
    add_line(modelName, 'Airspeed_Const/1', 'TECS/4');

    % --- Scopes ---
    % Pitch: pitch_sp vs pitch_meas
    add_line(modelName, 'Pitch_SP_Switch/1', 'Pitch_Scope/1');
    add_line(modelName, 'Sensors/1', 'Pitch_Scope/2');

    % Elevator
    add_line(modelName, 'Pitch_Rate_Ctrl/1', 'Elevator_Scope/1');

    % Altitude: alt_sp vs alt_meas
    add_line(modelName, 'Alt_Setpoint/1', 'Altitude_Scope/1');
    add_line(modelName, 'Sensors/3', 'Altitude_Scope/2');

    % Rate: rate_sp vs rate_meas
    add_line(modelName, 'Pitch_Attitude_Ctrl/1', 'Rate_Scope/1');
    add_line(modelName, 'Sensors/2', 'Rate_Scope/2');

    %% ================================================================
    %  SECTION 11: ANNOTATIONS
    %  ================================================================
    fprintf('  Adding annotations...\n');

    add_block('simulink/Commonly Used Blocks/DocBlock', ...
        [modelName '/Architecture_Note']);
    set_param([modelName '/Architecture_Note'], ...
        'Position', [50 430 350 550], ...
        'DocumentType', 'Text');

    %% ================================================================
    %  SAVE
    %  ================================================================
    fprintf('  Saving model: %s.slx\n', modelName);
    save_system(modelName);

    fprintf('\n');
    fprintf('  =====================================================\n');
    fprintf('  PX4 Fixed-Wing Pitch Architecture Model Built!\n');
    fprintf('  =====================================================\n');
    fprintf('  Model: %s.slx\n', modelName);
    fprintf('\n');
    fprintf('  BLOCKS:\n');
    fprintf('    [ORANGE]  TECS          - altitude/airspeed -> pitch_sp\n');
    fprintf('    [BLUE]    Attitude Ctrl - pitch error -> rate_sp (P + turn comp)\n');
    fprintf('    [GREEN]   Rate Ctrl     - rate error -> elevator (PID)\n');
    fprintf('    [RED]     Aircraft      - pitch-unstable plant dynamics\n');
    fprintf('    [GRAY]    Sensors       - AHRS, gyro, baro filters\n');
    fprintf('\n');
    fprintf('  PX4 PARAMETER MAPPING:\n');
    fprintf('    FW_PR_P   = 12.0   (pitch attitude P gain)\n');
    fprintf('    FW_PR_I   = 0.3    (pitch rate I gain)\n');
    fprintf('    FW_RR_P   = 0.4    (pitch rate P gain)\n');
    fprintf('    FW_RR_D   = 0.015  (pitch rate D gain)\n');
    fprintf('    FW_T_CLMB = 5.0    (TECS max climb rate)\n');
    fprintf('    FW_T_SINK = 3.0    (TECS max sink rate)\n');
    fprintf('\n');
    fprintf('  To simulate:  sim(''%s'')\n', modelName);
    fprintf('  =====================================================\n');
end
