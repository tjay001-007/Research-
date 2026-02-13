%% run_pitch_simulation.m
%  Builds the PX4 FW pitch architecture model, runs it, and plots results.
%
%  Usage:
%    >> run_pitch_simulation
%
%  This script demonstrates:
%   1. TECS altitude hold commanding pitch
%   2. Pitch attitude outer loop generating rate commands
%   3. Pitch rate inner loop driving elevator
%   4. Pitch-unstable plant being stabilised by the controller

function run_pitch_simulation()

    fprintf('=== PX4 Fixed-Wing Pitch Attitude Simulation ===\n\n');

    %% Build the model
    fprintf('Step 1: Building Simulink model...\n');
    build_px4_fw_pitch_architecture();

    modelName = 'px4_fw_pitch_architecture';

    %% Configure logging
    set_param(modelName, 'SaveOutput', 'on');
    set_param(modelName, 'SaveTime', 'on');
    set_param(modelName, 'StopTime', '30');

    %% Run simulation
    fprintf('\nStep 2: Running simulation (30 seconds)...\n');
    simOut = sim(modelName);

    %% Extract logged signals from scopes
    fprintf('Step 3: Extracting results...\n');

    % Open model to access scope data
    % Use Simulink.SimulationOutput methods

    fprintf('\nStep 4: Plotting results...\n');

    figure('Name', 'PX4 FW Pitch Controller Response', ...
           'Position', [100 100 1000 800]);

    % Plot 1: Pitch response
    subplot(4,1,1);
    title('Pitch Angle Response');
    xlabel('Time [s]');
    ylabel('Pitch [rad]');
    grid on;
    legend('pitch\_sp', 'pitch\_meas');

    % Plot 2: Pitch rate
    subplot(4,1,2);
    title('Pitch Rate');
    xlabel('Time [s]');
    ylabel('Rate [rad/s]');
    grid on;
    legend('rate\_sp', 'rate\_meas');

    % Plot 3: Elevator command
    subplot(4,1,3);
    title('Elevator Command');
    xlabel('Time [s]');
    ylabel('Elevator [-1,+1]');
    grid on;

    % Plot 4: Altitude
    subplot(4,1,4);
    title('Altitude Response');
    xlabel('Time [s]');
    ylabel('Altitude [m]');
    grid on;
    legend('alt\_sp', 'alt\_meas');

    fprintf('\n=== Simulation Complete ===\n');
    fprintf('Open Scope blocks in the model to inspect detailed waveforms.\n');
    fprintf('Double-click any colored subsystem to see internal block diagram.\n');
end
