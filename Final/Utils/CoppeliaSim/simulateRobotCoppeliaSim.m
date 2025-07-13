function ee_positions_coppelia = simulateRobotCoppeliaSim(q, time_samples, simClient, jointNames, eeName)
% Simulate a robot trajectory in CoppeliaSim and record the end-effector positions.
%
% Parameters:
%   q             - Joint trajectories (size: [n_samples, n_joints]).
%   time_samples  - Time samples for each trajectory step (size: [n_samples, 1]).
%   simClient     - RemoteAPIClient object connected to CoppeliaSim.
%   jointNames    - Cell array of joint names in CoppeliaSim (e.g., {'/Mirobot/joint1', ...}).
%   eeName        - Name of the end-effector object in CoppeliaSim (e.g., '/Mirobot/Tip').
%
% Returns:
%   ee_positions_coppelia - End-effector positions recorded during trajectory execution (size: [n_samples, 3]).

    fprintf('[LOG INFO] -- Connecting to CoppeliaSim...\n');
    coppelia = simClient.require('sim'); % Get the handle for CoppeliaSim API

    % Start simulation
    try
        coppelia.startSimulation();
        fprintf('[LOG INFO] -- Simulation started successfully.\n');
    catch
        error('[WARNING] -- Failed to start simulation. Ensure CoppeliaSim is running.');
    end

    % Get handles for joints and end-effector
    joint_handles = zeros(length(jointNames), 1);
    for i = 1:length(jointNames)
        joint_handles(i) = coppelia.getObject(jointNames{i});
    end
    end_effector_handle = coppelia.getObject(eeName);

    % Initialize storage for end-effector positions
    ee_positions_coppelia = zeros(size(q, 1), 3);

    % Execute the trajectory and record end-effector positions
    fprintf('[LOG INFO] -- Executing trajectory and recording end-effector positions...\n');
    for step = 1:size(q, 1)
        % Set target position for each joint
        for j = 1:length(joint_handles)
            coppelia.setJointTargetPosition(joint_handles(j), q(step, j));
        end

        % Retrieve the end-effector position
        position = cell2mat(coppelia.getObjectPosition(end_effector_handle, -1));
        ee_positions_coppelia(step, :) = position;

        % Synchronize with the planned time samples
        if step < size(q, 1)
            pause((time_samples(step+1) - time_samples(step))/20); % Coppelia dt = 50 ms
        end
    end

    % Stop the simulation
    coppelia.stopSimulation();
    fprintf('[LOG INFO] -- Simulation stopped.\n');
    fprintf('[LOG INFO] -- End-effector positions recorded.\n');
end
