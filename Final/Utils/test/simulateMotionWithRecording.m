function [joint_positions, joint_velocities, tip_positions, time_stamps] = simulateMotionWithRecording(ik_solver, ...
    commands, weights, ...
    initial_guess_coppelia, ...
    offset, coppelia, ...
    joint_handles, ee_handle, ...
    cube_handles,...
    sample_rate)
% simulateMotionWithRecording - Compute joint angles for multiple poses, send to CoppeliaSim, and record data.
%
% INPUT:
%   ik_solver     - InverseKinematics solver object in MATLAB space.
%   commands      - Nx7 matrix of target poses [x y z roll pitch yaw gripper_Status] (mm and degrees, gripper_Status as on/off).
%   weights       - A 1x6 array of weights for the IK solver [x y z roll pitch yaw].
%   initial_guess_coppelia - Initial joint configuration in CoppeliaSim space (1x6).
%   offset        - Offset between MATLAB and CoppeliaSim configurations (1x6).
%   coppelia      - CoppeliaSim remote API client.
%   joint_handles - Handles for robot joints in CoppeliaSim.
%   ee_handle     - Handle for the end-effector in CoppeliaSim.
%   sample_rate   - Sampling rate for recording (seconds).
%
% OUTPUT:
%   joint_positions - Recorded joint positions (time_steps x 6).
%   joint_velocities - Recorded joint velocities (time_steps x 6).
%   tip_positions   - Recorded end-effector positions (time_steps x 3).
%   time_stamps     - Timestamps for each recorded sample.

    % Initialize storage
    joint_positions = [];
    joint_velocities = [];
    tip_positions = [];
    time_stamps = [];

    global_timer = tic; % Global timer for timestamps

    offset = offset';
    current_angles_coppelia = initial_guess_coppelia';

    fprintf('[LOG INFO] -- Starting Motion Simulation and Recording...\n');

    % Iterate through each command
    for command_idx = 1:size(commands, 1)
        fprintf('[LOG INFO] -- COMMAND %02d -- %s\n', command_idx, mat2str(commands(command_idx, :)));
        % Parse command
        t = commands(command_idx, 1:3) / 1000; % Convert mm to meters
        r = deg2rad(commands(command_idx, 4:6)); % Convert degrees to radians
        r = r(end:-1:1);
        target_pose = trvec2tform(t) * eul2tform(r);
        gripper_status = commands(command_idx, 7); % Gripper status

        % Solve IK in MATLAB space
        if(~size(current_angles_coppelia,1)==1)
            current_angles_coppelia = current_angles_coppelia';
        end

        current_angles_matlab = current_angles_coppelia - offset;
        [joint_angles_matlab, solution_info] = ik_solver('endeffector', target_pose, weights, current_angles_matlab);

        if ~strcmp(solution_info.Status, 'success')
            error('[WARNNING] -- IK Solver Failed for Command %02d', command_idx);
        end

        % Convert MATLAB joint angles to CoppeliaSim space
        joint_angles_coppelia = joint_angles_matlab + offset;

        % Move robot joints in CoppeliaSim
        for i = 1:length(joint_handles)
            coppelia.setJointTargetPosition(joint_handles(i), joint_angles_coppelia(i));
        end

        % Recording during motion
        % Record motion while the robot is moving
        motion_start_time = tic; % Start motion timer

        while toc(motion_start_time) < 1
            pause(sample_rate);

            % Record timestamp
            elapsed_time = toc(global_timer);
            time_stamps = [time_stamps; elapsed_time];

            % Record joint positions and velocities
            current_positions = zeros(1, length(joint_handles));
            current_velocities = zeros(1, length(joint_handles));
            for i = 1:length(joint_handles)
                joint_position = coppelia.getJointPosition(joint_handles(i));
                joint_velocity= coppelia.getObjectVelocity(joint_handles(i));

                current_positions(i) = joint_position;
                
                % Process velocity data
                vx = joint_velocity(1); vx = vx{1};
                vy = joint_velocity(2); vy = vy{1};
                vz = joint_velocity(3); vz = vz{1};
                v = [vx, vy,vz];
                current_velocities(i) = norm(v);
            end

            % Record end-effector position
            ee_position = cell2mat(coppelia.getObjectPosition(ee_handle, -1));

            % Append to storage
            joint_positions = [joint_positions; current_positions];
            joint_velocities = [joint_velocities; current_velocities];
            tip_positions = [tip_positions; ee_position];
            time_stamps = [time_stamps; elapsed_time];
        end

        % Set gripper status if applicable
        if gripper_status == 11
            fprintf('[LOG INFO] -- Gripper OPEN - Cube 1 Caught.\n');
            coppelia.setObjectParent(cube_handles(1), ee_handle, true);
        elseif gripper_status == 10
            fprintf('[LOG INFO] -- Gripper OFF - Cube 1 Dropped.\n');
            coppelia.setObjectParent(cube_handles(1), -1, true);
        elseif gripper_status == 21
            fprintf('[LOG INFO] -- Gripper OPEN - Cube 2 Caught.\n');
            coppelia.setObjectParent(cube_handles(2), ee_handle, true);
        elseif gripper_status == 20
            fprintf('[LOG INFO] -- Gripper OFF - Cube 2 Dropped.\n');
            coppelia.setObjectParent(cube_handles(2), -1, true);
        end
        
        % Update current joint angles
        current_angles_coppelia = joint_angles_coppelia;
        disp('----');
    end

    fprintf('[LOG INFO] -- Simulation and Recording Complete.\n');
end
