function joint_trajectory = inverseKinematiceCoppelia(trajectory, time_trajectory, ...
                                                   joint_angles, ...
                                                   home_position, ...
                                                   pos_coppelia, rot_coppelia, ...
                                                   jacobian_translation, jacobian_orientation, ...
                                                   joint_limits, velocity_limits, ...
                                                   rot_sagittal, ...
                                                   max_iterations, tolerance, damping_factor, speed_safty_coeficient)
% Compute the joint trajectory using Newton-Raphson inverse kinematics.
%
% This function calculates the joint angles required to follow a given Cartesian
% trajectory using the Newton-Raphson method with damping.
%
% Parameters:
%   trajectory           - Nx3 matrix of Cartesian waypoints (desired positions).
%   time_trajectory      - 1xN vector of time samples for each waypoint.
%   home_position        - 1xM vector of initial joint angles (home configuration).
%   pos_coppelia         - Symbolic expression for end-effector position.
%   rot_coppelia         - Symbolic expression for end-effector Z-axis orientation.
%   jacobian_translation - Symbolic Jacobian for translation.
%   jacobian_orientation - Symbolic Jacobian for orientation.
%   joint_limits         - Mx2 matrix of joint angle limits [min, max] (radians).
%   velocity_limits      - 1xM vector of joint velocity limits (radians/second).
%   rot_sagittal         - Desired Z-axis direction for the end-effector.
%   max_iterations       - Maximum number of iterations for Newton-Raphson (integer).
%   tolerance            - Convergence tolerance for error (float).
%   damping_factor       - Damping factor for Jacobian pseudo-inverse (float).
%   speed_safety_coefficient - Scaling factor for dynamic step limits. The
%   small one provide more safty.
%
% Returns:
%   joint_trajectory     - NxM matrix of joint angles for each time step.

    % Initialization
    num_steps = size(trajectory, 1); % Total number of trajectory points
    num_joints = length(home_position); % Number of joints
    joint_trajectory = zeros(num_steps, num_joints); % Initialize joint trajectory
    joint_trajectory(1, :) = home_position; % Set home position as the initial step

    % Iterate through each trajectory point
    for step_idx = 2:num_steps
        % Calculate the time step
        time_step = time_trajectory(step_idx) - time_trajectory(step_idx - 1);

        % Scale step limit based on velocity limits and time step
        dynamic_step_limit = speed_safty_coeficient * velocity_limits * time_step;

        % Target Cartesian position
        target_position = trajectory(step_idx, :)';

        % Initial guess for joint angles (use previous step's solution)
        current_joint_angles = joint_trajectory(step_idx - 1, :);
        cumulative_update = zeros(size(current_joint_angles));

        % Newton-Raphson iterative solution
        for iteration_idx = 1:max_iterations
            % Compute current end-effector position and Z-axis direction
            current_position = double(subs(pos_coppelia, joint_angles, current_joint_angles));
            current_z_axis = double(subs(rot_coppelia, joint_angles, current_joint_angles));

            % Compute errors
            position_error = target_position - current_position;
            z_axis_error = rot_sagittal - current_z_axis;

            combined_error = [position_error; z_axis_error];

            % Compute Jacobian and pseudo-inverse
            jacobian_combined = [
                double(subs(jacobian_translation, joint_angles, current_joint_angles));
                double(subs(jacobian_orientation, joint_angles, current_joint_angles))
            ];

            [U, S, V] = svd(jacobian_combined); % Singular value decomposition
            S_inv = diag(1 ./ (diag(S) + damping_factor)); % Add damping factor
            jacobian_pseudo_inverse = V * S_inv * U';
            
            % Update joint angles
            joint_angle_update = jacobian_pseudo_inverse * combined_error;

            cumulative_update = cumulative_update + joint_angle_update';

            % Apply dynamic step limit
            if any(abs(cumulative_update) > dynamic_step_limit)
                fprintf('[LOG INFO] -- Step %03d - Iteration %03d: Cumulative Update Exceeded Threshold.\n', step_idx, iteration_idx);
                 % Truncate cumulative update to the dynamic step limit
                cumulative_update = max(min(cumulative_update, dynamic_step_limit), -dynamic_step_limit);
                current_joint_angles = joint_trajectory(step_idx - 1, :) + cumulative_update; % Apply cumulative update
                current_joint_angles = mod(current_joint_angles + pi, 2 * pi) - pi; % Normalize angles to [-pi, pi]
                % Check if joint angles exceed limits
                if any(current_joint_angles' < joint_limits(:, 1)) || any(current_joint_angles' > joint_limits(:, 2))
                    fprintf('[WARNNING] -- Step %03d - Iteration %03d: Joint Limits Exceeded.\n', step_idx, iteration_idx);
                    current_joint_angles = max(min(current_joint_angles', joint_limits(:, 2)), joint_limits(:, 1)); % Clamp angles to limits
                end
                break;
            end

            % Update joint angles and normalize to [-pi, pi]
            current_joint_angles = current_joint_angles + joint_angle_update';
            current_joint_angles = mod(current_joint_angles + pi, 2 * pi) - pi;

            % Check joint limits
            if any(current_joint_angles < joint_limits(:, 1)' | current_joint_angles > joint_limits(:, 2)')
                fprintf('[WARNNING] -- Step %03d - Iteration %03d: Joint Limits Exceeded.\n', step_idx, iteration_idx);
                current_joint_angles = max(min(current_joint_angles, joint_limits(:, 2)'), joint_limits(:, 1)');
                break;
            end

            % Check for convergence
            if norm(combined_error) < tolerance
                fprintf("[LOG INFO] -- Step %03d - Iteration %03d: Coveraged Successfully.\n", step_idx, iteration_idx);
                break;
            end

            % Check if maximum iterations are reached
            if iteration_idx == max_iterations
                fprintf('[WARNNING] -- Step %03d: Newton-Raphson did not converge.\n', step_idx);
            end
        end

        % Save the joint angles for the current step
        joint_trajectory(step_idx, :) = current_joint_angles;
    end
end


