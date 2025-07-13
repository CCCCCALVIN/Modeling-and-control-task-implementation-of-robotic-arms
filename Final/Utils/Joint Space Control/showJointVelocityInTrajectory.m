function showJointVelocityInTrajectory(velocities, velocity_limits, time_samples)
% Show joint velocity trajectories with limits in a trajectory.
%
% This function visualizes the joint velocity trajectories over time, with
% upper and lower limits displayed as reference.
%
% INPUTS:
%   time_samples   - A 1xT vector of time sample points (in seconds).
%   velocities     - A TxN matrix of joint velocities (in radians/second), where T is the
%                    number of time samples and N is the number of joints.
%   velocity_limits - An Nx2 matrix of velocity limits (in radians/second), where N is the
%                    number of joints. Each row specifies the [lower_limit, upper_limit] for a joint.
%
% OUTPUTS:
%   None. This function generates a figure with joint velocity plots.
%
% USAGE:
%   showJointVelocityInTrajectory(time_samples, velocities, velocity_limits);

    % Convert velocities and limits to degrees/second
    velocities_deg = rad2deg(velocities); % Convert joint velocities from radians/second to degrees/second
    velocity_limits_deg = rad2deg(velocity_limits); % Convert velocity limits from radians/second to degrees/second

    % Get the number of joints
    n_joints = size(velocities, 2);

    % Set distinct colors for each joint trajectory
    colors = lines(n_joints); 

    % Create the figure for plots
    figure;
    for joint_idx = 1:n_joints
        % Create subplots based on the number of joints
        subplot(ceil(n_joints / 2), 2, joint_idx); 
        % Plot the joint velocity trajectory
        plot(time_samples, velocities_deg(:, joint_idx), 'LineWidth', 1.5, 'Color', colors(joint_idx, :)); 
        hold on;

        % Add velocity limit lines (red dashed lines for upper and lower limits)
        yline(- velocity_limits_deg(joint_idx), 'r--', 'LineWidth', 1.2, 'Label', 'Lower Limit'); % Lower limit
        yline(+ velocity_limits_deg(joint_idx), 'r--', 'LineWidth', 1.2, 'Label', 'Upper Limit'); % Upper limit

        % Configure chart settings
        title(['Joint ', num2str(joint_idx), ' Velocity']);
        xlabel('Time (s)');
        ylabel('Velocity (deg/s)');
        grid on;
        legend({'Velocity', 'Lower Limit', 'Upper Limit'}, 'Location', 'Best');
        hold off;
    end

    % Add a shared title for the entire figure
    sgtitle('Joint Velocity Trajectories with Limits'); 
end

