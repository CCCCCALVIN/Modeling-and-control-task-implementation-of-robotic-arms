function showJointAngleInTrajectory(angles, joint_limits, time_samples)
% Show joint angle trajectories with limits in a trajectory.
%
% This function visualizes the joint angle trajectories over time, with
% upper and lower limits displayed as reference.
%
% INPUTS:
%   time_samples - A 1xT vector of time sample points (in seconds).
%   angles       - A TxN matrix of joint angles (in radians), where T is the number of time samples
%                  and N is the number of joints.
%   joint_limits - An Nx2 matrix of joint limits (in radians), where N is the number of joints.
%                  Each row specifies the [lower_limit, upper_limit] for a joint.
%
% OUTPUTS:
%   None. This function generates a figure with joint angle plots.
%
% USAGE:
%   showJointAngleInTrajectory(time_samples, angles, joint_limits);

    % Convert joint angles and limits to degrees
    angles_deg = rad2deg(angles); % Convert joint angles from radians to degrees
    joint_limits_deg = rad2deg(joint_limits); % Convert joint limits from radians to degrees

    % Get the number of joints
    n_joints = size(angles, 2);

    % Set distinct colors for each joint trajectory
    colors = lines(n_joints); 

    % Create the figure for plots
    figure;
    for j = 1:n_joints
        % Create subplots based on the number of joints
        subplot(ceil(n_joints / 2), 2, j); 
        % Plot the joint angle trajectory
        plot(time_samples, angles_deg(:, j), 'LineWidth', 1.5, 'Color', colors(j, :)); 
        hold on;

        % Add limit lines (red dashed lines for upper and lower limits)
        yline(joint_limits_deg(j, 1), 'r--', 'LineWidth', 1.2, 'Label', 'Lower Limit'); % Lower limit
        yline(joint_limits_deg(j, 2), 'r--', 'LineWidth', 1.2, 'Label', 'Upper Limit'); % Upper limit

        % Configure chart settings
        title(['Joint ', num2str(j), ' Angle']);
        xlabel('Time (s)');
        ylabel('Angle (deg)');
        grid on;
        legend({'Angle', 'Lower Limit', 'Upper Limit'}, 'Location', 'Best');
        hold off;
    end

    % Add a shared title for the entire figure
    sgtitle('Joint Angle Trajectories with Limits'); 
end


