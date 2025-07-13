function showJointAccelerationInTrajectory(accelerations, time_samples)
% Show joint acceleration trajectories in a trajectory.
%
% This function visualizes the joint acceleration trajectories over time.
% Unlike angle and velocity plots, acceleration trajectories do not include
% upper or lower limits.
%
% INPUTS:
%   time_samples  - A 1xT vector of time sample points (in seconds).
%   accelerations - A TxN matrix of joint accelerations (in radians/second^2),
%                   where T is the number of time samples and N is the number of joints.
%
% OUTPUTS:
%   None. This function generates a figure with joint acceleration plots.
%
% USAGE:
%   showJointAccelerationInTrajectory(time_samples, accelerations);

    % Convert accelerations to degrees/second^2
    accelerations_deg = rad2deg(accelerations); % Convert accelerations from radians/second^2 to degrees/second^2

    % Get the number of joints
    n_joints = size(accelerations, 2);

    % Set distinct colors for each joint trajectory
    colors = lines(n_joints); 

    % Create the figure for plots
    figure;
    for joint_idx = 1:n_joints
        % Create subplots based on the number of joints
        subplot(ceil(n_joints / 2), 2, joint_idx); 
        % Plot the joint acceleration trajectory
        plot(time_samples, accelerations_deg(:, joint_idx), 'LineWidth', 1.5, 'Color', colors(joint_idx, :)); 

        % Configure chart settings
        title(['Joint ', num2str(joint_idx), ' Acceleration']);
        xlabel('Time (s)');
        ylabel('Acceleration (deg/s^2)');
        grid on;
        legend({'Acceleration'}, 'Location', 'Best');
    end

    % Add a shared title for the entire figure
    sgtitle('Joint Acceleration Trajectories'); 
end

