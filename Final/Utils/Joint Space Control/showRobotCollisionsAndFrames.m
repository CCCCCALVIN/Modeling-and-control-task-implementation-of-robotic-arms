function showRobotCollisionsAndFrames(robot, angle_matlab)
% Function to visualize a robot in two views:
% 1. With collision models enabled (collisions only)
% 2. With coordinate frames enabled (frames only)
%
% INPUTS:
%   robot         - The rigidBodyTree object representing the robot model.
%   angle_matlab  - A 6x1 vector representing the joint angles of the robot in radians.
%                   These joint angles are used to set the robot's configuration.
%
% OUTPUTS:
%   This function does not return any output. It generates a figure with two subplots:
%   - Left subplot: Robot visualized with collision models.
%   - Right subplot: Robot visualized with coordinate frames.
%
% USAGE:
%   showRobotCollisionsAndFrames(robot, angle_matlab)

    % Create a new figure for the visualization
    figure(1);
    set(gcf, 'Color', 'w');
    sgtitle('Robot Visualization'); % Title for the entire figure

    % Subplot 1: Display robot with collision models
    subplot(1, 2, 1);
    show(robot, angle_matlab, "Collisions", "on", "Frames", "off"); % Enable collisions, disable frames
    title('Collisions Only'); % Title for this subplot
    xlim([-0.1 0.3]); % Set x-axis limits
    ylim([-0.2 0.2]); % Set y-axis limits
    zlim([0 0.4]);    % Set z-axis limits

    % Subplot 2: Display robot with coordinate frames
    subplot(1, 2, 2);
    show(robot, angle_matlab, "Collisions", "off", "Frames", "on"); % Disable collisions, enable frames
    title('Frames Only'); % Title for this subplot
    xlim([-0.1 0.3]); % Set x-axis limits
    ylim([-0.2 0.2]); % Set y-axis limits
    zlim([0 0.4]);    % Set z-axis limits
    
end


