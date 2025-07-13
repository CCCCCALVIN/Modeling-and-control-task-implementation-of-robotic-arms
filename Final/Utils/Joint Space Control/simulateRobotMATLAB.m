function simulateRobotMATLAB(robot, q, k, type, frame_skip, draw_path)
% Simulate and visualize robot motion in MATLAB with different modes.
%
% Parameters:
%   robot      - rigidBodyTree object representing the robot model.
%   q          - Joint trajectories (size: [n_samples, n_joints]).
%   k          - Key Points (size: [n_keys, n_joints])
%   type       - Visualization type ('endeffector', 'frame', 'collision'):
%                'frame'       - Show the entire robot with frames.
%                'collision'   - Show the entire robot's collision model.
%   frame_skip - Number of frames to skip between animation frames (positive integer, e.g., 5 or 10).
%   draw_path  - Boolean flag (true/false) to enable/disable end-effector trajectory visualization.
%
% Usage:
%   simulateRobotMATLAB(robot, q, k, 'frame', 10, false);
%   simulateRobotMATLAB(robot, q, k, 'collision', 10, true);

    % Validate inputs
    assert(~isempty(robot) && ~isempty(q), '[ERROR] Robot model or trajectory is empty.');
    assert(frame_skip > 0, '[ERROR] Frame skip must be a positive integer.');
    assert(ismember(type, {'endeffector', 'frame', 'collision'}), ...
        '[ERROR] Invalid type. Use "endeffector", "frame", or "collision".');

    % Configure the figure
    figure;
    hold on;
    xlim([-0.5, 0.5]);
    ylim([-0.5, 0.5]);
    zlim([0, 0.5]);
    grid on;
    view([45, 30]); % Set fixed view angle
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    % Initialize end-effector trajectory storage
    ee_positions = []; % Empty array to store end-effector positions

    % Animation loop
    for i = 1:frame_skip:size(q, 1)
        if strcmp(type, 'frame')
            % Show robot with frames
            show(robot, q(i, :)', "PreservePlot", false, "Frames", "on", "Collisions", "off");
            title('Robot Motion with Frames');

        elseif strcmp(type, 'collision')
            % Show robot with collision models
            show(robot, q(i, :)', "PreservePlot", false, "Frames", "off", "Collisions", "on");
            title('Robot Motion with Collisions');
        end

        % Store and plot end-effector trajectory if draw_path is true
        if draw_path
            transform = getTransform(robot, q(i, :)', 'endeffector');
            ee_position = transform(1:3, 4)'; % Extract the position
            ee_positions = [ee_positions; ee_position]; % Append position to trajectory

            % Scatter all trajectory points
            scatter3(ee_positions(:, 1), ee_positions(:, 2), ee_positions(:, 3), ...
                3, 'r', 'filled'); % Red dots for trajectory
            if k == -1
                legend('Trajectory');
            else
                scatter3(k(:, 1), k(:, 2), k(:, 3), 'bo', 'filled');
                legend('Trajectory', 'Waypoints');
            end
            
        end

        drawnow;
    end

    fprintf('[LOG INFO] -- Simulation complete.\n');
end