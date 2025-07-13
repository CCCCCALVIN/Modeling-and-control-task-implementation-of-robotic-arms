function [trajectory, time_samples, total_steps, key_indices] = generateLinearTrajectory(waypoints, segment_time, segment_steps, key_indices_idxes)
% Generate a linear trajectory in Cartesian space between waypoints.
%
% This function creates a piecewise linear trajectory between a series of
% waypoints in 3D space. The trajectory is evenly spaced and parameterized
% by time and the number of steps per segment.
%
% Parameters:
%   waypoints      - A Nx3 matrix where each row is a waypoint [x, y, z].
%   segment_time   - Time (in seconds) for each segment of the trajectory.
%   segment_steps  - Number of discrete steps per segment.
%   key_indices_idxes - indexes of the Key Point in Waypoints
%
% Returns:
%   trajectory     - A Mx3 matrix where M = total_steps, representing the
%                    interpolated positions along the trajectory.
%   time_samples   - A 1xM vector of time samples corresponding to each step.
%   total_steps    - Total number of steps for the entire trajectory.
%   key_indices    - The indices in trajectory for key points;
% Example:
%   waypoints = [0, 0, 0; 1, 1, 1; 2, 2, 2];
%   [trajectory, time_samples, total_steps] = generateLinearTrajectory(waypoints, 5, 50);

    % Number of segments in the trajectory
    num_segments = size(waypoints, 1) - 1;

    % Total number of trajectory steps
    total_steps = num_segments * segment_steps;
    
    key_indices = zeros(1, numel(key_indices_idxes)); % Preallocate for key indices in trajectory
    for i = 1:numel(key_indices_idxes)
        key_indices(i) = round(segment_steps * (key_indices_idxes(i) - 1)); % Map key waypoints to trajectory indices
    end

    % Initialize trajectory and time samples
    trajectory = zeros(total_steps, 3); % 3D trajectory points
    time_samples = linspace(0, num_segments * segment_time, total_steps); % Time samples

    % Generate the trajectory for each segment
    for i = 1:num_segments
        % Extract start and end waypoints for the current segment
        start_pos = waypoints(i, :);
        end_pos = waypoints(i + 1, :);

        % Generate interpolation points for the segment
        t = linspace(0, 1, segment_steps).'; % Normalized time within segment
        pos_segment = start_pos + t .* (end_pos - start_pos); % Linear interpolation

        % Assign interpolated points to the overall trajectory
        start_idx = (i - 1) * segment_steps + 1;
        end_idx = i * segment_steps;
        trajectory(start_idx:end_idx, :) = pos_segment;
    end
end
