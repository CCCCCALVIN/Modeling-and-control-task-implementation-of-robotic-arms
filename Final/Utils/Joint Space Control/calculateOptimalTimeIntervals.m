function adjusted_times = calculateOptimalTimeIntervals(key_positions, time_points, velocity_limits, coefficient)
% Calculate optimal time intervals for a trajectory based on joint velocity limits.
%
% This function adjusts the time intervals between key trajectory points to ensure
% that the joint velocities do not exceed their specified limits, scaled by a 
% customizable coefficient.
%
% INPUT:
%   key_positions  - A NxM matrix where N is the number of key positions
%                    and M is the number of joints. Each row represents
%                    the joint positions (in radians) at a specific time.
%   time_points     - A 1xN vector of time points corresponding to the key positions.
%                    These are the initial time intervals.
%   velocity_limits - A 1xM vector representing the maximum velocity limits (radians/s)
%                    for each joint.
%   coefficient     - A scaling factor applied to the calculated time intervals.
%                    This allows for adjustment of the time intervals based on
%                    application-specific considerations (e.g., safety margins or
%                    dynamic limitations). A typical value is 1 for unscaled limits.
%
% OUTPUT:
%   adjusted_times  - A 1xN vector of adjusted time points that satisfy
%                    the joint velocity limits.
%
% USAGE:
%   adjusted_times = calculateOptimalTimeIntervals(key_positions, time_points, velocity_limits, coefficient);

    fprintf('[LOG INFO] -- Calculating Optimal Time Intervals...\n');
    
    % Initialization
    n_joints = size(key_positions, 2); % Number of joints
    n_segments = size(key_positions, 1) - 1; % Number of trajectory segments
    adjusted_times = time_points; % Initialize adjusted time points

    % Iterate through each trajectory segment to calculate the minimum required time
    for i = 1:n_segments
        max_time_required = 0; % Track the maximum time required for the segment
        for j = 1:n_joints
            % Calculate the joint angle difference
            delta_q = abs(key_positions(i+1, j) - key_positions(i, j));
            % Calculate the minimum time required based on velocity limits and coefficient
            time_required = coefficient * delta_q / velocity_limits(j);
            % Update the maximum time required for the current segment
            max_time_required = max(max_time_required, time_required);
        end
        % Update the next time point based on the maximum required time
        adjusted_times(i+1) = adjusted_times(i) + max_time_required;
    end

    % Log results
    fprintf('[LOG INFO] -- Optimal Time Intervals Calculated\n');
    % Convert adjusted_times to a formatted string with two decimal places
    formatted_times = sprintf('%.2f, ', adjusted_times); % Format each value with 2 decimal places
    formatted_times = formatted_times(1:end-2); % Remove the trailing comma and space

    % Print the formatted string
    fprintf('[LOG INFO] -- Adjusted Time Points: [%s] \n', formatted_times);
end
