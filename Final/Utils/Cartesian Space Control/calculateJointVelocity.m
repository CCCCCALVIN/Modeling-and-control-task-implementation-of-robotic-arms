function joint_velocity = calculateJointVelocity(joint_trajectory, time_samples)
% Calculate joint velocities using numerical differentiation with varying time intervals.
%
% Parameters:
%   joint_trajectory - NxM matrix of joint angles (N: time steps, M: joints).
%   time_samples     - 1xN vector of time samples corresponding to joint angles.
%
% Returns:
%   joint_velocity   - (N-1)xM matrix of joint velocities.

    % Get number of time steps and joints
    num_steps = size(joint_trajectory, 1);
    num_joints = size(joint_trajectory, 2);

    % Initialize velocity matrix
    joint_velocity = zeros(num_steps - 1, num_joints);

    % Compute velocities using central differences
    for i = 2:num_steps-1
        dt_prev = time_samples(i) - time_samples(i-1); % Time interval before
        dt_next = time_samples(i+1) - time_samples(i); % Time interval after

        joint_velocity(i, :) = (joint_trajectory(i+1, :) - joint_trajectory(i-1, :)) / (dt_prev + dt_next);
    end

    % Forward difference for the first step
    joint_velocity(1, :) = (joint_trajectory(2, :) - joint_trajectory(1, :)) / (time_samples(2) - time_samples(1));

    % Backward difference for the last step
    joint_velocity(end, :) = (joint_trajectory(end, :) - joint_trajectory(end-1, :)) / ...
                             (time_samples(end) - time_samples(end-1));
end


