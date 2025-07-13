function random_joint_angles = generateRandomJointAngles(joint_limits)
% Generate random joint angles within specified limits.
%
% INPUT:
%   joint_limits - A Nx2 matrix where N is the number of joints.
%                  Each row represents the [lower_limit, upper_limit] (in radians)
%                  for the corresponding joint.
%
% OUTPUT:
%   random_joint_angles - A Nx1 vector of randomly generated joint angles,
%                         where each angle is sampled uniformly within its
%                         corresponding joint limits.
%
% USAGE:
%   random_joint_angles = generateRandomJointAngles(joint_limits);

    % Number of joints
    num_joints = size(joint_limits, 1);

    % Initialize random joint angles vector
    random_joint_angles = zeros(num_joints, 1);

    % Generate random angles for each joint
    for i = 1:num_joints
        % Randomly sample a value within the range of each joint
        random_joint_angles(i) = joint_limits(i, 1) + ...
            (joint_limits(i, 2) - joint_limits(i, 1)) * rand;
    end
    random_joint_angles = random_joint_angles';
end


