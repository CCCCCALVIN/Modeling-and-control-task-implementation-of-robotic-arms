function flag = validateAngles(angles, joint_limits)
% Validate joint angles against specified joint limits.
%
% This function checks whether a set of joint angles lies within
% the given joint limits. If any angle is out of bounds, the function
% prints a warning message and returns a flag indicating failure.
%
% INPUT:
%   angles       - A NxM matrix where N is the number of configurations
%                  and M is the number of joints. Each row represents a
%                  set of joint angles (in radians).
%   joint_limits - A Mx2 matrix where M is the number of joints.
%                  Each row specifies the [lower_limit, upper_limit] (in radians)
%                  for the corresponding joint.
%
% OUTPUT:
%   flag         - A boolean value. Returns true if all angles are within limits,
%                  false otherwise.
%
% USAGE:
%   flag = validateAngles(angles, joint_limits);

    fprintf('[LOG INFO] -- Validate Angles...\n');
    flag = true;
    for i = 1:size(angles, 1)
        if any(angles(i,:)'<joint_limits(:,1)) || any(angles(i,:)'> joint_limits(:,2))
            flag = false;
            fprintf('[WARNNING] -- Configuration %d is out of bounds.\n', i);
            fprintf('Angles: %s\n', mat2str(angles(i, :)));
        end
    end

    if flag
        fprintf('[LOG INFO] -- All joint angles are within limits.\n');
    else
        fprintf('[LOG INFO] -- Joint angle validation failed.\n');
    end
end

