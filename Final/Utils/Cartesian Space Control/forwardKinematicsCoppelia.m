function T_COPPELIA_FINAL = forwardKinematicsCoppelia(mdh, joint_angles)
% Calculate the forward kinematics for a robot using MDH parameters.
%
% This function computes the forward kinematics of a robot given its MDH parameters
% and joint angles. It returns the homogeneous transformation matrix from the base 
% to the end-effector.
%
% Parameters:
%   mdh       - Modified Denavit-Hartenberg (MDH) parameters (Nx4 matrix):
%                               [a_i, alpha_i, d_i, theta_offset_i].
%   joint_angles - A symbolic array of joint angles (e.g., [th1, th2, ..., thN]).
%
% Returns:
%   T_COPPELIA_FINAL - A 4x4 symbolic homogeneous transformation matrix representing 
%                      the pose of the end-effector relative to the base frame.

    % Initialize transformation matrix as identity
    T_COPPELIA = eye(4); 

    % Number of joints
    num_joints = length(joint_angles);

    % Compute forward kinematics
    for i = 1:num_joints
        % Extract MDH parameters
        a = mdh(i, 1); 
        alpha = mdh(i, 2);
        d = mdh(i, 3); 
        theta = joint_angles(i) + mdh(i, 4); % Include joint angle and offset

        % Compute individual transformation matrix
        T_COPPELIA_i = [
            cos(theta),             -sin(theta),              0,            a;
            sin(theta)*cos(alpha),   cos(theta)*cos(alpha),  -sin(alpha),  -sin(alpha)*d;
            sin(theta)*sin(alpha),   cos(theta)*sin(alpha),   cos(alpha),   cos(alpha)*d;
            0,                       0,                       0,            1
        ];

        % Update cumulative transformation
        T_COPPELIA = T_COPPELIA * T_COPPELIA_i;
    end

    % Simplify the final transformation matrix
    T_COPPELIA_FINAL = simplify(T_COPPELIA);
end


