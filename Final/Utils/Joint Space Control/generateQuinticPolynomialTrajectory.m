function [q, qd, qdd, time_samples] = generateQuinticPolynomialTrajectory(key_positions, time_points)
% Generate a quintic polynomial trajectory for joint positions, velocities, and accelerations.
%
% This function calculates the joint positions (q), velocities (qd), and accelerations (qdd)
% using quintic polynomial interpolation for a given set of key positions and time points.
%
% INPUT:
%   key_positions - A NxM matrix where N is the number of key positions
%                   and M is the number of joints. Each row represents
%                   the joint positions (in radians) at a specific time.
%   time_points    - A 1xN vector of time points corresponding to the key positions.
%                   These specify the time intervals for each segment.
%
% OUTPUT:
%   q              - A TxM matrix of joint positions, where T is the number of time samples.
%   qd             - A TxM matrix of joint velocities.
%   qdd            - A TxM matrix of joint accelerations.
%   time_samples   - A 1xT vector of time samples.
%
% USAGE:
%   [q, qd, qdd, time_samples] = generateQuinticPolynomialTrajectory(key_positions, time_points);

    % LOG: Start trajectory generation
    fprintf('[LOG INFO] -- Starting Quintic Polynomial Trajectory Generation...\n');

    % Initialization
    n_joints = size(key_positions, 2); % Number of joints
    n_segments = size(key_positions, 1) - 1; % Number of trajectory segments
    time_samples = linspace(time_points(1), time_points(end), 600); % Time samples for trajectory

    q = zeros(length(time_samples), n_joints); % Joint positions
    qd = zeros(length(time_samples), n_joints); % Joint velocities
    qdd = zeros(length(time_samples), n_joints); % Joint accelerations

    % Iterate through each joint and each trajectory segment
    for joint_idx = 1:n_joints
        for segment_idx = 1:n_segments
            % Extract time interval and key positions
            t0 = time_points(segment_idx);
            tf = time_points(segment_idx + 1);
            p0 = key_positions(segment_idx, joint_idx);
            pf = key_positions(segment_idx + 1, joint_idx);

            % Initial and final velocities and accelerations
            v0 = 0; a0 = 0;
            vf = 0; af = 0;

            % Quintic polynomial coefficients
            A = [1, t0, t0^2, t0^3, t0^4, t0^5;
                 0, 1, 2*t0, 3*t0^2, 4*t0^3, 5*t0^4;
                 0, 0, 2, 6*t0, 12*t0^2, 20*t0^3;
                 1, tf, tf^2, tf^3, tf^4, tf^5;
                 0, 1, 2*tf, 3*tf^2, 4*tf^3, 5*tf^4;
                 0, 0, 2, 6*tf, 12*tf^2, 20*tf^3];
            b = [p0; v0; a0; pf; vf; af];
            coeffs = A \ b;

            % Generate interpolation points for this segment
            segment_samples = find(time_samples >= t0 & time_samples <= tf);
            for sample_idx = segment_samples
                t = time_samples(sample_idx);
                q(sample_idx, joint_idx) = coeffs(1) + coeffs(2)*t + coeffs(3)*t^2 + ...
                                           coeffs(4)*t^3 + coeffs(5)*t^4 + coeffs(6)*t^5;
                qd(sample_idx, joint_idx) = coeffs(2) + 2*coeffs(3)*t + 3*coeffs(4)*t^2 + ...
                                            4*coeffs(5)*t^3 + 5*coeffs(6)*t^4;
                qdd(sample_idx, joint_idx) = 2*coeffs(3) + 6*coeffs(4)*t + 12*coeffs(5)*t^2 + ...
                                             20*coeffs(6)*t^3;
            end
        end
    end

    % LOG: Trajectory generation complete
    fprintf('[LOG INFO] -- Quintic Polynomial Trajectory Successfully Generated.\n');
end
