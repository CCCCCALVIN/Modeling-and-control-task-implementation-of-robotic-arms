function config = initConfig()
    config = struct();

    % Deault Radius
    config.default_radius = 0.01;

    % 
    config.joint_cnt = 6;

    % Links Length
    config.link_lengths = [127, 29.69, 108, 20, 168.98, 24.29] / 1000;

    % Home Configuration
    config.home_configuration_coppelia = zeros(1, 6);
    config.home_configuration_matlab = [0, -pi/2, 0, 0, pi/2, 0];

    % Joint Angle Limits (joints_cnt x 2)
    config.joint_angle_limits_coppelia = deg2rad([-110, 160; -35, 70; -120, 60; -180, 180; -200, 30; -360, 360;]);
    config.joint_angle_limits_matlab = config.joint_angle_limits_coppelia + (config.home_configuration_matlab - config.home_configuration_coppelia)';

    % Joint Velocity Limits (1 x joints_cnt)
    config.joint_velocity_limits = deg2rad([85, 60, 65, 200, 200, 450]);

    % Modi
    config.mdh =  [
        0,                          0,        config.link_lengths(1),       0;       % Link 1
        config.link_lengths(2),    -pi/2,     0,                           -pi/2;    % Link 2
        config.link_lengths(3),     0,        0,                            0;       % Link 3
        config.link_lengths(4),    -pi/2,     config.link_lengths(5),       0;       % Link 4
        0,                          pi/2,     0,                            pi/2;    % Link 5
        0,                          pi/2,    -config.link_lengths(6),       0;       % Link 6 (End-Effector)
        ];

end

