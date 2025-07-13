function robot = initRobot(config)
    % Create the robot modelL6
    robot = rigidBodyTree("DataFormat", "column");

    bodies = {
        rigidBody("base"),...
        rigidBody("part1"),...
        rigidBody("part2"),...
        rigidBody("part3"),...
        rigidBody("part4"),...
        rigidBody("part5"),...
        rigidBody("endeffector")
        };

    joints = {
        [], ...
        rigidBodyJoint("jnt1", "revolute"), ...
        rigidBodyJoint("jnt2", "revolute"), ...
        rigidBodyJoint("jnt3", "revolute"), ...
        rigidBodyJoint("jnt4", "revolute"), ...
        rigidBodyJoint("jnt5", "revolute"), ...
        rigidBodyJoint("jnt6", "revolute")
        };

    colls = {
        collisionCylinder(config.default_radius*4,0.06),... % Base
        collisionCylinder(config.default_radius,config.link_lengths(1)),...% Link 1
        collisionBox(config.link_lengths(2), config.default_radius,config.default_radius),...    % Link 1
        collisionBox(config.link_lengths(3), config.default_radius,config.default_radius),...     % Link 2
        collisionCylinder(config.link_lengths(4)/2,config.default_radius*2),... .    % Link 3
        collisionCylinder(config.default_radius/2, config.link_lengths(5)),...    % Link 4
        collisionBox(config.default_radius,config.link_lengths(6), config.default_radius),...    % Link 5
        collisionCylinder(config.default_radius/2 , config.link_lengths(6)),...    % Link 6
        };

    transforms = {
        trvec2tform([0 , 0, 0.03]),... % Base
        trvec2tform([0 , 0, config.link_lengths(1)/2]),...
        trvec2tform([config.link_lengths(2)/2 , 0, 0]),...
        trvec2tform([config.link_lengths(3)/2  ,0, 0]),...
        trvec2tform([0, 0, 0]),...
        trvec2tform([-config.link_lengths(4)/2 ,0, - config.link_lengths(5) / 2]),...
        trvec2tform([0, config.link_lengths(6), 0]),...
        trvec2tform([0 ,0, -config.link_lengths(6)/2])
        };

    addCollision(robot.Base, colls{1}, transforms{1});
    addCollision(robot.Base, colls{2}, transforms{2});

    % Add body to robot and Init Home_Position
    for i = 2:length(bodies)
        % Set home positions for joints
        joints{i}.HomePosition = config.home_configuration_matlab(i-1);

        % Set fixed transform using mDH parameters
        setFixedTransform(joints{i}, config.mdh(i - 1, :), "mdh");
        bodies{i}.Joint = joints{i};

        % Add collision
        addCollision(bodies{i}, colls{i+1}, transforms{i+1});
        % Attach body to robot
        addBody(robot, bodies{i}, bodies{i - 1}.Name);
    end
end

