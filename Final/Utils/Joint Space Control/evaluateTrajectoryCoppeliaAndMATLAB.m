function evaluateTrajectoryCoppeliaAndMATLAB(ee_positions_coppelia, ee_positions_matlab)
% Calculate the initial offset (CoppeliaSim - MATLAB)
initial_error = ee_positions_coppelia(1, :) - ee_positions_matlab(1, :);

formatted_ = sprintf('%.4f, ', initial_error); % Format each value with 2 decimal places
formatted_ = formatted_(1:end-2); % Remove the trailing comma and space

% Log the offset information
fprintf('[LOG INFO] -- ERROR INITIAL: [%s] \n', formatted_);

% Calculate trajectory errors
trajectory_error = ee_positions_matlab  - ee_positions_coppelia;
error_magnitude = vecnorm(trajectory_error, 2, 2);

% Compute error statistics
avg_error = mean(error_magnitude);
formatted_ = sprintf('%.4f, ', avg_error);
formatted_ = formatted_(1:end-2);
fprintf('[LOG INFO] -- ERROR AVG: %s m\n', formatted_);

std_error = std(error_magnitude);
formatted_ = sprintf('%.4f, ', std_error);
formatted_ = formatted_(1:end-2);
fprintf('[LOG INFO] -- ERROR STD: %s m\n', formatted_);

max_error = max(error_magnitude);
formatted_ = sprintf('%.4f, ', max_error);
formatted_ = formatted_(1:end-2);
fprintf('[LOG INFO] -- ERROR MAX: %s m\n', formatted_);

min_error = min(error_magnitude);
formatted_ = sprintf('%.4f, ', min_error);
formatted_ = formatted_(1:end-2);
fprintf('[LOG INFO] -- ERROR MIN: %s m\n', formatted_);

% Plot the trajectories and error vectors

figure;
hold on;
% MATLAB trajectory (corrected)
plot3(ee_positions_matlab (:, 1), ee_positions_matlab (:, 2), ee_positions_matlab (:, 3), 'g-', 'LineWidth', 1.5);

% CoppeliaSim trajectory
plot3(ee_positions_coppelia(:, 1), ee_positions_coppelia(:, 2), ee_positions_coppelia(:, 3), 'r--', 'LineWidth', 1.5);

% Plot error vectors
for i = 1:size(ee_positions_matlab, 1)
    plot3([ee_positions_matlab(i, 1), ee_positions_coppelia(i, 1)], ...
        [ee_positions_matlab(i, 2), ee_positions_coppelia(i, 2)], ...
        [ee_positions_matlab(i, 3), ee_positions_coppelia(i, 3)], 'k:');
end

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('MATLAB vs CoppeliaSim End-Effector Trajectory');
legend('MATLAB Trajectory', 'CoppeliaSim Trajectory', 'Error');
hold off;

end

