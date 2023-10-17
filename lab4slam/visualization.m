function visualization(robot_positions, reference_trajectory, robot_covariance, landmarks, landmark_covariances)
    % robot_positions: Nx2 matrix containing the estimated x and y positions of the robot over time
    % reference_trajectory: Nx2 matrix containing the reference x and y positions
    % robot_covariance: 2x2 matrix representing the covariance of the robot's position
    % landmarks: Mx2 matrix containing the x and y positions of the landmarks
    % landmark_covariances: Mx1 cell array, where each cell contains a 2x2 covariance matrix for the corresponding landmark

    % Plot the robot's trajectory
    plot(robot_positions(:, 1), robot_positions(:, 2), 'b-', 'LineWidth', 2);
    hold on;

    % Plot the reference trajectory
    plot(reference_trajectory(:, 1), reference_trajectory(:, 2), 'g--', 'LineWidth', 2);

    % Draw ellipse for the robot's covariance
    for i = 1:size(robot_positions, 1)
        draw_ellipse(robot_positions(i, :), robot_covariance);
    end

    % Plot landmarks
    plot(landmarks(:, 1), landmarks(:, 2), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

    % Draw ellipses for the landmark covariances
    for i = 1:size(landmarks, 1)
        draw_ellipse(landmarks(i, :), landmark_covariances{i});
    end

    legend('Estimated Trajectory', 'Reference Trajectory', 'Robot Covariance', 'Landmarks', 'Landmark Covariance');
    grid on;
    hold off;
end

function draw_ellipse(center, covariance)
    % This function will draw an ellipse centered at 'center' with shape determined by 'covariance'
    % using the eigenvectors of the covariance as axes of the ellipse and the square root of the eigenvalues
    % as the lengths of the axes.

    [eigvec, eigval] = eig(covariance);
    angle = atan2(eigvec(2, 1), eigvec(1, 1));
    a = sqrt(eigval(1, 1));
    b = sqrt(eigval(2, 2));

    t = linspace(0, 2*pi, 100);
    ell_x = center(1) + a*cos(t)*cos(angle) - b*sin(t)*sin(angle);
    ell_y = center(2) + a*cos(t)*sin(angle) + b*sin(t)*cos(angle);

    plot(ell_x, ell_y, 'r-');
end
