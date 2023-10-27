data_pred = load("collected_data.mat");
data_gt = load("gt_lab.mat");
landmarks = data_pred.all_landmarks;
landmarks_gt = data_gt.all_landmarks;
disp(size(landmarks(:, 1)))
x_coord = landmarks(2, :);
y_coord = landmarks(1, :);
x_coord_gt = landmarks_gt(1, :);
y_coord_gt = landmarks_gt(2, :);

% Plot the data
% figure; % Create a new figure
plot(x_coord, y_coord, 'ro', 'MarkerSize', 5, 'LineWidth', 2); % Plot predicted data with blue crosses
hold on; % Retain current plot when adding new plots
plot(x_coord_gt, y_coord_gt,'bx', 'MarkerSize', 5, 'LineWidth', 2);

