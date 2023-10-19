%% Preoperation
clear all; close all; clc;

pb = PiBot('192.168.50.1'); % Use this command instead if using PiBot.

addpath("../arucoDetector/include/");
addpath("../arucoDetector/");
load("arucoDict.mat");
load("cameraParams.mat");

%% Create a window to visualise the robot camera
figure;
camAxes = axes();
slam = ekf_slam();
marker_length = 0.7;
%% Follow the line in a loop
while true
    t1 = tic;
    % First, get the current camera frame
    image = pb.getImage();
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(image, marker_length, cameraParams, arucoDict);
   
    % To crop the last 100 rows
    end_row = size(img, 1);
    start_row = end_row - 100 + 1;
    img = image(start_row:end_row, :, :);

    % Convert to grayscale and binarize
    gray_img = rgb2gray(img);
    bin_img = ~imbinarize(gray_img, 0.3);

    imshow(bin_img, "Parent", camAxes); % Check the video

    % Calculate the percentage of black pixels
    total_pixels = numel(bin_img);
    black_pixels = sum(~bin_img, 'all');
    percentage_black = (black_pixels / total_pixels) * 100;
    
    % disp(['Percentage of black pixels: ', num2str(percentage_black), '%']);

    % Check if the percentage of black pixels exceeds a threshold
    if percentage_black > 99.5
        display("End of line");
        end_of_line = true;
        if (end_of_line)
            pb.stop
            break;
        end
    end
    

    % Find the centre of mass of the line pixels
    % we only need to get line center, so we don't need to care about rows
    [r, c] = find(bin_img == 1);
    line_centre = mean(c); 

    % Normalize the line centre to lie between [-1, 1]
    line_centre = (line_centre - size(img)/2) / (size(img)/2);
    
  
    
    if abs(line_centre) > 0.5
        u = 0.1;
        q = -0.5*line_centre;
    else
        u = 0.15;
        q = - 0.3*line_centre; % replace with computed values
    end
    
    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u, q);

    
    % Apply the wheel velocities
    pb.setVelocity(wl, wr);
    end_t1 = toc(t1);
    slam.input_velocity(end_t1,u,q);
    % slam.measure_landmarks
    slam.input_measurements(reshape(landmark_centres(:, 1:2)', [], 1),marker_nums);
    % display(wl)
    % display(wr)
    [robot,robot_cv] = slam.output_robot();
    disp(robot)
    disp(robot_cv)
    [landmarks, cov] = slam.output_landmarks();
    disp(landmarks)
    disp(cov)
    % Update the figure window
    drawnow;
end

% Save the trajectory of the robot to a file.
% Plot the integrated trajectory
figure;