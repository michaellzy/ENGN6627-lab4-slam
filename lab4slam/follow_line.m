%% Preoperation
clear all; close all; clc;

% Calibrate the scale parameter and wheel track of the robot
addpath("../simulator/"); % Add the simulator to the MATLAB path.
pb = piBotSim("floor_spiral.jpg");
% pb = piBotSim("floor_course.jpg");

% Start by placing your robot at the start of the line
pb.place([2.5;2.5], 0.6421);

% pb = PiBot('192.168.50.1'); % Use this command instead if using PiBot.

%% Set parameters for draw the trajectory
state = [0; 0; 0]; % [x, y, theta]
% trajectory = state;
trajectory = [];

% Time step
dt = 0.1;  % in seconds

%% Create a window to visualise the robot camera
figure;
camAxes = axes();

%% Follow the line in a loop
while true

    tic

    % First, get the current camera frame
    img = pb.getImage();
    % To crop the last 100 rows
    end_row = size(img, 1);
    start_row = end_row - 100 + 1;
    img = img(start_row:end_row, :, :);

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
        display("End of line")
        end_of_line = true;
        if (end_of_line)
            pb.stop
            break;
        end
    end

    % If you have reached the end of the line, you need to stop by breaking the loop.
    % end_of_line = false;
    % if (end_of_line)
    %     break;
    % end

    % Find the centre of mass of the line pixels
    % we only need to get line center, so we don't need to care about rows
    [r, c] = find(bin_img == 1);
    line_centre = mean(c); 

    % Normalize the line centre to lie between [-1, 1]
    line_centre = (line_centre - size(img)/2) / (size(img)/2);
    % line_centre = (line_centre - size(img, 2)/2) / (size(img, 2)/2);

    % Use the line centre to compute a velocity command
    % u = 0.1; % Linear velocity
    % q = -0.5 * line_centre; % Angular velocity based on line centre

    % I prefer the above lines of code, since it just keep going and use
    % other algorithm to control whether it will stop or not
    % Check the condition when the point is invisible (as the line shrinks)
    if abs(line_centre) > 0.5
        u = 0.0;
    else
        u = 0.1;
    end
    q = -0.5*line_centre; % replace with computed values

    % Compute the required wheel velocities
    [wl, wr] = inverse_kinematics(u, q);

    new_state = integrate_kinematics(state, dt, u, q);
    new_state = reshape(new_state, [3, 1]);
    state = new_state; % update
    trajectory = [trajectory, new_state]; % Store the new state

    % disp(new_state);
    % disp(trajectory);

    % Constrain the min wheel velocity
    if abs(wl)<5
        wl = 5*(wl/abs(wl));
    end
    if abs(wr)<5
        wr = 5*(wr/abs(wr));
    end

    % Apply the wheel velocities
    pb.setVelocity(wl, wr);

    toc
    dt = toc;

    % display(wl)
    % display(wr)

    % Update the figure window
    drawnow;
end

% Save the trajectory of the robot to a file.
% Plot the integrated trajectory
figure;
plot(trajectory(1, :), trajectory(2, :));
title('Integrated Trajectory');
xlabel('X');
ylabel('Y');
% Don't use this if you are using PiBot.
pb.saveTrail();

