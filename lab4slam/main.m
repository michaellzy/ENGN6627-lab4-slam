% Always begin by using addpath
% You can always test your algorithm in simulator
% addpath("../simulator")

% Add the ARUCO detector
% Check the example in the folder
addpath("../arucoDetector")
addpath("../arucoDetector/include")
addpath("../arucoDetector/dictionary")

% Load parameters
load("arucoDict.mat")
load("cameraParams.mat")
marker_length = 0.070;

% Initialize the pibot connection
pb = Pibot('192.168.50.1');

% Initialise your EKF class
EKF = ekf_slam();

% set initial state
state = [0; 0; 0];

while(true)
    
    % line follow module
    follow_line();
    
    % measure landmarks
    % This is the function you'll use. Check the file for more details.
    [marker_nums, landmark_centres, marker_corners] = detectArucoPoses(image, marker_length, cameraParams, arucoDict);

    for 
    
    % run EKF functions
    
    % plot estimates
    
end
