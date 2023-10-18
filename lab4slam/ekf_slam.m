classdef ekf_slam < handle
    %EKF_SLAM The EKF algorithm for SLAM
    
    properties
        x = zeros(3,1); % The estimated state vector
        P = zeros(3,3); % The estimated state covariance
        
		% The covariance values provided here are NOT correct!
        sigxy = 0.1; % The covariance of linear velocity
        sigth = 0.01; % The covariance of angular velocity
        siglm = 0.01; % The covariance of landmark measurements
        
        idx2num = []; % The map from state vector index to landmark id.
        % to store the order of the landmark index seen by the robot
    end
    
    methods
        function input_velocity(obj, dt, lin_velocity, ang_velocity)
            % Perform the prediction step of the EKF. This involves updating
            % the state and covariance estimates using the input velocity,
            % the time step, and the covariance of the update step.
            % The eta means the total noise which implemented on the APA^T
            eta = [obj.sigxy*dt 0 0, 0 obj.sigxy*dt 0, 0 0 obj.sigth*dt];

        end
        
        function input_measurements(obj, measurements, nums)
            % Perform the update step of the EKF. This involves adding
            % new (not previously seen) landmarks to the state vector and
            % implementing the EKF innovation equations. You will need the
            % landmark measurements and you will need to be careful about
            % matching up landmark ids with their indices in the state
            % vector and covariance matrix.
            
        end
        
        function add_new_landmarks(obj, y, nums)
            % Add a new (not seen before) landmark to the state vector and
            % covariance matrix. You will need to associate the landmark's
            % id number with its index in the state vector.
            
        end
        
        function [robot, cov] = output_robot(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the robot.

        end
        
        function [landmarks, cov] = output_landmarks(obj)
            % Suggested: output the part of the state vector and covariance
            % matrix corresponding only to the landmarks.

        end
        
    end
end

 % Jacobians and System Functions

function x1 = f(x0,u)
    % integrate the input u from the state x0 to obtain x1.
    % The state equation with D(3+2N), p22
    % Q1: where is the input x0 and u
    % x0 is the default value, u is from the inverse kinemics
    x1(1) = x0(1) + cos(x0(3)) * u(1);
    x1(2) = x0(2) + sin(x0(3)) * u(1);
    x1(3) = x0(3) + u(2);
    x1(4:end) = x0(4:end);
end

function F = jac_f(x0, u, dt)
% The noise of BQB.T is exactly the dt times the variance Q
    n = length(x0);
    F = eye(n);
    F(1,3) = -sin(x0(3)) * u(1) * dt;
    F(2,3) = cos(x0(3)) * u(1) * dt;
end


function y = h(x, idx)
    % Given the state x and a list of indices idx, compute the state
    % measurement y.
    % x is the current state which contains the vehicle and landmarks
    % state in tantem
    m = length(idx); % get number of landmarks
    y = zeros(2 * m); % dimension of y: 2 * m
    x_k = x(1);
    y_k = x(2);
    theta = x(3);
    for i = 1 : m
        ix = 3 + (2 * (i - 1)) + 1; % get the x coordinate's index of i's landmark in the state
        iy = 3 + (2 * (i - 1)) + 2; % get the y coordinate's index       
        x_ix = -cos(theta) * (x_k - x(ix)) - sin(theta) * (y_k - x(iy));
        y_iy = sin(theta) * (x_k - x(ix)) - cos(theta) * (y_k - x(iy));
        y(2*i-1) = x_ix;
        y(2*i) = y_iy;
    end

end

function [H, idx2num] = jac_h(x, idx, idx2num)
    % Given the state x and a list of indices idx, compute the Jacobian of
    % the measurement function h.
    % need to check whether we should use this ~ismember method or not
    for i = idx
        if ~ismember(i, idx2num)
            idx2num(end+1) = i;
        end
    end
    m = length(idx); % get current observing number of landmarks
    N = length(idx2num); % get total number of landmarks
    y = zeros(2*m * (3 + 2*N)); % dimension of y: 2m * (3+2N)
    x_k = x(1);
    y_k = x(2);
    theta = x(3);
    for i = 1 : m
        ix = 3 + (2 * (i - 1)) + 1; % get the x coordinate's index of i's landmark in the state
        iy = 3 + (2 * (i - 1)) + 2; % get the y coordinate's index       
        y(2*i-1,1) = -cos(theta);
        y(2*i-1,2) = -sin(theta);
        y(2*i-1,3) = sin(theta) * (x_k - x(ix)) - cos(theta) * (y_k - x(iy));
        y(2*i-1,3+2*i-1) = cos(theta);
        y(2*i-1,3+2*i) = sin(theta);
        y(2*i,1) = sin(theta);
        y(2*i,2) = -cos(theta);
        y(2*i,3) = cos(theta) * (x_k - x(ix)) + sin(theta) * (y_k - x(iy));
        y(2*i,3+2*i-1) = -sin(theta);
        y(2*i,3+2*i) = cos(theta);
    end


end