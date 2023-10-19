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
            % covariance and variance
            u = [lin_velocity; ang_velocity];
            obj.x(1:3) = f(obj.x(1:3), u);
            F = jac_f(obj.x(1:3), u, dt);
            Q = diag([obj.sigxy^2*dt, obj.sigxy^2*dt, obj.sigth^2*dt]);
            obj.P(1:3,1:3) = F * obj.P(1:3,1:3) * F' + Q;
        end
        
        function input_measurements(obj, measurements, nums)
            % Perform the update step of the EKF. This involves adding
            % new (not previously seen) landmarks to the state vector and
            % implementing the EKF innovation equations.
            % Q1. we should check the dimension, especially the dimension
            % of the P matrix
            % Q2. How about change the whole structure with a more concise
            % way? Because we have already implemented the new landmark
            % check in the add_new_landmarks(obj, y, nums) function.
            % However, the current sturcture can solve the compute the 
            % innovation problem. So we should understand what is the
            % meaning of this step first.
        
            % Extract robot pose from the state
            x_k = obj.x(1);
            y_k = obj.x(2);
            theta = obj.x(3);
        
            % Iterate over the measurements
            % This structure has an implicit issue since only one index
            % will be passed as input parameter
            for i = 1:length(nums)
                landmark_idx = find(obj.idx2num == nums(i));
                
                % If the landmark is new
                if isempty(landmark_idx)
                    % update the state x and P matrix
                    add_new_landmarks(obj, measurements(2*i-1:2*i), nums(i));
                    landmark_idx = find(obj.idx2num == nums(i)); % Update the landmark index
                end
                
                % Compute expected measurement
                ix = 3 + 2*(landmark_idx - 1) + 1;
                iy = ix + 1;
                expected_measurement = [-cos(theta) * (x_k - obj.x(ix)) - sin(theta) * (y_k - obj.x(iy));
                                         sin(theta) * (x_k - obj.x(ix)) - cos(theta) * (y_k - obj.x(iy))];
                
                % Compute the innovation
                innovation = measurements(2*i-1:2*i) - expected_measurement;
                
                % Compute the Jacobian H using the jac_h function
                % There is a tinny problem since only one num(i) will be 
                % counted, but the input nums(i) was an array in the 
                % implemented jac_h function
                H = jac_h(obj.x, nums(i), obj.idx2num); % 2*(3+2N)
                
                % EKF update equations
                S = H * obj.P * H' + obj.siglm * eye(2); % Measurement covariance
                K = obj.P * H' / S; % Kalman gain
                obj.x = obj.x + K * innovation; % Update state
                obj.P = (eye(size(obj.P)) - K * H) * obj.P; % Update covariance
            end
        end

        
        function add_new_landmarks(obj, y, nums)
            % Add a new (not seen before) landmark to the state vector and
            % covariance matrix. You will need to associate the landmark's
            % id number with its index in the state vector.
            % Q1: do we need to add a noise into the y value, which is the
            % output of the measurement step
            % Q2: How to augment the covariance matrix?
        
            % Extract robot pose from the state
            x_k = obj.x(1);
            y_k = obj.x(2);
            theta = obj.x(3);
        
            % Iterate over the new measurements
            for i = 1:length(nums)
                if ~ismember(nums(i), obj.idx2num)
                    % Compute the initial estimate of the landmark's position
                    l_x = x_k + cos(theta) * y(2*i-1) - sin(theta) * y(2*i);
                    l_y = y_k + sin(theta) * y(2*i-1) + cos(theta) * y(2*i);
                    
                    % Augment the state vector with the new landmark position
                    obj.x = [obj.x; l_x; l_y];
                    
                    % Augment the covariance matrix
                    n = length(obj.x);
                    P_new = LARGE * eye(n); % LARGE is a large value for initial uncertainty
                    P_new(1:n-2, 1:n-2) = obj.P; % Copy the old covariance values
                    obj.P = P_new;
                    
                    % Update the idx2num mapping
                    obj.idx2num = [obj.idx2num; nums(i)];
                end
            end
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


function y = h(x, idx, idx2num)
    % Given the state x and a list of indices idx, compute the state
    % measurement y.
    % x is the current state which contains the vehicle and landmarks
    % state in tantem
    for i = idx
        if ~ismember(i, idx2num)
            idx2num(end+1) = i;
        end
    end
    m = length(idx); % get number of landmarks
    y = zeros(2 * m); % dimension of y: 2 * m
    x_k = x(1);
    y_k = x(2);
    theta = x(3);
    for i = 1 : m
        landmark_idx = find(idx2num == idx(i)); % Find the order of appearance of the landmark in idx2num
        ix = 3 + 2*(landmark_idx - 1) + 1; % get the x coordinate's index of i's landmark in the state
        iy = ix + 1; % get the y coordinate's index      
        x_ix = -cos(theta) * (x_k - x(ix)) - sin(theta) * (y_k - x(iy));
        y_iy = sin(theta) * (x_k - x(ix)) - cos(theta) * (y_k - x(iy));
        y(2*i-1) = x_ix;
        y(2*i) = y_iy;
    end
end

function H = jac_h(x, idx, idx2num)
    % Given the state x and a list of indices idx, compute the Jacobian of
    % the measurement function h.
    % need to check whether we should use this ~ismember method or not
    for i = idx
        if ~ismember(i, idx2num)
            idx2num(end+1) = i;
        end
    end
    m = length(idx); % get current observing number of landmarks
    N = length(idx2num); % get total number of landmarks we have already seen
    H = zeros(2*m, (3 + 2*N)); % dimension of y: 2m * (3+2N)
    x_k = x(1);
    y_k = x(2);
    theta = x(3);
    for i = 1 : m
        landmark_idx = find(idx2num == idx(i)); % Find the order of appearance of the landmark in idx2num
        ix = 3 + 2*(landmark_idx - 1) + 1; % get the x coordinate's index of i's landmark in the state
        iy = ix + 1; % get the y coordinate's index
        H(2*i-1,1) = -cos(theta);
        H(2*i-1,2) = -sin(theta);
        H(2*i-1,3) = sin(theta) * (x_k - x(ix)) - cos(theta) * (y_k - x(iy));
        H(2*i-1,3+2*i-1) = cos(theta);
        H(2*i-1,3+2*i) = sin(theta);
        H(2*i,1) = sin(theta);
        H(2*i,2) = -cos(theta);
        H(2*i,3) = cos(theta) * (x_k - x(ix)) + sin(theta) * (y_k - x(iy));
        H(2*i,3+2*i-1) = -sin(theta);
        H(2*i,3+2*i) = cos(theta);
    end
end