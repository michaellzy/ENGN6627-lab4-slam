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
    end
    
    methods
        function input_velocity(obj, dt, lin_velocity, ang_velocity)
            % Perform the update step of the EKF. This involves updating
            % the state and covariance estimates using the input velocity,
            % the time step, and the covariance of the update step.
            
        end
        
        function input_measurements(obj, measurements, nums)
            % Perform the innovation step of the EKF. This involves adding
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
    x1(1) = x0(1) + cos(x0(3)) * u(1);
    x1(2) = x0(2) + sin(x0(3)) * u(1);
    x1(3) = x0(3) + u(2);
    x1(4:end) = x0(4:end);
end

function F = jac_f(x0,u)
    % Given the state x0 and input signal u, compute the Jacobian of f.
    n = length(x0);
    F = eye(n);
    F(1,3) = -sin(x0(3)) * x0(1);
    F(2,3) = cos(x0(3)) * x0(1);
    
end

function y = h(x, idx)
    % Given the state x and a list of indices idx, compute the state
    % measurement y.
    m = length(idx); % get number of landmarks
    y = zeros(2 * m); % dimension of y: 2 * m
    x_k = x(1);
    y_k = x(2);
    theta = x(3);
    for i = 1 : m
        ix = 3 + (2 * (idx(i) - 1)) + 1;
        iy = 3 + (2 * (idx(i) - 1)) + 2;        
        x_ix = -cos(theta) * (x_k - x(ix)) - sin(theta) * (y_k - x(iy));
        y_iy = sin(theta) * (x_k - x(ix)) - cos(theta) * (y_k - x(iy));
        y(2*i-1) = x_ix;
        y(2*i) = y_iy;
    end    

end

function H = jac_h(x, idx)
    % Given the state x and a list of indices idx, compute the Jacobian of
    % the measurement function h.

end