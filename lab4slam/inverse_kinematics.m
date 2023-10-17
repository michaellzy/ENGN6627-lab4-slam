function [wl, wr] = inverse_kinematics(u, q)
% Compute the left and right wheel velocities (wl, wr) required for the robot
% to achieve a forward speed u and angular speed q.

% The scale parameter and wheel track required to solve this are provided here.
% You can find these values in the robot simulator as well.
% In real-life, you would have to measure or calibrate them!
scale_parameter = 5.58e-3;
wheel_track = 0.14;

A = [1/ scale_parameter -(wheel_track / (2 * scale_parameter));
    1 / scale_parameter wheel_track / (2 * scale_parameter)];
% disp(A);
x = [u ;q];
% disp(x);
b = A * x;
% disp(b);
wl = b(1, 1);
wr = b(2, 1);
end