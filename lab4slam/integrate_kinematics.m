function new_state = integrate_kinematics(state, dt, lin_velocity, ang_velocity)
%INTEGRATE_KINEMATICS integrate the kinematics of the robot

%   state is the current state, and has the form [x;y;theta]
%   dt is the length of time for which to integrate
%   lin_velocity is the (forward) linear velocity of the robot
%   ang_velocity is the angular velocity of the robot

%   new_state is the state after integration, also in the form [x;y;theta]
if ang_velocity ~= 0
    x = state(1) + (lin_velocity/ang_velocity)*(sin(state(3) + dt*ang_velocity) - sin(state(3)));
    y = state(2) + (lin_velocity/ang_velocity)*(-cos(state(3) + dt*ang_velocity) + cos(state(3)));
    theta = state(3) + dt*ang_velocity;
else
    x = state(1) + dt*cos(state(3))*lin_velocity;
    y = state(2) + dt*sin(state(3))*lin_velocity;
    theta = state(3);
end
new_state = [x, y, theta];

end