function [dX] = example_eom_function(t, X)

% t is a required input for the function, because ode45 in MATLAB
% will call your function with "time" as a first input.  In HW 3,
% even if we ignore time in calculating dX, it must still be listed 
% on the top line as a function input.
%
% X is a 4x1 vector with the states. Each of the state variables will
% be either a position (distance or angle measurement) or a 
% velocity (linear velocity or angular velocity).
%
% Output dX is a 4x1 vector of the DERIVATIVES of the states in
% vector X. So, each element in dX is either a velocity or an
% acceleration.  The velocity terms are just elements taken directly
% from input X, and the accelerations are to be calculated using
% your equations of motion.
%
% Toy system: two carts, connected by springs and dampers.
%
% To test this function, type the following in MATLAB:
%
% X0 = [0; .4; 0; 0];
% [tout,yout] = ode45(@example_eom_function,[0 20],X0');
% figure(11); clf
% plot(tout,yout)

x1 = X(1); % first cart position
x2 = X(2); % 2nd cart position
dx1 = X(3); % 1st cart velocity
dx2 = X(4); % 2nd cart veloicty

k1 = 100;
k2 = 200;
b = 50;
m1 = 1;
m2 = 2;

F = 10*sin(t); % we could make F a constant, or zero... whatever you like.

d2x1 = (1/m1)*(-k1*x1+k2*(x2-x1)+b*(dx2-dx1));
d2x2 = (1/m2)*(k2*(x1-x2)+b*(dx1-dx2) + F);

dX = [dx1; dx2; d2x1; d2x2];  % velocities and accelerations


