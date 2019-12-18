% segway_eom.m
%
% "Segway-Style" Inverted Pendulum: Equations of Motion.
% (See slides from Lecture 13 for a visual description of the system.)
% Katie Byl. ECE/ME 179D, UCSB.

clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms theta1 theta2 phiw mc L m1 J1 m2 J2 Rw g b tau1 tau2 F

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{theta1},{theta2},{phiw}]; % Using ABSOLUTE angles here
dtheta1 = fulldiff(theta1,GC); % time derivative. GC are variables (over time)
dtheta2 = fulldiff(theta2,GC);
dphiw = fulldiff(phiw,GC);
d2theta1 = fulldiff(dtheta1);
d2theta2 = fulldiff(dtheta2);
d2phiw = fulldiff(dphiw,GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xw = Rw*phiw; 
x1 = xw+L*cos(theta1);
yw = 0;    %% Or should it be radius of wheel?
y1 = L*sin(theta1);

x2 = x1 + L*cos(theta1+theta2);
y2 = y1 + L*sin(theta1+theta2);

% 1c. Define any required velocity terms (for masses):
dxw = fulldiff(xw,GC);
dx1 = fulldiff(x1,GC);
dy1 = fulldiff(y1,GC);

dx2 = fulldiff(x2,GC);
dy2 = fulldiff(y2,GC);

% 2. Kinetic Energy: (Rotation energy for cart wheel is not included)

T = (1/2)*(m1*(dx1^2+dy1^2)+m2*(dx2^2+dy2^2)+mc*(dxw^2)+J1*dtheta1^2+J2*dtheta2^2);

% 3. Potential Energy:
V = m1*g*y1 + m2*g*y2;

% 4. Lagrangian:
Lagrangian = T-V;

% 5. EOMs:
eq1 = fulldiff(diff(Lagrangian,dphiw),GC) - diff(Lagrangian,phiw);
eq2 = fulldiff(diff(Lagrangian,dtheta1),GC) - diff(Lagrangian,theta1);
eq3 = fulldiff(diff(Lagrangian,dtheta2),GC) - diff(Lagrangian,theta2);
eq2 = simplify(eq2);

% 6. Xi: non-conservative terms
Xi1 = tau1 + F*(-y1);
Xi2 = tau2 + F*(-y2);
Xi3 = F;

X = [phiw theta1 theta2 dphiw dtheta1 dtheta2];

dX = output_dX(X);

