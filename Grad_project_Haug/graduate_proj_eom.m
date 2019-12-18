
clear all; format compact  % compact produces single-spaced output


% Defining symbolic variables
syms xc theta1 theta2 mc L m1 J1 m2 J2 g F

% Finding derivatives of all the generalized coordinates
GC = [{xc},{theta1},{theta2}]; 
dxc = fulldiff(xc,GC);
dtheta1 = fulldiff(theta1,GC); 
dtheta2 = fulldiff(theta2,GC);
d2xc = fulldiff(dxc,GC);
d2theta1 = fulldiff(dtheta1,GC);
d2theta2 = fulldiff(dtheta2,GC);

% Solving for the X and Y positions for the pendulums
x1 = xc+L*sin(theta1);
y1 = L*cos(theta1);

x2 = x1 + L*sin(theta1+theta2);
y2 = y1 + L*cos(theta1+theta2);

% Differentiating the positions
dx_a = fulldiff(x1,GC);
dy_a = fulldiff(y1,GC);

dx_b = fulldiff(x2,GC);
dy_b = fulldiff(y2,GC);

% Kinetic Energy:
T = 0.5*m1*(dx_a^2+dy_a^2)+0.5*m2*(dx_b^2+dy_b^2)+0.5*mc*(dxc^2)+0.5*J1*(dtheta1^2)+0.5*J2*(dtheta2^2);

% Potential Energy:
V = m1*(g)*y1*0.5 + m2*(g)*(y1+0.5*(y2-y1));

% Lagrangian:
Lagrangian = T-V;

% EOMs:
eq1 = fulldiff(diff(Lagrangian,dxc),GC) - diff(Lagrangian,xc);
eq2 = fulldiff(diff(Lagrangian,dtheta1),GC) - diff(Lagrangian,theta1);
eq3 = fulldiff(diff(Lagrangian,dtheta2),GC) - diff(Lagrangian,theta2);
eq1 = simplify(eq1);
eq2 = simplify(eq2);
eq3 = simplify(eq3);

% Xi: non-conservative terms
Xi1 = F; 
Xi2 = 0;
Xi3 = 0;


 
% Solving for double derivative 
var = [d2xc d2theta1 d2theta2];
eq = [eq1==Xi1, eq2==Xi2, eq3==Xi3]; 
[f1, f2, f3] = solve(eq,var);



%% Linearization


% Approximating values
eq1 = subs(eq1, [sin(theta1) cos(theta1) sin(theta2) cos(theta2) dtheta1^2 dtheta2^2], [theta1 1 theta2 1 0 0]);  
eq2 = subs(eq2, [sin(theta1) cos(theta1) sin(theta2) cos(theta2) dtheta1^2 dtheta2^2], [theta1 1 theta2 1 0 0]);  
eq3 = subs(eq3, [sin(theta1) cos(theta1) sin(theta2) cos(theta2) dtheta1^2 dtheta2^2], [theta1 1 theta2 1 0 0]);

A = [0                      0                0                     1           0                  0;
     0                      0                0                     0           1                  0;
     0                      0                0                     0           0                  1;                      
     diff(f1, xc)    diff(f1, theta1)   diff(f1, theta2)    diff(f1, dxc) diff(f1, dtheta1) diff(f1, dtheta2);
     diff(f2, xc)    diff(f2, theta1)   diff(f2, theta2)    diff(f2, dxc) diff(f2, dtheta1) diff(f2, dtheta2);
     diff(f3, xc)    diff(f3, theta1)   diff(f3, theta2)    diff(f3, dxc) diff(f3, dtheta1) diff(f3, dtheta2)];

% Substituting values for constants
A = subs(A, [mc L m1 J1 m2 J2 g], [4 1.5 0.5 (1/12)*0.5*1.5^2 0.5 (1/12)*0.5*1.5^2 9.81]);

% Adding values for the equilibrium point
A = subs(A, [xc theta1 theta2 dxc dtheta1 dtheta2], [0 0 0 0 0 0]);
A = double(A);

% Doing the same for B as done for A
B = [0;
    0;
    0;
    diff(f1, F);
    diff(f2, F);
    diff(f3, F)];

B = subs(B, [mc L m1 J1 m2 J2 g], [4 1.5 0.5 (1/12)*0.5*1.5^2 0.5 (1/12)*0.5*1.5^2 9.81]);
B = subs(B, [xc theta1 theta2 dxc dtheta1 dtheta2], [0 0 0 0 0 0]);

B = double(B);

C = diag([1 1 1 1 1 1]);
D = 0;

% Pentalty matrices. Made to mainly penalize deviations in theta1 and
% theta2, as this is more important than where the cart is located. 
Q = diag([1 50 50 0.01 10 10]);
R = 0.001;

% Calculating the gain K
K = lqr(A,B,Q,R)



