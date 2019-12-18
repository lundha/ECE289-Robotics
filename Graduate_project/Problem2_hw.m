
clear all; format compact  % compact produces single-spaced output

% Define symbolic variables in matlab:
syms phiw thetab L mb Jb mw Jw Rw g b tau

% 1a. GC's (generalized coordinates), and their derivatives:
GC = [{phiw},{thetab}]; % Using ABSOLUTE angles here
dphiw = fulldiff(phiw,GC); % time derivative. GC are variables (over time)
dthetab = fulldiff(thetab,GC);
d2phiw = fulldiff(dphiw, GC);
d2thetab = fulldiff(dthetab, GC);

% 1b. Geometry of the masses/inertias, given GC's are freely changing...
xw = Rw*phiw;
xb = xw+L*sin(thetab);
yw = 0;
yb = L*cos(thetab);

% 1c. Define any required velocity terms (for masses):
dxw = fulldiff(xw,GC);
dxb = fulldiff(xb,GC);
dyb = fulldiff(yb,GC);

% 2. Kinetic Energy:
T = (1/2)*(mw*dxw^2 + Jw*dphiw^2 + mb*(dxb^2 + dyb^2) + Jb*dthetab^2)

% 3. Potential Energy:
V = mb*g*yb

% 4. Lagrangian:
Lagrangian = T-V
%%
% 5. EOMs:
eq1 = fulldiff(diff(Lagrangian,dphiw),GC) - diff(Lagrangian,phiw)
eq2 = fulldiff(diff(Lagrangian,dthetab),GC) - diff(Lagrangian,thetab);
eq2 = simplify(eq2)

% 6. Xi: non-conservative terms
Xi1 = tau - b*(dphiw-dthetab)  % Motor torque tau, and back emf damping b
Xi2 = -tau + b*(dphiw-dthetab)  % (equal and opposite to above)

%% 2a
X = [dphiw dthetab d2phiw d2thetab];

% Approximating values
eq1 = subs(eq1, [sin(thetab) cos(thetab) dthetab^2], [thetab 1 0]);  
eq2 = subs(eq2, [sin(thetab) cos(thetab) dthetab^2], [thetab 1 0]);

% Solving for double derivative
d2phiw = solve(eq1==Xi1, d2phiw); 
d2thetab = solve(eq2==Xi2, d2thetab);

A = [0                       0                   1                       0;
     0                       0                   0                       1;
     diff(d2phiw, phiw)      diff(d2phiw, thetab)     diff(d2phiw, dphiw)      diff(d2phiw, dthetab);
     diff(d2thetab, phiw)    diff(d2thetab, thetab)   diff(d2thetab, dphiw)    diff(d2thetab, dthetab);]
 
A = subs(A, [L mb Jb mw Jw Rw g b], [0.0950 0.5910 0.0019 0.034 1.6e-05 0.0310 9.81 0.062]);
A = double(A);

B = [0;
    0;
    diff(d2phiw, tau);
    diff(d2thetab, tau)];
B = subs(B, [L mb Jb mw Jw Rw g b], [0.0950 0.5910 0.0019 0.034 1.6e-05 0.0310 9.81 0.062]);
B = double(B);

C = diag([1 1 1 1]);
D = 0;

%% 2b


dt = 0.001;
t = 0:dt:50;
Q = diag([1 0 0 0]);
R = [0.1];
% Below, we also define an "initial condition" for the system:
X0 = [0;0;1;0];
% MATLAB can calculate the optimal gain matrix, K, for the
% LQR problem defined by the system dynamics (A and B matrices)
% and your costs, Q and R:
K01 = lqr(A,B,Q,R);

ss2 = ss(A-B*K01,[],C,[]);
% Below is the CLOSE-LOOP system, using u=-K*x as a control law:
y2 = initial(ss2,X0,t);

% Closed-loop poles are eigenvalues of "A-BK", for u=-K*x
closed_loop_poles01 = eig(A-B*K01)
% Line below calculates the CONTROL EFFORT, u:
u2 = -y2*K01'; 


dt = 0.001;
t = 0:dt:50;



figure(1);
for n=1:4
    subplot(5,1,n)
    plot(t,y2(:,n),'b-');
    hold on
    xlim([0 5]);
    ylabel(['X_',num2str(n)])
end

subplot(515);
plot(t, u2);
ylabel('u');
xlim([0 5]);



%% 2c


Q = diag([1 0 0 0]);
R = [1];

% MATLAB can calculate the optimal gain matrix, K, for the
% LQR problem defined by the system dynamics (A and B matrices)
% and your costs, Q and R:
K1 = lqr(A,B,Q,R);

ss2 = ss(A-B*K1,[],C,[]);
% Below is the CLOSE-LOOP system, using u=-K*x as a control law:
y2 = initial(ss2,X0,t);

% Closed-loop poles are eigenvalues of "A-BK", for u=-K*x
closed_loop_poles1 = eig(A-B*K1)
% Line below calculates the CONTROL EFFORT, u:
u2 = -y2*K1'; 



% Below, we also define an "initial condition" for the system:
X0 = [0;0;1;0];


figure(2);
for n=1:4
    subplot(5,1,n)
    plot(t,y2(:,n),'b-');
    hold on
    xlim([0 5]);
    ylabel(['X_',num2str(n)])
end

subplot(515);
plot(t, u2);
xlim([0 5]);
ylabel('u');

%% 2d


Q = diag([1 0 0 0]);
R = [10];

% MATLAB can calculate the optimal gain matrix, K, for the
% LQR problem defined by the system dynamics (A and B matrices)
% and your costs, Q and R:
K10 = lqr(A,B,Q,R);

ss2 = ss(A-B*K10,[],C,[]);
% Below is the CLOSE-LOOP system, using u=-K*x as a control law:
y2 = initial(ss2,X0,t);

% Closed-loop poles are eigenvalues of "A-BK", for u=-K*x
closed_loop_poles10 = eig(A-B*K10)
% Line below calculates the CONTROL EFFORT, u:
u2 = -y2*K10'; 


dt = 0.001;
t = 0:dt:50;
% Below, we also define an "initial condition" for the system:
X0 = [0;0;1;0];


figure(3);
for n=1:4
    subplot(5,1,n)
    plot(t,y2(:,n),'b-');
    hold on
    ylabel(['X_',num2str(n)])
    xlim([0 5]);
end

subplot(515);
plot(t, u2);
ylabel('u');
xlim([0 5]);

%% 2E


open_loop_poles = eig(A)
closed_loop_poles01
K01
closed_loop_poles1
K1
closed_loop_poles10
K10
A
B
C
D


