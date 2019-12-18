clear all
% This code models a two-cart system, (time permitting) to
% be covered in class during Lecture 14. 
%
% The code is written in a "tutorial" way, to give comments
% as it is run.
% 
% For a sketch of the system and some additional handwritten
% comments, please see (on GauchoSpace, under "Lecture 14"):
%       sample_lqr_mfile_comments.pdf
%
% The system has to DOF's (x1 and x2) and one input, u.
% x is typically used as the variable for "states" and u is
% typically used as the variable for "input(s)", in state space
% representations of dynamic systems.
% 
% More specifically: A force is input at m1. 
% m1 and m2 are connected by k1 and b1, and k2 connects m2 to a wall.
%
% Katie Byl, UCSB, 2019.

m1 = 5; m2 = 2;
k1 = 200; b1 = 10; k2 = 100;

A = [0 0 1 0
    0 0 0 1
    -k1/m1 k1/m1 -b1/m1 b1/m1
    k1/m2 -(k1+k2)/m2 b1/m2 -b1/m2];
B = [0 0 1/m1 0]'
C = diag([1 1 1 1]);
D = 0;

% Commands below create a "state space system"
ss1 = ss(A,B,C,D)

% Open loop poles are the eigenvalues of matrix A
open_loop_poles = eig(A)

%=== Code below finds feedback gains "K" via LQR (linear
%    quadratic regulator) approach from Lecture 15. ===

% Q gives penalties on STATE (squared errors), and
% R gives a penalty on the square of the control effort, u.
% (i.e., J = x'*Q*x + u'*R*u, from Lecture 15.)
% Usually, Q has more than one non-zero diagonal entry, but
% note it still works fine as shown below. In general, Q will
% be diagonal (with no off-diagonal terms), however:
Q = diag([1 0 0 0]);
R = .001;

% MATLAB can calculate the optimal gain matrix, K, for the
% LQR problem defined by the system dynamics (A and B matrices)
% and your costs, Q and R:
K = lqr(A,B,Q,R);

% Below is the CLOSE-LOOP system, using u=-K*x as a control law:
ss2 = ss(A-B*K,[],C,[]);


% ==== We will then compare the above system to one with a new R ====

Qnew = Q;
% Below, let us change ONLY the value of R (with same Q):
Rnew = 1;
Knew = lqr(A,B,Qnew,Rnew);
% Define a TIME vector, to simulate the response, using MATLAB:
dt = 0.001;
t = 0:dt:50;
% Below, we also define an "initial condition" for the system:
X0 = [0;0;1;0];

% === Now, we will do LOTS of plots, both of STATES and of 
%     control effort, u
figure(11); clf
% "initial" simulated the initial condition response, given X0.
y1 = initial(ss1,X0,t);

for n=1:4
    subplot(5,1,n)
    plot(t,y1(:,n),'b-');
end
u1 = 0*t;  % Free response has "u=0", always...
subplot(515)
plot(t,u1,'r-');
subplot(511)
title('Free Response (u=0)')

J1terms(5) = 0; % u1=0
for n=1:4
    J1terms(n) = dt*sum(y1(:,n).^2);
end
fprintf('sqrd er:  X(1)       X(2)      X(3)       X(4)        u\n')
fprintf('u=0: %8.3f   %8.3f   %8.3f   %8.3f     %8.3f  \n',J1terms)

input('\n\nNow, let''s use LQR feedback control: [Hit Enter]')
figure(12); clf
y2 = initial(ss2,X0,t);

% Closed-loop poles are eigenvalues of "A-BK", for u=-K*x
closed_loop_poles1 = eig(A-B*K)

for n=1:4
    subplot(5,1,n)
    plot(t,y2(:,n),'b-');
    hold on
    
    ylabel(['X_',num2str(n)])
end
subplot(515)

% Line below calculates the CONTROL EFFORT, u:
u2 = -y2*K'; 

hold on
plot(t,u2,'r-'); % u is the actuation, assuming "u=-K*x" is the law
J2terms(5) = dt*sum(u2.^2);
for n=1:4
    J2terms(n) = dt*sum(y2(:,n).^2);
end
fprintf('\n');
fprintf('C1:  Q1:%5.3f   Q2:%5.3f   Q3:%5.3f   Q4:%5.3f     R:%5.3f  \n',...
    Q(1,1),Q(2,2),Q(3,3),Q(4,4),R(1))
fprintf('C1:  %8.3f   %8.3f   %8.3f   %8.3f     %8.3f  \n',J2terms)
J2add = J2terms.*[diag(Q)', R];
fprintf('J1 = %8.3f + %8.3f + %8.3f + %8.3f   + %8.3f = %12.8f \n',...
    J2add, sum(J2add));


% Set to 1 or 0, below, to tweak vs redefine R and Q...
if 0
    % try "tweaking" K:
    input('Now, let''s TWEAK the values in K: [Hit Enter]')
    Kfac = .7+.6*rand(1,4)
    Knew = Kfac.*K;
    Qnew = Q; Rnew = R;
else
    input('\n\nNow, let''s try LQR with same Q but larger R penalty: [Hit Enter]')
    Rnew
end

% Closed loop poles with Qnew and Rnew, below:
closed_loop_poles2 = eig(A-B*Knew)

figure(12); %clf
ss3 = ss(A-B*Knew,[],C,[]);
y3 = initial(ss3,X0,t);
for n=1:4
    subplot(5,1,n)
    plot(t,y3(:,n),'k-');
    Ax = axis;
    axis([0 15 Ax(3:4)])
end
legend('Smaller R','Larger R')

subplot(515)
u3 = -y3*Knew';
plot(t,u3,'k-');
Ax = axis;
axis([0 15 Ax(3:4)])
ylabel('u (control action)')
subplot(511)
title('u = -B*K, with K set via LQR')

fprintf('\n\nNote R penalizes control action. So, when R gets larger,\n');
fprintf('The system tends to use less control action, and the response\n');
fprintf('will often have more overshoot.  (Errors on STATE matter less\n');
fprintf('as compared with penalties on CONTROL, as R gets larger...)\n');
input('[Hit enter]');

J3terms(5) = dt*sum(u3.^2);
for n=1:4
    J3terms(n) = dt*sum(y3(:,n).^2);
end
fprintf('\n');
fprintf('C2:  Q1:%5.3f   Q2:%5.3f   Q3:%5.3f   Q4:%5.3f     R:%5.3f  \n',...
    Qnew(1,1),Qnew(2,2),Qnew(3,3),Qnew(4,4),Rnew(1))
fprintf('C2:  %8.3f   %8.3f   %8.3f   %8.3f     %8.3f  \n',J3terms)
J3add = J3terms.*[diag(Qnew)', Rnew];
fprintf('J2 = %8.3f + %8.3f + %8.3f + %8.3f   + %8.3f = %12.8f \n',...
    J3add, sum(J3add));

% Now, try pole placement
pdes = [-20 -30 -100 -200]
Kplace = acker(A,B,pdes)
ss4 = ss(A-B*Kplace,[],C,[]);
y4 = initial(ss4,X0,t);
u4 = y4*Kplace';

figure(13); clf
for n=1:4
    subplot(5,1,n)
    plot(t,y4(:,n),'b-');
    Ax = axis;
    axis([0 .5 Ax(3:4)])
end
subplot(515)
plot(t,u4,'r-');
Ax = axis;
axis([0 .5 Ax(3:4)])
ylabel('u: note scale!')
subplot(511)
title('u = -B*K, with K set via Pole Placement')

J4terms(5) = dt*sum(u4.^2);
for n=1:4
    J4terms(n) = dt*sum(y4(:,n).^2);
end
fprintf('\n');
fprintf('\n');
fprintf('C2:  Q1:%5.3f   Q2:%5.3f   Q3:%5.3f   Q4:%5.3f     R:%5.3f  \n',...
    Qnew(1,1),Qnew(2,2),Qnew(3,3),Qnew(4,4),Rnew(1))
fprintf('C2:  %8.3f   %8.3f   %8.3f   %8.3f     %8.3f  \n',J4terms)
J4add = J4terms.*[diag(Qnew)', Rnew];
fprintf('Jp = %8.3f + %8.3f + %8.3f + %8.3f   + %8.3f = %12.8f \n',...
    J4add, sum(J4add));


