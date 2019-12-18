%% Problem 3 d

% Initial condition
X0 = [0; 0; 0.2; 0; 0; 0];

% Non-linear system
[t, y_cl_nonlin] = ode45(@dx_out_cl, [0 10], X0');


% Linearized system
dt = 0.001;
t2 = 0:dt:10;

sys = ss(A-B*K,[],C,[]);
y_cl_lin = initial(sys,X0',t2);

%% Animation 
for n=1:length(t)
    animate_function2(y_cl_nonlin(n,1:3))
    drawnow
end
%% Plot

% First three plots compares the first three states between the 
% linearized and non-linearized response
figure(1); clf;

subplot(311);
plot(t2,y_cl_lin(:,1), '--');
title('X_{0} close to equilibrium')
hold on
plot(t,y_cl_nonlin(:,1));
hl = legend('linearized $x_{c}$', '$x_{c}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('X position(m)');

subplot(312);
plot(t2,y_cl_lin(:,2), '--');
hold on
plot(t,y_cl_nonlin(:,2));
hl = legend('linearized $\theta_{1}$''$\theta_{1}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('Angle(rad)');

subplot(313);
plot(t2,y_cl_lin(:,3), '--');
hold on
plot(t,y_cl_nonlin(:,3));
hl = legend('linearized $\theta_{2}$','$\theta_{2}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('Angle(rad)');

% Last three plots compares the last three states between the 
% linearized and non-linearized response
figure(2); clf;

subplot(311);
plot(t2,y_cl_lin(:,4), '--');
title('X_{0} close to equilibrium')
hold on
plot(t,y_cl_nonlin(:,4));
hl = legend('linearized $\dot{x_{c}}$','$\dot{x_{c}}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('X position(m)');

subplot(312);
plot(t2,y_cl_lin(:,5), '--');
hold on
plot(t,y_cl_nonlin(:,5));
hl = legend('linearized $\dot{\theta_{1}}$','$\dot{\theta_{1}}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('Angle(rad)');

subplot(313);
plot(t2,y_cl_lin(:,6), '--');
hold on
plot(t,y_cl_nonlin(:,6));
hl = legend('linearized $\dot{\theta_{2}}$', '$\dot{\theta_{2}}$');
set(hl, 'Interpreter', 'latex');
set(gca,'fontsize',10)
xlabel('Time (s)');
ylabel('Angle(rad)');


