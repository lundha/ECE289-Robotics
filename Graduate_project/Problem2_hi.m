%% Problem 3

% Initial condition
X0 = [0; 0; 0.2; 0; 0; 0];

% Non-linear system
[t, y_cl_nonlin] = ode45(@output_dX, [0 10], X0');


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

figure(1); clf;

for i = 1:3
    plot(t2,y_cl_lin(:,i), '--');
    hold on
end

for i = 1:3
    plot(t,y_cl_nonlin(:,i));
    hold on
    hl = legend('linearized $x_{c}$','linearized $\theta_{1}$','linearized $\theta_{2}$', '$x_{c}$', '$\theta_{1}$', '$\theta_{2}$');
    set(hl, 'Interpreter', 'latex');
    set(gca,'fontsize',15)
end

figure(3); clf;

for i = 4:6
    plot(t2,y_cl_lin(:,i), '--');
    hold on
end

for i = 4:6
    plot(t,y_cl_nonlin(:,i));
    hold on
    hl = legend('linearized $\dot{x_{c}}$','linearized $\dot{\theta_{1}}$','linearized $\dot{\theta_{2}}$', '$\dot{x_{c}}$', '$\dot{\theta_{1}}$', '$\dot{\theta_{2}}$');
    set(hl, 'Interpreter', 'latex');
    set(gca,'fontsize',15)
end


