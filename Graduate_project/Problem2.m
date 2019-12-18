%% Problem 1

X0 = [0; 0; -0.2; 0; 0; 0];

[t, y_ol] = ode45(@dx_out, [0 5], X0');

for n=1:length(t)
    animate_function2(y_ol(n,1:3))
    drawnow
end






