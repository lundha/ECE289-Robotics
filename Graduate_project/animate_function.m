function animate_function(X)

xc = X(1);
theta1 = X(2);
theta2 = X(3);
L1 = 1.5;
L2 = 1.5;

figure(11); clf
plot([-5 5],[0 0],'k-'); hold on
plot(xc+[-1 1 1 -1 -1],[0 0 1 1 0 ],'k','LineWidth',2);
plot(xc+[0 L1*sin(theta1)],1+[0 L1*cos(theta1)],'r',...
    'LineWidth',2);
plot(xc+L1*sin(theta1)+[0 L2*sin(theta1+theta2)],...
    1+L1*cos(theta1)+[0 L2*cos(theta1+theta2)],'b',...
    'LineWidth',2);

axis image; % make x and y axes 1:1
axis([-5 5 -4 5])