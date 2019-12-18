%% Cart with two links

% Initial position
xgo1 = 1; % Middle of cart position in X-direction
theta1 = 10*pi/180; % Angle of first pendulum
theta2 = -15*pi/180; % Angle of second pendulum

cgreen = [0 .9 0];

figure(17); clf

% The line the cart stands on
plot([-5 5],-1+[0 0],'k-','LineWidth',2); hold on
atemp = pi*[0:.02:1];
rt = .2;
xcart = [rt*cos(atemp), -1 -1 1 1 rt*cos(atemp(1))];
ycart = [rt*sin(atemp), 0 -.8 -.8 0 rt*sin(atemp(1))];

% Cart
plot(xgo1+xcart,ycart,'g','LineWidth',2); hold on
axis image
rw = .2;
aw = -xgo1/rw;
av = 2*pi*[0:.01:1];

% Wheel 1
patch(xgo1-.7+rw*cos(av),-.8+rw*sin(av),.1+0*av,'w','EdgeColor','r',...
    'LineWidth',2)

% Wheel 2
patch(xgo1+.7+rw*cos(av),-.8+rw*sin(av),.1+0*av,'w','EdgeColor','r',...
    'LineWidth',2)

ra = .1;
L1 = 1.5
x1 = [ra*cos(atemp), ra*cos(atemp+pi), ra*cos(atemp(1))];
y1 = [L1+ra*sin(atemp), ra*sin(atemp+pi), L1+ra*sin(atemp(1))];
a1 = atan2(y1,x1);
r1 = (x1.^2 + y1.^2).^.5;

% Pendulum 1
plot(xgo1+r1.*cos(a1+theta1),r1.*sin(a1+theta1),'g',...
    'Color',cgreen,'LineWidth',2);

xm = xgo1-L1*sin(theta1);
ym = L1*cos(theta1);

% Pendulum 2
plot(xm+r1.*cos(a1+theta2),ym+r1.*sin(a1+theta2),'r',...
    'LineWidth',2);

axis([-5 5 -1.2 6])