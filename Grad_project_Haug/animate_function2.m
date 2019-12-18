%% Cart with two links

function animate_function2(X)

  xgo1 = X(1);
theta1 = X(2);
theta2 = X(3);

cgreen = [0 .7 0];

figure(17); clf
plot([-10 10],-1+[0 0],'k-','LineWidth',2); hold on
atemp = pi*[0:.02:1];
rt = .2;
xcart = [rt*cos(atemp), -1 -1 1 1 rt*cos(atemp(1))];
ycart = [rt*sin(atemp), 0 -.8 -.8 0 rt*sin(atemp(1))];
plot(xgo1+xcart,ycart,'b','LineWidth',2); hold on
axis image
rw = .2;
aw = -xgo1/rw;
av = 2*pi*[0:.01:1];
patch(xgo1-.7+rw*cos(av),-.8+rw*sin(av),.1+0*av,'w','EdgeColor','b',...
    'LineWidth',2)
patch(xgo1+.7+rw*cos(av),-.8+rw*sin(av),.1+0*av,'w','EdgeColor','b',...
    'LineWidth',2)

ra = .1;
L1 = 1.5;
x1 = [ra*cos(atemp), ra*cos(atemp+pi), ra*cos(atemp(1))];
y1 = [L1+ra*sin(atemp), ra*sin(atemp+pi), L1+ra*sin(atemp(1))];
a1 = atan2(y1,x1);
r1 = (x1.^2 + y1.^2).^.5;
plot(xgo1-r1.*cos(a1+theta1),r1.*sin(a1+theta1),'g',...
    'Color',cgreen,'LineWidth',2);

xm = xgo1+L1*sin(theta1);
ym = L1*cos(theta1);
plot(xm-r1.*cos(a1+theta1+theta2),ym+r1.*sin(a1+theta1+theta2),'r',...
    'LineWidth',2);

axis([-10 10 -5 5])
end