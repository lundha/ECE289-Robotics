function  dx = sys(t, X)
mc = 4;
L = 0.2;
m1 = 0.1;
m2 = m1;
J1 = 1.6e-04;
J2 = J1;
Rw = 0.05;
g  = -9.81;
F = 0;


xc = X(1);
theta1 = X(2);
theta2 = X(3);
dxc = X(4);
dtheta1 = X(5);
dtheta2 = X(6);

d2xc = (F + L*dtheta1^2*m1*theta1 + L*dtheta1^2*m2*theta1)/(m1 + m2 + mc); 
d2theta1 = (J2*L*g*m1*theta1 + J2*L*g*m2*theta1 + L^3*g*m1*m2*theta1*theta2^2)/(J1*J2 + J2*L^2*m1*theta1^2 + J1*L^2*m2*theta2^2 + J2*L^2*m2*theta1^2 + L^4*m1*m2*theta1^2*theta2^2);

d2theta2 = (J1*L*g*m2*theta2)/(J1*J2 + J2*L^2*m1*theta1^2 + J1*L^2*m2*theta2^2 + J2*L^2*m2*theta1^2 + L^4*m1*m2*theta1^2*theta2^2);


dx = [dxc; dtheta1; dtheta2; d2xc; d2theta1; d2theta2];

end