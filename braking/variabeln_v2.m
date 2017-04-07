%% Plant
m=800;
g=9.81;
Fz= m*g/2;
r=0.25;
J=1;
theta1=1.28;
theta2=23.99;
theta3=0.52;
lambda_target=0.1;

%0.5*r^2 *14
% J=1.4 /1.6
%mu=theta1*(1-exp(-lamda*theta2))-lamda*theta3

v0=50;
%% Controller

kp=1000;
Ti=1;
Td=0;



