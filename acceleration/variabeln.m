clear variables
clc

%% Transfer Function Torque to lambda
m=700;
g=9.81;
r=0.25;
J=1;
theta1=0.86; %wet asphalt conditions
theta2=33.82;
theta3=0.35;
v0=1;
gear_ratio=7.13; %Getriebeübersetzung
v_max=100/3.6; %Höchstgeschwindigkeit
power_engine=45000; %Leistung Motor --> noch nachfragen

h_CoM=0.5; %Hight Center of Mass
l_f=1.5; 
l_r=1.5;
    l=l_f+l_r; %Wheelbase
    
cw=0.3;
A=2.5;
rho=1.2;
    W_luft=0.5*cw*A*rho; % Airdrag
    

%% Transfer function voltage to torque

a=31.111;
b=-12.44444; %f(x)=ax+b
saturation_voltage_high=4.9;
saturation_voltage_low=0.4;

%% Matrix A, B

Ta0=800;
lambda_0=0.1;
w0=200;
Fz0=450*g;

a1= Ta0/(J*w0)-Fz0*r/(J*w0)*(theta1*theta2*exp(-lambda_0*theta2)-theta3)+Fz0*r/(J*w0)*(theta1*(1-exp(-lambda_0*theta2)-lambda_0*theta2*exp(-lambda_0*theta2))-2*lambda_0*theta3)-Fz0/(m*r*w0)*(theta1*theta2*exp(-lambda_0*theta2)-theta3);

b1=1/(J*w0)-lambda_0/(J*w0);



%% Controller LQR

A= a1;

B= b1;


Q=1000;

R=0.01;

lambda_target=0.1;


K_lqr= lqr(A,B,Q,R);
K_i=10000;



 
max_torque=140; %Max. Drehmoment Motor
saturation_torque_high=max_torque*gear_ratio;




%% 

sim('Acceleration',5)
Simulink.sdi.view



