%clear variables
clc

%% Transfer Function Torque to lambda
m=700; %600
g=9.81;
r=0.3; %0.3
J=1; %1

% 
% theta1=1.55; %eigene Bedingungen
% theta2=28.55;
% theta3=0.6;
%  


theta1=1.28; %dry asphalt conditions
theta2=23.99;
theta3=0.52;

% theta1=0.86; %wet asphalt conditions
% theta2=33.82;
% theta3=0.35;
 
%  theta1=0.19; %snow conditions
%  theta2=94.13;
%  theta3=0.06;

v0=0.1;
gear_ratio=7.13; %Getriebeübersetzung
torque_max_engine=140; %Maximales Drehmoment vom Motor
torque_max_onwheel=torque_max_engine*gear_ratio/2; %Maximales Moment je Hinterrad
v_max=100/3.6; %Höchstgeschwindigkeit
power_engine=45000; %Leistung Motor --> noch nachfragen

h_CoM=0.5; %Hight Center of Mass
l_f=1.5; %1.5
l_r=1;   %1.5
    l=l_f+l_r; %Wheelbase
    
cw=0.3; %0.3
A=2.5;
rho=1.2;
    W_luft=0.5*cw*A*rho; % Airdrag
    
f_r=0.05; % wheel resistance coeff. 0.05
F_rtot=f_r*m*g; %wheel resistance of all wheels
    

%% Transfer function voltage to torque

a=31.111;
b=-12.44444; %f(x)=ax+b
saturation_voltage_high=4.9;
saturation_voltage_low=0.4;

%% Matrix A, B

Ta0=800;
lambda_0=0.1;
w0=200;
Fz0=400*g;

a1= Ta0/(J*w0)-Fz0*r/(J*w0)*(theta1*theta2*exp(-lambda_0*theta2)-theta3)+Fz0*r/(J*w0)*(theta1*(1-exp(-lambda_0*theta2)-lambda_0*theta2*exp(-lambda_0*theta2))-2*lambda_0*theta3)-Fz0/(m*r*w0)*(theta1*theta2*exp(-lambda_0*theta2)-theta3);

b1=1/(J*w0)-lambda_0/(J*w0);



%% Controller LQR

A= a1;

B= b1;


Q=100;

R=0.01;

lambda_target=0.1;


K_lqr= lqr(A,B,Q,R);

 
max_torque=140; %Max. Drehmoment Motor
saturation_torque_high=max_torque*gear_ratio;
%%

x = [0,0 ; 250,1050 ; 250,1051 ; 1173,3050 ; 1173 3150];
[~,idx] = unique(x(:,1));
out = x(idx,:)


%% Model Verification 
size_vector=2013; %grösse der importierten Daten
sampling_frequency=1000;
sampling_time= 1/sampling_frequency; %1/sampling_frequency;

voltage_import=cutrun5bearbeitet(:,2);
voltage_measured.time = 0:sampling_time:(size_vector-1)*sampling_time;
voltage_measured.signals.values = voltage_import;
voltage_measured.signals.dimensions = 1;


velocity_front_import=cutrun5bearbeitet(:,4);

velocity_front_measured.time=0:sampling_time:(size_vector-1)*sampling_time;
velocity_front_measured.signals.values = velocity_front_import;
velocity_front_measured.signals.dimensions = 1;

velocity_rear_import=cutrun5bearbeitet(:,3);

velocity_rear_measured.time=0:sampling_time:(size_vector-1)*sampling_time;
velocity_rear_measured.signals.values = velocity_rear_import;
velocity_rear_measured.signals.dimensions = 1;


 
 lambda_import = cutrun5bearbeitet(:,1);
 
 lambda_measured.time = 0:sampling_time:(size_vector-1)*sampling_time;
 lambda_measured.signals.values = lambda_import;
 lambda_measured.signals.dimensions = 1;








%sim('Acceleration',60)
%Simulink.sdi.view




