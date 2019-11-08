clear
clc
close all

Ts=0.1;
% Parameters of the robot physic structure
r = 0.07; %raio da roda
b = 0.4; % comprimento do eixo entre rodas;

% Parameters of the DC motors and Speed Controller
J = 0.01 %kg.m^2  moment of inertia of the rotor     
b = 0.1; %N.m.s   motor viscous friction constant     
Ke = 0.08 % V/rad/sec    electromotive force constant       
Kt = 0.08 % N.m/Amp   motor torque constant    
R = 1 % Ohm electric resistance                
L = 0.5 %H  electric inductance                
%PID
Kp=5.0;
Ki=10.0;
Kd=0;

% Kinematic Controller
ksi = 0.3;
%ohmega_n = 1;
g = 40;

%Initial Condition
xo = 0.0
yo = -0.3;
thetao = 0.0;

Tsim = 37;
sim('simulink_robo')
plota
