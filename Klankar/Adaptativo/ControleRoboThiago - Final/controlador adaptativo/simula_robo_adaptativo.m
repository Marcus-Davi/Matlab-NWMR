clear
clc
close all

Ts = 0.1; % tempo de amostragem
% Parâmetros da estrutura física do robô % Parameters of the robot physic structure
r = 0.07; % Raio da roda
b = 0.2; % Comprimento do eixo entre rodas; % 2L no artigo

% Parameters of robot
a = 0.0; % distância entre o eixo e o centro de massa
m = 50; % peso do robô
Ic = 1;%m*(b/2)^2; % momento de inércia

% PID
Kp=0.05;%50;%5.0;
Ki=0;%0;%10.0;
Kd=0;

% Condições iniciais %Initial Condition
xo = 0;
yo = -0.5;
thetao = 0.0;

Tsim = 80%1588; %37 % tempo de simulação
%sim('simulink_robo_thiago') % inicia o simulink
sim('simulink_robo_adaptativo_sem_motores') % inicia o simulink

plota % plota os gráficos
