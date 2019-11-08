clear; clc; close all;

Ts = 0.1; % tempo de amostragem

% Parâmetros da estrutura física do robô % Parameters of the robot physic structure
r = 0.07; % Raio da roda
b = 0.2; % Comprimento do eixo entre rodas; % 2L no artigo
a = 0.0; % distância entre o eixo e o centro de massa
m = 10; % peso do robô
Ic = 1; % momento de inércia: m*(b/2)^2;

% PID
Kp=0.05;
Ki=0;
Kd=0;

% Condições iniciais %Initial Condition
xo = -1.5;
yo = -2.5;
thetao = 0.0;

Tsim = 40; % tempo de simulação
sim('simulink_robo_fixo_sem_motores') % inicia o simulink

plota % plota os gráficos
