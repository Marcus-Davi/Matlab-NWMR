clear; clc; close all;

Ts = 0.1; % tempo de amostragem

% Par�metros da estrutura f�sica do rob� % Parameters of the robot physic structure
r = 0.07; % Raio da roda
b = 0.2; % Comprimento do eixo entre rodas; % 2L no artigo
a = 0.0; % dist�ncia entre o eixo e o centro de massa
m = 10; % peso do rob�
Ic = 1; % momento de in�rcia: m*(b/2)^2;

% PID
Kp=0.05;
Ki=0;
Kd=0;

% Condi��es iniciais %Initial Condition
xo = -1.5;
yo = -2.5;
thetao = 0.0;

Tsim = 40; % tempo de simula��o
sim('simulink_robo_fixo_sem_motores') % inicia o simulink

plota % plota os gr�ficos
