clear; clc; close all;

Ts = 0.01; % tempo de amostragem
%Tsim = 30; % tempo de simulação 1curva
%Tsim = 25; % tempo de simulação 1
%Tsim = 25; % tempo de simulação 2
%Tsim = 100; % tempo de simulação 3

% tipos de trajetória de referência:
trajetoria = 1; Tsim = 25; % oito pequeno  v=0.1 w=0.5(-0.5;t=12.5)
%trajetoria = 1; Tsim = 12.5; % círculo pequeno  v=0.1 w=0.5(-0.5;t=12.5)
%trajetoria = 2; Tsim = 250; % oito grande   v=0.1 w=0.05(-0.05;t=1200)
%trajetoria = 3; Tsim = 60; % curva        v=0.1 w=0.05(-0.05;t=1200)
%trajetoria = 3; Tsim = 25; % quadrado      v=0.1 theta=+pi/2 (t cada 20s)
%trajetoria = 4; Tsim = 25; % senoite       v=?   w=?
%

% Parâmetros da estrutura física do robô % Parameters of the robot physic structure
r = 0.07; % Raio da roda
b = 0.2; % Comprimento do eixo entre rodas; % 2L no artigo
L = b/2;
a = 0.01; % distância entre o eixo e o centro de massa
m = 20; % peso do robô
Ic = m*(b/2)^2; % momento de inércia: m*(b/2)^2;

% PID dos motores
Kp=10;
Ki=0;
Kd=0;

% Condições iniciais %Initial Condition
xo = -0.5;
yo = -0.5;
thetao = -0.5;

% condições iniciais dos ganhos do controlador fixo
% Kx0 = 20;
% Ky0 = 60;
% Ktt0 = 10;
Kx0 = 20;
Ky0 = 60;
Ktt0 = 10;

sim('simulink_robo_TESTE'); % inicia o simulink
% variáveis de estado do controlador fixo
x_real  =q(:,1);
y_real  =q(:,2);
%Theta   =q(:,3);
% variáveis de estado do controlador adaptativo
x_real1 =q1(:,1);
y_real1 =q1(:,2);
%Theta1  =q1(:,3);

% plota os gráficos
plota1; % plota os gráficos dos controladores adaptativo e fixo.
%plota2; % plota ANIMAÇÃO.