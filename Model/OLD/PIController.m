%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% mobile robot control on a reference path
%%% Gregor Klancar,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modificado por Marcus
% 28.12.2017
%Assumir o sistema com referencias constantes de velocidade !!!
clear all;close all;clc
Ts = 0.1;
ur1 = 1; %v_r feedforward
ur2 = 0; %w_r feedforard
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = zeros(3,2);
SYS = ss(A,B,C,D);
%% SVD

%%
s = tf('s');
M = C*inv((s*eye(3)-A))*B;
fc = 0.2;
Mo = evalfr(M,fc);
Mo_ = [Mo(2,1) 0;0 Mo(3,2)];
RGA1 = Mo_.*(inv(Mo_)');
[U,S,V] = svd(Mo);

PID = 100*tf([0.5 1],[1 0]); %sisotool
PID2 = 10*tf([1 5],[0.1 1]); %sisotool

%PID =  13.257*(1+1.1*s+(0.72*s)^2)/(s^2);
%PID2 = 1*tf([0 1],[0 1]);

% GET NUMERATOR
[NUM,DEN] = tfdata(PID,'v');
REF = tf(1,NUM);
REF = REF/evalfr(REF,0);

[NUM,DEN] = tfdata(PID2,'v');
REF2 = tf(1,NUM);
REF2 = REF2/evalfr(REF2,0);
%CLOSE LOOP
MF = REF*feedback(PID*M(1,1),1);
MF2 = REF2*feedback(PID2*M(2,2),1);



Csvd = [PID 0 0;0 PID2 0];
Ksvd = V*Csvd*U';


Krga = [2.3 0 0;0 2.3 PID]; %VERIFICAR DIMENSAO
Krga = [2.23606797749979 0 0;0 2.23606797749979 0.739738869804715]; %LQR
Kref = [1 0 0;0 1 0; 0 0 1]; %FILTRO

K_PI = Krga;
save("PIController.mat",'K_PI');
load('GLOBAL_REF.mat');
load('GLOBAL_FEEDFORWARD_REF.mat');
%% SIM & PLOTS
sim('PISim');
plot(XrSIM.data(:,1),XrSIM.data(:,2)); %REFERENCIA
hold on;
plot(posture.data(:,1),posture.data(:,2))