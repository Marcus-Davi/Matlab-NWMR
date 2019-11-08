%% Simulação situação real.
clear all
close all
clc             

Ts = 0.01;
H_Direito = tf([30.43], [0.01786 1]) %plantas levantadas com ensaio
H_Esquerdo = tf([-30.19], [0.01964 1])

Hd_Direito = c2d(H_Direito,Ts)
Hd_Esquerdo = c2d(H_Esquerdo,Ts)

%% Planta Desejada & PI modelo SRT
K = 1;
Wn = 90;
Ksi = 0.8;
[W,QSI] = omega_dmp(0.1,1);
%D = tf(K*[Wn^2],[1 2*Ksi*Wn Wn^2]);
D = tf(K*[W^2],[1 2*QSI*W W^2]);
Dd = c2d(D,Ts);
[zer,poles] = tfdata(Dd,'v');
%motor direito
[b,a] = tfdata(Hd_Esquerdo,'v');
r0_Esquerdo = (poles(2)-a(2)+1)/b(2);
r1_Esquerdo = (poles(3)+a(2)  )/b(2);
%moto esquerdo
[b,a] = tfdata(Hd_Direito,'v');
r0_Direito = (poles(2)-a(2)+1)/b(2);
r1_Direito = (poles(3)+a(2)  )/b(2);
%Controladores
R_Direito = tf([r0_Direito r1_Direito],1,Ts);
S_Direito = tf([1],[1 -1],Ts);
%T_Direito = tf([r0_Direito r1_Direito],1,Ts);
T_Direito = r0_Direito + r1_Direito; %Eliminacao de zeros

R_Esquerdo = tf([r0_Esquerdo r1_Esquerdo],1,Ts);
S_Esquerdo = tf([1],[1 -1],Ts);
%T_Esquerdo = tf([r0_Esquerdo r1_Esquerdo],1,Ts);
T_Esquerdo = r0_Esquerdo + r1_Esquerdo; %Eliminacao de zeros

%Malhas Fechadas
CL_Direito = T_Direito*feedback(S_Direito*Hd_Direito,R_Direito);
CL_Esquerdo = T_Esquerdo*feedback(S_Esquerdo*Hd_Esquerdo,R_Esquerdo);




step(30*Dd)
hold on

step(Hd_Direito)
step(30*CL_Direito)

%step(Hd_Esquerdo)
%step(30*CL_Esquerdo)
legend('Planta Desejada','Malha Aberta','Malha Fechada')

%%
%Simulacao em Microcontrolador / Raspberry PI
%u(k) = u(k-1) +R*y(k) + T*r(k)
Amostras = 50;
y_Esquerdo = 0;  
y_Direito = 0;
u_Direito=0;
time =0;
k = 0;
%Variaveis de Controle
u_k_Esquerdo = 0;
u_k1_Esquerdo = 0;
e_k_Esquerdo = 0;
e_k1_Esquerdo = 0;
y_k_Esquerdo = 0;
y_k1_Esquerdo = 0;
%--------------------
u_k_Direito = 0;
u_k1_Direito = 0;
e_k_Direito = 0;
e_k1_Direito = 0;
y_k_Direito = 0;
y_k1_Direito = 0;
%Variaveis de malha aberta
Y_Esquerdo=0;
Y_K_Esquerdo=0;
Y_K1_Esquerdo=0;
U_K_Esquerdo=0;
U_K1_Esquerdo=0;
%-------------------%
Y_Direito=0;
Y_K_Direito=0;
Y_K1_Direito=0;
U_K_Direito=0;
U_K1_Direito=0;
%-------------------

%Referencias
r_k_Esquerdo = 60;
r_k1_Esquerdo=0;
%---------------
r_k_Direito = 60;
r_k1_Direito=0;

for i = 0:Amostras
    
r_k_Esquerdo = 10; %RPM
r_k_Direito = 15; %RPM
%MALHA FECHADA
y_k_Esquerdo = 0.601*y_k1_Esquerdo -12.05*u_k1_Esquerdo; %PLANTA DISCRETIZADA
y_k_Direito = 0.5713*y_k1_Direito +13.05*u_k1_Direito; %PLANTA DISCRETIZADA


% u_k_Esquerdo = u_k1_Esquerdo - r0_Esquerdo*y_k_Esquerdo - r1_Esquerdo*y_k1_Esquerdo + r0_Esquerdo*r_k_Esquerdo + r1_Esquerdo*r_k1_Esquerdo;  %controlador PI RST
% u_k_Direito   = u_k1_Direito - r0_Direito*y_k_Direito - r1_Direito*y_k1_Direito + r0_Direito*r_k_Direito + r1_Direito*r_k1_Direito;  %controlador PI RST

u_k_Esquerdo = u_k1_Esquerdo - r0_Esquerdo*y_k_Esquerdo - r1_Esquerdo*y_k1_Esquerdo + (r0_Esquerdo+r1_Esquerdo)*r_k_Esquerdo;  %Eliminacao de zeros
u_k_Direito   = u_k1_Direito - r0_Direito*y_k_Direito - r1_Direito*y_k1_Direito + (r0_Direito+r1_Direito)*r_k_Direito;  %Eliminacao de zeros




%Atualizacao
y_k1_Esquerdo=y_k_Esquerdo;
u_k1_Esquerdo=u_k_Esquerdo;
r_k1_Esquerdo=r_k_Esquerdo;
e_k1_Esquerdo = e_k_Esquerdo;

y_k1_Direito=y_k_Direito;
u_k1_Direito=u_k_Direito;
r_k1_Direito=r_k_Direito;
e_k1_Direito = e_k_Direito;


%MALHA ABERTA
U_K_Esquerdo = 1;
Y_K_Esquerdo = 0.601*Y_K1_Esquerdo -12.05*U_K1_Esquerdo;
U_K1_Esquerdo = U_K_Esquerdo;
Y_K1_Esquerdo = Y_K_Esquerdo;

U_K_Direito = 1;
Y_K_Direito = 0.5713*Y_K1_Direito +13.05*U_K1_Direito;
U_K1_Direito = U_K_Direito;
Y_K1_Direito = Y_K_Direito;

u_Direito = [u_Direito u_k_Direito];
y_Esquerdo = [y_Esquerdo y_k_Esquerdo];
Y_Esquerdo = [Y_Esquerdo Y_K_Esquerdo];
y_Direito = [y_Direito y_k_Direito];
Y_Direito = [Y_Direito Y_K_Direito];
time = [time k];
k=k+Ts;
end
hold on
% plot(time,y_Esquerdo,'r'); %MF
% plot(time,Y_Esquerdo,'b'); %MA
% plot(time,y_Direito,'m'); %MA
% plot(time,Y_Direito,'k'); %MF
plot(time,u_Direito,'b') %Esforco
legend('Malha Fechada - M_E','Malha Aberta - M_E','Malha Fechada - M_D','Malha Aberta - M_D','Esforco de Controle')

