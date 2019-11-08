% cálculo das matrizes
clear all, clc
% a = 1; % distância entre o eixo e o centro de massa
% m = 1000; % peso do robô
% Ic = 1; % momento de inércia
syms a m Ic theta R L

M_ = [m 0; 0 (m*a*a+Ic)]
V_ = [0 -m*a*theta; m*a*theta 0]
B_ = 1/R*[1 1; L -L]

syms u w
v = [u; w]

syms tau_r tau_l
tau = [tau_r; tau_l]

v_1 = -inv(M_)*V_*v
v_2 = inv(M_)*B_*tau
v_ = -inv(M_)*V_*v + inv(M_)*B_*tau
%v_ = integral(v_,0,t) % dt?