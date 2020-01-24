clear;close all;clc
load('Nanook_Vels.mat')

n = length(S_P);
left_adjust = 0.42;
rigth_adjust = 0.4137;

X = [S_P];
Y1 = M_E;
Y2 = M_D;

% Least Square
B_E= inv(X'*X)*X'*Y1;
B_D = inv(X'*X)*X'*Y2;

plot(S_P,Y1);
hold on
plot(S_P,Y2);

plot(S_P,X*B_E);
plot(S_P,X*B_D);

legend('Tac_e','Tac_d','LS_e','LS_d')




