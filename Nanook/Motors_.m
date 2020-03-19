clear;close all;clc


s = tf('s');
Ts = 1/100;
H_d = 1203/(s+40.98);
H_e = 1260/(s+41.88);

H_dz = c2d(H_d,Ts);
H_ez = c2d(H_e,Ts);


%% Kaio

H_dz_k = tf([9.8494 9.3951],[1 -0.3327 0.01],Ts);
H_ez_k = tf([8.3695 8.6422],[1 -0.3666 0.0064],Ts);

step(H_dz_k)
hold on
step(H_ez_k)
step(H_dz)
step(H_ez)

legend('Kaio D','Kaio E','My D','My E')