clear;close all;clc


s = tf('s');
Ts = 1/100;
H_d = 1203/(s+40.98);
H_e = 1260/(s+41.88);

H_dz = c2d(H_d,Ts);
H_ez = c2d(H_e,Ts);

