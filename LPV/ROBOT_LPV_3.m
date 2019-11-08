clear;close all;clc

%x_ = Ax + B1w + B2u
%z = C1x + D11w + D12u
%y = C2x + D21w + D22u

%NORMA Minimiza Tzw

%Velcidades de feedforward politópicas
vr_min = 0.2; 
vr_max = 0.5;
wr_min = 0.0;
wr_max = 0.0;

rho = sdpvar(1);

%Politopos
A = [0 0 0;0 0 rho;0 0 0];
 
B2 = [1 0;0 0;0 1];
B1 = [0 0 0]'; 
C2 = eye(3);
C1 = C2;
D12 = [0 0;0 0;0 0];
D22 = D12;
D11 = [0 0 0]';
D21 = D11;
%LQR ?
% Q = diag([1 50 0]);
% R = eye(2);
%% D - Estabilidade

nx = length(A);
[~,nb] = size(B2); %n de colunas de Bn = n_entradas
[nc,~] = size(C2); %n de linhas de Cn = n_saidas
[~,nb1] = size(B1);
N = 2; %Vertices

%Especificações de D-estabilidade
h1 = -1.7; %bom valor = -1.5
raio = 5; %bom valor = 3
h2 = -raio; %apenas para plot
theta = deg2rad(25); %bom valor = 15, 25
%% LPV - State Feedback
disp('LPV - State Feedback');


% gamma = sdpvar(1);

Y0 = sdpvar(nb,nx,'full');   
Y1 = sdpvar(nb,nx,'full');     
Y = Y0 + rho*Y1;      
W = sdpvar(nx,nx); %W único

Reta =   (A*W + W*A' - B2*Y - Y'*B2' - 2*h1*W) <= 0;

% Reta2 =   (A*W + W*A' - B2*Y - Y'*B2' - 2*h2*W) >= 0;
    

Cone = [sin(theta)*(A*W + W*A' - B2*Y - Y'*B2') cos(theta)*(A*W - W*A' - B2*Y + Y'*B2');
        cos(theta)*(W*A' - A*W + B2*Y - Y'*B2') sin(theta)*(A*W + W*A' - B2*Y - Y'*B2')] <= 0;
        
       
Circulo = [-raio*W A*W-B2*Y;
            W*A'-Y'*B2' -raio*W] < 0;
        
% H_inf = [W*A'+A*W-B2*Y-Y'*B2' B1 W*C1'+Y'*D12';
%         B1' -gamma*eye(1) D11';
%         (W*C1'+Y'*D12')' D11 -gamma*eye(3)] <= 0;
    
              
LMI = [W>=0,Reta,Circulo,Cone, vr_min <= rho <= vr_max,uncertain(rho)]

% opt = sdpsettings('solver','SDPT3');
optimize(LMI)

K0 = value(Y0)/value(W);
K1 = value(Y1)/value(W);

Controlador_LPV = struct('K0',K0,'K1',K1,'Vmax',vr_max,'Vmin',vr_min);
save('Controlador_LPV','Controlador_LPV');
disp('Controladores salvos em "Controladores.mat"');
disp(Controlador_LPV.K0)
disp(Controlador_LPV.K1)
%K = K0 + rho*K1;
% return
%% Plot D-Região 
close all
figure; hold on; %Plota D-Região
RHO{1} = vr_min;Ai{1} = [0 0 0;0 0 vr_min;0 0 0];
RHO{2} = vr_max;Ai{2} = [0 0 0;0 0 vr_max;0 0 0];
EIG = [];
for i=1:N %Numero de vértices
K = K0 + RHO{i}*K1;
CL = ss(Ai{i}-B2*K,B2,C2,D22);
EIG = [EIG eig(CL)];
disp('Autovalores(Polos) Vertice');disp(i-1);disp(eig(CL));
pzmap(CL);
end
r_color = 'red';
YLIM = max([max(imag(EIG)) raio])*1.1;
XLIM = min([min(real(EIG)) h2])*1.1;
xc = [0 h2];
yc = xc*tan(theta);
plot(xc,yc,r_color);
plot(xc,-yc,r_color);
plot([h1 h1],[-YLIM YLIM],r_color)
plot([h2 h2],[-YLIM YLIM],r_color)
xcirc = 0:-raio/100:-raio;
ycirc = sqrt(raio^2-xcirc.^2);
plot(xcirc,ycirc,r_color)
plot(xcirc,-ycirc,r_color)
xlim([XLIM 0])
ylim([-YLIM YLIM])
grid;
