clear;close all;clc

%x_ = Ax + B1w + B2u
%z = C1x + D11w + D12u
%y = C2x + D21w + D22u

%NORMA Minimiza Tzw

%Velcidades de feedforward politópicas
vr_min = 0.1; 
vr_max = 0.3;
wr_min = 0.0;
wr_max = 0.0;

%Politopos
A0 = [0 wr_min 0;-wr_min 0 vr_min;0 0 0]; % LPV
A1 = [0 wr_max 0;-wr_max 0 vr_max;0 0 0]; % LPV
 
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

nx = length(A0);
[~,nb] = size(B2); %n de colunas de Bn = n_entradas
[nc,~] = size(C2); %n de linhas de Cn = n_saidas
[~,nb1] = size(B1);
N = 2; %Vertices

%Especificações de D-estabilidade
h1 = -1; %bom valor = -1.5
%h2 = -5000; %reta 2 desnecessária por causa do circulo
% raio = 2*pi/(10*Ts); % =~ 125000
raio = 2; %bom valor = 3
h2 = -raio; %apenas para plot
theta = deg2rad(30); %bom valor = 15, 25
%% LPV - State Feedback
disp('LPV - State Feedback');

Ai{1} = A0;
Ai{2} = A1;

W = sdpvar(nx,nx); %W único

Y0 = sdpvar(nb,nx,'full');   
Y1 = sdpvar(nb,nx,'full');     
    
    
    Reta0 =   (A0*W + W*A0' - B2*Y0 - Y0'*B2' - 2*h1*W) <= 0;
    Reta1 =   (A1*W + W*A1' - B2*Y1 - Y1'*B2' - 2*h1*W) <= 0;

    Cone0 = [sin(theta)*(A0*W + W*A0' - B2*Y0 - Y0'*B2') cos(theta)*(A0*W - W*A0' - B2*Y0 + Y0'*B2');
            cos(theta)*(W*A0' - A0*W + B2*Y0 - Y0'*B2') sin(theta)*(A0*W + W*A0' - B2*Y0 - Y0'*B2')] <= 0;
        
    Cone1 = [sin(theta)*(A1*W + W*A1' - B2*Y1 - Y1'*B2') cos(theta)*(A1*W - W*A1' - B2*Y1 + Y1'*B2');
            cos(theta)*(W*A1' - A1*W + B2*Y1 - Y1'*B2') sin(theta)*(A1*W + W*A1' - B2*Y1 - Y1'*B2')] <= 0;
        
    Circulo0 = [-raio*W W*A0'-Y0'*B2';
               (W*A0'-Y0'*B2')' -raio*W] <= 0;
    
    Circulo1 = [-raio*W W*A1'-Y1'*B2';
               (W*A1'-Y1'*B2')' -raio*W] <= 0;
               
    LMI = [W>=0,Reta0,Reta1,Cone0,Cone1,Circulo0,Circulo1]; %concatena tudo

% opt = sdpsettings('solver','sdpt3');
optimize(LMI,'')

K0 = value(Y0)/value(W)
K1 = value(Y1)/value(W)
K_LPV_Wu{1} = K0;
K_LPV_Wu{2} = K1;
% return
%% Plot D-Região 
close all
figure; hold on; %Plota D-Região
for i=1:N %Numero de vértices
CL = ss(Ai{i}-B2*K_LPV_Wu{i},B2,C2,D22);
eig(CL)
pzmap(CL);
end
r_color = 'red';
YLIM = raio;
xc = [0 h2];
yc = xc/cos(theta);
plot(xc,yc,r_color);
plot(xc,-yc,r_color);
plot([h1 h1],[-YLIM YLIM],r_color)
plot([h2 h2],[-YLIM YLIM],r_color)
xcirc = 0:-raio/100:-raio;
ycirc = sqrt(raio^2-xcirc.^2);
plot(xcirc,ycirc,r_color)
plot(xcirc,-ycirc,r_color)
xlim([h2+h2/10 0])
ylim([-YLIM YLIM])
grid;
