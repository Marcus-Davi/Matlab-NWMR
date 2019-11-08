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
  
%% Nominal - State Feedback
disp('Nominal - State Feedback');
%k : nu x nx
%y = KX : nu x nx X nx x nx = nu x nx

W = sdpvar(nx,nx);
Y = sdpvar(nb,nx)
gamma = sdpvar(1); %CUIDADO COM GAMMA

A = A0;

Reta1 =   (A*W + W*A' - B2*Y - Y'*B2' - 2*h1*W) <= 0;

% Reta2 =   (A_a*W + W*A_a' - B2_a*Y - Y'*B2_a' - 2*h2*W) > 0;

Cone = [sin(theta)*(A*W + W*A' - B2*Y - Y'*B2') cos(theta)*(A*W - W*A' - B2*Y + Y'*B2');
        cos(theta)*(W*A' - A*W + B2*Y - Y'*B2') sin(theta)*(A*W + W*A' - B2*Y - Y'*B2')] <= 0;

Circulo = [-raio*W W*A'-Y'*B2';
           (W*A'-Y'*B2')' -raio*W] <= 0;

% REVISAR LIMS
H_inf = [W*A'+A*W-B2*Y-Y'*B2' B1 W*C1'+Y'*D12';
    B1' -gamma*eye(1) D11';
    (W*C1'+Y'*D12')' D11 -gamma*eye(3)] <= 0;


LMI = [W >= 0,Reta1,Cone,Circulo];
% LMI = [W >= 0, LQR_LMI >= 0]

optimize(LMI)
K_LPV_n = value(Y)/(value(W));
%% LPV - State Feedback
disp('LPV - State Feedback');

Ai{1} = A0;
Ai{2} = A1;

W = sdpvar(nx,nx); %W único

Yi{1} = sdpvar(nb,nx,'full');   
Yi{2} = sdpvar(nb,nx,'full');   

%K =     beta1*K1 + beta2*K2 + beta3*K3 + beta4*K4


LMI = [W >= 0];
% gamma = sdpvar(1); % gamma Único
for i=1:N %loop pra cada vértice
    
    
    
    
    Reta1 =   (Ai{i}*W + W*Ai{i}' - B2*Yi{i} - Yi{i}'*B2' - 2*h1*W) <= 0;

%     Reta2 =   (A_a*W + W*A_a' - B2_a*Yi{i} - Yi{i}'*B2_a' - 2*h2*W) > 0;

    Cone = [sin(theta)*(Ai{i}*W + W*Ai{i}' - B2*Yi{i} - Yi{i}'*B2') cos(theta)*(Ai{i}*W - W*Ai{i}' - B2*Yi{i} + Yi{i}'*B2');
            cos(theta)*(W*Ai{i}' - Ai{i}*W + B2*Yi{i} - Yi{i}'*B2') sin(theta)*(Ai{i}*W + W*Ai{i}' - B2*Yi{i} - Yi{i}'*B2')] <= 0;

    Circulo = [-raio*W W*Ai{i}'-Yi{i}'*B2';
               (W*Ai{i}'-Yi{i}'*B2')' -raio*W] <= 0;
    
    % REVISAR LIMS
%     H_inf = [W*A'+A*W-B2*Y-Y'*B2' B1 W*C1'+Y'*D12';
%     B1' -gamma*eye(1) D11';
%     (W*C1'+Y'*D12')' D11 -gamma*eye(3)] <= 0;
    
    LMI = [LMI,Reta1,Cone,Circulo]; %concatena tudo
end

optimize(LMI)
for i=1:N
    K_LPV_Wu{i} = value(Yi{i})/(value(W));
end

%% LPV - State Feedback - Multiple W (MENOS CONSERVADOR)
disp('LPV - State Feedback - Multiple W');

for i=1:N
    
    W = sdpvar(nx,nx);
    Y = sdpvar(nb,nx);
    gamma = sdpvar(1);    %CUIDADO COM GAMMA
    A = Ai{i};
    
    Reta1 =   (A*W + W*A' - B2*Y - Y'*B2' - 2*h1*W) <= 0;
    
%     Reta2 =   (A_a*W + W*A_a' - B2_a*Y - Y'*B2_a' - 2*h2*W) > 0;
    
    Cone = [sin(theta)*(A*W + W*A' - B2*Y - Y'*B2') cos(theta)*(A*W - W*A' - B2*Y + Y'*B2');
        cos(theta)*(W*A' - A*W + B2*Y - Y'*B2') sin(theta)*(A*W + W*A' - B2*Y - Y'*B2')] <= 0;
    
    Circulo = [-raio*W W*A'-Y'*B2';
        (W*A'-Y'*B2')' -raio*W] <= 0;
    
    % REVISAR LIMS
    H_inf = [W*A'+A*W-B2*Y-Y'*B2' B1 W*C1'+Y'*D12';
    B1' -gamma*eye(1) D11';
    (W*C1'+Y'*D12')' D11 -gamma*eye(3)] <= 0;
    
    
    LMI = [W >= 0,Reta1,Cone,Circulo];
    optimize(LMI)
    K_LPV_Wm{i} = value(Y)/(value(W));
    Yi2{i} = value(Y);
    Wi{i} = value(W);
end


Controlador_LPV = struct('K1',K_LPV_Wm{1},'K2',K_LPV_Wm{2},'Vmax',vr_max,'Vmin',vr_min);
save('Controlador_LPV','Controlador_LPV');
disp('Controladores salvos em "Controladores.mat"');
disp(Controlador_LPV.K1)
disp(Controlador_LPV.K2)
%Código daqui pra baixo é manual
% return
%% Plot D-Região 
close all
figure; hold on; %Plota D-Região
for i=1:N %Numero de vértices
CL = ss(Ai{i}-B2*K_LPV_Wu{i},B2,C2,D22);
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
