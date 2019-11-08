%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% mobile robot control on a reference path 
%%% Gregor Klancar, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reprodu��o do artigo, c�digo com ru�do
% 2016
%


%valores de entrada do controlador v e w (velocidades linear e angular)
%valores de sa�da posi��o 


clear all
close all
clc
format long g

Ts = 0.1;             % sampling time
Tsim = 85;
h = 0.1;           % integrating time

% Dados do Rob�
% Di�metro do Rob�
D = 0.4;
% Raio das rodas
raio =0.08;
% Dados do Rob� Real

% Dr = D;
% raio_e = raio;
% raio_d = raio;

Dr = 0.35;
raio_e = 0.1;
raio_d = 0.08;

x0 = [0; 0; 0];  % x,y,theta
X = x0;


xa = [0; 0; 0];  % x,y,theta
X1 = xa;

%definir a trajet�ria de refer�ncia 

for k = 1:round(Tsim/h)
    v(k) = 0.6;
    if k<= 419;
        %w(k) = 0.15;
        w(k) = 0.30;
    else
        %w(k) = -0.15;
        w(k) = -0.30;
    end
    
    k1 = h*[v(k)*cos(x0(3)); v(k)*sin(x0(3));w(k)];
    k2 = h*[v(k)*cos(x0(3)+k1(3)/2); v(k)*sin(x0(3)+k1(3)/2);w(k)];
    k3 = h*[v(k)*cos(x0(3)+k2(3)/2); v(k)*sin(x0(3)+k2(3)/2);w(k)];
    k4 = h*[v(k)*cos(x0(3)+k3(3)); v(k)*sin(x0(3)+k3(3));w(k)];
    
    X = [X, X(:,k) + (1/6)*(k1+2*k2+2*k3+k4)];
    x0 = X(:,k+1);
end

Xr = X;
Ur = [v;w];
plot(X(1,:),X(2,:),'r--')
hold on
grid

vr = v;
wr = w;

%os valores de posi��o inicial e ruido (m�dia e desvio padr�o)  s�o
%utilizados apenas para a simula��o 

%disp('Posi��o inicial')
%disp('[x;y;theta]')
%x0 = input('[x;y;theta] = ')%[1;-1; pi\4];  % x,y,theta
x0 = [1; -3; 0]
X = x0;
v = [];
w = [];


Xm = x0;

%coeficientes do controlador 
ksi = 0.6; %coeficiente de amortecimento - valor dado no artigo
ohmega_n = 2; %ohmega_n= sqrt(Ur2(t)^2+g*Ur1(t)^2); 

% disp('Par�metro do contolador')
% disp('g>0')
%g = input('g = ') %10; %grau de liberdade adicional para o controlador g>0
g = 10

% GERAR RU�DO:
%     randn
%      Generate values from a normal distribution with mean 1 and standard
%        deviation 2.
%           r = 1 + 2.*randn(100,1);

%disp('par�metros do ru�do')

% m = input('m�dia =  ') %0; 
% d = input('desvio padr�o = ')%.025;
m = 0;
d = 0;

r = m + d.*randn(1,round(Tsim/h));

% % Generate values from the uniform distribution on the interval [a, b].
% %           r = a + (b-a).*rand(100,1);
% 
% r = m + (d - m).*rand(1,round(Tsim/h));

r_max = max(abs(r));

disp('amplitude m�xima do sinal de ru�do')
r_max

ie0 = 0;
id0 = 0;

for k = 1:round(Tsim/h);
    
%valores utilizados para determinar os par�metros do controlador

    a1 = Xr(1,k)-x0(1);
    a2 = Xr(2,k)-x0(2);
    a3 = Xr(3,k)-x0(3);
    
    e1 = cos(x0(3))*a1+sin(x0(3))*a2;
    e2 = -sin(x0(3))*a1+cos(x0(3))*a2;
    e3 = a3;
    
    %controlador K
    
    k1 = 2*ksi*ohmega_n;
    k3 = k1;
    k2 = g*abs(vr(k)); 

    v1 = -k1*(e1);
    v2 = - sign(vr(k))*k2*(e2)-k3*e3;

    v(k) = vr(k)*cos(e3) - v1;
    w(k) = wr(k) -  v2;

    %restri��es
    %valores da velocidade m�x e min 
    % vsat = 0.8
    vsat = 0.8
    if v(k) >=vsat
        v(k)=vsat;
    end
    
    if v(k)< -vsat;
        v(k) = -vsat;
    end
    %wsat = 0.4;
    wsat = 0.8;
    if w(k)>= wsat;
        w(k) = wsat;
    end
    
    if w(k)<-wsat;
        w(k)=-wsat;
    end
    
    % Velocidade nas rodas
    
    ve(k) = v(k)-w(k)*D/2;
    vd(k) = v(k)+w(k)*D/2;
    
    we(k) = (ve(k)+ie0)/raio;
    wd(k) = (vd(k)+id0)/raio;
    
    ver(k) = raio_e*we(k);
    vdr(k) = raio_d*wd(k);
    
    eve(k) = ve(k) - ver(k);
    evd(k) = vd(k) - vdr(k);
    
    ki = 1;
    ie(k) = ie0+ki*eve(k);
    id(k) = id0+ki*evd(k);
    ie0 = ie(k);
    id0 = id(k);
    
    
    wr(k) = (vdr(k)-ver(k))/Dr;
    vr(k) = (ver(k)+vdr(k))/2;

    %definir os novos valores da posi��o e orienta��o a partir dos valores
    %de velocidade
    
    Xm(1,1) = x0(1) + Ts*vr(k)*cos(x0(3));
    Xm(2,1) = x0(2) + Ts*vr(k)*sin(x0(3));
    Xm(3,1) = x0(3) + Ts*wr(k);
    
    
    %inserir ruido de medi��o
    Xm = Xm + r(k);
    
    X = [X,Xm];
    x0 =Xm;
    
    
    %x0 = X(:,k+1);
    
    Erro(:,k) = Xr(:,k+1)- X(:,k+1);
    k
end

%gr�fico trajet�ria refer�ncia e a trajet�ria do rob�

plot(X(1,:),X(2,:),'m')
legend('refer�ncia','trajet�ria do rob�')
xlabel('x[m]')
ylabel('y[m]')

% valores da velocidade do rob�

figure
ne = length(Erro);
tempo = Ts*(0:ne-1);
subplot(2,1,1)
plot(tempo,v,'g')
hold on
plot(tempo,vr);
legend('v','vr')
grid
hold on
subplot(2,1,2)
plot (tempo,w,'g');
hold on
plot(tempo,wr)
legend('w','wr')
grid
figure

% erros

plot(tempo,Erro(1,:),tempo,Erro(2,:),tempo,Erro(3,:));
legend('x','y','theta')

 E1 = Erro(1,:);
 E2 = Erro(2,:);
 E3 = Erro(3,:);
grid

figure
plot(tempo,ve,tempo,vd)
legend('ve','vd')

figure(1)
commandwindow
