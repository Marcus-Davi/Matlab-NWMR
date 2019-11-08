%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% mobile robot control on a reference path 
%%% Gregor Klancar, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% reprodução do artigo, código com ruído
% 29.12.12


clear all
close all
clc
format long g

Ts = 0.1;             % sampling time
Tsim = 85;
h = 0.1;           % integrating time


x0 = [0; 0; 0];  % x,y,theta
X = x0;


xa = [0; 0; 0];  % x,y,theta
X1 = xa;

% Runge kutta

% Y(n+1) = Y(n) + (1/6)(k1 +2 k2 +2 k3 +k4);
% k1 = h f(X(n),Y(n))
% k2 = h f(X(n)+h/2, Y(n)+k1/2)
% k3 = h f(X(n)+h/2, Y(n) +k2/2)
% k4 = h f(X(n)+h, Y(n)+k3)

% The next loop difines the reference


for k = 1:round(Tsim/h)
    v(k) = 0.3;
    if k<= 419;
        w(k) = 0.15;
     else
         w(k) = -0.15;
   end
    
    k1 = h*[v(k)*cos(x0(3)); v(k)*sin(x0(3));w(k)];
    k2 = h*[v(k)*cos(x0(3)+k1(3)/2); v(k)*sin(x0(3)+k1(3)/2);w(k)];
    k3 = h*[v(k)*cos(x0(3)+k2(3)/2); v(k)*sin(x0(3)+k2(3)/2);w(k)];
    k4 = h*[v(k)*cos(x0(3)+k3(3)); v(k)*sin(x0(3)+k3(3));w(k)];
    
    X = [X, X(:,k) + (1/6)*(k1+2*k2+2*k3+k4)];
    x0 = X(:,k+1);
    
    X1 = [X1, X1(:,k)+ h*([v(k)*cos(xa(3));v(k)*sin(xa(3));w(k)])];
    xa = X1(:,k+1);
    

end

Xref = ones(3,851); %NAO FUNCIONA
for k = 1:round(Tsim/h)
    Xref(1,k)=2;
    Xref(2,k)=2;
    Xref(3,k)=0;
end

Xr = X;
%Xr = Xref;
Ur = [v;w];
plot(X(1,:),X(2,:),'r--')
hold on
plot(X1(1,:),X1(2,:),':')
%legend('runge_kutta','euler')
grid

vr = v;
wr = w;

% N = 5;
% Q = [1 0 0;
%     0 1 0;
%     0 0 0.1];   %no segundo exemplo foi usado 0.1
% R = 0.1*eye(2);
% vmin = -0.4;
% vmax = 0.4;
% wmin = -0.4;
% wmax = 0.4;
% lb = [];
% ub = [];

% for k = 1:N
%     Qb(3*k-2:3*k,3*k-2:3*k) =Q;
%     Rb(2*k-1:2*k,2*k-1:2*k)=R;
%     lb = [lb;[vmin;wmin]];
%     ub = [ub;[vmax;wmax]]
% end
% 
% % matrizes de ponderação do Nepsac
% Qn = Qb;
% Rn = [R(1,1)*eye(N),zeros(N);zeros(N),R(2,2)*eye(N)];


%disp('Posição inicial')
%disp('[x;y;theta]')
%x0 = input('[x;y;theta] = ')%[1;-1; pi/2];  % x,y,theta
x0 = [0;-1; pi];
X = x0;
v = [];
w = [];


Xm = x0;

%coeficientes do controlador 
ksi = 0.3; %coeficiente de amortecimento - valor dado no artigo
ohmega_n = 1.2; %ohmega_n= sqrt(Ur2(t)^2+g*Ur1(t)^2); 
g = 20;

% disp('Parâmetro do contolador')
% disp('g>0')
% g = input('g = ') %10; %grau de liberdade adicional para o controlador g>0

% GERAR RUÍDO:
%     randn
%      Generate values from a normal distribution with mean 1 and standard
%        deviation 2.
%           r = 1 + 2.*randn(100,1);

% disp('parâmetros do ruído')
% 
% m = input('média =  ') %0; 
% d = input('desvio padrão = ')%.025;
% 
% r = m + d.*randn(1,round(Tsim/h));
% 
% % % Generate values from the uniform distribution on the interval [a, b].
% % %           r = a + (b-a).*rand(100,1);
% % 
% % r = m + (d - m).*rand(1,round(Tsim/h));
% 
% r_max = max(abs(r));
% 
% disp('amplitude máxima do sinal de ruído')
% r_max


for k = 1:round(Tsim/h);
    
    
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

   %K = [-k1 0 0; 0 -sign(v(k))*k2 -k3];
%   v = K*e;
%    v1 = v(1,:);
%    v2 = v(2,:);
   
    v1 = -k1*(e1);
    v2 = - sign(vr(k))*k2*(e2)-k3*e3;
    
%     v(k) = -2.4*(e1);
%     w(k) = - (e2)*k2-2.4*e3;

    v(k) = vr(k)*cos(e3) - v1;
    w(k) = wr(k) -  v2;

    %restrições
   
    if v(k) >=0.4
        v(k)=0.4;
    end
    
    if v(k)< -0.4;
        v(k) = -0.4;
    end
    
    if w(k)>= 0.4;
        w(k) = 0.4;
    end
    
    if w(k)<-0.4;
        w(k)=-0.4;
    end 
   
%     %inserir o ruído 
%     
%     v(k) = r(k)+v(k);
%     w(k) = r(k)+w(k);
    

%         k1 = h*[v(k)*cos(x0(3)); v(k)*sin(x0(3));w(k)];
%      k2 = h*[v(k)*cos(x0(3)+k1(3)/2); v(k)*sin(x0(3)+k1(3)/2);w(k)];
%      k3 = h*[v(k)*cos(x0(3)+k2(3)/2); v(k)*sin(x0(3)+k2(3)/2);w(k)];
%      k4 = h*[v(k)*cos(x0(3)+k3(3)); v(k)*sin(x0(3)+k3(3));w(k)];
%     
%     X = [X, X(:,k) + (1/6)*(k1+2*k2+2*k3+k4)];
    
 %   cálculo da saída do modelo
    
    Xm(1,1) = x0(1) + Ts*v(k)*cos(x0(3));
    Xm(2,1) = x0(2) + Ts*v(k)*sin(x0(3));
    Xm(3,1) = x0(3) + Ts*w(k);
    
    
%     %inserir ruido de medição
%    Xm = Xm + r(k);
%     
    X = [X,Xm];
    x0 =Xm;
    
    
    %x0 = X(:,k+1);
    
    Erro(:,k) = Xr(:,k+1)- X(:,k+1);
    k;
end


plot(X(1,:),X(2,:),'m')
legend('runge_kutta','euler','trajetória do robô')
xlabel('x[m]')
ylabel('y[m]')
% print  -depsc trajetoriaruido
% print  -djpeg90 trajetoriaruido
% print  -dpdf trajetoriaruido



figure
ne = length(Erro);
tempo = 50*(0:ne-1);
subplot(2,1,1)
plot(tempo,v)
legend('v')
hold on
subplot(2,1,2)
plot (tempo,w,'g')
legend('w')
% print -depsc velocidaderuido
% print -djpeg90 velocidaderuido
figure

plot(tempo,Erro(1,:),tempo,Erro(2,:),tempo,Erro(3,:));
legend('x','y','theta')

 E1 = Erro(1,:);
 E2 = Erro(2,:);
 E3 = Erro(3,:);
%ERRO_TOTAL = E1*E1'*Q(1,1)+E2*E2'*Q(2,2)+E3*E3'*Q(3,3);
grid
% print -depsc erroruido
% print -djpeg90 erroruido
 
commandwindow
