%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% mobile robot control on a reference path 
%%% Gregor Klancar, 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modificado por Marcus
% 08.04.2016

clear all
close all
clc

format long g     

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Inicialização de Variáveis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.1;          % sampling time
Tsim = (2*pi*1)/Ts;   %90      % tempo de simulação
h = 0.1;           % integrating time
D = 0.4;

xa = [0; 0; 0];  % x,y,theta referenciais para início de trajetória. 
X1 = xa;         %Euler



% The next loop difines the reference
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Geração da tragetória e métodos de seguimento de trajetória.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:round(Tsim/h);  % 1 a tempo de simulação/ tempo de integração   
   
    v(k)=0.2;
    w(k)=0.2;
    if(k>round(Tsim/h)/2)
        w(k)=-0.2;
    end
 


    X1 = [X1, X1(:,k)+ h*([v(k)*cos(xa(3));v(k)*sin(xa(3));w(k)])];  %Euler
    xa = X1(:,k+1);
    
end


Xr = X1;
Ur = [v;w];
%plot(X(1,:),X(2,:),'k--')
hold on
plot(X1(1,:),X1(2,:),':')
%legend('runge_kutta','euler')
% grid

vr = v;
wr = w;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Simulação do robô.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = [0;-0.5;0];  
X = x0;
v = [];
w = [];

Xm = x0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Coeficientes do controlador 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ksi = 0.3;      %ksi = 0.6; %coeficiente de amortecimento - valor dado no artigo  0.6  Ksi
ohmega_n = 1;   %ohmega_n = 2; %ohmega_n= sqrt(Ur2(t)^2+g*Ur1(t)^2); 
g = 20;          %g = 5;

for k = 1:round(Tsim/h);    % 1 a tempo de simulação/ tempo de integração  
    
    a1 = Xr(1,k)-x0(1);
    a2 = Xr(2,k)-x0(2);
    a3 = Xr(3,k)-x0(3);
    
    e1 = cos(x0(3))*a1+sin(x0(3))*a2;
    e2 = -sin(x0(3))*a1+cos(x0(3))*a2;
    e3 = a3;
    
    %controlador K
    
   k1 = 2*ksi*ohmega_n;
   k2 = g*abs(vr(k)); 
   k3 = k1;
%     K = [-k1 0 0; 0 -sign(v(k))*k2 -k3];
%     v = K*e;
%    v1 = v(1,:);
%    v2 = v(2,:);
   
    v1 = -k1*(e1);                     % controlador  
    v2 = - sign(vr(k))*k2*(e2)-k3*e3;  % controlador
    
%     v(k) = -2.4*(e1);
%     w(k) = - (e2)*k2-2.4*e3;

    v(k) = vr(k)*cos(e3) - v1; 
    w(k) = wr(k) -  v2;

    %restrições
    
    if v(k) >=0.4  %0.4
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
    


    vd = (v(k)+w(k)*0.5*D);
    ve = (v(k)-w(k)*0.5*D);

%     if(k>300 && k<320)
%         vd=vd+0.1;
%     end
 
    vt = (ve+vd)/2;
    wt =  (vd-ve)/D;
    
    Xm(1,1) = x0(1) + Ts*vt*cos(x0(3));
    Xm(2,1) = x0(2) + Ts*vt*sin(x0(3));
    Xm(3,1) = x0(3) + Ts*wt;
    
%     %inserir ruido de medição
%    Xm = Xm + r(k);
%     
    X = [X,Xm];
    x0 =Xm;
    
    %x0 = X(:,k+1);
    
    Erro(:,k) = Xr(:,k+1)- X(:,k+1);
%     k
end


plot(X(1,:),X(2,:),'r')
legend('rungeKutta','euler','trajetória do robô')
xlabel('x[m]')
ylabel('y[m]')

% print  -depsc trajetoriaruido
% print  -djpeg90 trajetoriaruido
% print  -dpdf trajetoriaruido

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      Plotagem de v e w 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
ne = length(Erro);
tempo = Ts*(0:ne-1);
subplot(2,1,1)
plot(tempo,v,'b')
legend('v')

hold on
subplot(2,1,2)
plot (tempo,w,'r')
legend('w')

% print -depsc velocidaderuido
% print -djpeg90 velocidaderuido

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%      Plotagem do erro x y e theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
plot(tempo,Erro(1,:),tempo,Erro(2,:),tempo,Erro(3,:));
legend('x','y','theta')

 E1 = Erro(1,:);
 E2 = Erro(2,:);
 E3 = Erro(3,:);
%ERRO_TOTAL = E1*E1'*Q(1,1)+E2*E2'*Q(2,2)+E3*E3'*Q(3,3);
% grid
% print -depsc erroruido
% print -djpeg90 erroruido
 
%commandwindow  % mostra a pagina principal matlab
