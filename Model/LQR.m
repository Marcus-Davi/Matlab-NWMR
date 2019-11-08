%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% mobile robot control on a reference path
%%% Gregor Klancar,
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Modificado por Marcus
% 28.12.2017
%Assumir o sistema com referencias constantes de velocidade !!!
clear all;close all;clc
ur1 = 0.1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = zeros(3,2);
SYS = ss(A,B,C,D);
Q = diag([1 50 0]);
R = eye(2);
[K,S,E] = lqr(SYS,Q,R);

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%            Inicializa��o de Vari�veis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 0.1;          % sampling time
Tsim = 63;   %63 para oito. 70 pra "S"
h = 0.1;           % integrating time
D = 0.4;            %distancia entre rodas

limit = 0.4;

xa = [0; 0; 0];  % x,y,theta referenciais para in�cio de trajet�ria.
X1 = xa;         %Euler

xb = [0;0;0.0];
X2 = xb;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Gera��o da traget�ria e m�todos de seguimento de trajet�ria.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lado = 1.5; %lado do quadrado
for k = 1:round(Tsim/h);  % 1 a tempo de simula��o/ tempo de integra��o
    
    %     if (k<=lado/(Ts*h));   %419
    %         xb(3)=0;
    %         xa(3)=0;
    %         v(k) = 0.10; %= 0.3;
    %         w(k) = 0;  % varia��o da velocidade angular positiva
    %     elseif (k>=lado/(Ts*h))&&(k<=2*lado/(Ts*h));
    %         xb(3) = pi/2;
    %         xa(3)=pi/2;
    %         v(k) = 0.10; %= 0.3;
    %         w(k) = 0.0;  % varia��o da velocidade angular positiva
    %     elseif (k>=2*lado/(Ts*h))&&(k<=3*lado/(Ts*h));
    %         xb(3) = pi;
    %         xa(3)=pi;
    %         %  x0(3)= pi;
    %         v(k) = 0.10; %= 0.3;
    %         w(k) = 0.0;  % varia��o da velocidade angular positiva
    %     elseif (k>=3*lado/(Ts*h))&&(k<=4*lado/(Ts*h));
    %         xb(3)= 3*pi/2;
    %         xa(3)=3*pi/2;
    %         v(k) = 0.10; %= 0.3;
    %         w(k) = 0.00; % varia��o da velocidade angular negativa
    %     else
    %         v(k)=0;
    %         w(k)=0;
    %     end
    
    %ovoide
    %              v(k) = 0.10; %= 0.3;
    %            if (k<=150);   %419
    %             w(k) = 0.025;  % varia��o da velocidade angular positiva
    %            elseif (k>=150)&&(k<=270);
    %             w(k) = 0.30;
    %            elseif (k>=270)&&(k<=360);
    %             w(k) = -0.14;
    %            else %if (k>=70)&&(k<=90);
    %             w(k) = 0.287; % varia��o da velocidade angular negativa
    %            end
    
    %Oito
%             if(k<round(Tsim/h/2)) 
%             w(k)=0.2;
%             v(k)=0.1;
%             else
%             w(k)=-0.2;
%             v(k)=0.1;
%             end
    
    
    %    "Z"
        w(k)=0;
        v(k)=0.1;
        if(k<Tsim/h/3)
            xb(3)=0;
        elseif(k<Tsim/h/2 && k>=Tsim/h/3)
            xb(3)=pi/2;
        else
            xb(3)=0;
        end
    
    
    
    X1 = [X1, X1(:,k)+ h*([v(k)*cos(xa(3));v(k)*sin(xa(3));w(k)])];  %Euler
    xa = X1(:,k+1);
    X2 = [X2, X2(:,k)+ h*([v(k)*cos(xb(3));v(k)*sin(xb(3));0])];  %Marcus
    % xb = X2(:,k+1);
    X2(3,k)=xb(3);
    
end


Xr = X2;
Ur = [v;w];
hold on
plot(Xr(1,:),Xr(2,:),'-')
legend('Referencia')
grid

vr = v;
wr = w;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                   Simula��o do rob�.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x0 = [0;-0.5;0];
X = x0;
v = [];
w = [];

Xm = x0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Coeficientes do controlador
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ksi = 0.3;
ohmega_n = 1;
g =40;




for k = 1:round(Tsim/h);    % 1 a tempo de simula��o/ tempo de integra��o
    
    e_x = Xr(1,k)-x0(1);
    e_y = Xr(2,k)-x0(2);
    e_t = Xr(3,k)-x0(3);
    
    e1 = cos(x0(3))*e_x+sin(x0(3))*e_y;
    e2 = -sin(x0(3))*e_x+cos(x0(3))*e_y;
    e3 = e_t;
    
    
    
%        k1 = 2*ksi*ohmega_n;
%        k2 = g*abs(vr(k));
%        k3 = k1;
%     
%        v1 = -k1*(e1);                    % controlador
%        v2 = -sign(vr(k))*k2*(e2)-k3*e3;  % controlador
    
    V = -K*[e1 e2 e3]';
    v1 = V(1);
    v2 = V(2);
    
    v(k) = vr(k)*cos(e3) - v1;
    w(k) = wr(k) -  v2;
    
    %restri��es
    
    if v(k) >=limit  %0.4
        v(k)=limit;
    end
    
    if v(k)< 0;
        v(k) = 0;
    end
    
    if w(k)>= limit;
        w(k) = limit;
    end
    
    if w(k)<-limit;
        w(k)=-limit;
    end
    
    
    
    vd = (v(k)+w(k)*0.5*D);
    ve = (v(k)-w(k)*0.5*D);
    
    %RUIDO
    if(k>500 && k<520)
        vd=0;
    end
    
    vt = (ve+vd)/2;
    wt =  (vd-ve)/D;
    
    Xm(1,1) = x0(1) + Ts*vt*cos(x0(3));
    Xm(2,1) = x0(2) + Ts*vt*sin(x0(3));
    Xm(3,1) = x0(3) + Ts*wt;
    
    
    X = [X,Xm];
    x0 =Xm;
    
    
    Erro(:,k) = Xr(:,k+1)- X(:,k+1);
    
end


plot(X(1,:),X(2,:),'r')
legend('Refer�ncia','trajet�ria do rob�')
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
