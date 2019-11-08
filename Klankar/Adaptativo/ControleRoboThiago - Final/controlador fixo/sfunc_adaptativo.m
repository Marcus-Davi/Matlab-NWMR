function [sys, x0, str, ts] = sfunc_adaptativo(t,x,u,flag,ts)
persistent kx_1 ky_1 ktt_1

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 3;%3
    sizes.NumInputs      = 12;%7
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [ ];
    str = [ ];
    ts = 0.1; %tempo de amostragem variável
    
    % condições iniciais dos ganhos
    kx_1  = 50.01;
    ky_1  = 50.02;
    ktt_1 = 50.03;

%Calcula próximo instante de amostragem
elseif flag == 4
    sys=[];

%Calcula a saída como a diferença entre a entrada actual e a anterior %<---
elseif flag == 3
    ex=u(1);
    ey=u(2);
    ett=u(3);
    vr = u(4);
    wr = u(5);
    J1 = u(6);
    J2 = u(7);
    J3 = u(8);
    J4 = u(9);
    J5 = u(10);
    J6 = u(11);
    tt = u(12);
    
    
    J = [J1 J2
         J3 J4
         J5 J6];        
    
    ep = [ex; ey; ett];
    
    Te = [cos(tt) sin(tt) 0;
         -sin(tt) cos(tt) 0;
          0       0       1];
    g = [1 0.1 1];
    G = diag(g);
    
    dF = -ep'*G*Te*J*[ex 0     0
                      0  vr*ey vr*sin(ett)];
    
%     nx = 0*10;%10%0.9;
%     ny = 0*50;%50%0.9;
%     ntt = 0*5.7;%0.9;
    nx  = 0.0;%10%0.9;
    ny  = 0.0;%50%0.9; % NEGATIVO!? PQ?????
    ntt = 0.0;%0.9;

    kx = kx_1 - nx*dF(1);
    ky = ky_1 - ny*dF(2);
    ktt = ktt_1 - ntt*dF(3);
%     kx = 10;%1;
%     ky = 10;%120;
%     ktt = 50;%15;
    
    kx_1 = kx;
    ky_1 = ky;
    ktt_1 = ktt;    
 
    out = [kx ky ktt];
    sys = out;
    
    %default
else
    sys = [ ]; %não faz nada
end
