function [sys, x0, str, ts] = sfunc_rls(t,x,u,flag,ts)
persistent kx_1 ky_1 ktt_1

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 6;
    sizes.NumInputs      = 7;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [ ];
    str = [ ];
    ts = 0.1; %tempo de amostragem variável
    
    kx_1=0;
    ky_1=0;
    ktt_1=0;

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
    T = u(6);
    tt = u(7);
                  
%     nx = 0.2;%0.9;
%     ny = 0.5;%0.9;
%     ntt = 1.7;%0.9;    
%     
%     F = 0.5*(ex^2+ey^2+ett^2);
%     %dF = jacobian(F,[kx_1,ky_1,ktt_1]);
%     %dF = jacobian(F,[x,y,tt]);
%     
%     m = 100;
%     a = 0;
%     %L = 0.05;
%     Ic = 1; %m*L^2;
%     dqv = [sin(tt)*t 0; -cos(tt)*t 0; 0 1];
%     dvt = [(1/m)*t 0; 0 (1/(Ic+m*a^2))*t];
%     dtec = [5 0; 0 5]; % considerando Kp=5;
%     decvc = 1;
%     J = dqv*dvt*dtec*decvc;
%     
%     ep = [ex; ey; ett];
%     Te = [cos(tt) sin(tt) 0; -sin(tt) cos(tt) 0; 0 0 1];
%     dF = -ep'*Te*J*[ex 0 0; 0 vr*ey vr*sin(ett)];
%     
%     kx = kx_1 - nx*dF(1);
%     ky = ky_1 - ny*dF(2);
%     ktt = ktt_1 - ntt*dF(3);
    kx = 10;%1;
    ky = 10;%120;
    ktt = 50;%15;
    
errot = t-T;
    Vc = [ vr*cos(ett) + kx*ex;
        wr + ky*vr*ey + ktt*vr*sin(ett)];
    out = [Vc' kx ky ktt errot];

    sys = out;
    
    
    kx_1 = kx;
    ky_1 = ky;
    ktt_1 = ktt;

    %default
else
    sys = [ ]; %não faz nada
end
