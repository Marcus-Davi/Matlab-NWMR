function [sys, x0, str, ts] = sfunc_adaptativo(t,x,u,flag,ts)
persistent kx_1 ky_1 ktt_1

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 6;%3
    sizes.NumInputs      = 11;%7
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
    
    Kp = u(1);
    S11 = u(2);
    S12 = u(3);
    S21 = u(4);
    S22 = u(5);
    S31 = u(6);
    S32 = u(7);    
    V11 = u(8);
    V12 = u(9);
    V21 = u(10);
    V22 = u(11); 
    
    S_ = [S11 S12; S21 S22; S31 S32];        
    V_ = [V11 V12; V21 V22];
    
    dqv = S_;    
    dvt = V_;        
    dtec = [Kp 0; 0 Kp];    
    decvc = 1;
    
    J = dqv*dvt*dtec*decvc;
    
    J1 = J(1,1);
    J2 = J(1,2);
    J3 = J(2,1);
    J4 = J(2,2);
    J5 = J(3,1);
    J6 = J(3,2);
    
    out = [J1 J2 J3 J4 J5 J6];
    sys = out;
    
    %default
else
    sys = [ ]; %não faz nada
end
