function [sys, x0, str, ts] = sfunc_rls(t,x,u,flag,ts)

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 4;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [ ];
    str = [ ];
    ts = [0.01]; %tempo de amostragem variável

%Calcula próximo instante de amostragem
elseif flag == 4
    sys=[];

%Calcula a saída como a diferença entre a entrada actual e a anterior %<---
elseif flag == 3
    
    ex=u(1);
    ey=u(2);
    etheta=u(3);
    theta = u(4);
    
    M = [cos(theta) sin(theta) 0 ; 
        -sin(theta) cos(theta) 0;
        0   0   1];
    e = M*[ex; ey; etheta];
    
    sys = e;


    %default
else
    sys = [ ]; %não faz nada
end
