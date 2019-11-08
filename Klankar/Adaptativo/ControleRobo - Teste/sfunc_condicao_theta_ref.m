function [sys, x0, str, ts] = sfunc_rls(t,x,u,flag,ts)

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 1;
    sizes.NumInputs      = 1;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [ ];
    str = [ ];
    ts = [0.1]; %tempo de amostragem variável

%Calcula próximo instante de amostragem
elseif flag == 4
    sys=[];

%Calcula a saída como a diferença entre a entrada actual e a anterior %<---
elseif flag == 3
    
    theta=u(1);
        
    if theta > 2*pi
        theta = theta - 2*pi;        
    elseif theta < -2*pi
        theta = theta + 2*pi;    
    end  
    
    sys = theta;


    %default
else
    sys = [ ]; %não faz nada
end
