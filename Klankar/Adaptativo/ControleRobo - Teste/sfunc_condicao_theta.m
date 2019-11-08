function [sys, x0, str, ts] = sfunc_rls(t,x,u,flag,ts)

%inicializa��o
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
    ts = [0.01]; %tempo de amostragem vari�vel

%Calcula pr�ximo instante de amostragem
elseif flag == 4
    sys=[];

%Calcula a sa�da como a diferen�a entre a entrada actual e a anterior %<---
elseif flag == 3
    
    theta=u(1);
        
    if theta > 2*pi
         theta = theta - 2*pi;
%           theta = 0;
    elseif theta < -2*pi
         theta = theta + 2*pi;        
%           theta = 0;        
    end  
    
    sys = theta;


    %default
else
    sys = [ ]; %n�o faz nada
end
