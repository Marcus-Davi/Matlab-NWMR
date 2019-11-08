function [sys, x0, str,ts] = sfunc(t,x,u,flag,Ts)
%t = tempo
%x = estados
%u = entrada
%flag = indicador de task

%sys = saida
%x0 = condicao inciial
%str = vazio
%ts = tempo de amostragem

if flag == 0 %Inicializacao
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 4;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [];
    str = [ ];
    ts = Ts; %tempo de amostragem variável
%elseif flag == 2 %State Update
    % sys =  x + u;
    
elseif flag == 3 %Output
    RealFrame = [u(1) u(2) u(3)]'; %real frame
    theta = u(4); %rotation angle
    
    Transform = [cos(theta) sin(theta) 0;
                -sin(theta) cos(theta) 0;
                  0 0 1];
    sys = Transform*RealFrame;
    %sys = [Xr Yr Thr]'


    %default

elseif flag == 9
    sys = [ ]; %não faz nada
end
