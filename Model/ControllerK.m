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
    sizes.NumOutputs     = 2;
    sizes.NumInputs      = 7;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [];
    str = [ ];
    ts = Ts; %tempo de amostragem variável
%elseif flag == 2 %State Update
    % sys =  x + u;
    
elseif flag == 3 %Output
    ErrorFrame = [u(1) u(2) u(3)]'; %real frame
    ur1 = u(4);
    ur2 = u(5);
    
    ksi = u(6);
    g = u(7);
    
    
   ohmega_n = sqrt((ur2^2)+g*(ur1^2));
    k1 = 2*ksi*ohmega_n;
    k2 = g*abs(ur1); 
    k3 = k1;
    
    K = [-k1 0 0;
                 0 -sign(ur1)*k2 -k3];
             
    sys = K*ErrorFrame;
  


    %default

elseif flag == 9
    sys = [ ]; %não faz nada
end
