function [sys, x0, str, ts] = sfunc_rls(t,x,u,flag,ts)

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2;
    sizes.NumInputs      = 7;
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
    e1=u(1);
    e2=u(2);
    e3=u(3);
    vr = u(4);
    wr = u(5);
    ksi = u(6);
    g = u(7);
    
    e = [e1; e2; e3];

   % ksi = 0.3;       
   % ohmega_n = 1;   % %ohmega_n= sqrt(Ur2(t)^2+g*Ur1(t)^2); 
   % g = 20;    
    ohmega_n = sqrt((wr^2)+g*(vr^2));
    k1 = 2*ksi*ohmega_n;
    k2 = g*abs(vr); 
    k3 = k1;

    M = [-k1    0   0; 
      0 -sign(vr)*k2 -k3 ];

    v = M*e;

    sys = v;


    %default
else
    sys = [ ]; %não faz nada
end
