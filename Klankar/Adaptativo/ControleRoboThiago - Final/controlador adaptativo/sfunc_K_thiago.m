function [sys, x0, str, ts] = sfunc_K_thiago(t,x,u,flag,ts)
persistent kx_1 ky_1 ktt_1

%inicialização
if flag == 0
    sizes = simsizes;
    sizes.NumContStates  = 0;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 2;
    sizes.NumInputs      = 8;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = [ ];
    str = [ ];
    ts = 0.1; %tempo de amostragem variável
    
%Calcula próximo instante de amostragem
elseif flag == 4
    sys=[];

%Calcula a saída como a diferença entre a entrada actual e a anterior %<---
elseif flag == 3
    ex = u(1);
    ey = u(2);
    ett= u(3);
    vr = u(4);
    wr = u(5);
    kx = u(6);
    ky = u(7);
    ktt= u(8);
                
    Vc = [ vr*cos(ett) + kx*ex;
           wr + ky*vr*ey + ktt*vr*sin(ett)];
       
       % condições de saturação do controlador
       if Vc(1) > 0.8
           Vc(1) = 0.8
       elseif Vc(1) < 0
           Vc(1) = 0;
       end       
           
       if Vc(2) > 0.8
           Vc(2) = 0.8;
       elseif Vc(2) < -0.8
           Vc(2) = -0.8;
       end
               
    
    out = Vc';
    sys = out;
    
    
    kx_1 = kx;
    ky_1 = ky;
    ktt_1 = ktt;

    %default
else
    sys = [ ]; %não faz nada
end
