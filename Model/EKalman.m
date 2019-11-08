function [sys, x0, str,ts] = sfunc(t,x,u,flag,Ts,Qn,Rn)
persistent Pk;
persistent Kk;
global alfa;
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
    sizes.NumDiscStates  = 3;%x,y,V,a
    sizes.NumOutputs     = 3;
    sizes.NumInputs      = 3; %a,v,theta
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = zeros(sizes.NumDiscStates,1);
    str = [ ];
    ts = Ts; %tempo de amostragem vari�vel
    
    Pk = zeros(size(3));
    
    
  
elseif flag == 2 %State Update
      

    ax = u(1);
    yin = u(2); 
    theta = u(3);  
    
    Jf = [1 0 cos(theta)*Ts;
          0 1 sin(theta)*Ts;
          0 0 1];
          
    Jh = [0 0 1];
    
    
    %PREDICT
    xhat = x + [Ts*cos(theta)*x(3) + (Ts^2/2)*cos(theta)*ax;
                Ts*sin(theta)*x(3) + (Ts^2/2)*sin(theta)*ax;
                Ts*ax];
    Pk = Jf*Pk*Jf'+Qn; %P_k = Jf(xhar_k-1,u_k)P_k-1J'(xhat_k-1,u_k) + Q_k
    %CORRECT
    Kk = Pk*Jh'/(Jh*Pk*Jh'+Rn);
    sys = xhat + Kk*(yin-Jh*xhat);
    Pk = (eye(3) - Kk*Jh)*Pk;
    
%     Linear
%     xhat = A*x + B*uin;   %1
%     P = A*P*A' + Rw; %2 
%     
%     K = (P*C')/(C*P*C' + Rv); %3
%     sys = xhat+K*(yin-C*xhat); %4
%     P = (eye(size(K,1)) - K*C)*P; %5
   

elseif flag == 3 %Output

    sys = x;
    
    

elseif flag == 9
    sys = [ ]; %n�o faz nada
end
