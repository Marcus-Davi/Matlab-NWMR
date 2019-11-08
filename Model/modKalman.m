function [sys, x0, str,ts] = sfunc(t,x,u,flag,Ts,A,C,Rw,Rv,disc,zvu)
persistent P;
persistent K;
persistent buffer;
persistent k;
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
    sizes.NumDiscStates  = size(A,1);
    sizes.NumOutputs     = size(A,1);
    sizes.NumInputs      = size(C,1);
    sizes.DirFeedthrough = 0;
    sizes.NumSampleTimes = 1;

    sys = simsizes(sizes);   
    
    x0 = zeros(sizes.NumDiscStates,1);
    str = [ ];
    ts = Ts; %tempo de amostragem vari�vel
    
    P = zeros(size(A));
    buffer = zeros(zvu,sizes.NumInputs);
    k = 1;
    
  
elseif flag == 2 %State Update
   y = u; 
   if(y < disc && y > -disc)
       y = 0;
   end
   
  
   buffer(k,:) = y;
   k=k+1;
   if(k>zvu)
       k = 1;
   end
   
   soma = sum(buffer);
   if soma==0
       x(2) = 0; %velocity
   end
   
    xhat = A*x;   %1
    P = A*P*A' + Rw; %2 
    
    K = (P*C')/(C*P*C' + Rv); %3
    sys = xhat+K*(y-C*xhat); %4
    P = (eye(size(K,1)) - K*C)*P; %5

   

elseif flag == 3 %Output

    sys = x;
    
    

elseif flag == 9
    sys = [ ]; %n�o faz nada
end
