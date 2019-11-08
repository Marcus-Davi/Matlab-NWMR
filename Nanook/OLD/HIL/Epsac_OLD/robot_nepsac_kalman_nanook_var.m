%% Bluetooth Interface
close all;clc
if exist('b')
    fclose(b);
end
clear
b = blueNanook;
load('CalibrationAccGyr')
load('CalibrationMag')
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.1;
% [Xr,Ur,Tsim] = path_oito(R,v,Ts);
x0 = [0; 0; 0];
[Xr,Ur,Tsim] = path_S(1.2,v,Ts,x0);
iterations = round((Tsim/Ts));

plot(Xr(1,:),Xr(2,:));
hold on;
% return %testing

%0 -> PARAELO | 1-> SERIE_PARALELO , 0 -> PARALELO  <----- ESCOLHA AQUI ###
SERIE_PARALELO = 1; 

%% ROBOT PARAMETERS
vmax = 0.4;vmin = 0;
wmax = 0.6;wmin = -wmax;

%% EPSAC PARAMETERS
N = 5;
Nu = 1;
n_in = 2;
n_out = 3;

Repsac = 0.00001*eye(Nu*n_in); %Control %0.01

Qt = 0.01;
Q = diag([1 1 Qt]);
Qcell = repmat({Q},1,N);
Qepsac = blkdiag(Qcell{:}); %Ref
du = 0.00001; %increment

M_inv = eye(Nu*n_in);
n_ones = -1*ones(n_in*(Nu-1),1);
n_diag = diag(n_ones,-n_in);
M_inv = M_inv+n_diag;
M = inv(M_inv);

%% Simula
YK = [];
UK = [];
EK = [];
uk = Ur(:,1);
YKALM = [];
YKEST = [];
YKNOISE = [];
Sensors = [];
yk = [0 0 0]';
ykest = yk;
ykalman = yk;

% uk = [0 0]';
pert = [0 0];
% Creates noise profile
Mean = 0; % zero mean
sd_xy = 0; % standard deviation
sd_t = 0.01; % standard deviation
noise_xy = Mean + sd_xy.*randn(2,iterations)/2;
noise_t = Mean + sd_t.*randn(1,iterations)/2;
noise = [noise_xy;noise_t];

% for i=1:iterations/2
%    noise(:,i) = [0 0 0]'; 
% end

%% KALMAN FILTER PARAMETERS
Q_kal = 0.1*eye(3); %0.0002
Q_kal(3,3)=0.00001; %compass
R_kal = [sd_xy^2 0 0;0 sd_xy^2 0;0 0 sd_t^2];
Pk = zeros(3);

%% Simulation

for k=1:iterations
     
    tic %sync
       
    if(SERIE_PARALELO)
    ykest = robot_model(yk,uk,Ts); %MODELO
    else
    ykest = robot_model(ykest,uk,Ts); %MODELO    
    end
    
    
        % MEASUREMENT
    s = sensorGet(b); %ajeitar transposto
    if(~isempty(s))
    S = sensorCalibrate(s,CalibrationAccGyr,CalibrationMag);
    vd = S(10);
    ve = S(11);        
    end
    
    [v_measure,w_measure] = rpm2vw(vd,ve);
    
    %MODEL     
    yk = robot_model(yk,[v_measure w_measure],Ts);
           
    ykm = yk + noise(:,k); %yk measured
    

    
    
    [ykalman,~,Pk,err] = kalman_ext(Ts,@robot_model,@measurement_model,Q_kal,R_kal,ykalman,uk,Pk,ykm);
    
    ne_kalman = (ykm - ykalman);% +  noise(:,k) ;%n(t) = y(t) - x(t)
    ne_raw = ykm-ykest;   

    nfiltro = repmat(ne_kalman,1,N); %@FILTER OVERRIDE
    

    %preditctions
    ub = [0.3 0]'; 
%     ub = Ur(:,k);
    Yb = [];
    
    if(SERIE_PARALELO)
    yb = ykalman;
    for j=1:N
     yb = robot_model(yb,ub,j*Ts); %+ nfiltro(:,j);
     Yb = [Yb; yb];
    end
       
    else %PARALELO
   yb = ykest; %ykest
    for j=1:N
        yb = robot_model(yb,ub,j*Ts) ;
        Yb = [Yb; yb];     
    end
        Yb = Yb + repmat(ne_raw,N,1);%repmat(ne,N,1);
    end
    
    %condições inicias
        IC.x0 = ykalman;
        IC.u0 = ub;
        %Toma G do modelo.
      G = get_G_var(IC,@robot_model,du,N,Nu,Ts);
%       G = get_G(IC,@robot_model,du,N,Nu,Ts);
      
      %Pega referencias futuras
      
%      [Wr,Uref] = getRef(Xr,Ur,k,N); 
      [Wr,Uref] = getRef_var(Xr,Ur,k,N); 
     
     %Calcula o erro da trajetória e base
     E = getErr(Wr,Yb,N);
     Ub = repmat(ub,Nu,1);
     %Erro de velocidades de referência e base
     EU = (Uref(1:2*Nu)-Ub);
            
%    K0 = 2*(G'*Qepsac*G+M_inv'*Repsac*M_inv);
%    K1 = 2*(G'*Qepsac*E + Repsac*EU); %incluir velocidade
       
      K0 = 2*(G'*Qepsac*G+Repsac);
      K1 = 2*(G'*Qepsac*E + Repsac*EU); %incluir velocidade
      
     Uo = K0\K1; %Solução Analítica
    
     uk = Ub + Uo;
     
     uk = uk(1:2); %Extrai apenas o atual
     
    
    % saturation
    uk(1) = min(uk(1),vmax);
    uk(1) = max(uk(1),vmin);
    uk(2) = min(uk(2),wmax);
    uk(2) = max(uk(2),wmin);
    
    [vd_rpm,ve_rpm] = vw2rpm(uk(1),uk(2));
    motorGo(b,vd_rpm,ve_rpm)
    
    
    ek = Xr(:,k)-yk; %plotagem
    YK = [YK yk];
    UK = [UK uk];
    EK = [EK ek];
    YKALM = [YKALM ykalman];
    YKEST= [YKEST ykest];
    YKNOISE = [YKNOISE ykm];
    Sensors = [Sensors; S'];
    
    plot(yk(1),yk(2),'b*');
    drawnow;
    
    
        toc;
    delta = abs(Ts-toc);
    if delta < Ts
    pause(delta);
    end
end

motorGo(b,0,0)
fclose(b)


%% PLOTS
close all
plot(YKNOISE(1,:),YKNOISE(2,:),'green*');hold on
plot(Xr(1,:),Xr(2,:),'black--'); hold on
grid on;
plot(YKALM(1,:),YKALM(2,:),'blue');
plot(YK(1,:),YK(2,:),'red');

% plot(YK_Noiseless(1,:),YK_Noiseless(2,:))
title('Controle de Robô Ñ-Holonômico em trajetória')
legend('Noise','Trajetória Referência','Kalman','Real Robot')
% plot(time,YK)
time = 1:iterations;
grid on;

figure
plot(time*Ts,UK);
grid on;

figure
plot(time*Ts,EK);
legend('ex','ey','ez')
grid on;

% figure
% plot(Xr(1,:))
% hold on;
% plot(YKALM(1,:))
% plot(YK(1,:))
% legend('Ref','Kalman','Real')






%% Funções Auxiliares

function e = getErr(W,Y,nu)
e = W-Y;
%suavização do erro em theta
for i=1:nu
   e(i*3) = atan2(sin(e(i*3)),cos(e(i*3)));  %3 pq são 3 saídas!     
end
end

function [wr,ur] = getRef(Xr,Ur,k,nu)
wr = [];
ur = [];
[~,kend] = size(Xr);
for i=1:nu
    
     %testa final da traj. se sim, repete
    if(k+i < kend)
    wr = [wr;Xr(:,k+i)];    
    ur = [ur;Ur(:,k+i)];    
    else
    wr = [wr;Xr(:,end)];
    ur = [ur;Ur(:,end)];
    end

end

end


function [wr,ur] = getRef_var(Xr,Ur,k,nu)
wr = [];
ur = [];
[~,kend] = size(Xr);
Seq = getSequence(nu);
for i=1:nu    
     %testa final da traj. se sim, repete
    if(k+Seq(i) < kend)
    wr = [wr;Xr(:,k+Seq(i))];    
    ur = [ur;Ur(:,k+Seq(i))];    
    else
    wr = [wr;Xr(:,end)];
    ur = [ur;Ur(:,end)];
    end
end

end

function seq = getSequence(n)
seq = zeros(n,1);
for i=1:n
   seq(i) = i*(i+1)/2;
end

end










