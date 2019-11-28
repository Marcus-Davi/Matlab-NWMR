clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
% R = 10;
v = 0.1;
x0 = [0; 0; 0]; %-1.4516

% [Xr,Ur,Tsim] = path_oito(2,v,Ts,x0); 
% [Xr,Ur,Tsim] = path_reta(15,v,Ts,x0); 
[Xr,Ur,Tsim] = path_S_(0.8,v,Ts,x0);
iterations = round((Tsim/Ts));
plot(Xr(1,:),Xr(2,:));
hold on;
%% HARD IN LOOP
HIL = 0;
%% LQR
ur1 = 0.1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = zeros(3,2);
SYS = ss(A,B,C,D);
Q = diag([1 100 0]);
R = eye(2);
[K_LQ,S,E] = lqr(SYS,Q,R);


%% EPSAC PARAMETERS

N = 5;
Nu = 5;
n_in = 2;
n_out = 3;

Repsac = 0.001*eye(Nu*n_in); %Control

Qt = 0.001;
Q = diag([1 1 Qt]);
Qcell = repmat({Q},1,N);
Qepsac = blkdiag(Qcell{:}); %Ref
du = 0.000001; %increment

M_inv = eye(Nu*n_in);
n_ones = -1*ones(n_in*(Nu-1),1);
n_diag = diag(n_ones,-n_in);
M_inv = M_inv+n_diag;
M = inv(M_inv);
%% Simulation/Experiment Parameters
YK = [];
UK = [];
EK = [];
uk = Ur(:,1);
YKALM = [];
GPS = [];
YKEST = [];
YKM = [];
% yk = x0;
yk = [0 0 0]';
ykalman = yk;

% SATURATIONS
vmax = 0.4;vmin = 0;
wmax = 0.7;wmin = -wmax;


% NOISE PROFILE
Ts_noise_xy = Ts;
Noise_time = iterations/2;
Decim = Ts_noise_xy/Ts;


Mean = 0; % zero mean
sd_xy = 0; % standard deviation
sd_t = 0.1; % standard deviation
noise_xy = [0;0];
noise_t = 0;



%% KALMAN FILTER PARAMETERS
Q_kal = Ts^2*eye(3); %0.0005 GPS, 0.05 ODO
Q_kal(3,3) = 0.001;
Q_kal = 0.001*Q_kal;
R_kal = 1*[sd_xy^2 0 0;0 sd_xy^2 0;0 0 sd_t^2];
Pk = zeros(3);

%% ROS STUFF
if(HIL)
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
gps = rossubscriber('/gps');
% slam = rossubscriber('/slam_out_pose');
rate = rosrate(1/Ts);
end
%% SENSORS

load('MagCalibration.mat');
load('GyrCalibration.mat');

%% EXECUTION
zero_angle = [];
for k=1:iterations
    
    if(HIL)
    sens = receive(sensors);
%     slam_msg = receive(slam);   
%     quat = [slam_msg.Pose.Orientation.W slam_msg.Pose.Orientation.X slam_msg.Pose.Orientation.Y slam_msg.Pose.Orientation.Z];
%     eul = quat2eul(quat);
%     yk_slam = [slam_msg.Pose.Position.X slam_msg.Pose.Position.Y eul(1)]';
    data = sens.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');
    Mag.x = data(7);
    Mag.y = data(8);
    angle_cal = atan2((Mag.y-MagOff.y),(Mag.x-MagOff.x)); %angulo mag. se liga sinal
    if(isempty(zero_angle))
       zero_angle = angle_cal; 
    end
    vd = data(10);
    ve = data(11);
    [v,w] = rpm2vw(vd,ve);
    uk0 = [v w]';
    else
     uk0 = uk; 
    end
                        
      [ykalman,Pk] = kalman_ext_predict(Ts,@robot_model,Q_kal,ykalman,uk0,Pk);
      
      
      if(HIL)
      ykalman(3) = angle_cal-zero_angle ;
      end
      
%     ne_kalman = (ykm - ykalman);% +  noise(:,k) ;%n(t) = y(t) - x(t)
    
%     nfiltro = repmat(ne_kalman,1,N); %@FILTER OVERRIDE
    
    %preditctions
    ub = [0.3 0]'; 
    Yb = [];

    yb = ykalman;
    for j=1:N %PREDIÇÔES ESPAÇADAS
     yb = robot_model(yb,ub,j*Ts);
     Yb = [Yb; yb];
    end
       
%     else
%    yb = ykest; %ykest
%     for j=1:N
%         yb = robot_model(yb,ub,j*Ts) ;
%         Yb = [Yb; yb];     
%     end
%         Yb = Yb + repmat(ne_raw,N,1);%repmat(ne,N,1);
%     end
    
    %condições inicias
        IC.x0 = ykalman;
        IC.u0 = ub;
        %Toma G do modelo.
        
        
      G = get_G(IC,@robot_model,du,N,Nu,Ts);
%     G = get_G_var(IC,@robot_model,du,N,Nu,Ts);
      
      %Pega referencias futuras
     
      [Wr,Uref] = getRef_var(Xr,Ur,k,N); 
%         [Wr,Uref] = getRef(Xr,Ur,k,N); 
     
     %Calcula o erro da trajetória e base
     
     E = getErr(Wr,Yb,N);
     Ub = repmat(ub,Nu,1);
     
     %Erro de velocidades de referência e base
     EU = (Uref(1:2*Nu)-Ub);
            
     
     % REVISAR AQUI !!
%    K0 = 2*(G'*Qepsac*G+M_inv'*Repsac*M_inv);
%    K1 = 2*(G'*Qepsac*E + Repsac*EU); %incluir velocidade
   
      K0 = 2*(G'*Qepsac*G+Repsac);
      K1 = 2*(G'*Qepsac*E + Repsac*EU); %incluir velocidade
      
     Uo = K0\K1; %Solução Analítica
    
     uk = Ub + Uo;
     
     uk = uk(1:2); %Extrai apenas o atual
     
  
    
    % LQR OVERRIDE ---- INIT
%     
%     e_x = Xr(1,k)-ykalman(1);
%     e_y = Xr(2,k)-ykalman(2);
%     e_t = Xr(3,k)-ykalman(3);
%     
%     e1 = cos(ykalman(3))*e_x+sin(ykalman(3))*e_y;
%     e2 = -sin(ykalman(3))*e_x+cos(ykalman(3))*e_y;
%     e3 = e_t;
% 
%    V = -K_LQ*[e1 e2 e3]';
%     v1 = V(1);
%     v2 = V(2);
%     
%     uk(1) = Ur(1,k)*cos(e3) - v1;
%     uk(2) = Ur(2,k) -  v2;
%     
    % LQR OVERRIDE ---- END
    
    % saturation
    uk(1) = min(uk(1),vmax);
    uk(1) = max(uk(1),vmin);
    uk(2) = min(uk(2),wmax);
    uk(2) = max(uk(2),wmin);
      
    ek = Xr(:,k)-yk; %plotagem
  
    UK = [UK uk];
    EK = [EK ek];
    YKALM = [YKALM ykalman];
    
    if(HIL)
    plot(YKALM(1,1:k),YKALM(2,1:k),'red');
    motorGo(pub,uk(1),uk(2));
    rate.statistics
    waitfor(rate);
    else
        
    end
        
    

    

end
if(HIL)
motorGo(pub,0,0)
else
 plot(YKALM(1,1:k),YKALM(2,1:k),'red');  
 grid on
end
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









