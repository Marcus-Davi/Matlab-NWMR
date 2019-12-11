clear;close all;clc
addpath('../Paths')
HIL = 1; %HARD IN LOOP
%% SIMULATION PARAMETERS
Ts = 0.1;
% R = 10;
v = 0.1;
x0 = [0; 0; 0]; %-1.4516

% [Xr,Ur,Tsim] = path_oito(2,v,Ts,x0); 
%  [Xr,Ur,Tsim] = path_reta(1,v,Ts,x0); 
[Xr,Ur,Tsim] = path_S(1.2,v,Ts,x0);


%ir de 0,0 -> (-6.5,-6.5)
iterations = round((Tsim/Ts));

% x0 = [0 ; 0 ; 0];

plot(Xr(1,:),Xr(2,:));
hold on;
% return
%% ROS VARIABLES
%rosinit
if(HIL)
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
slam = rossubscriber('/slam_out_pose');
rate = rosrate(1/Ts);
   map_topic = rossubscriber('map');
end

%% Robot PARAMETERS
vmax = 0.2;vmin = 0;
wmax = 0.5;wmin = -wmax;

%% Sensors

%% LQR
ur1 = v;
ur2 = 0.1;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = zeros(3,2);
SYS = ss(A,B,C,D);
Q = diag([1 100 0]);
R = eye(2);
[K_LQ,S,E] = lqr(SYS,Q,R);

% Klankar

ksi = 0.8;      
ohmega_n = 0.1;   
g  = 40;

%% Simulation Parameters
yk = [0 0 0]';
uk = [0 0]';
YK = zeros(length(yk),iterations);
YK_SLAM = zeros(length(yk),iterations);
UK = zeros(length(uk),iterations);
EK = zeros(length(yk),iterations);


%% Simulation / Experiment (HIL = 1)
zero_angle = [];
for k=1:iterations
                
    if(HIL)
    sens = receive(sensors);
    slam_msg = receive(slam);   
    quat = [slam_msg.Pose.Orientation.W slam_msg.Pose.Orientation.X slam_msg.Pose.Orientation.Y slam_msg.Pose.Orientation.Z];
    eul = quat2eul(quat);
    yk_slam = [slam_msg.Pose.Position.X slam_msg.Pose.Position.Y eul(1)]';
        yk = yk_slam;
    else
    uk0 = uk;
    yk = robot_model(yk,uk0,Ts); % Odometry
    end
 
  

   
    % LQR OVERRIDE ---- INIT
%     
    e_x = Xr(1,k)-yk(1);
    e_y = Xr(2,k)-yk(2);
    e_t = Xr(3,k)-yk(3);
    
    e1 = cos(yk(3))*e_x+sin(yk(3))*e_y;
    e2 = -sin(yk(3))*e_x+cos(yk(3))*e_y;
    e3 = e_t;
    
    e3 = atan2(sin(e3),cos(e3)); %true error


    % KLANKAR
   k1 = 2*ksi*ohmega_n;
   k2 = g*abs(Ur(1,k)); 
   k3 = k1;
   
    v1 = k1*(e1);                     % controlador  
    v2 =  sign(Ur(1,k))*k2*(e2)+k3*e3;  % controlador
    
    uk(1) = Ur(1,k)*cos(e3) + v1;
    uk(2) = Ur(2,k) +  v2; % se liga sinal
    
    % KLANKAR FIM
    
    % LQR
%     V = -K_LQ*[e1 e2 e3]';
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
    YK(:,k) = yk;
    
    UK(:,k) = uk;
    EK(:,k) = ek;
    
    
    
    plot(YK(1,1:k),YK(2,1:k),'red');
    
    
    if(HIL)
    YK_SLAM(:,k) = yk_slam;
    plot(YK_SLAM(1,1:k),YK_SLAM(2,1:k),'blue');
    motorGo(pub,uk(1),uk(2));
    rate.statistics
    waitfor(rate);

    end

end
if(HIL)
motorGo(pub,0,0);
end


%% PLOTS
close all
figure;hold on;
map = receive(map_topic);
map_matlab = readBinaryOccupancyGrid(map);
show(map_matlab)
plot(Xr(1,:),Xr(2,:),'black--');
plot(YK(1,:),YK(2,:),'red');
grid on;

% plot(YK_Noiseless(1,:),YK_Noiseless(2,:))
title('Controle de Robô Ñ-Holonômico em trajetória')
legend('Trajetória Referência','Real Robot')
% plot(time,YK)
time = 1:iterations;
grid on;

figure
plot(time*Ts,UK);
grid on;

figure
plot(time*Ts,EK);
legend('ex','ey','e_\theta')
grid on;

% figure
% plot(Xr(1,:))
% hold on;
% plot(YKALM(1,:))
% plot(YK(1,:))
% legend('Ref','Kalman','Real')

    %% 





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









