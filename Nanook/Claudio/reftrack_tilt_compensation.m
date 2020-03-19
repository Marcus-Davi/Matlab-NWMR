clear;close all;clc
addpath('../Paths')
HIL = 0; %HARD IN LOOP
%% SIMULATION PARAMETERS
Ts = 0.1;
% R = 10;
v = [0.2]';
x0 = [0; 0; 0]; %-1.4516

% [Xr,Ur,Tsim] = path_oito(2,v,Ts,x0); 
 [Xr,Ur,Tsim] = path_reta(6.5,v,Ts,x0); 
% [Xr,Ur,Tsim] = path_square(2.5,v,Ts,x0);


%ir de 0,0 -> (-6.5,-6.5)
iterations = round((Tsim/Ts));

% x0 = [0 ; 0 ; 0];

plot(Xr(1,:),Xr(2,:));
hold on;
figure
plot(Ur(1,:));
% return
%% ROS VARIABLES
%rosinit
if(HIL)
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
%slam = rossubscriber('/slam_out_pose');
rate = rosrate(1/Ts);
%map_topic = rossubscriber('map');

% Sensors
load('GyrCalibration.mat')
load('MagCalibration.mat')
end



%% Robot PARAMETERS
vmax = 0.35;vmin = 0;
wmax = 0.6;wmin = -wmax;
%% LQR
% ur1 = (v(1) + v(2))/2;
% ur2 = 0.;
% A = [0 ur2 0;-ur2 0 ur1;0 0 0];
% B = [1 0;0 0;0 1];
% C = eye(3);
% D = zeros(3,2);
% SYS = ss(A,B,C,D);
% Q = diag([1 10 0]);
% R = eye(2);
% [K_LQ,S,E] = lqr(SYS,Q,R);

%% Klankar

ksi = 0.8;      
ohmega_n = 0.1;   
g  = 40;

%% LPV
load('../../LPV/Controlador_LPV.mat')
K_lpv = Controlador_LPV;
%% Simulation Parameters
yk = [-0.25 0 0]';
uk = [0 0]';
vk = [0 0]';
YK = zeros(length(yk),iterations);
UK = zeros(length(uk),iterations);
VK = zeros(length(vk),iterations);
EK = zeros(length(yk),iterations);
VR_COMPENSATED = zeros(1,iterations);
PITCH = zeros(1,iterations);

zero_angle = 0;
pitch_filtered = 0;
vr_compensated = 0.2;
% return

%% Simulation / Experiment (HIL = 1)
for k=1:iterations
                
    if(HIL)
    sens = receive(sensors);
    sens_data = sens.Data;
    data = sscanf(sens_data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');   
    ax = data(1);ay=data(2);az=data(3);
    gx = data(4);gy=data(5);gz=data(6);
    mx = data(7);my=data(8);mz=data(9);
    vd = data(10);
    ve = data(11);
    [v,w] = rpm2vw(vd,ve);
    
    
    angle_cal = atan2(-(my-MagOff.y),(mx-MagOff.x));
    
    if(zero_angle == 0)
       zero_angle = angle_cal; 
    end
    
    pitch = atan2(-ax,sqrt(ay*ay+az*az));
    alfa = 0.5;
    pitch_filtered = alfa*pitch + (1-alfa)*pitch_filtered
    vr_compensated = Ur(1,k)/cos(pitch_filtered)
    
    % Calcular Pitch
    % Calcular Yaw
    % Calcular tilt compensation
    
    %Odometria
    vk = [v w];
    yk = robot_model(yk,vk,Ts); % Odometry
    
    yk(3) = wrapTo2Pi(zero_angle - angle_cal);
   
    else
    uk0 = uk;
    yk = robot_model(yk,uk0,Ts); % Odometry
    end
 
     
    e_x = Xr(1,k)-yk(1);
    e_y = Xr(2,k)-yk(2);
    e_t = Xr(3,k)-yk(3);
    
    e1 = cos(yk(3))*e_x+sin(yk(3))*e_y;
    e2 = -sin(yk(3))*e_x+cos(yk(3))*e_y;
    e3 = e_t;
    
    e3 = atan2(sin(e3),cos(e3)); %true error


    % KLANKAR ------------------------------------
%    k1 = 2*ksi*ohmega_n;
%    k2 = g*abs(Ur(1,k)); 
%    k3 = k1;
%    
%     v1 = k1*(e1);                     % controlador  
%     v2 =  sign(Ur(1,k))*k2*(e2)+k3*e3;  % controlador
%     
%     uk(1) = Ur(1,k)*cos(e3) + v1;
%     uk(2) = Ur(2,k) +  v2; % se liga sinal
% %     
    % KLANKAR END ------------------------------
    
    % LQR ---------------------------------------
%     V = -K_LQ*[e1 e2 e3]';
%     v1 = V(1);
%     v2 = V(2);
%     
%     uk(1) = Ur(1,k)*cos(e3) - v1;
%     uk(2) = Ur(2,k) -  v2;
    
    % LQR END ----------------------------------
        
    % LPV --------------------------------------
    K_LPV = K_lpv.K0 + vr_compensated*K_lpv.K1;
    V = -K_LPV*[e1 e2 e3]';
    v1 = V(1);
    v2 = V(2);
    
    uk(1) = 0.*cos(e3) - v1
    uk(2) = Ur(2,k) -  v2
    % LPV END ----------------------------------
    
    
    
    % saturation
    uk(1) = min(uk(1),vmax);
    uk(1) = max(uk(1),vmin);
    uk(2) = min(uk(2),wmax);
    uk(2) = max(uk(2),wmin);
      

    YK(:,k) = yk;
    VK(:,k) = vk;
    UK(:,k) = uk;
    EK(:,k) = [e_x e_y e3]';
    VR_COMPENSATED(k) = vr_compensated;
    PITCH(k) = pitch_filtered;
    
    
    
    if(HIL)
    YK_ODOM(:,k) = yk;
%     plot(YK_SLAM(1,1:k),YK_SLAM(2,1:k),'blue');
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
if(HIL)
% map = receive(map_topic);
% map_matlab = readBinaryOccupancyGrid(map);
% show(map_matlab)
end
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
% hold on
% plot(time*Ts,VK);
grid on;

figure
plot(PITCH)
hold on
plot(VR_COMPENSATED)
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









