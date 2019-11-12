clear;close all;clc
addpath('Paths')
addpath('Ros')
addpath('Functions')

HIL = 1; %HARD IN LOOP
%% SIMULATION PARAMETERS
Ts = 0.2;
% R = 10;
v = 0.1;
x0 = [0; 0; 0]; %-1.4516

% [Xr,Ur,Tsim] = path_oito(2,v,Ts,x0); 
%  [Xr,Ur,Tsim] = path_reta(1,v,Ts,x0); 
[Xr,Ur,Tsim] = path_S(1,v,Ts,x0);


%ir de 0,0 -> (-6.5,-6.5)
iterations = round((Tsim/Ts));

% x0 = [0 ; 0 ; 0];

plot(Xr(1,:),Xr(2,:));
hold on;

%% ROS VARIABLES
%rosinit
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
tftree = rostf;
laser = rossubscriber('/scan');
rate = rosrate(1/Ts);
odom = getTransform(tftree, 'map', 'odom');

%% Robot PARAMETERS
vmax = 0.4;vmin = 0;
wmax = 0.7;wmin = -wmax;

%% Sensors

load('MagCalibration.mat');
load('GyrCalibration.mat');

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

ksi = 0.3;      
ohmega_n = 1;   
g =40;  


%% Simulation Parameters
yk = [0 0 -pi/2]';
uk = [0 0]';
YK = zeros(length(yk),iterations);
YK_SLAM = zeros(length(yk),iterations);
UK = zeros(length(uk),iterations);
EK = zeros(length(yk),iterations);


%% Simulation / Experiment (HIL = 1)
zero_angle = [];
for k=1:iterations
                
    if(HIL)
        tic
        %Laser Scan
        scan = receive(laser);
        ranges = scan.Ranges;
        angleI = scan.AngleIncrement;
        angleMin = scan.AngleMin;
        F = ranges2force(ranges,angleI,angleMin);
        [v_apf,w_apf] = force2vw(F);
         %
        
        
    sens = receive(sensors);
    
     odom = getTransform(tftree, 'map', 'odom');
    quat = [odom.Transform.Rotation.W odom.Transform.Rotation.X odom.Transform.Rotation.Y odom.Transform.Rotation.Z];
    eul = quat2eul(quat);
    yk_slam = [odom.Transform.Translation.X odom.Transform.Translation.Y eul(1)]';
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
    
%     yk = robot_model(yk,uk0,Ts); % Odometry
%     yk(3) = angle_cal-zero_angle;
    yk = robot_model(yk_slam,uk0,Ts); % SLAM

   
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
    
    % LQR OVERRIDE ---- END
    
    % saturation
    uk(1) = min(uk(1),vmax);
    uk(1) = max(uk(1),vmin);
    uk(2) = min(uk(2),wmax);
    uk(2) = max(uk(2),wmin);
      
    ek = Xr(:,k)-yk; %plotagem
    YK(:,k) = yk;
    YK_SLAM(:,k) = yk_slam;
    %APF + REFTRACK
    uk(1) = uk(1) + v_apf;
    uk(2) = uk(2) + w_apf;
    
    
    UK(:,k) = uk;
    EK(:,k) = ek;
    
    
    
    plot(YK(1,1:k),YK(2,1:k),'red');
    plot(YK_SLAM(1,1:k),YK_SLAM(2,1:k),'blue');
    
    if(HIL)
    motorGo(pub,uk(1),uk(2));
    rate.statistics
    waitfor(rate);
    toc
    end

end
motorGo(pub,0,0)



%% PLOTS
close all
figure;hold on;
plot(Xr(1,:),Xr(2,:),'black--');
grid on;
plot(YK(1,:),YK(2,:),'red');
plot(YK_SLAM(1,1:k),YK_SLAM(2,1:k),'blue');

% plot(YK_Noiseless(1,:),YK_Noiseless(2,:))
title('Controle de Robô Ñ-Holonômico em trajetória')
legend('Trajetória Referência','Odom+Mag Robot','Slam Robot')
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









