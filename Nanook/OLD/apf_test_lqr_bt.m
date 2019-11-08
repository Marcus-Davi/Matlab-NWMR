%% Estabelecimento da Conexao Bluetooth
if exist('b')
    fclose(b);
end
addpath('PATHS');
addpath('BT_HIL_LIB');
clear ;close all;clc
b = blueNanook;
load('CalibrationAccGyr')
load('CalibrationMag')

%% LQR
ur1 = 0.1;
ur2 = 0.0;
A = [0 ur2 0;-ur2 0 ur1;0 0 0];
B = [1 0;0 0;0 1];
C = eye(3);
D = zeros(3,2);
SYS = ss(A,B,C,D);
Q = diag([1 30 0.15]);
R = eye(2);
[K,S,E] = lqr(SYS,Q,R);

%% TRAJ
Ts = 0.1;          % sampling time
limit = 0.4;
x0 = [0 0 0.]';
% x0 = [0 0 pi]'; %reverse path
[Xr,Ur,Tsim] = path_S(1.2,0.1,Ts,x0);

plot(Xr(1,:),Xr(2,:),'-')
hold on
grid


%% ROS

laser = rossubscriber('/scan');

%% SIMULA
X = [];
x = x0;

vd = 0;
ve = 0;

Sensors = [];
%% Bluetooth Interface
deltaTs = Ts;

for k = 1:round(Tsim/Ts)    % 1 a tempo de simula��o/ tempo de integra��o
    
    tic;
    
    
    % LIDAR PROCESSING
   scan = receive(laser); %10Hz
%    
   ranges = scan.Ranges;
   angleI = scan.AngleIncrement;
   angleMin = scan.AngleMin;
   F = ranges2force(ranges,angleI,angleMin);
   [v_apf,w_apf] = force2vw(F)


%    plot(scan);
%    hold on;
%  plotv(F)
%    quiver(0,0,F(1),F(2),'Color','r','linewidth',2);
%    hold off;


    % MEASUREMENT
    s = sensorGet(b); %ajeitar transposto
    if(~isempty(s))
    S = sensorCalibrate(s,CalibrationAccGyr,CalibrationMag);
    vd = S(10);
    ve = S(11);
    end
    

    [v_measure,w_measure] = rpm2vw(vd,ve);
    
    %MODEL 
    
    x = robot_model(x,[v_measure w_measure],deltaTs);
    
    
    
    
    
    
    e_x = Xr(1,k)-x(1);
    e_y = Xr(2,k)-x(2);
    e_t = Xr(3,k)-x(3);
    
    e_t = atan2(sin(e_t),cos(e_t)); %true error
    
    e1 = cos(x(3))*e_x+sin(x(3))*e_y;
    e2 = -sin(x(3))*e_x+cos(x(3))*e_y;
    e3 = e_t;
       
    
%        k1 = 2*ksi*ohmega_n;
%        k2 = g*abs(vr(k));
%        k3 = k1;
%     
%        v1 = -k1*(e1);                    % controlador
%        v2 = -sign(vr(k))*k2*(e2)-k3*e3;  % controlador
    
    U = -K*[e1 e2 e3]';
    u1 = U(1);
    u2 = U(2);
    
    v = Ur(1,k)*cos(e3) - u1;
    w = Ur(2,k) -  u2;
    
    %restri��es
    
    if v >=limit  %0.4
        v=limit;
    end
    
    if v< 0
        v = 0;
    end
    
    if w>= limit*2
        w = limit*2;
    end
    
    if w<-limit*2
        w=-limit*2;
    end
      
    
    [vd_rpm,ve_rpm] = vw2rpm(v,w);
    [vd_apf,ve_apf] = vw2rpm(v_apf,w_apf);
    
    
    vd_total = vd_rpm + vd_apf;
    ve_total = ve_rpm + ve_apf;
%     
    
%     motorGo(b,vd_rpm,ve_rpm); % SO TRAJ
%     motorGo(b,vd_apf,ve_apf); % SO APF
    motorGo(b,vd_total,ve_total); % SO APF
%     motorGo(b,15,-15); %override
    plot(x(1),x(2),'*')

%     y(position) = inertial(1);
%     plot(y)
%     position = position + 1;
    
%     drawnow;
    deltaTs = toc

% delta = abs(Ts-toc);
%     if delta < Ts
%     pause(delta);
%     end
    
    Sensors = [Sensors; S'];
    X = [X x];
    
end

motorGo(b,0,0)
fclose(b)

%% Plots
return
% Gyro
close all
plot(Sensors(:,6)*17e-3*pi/180)
[V,W] = rpm2vw(Sensors(:,10),Sensors(:,11))
hold on
plot(W)
grid on
legend('Gyroscope','Odometry')
figure
Bx = Sensors(:,7);
By = Sensors(:,8);
angle = atan2(-Bx,By);
plot(angle)
hold on
plot(X(3,:))







% angle = atan2(F(2),F(1))