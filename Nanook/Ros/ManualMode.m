% PUBLICAR TOPICO
clear;clc;close all


%% Plot some trajectory
Ts = 0.1;
x0 = [0 0 0]';
yk = x0;
v = 0.1;
% [Xr,Ur,Tsim] = path_S(1,v,Ts,x0);

%% Ros Parameters
pub = rospublisher('/nanook_move');
msg = rosmessage('geometry_msgs/Twist'); % msg = rosmessage(pub);
sensors = rossubscriber('/sensors');
% slam = rossubscriber('/slam_out_pose');
r = rosrate(1/Ts);
yaw_odom = 0;
%% Sensor
load('GyrCalibration.mat');
load('MagCalibration.mat');

%% Remote
global dir;
dir = 0;
h_fig =figure;
set(h_fig,'KeyPressFcn',@(fig_obj,event) myfun(fig_obj,event));

%% loop

zero_angle = [];
while true
        
   switch(dir)
       case 0 %space
        msg.Linear.X = 0.0;
        msg.Angular.Z = 0.0;
        send(pub,msg);
       case 1  %up
        msg.Linear.X = 0.2;
        msg.Angular.Z = 0.0;
        send(pub,msg);
       case -1 %down
        msg.Linear.X = -0.2;
        msg.Angular.Z = 0.0;
        send(pub,msg);
       case 2 %left
        msg.Linear.X = 0.0;
        msg.Angular.Z = -0.35;
        send(pub,msg);
       case -2 %right
        msg.Linear.X = 0.0;
        msg.Angular.Z = 0.35;
        send(pub,msg);
   end
    sens = receive(sensors);
    data = sens.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');
%     slam_msg = receive(slam); 
%     yk_slam = [slam_msg.Pose.Position.X slam_msg.Pose.Position.Y 0]';
    Mag.x = data(7);
    Mag.y = data(8);
    angle_cal = atan2((Mag.y-MagOff.y),(Mag.x-MagOff.x)); %angulo mag. se liga sinal
    if(isempty(zero_angle))
    zero_angle = angle_cal; 
    end
    vd = data(10);
    ve = data(11);
    [v,w] = rpm2vw(vd,ve);
    yk = robot_model(yk,[v w]',Ts); % Odometry
    yk(3) = angle_cal - zero_angle;
    yaw_odom = yaw_odom + w*Ts;
    yk_arrow = 0.3*[cos(yk(3)) sin(yk(3))];
    yk_arrow_odom = 0.3*[cos(yaw_odom) sin(yaw_odom)];
   
    
%     plot(Xr(1,:),Xr(2,:));
%     hold on;
    quiver(yk(1),yk(2),yk_arrow(1),yk_arrow(2),'b','linewidth',2);
    hold on;
    quiver(yk(1),yk(2),yk_arrow_odom(1),yk_arrow_odom(2),'r','linewidth',2);
    
    plot(yk(1),yk(2),'*b');
    xlim([-2 2]);
    ylim([-2 2]);

%     plot(yk_slam(1),yk_slam(2),'*b')
    grid on
    hold off;
    r.statistics
              waitfor(r);
end





%%
function myfun(~,event,port)
global dir;

if (strcmp(event.Key,'uparrow'))
dir = 1;
disp('forward')
elseif (strcmp(event.Key,'downarrow'))
dir = -1;
disp('back')
elseif (strcmp(event.Key,'leftarrow'))
dir = -2;         
disp('left')
elseif (strcmp(event.Key,'rightarrow'))
dir = 2 ;          
disp('right')
elseif (strcmp(event.Key,'space'))
dir = 0;
disp('stop')
end


end