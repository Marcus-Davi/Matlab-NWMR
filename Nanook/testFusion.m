% PUBLICAR TOPICO
clear;clc;close all
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
sensors = rossubscriber('/sensors');
msg = rosmessage('geometry_msgs/Twist');
Ts = 1/10;
r = rosrate(1/Ts);
%% Load Mag Calibration

load('MagCalibration.mat');
load('GyrCalibration.mat');
GyrRes = 17.5e-3; %bit -> deg/s
MagRes = 0.1;
%% Remote
global dir;
dir = 0;
h_fig =figure;
grid on
set(h_fig,'KeyPressFcn',@(fig_obj,event) myfun(fig_obj,event));
A = [];
angle_gyro = 0;
%% loop
vel = 30; 
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
        msg.Angular.Z = -0.5;
        send(pub,msg);
       case -2 %right
        msg.Linear.X = 0.0;
        msg.Angular.Z = 0.5;
        send(pub,msg);
   end
              
    waitfor(r);
                  sens = receive(sensors);
                    data = sens.Data;
                 data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');
                 Mag.x = data(7);
                 Mag.y = data(8);
                 Gyr.z = data(6);
                 gyr = deg2rad((Gyr.z - GyrOff.z)*GyrRes);
                 angle = atan2(-Mag.y,Mag.x);
                 angle_cal = atan2(-(Mag.y-MagOff.y),(Mag.x-MagOff.x));
                 
                 angle_fused = simpleFusion(angle_cal,gyr,Ts);
                 if(angle_gyro == 0)
                 angle_gyro = angle_cal; 
                 zero_angle = angle_cal; %começa sempre em 0
                 end
                 
                 angle_gyro = angle_gyro - gyr*Ts;
%                  A = [A angle_cal];
              circle(0,0,1)
              hold on
%               quiver(0,0,cos(angle),sin(angle),'Color','r','linewidth',2);
%               quiver(0,0,cos(angle_cal),sin(angle_cal),'Color','r','linewidth',2);
              quiver(0,0,cos(angle_cal-zero_angle),sin(angle_cal-zero_angle),'Color','r','linewidth',2);
              quiver(0,0,cos(angle_fused),sin(angle_fused),'Color','b','linewidth',2);
              quiver(0,0,cos(angle_gyro),sin(angle_gyro),'linewidth',2);
              grid on
              hold off
              r.statistics;
%               disp(var(A));
              
            
end





%%


function circle(x,y,r)
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,'linewidth',2);
end
%%
function myfun(~,event,port)
global dir;
vel = 45;
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