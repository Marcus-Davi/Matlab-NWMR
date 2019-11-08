clear;close all;clc
%rosinit
Ts = 0.1;
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
rate = rosrate(1/Ts);

iterations = 100;
w = 0.4; %rad/s
arc = 2*pi; %2pi
time = arc/w;
iterations = round(time/Ts);
motorGo(pub,0,w);
S = zeros(13,iterations);
for i=1:iterations
    sens = receive(sensors);
    data = sens.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');    
    
S(:,i) =  data;
rate.statistics    
waitfor(rate);
end
motorGo(pub,0,0)

%% 
close all
Mag.x = S(7,:);
Mag.y = S(8,:);
Mag.z = S(8,:);
% plot3(Mag.x,Mag.y,Mag.z)
plot(Mag.x,Mag.y)

Xmax = max(Mag.x);
Xmin = min(Mag.x);

Ymax = max(Mag.y);
Ymin = min(Mag.y);

Zmax = max(Mag.z);
Zmin = min(Mag.z);

MagOff.x = (Xmax+Xmin)/2;
MagOff.y = (Ymax+Ymin)/2;
hold on;
grid on
plot(Mag.x - MagOff.x,Mag.y - MagOff.y)
legend('Descalibrado','Compensado')
save('MagCalibration','MagOff')


