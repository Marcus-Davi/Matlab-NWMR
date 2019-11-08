clear;close all;clc
%rosinit
Ts = 1/20;
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');
sensors = rossubscriber('/sensors');
rate = rosrate(1/Ts);

time = 10;
iterations = round(time/Ts);

S = zeros(13,iterations);
Yaw = zeros(1,iterations);
for i=1:iterations
    sens = receive(sensors);
    data = sens.Data;
    data = sscanf(data,'%d %d %d %d %d %d %d %d %d %f %f %f %f');    
    
S(:,i) =  data;
Yaw(i) = atan2(-S(8,i),S(7,i));
rate.statistics    
waitfor(rate);
end

%% 
GyrRes = 17.5e-3;
GyrOff.x = mean(S(4,:));
GyrOff.y = mean(S(5,:));
GyrOff.z = mean(S(6,:));
variance.x = var(deg2rad(S(4,:)*GyrRes));
variance.y = var(deg2rad(S(5,:)*GyrRes));
variance.z = var(deg2rad(S(6,:)*GyrRes));
variance.yaw = var(Yaw);


save('GyrCalibration','GyrOff')


