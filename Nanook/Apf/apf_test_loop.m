close all;clc;clear
%rosinit
addpath('ROS_HIL')
pub = rospublisher('/nanook_move');
msg = rosmessage('geometry_msgs/Twist');
laser = rossubscriber('/scan');



while(1)
   scan = receive(laser);

   
   ranges = scan.Ranges;
angleI = scan.AngleIncrement;
angleMin = scan.AngleMin;
   F = ranges2force(ranges,angleI,angleMin)
   [v,w] = force2vw(F);
   v
   w
   motorGo(pub,v,w)

   plot(scan);
   hold on;
%    plotv(F)
    quiver(0,0,F(1),F(2),'Color','r','linewidth',2);
   hold off;
   
end
    











% angle = atan2(F(2),F(1))