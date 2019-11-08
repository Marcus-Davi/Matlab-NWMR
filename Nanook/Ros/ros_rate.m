%rosinit
clear;clc
pub = rospublisher('/nanook_move');
msg = rosmessage(pub);
msg.Linear.X = 0.0;
msg.Angular.Z = 0.0;

rate = rosrate(100);

vels = -0.2:0.01:0.2;
si = length(vels);
i = 1;

XPOINTS = 200;
time = 1;
x = 1:XPOINTS;
y = zeros(XPOINTS,1);

flag = 1;
while(1)
%    time = rate.TotalElapsedTime
   
 
   if(flag == 1)
       i = i+1;
       if(i == si)
           flag = 0;
       end
   else %flag = 0
       i = i-1;
       if(i==1)
          flag = 1; 
       end
       
      
   end
   
   y(time) = vels(i);
   time = time + 1;
   if(time == XPOINTS)
       time = 1;
   end
  
   figure(1)
   plot(x,y)
%    drawnow;
   
   
     rate.statistics
    msg.Linear.X = vels(i);
   % send(pub,msg);
    waitfor(rate);
    
end
