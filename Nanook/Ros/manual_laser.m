% PUBLICAR TOPICO
clear;clc;close all
pub = rospublisher('/nanook_move');
% msg = rosmessage(pub);
msg = rosmessage('geometry_msgs/Twist');

laser = rossubscriber('/scan0');

% msg.Linear.X = 0.1;
% msg.Angular.Z = 0.0;
% send(pub,msg);


%% Remote
global dir;
dir = 0;
    scan = receive(laser);
    plot(scan)
h_fig =figure(1);
set(h_fig,'KeyPressFcn',@(fig_obj,event) myfun(fig_obj,event));

%% loop
vel = 30; 
while true
        scan = receive(laser);
plot(scan)
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

%        case 0
%             fprintf(b,'G!0,0\r'); %space
%        case 1
%             fprintf(b,'G!45,45\r'); %up
%        case -1
%             fprintf(b,'G!-45,-45\r'); %down
%        case 2
%            fprintf(b,'G!-30,30\r'); %left
%        case -2
%            fprintf(b,'G!30,-30\r'); %right
   end
              
%               pause(0.1);
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