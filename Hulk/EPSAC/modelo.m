function [x] = modelo(u,xo)
%This function represents the robot model
%   Detailed explanation goes here
global Ts

M = [cos(xo(3)) 0;sin(xo(3)) 0;0 1];

x = xo + Ts*M*u;
N = x(3)/(2*pi);
    
% Put theta in range [-pi,pi]
    if N > 1;
        x(3) = x(3) - (round(N))*2*pi;
    end
    
    if N < -1;
        x(3) = x(3) - (round(N))*2*pi;
    end
    
    if x(3) > pi
        x(3) = x(3) - 2*pi;
    end
    
    if x(3) < -pi
        x(3) = x(3) + 2*pi;
    end
    
end

