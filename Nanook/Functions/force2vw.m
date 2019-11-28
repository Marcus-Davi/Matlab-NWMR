function [v,w] = force2vw(F)

kv = 0.5; %0
kw = 1.2; %1.2
v = kv*F(1); %modulo
w = kw*F(2);
end


