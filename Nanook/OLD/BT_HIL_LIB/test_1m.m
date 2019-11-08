function test_1m(b,time,dist)
Ts = 0.1;
samples = time/Ts;
v = dist/time;
w = 0;
[vd,ve] = vw2rpm(v,w);
motorGo(b,vd,ve);
y = [];


for k=1:samples
    tic
[vd_r,ve_r] = motorGet(b);
[v_r,w_r] = rpm2vw(vd_r,ve_r);
% y = [y;[vd_r ve_r]]
y = [y;[v_r w_r]];
plot(y)

toc;
delta = abs(Ts-toc);
    if delta < Ts
    pause(delta);
    end   
end
motorGo(b,0,0);



end