clear;close all;clc
%% SIMULATION PARAMETERS
Ts = 0.1;
v = 0.15;
% R = 1;
% [Xr,Ur,Tsim] = path_oito(R,v,Ts);
[Xr,Ur,Tsim] = path_S(1,v,Ts,[0 0 0]');
iterations = round((Tsim/Ts));



%% Controller Parameters
% Pi Controller
Kp = 0;
Ki = 100; %206.157 @ Ts = 0.1

%Feuler
pi_a = Kp+ Ki*Ts;
pi_b = Ki*Ts;

%% Simulation
yk = [-0.5 -0.5 0]';
u = [0.0 0]';
w = u(2);
e_t1 = 0; %past error
YK = zeros(3,iterations);
Uk = zeros(2,iterations);

for k=1:iterations
    
    
    e = Xr(:,k) - yk;
    ux = e(1);
    uy = e(2);
    
    t_r = atan2(uy,ux);
    
    v = sqrt(ux*ux + uy*uy);
    e_t = t_r - yk(3);
    w = w + pi_a*e_t - pi_b*e_t1;
% w = 0.1*e_t;

   
    
    if v > 2
        v = 2;
    elseif v< 0
        v = 0;
    end
    
    u = [v w]';
    
    yk = robot_model(yk,u,Ts); %yk = measurement
    
    %updates
    e_t1 = e_t;
    
    
    YK(:,k) = yk;
    Uk(:,k) = u;
% plot(YK(1,1:k),YK(2,1:k),'b','linewidth',2)
% hold on
% plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
% hold off
% grid on
% drawnow
% pause(0.05)
    
end
plot(YK(1,1:k),YK(2,1:k),'b','linewidth',2)
hold on
plot(Xr(1,:),Xr(2,:),'k','linewidth',2)
hold off
grid on

figure
plot(Uk(1,:))
% drawnow

    

